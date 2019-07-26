/*
 * PlannerInterfaceDebugWrapper.hpp
 *
 *  Created on: Mar 3, 2015
 *      Author: pfmark
 */

#ifndef PLANNERINTERFACEDEBUGWRAPPER_HPP_
#define PLANNERINTERFACEDEBUGWRAPPER_HPP_


#include <memory>
#include <atomic>
#include <fstream>
#include <string>

#include <ctype.h>

#include <boost/shared_ptr.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "TargetPlannerInterface.hpp"
#include "State.hpp"

BOOST_CLASS_TRACKING(planning2d::State, boost::serialization::track_always)

namespace planning2d {

class PlannerInterfacePlayer {
public:
  PLANNING_2D_POINTER_TYPEDEFS(PlannerInterfacePlayer);
  virtual void seek(int counter) = 0;
  virtual std::string step() = 0;
  virtual size_t getCounter() = 0;
  virtual double getCurrentTimeComputePlan() = 0;

};

template <typename AgentT> class PlannerInterfacePlayerImpl;

template <typename AgentT>
class PlannerInterfaceDebugWrapper : public TargetPlannerInterface //TODO support general planner
{

  friend class PlannerInterfacePlayerImpl<AgentT>;

 public:
  PLANNING_2D_POINTER_TYPEDEFS(PlannerInterfaceDebugWrapper);

  PlannerInterfaceDebugWrapper(boost::shared_ptr<PlannerInterface> wrappedPlanner, std::string outputFolderPath);
  virtual ~PlannerInterfaceDebugWrapper();

  planning2d::ReturnCode computePlan(const planning2d::Time& currentTime,
    planning2d::StateInputTrajectory& stateInputSequency) override;

  planning2d::ReturnCode callbackOccupancyGrid(
    const planning2d::OccupancyGridStamped& grid) override;

  planning2d::ReturnCode callbackCurrentState(
    const planning2d::StateStamped& stateStamped) override;

  planning2d::ReturnCode callbackSetTarget(
    const planning2d::Pose2d& target, bool isFinal) override;

  planning2d::ReturnCode callbackDynamicObjects(
    const std::vector<planning2d::Agent::ConstPtr>& dynamicObjects) override;

  void skipComputePlan(bool s) { _skipComputePlan = s; }


  void playback(size_t startCounter, size_t endCounter);
  /**
   * The returned pointer is only valid to be used while the wrapper object exists!!!
   */
  PlannerInterfacePlayer::Ptr getPlayer();

  const boost::shared_ptr<PlannerInterface>& getWrappedPlanner() const
  {
    return _wrappedPlanner;
  }

 private:
  typedef boost::archive::binary_oarchive OutputArchive;
  typedef boost::archive::binary_iarchive InputArchive;

  struct RegistrationInterface {
    virtual ~RegistrationInterface() { }
    virtual void registerTypes(OutputArchive & a) = 0;
    virtual void registerTypes(InputArchive & a) = 0;
  };

  template <typename ... Types>
  struct TypeRegistry : public RegistrationInterface {
    void registerTypes(OutputArchive & a) override { _registerTypes<OutputArchive, Types...>(a); }
    void registerTypes(InputArchive & a) override { _registerTypes<InputArchive, Types...>(a); }
   private:
    template <typename Archive, typename MyType, typename ... MyTypes>
    void _registerTypes(Archive & a){
      a.template register_type<MyType>();
      _registerTypes<Archive, MyTypes...>(a);
    }

    template <typename Archive>
    void _registerTypes(Archive & /*a*/){
    }

  };
 public:
  template <typename ... Types>
  void registerTypes(){
    registrationObject.reset(new TypeRegistry<Types...>());
  }

 private:
  std::string computeNewCallbackPath(std::string name){
    size_t c = _callbackCounter++;
    std::stringstream ss;
    ss << _outputFolderPath << "/";
    ss << std::setw(5) << std::setfill('0') << c;
    ss << "_" << name;
    ss << ".pc"; // planner callback
    return ss.str();
  }


  template <typename Archive>
  void prepareArchive(Archive & a){
    a.template register_type<AgentT>();
    if(registrationObject){ //TODO else : warn!
      registrationObject->registerTypes(a);
    }
  }

  struct Archive {
    Archive(std::string path, PlannerInterfaceDebugWrapper & wrapper)
    : of(path), oa(of)
    {
      wrapper.prepareArchive(oa);
    }
    std::ofstream of;
    OutputArchive oa;
  };


  std::unique_ptr<Archive > openCallbackFile(std::string name){
    std::unique_ptr<Archive > archive(new Archive(computeNewCallbackPath(name), *this));
    archive->oa << name;
    return std::move(archive);
  }

  void createFileMap(std::string folderPath);
  bool startsWithCounter(const std::string filename);
  size_t getCounter(const std::string filename);
  std::string deserializeRunCallback(boost::filesystem::path);
  std::string getCallback(boost::filesystem::path);


  boost::shared_ptr<PlannerInterface> _wrappedPlanner;
  std::string _outputFolderPath;
  std::atomic<size_t> _callbackCounter;
  std::map<int, boost::filesystem::path> _filesInDirectory;
  std::unique_ptr<RegistrationInterface> registrationObject;
  bool _ranSetTarget, _ranCurrentState, _ranOccupancyGrid, _ranDynamicObjects;
  bool _skipComputePlan = false;
};

} /* namespace planning_2d */


#include "planner_interfaces/implementation/PlannerInterfaceDebugWrapperImplementation.hpp"

#endif /* PLANNERINTERFACEDEBUGWRAPPER_HPP_ */
