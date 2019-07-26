/*
 * PlannerInterfaceDebugWrapperImplementation.hpp
 *
 *  Created on: Nov 6, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PLANNER_INTERFACES_IMPLEMENTATION_PLANNERINTERFACEDEBUGWRAPPERIMPLEMENTATION_HPP_
#define INCLUDE_PLANNER_INTERFACES_IMPLEMENTATION_PLANNERINTERFACEDEBUGWRAPPERIMPLEMENTATION_HPP_

namespace planning2d {

template <typename AgentT>
PlannerInterfaceDebugWrapper<AgentT>::PlannerInterfaceDebugWrapper(boost::shared_ptr<PlannerInterface> wrappedPlanner, std::string outputFolderPath)
:
_wrappedPlanner(wrappedPlanner),
_outputFolderPath(outputFolderPath),
_callbackCounter(0),
_ranSetTarget(false),
_ranCurrentState(false),
_ranOccupancyGrid(false),
_ranDynamicObjects(false)
{
  // TODO: solve folder structure
  if (!boost::filesystem::is_directory(outputFolderPath)) {
    SM_ASSERT_TRUE(std::runtime_error, boost::filesystem::create_directories(outputFolderPath),
                   "Could not create directory " << outputFolderPath);
  }
  /* TODO maybe split the wrapper and the player into two classes
  SM_ASSERT_TRUE(std::runtime_error, boost::filesystem::is_empty(outputFolderPath),
                 "Folder " << outputFolderPath << " has to be empty.");
                 */
}


template <typename AgentT>
PlannerInterfaceDebugWrapper<AgentT>::~PlannerInterfaceDebugWrapper() {
  // TODO Auto-generated destructor stub
}


template <typename AgentT>
planning2d::ReturnCode PlannerInterfaceDebugWrapper<AgentT>::computePlan(
    const planning2d::Time& currentTime,
    planning2d::StateInputTrajectory& stateInputSequency)
{
  // serialize
    auto a = openCallbackFile("computePlan");
    a->oa << currentTime;

    // call planner
    planning2d::ReturnCode retCode = planning2d::ReturnCode::RETCODE_OK;
    if (!_skipComputePlan)
      retCode = _wrappedPlanner->computePlan(currentTime, stateInputSequency);

    return retCode;
}


template <typename AgentT>
planning2d::ReturnCode PlannerInterfaceDebugWrapper<AgentT>::callbackOccupancyGrid(
    const planning2d::OccupancyGridStamped& grid)
{
  // serialize
  auto a = openCallbackFile("occupancyGrid");
  a->oa << grid;

  // call planner
  return _wrappedPlanner->callbackOccupancyGrid(grid);

}


template <typename AgentT>
planning2d::ReturnCode PlannerInterfaceDebugWrapper<AgentT>::callbackCurrentState(
    const planning2d::StateStamped& stateStamped)
{
  // serialize
  auto a = openCallbackFile("currentState");
  auto ptr = &stateStamped;
  a->oa << BOOST_SERIALIZATION_NVP(ptr);

  // call planner
  return _wrappedPlanner->callbackCurrentState(stateStamped);

}


template <typename AgentT>
planning2d::ReturnCode PlannerInterfaceDebugWrapper<AgentT>::callbackSetTarget(
    const planning2d::Pose2d& target, bool isFinal)
{
  auto wrappedTragetPlanner = boost::dynamic_pointer_cast<TargetPlannerInterface>(_wrappedPlanner);
  SM_ASSERT_TRUE(std::runtime_error, wrappedTragetPlanner, "Only target planner interface implementors supported, yet");
  // serialize
  auto a = openCallbackFile("setTarget");
  a->oa << target;
  a->oa << isFinal;

  // call planner
  return wrappedTragetPlanner->callbackSetTarget(target, isFinal);

}


template <typename AgentT>
planning2d::ReturnCode PlannerInterfaceDebugWrapper<AgentT>::callbackDynamicObjects(
    const std::vector<planning2d::Agent::ConstPtr>& dynamicObjects)
{
  // serialize
  auto a = openCallbackFile("dynamicObjects");

  std::vector<typename AgentT::ConstPtr> dynamicObjectsClones;
  for(auto dO : dynamicObjects){
    dynamicObjectsClones.push_back(typename AgentT::ConstPtr(new AgentT(dynamic_cast<const AgentT&>(*dO))));
  }

  a->oa << dynamicObjectsClones;

  // call planner
  return _wrappedPlanner->callbackDynamicObjects(dynamicObjects);

}

template <typename AgentT>
class PlannerInterfacePlayerImpl : public PlannerInterfacePlayer {

 public:
  PlannerInterfacePlayerImpl(PlannerInterfaceDebugWrapper<AgentT> * wrapper) : _wrapper(wrapper) {
    _wrapper->createFileMap(_wrapper->_outputFolderPath);
    _it = wrapper->_filesInDirectory.begin();
  }

  void seek(int counter) override{
    _it = _wrapper->_filesInDirectory.find(counter);
    if(_it == _wrapper->_filesInDirectory.end()){
      //SM_WARN_STREAM("File with counter " << counter << " does not exist.");
      throw "File with the desired counter does not exist!";
    }
  }

  std::string step() override{

    if (_wrapper->_ranSetTarget == false){
      SM_FINE_STREAM_NAMED("iface_wrapper","Running latest callbackSetTarget to specify agent target.");
      runLatestSetTarget();
    }

    std::string callback = _wrapper->getCallback(_it->second);
    // only run computePlan if acquired information and target is set
    if (callback == "computePlan"){
      if ((_wrapper->_ranSetTarget == true) & (_wrapper->_ranCurrentState == true) /*&      // TODO enable when dealing with occupancy grid
          (_wrapper->_ranOccupancyGrid == true)*/ & (_wrapper->_ranDynamicObjects == true)){
        callback = _wrapper->deserializeRunCallback(_it->second);
      }
      else {
        SM_WARN_STREAM("Skipping computePlan since not enough information was acquired.");
      }
    }
    else {
      callback = _wrapper->deserializeRunCallback(_it->second);
    }

    _it ++;
    return callback;
  }

  double getCurrentTimeComputePlan() {
    auto it = _it;
    --it;
    std::string filename = it->second.filename().native();
    std::ifstream ifs(it->second.string());
    boost::archive::binary_iarchive ia(ifs);

    ia.register_type<AgentT>();

    std::string callback;
    Time currentTime;
    ia >> callback;
    ia >> currentTime;
    return currentTime.toSec();

  }

  void runLatestSetTarget(){
    for (auto it=_wrapper->_filesInDirectory.rbegin(); it!=_wrapper->_filesInDirectory.rend(); ++it){
      SM_DEBUG_STREAM_NAMED("iface_wrapper", "Callback at counter " << it->first << " is " << _wrapper->getCallback(it->second));
      if (_wrapper->getCallback(it->second) == "setTarget"){
        SM_DEBUG_STREAM_NAMED("iface_wrapper", "Running setTarget at counter " << it->first);
        _wrapper->deserializeRunCallback(it->second);
        if (it->first <= _it->first){
          SM_FINE_STREAM_NAMED("iface_wrapper", "Found target at counter " << it->first <<
                         " which is before current one (" << _it->first << ")");
          break;
        }
      }
    }
  }

  size_t getCounter() override {
    return _it->first;
  }

 private:
  PlannerInterfaceDebugWrapper<AgentT> * _wrapper;
  typename decltype(_wrapper->_filesInDirectory)::iterator _it;
};

template <typename AgentT>
PlannerInterfacePlayer::Ptr PlannerInterfaceDebugWrapper<AgentT>::getPlayer()
{

  createFileMap(_outputFolderPath);

  // iterate over map and run callbacks
  return PlannerInterfacePlayer::Ptr(new PlannerInterfacePlayerImpl<AgentT>(this));
}


template <typename AgentT>
void PlannerInterfaceDebugWrapper<AgentT>::playback(size_t startCounter, size_t /* endCounter */)
{
  auto player = getPlayer();
  player->seek(startCounter);


  // TODO make sure that all callbacks ran before computePlan is called??

  createFileMap(_outputFolderPath);

  // iterate over map and run callbacks
  for (auto it=_filesInDirectory.begin(); it!=_filesInDirectory.end(); ++it){
    deserializeRunCallback(it->second);
  }
}


template <typename AgentT>
void PlannerInterfaceDebugWrapper<AgentT>::createFileMap(std::string folderPath)
{
  std::string filename;

  // iterate over files within counter range
  if (!boost::filesystem::exists(folderPath)) {
    SM_WARN_STREAM(__FUNCTION__ << ": Folder with the name '" << folderPath << "' does not exist.");
  }

  boost::filesystem::directory_iterator endItr;

  for (boost::filesystem::directory_iterator itr(folderPath); itr!=endItr; ++itr){

    filename = itr->path().filename().native();

    // store in map if file starts with correct counter
    if (startsWithCounter(filename)){
      size_t counter = getCounter(filename);
      _filesInDirectory[counter] = itr->path();
    }
  }
}


template <typename AgentT>
bool PlannerInterfaceDebugWrapper<AgentT>::startsWithCounter(const std::string filename)
{
  return isdigit(filename[0]);
}


template <typename AgentT>
size_t PlannerInterfaceDebugWrapper<AgentT>::getCounter(const std::string filename)
{

  std::string counterAsString;
  for(size_t i=0; i<filename.size(); ++i){
    if (isdigit(filename[i])){
      counterAsString += filename[i];
    }
  }

  size_t counter = static_cast<size_t>(stoi(counterAsString));

  return counter;
}


template <typename AgentT>
std::string PlannerInterfaceDebugWrapper<AgentT>::deserializeRunCallback(
    boost::filesystem::path path)
{
  std::string filename = path.filename().native();
  std::ifstream ifs(path.string());
  boost::archive::binary_iarchive ia(ifs);

  prepareArchive(ia);

  std::string callback;
  ia >> callback;

  if (callback == "computePlan"){
    StateInputTrajectory stateInputSequence;
    Time currentTime;
    ia >> currentTime;
    _wrappedPlanner->computePlan(currentTime, stateInputSequence);
  }

  else if (callback == "occupancyGrid"){
    OccupancyGridStamped grid;
    ia >> grid;
    _wrappedPlanner->callbackOccupancyGrid(grid);
    _ranOccupancyGrid = true;
  }

  else if (callback == "currentState"){
    StateStamped* stateStampedPtr;
    ia >> BOOST_SERIALIZATION_NVP(stateStampedPtr);
    _wrappedPlanner->callbackCurrentState(*stateStampedPtr);
    _ranCurrentState = true;
    delete stateStampedPtr;
  }

  else if (callback == "setTarget"){
    Pose2d target;
    bool isFinal;
    ia >> target;
    ia >> isFinal;
    auto wrappedTragetPlanner = boost::dynamic_pointer_cast<TargetPlannerInterface>(_wrappedPlanner);
    SM_ASSERT_TRUE(std::runtime_error, wrappedTragetPlanner != nullptr, "Only target planner interface implementors supported, yet");
    wrappedTragetPlanner->callbackSetTarget(target, isFinal);
    _ranSetTarget = true;
  }

  else if (callback == "dynamicObjects"){
    std::vector<typename AgentT::ConstPtr> dynamicObjects;
    ia >> dynamicObjects;

    std::vector<Agent::ConstPtr> dynamicObjectClones;
    for (auto agent : dynamicObjects) {
      dynamicObjectClones.push_back(static_pointer_cast<const Agent>(agent));
    }

    SM_FINE_STREAM_NAMED("iface_wrapper", __FUNCTION__ << ": Size of deserialized dynamic objects is " << dynamicObjects.size() << ".");
    _wrappedPlanner->callbackDynamicObjects(dynamicObjectClones);
    _ranDynamicObjects = true;
  }

  else {
    throw "No such callback defined";
  }

  return callback;
}

template <typename AgentT>
std::string PlannerInterfaceDebugWrapper<AgentT>::getCallback(
    boost::filesystem::path path)
{
  std::string name = path.filename().native();
  size_t startIndex = name.find("_") + 1;
  int stringLength = name.size() - 3 - startIndex;
  std:: string callback = name.substr(startIndex, stringLength);

  return callback;
}


}  /* end namespace planning2d */

#endif /* INCLUDE_PLANNER_INTERFACES_IMPLEMENTATION_PLANNERINTERFACEDEBUGWRAPPERIMPLEMENTATION_HPP_ */
