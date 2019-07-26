/*
 * SceneSnapshot.hpp
 *
 *  Created on: Apr 30, 2015
 *      Author: pfmark
 */

#ifndef PROBABILISTIC_PLANNER_STATE_REPRESENTATION_SCENESNAPSHOT_HPP_
#define PROBABILISTIC_PLANNER_STATE_REPRESENTATION_SCENESNAPSHOT_HPP_

// standard
#include <string>
#include <initializer_list>

// boost
#include <boost/optional.hpp>

// planner interfaces
#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/State.hpp>
#include <planner_interfaces/StampedType.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>

// cache
#include <simple_cache/SimpleCache.hpp>

namespace prob_planner {

class StateWithUncertainty: public planning2d::State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PLANNING_2D_POINTER_TYPEDEFS(StateWithUncertainty);
  StateWithUncertainty() : planning2d::State() { }
  StateWithUncertainty(const planning2d::State& state, const Eigen::MatrixXd& invCov) :
    planning2d::State(state),
    _invCov(invCov) {
    SM_ASSERT_EQ( planning2d::InitializationException, state.fullDimension(), static_cast<std::size_t>(_invCov.rows()), "");
    SM_ASSERT_EQ( planning2d::InitializationException, state.fullDimension(), static_cast<std::size_t>(_invCov.cols()), "");
  }
  ~StateWithUncertainty() { }
  Eigen::MatrixXd& invCov() { return _invCov; }
  const Eigen::MatrixXd& invCov() const { return _invCov; }

  bool operator==(const StateWithUncertainty& su) const { return planning2d::State::operator==(su) && this->invCov().isApprox(su.invCov()); }
  bool operator!=(const StateWithUncertainty& su) const { return !(*this == su); }

  /// \brief Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:
  Eigen::MatrixXd _invCov;
};

class SceneSnapshot : public planning2d::StampedType {

 public:
  typedef std::map<planning2d::Id, StateWithUncertainty> StateMap;
  typedef cache::SimpleCache<std::string> Cache;
  PLANNING_2D_POINTER_TYPEDEFS(SceneSnapshot);
  friend class boost::serialization::access;

 public:
  /// \brief Constructor
  SceneSnapshot(const planning2d::Time& stamp);
  /// \brief Convenience constructor for one agent observation
  SceneSnapshot(const planning2d::Time& stamp, const planning2d::Id id, const StateWithUncertainty& meas);
  /// \brief Constructor taking a list of agent measurements
  SceneSnapshot(const planning2d::Time& stamp, std::initializer_list< std::pair<const planning2d::Id,StateWithUncertainty> > l);
  /// \brief Convenience constructor for one grid
  SceneSnapshot(const planning2d::Time& stamp, const planning2d::OccupancyGrid& grid);
  /// \brief Destructor
  ~SceneSnapshot() { }

  void addObject(const planning2d::Id& id, const StateWithUncertainty& state);
  const StateWithUncertainty& getObject(const planning2d::Id& id) const;
  bool hasObject(const planning2d::Id& id) const;
  const StateMap& objectContainer() const { return _objectContainer; }
  StateMap& objectContainer() { return _objectContainer; }

  void setOccupancyGrid(const planning2d::OccupancyGrid& grid) { _occupancyGrid = grid; }
  boost::optional<planning2d::OccupancyGrid> getOccupancyGrid() const { return _occupancyGrid; }

  Cache& cache() const { return _cache; }

  /// \brief Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:
  SceneSnapshot() { }

 private:
  StateMap _objectContainer;                      // Current states of agents
  boost::optional<planning2d::OccupancyGrid> _occupancyGrid = boost::optional<planning2d::OccupancyGrid>();

  mutable Cache _cache;

};

} /* namespace prob_planner */

#include "impl/SceneSnapshotImpl.hpp"

#endif /* PROBABILISTIC_PLANNER_STATE_REPRESENTATION_SCENESNAPSHOT_HPP_ */
