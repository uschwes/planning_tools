/*
 * Trajectory.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_TRAJECTORY_HPP_
#define PLANNING2D_TRAJECTORY_HPP_

// standard includes
#include <vector>
#include <algorithm>

// boost includes
#include <boost/serialization/base_object.hpp>

// self includes
#include "Support.hpp"
#include "Time.hpp"
#include "State.hpp"
#include "Pose2d.hpp"
#include "SystemInput.hpp"

namespace planning2d {


class StateInputPairStamped : public StampedType {

 public:
  //! Default constructor
  inline StateInputPairStamped();
  //! constructor that initializes all members
  inline StateInputPairStamped(const State& state, const SystemInput& input, const Time& stamp);

  //! returns const reference to state
  inline const State& state() const;
  //! returns mutable reference to state
  inline State& state();
  //! returns const reference to system input
  inline const SystemInput& input() const;
  //! returns mutable reference to system input
  inline SystemInput& input();

  //! equality operator
  inline bool operator==(const StateInputPairStamped& d) const;
  //! inequality operator
  inline bool operator!=(const StateInputPairStamped& d) const;

  //! Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & _state & _input & boost::serialization::base_object<StampedType>(*this);
  }

 private:
  State _state;
  SystemInput _input;

}; /* class StateInputPairStamped */

typedef std::vector<Position2dStamped> PositionTrajectory;
typedef std::vector<Pose2dStamped> PoseTrajectory;
typedef std::vector<StateStamped> StateTrajectory;
typedef std::vector<SystemInputStamped> SystemInputTrajectory;
typedef std::vector<StateInputPairStamped> StateInputTrajectory;
typedef std::vector<Pose2d> Path;

/**
 * Extracts a range from a trajectory type, so that the timestamps items in the extracted
 * trajectory are in the range [\p start, \p end]
 * @param start lower bound
 * @param end upper bound
 * @param trajectory full trajectory
 * @return trajectory with all timestamps >= \p start and <= \p end
 */
template <typename T>
T extractRange(const Time& start, const Time& end, const T& trajectory) {
  static StampedTypeComparator comp;
  auto lb = std::lower_bound(trajectory.begin(), trajectory.end(), start, comp);
  auto ub = std::upper_bound(trajectory.begin(), trajectory.end(), end, comp);
  return T(lb, ub);
}

} /* namespace planning2d */

#include "implementation/TrajectoryImplementation.hpp"

#endif /* PLANNING2D_TRAJECTORY_HPP_ */
