/*
 * TrajectoryImplementation.hpp
 *
 *  Created on: Oct 22, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_TRAJECTORY_IMPLEMENTATION_HPP_
#define PLANNING2D_TRAJECTORY_IMPLEMENTATION_HPP_

namespace planning2d {

StateInputPairStamped::StateInputPairStamped() {

}

StateInputPairStamped::StateInputPairStamped(const State& state, const SystemInput& input, const Time& stamp) :
    StampedType(stamp),
    _state(state),
    _input(input) {

}

const State& StateInputPairStamped::state() const {
  return _state;
}

State& StateInputPairStamped::state() {
  return _state;
}

const SystemInput& StateInputPairStamped::input() const{
  return _input;
}

SystemInput& StateInputPairStamped::input() {
  return _input;
}

inline bool StateInputPairStamped::operator==(const StateInputPairStamped& d) const{
  return _state == d.state() && _input == d.input() && StampedType::operator==(d);
}

inline bool StateInputPairStamped::operator!=(const StateInputPairStamped& d) const {
  return !(*this==d);
}

} /* namespace planning2d */

#endif /* PLANNING2D_TRAJECTORY_IMPLEMENTATION_HPP_ */
