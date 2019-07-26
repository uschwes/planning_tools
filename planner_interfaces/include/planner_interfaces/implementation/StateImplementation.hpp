/*
 * StateImplementation.hpp
 *
 *  Created on: Oct 22, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_STATE_IMPLEMENTATION_HPP_
#define PLANNING2D_STATE_IMPLEMENTATION_HPP_

// self includes
#include "../Support.hpp"
#include "../Exceptions.hpp"

// Boost includes
#include <boost/serialization/base_object.hpp>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

namespace planning2d {

  // *********************** //
  // **** State methods **** //
  // *********************** //

  inline State::State() :
      _pose(),
      _states() {

  }

  inline State::State(const std::size_t sz) :
      _states(sz) {
#ifndef NDEBUG
      _states = State::T::Constant(sz,1,SIGNAN);
#endif
  }

  inline State::State(const T& states, const Pose2d& pose) :
      _pose(pose),
      _states(states) {

  }

  inline double State::x() const { return _pose.position().x(); }
  inline double& State::x() { return _pose.position().x(); }
  inline double State::y() const { return _pose.position().y(); }
  inline double& State::y() { return _pose.position().y(); }
  inline double State::yaw() const { return _pose.yaw(); }
  inline double& State::yaw() { return _pose.yaw(); }

  inline const Position2d& State::position() const { return _pose.position(); }
  inline Position2d& State::position() { return _pose.position(); }
  inline State::operator const Position2d& () const { return this->position(); }
  inline State::operator Position2d& () { return this->position(); }

  inline const Pose2d& State::pose() const { return _pose; }
  inline Pose2d& State::pose() { return _pose; }
  inline State::operator const Pose2d& () const { return this->pose(); }
  inline State::operator Pose2d& () { return this->pose(); }

  inline const State::T& State::state() const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->_states.isApprox(this->_states)),
                       "The state contains at least one NaN value.");
    return _states;
  }
  inline State::T& State::state() { return _states; }

  inline std::size_t State::dimension() const { return _states.rows(); }
  inline std::size_t State::fullDimension() const { return _states.rows() + _pose.dimension(); }
  inline const double& State::operator()(std::size_t i) const { return _states(i); }
  inline double& State::operator()(std::size_t i) { return _states(i); }

  inline bool State::operator==(const State& state) const {
    SM_ASSERT_TRUE_DBG(InitializationException, (this->_states.isApprox(this->_states)),
                       "The state contains at least one NaN value.");
    SM_ASSERT_TRUE_DBG(InitializationException, (state.state().isApprox(state.state())),
                       "The state contains at least one NaN value.");
    return _pose == state.pose() && _states == state.state();
  }

  inline bool State::operator!=(const State& state) const {
    return !(*this == state);
  }

  inline State State::cwisePlus(const State& state) const {
    return State(this->state() + state.state(), this->pose().cwisePlus(state.pose()));
  }

  inline State State::cwiseProduct(const double factor) const {
    return State(this->state() * factor, this->pose().cwiseProduct(factor));
  }

  template<class Archive>
  inline void State::serialize(Archive & ar, const unsigned int /*version*/) {
    ar & _pose;
    ar & _states;
  }

  // ************************ //
  // * StateStamped methods * //
  // ************************ //

  inline StateStamped::StateStamped() : State() { }

  inline StateStamped::StateStamped(const std::size_t sz) : State(sz) { }

  inline StateStamped::StateStamped(const State& state, const Time& stamp) : State(state), StampedType(stamp) { }

  inline StateStamped::operator Position2dStamped () const {
    return Position2dStamped(this->position(), this->stamp());
  }

  inline StateStamped::operator Pose2dStamped () const {
    return Pose2dStamped(this->pose(), this->stamp());
  }

  template<class Archive>
  inline void StateStamped::serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
    ar & boost::serialization::base_object<StampedType>(*this);
  }

} /* namespace planning2d */

#endif /* PLANNING2D_STATE_IMPLEMENTATION_HPP_ */
