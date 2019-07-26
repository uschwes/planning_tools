/*
 * State.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef PLANNING2D_STATE_HPP_
#define PLANNING2D_STATE_HPP_

// self includes
#include "Support.hpp"
#include "Pose2d.hpp"
#include "Time.hpp"
#include "StampedType.hpp"

namespace planning2d {

/**
 * @brief An agent's state
 * A state contains a 2D pose plus additional state variables like e.g. speed, rotation rate etc...
 */
class State {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(State);
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> T; //! data type for storing the state variables

  //! Default constructor. Sets state size to zero.
  inline State();
  //! Constructor initializing state variables with size \var sz. Initializes them to sinaling_NaN in Debug mode. Uninitialized in Release mode.
  inline State(const std::size_t sz);
  //! Construct state object from pose and state variables given as Eigen vector
  inline State(const T& states, const Pose2d& pose);
  //! Destructor
  virtual ~State() { }

  //! Returns x-position
  inline double x() const;
  //! Returns mutable reference to x-position
  inline double& x();
  //! Returns y-position
  inline double y() const;
  //! Returns mutable reference to y-position
  inline double& y();
  //! Returns yaw angle
  inline double yaw() const;
  //! Returns mutable reference to yaw angle
  inline double& yaw();
  //! Returns const reference to position
  inline const Position2d& position() const;
  //! Returns mutable reference to position
  inline Position2d& position();
  //! Cast to const reference to position
  inline operator const Position2d& () const;
  //! Cast to mutable reference to position
  inline operator Position2d& ();
  //! Returns const reference to pose
  inline const Pose2d& pose() const;
  //! Returns mutable reference to pose
  inline Pose2d& pose();
  //! Cast to const reference to pose
  inline operator const Pose2d& () const;
  //! Cast to mutable reference to pose
  inline operator Pose2d& ();
  //! Returns const reference to state variables
  inline const T& state() const;
  //! Returns mutable reference to state variables
  inline T& state();
  //! Returns number of state variables
  inline std::size_t dimension() const;
  //! Returns number of state variables + pose2d dimension
  inline std::size_t fullDimension() const;
  //! returns const reference to state variable at index \var i
  inline const double& operator()(std::size_t i) const;
  //! returns mutable reference to state variable at index \var i
  inline double& operator()(std::size_t i);

  //! equality operator
  inline bool operator==(const State& state) const;
  //! inequality operator
  inline bool operator!=(const State& state) const;
  //! addition operator
  inline State cwisePlus(const State& state) const;
  //! multiplication operator
  inline State cwiseProduct(const double factor) const;

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:
  Pose2d _pose; //! The two-dimensional pose
  T _states; //! additional state variables

}; /* class State */

/**
 * @brief State with a timestamp
 */
class StateStamped : public virtual State, public StampedType {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(StateStamped);

  inline StateStamped();
  inline StateStamped(const std::size_t sz);
  inline StateStamped(const State& state, const Time& stamp);
  virtual ~StateStamped() { }

  //! Cast to Position2dStamped
  inline operator Position2dStamped () const;
  //! Cast to Pose2dStamped
  inline operator Pose2dStamped () const;

  //! equality operator
  inline bool operator==(const StateStamped& s) const { return State::operator==(s) && StampedType::operator==(s); }
  //! inequality operator
  inline bool operator!=(const StateStamped& s) const { return !(*this == s); }
  //! addition operator
  inline State operator+(const State& state) const = delete;
  //! multiplication operator
  inline State operator*(const double factor) const = delete;

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 protected:
  inline StateStamped(const Time& stamp) : StampedType(stamp) { }

}; /* class State */

} /* namespace planning2d */

#include "implementation/StateImplementation.hpp"

#endif /* PLANNING2D_STATE_HPP_ */
