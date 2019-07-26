/*
 * HolonomicState.hpp
 *
 *  Created on: Jul 6, 2015
 *      Author: uschwes
 */

#ifndef COMMON_AGENTS_HOLONOMICSTATE_HPP_
#define COMMON_AGENTS_HOLONOMICSTATE_HPP_

#include <iostream>

#include <Eigen/Core>

#include <planner_interfaces/State.hpp>
#include <planner_interfaces/StampedType.hpp>
#include <planner_interfaces/Support.hpp>

// Boost includes
#include <boost/serialization/base_object.hpp>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

namespace common_agents {

class HolonomicState : public virtual planning2d::State {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(HolonomicState);
  HolonomicState();
  HolonomicState(const planning2d::State& state);
  HolonomicState(const HolonomicState& state);
  HolonomicState(double vx, double vy, const planning2d::Pose2d& pose);
  HolonomicState(double vx, double vy, double yaw, const planning2d::Position2d& pos);
  HolonomicState(const Eigen::Vector2d& velocity, const planning2d::Pose2d& pose);
  HolonomicState(const double x, const double y, const double yaw, const double vx, const double vy);
  ~HolonomicState();

  /**
   * Access to velocity vector
   * @return translational and rotational velocity
   */
  inline const T& velocity() const { return this->state(); }
  inline T& velocity() { return this->state(); }

  inline const double& getVelX() const { return this->state()(0); }
  inline const double& getVelY() const { return this->state()(1); }

  inline void setVelX(const double vx) { this->state()(0) = vx; }
  inline void setVelY(const double vy) { this->state()(1) = vy; }

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:

};

template<class Archive>
inline void HolonomicState::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
}

std::ostream& operator<<(std::ostream& os, const HolonomicState& state);


class HolonomicStateStamped : public HolonomicState, public planning2d::StateStamped {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(HolonomicStateStamped);
  HolonomicStateStamped();
  HolonomicStateStamped(const planning2d::State& state, const planning2d::Time& timestamp);
  HolonomicStateStamped(const planning2d::StateStamped& stateStamped);
  HolonomicStateStamped(const HolonomicState& state, const planning2d::Time& timestamp);
  virtual ~HolonomicStateStamped();

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

};


template<class Archive>
inline void HolonomicStateStamped::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & boost::serialization::base_object<common_agents::HolonomicState>(*this);
  ar & boost::serialization::base_object<planning2d::StateStamped>(*this);
}

std::ostream& operator<<(std::ostream& os, const HolonomicStateStamped& state);

}

#endif /* COMMON_AGENTS_HOLONOMICSTATE_HPP_ */
