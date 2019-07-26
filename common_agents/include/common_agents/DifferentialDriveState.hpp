/*
 * DifferentialDriveState.hpp
 *
 *  Created on: Apr 30, 2015
 *      Author: pfmark
 */

#ifndef COMMON_AGENTS_DIFFERENTIALDRIVESTATE_HPP_
#define COMMON_AGENTS_DIFFERENTIALDRIVESTATE_HPP_

#include <iostream>
#include <math.h>

#include <Eigen/Core>

#include <planner_interfaces/State.hpp>
#include <planner_interfaces/StampedType.hpp>
#include <planner_interfaces/Support.hpp>

// Boost includes
#include <boost/serialization/base_object.hpp>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

namespace common_agents {

class DifferentialDriveState : public virtual planning2d::State {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(DifferentialDriveState);
  DifferentialDriveState();
  DifferentialDriveState(const planning2d::State& state);
  DifferentialDriveState(const DifferentialDriveState& state);
  DifferentialDriveState(const double velTrans, const double velRot, const planning2d::Pose2d& pose);
  DifferentialDriveState(double x, const double y, const double yaw, const double velTrans, const double velRot);
  DifferentialDriveState(const Eigen::Vector2d& velocity, const planning2d::Pose2d& pose);  // 2D velocity consists of translational and rotational velocity
  ~DifferentialDriveState();

  /**
   * Access to velocity vector
   * @return translational and rotational velocity
   */
  inline const T& velocity() const { return this->state(); }
  inline T& velocity() { return this->state(); }


  inline const double& getVelTrans() const { return this->state()(0); }
  inline const double& getVelRot() const { return this->state()(1); }
  inline double getVelX() const { return this->state()(0)*cos(this->pose().yaw()); }
  inline double getVelY() const { return this->state()(0)*sin(this->pose().yaw()); }

  inline void setVelTrans(const double velTrans) { this->state()(0) = velTrans; }
  inline void setVelRot(const double velRot) { this->state()(1) = velRot; }

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:

};

template<class Archive>
inline void DifferentialDriveState::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(planning2d::State);
}

std::ostream& operator<<(std::ostream& os, const DifferentialDriveState& state);

class DifferentialDriveStateStamped : public DifferentialDriveState, public planning2d::StateStamped {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(DifferentialDriveStateStamped);
  DifferentialDriveStateStamped();
  DifferentialDriveStateStamped(const planning2d::State& state, const planning2d::Time& timestamp);
  DifferentialDriveStateStamped(const planning2d::StateStamped& stateStamped);
  DifferentialDriveStateStamped(const DifferentialDriveState& state, const planning2d::Time& timestamp);
  virtual ~DifferentialDriveStateStamped();

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

};

template<class Archive>
inline void DifferentialDriveStateStamped::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & boost::serialization::base_object<common_agents::DifferentialDriveState>(*this);
  ar & boost::serialization::base_object<planning2d::StateStamped>(*this);
}

std::ostream& operator<<(std::ostream& os, const DifferentialDriveStateStamped& state);

}

#endif /* COMMON_AGENTS_DIFFERENTIALDRIVESTATE_HPP_ */
