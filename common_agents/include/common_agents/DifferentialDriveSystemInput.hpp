/*
 * DifferentialDriveSystemInput.hpp
 *
 *  Created on: Jul 06, 2015
 *      Author: uschwes
 */

#ifndef COMMON_AGENTS_DIFFERENTIALDRIVESYSTEMINPUT_HPP_
#define COMMON_AGENTS_DIFFERENTIALDRIVESYSTEMINPUT_HPP_

#include <iostream>

#include <Eigen/Core>

#include <planner_interfaces/SystemInput.hpp>
#include <planner_interfaces/StampedType.hpp>

namespace common_agents {

class DifferentialDriveSystemInput : public virtual planning2d::SystemInput {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(DifferentialDriveSystemInput);
  DifferentialDriveSystemInput();
  DifferentialDriveSystemInput(const planning2d::SystemInput& input);
  DifferentialDriveSystemInput(double velTrans, double velRot);
  DifferentialDriveSystemInput(const Eigen::Vector2d& transRotVel);
  virtual ~DifferentialDriveSystemInput();

  inline const T& velocity() const { return this->data(); }
  inline T& velocity() { return this->data(); }

  inline const double& getVelTrans() const { return this->operator()(0); }
  inline const double& getVelRot() const { return this->operator()(1); }

  inline void setVelTrans(const double velTrans) { this->operator()(0) = velTrans; }
  inline void setVelRot(const double velRot) { this->operator()(1) = velRot; }

 private:

};

std::ostream& operator<<(std::ostream& os, const DifferentialDriveSystemInput& input);

class DifferentialDriveSystemInputStamped : public virtual DifferentialDriveSystemInput, public virtual planning2d::SystemInputStamped {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(DifferentialDriveSystemInputStamped);
  DifferentialDriveSystemInputStamped();
  DifferentialDriveSystemInputStamped(const planning2d::SystemInput& systemInput, const planning2d::Time& timestamp);
  DifferentialDriveSystemInputStamped(const DifferentialDriveSystemInput& systemInput, const planning2d::Time& timestamp);
  virtual ~DifferentialDriveSystemInputStamped();

};

std::ostream& operator<<(std::ostream& os, const DifferentialDriveSystemInputStamped& input);

}

#endif /* COMMON_AGENTS_DIFFERENTIALDRIVESYSTEMINPUT_HPP_ */
