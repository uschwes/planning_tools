/*
 * HolonomicSystemInput.hpp
 *
 *  Created on: Jul 6, 2015
 *      Author: uschwes
 */

#ifndef COMMON_AGENTS_HOLONOMICSYSTEMINPUT_HPP_
#define COMMON_AGENTS_HOLONOMICSYSTEMINPUT_HPP_

#include <iostream>

#include <Eigen/Core>

#include <planner_interfaces/SystemInput.hpp>
#include <planner_interfaces/StampedType.hpp>

namespace common_agents {

class HolonomicSystemInput : public virtual planning2d::SystemInput {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(HolonomicSystemInput);
  HolonomicSystemInput();
  HolonomicSystemInput(const planning2d::SystemInput& input);
  HolonomicSystemInput(double vx, double vy);
  HolonomicSystemInput(const Eigen::Vector2d& transRotVel);
  virtual ~HolonomicSystemInput();

  inline const T& velocity() const { return this->data(); }
  inline T& velocity() { return this->data(); }

  inline const double& getVelX() const { return this->operator()(0); }
  inline const double& getVelY() const { return this->operator()(1); }

  inline void setVelX(const double vx) { this->operator()(0) = vx; }
  inline void setVelY(const double vy) { this->operator()(1) = vy; }

 private:

};

std::ostream& operator<<(std::ostream& os, const HolonomicSystemInput& input);


class HolonomicSystemInputStamped : public virtual HolonomicSystemInput, public virtual planning2d::SystemInputStamped {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(HolonomicSystemInputStamped);
  HolonomicSystemInputStamped();
  HolonomicSystemInputStamped(const planning2d::SystemInput& systemInput, const planning2d::Time& timestamp);
  HolonomicSystemInputStamped(const HolonomicSystemInput& systemInput, const planning2d::Time& timestamp);
  virtual ~HolonomicSystemInputStamped();

};

std::ostream& operator<<(std::ostream& os, const HolonomicSystemInputStamped& input);

}

#endif /* COMMON_AGENTS_HOLONOMICSYSTEMINPUT_HPP_ */
