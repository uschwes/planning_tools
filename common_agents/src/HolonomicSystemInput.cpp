/*
 * HolonomicSystemInput.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: uschwes
 */

#include <common_agents/HolonomicSystemInput.hpp>

namespace common_agents {

using namespace planning2d;

/**
 * Constructor
 */
HolonomicSystemInput::HolonomicSystemInput() : planning2d::SystemInput(2) {

}

/**
 * Constructor
 * @param[in] vx
 * @param[in] vy
 */
HolonomicSystemInput::HolonomicSystemInput(double vx, double vy) :
    planning2d::SystemInput(Eigen::Vector2d(vx, vy)) {

}

/**
 * Constructor
 * @param[in] velocity  2D velocity
 * @param[in] pose      pose with position and heading information
 */
HolonomicSystemInput::HolonomicSystemInput(const Eigen::Vector2d& velocity) :
    planning2d::SystemInput(velocity)
{
}

/**
 * Constructor
 * @param[in] state object from parent class
 */
HolonomicSystemInput::HolonomicSystemInput(const SystemInput& systemInput) :
    planning2d::SystemInput(systemInput)
{
}

/**
 * Destructor
 */
HolonomicSystemInput::~HolonomicSystemInput()
{
}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const HolonomicSystemInput& input) {
  os << "(vx: " << input.getVelX() << ", vy: " << input.getVelY() << ")";
  return os;
}

/**
 * Constructor
 */
HolonomicSystemInputStamped::HolonomicSystemInputStamped()
  : planning2d::SystemInput(),
    planning2d::StampedType()
{

}

/**
 * Constructor
 * @param[in] u input for holonomic agent without timestamp
 * @param[in] timestamp
 */
HolonomicSystemInputStamped::HolonomicSystemInputStamped(const planning2d::SystemInput& u,
                                                         const planning2d::Time& timestamp)
  : planning2d::SystemInput(u),
    planning2d::StampedType(timestamp)
{

}

/**
 * Constructor
 * @param[in] u input for holonomic agent without timestamp
 * @param[in] timestamp
 */
HolonomicSystemInputStamped::HolonomicSystemInputStamped(const HolonomicSystemInput& u,
                                                         const planning2d::Time& timestamp)
  : planning2d::SystemInput(u),
    planning2d::StampedType(timestamp)
{

}

/**
 * Destructor
 */
HolonomicSystemInputStamped::~HolonomicSystemInputStamped()
{

}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const HolonomicSystemInputStamped& input) {
  os << "(vx: " << input.getVelX() << ", vy: " << input.getVelY() << ", stamp: " << input.stamp() << ")";
  return os;
}

}   /* namespace planning2d */
