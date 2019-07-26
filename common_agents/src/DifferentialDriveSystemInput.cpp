/*
 * DifferentialDriveSystemInput.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: uschwes
 */

#include <common_agents/DifferentialDriveSystemInput.hpp>

namespace common_agents {

using namespace planning2d;

/**
 * Constructor
 */
DifferentialDriveSystemInput::DifferentialDriveSystemInput() : planning2d::SystemInput(2) {

}

/**
 * Constructor
 * @param[in] velTrans  translational velocity
 * @param[in] velRot    rotational velocity
 */
DifferentialDriveSystemInput::DifferentialDriveSystemInput(double velTrans, double velRot) :
    planning2d::SystemInput(Eigen::Vector2d(velTrans, velRot)) {

}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const DifferentialDriveSystemInput& input) {
  os << "(tv: " << input.getVelTrans() << ", rv: " << input.getVelRot() << ")";
  return os;
}

/**
 * Constructor
 * @param[in] velocity  2D velocity with translational (index 0) and rotational (index 1) component
 * @param[in] pose      pose with position and heading information
 */
DifferentialDriveSystemInput::DifferentialDriveSystemInput(const Eigen::Vector2d& velocity) :
    planning2d::SystemInput(velocity)
{
}

/**
 * Constructor
 * @param[in] state object from parent class
 */
DifferentialDriveSystemInput::DifferentialDriveSystemInput(const SystemInput& systemInput) :
    planning2d::SystemInput(systemInput)
{
}

/**
 * Destructor
 */
DifferentialDriveSystemInput::~DifferentialDriveSystemInput()
{
}

/**
 * Constructor
 */
DifferentialDriveSystemInputStamped::DifferentialDriveSystemInputStamped()
  : planning2d::SystemInput(),
    planning2d::StampedType()
{

}

/**
 * Constructor
 * @param[in] u input for differential drive agent without timestamp
 * @param[in] timestamp
 */
DifferentialDriveSystemInputStamped::DifferentialDriveSystemInputStamped(const planning2d::SystemInput& u,
                                                             const planning2d::Time& timestamp)
  : planning2d::SystemInput(u),
    planning2d::StampedType(timestamp)
{

}

/**
 * Constructor
 * @param[in] u input for differential drive agent without timestamp
 * @param[in] timestamp
 */
DifferentialDriveSystemInputStamped::DifferentialDriveSystemInputStamped(const DifferentialDriveSystemInput& u,
                                                             const planning2d::Time& timestamp)
  : planning2d::SystemInput(u),
    planning2d::StampedType(timestamp)
{

}

/**
 * Destructor
 */
DifferentialDriveSystemInputStamped::~DifferentialDriveSystemInputStamped()
{

}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const DifferentialDriveSystemInputStamped& input) {
  os << "(tv: " << input.getVelTrans() << ", rv: " << input.getVelRot() << ", stamp: " << input.stamp() << ")";
  return os;
}

}   /* namespace planning2d */
