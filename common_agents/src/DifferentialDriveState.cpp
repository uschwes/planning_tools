/*
 * DifferentialDriveState.cpp
 *
 *  Created on: May 4, 2015
 *      Author: pfmark
 */

#include <common_agents/DifferentialDriveState.hpp>

namespace common_agents {

using namespace planning2d;

/**
 * Constructor
 */
DifferentialDriveState::DifferentialDriveState() : State(2) {

}

/**
 * Constructor
 * @param[in] velTrans  translational velocity
 * @param[in] velRot    rotational velocity
 * @param[in] pose      pose with position and heading information
 */
DifferentialDriveState::DifferentialDriveState(const double velTrans, const double velRot, const Pose2d& pose) :
    planning2d::State(Eigen::Vector2d(velTrans, velRot), pose) {

}

/**
 * Constructor
 * @param[in] x
 * @param[in] y
 * @param[in] yaw
 * @param[in] velTrans
 * @param[in] velRot
 */
DifferentialDriveState::DifferentialDriveState(double x, const double y, const double yaw, const double velTrans, const double velRot) :
    planning2d::State(Eigen::Vector2d(velTrans, velRot), Pose2d(x, y, yaw)) {

}

/**
 * Constructor
 * @param[in] velocity  2D velocity with translational (index 0) and rotational (index 1) component
 * @param[in] pose      pose with position and heading information
 */
DifferentialDriveState::DifferentialDriveState(const Eigen::Vector2d& velocity, const Pose2d& pose) :
    planning2d::State(velocity, pose)
{
}

/**
 * Constructor
 * @param[in] state object from parent class
 */
DifferentialDriveState::DifferentialDriveState(const planning2d::State& state) :
    planning2d::State(state)
{
}

/**
 * Constructor
 * @param[in] state object from parent class
 */
DifferentialDriveState::DifferentialDriveState(const DifferentialDriveState& state) :
    planning2d::State(state)
{
}

/**
 * Destructor
 */
DifferentialDriveState::~DifferentialDriveState()
{
}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const DifferentialDriveState& state) {
  os << "(pose: " << state.pose() << ", tv: " << state.getVelTrans() << ", rv: " << state.getVelRot() << ")";
  return os;
}

/**
 * Constructor
 */
DifferentialDriveStateStamped::DifferentialDriveStateStamped()
  : planning2d::State(2)
{

}


/**
 * Constructor
 * @param[in] state state for differential drive agent without timestamp
 * @param[in] timestamp
 */
DifferentialDriveStateStamped::DifferentialDriveStateStamped(const DifferentialDriveState& state,
                                                             const planning2d::Time& timestamp)
  : planning2d::State(state),
    planning2d::StateStamped(timestamp) {

}

/**
 * Constructor
 * @param[in] state state for differential drive agent without timestamp
 * @param[in] timestamp
 */
DifferentialDriveStateStamped::DifferentialDriveStateStamped(const planning2d::State& state,
                                                             const planning2d::Time& timestamp)
  : planning2d::State(state),
    planning2d::StateStamped(timestamp) {

}

DifferentialDriveStateStamped::DifferentialDriveStateStamped(const planning2d::StateStamped& stateStamped)
  : planning2d::State(stateStamped),
    planning2d::StateStamped(stateStamped) {

}

/**
 * Destructor
 */
DifferentialDriveStateStamped::~DifferentialDriveStateStamped()
{

}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const DifferentialDriveStateStamped& state) {
  os << "(pose: " << state.pose() << ", tv: " << state.getVelTrans() << ", rv: " << state.getVelRot() << ", stamp: " << state.stamp() << ")";
  return os;
}


}   /* namespace planning2d */
