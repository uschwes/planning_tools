/*
 * HolonomicState.cpp
 *
 *  Created on: Jul 6, 2015
 *      Author: uschwes
 */

#include <common_agents/HolonomicState.hpp>

// Boost includes
#include <boost/serialization/base_object.hpp>

// Schweizer Messer includes
#include <sm/eigen/serialization.hpp>

namespace common_agents {

using namespace planning2d;

/**
 * Constructor
 */
HolonomicState::HolonomicState() : State(2) {

}

/**
 * Constructor
 * @param[in] vx   velocity in x-direction
 * @param[in] vy   velocity in y-direction
 * @param[in] pose pose with position and heading information
 */
HolonomicState::HolonomicState(double vx, double vy, const Pose2d& pose) :
    planning2d::State(Eigen::Vector2d(vx, vy), pose) {

}

/**
 * Constructor
 * @param vx  velocity in x-direction
 * @param vy  velocity in y-direction
 * @param yaw heading
 * @param pos 2D position
 */
HolonomicState::HolonomicState(double vx, double vy, double yaw, const planning2d::Position2d& pos) :
    planning2d::State(Eigen::Vector2d(vx, vy), planning2d::Pose2d(pos, yaw)) {

}

/**
 * Constructor
 * @param[in] velocity  2D velocity
 * @param[in] pose      pose with position and heading information
 */
HolonomicState::HolonomicState(const Eigen::Vector2d& velocity, const Pose2d& pose) :
    planning2d::State(velocity, pose)
{
}

/**
 * Constructor
 * @param[in] state object from parent class
 */
HolonomicState::HolonomicState(const planning2d::State& state) :
    planning2d::State(state)
{
}

/**
 * Copy constructor
 * @param[in] state
 */
HolonomicState::HolonomicState(const HolonomicState& state) :
    planning2d::State(state)
{
}

/**
 * Constructor
 * @param x     x-coordinate
 * @param y     y-coordinate
 * @param yaw   heading
 * @param vx    x-velocity
 * @param vy    y-velocity
 */
HolonomicState::HolonomicState(const double x, const double y, const double yaw, const double vx, const double vy) :
    planning2d::State(Eigen::Vector2d(vx, vy), planning2d::Pose2d(x, y, yaw)) {

}

/**
 * Destructor
 */
HolonomicState::~HolonomicState()
{
}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const HolonomicState& state) {
  os << "(pose: " << state.pose() << ", vx: " << state.getVelX() << ", vy: " << state.getVelY() << ")";
  return os;
}

/**
 * Constructor
 */
HolonomicStateStamped::HolonomicStateStamped()
  : planning2d::State(2)
{

}

/**
 * Constructor
 * @param[in] state state for holonomic agent without timestamp
 * @param[in] timestamp
 */
HolonomicStateStamped::HolonomicStateStamped(const HolonomicState& state,
                                             const planning2d::Time& timestamp)
  : planning2d::State(state),
    planning2d::StateStamped(timestamp) {

}

/**
 * Constructor
 * @param[in] state state for differential drive agent without timestamp
 * @param[in] timestamp
 */
HolonomicStateStamped::HolonomicStateStamped(const planning2d::State& state,
                                             const planning2d::Time& timestamp)
  : planning2d::State(state),
    planning2d::StateStamped(timestamp) {

}

/**
 * Constructor
 * @param[in] state state for differential drive agent without timestamp
 * @param[in] timestamp
 */
HolonomicStateStamped::HolonomicStateStamped(const planning2d::StateStamped& stateStamped)
  : planning2d::State(stateStamped),
    planning2d::StateStamped(stateStamped) {

}

/**
 * Destructor
 */
HolonomicStateStamped::~HolonomicStateStamped()
{

}

/// @brief Overloaded stream operator
std::ostream& operator<<(std::ostream& os, const HolonomicStateStamped& state) {
  os << "(pose: " << state.pose() << ", vx: " << state.getVelX() << ", vy: " << state.getVelY() << ", stamp: " << state.stamp() << ")";
  return os;
}

}   /* namespace planning2d */
