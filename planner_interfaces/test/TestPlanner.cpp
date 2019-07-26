/*
 * TestPlanner.cpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

// standard includes
#include <algorithm>

// Schweizer Messer includes
#include <sm/logging.hpp>

// self includes
#include "../include/planner_interfaces/Exceptions.hpp"
#include "TestPlanner.hpp"

using namespace planning2d;

TestPlanner::TestPlanner() {
  _egoAgent.reset(new TestAgent(0 /*id*/));
}

ReturnCode TestPlanner::callbackOccupancyGrid(const OccupancyGridStamped& grid) {
  _occupancyGrid = grid;
  return RETCODE_OK;
}

ReturnCode TestPlanner::callbackCurrentState(const StateStamped & state) {
  _egoAgent->stateStamped() = state;
  return RETCODE_OK;
}

ReturnCode TestPlanner::callbackReferencePath(const Path& path) {
  _referencePath = path;
  return RETCODE_OK;
}

ReturnCode TestPlanner::callbackDynamicObjects(const std::vector<Agent::ConstPtr>& dynamicObjects) {

  // make a deep copy
  _dynamicObjects.clear();
  struct Cloner { Agent::Ptr operator()(const Agent::ConstPtr& ptr) const { return ptr->clone(); } };
  std::transform(dynamicObjects.begin(), dynamicObjects.end(), std::back_inserter(_dynamicObjects), Cloner());

  SM_ASSERT_EQ(RuntimeException, _dynamicObjects.size(), dynamicObjects.size(), "");
  for (std::size_t i=0; i<dynamicObjects.size(); i++)
    SM_ASSERT_TRUE(RuntimeException, _dynamicObjects[i]->stateStamped() == dynamicObjects[i]->stateStamped(), "");

  return RETCODE_OK;
}

ReturnCode TestPlanner::computePlan(const Time& /* currentTime */, StateInputTrajectory& stateInputSequency) {

  if (_occupancyGrid.isOccupied(_egoAgent->stateStamped().pose().position())) {
    SM_ERROR_STREAM("Ego agent is in collision at beginning of planning cycle");
    return RETCODE_ROBOT_IN_COLLISION_NOW;
  }

  // Simulate all agents forward and check for collision in every step
  for (Time t = Time(_egoAgent->stateStamped().stamp()); t < Time(_egoAgent->stateStamped().stamp()) + _simulationHorizon; t+=_simulationGranularity) {

    // simulate all other agents
    for (std::size_t i=0; i<_dynamicObjects.size(); i++) {
      _dynamicObjects[i]->applyInput(_dynamicObjects[i]->computeControlOutput(_referencePath[0]), _simulationGranularity);
    }
    // simulate ego agent
    ::planning2d::SystemInput input = _egoAgent->computeControlOutput(_referencePath[0]);
    stateInputSequency.push_back(StateInputPairStamped((State)_egoAgent->stateStamped(), input, t));
    _egoAgent->applyInput(input, _simulationGranularity);
  }

  return RETCODE_OK;
}
