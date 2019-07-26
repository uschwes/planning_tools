/*
 * TestPlanner.hpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

#ifndef TESTPLANNER_HPP_
#define TESTPLANNER_HPP_

// self includes
#include <planner_interfaces/TestAgent.hpp>
#include <planner_interfaces/ReferencePathPlannerInterface.hpp>

namespace planning2d {

class TestPlanner : public ReferencePathPlannerInterface {

 public:

  TestPlanner();

  virtual ReturnCode computePlan(const Time& currentTime, StateInputTrajectory& stateInputSequency);
  virtual ReturnCode callbackOccupancyGrid(const OccupancyGridStamped& grid);
  virtual ReturnCode callbackCurrentState(const StateStamped & state);
  virtual ReturnCode callbackReferencePath(const Path& path);
  virtual ReturnCode callbackDynamicObjects(const std::vector<Agent::ConstPtr>& dynamicObjects);

 private:
  Agent::Ptr _egoAgent;
  OccupancyGridStamped _occupancyGrid;
  std::vector<Agent::Ptr> _dynamicObjects;
  Path _referencePath;

  Duration _simulationHorizon;
  Duration _simulationGranularity;

}; /* class TestPlanner */

} /* namespace planning2d */

#endif /* TESTPLANNER_HPP_ */
