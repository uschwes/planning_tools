/*
 * OptAgentPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp>
#include <probabilistic_planner/state_representation/OptAgent.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

inline void setTrajectoryWrapper(OptAgent& oa, Trajectory& t) {
  oa.trajectory() = t;
}

void exportOptAgent() {

  enum_<OptAgentType>("OptAgentType")
    .value("UNKNOWN", OptAgentType::UNKNOWN)
    .value("PEDESTRIAN", OptAgentType::PEDESTRIAN)
    .value("CAR", OptAgentType::CAR)
    .value("ROBOT", OptAgentType::ROBOT)
    .value("ALL", OptAgentType::ALL)
  ;

  class_<OptAgent, OptAgent::Ptr>("OptAgent", init<planning2d::Agent::ConstPtr, Trajectory::Ptr, const OptAgentType&>("OptAgent(Agent a, Trajectory t, OptAgentType type): Constructor"))
    .add_property("agent", &OptAgent::getAgent, &OptAgent::setAgent)
    .add_property("trajectory", make_function( (const Trajectory& (OptAgent::*) (void) const)&OptAgent::trajectory, return_internal_reference<>()), &setTrajectoryWrapper, "Optimizable trajectory")
    .add_property("type", &OptAgent::getType, &OptAgent::setType, "The agent type")
    .add_property("id", &OptAgent::getId, "The id stored in the Agent attached to the OptAgent")
    .add_property("active", &OptAgent::isActive, &OptAgent::setActive, "Whether ot not the opt agent is active for optimization")
  ;

  implicitly_convertible<OptAgent::Ptr, OptAgent::ConstPtr >();

} /* void exportOptAgent() */
