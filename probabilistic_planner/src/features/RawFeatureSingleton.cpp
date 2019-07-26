/*
 * RawFeatureSingleton.cpp
 *
 *  Created on: Jul 14, 2015
 *      Author: pfmark
 */

#include <probabilistic_planner/features/RawFeatureSingleton.hpp>

namespace prob_planner {

const std::vector<OptAgent::ConstRef> RawFeatureSingleton::getSuitableAgents(const planning2d::Time& timestamp,
                                                                             const ContinuousScene& scene) const {

  std::vector<OptAgent::ConstRef> agents;
  agents.reserve(scene.numberOfAgents());

  for ( auto& agent : scene.getOptAgentContainer()) {
    // has to be the correct agent type and the trajectory needs to be defined at the desired timestamp
    if (agent.second.isActive()
        && ( (agent.second.getType() == getOptAgentType()) || (getOptAgentType() == OptAgentType::ALL) )
        && agent.second.trajectory().contains(timestamp))
    {
      agents.push_back( std::ref(agent.second) );
      SM_ALL_STREAM_NAMED("feature_computation", "Agent (" << agent.first << ", " << agent.second.getType() << ") FITS for feature \"" << name() << "\" with agent type " <<
                          getOptAgentType() << " at timestamp " << timestamp.format(planning2d::time::Formatter(planning2d::time::SEC,2)));
    } else {
      SM_ALL_STREAM_NAMED("feature_computation", "Agent (" << agent.first << ", " << agent.second.getType() << ") DOES NOT fit for feature \"" << name() << "\" with agent type " <<
                          getOptAgentType() << " at timestamp " << timestamp.format(planning2d::time::Formatter(planning2d::time::SEC,2)));
    }
  }
  return agents;
}

aslam::backend::GenericMatrixExpression<1, 1> RawFeatureSingleton::getExpression(const planning2d::Time& /*timestamp*/, const OptAgent& /*agent*/) const {
  SM_THROW(planning2d::NoImplementationException, "Has to be implemented by derived class");
}

} /* namespace prob_planner */
