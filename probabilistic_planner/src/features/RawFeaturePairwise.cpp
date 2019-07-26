/*
 * RawFeaturePairwise.cpp
 *
 *  Created on: Jul 14, 2015
 *      Author: pfmark
 */

#include <probabilistic_planner/features/RawFeaturePairwise.hpp>

using namespace std;
using namespace planning2d;

namespace prob_planner {

std::vector<RawFeaturePairwise::OptAgentPair> RawFeaturePairwise::getSuitableAgentPairs(const planning2d::Time& timestamp,
                                                                                        const ContinuousScene& scene) const {

  std::vector<OptAgentPair> agentPairs;

  if (scene.numberOfAgents() < 2)
    return agentPairs;

  agentPairs.reserve(scene.numberOfAgents()*scene.numberOfAgents());

  for (auto it0 = scene.getOptAgentContainer().begin(); it0 != scene.getOptAgentContainer().end(); ++it0) {
    auto it1 = it0;
    it1++;
    for (; it1 != scene.getOptAgentContainer().end(); ++it1) {
      // has to be the correct agent types and the trajectory needs to be defined at the desired timestamp
      if ((it0->second.isActive() && it1->second.isActive())
          &&  ( ( it0->second.getType() == getOptAgentType()
                 && it1->second.getType() == getOptAgentType() )
              || (getOptAgentType() == OptAgentType::ALL) )
          && it0->second.trajectory().contains(timestamp)
          && it1->second.trajectory().contains(timestamp))
      {
        agentPairs.push_back( std::make_pair(std::ref(it0->second), std::ref(it1->second)) );
      }
    }
  }
  return agentPairs;
}

} /* namespace prob_planner */
