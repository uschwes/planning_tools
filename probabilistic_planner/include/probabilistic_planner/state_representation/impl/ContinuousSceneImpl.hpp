/*
 * ContinuousSceneImpl.hpp
 *
 *  Created on: 03.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_CONTINUOUSSCENEIMPL_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_CONTINUOUSSCENEIMPL_HPP_

#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

namespace prob_planner {

template<class Archive>
inline void ContinuousScene::serialize(Archive & ar, const unsigned int /*version*/)
{
  ar & _observations;
  ar & _agentContainer;
}

template <template <typename, typename...> class Container>
void ContinuousScene::removeAllAgentsExcept(const Container<planning2d::Id>& agentsToKeep) {
  for (auto it = _agentContainer.begin(); it != _agentContainer.end(); ) {
    if (agentsToKeep.find(it->first) == agentsToKeep.end()) {
      SM_FINEST_STREAM_NAMED("probabilistic_planner", "Removing agent " << it->first);
      it = _agentContainer.erase(it);
    } else {
      ++it;
    }
  }
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_CONTINUOUSSCENEIMPL_HPP_ */
