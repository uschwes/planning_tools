/*
 * OptAgentImpl.hpp
 *
 *  Created on: 03.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_OPTAGENTIMPL_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_OPTAGENTIMPL_HPP_

#include <boost/serialization/shared_ptr.hpp>

namespace prob_planner {

void OptAgent::addDesignVariables(aslam::backend::OptimizationProblem& problem, bool autoActivate) {
  _trajectory->addDesignVariables(problem, isActive() && autoActivate);
  if (!isActive()) { // deactivate all design variables if agent is not active
    _trajectory->activateAllDesignVariables(false);
  }
}

template<class Archive>
inline void OptAgent::serialize(Archive & ar, const unsigned int /*version*/)
{
  ar & _agent;
  ar & _trajectory;
  ar & _type;
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_OPTAGENTIMPL_HPP_ */
