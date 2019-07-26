/*
 * RawFeaturePairwise.hpp
 *
 *  Created on: Jun 16, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATUREPAIRWISE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATUREPAIRWISE_HPP_

#include <vector> // std::vector
#include <utility> // std::pair

#include <boost/shared_ptr.hpp>

#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Time.hpp>

#include <probabilistic_planner/features/RawFeature.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/OptAgent.hpp>

namespace prob_planner {

class RawFeaturePairwise : public RawFeature {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(RawFeaturePairwise);
  typedef std::pair<OptAgent::ConstRef, OptAgent::ConstRef> OptAgentPair;

  RawFeaturePairwise(const std::string& name,
                     const OptAgentType& optAgentType,
                     const std::size_t dimension = 1,
                     const double scalingFactor = 1.0,
                     const bool activateScaling = true) : RawFeature(name, optAgentType, dimension, scalingFactor, activateScaling) { }
  RawFeaturePairwise(const std::string& name,
                     const sm::value_store::ValueStore& vpt,
                     const std::size_t dimension = 1) : RawFeature(name, vpt, dimension) { }
  ~RawFeaturePairwise() { }

  /**
   * Get all the agents which are taken into account by the specific feature
   * @param timestamp time at which feature has to be evaluated
   * @param scene overall scene including all agents
   * @return vector of agent pairs that are taken into account by the feature at the current timestamp
   */
  virtual std::vector<OptAgentPair> getSuitableAgentPairs(const planning2d::Time& timestamp,
                                                          const ContinuousScene& scene) const;

 protected:
  RawFeaturePairwise() : RawFeature() { }

};

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATUREPAIRWISE_HPP_ */
