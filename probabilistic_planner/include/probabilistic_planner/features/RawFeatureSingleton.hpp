/*
 * RawFeatureSingleton.hpp
 *
 *  Created on: Jun 16, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATURESINGLETON_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATURESINGLETON_HPP_

#include <vector>

#include <sm/BoostPropertyTree.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/Support.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/ErrorTerm.hpp>

#include <probabilistic_planner/features/RawFeature.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/OptAgent.hpp>

namespace prob_planner {

class RawFeatureSingleton: public RawFeature {

 public:
  typedef RawFeature parent_t;
  PLANNING_2D_POINTER_TYPEDEFS(RawFeatureSingleton);

 public:
  RawFeatureSingleton(const std::string& name,
                      const OptAgentType& optAgentType,
                      const std::size_t dimension = 1,
                      const double scalingFactor = 1.0,
                      const bool activateScaling = true) : RawFeature(name, optAgentType, dimension, scalingFactor, activateScaling) { }
  RawFeatureSingleton(const std::string& name,
                      const sm::value_store::ValueStore& vpt,
                      const std::size_t dimension = 1) : RawFeature(name, vpt, dimension) { }
  ~RawFeatureSingleton() { }

  /**
   * Get all the agents which are taken into account by the specific feature
   * @param timestamp time at which feature has to be evaluated
   * @param scene overall scene including all agents
   * @return vector of agents which are suitable for this feature at the given timestamp (agents extracted from scene)
   */
  virtual const std::vector<OptAgent::ConstRef> getSuitableAgents(const planning2d::Time& timestamp,
                                                                  const ContinuousScene& scene) const;

  /**
   * Interface required for product features
   * @return 1D matrix expression, TODO: extend to vector-valued expressions at some point
   */
  virtual aslam::backend::GenericMatrixExpression<1, 1> getExpression(const planning2d::Time& timestamp, const OptAgent& agent) const;

 protected:
  RawFeatureSingleton() : RawFeature() { }

};

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_RAWFEATURESINGLETON_HPP_ */
