/*
 * FeatureSingletonIntegratedDirectionOfMotion.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDDIRECTIONOFMOTION_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDDIRECTIONOFMOTION_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"
#include <probabilistic_planner/MathSupport.hpp>

namespace prob_planner {

class FeatureSingletonIntegratedDirectionOfMotion: public IntegratedSingletonFeature<FeatureSingletonIntegratedDirectionOfMotion> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedDirectionOfMotion);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_integrated_direction_of_motion";
  using IntegratedSingletonFeature<FeatureSingletonIntegratedDirectionOfMotion>::evaluate;

  FeatureSingletonIntegratedDirectionOfMotion(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedDirectionOfMotion>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonIntegratedDirectionOfMotion(const sm::value_store::ValueStore& vpt) : IntegratedSingletonFeature<FeatureSingletonIntegratedDirectionOfMotion>(CLASS_NAME, vpt) { }
  ~FeatureSingletonIntegratedDirectionOfMotion() { }

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonIntegratedDirectionOfMotion() : IntegratedSingletonFeature<FeatureSingletonIntegratedDirectionOfMotion>() { }
};


template <typename Return, typename OptAgent_>
Return FeatureSingletonIntegratedDirectionOfMotion::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  using namespace ::prob_planner::math;
  auto initialVelocity = agent.getAgent()->stateStamped().state();  // initial velocity
  auto velSpline = evalIfEigen(agent.trajectory().getVelocityXY(timestamp));     // current velocity
  auto diff = initialVelocity - velSpline;
  return diff.squaredNorm();

} /* namespace prob_planner */

} /* end namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDDIRECTIONOFMOTION_HPP_ */
