/*
 * FeatureSingletonIntegratedRotationRate.hpp
 *
 *  Created on: 22.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDROTATIONRATE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDROTATIONRATE_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonIntegratedRotationRate: public IntegratedSingletonFeature<FeatureSingletonIntegratedRotationRate> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedRotationRate);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_integrated_rotation_rate";

  using IntegratedSingletonFeature<FeatureSingletonIntegratedRotationRate>::evaluate;

  FeatureSingletonIntegratedRotationRate(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedRotationRate>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonIntegratedRotationRate(const sm::value_store::ValueStore& vpt) : IntegratedSingletonFeature<FeatureSingletonIntegratedRotationRate>(CLASS_NAME, vpt) { }
  ~FeatureSingletonIntegratedRotationRate() { }

  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonIntegratedRotationRate() : IntegratedSingletonFeature<FeatureSingletonIntegratedRotationRate>() { }
};




template <typename Return, typename OptAgent_>
Return FeatureSingletonIntegratedRotationRate::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  return agent.trajectory().getRotationRateSquared(timestamp);
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDROTATIONRATE_HPP_ */
