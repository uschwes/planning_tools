/*
 * FeatureSingletonIntegratedAcceleration.hpp
 *
 *  Created on: 22.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDACCELERATION_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDACCELERATION_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonIntegratedAcceleration: public IntegratedSingletonFeature<FeatureSingletonIntegratedAcceleration> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedAcceleration);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_integrated_acceleration";

  using IntegratedSingletonFeature<FeatureSingletonIntegratedAcceleration>::evaluate;

  FeatureSingletonIntegratedAcceleration(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedAcceleration>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonIntegratedAcceleration(const sm::value_store::ValueStore& vpt) : IntegratedSingletonFeature<FeatureSingletonIntegratedAcceleration>(CLASS_NAME, vpt) { }
  ~FeatureSingletonIntegratedAcceleration() { }

  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonIntegratedAcceleration() : IntegratedSingletonFeature<FeatureSingletonIntegratedAcceleration>() { }

};



template <typename Return, typename OptAgent_>
Return FeatureSingletonIntegratedAcceleration::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  auto acc = agent.trajectory().getAccelerationXY(timestamp);
  return acc.squaredNorm();
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDACCELERATION_HPP_ */
