/*
 * FeatureSingletonIntegratedVelocity.hpp
 *
 *  Created on: 21.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDVELOCITY_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDVELOCITY_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonIntegratedVelocity: public IntegratedSingletonFeature<FeatureSingletonIntegratedVelocity> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedVelocity);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_integrated_velocity";

  using IntegratedSingletonFeature<FeatureSingletonIntegratedVelocity>::evaluate;

  FeatureSingletonIntegratedVelocity(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedVelocity>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonIntegratedVelocity(const sm::value_store::ValueStore& vpt) : IntegratedSingletonFeature<FeatureSingletonIntegratedVelocity>(CLASS_NAME, vpt) { }
  ~FeatureSingletonIntegratedVelocity() { }

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonIntegratedVelocity() : IntegratedSingletonFeature<FeatureSingletonIntegratedVelocity>() { }

};



template <typename Return, typename OptAgent_>
Return FeatureSingletonIntegratedVelocity::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  auto vel = evalIfEigen(agent.trajectory().getVelocityXY(timestamp));
  return vel.squaredNorm();
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDVELOCITY_HPP_ */
