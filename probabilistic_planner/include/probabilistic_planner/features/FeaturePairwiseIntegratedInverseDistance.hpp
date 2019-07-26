/*
 * FeaturePairwiseIntegratedInverseDistance.hpp
 *
 *  Created on: 21.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDINVERSEDISTANCE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDINVERSEDISTANCE_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeaturePairwiseIntegratedInverseDistance : public IntegratedPairwiseFeature<FeaturePairwiseIntegratedInverseDistance> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeaturePairwiseIntegratedInverseDistance);

 public:
  static constexpr const char CLASS_NAME[] = "pairwise_integrated_inverse_distance";

  using IntegratedPairwiseFeature<FeaturePairwiseIntegratedInverseDistance>::evaluate;
 
  FeaturePairwiseIntegratedInverseDistance(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedPairwiseFeature<FeaturePairwiseIntegratedInverseDistance>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeaturePairwiseIntegratedInverseDistance(const sm::value_store::ValueStore& vpt) :
      IntegratedPairwiseFeature<FeaturePairwiseIntegratedInverseDistance>(CLASS_NAME, vpt) { }
  ~FeaturePairwiseIntegratedInverseDistance() { }

  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_& agent0, const OptAgent_& agent1) const;

 private:
  FeaturePairwiseIntegratedInverseDistance() : IntegratedPairwiseFeature<FeaturePairwiseIntegratedInverseDistance>() { }
};



template <typename Return, typename OptAgent_>
inline Return FeaturePairwiseIntegratedInverseDistance::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent0, const OptAgent_& agent1) const {
  auto dPos = evalIfEigen(agent0.trajectory().getPosition(timestamp) - agent1.trajectory().getPosition(timestamp));
  auto distSquared = dPos.squaredNorm();
  return 1./(distSquared + Return(1.0));
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDINVERSEDISTANCE_HPP_ */
