/*
 * FeaturePairwiseIntegratedDistance.hpp
 *
 *  Created on: 31.08.2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDDISTANCE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDDISTANCE_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeaturePairwiseIntegratedDistance : public IntegratedPairwiseFeature<FeaturePairwiseIntegratedDistance> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeaturePairwiseIntegratedDistance);

 public:
  static constexpr const char CLASS_NAME[] = "pairwise_integrated_distance";

  using IntegratedPairwiseFeature<FeaturePairwiseIntegratedDistance>::evaluate;
 
  FeaturePairwiseIntegratedDistance(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedPairwiseFeature<FeaturePairwiseIntegratedDistance>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeaturePairwiseIntegratedDistance(const sm::value_store::ValueStore& vpt) :
      IntegratedPairwiseFeature<FeaturePairwiseIntegratedDistance>(CLASS_NAME, vpt) { }
  ~FeaturePairwiseIntegratedDistance() { }

  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_& agent0, const OptAgent_& agent1) const;

 private:
  FeaturePairwiseIntegratedDistance() : IntegratedPairwiseFeature<FeaturePairwiseIntegratedDistance>() { }

};



template<typename Return, typename OptAgent_>
inline Return FeaturePairwiseIntegratedDistance::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent0, const OptAgent_& agent1) const {
  auto dPos = evalIfEigen(agent0.trajectory().getPosition(timestamp) - agent1.trajectory().getPosition(timestamp));
  return dPos.squaredNorm();
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDDISTANCE_HPP_ */
