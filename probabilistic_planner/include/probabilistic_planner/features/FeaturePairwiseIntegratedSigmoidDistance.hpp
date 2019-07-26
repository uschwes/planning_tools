/*
 * FeaturePairwiseIntegratedSigmoidDistance.hpp
 *
 *  Created on: 20.11.2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDSIGMOIDDISTANCE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDSIGMOIDDISTANCE_HPP_

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"
#include <probabilistic_planner/MathSupport.hpp>

namespace prob_planner {

class FeaturePairwiseIntegratedSigmoidDistance : public IntegratedPairwiseFeature<FeaturePairwiseIntegratedSigmoidDistance> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeaturePairwiseIntegratedSigmoidDistance);

 public:
  static constexpr const char CLASS_NAME[] = "pairwise_integrated_sigmoid_distance";

  using IntegratedPairwiseFeature<FeaturePairwiseIntegratedSigmoidDistance>::evaluate;
 
  FeaturePairwiseIntegratedSigmoidDistance(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : IntegratedPairwiseFeature<FeaturePairwiseIntegratedSigmoidDistance>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeaturePairwiseIntegratedSigmoidDistance(const sm::value_store::ValueStore& vpt) :
      IntegratedPairwiseFeature<FeaturePairwiseIntegratedSigmoidDistance>(CLASS_NAME, vpt) {
    this->setSigmoidParameters(vpt.getDouble("internal_parameters/sigmoidHeight").get(),
                               vpt.getDouble("internal_parameters/sigmoidScale").get(),
                               vpt.getDouble("internal_parameters/sigmoidShift").get());
  }
  ~FeaturePairwiseIntegratedSigmoidDistance() { }

  inline void setSigmoidParameters(const double height, const double scale, const double shift);

  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_& agent0, const OptAgent_& agent1) const;

 private:
  FeaturePairwiseIntegratedSigmoidDistance() : IntegratedPairwiseFeature<FeaturePairwiseIntegratedSigmoidDistance>() { }
  inline void saveImpl(sm::value_store::ExtendibleValueStore& vpt) const override;

  // Sigmoid parameters
  double _sigmoidHeight = 1.0;
  double _sigmoidScale = 1.0;
  double _sigmoidShift = 0.0;
};

inline void FeaturePairwiseIntegratedSigmoidDistance::setSigmoidParameters(const double height, const double scale, const double shift)
{
  SM_ASSERT_GT(planning2d::FunctionInputException, height, 0., "");
  SM_ASSERT_GT(planning2d::FunctionInputException, scale, 0., "");

  _sigmoidHeight = height;
  _sigmoidScale = scale;
  _sigmoidShift = shift;
}

inline void FeaturePairwiseIntegratedSigmoidDistance::saveImpl(sm::value_store::ExtendibleValueStore& vpt) const {
  vpt.addDouble("internal_parameters/sigmoidHeight", _sigmoidHeight);
  vpt.addDouble("internal_parameters/sigmoidScale", _sigmoidScale);
  vpt.addDouble("internal_parameters/sigmoidShift", _sigmoidShift);
}

template<typename Return, typename OptAgent_>
inline Return FeaturePairwiseIntegratedSigmoidDistance::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent0, const OptAgent_& agent1) const {
  using namespace ::prob_planner::math;
  auto dPos = evalIfEigen(agent0.trajectory().getPosition(timestamp) - agent1.trajectory().getPosition(timestamp));
  auto distSquared = dPos.squaredNorm();
  const double agentRadius0 = agent0.getAgent()->getDiscApproximation(1).getRadius(0);
  const double agentRadius1 = agent1.getAgent()->getDiscApproximation(1).getRadius(0);
  const double distCenters = agentRadius0 + agentRadius1;
  return inverseSigmoid(distSquared, _sigmoidHeight, _sigmoidScale, _sigmoidShift + distCenters*distCenters);
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATUREPAIRWISEINTEGRATEDSIGMOIDDISTANCE_HPP_ */
