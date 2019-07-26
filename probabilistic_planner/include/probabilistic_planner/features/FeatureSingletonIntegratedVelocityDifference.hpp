/*
 * FeatureSingletonIntegratedVelocityDifference.hpp
 *
 *  Created on: 04.09.2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDVELOCITYDIFFERENCE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDVELOCITYDIFFERENCE_HPP_

#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Exceptions.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonIntegratedVelocityDifference: public IntegratedSingletonFeature<FeatureSingletonIntegratedVelocityDifference> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedVelocityDifference);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_integrated_velocity_difference";

  using IntegratedSingletonFeature<FeatureSingletonIntegratedVelocityDifference>::evaluate;

  FeatureSingletonIntegratedVelocityDifference(const OptAgentType& optAgentType, const double weight, const double desiredVel = 1.0)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedVelocityDifference>(CLASS_NAME, optAgentType)
  {
    setDesiredVelocity(desiredVel);
    setWeight(0, weight);
  }
  FeatureSingletonIntegratedVelocityDifference(const sm::value_store::ValueStore& vpt)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedVelocityDifference>(CLASS_NAME, vpt)
  {
    setDesiredVelocity(vpt.getDouble("internal_parameters/desiredVelocity"));
  }
  ~FeatureSingletonIntegratedVelocityDifference() { }

  double getDesiredVelocity() const { return _desiredVelocity; }
  inline void setDesiredVelocity(double desiredVelocity);

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonIntegratedVelocityDifference() : IntegratedSingletonFeature<FeatureSingletonIntegratedVelocityDifference>() { }
  inline void saveImpl(sm::value_store::ExtendibleValueStore& vpt) const override;

  double _desiredVelocity = 1.0;
};



template <typename Return, typename OptAgent_>
Return FeatureSingletonIntegratedVelocityDifference::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  auto vel = agent.trajectory().getVelocityXY(timestamp);
  double eps = 0.1;
  auto diff = sqrt(vel.squaredNorm() + eps*eps) - (_desiredVelocity + eps);
  return diff * diff;
}

inline void FeatureSingletonIntegratedVelocityDifference::setDesiredVelocity(double desiredVelocity) {
  SM_ASSERT_GE(planning2d::FunctionInputException, desiredVelocity, 0., "");
  _desiredVelocity = desiredVelocity;
}

inline void FeatureSingletonIntegratedVelocityDifference::saveImpl(sm::value_store::ExtendibleValueStore& vpt) const {
  vpt.addDouble("internal_parameters/desiredVelocity", _desiredVelocity);
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDVELOCITYDIFFERENCE_HPP_ */
