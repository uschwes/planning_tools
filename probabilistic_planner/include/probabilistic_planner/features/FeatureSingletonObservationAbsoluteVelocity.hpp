/*
 * FeatureSingletonObservationAbsoluteVelocity.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONABSOLUTEVELOCITY_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONABSOLUTEVELOCITY_HPP_

#include <aslam/backend/ExpressionErrorTerm.hpp>

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonObservationAbsoluteVelocity: public ObservationSingletonFeature<FeatureSingletonObservationAbsoluteVelocity> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonObservationAbsoluteVelocity);
  static constexpr const char CLASS_NAME[] = "measurement_absolute_velocity";

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using ObservationSingletonFeature<FeatureSingletonObservationAbsoluteVelocity>::evaluate;

  FeatureSingletonObservationAbsoluteVelocity(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : ObservationSingletonFeature<FeatureSingletonObservationAbsoluteVelocity>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonObservationAbsoluteVelocity(const sm::value_store::ValueStore& vpt) :
    ObservationSingletonFeature<FeatureSingletonObservationAbsoluteVelocity>(CLASS_NAME, vpt) { }
  ~FeatureSingletonObservationAbsoluteVelocity() { }

  /// \brief Evaluate the feature at a given timestamp for an agent and a measurement
  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const;

 private:
  FeatureSingletonObservationAbsoluteVelocity() :
    ObservationSingletonFeature<FeatureSingletonObservationAbsoluteVelocity>() { }

};



template <typename Return, typename OptAgent_>
inline Return FeatureSingletonObservationAbsoluteVelocity::evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const {
  auto velocity = agent.trajectory().getVelocityXY(timestamp);
  auto sqrtVelSquaredNorm = sqrt(velocity.squaredNorm());
  auto dVel = (sqrtVelSquaredNorm - s.state().norm())
            * (sqrtVelSquaredNorm - s.state().norm())
            * 0.5
            * s.invCov()(3,3); // Assuming that the measurement uncertainty for the velocity is isotropic
  return dVel;
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONABSOLUTEVELOCITY_HPP_ */
