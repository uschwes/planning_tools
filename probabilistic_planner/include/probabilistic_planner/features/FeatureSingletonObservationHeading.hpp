/*
 * FeatureSingletonObservationHeading.hpp
 *
 *  Created on: 19.08.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONHEADING_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONHEADING_HPP_

#include <cmath>

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonObservationHeading: public ObservationSingletonFeature<FeatureSingletonObservationHeading> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonObservationHeading);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using ObservationSingletonFeature<FeatureSingletonObservationHeading>::evaluate;

  FeatureSingletonObservationHeading(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : ObservationSingletonFeature<FeatureSingletonObservationHeading>(CLASS_NAME, optAgentType, scalingFactor, activateScaling)  { setWeight(0, weight); }
  FeatureSingletonObservationHeading(const sm::value_store::ValueStore& vpt) :
    ObservationSingletonFeature<FeatureSingletonObservationHeading>(CLASS_NAME, vpt) { }
  ~FeatureSingletonObservationHeading() { }

  /// \brief Evaluate the feature at a given timestamp for an agent and a measurement
  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const;

 private:
  FeatureSingletonObservationHeading() :
    ObservationSingletonFeature<FeatureSingletonObservationHeading>() { }

 public:
  static constexpr const char CLASS_NAME[] = "observation_heading";

};



template <typename Return, typename OptAgent_>
inline Return FeatureSingletonObservationHeading::evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const {
  // TODO: singularity if |v| = 0.0
  auto velSpline = agent.trajectory().getVelocityXY(timestamp);
  auto velSplineNorm = sqrt((velSpline.squaredNorm()));
  Eigen::Vector2d velMeasUnit(cos(s.yaw()), sin(s.yaw()));
  auto theta = acos(((velSpline.transpose()*velMeasUnit)[0]/velSplineNorm));
  SM_ASSERT_TRUE(planning2d::RuntimeException, std::isfinite(evaluateIfExpression(theta)), "Norm of spline velocity: " << velSplineNorm);
  return theta*(0.5*s.invCov()(2,2))*theta;

}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONHEADING_HPP_ */
