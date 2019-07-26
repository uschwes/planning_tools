/*
 * FeatureSingletonObservationVelocityVector.hpp
 *
 *  Created on: Aug 31, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONVELOCITYVECTOR_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONVELOCITYVECTOR_HPP_

#include <aslam/backend/ExpressionErrorTerm.hpp>

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonObservationVelocityVector: public ObservationSingletonFeature<FeatureSingletonObservationVelocityVector> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonObservationVelocityVector);
  static constexpr const char CLASS_NAME[] = "measurement_velocity_vector";

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using ObservationSingletonFeature<FeatureSingletonObservationVelocityVector>::evaluate;

  FeatureSingletonObservationVelocityVector(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : ObservationSingletonFeature<FeatureSingletonObservationVelocityVector>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonObservationVelocityVector(const sm::value_store::ValueStore& vpt) :
    ObservationSingletonFeature<FeatureSingletonObservationVelocityVector>(CLASS_NAME, vpt) { }
  ~FeatureSingletonObservationVelocityVector() { }

  /// \brief Evaluate the feature at a given timestamp for an agent and a measurement
  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const;

 private:
  FeatureSingletonObservationVelocityVector() :
    ObservationSingletonFeature<FeatureSingletonObservationVelocityVector>() { }

};



template <typename Return, typename OptAgent_>
inline Return FeatureSingletonObservationVelocityVector::evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const {
  auto velocity = agent.trajectory().getVelocityXY(timestamp);
  auto dv = evalIfEigen(velocity - s.state());
  return (dv.transpose()*s.invCov().block<2,2>(3,3)*dv)[0] * 0.5;
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONVELOCITYVECTOR_HPP_ */
