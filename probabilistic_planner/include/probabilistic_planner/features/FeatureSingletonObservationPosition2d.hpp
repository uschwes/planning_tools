/*
 * FeatureSingletonObservationPosition2d.hpp
 *
 *  Created on: 03.08.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONPOSITION2D_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONPOSITION2D_HPP_

#include <aslam/backend/ExpressionErrorTerm.hpp>

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonObservationPosition2d: public ObservationSingletonFeature<FeatureSingletonObservationPosition2d> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonObservationPosition2d);

 public:
  static constexpr const char CLASS_NAME[] = "measurement_position2d";

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  using ObservationSingletonFeature<FeatureSingletonObservationPosition2d>::evaluate;

  FeatureSingletonObservationPosition2d(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : ObservationSingletonFeature<FeatureSingletonObservationPosition2d>(CLASS_NAME, optAgentType, scalingFactor, activateScaling) { setWeight(0, weight); }
  FeatureSingletonObservationPosition2d(const sm::value_store::ValueStore& vpt) :
    ObservationSingletonFeature<FeatureSingletonObservationPosition2d>(CLASS_NAME, vpt) { }
  ~FeatureSingletonObservationPosition2d() { }

  /// \brief Evaluate the feature at a given timestamp for an agent and a measurement
  template <typename Return, typename OptAgent_>
  inline Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const;

 private:
  FeatureSingletonObservationPosition2d() :
    ObservationSingletonFeature<FeatureSingletonObservationPosition2d>() { }

};



template <typename Return, typename OptAgent_>
inline Return FeatureSingletonObservationPosition2d::evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const StateWithUncertainty& s) const {
  auto dpos = evalIfEigen(agent.trajectory().getPosition(timestamp) - s.position().asVector());
  return ((dpos.transpose()*(0.5*s.invCov().block<2,2>(0,0)).eval()*dpos))[0];
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONOBSERVATIONPOSITION2D_HPP_ */
