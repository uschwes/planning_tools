/*
 * FeatureSingletonRobotTargetVelocity.hpp
 *
 *  Created on: Jan 21, 2016
 *      Author: uschwes
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONROBOTTARGETVELOCITY_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONROBOTTARGETVELOCITY_HPP_

//standard
#include <limits>

// Eigen
#include <Eigen/Core>

// planner interfaces
#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/MathSupport.hpp>
#include <planner_interfaces/Exceptions.hpp>

//Self
#include <probabilistic_planner/MathSupport.hpp>
#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonRobotTargetVelocity: public TargetSingletonFeature<FeatureSingletonRobotTargetVelocity> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonRobotTargetVelocity);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_robot_target_velocity";

  using TargetSingletonFeature<FeatureSingletonRobotTargetVelocity>::evaluate;

  FeatureSingletonRobotTargetVelocity(const OptAgentType& optAgentType,
                                      const double weight,
                                      const planning2d::Pose2d& target = planning2d::Pose2d(0.0, 0.0, 0.0),
                                      const double vref = 1.0)
      : TargetSingletonFeature<FeatureSingletonRobotTargetVelocity>(CLASS_NAME, optAgentType, target)
  {
    setWeight(0, weight);
    setParameters(vref);
  }
  FeatureSingletonRobotTargetVelocity(const sm::value_store::ValueStore& vpt,
                                      const planning2d::Pose2d& target = planning2d::Pose2d(0.0, 0.0, 0.0))
      : TargetSingletonFeature<FeatureSingletonRobotTargetVelocity>(CLASS_NAME, vpt, target)
  {
    setParameters(vpt.getDouble("internal_parameters/velTransRef"));
  }
  ~FeatureSingletonRobotTargetVelocity() { }

  inline void setParameters(const double velTransRef);

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonRobotTargetVelocity() : TargetSingletonFeature<FeatureSingletonRobotTargetVelocity>() { }
  inline void saveImpl(sm::value_store::ExtendibleValueStore& vpt) const override;

  double _velTransRef = std::numeric_limits<double>::signaling_NaN();
  double _velTransRefSquared = std::numeric_limits<double>::signaling_NaN();
};


void FeatureSingletonRobotTargetVelocity::setParameters(const double velTransRef) {
  SM_ASSERT_GE(planning2d::FunctionInputException, velTransRef, 0.0, "");
  _velTransRef = velTransRef;
  _velTransRefSquared = planning2d::math::square(_velTransRef);
}

template <typename Return, typename OptAgent_>
Return FeatureSingletonRobotTargetVelocity::evaluate(const planning2d::Time& /*timestamp*/, const OptAgent_& agent) const {
  using namespace ::prob_planner::math;
  SM_ASSERT_FALSE_DBG(planning2d::InitializationException, std::isnan(_velTransRef), "");
  auto vnow = agent.trajectory().getVelocityXY(agent.trajectory().getFinalTime());
  auto vref = evalIfEigen(this->getTarget().position().asVector() - agent.trajectory().getPosition(agent.trajectory().getFinalTime()));
  auto vrefSquaredNorm = vref.squaredNorm();
  auto vrefThresh = evalIfEigen(vref*piecewiseExpression(Return(1.0), _velTransRef/sqrt(vrefSquaredNorm), [vrefSquaredNorm, this](){ return evaluateIfExpression(vrefSquaredNorm) <= this->_velTransRefSquared;}));
  auto dv = evalIfEigen(vrefThresh - vnow);
  return dv.squaredNorm();
}

inline void FeatureSingletonRobotTargetVelocity::saveImpl(sm::value_store::ExtendibleValueStore& vpt) const {
  vpt.addDouble("internal_parameters/velTransRef", _velTransRef);
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONROBOTTARGETVELOCITY_HPP_ */
