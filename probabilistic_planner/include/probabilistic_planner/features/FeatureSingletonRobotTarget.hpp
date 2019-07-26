/*
 * FeatureSingletonRobotTarget.hpp
 *
 *  Created on: Sep 7, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONROBOTTARGET_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONROBOTTARGET_HPP_

#include <Eigen/Core>

#include <planner_interfaces/Support.hpp>

#include "FeatureStatistics.hpp"

namespace prob_planner {

class FeatureSingletonRobotTarget: public TargetSingletonFeature<FeatureSingletonRobotTarget> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonRobotTarget);

 public:
  static constexpr const char CLASS_NAME[] = "singleton_robot_target";

  using TargetSingletonFeature<FeatureSingletonRobotTarget>::evaluate;

  FeatureSingletonRobotTarget(const OptAgentType& optAgentType, const double weight, const planning2d::Pose2d& target = planning2d::Pose2d(0.0, 0.0, 0.0)) : TargetSingletonFeature<FeatureSingletonRobotTarget>(CLASS_NAME, optAgentType, target) { setWeight(0, weight); }
  FeatureSingletonRobotTarget(const sm::value_store::ValueStore& vpt, const planning2d::Pose2d& target = planning2d::Pose2d(0.0, 0.0, 0.0)) : TargetSingletonFeature<FeatureSingletonRobotTarget>(CLASS_NAME, vpt, target) { }
  ~FeatureSingletonRobotTarget() { }

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonRobotTarget() : TargetSingletonFeature<FeatureSingletonRobotTarget>() { }
};



template <typename Return, typename OptAgent_>
Return FeatureSingletonRobotTarget::evaluate(const planning2d::Time& /*timestamp*/, const OptAgent_& agent) const {
  auto dPos = evalIfEigen(agent.trajectory().getPosition(agent.trajectory().getFinalTime()) - this->getTarget().position().asVector());
  return dPos.squaredNorm();
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONROBOTTARGET_HPP_ */
