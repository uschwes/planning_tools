/*
 * FeatureSingletonIntegratedStaticObstacleDistance.hpp
 *
 *  Created on: 1.12.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDSTATICOBSTACLEDISTANCE_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDSTATICOBSTACLEDISTANCE_HPP_

// standard
#include <string>

// planner interfaces
#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Exceptions.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>

// self
#include <probabilistic_planner/MathSupport.hpp>
#include <probabilistic_planner/features/FeatureStatistics.hpp>
#include <probabilistic_planner/features/GridExpressionNode.hpp>
#include <probabilistic_planner/features/CacheEntryDistanceTransform.hpp>

namespace prob_planner {

class FeatureSingletonIntegratedStaticObstacleDistance: public GridSingletonFeature<FeatureSingletonIntegratedStaticObstacleDistance> {

 private:
  typedef GridExpressionNode<float,planning2d::grid::InterpolationMethod::CUBIC_CATMULL_ROM,
      planning2d::grid::ExtrapolationMethod::CONSTANT> GridExprNode;

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedStaticObstacleDistance);
  typedef typename planning2d::Map<float>::ExtrapolationMethod ExtrapolationMethod;

 public:
  static constexpr const char CLASS_NAME[] = "singleton_integrated_static_obstacle_distance";
  using GridSingletonFeature<FeatureSingletonIntegratedStaticObstacleDistance>::evaluate;

  FeatureSingletonIntegratedStaticObstacleDistance(const OptAgentType& optAgentType, const double weight, const double scalingFactor = 1.0, const bool activateScaling = true)
      : GridSingletonFeature<FeatureSingletonIntegratedStaticObstacleDistance>(CLASS_NAME, optAgentType, scalingFactor, activateScaling)
  {
    setWeight(0, weight);
  }
  FeatureSingletonIntegratedStaticObstacleDistance(const sm::value_store::ValueStore& vpt)
      : GridSingletonFeature<FeatureSingletonIntegratedStaticObstacleDistance>(CLASS_NAME, vpt)
  {
    this->setSigmoidParameters(vpt.getDouble("internal_parameters/sigmoidHeight").get(),
                               vpt.getDouble("internal_parameters/sigmoidScale").get(),
                               vpt.getDouble("internal_parameters/sigmoidShift").get());
  }
  ~FeatureSingletonIntegratedStaticObstacleDistance() { }

  inline void setSigmoidParameters(const double height, const double scale, const double shift);

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent, const SceneSnapshot& snapshot) const;

 private:
  FeatureSingletonIntegratedStaticObstacleDistance() : GridSingletonFeature<FeatureSingletonIntegratedStaticObstacleDistance>() { }

  inline virtual void saveImpl(sm::value_store::ExtendibleValueStore& vpt) const override;

 private:
  double _sigmoidHeight = 1.0;
  double _sigmoidScale = 1.0;
  double _sigmoidShift = 0.0;

};



template <typename Return, typename OptAgent_>
inline Return FeatureSingletonIntegratedStaticObstacleDistance::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent, const SceneSnapshot& snapshot) const {
  planning2d::Map<float>::Ptr dt;
  retrieveOrCreateCacheEntrySignedDistanceTransform(snapshot, dt);
  auto pos = agent.trajectory().getPosition(timestamp);
  auto vel = agent.trajectory().getVelocityXY(timestamp);
  auto vel2 = vel.squaredNorm();
  const double agentRadius = agent.getAgent()->getDiscApproximation(1).getRadius(0);
  // here we apply an inverse sigmoidal function on the signed distance transform of the grid.
  // we shift the center of the sigmoid by the agent radius to measure the distance from its border
  // we scale the sigmoid by squared velocity norm in order to allow the agent pass narrow passages with small speed
  return inverseSigmoid(aslam::backend::ScalarExpression(GridExprNode::Ptr(
      new GridExprNode(pos, dt))), _sigmoidHeight, _sigmoidScale, _sigmoidShift + agentRadius) * (vel2);
}

template<>
inline double FeatureSingletonIntegratedStaticObstacleDistance::evaluate<double, OptAgent>(const planning2d::Time& timestamp, const OptAgent& agent, const SceneSnapshot& snapshot) const {
  return FeatureSingletonIntegratedStaticObstacleDistance::evaluate<aslam::backend::ScalarExpression, collectors::OptAgentExpressionWrapper>(timestamp, agent, snapshot).toScalar();
}

inline void FeatureSingletonIntegratedStaticObstacleDistance::setSigmoidParameters(const double height, const double scale, const double shift)
{
  SM_ASSERT_GT(planning2d::FunctionInputException, height, 0., "");
  SM_ASSERT_GT(planning2d::FunctionInputException, scale, 0., "");

  _sigmoidHeight = height;
  _sigmoidScale = scale;
  _sigmoidShift = shift;
}

inline void FeatureSingletonIntegratedStaticObstacleDistance::saveImpl(sm::value_store::ExtendibleValueStore& vpt) const {
  vpt.addDouble("internal_parameters/sigmoidHeight", _sigmoidHeight);
  vpt.addDouble("internal_parameters/sigmoidScale", _sigmoidScale);
  vpt.addDouble("internal_parameters/sigmoidShift", _sigmoidShift);
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDSTATICOBSTACLEDISTANCE_HPP_ */
