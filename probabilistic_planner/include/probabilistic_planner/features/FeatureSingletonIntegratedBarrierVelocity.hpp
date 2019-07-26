/*
 * FeatureSingletonIntegratedBarrierVelocity.hpp
 *
 *  Created on: Aug 28, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDBARRIERVELOCITY_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDBARRIERVELOCITY_HPP_


#include <planner_interfaces/Support.hpp>
#include <planner_interfaces/Exceptions.hpp>

#include "FeatureStatistics.hpp"
#include <probabilistic_planner/MathSupport.hpp>

namespace prob_planner {

class FeatureSingletonIntegratedBarrierVelocity: public IntegratedSingletonFeature<FeatureSingletonIntegratedBarrierVelocity> {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(FeatureSingletonIntegratedBarrierVelocity);

 public:
  static constexpr const char CLASS_NAME[] = "barrier_velocity";

  using IntegratedSingletonFeature<FeatureSingletonIntegratedBarrierVelocity>::evaluate;

  FeatureSingletonIntegratedBarrierVelocity(const OptAgentType& optAgentType, const double weight, const double maxVel = 1.0, const double factor = 1000.0)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedBarrierVelocity>(CLASS_NAME, optAgentType)
  {
    setMaximumVelocity(maxVel);
    setWeight(0, weight);
    setFactor(factor);
  }
  FeatureSingletonIntegratedBarrierVelocity(const sm::value_store::ValueStore& vpt)
      : IntegratedSingletonFeature<FeatureSingletonIntegratedBarrierVelocity>(CLASS_NAME, vpt)
  {
    setMaximumVelocity(vpt.getDouble("internal_parameters/maximumVelocity"));
    setFactor(vpt.getDouble("internal_parameters/factor"));
  }

  ~FeatureSingletonIntegratedBarrierVelocity() { }

  double getMaximumVelocity() const { return _maximumVelocity; }
  inline void setMaximumVelocity(double maximumVelocity);

  double getFactor() const { return _factor; }
  void setFactor(double factor) { _factor = factor; }

  /// \brief Evaluate the feature at a given timestamp for an agent
  template <typename Return, typename OptAgent_>
  Return evaluate(const planning2d::Time& timestamp, const OptAgent_ & agent) const;

 private:
  FeatureSingletonIntegratedBarrierVelocity() : IntegratedSingletonFeature<FeatureSingletonIntegratedBarrierVelocity>() { }
  inline void saveImpl(sm::value_store::ExtendibleValueStore& vpt) const override;

  double _maximumVelocity = 1.0;
  double _maximumVelocitySquared = 1.0;
  double _factor = 1000.0;
};


template <typename Return, typename OptAgent_>
Return FeatureSingletonIntegratedBarrierVelocity::evaluate(const planning2d::Time& timestamp, const OptAgent_& agent) const {
  using namespace ::prob_planner::math;
  auto vel = agent.trajectory().getVelocityXY(timestamp);
  auto absVelSquared = vel.squaredNorm();
  return piecewiseExpression(Return(0.0), powerExpression((absVelSquared-_maximumVelocitySquared), 2) * _factor, [absVelSquared, this](){ return absVelSquared <= this->_maximumVelocitySquared;});
}

inline void FeatureSingletonIntegratedBarrierVelocity::setMaximumVelocity(double maximumVelocity)
{
  SM_ASSERT_GE(planning2d::ParameterException, maximumVelocity, 0.0, "Maximum velocity has to be >= 0.")
  _maximumVelocity = maximumVelocity;
  _maximumVelocitySquared = _maximumVelocity*_maximumVelocity;
}

inline void FeatureSingletonIntegratedBarrierVelocity::saveImpl(sm::value_store::ExtendibleValueStore& vpt) const {
  vpt.addDouble("internal_parameters/maximumVelocity", _maximumVelocity);
  vpt.addDouble("internal_parameters/factor", _factor);
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURESINGLETONINTEGRATEDBARRIERVELOCITY_HPP_ */
