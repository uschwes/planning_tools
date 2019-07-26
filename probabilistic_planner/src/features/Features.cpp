/*
 * Features.cpp
 *
 *  Created on: Aug 14, 2015
 *      Author: pfmark
 */

#include <probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedSigmoidDistance.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationAbsoluteVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationVelocityVector.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationHeading.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedDirectionOfMotion.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocityDifference.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTarget.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTargetVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp>
#include <probabilistic_planner/features/FeatureProduct.hpp>

namespace prob_planner {

  constexpr char FeatureSingletonIntegratedBarrierVelocity::CLASS_NAME[];
  constexpr char FeatureSingletonIntegratedAcceleration::CLASS_NAME[];
  constexpr char FeatureSingletonIntegratedRotationRate::CLASS_NAME[];
  constexpr char FeatureSingletonIntegratedVelocity::CLASS_NAME[];
  constexpr char FeatureSingletonIntegratedVelocityDifference::CLASS_NAME[];
  constexpr char FeatureSingletonIntegratedDirectionOfMotion::CLASS_NAME[];
  constexpr char FeatureSingletonObservationPosition2d::CLASS_NAME[];
  constexpr char FeatureSingletonObservationHeading::CLASS_NAME[];
  constexpr char FeatureSingletonObservationAbsoluteVelocity::CLASS_NAME[];
  constexpr char FeatureSingletonObservationVelocityVector::CLASS_NAME[];

  constexpr char FeaturePairwiseIntegratedInverseDistance::CLASS_NAME[];
  constexpr char FeaturePairwiseIntegratedDistance::CLASS_NAME[];
  constexpr char FeaturePairwiseIntegratedSigmoidDistance::CLASS_NAME[];

  constexpr char FeatureSingletonRobotTarget::CLASS_NAME[];
  constexpr char FeatureSingletonRobotTargetVelocity::CLASS_NAME[];

  constexpr char FeatureSingletonIntegratedStaticObstacleDistance::CLASS_NAME[];

  constexpr char FeatureSingletonProduct::CLASS_NAME[];

} /* namespace prob_planner */
