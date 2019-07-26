/*
 * generatePropertyTreeXml.cpp
 *
 *  Created on: Aug 17, 2015
 *      Author: pfmark
 */

// standard includes
#include <string>

// Boost
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>

// Schweizer Messer
#include <sm/BoostPropertyTree.hpp>
#include <sm/random.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>
#include <sm/logging.hpp>

// Pairwise features
#include <probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedSigmoidDistance.hpp>

// Physical trajectory attribute features
#include <probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocityDifference.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedDirectionOfMotion.hpp>

// Observation features
#include <probabilistic_planner/features/FeatureSingletonObservationAbsoluteVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationVelocityVector.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationHeading.hpp>

// Barrier functions / soft constraints
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>

// Target features
#include <probabilistic_planner/features/FeatureSingletonRobotTarget.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTargetVelocity.hpp>

// Grid features
#include <probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp>

void setCommonAttributes(sm::BoostPropertyTree& pt,
                         const std::string& entry,
                         const bool scalingActive = true,
                         const bool activeForLearning = true,
                         const Eigen::VectorXd& scalingFactor = Eigen::VectorXd::Ones(1),
                         const double weight = 1.0)
{
  pt.setString(entry + "/agentType", "PEDESTRIAN");
  pt.setDouble(entry + "/weight", weight);
  if (scalingFactor.size() == 1) {
    pt.setDouble(entry + "/scaling/factor", scalingFactor[0]);
  } else {
    for (int i=0; i<scalingFactor.size(); ++i)
      pt.setDouble(entry + "/scaling/factor/" + std::to_string(i), scalingFactor[i]);
  }
  pt.setBool(entry + "/scaling/active", scalingActive);
  pt.setBool(entry + "/activeForLearning", activeForLearning);
}

int main(int /*argc*/, char**/*argv*/) {

  using namespace prob_planner;
  sm::BoostPropertyTree::setHumanReadableInputOutput(true);
  sm::BoostPropertyTree pt;

  // Pairwise features
  pt.setString("distance_pedestrian/featureClass", FeaturePairwiseIntegratedDistance::CLASS_NAME);
  setCommonAttributes(pt, "distance_pedestrian");

  pt.setString("inverse_distance_pedestrian/featureClass", FeaturePairwiseIntegratedInverseDistance::CLASS_NAME);
  setCommonAttributes(pt, "inverse_distance_pedestrian");

  pt.setString("sigmoid_distance_pedestrian/featureClass", FeaturePairwiseIntegratedSigmoidDistance::CLASS_NAME);
  setCommonAttributes(pt, "sigmoid_distance_pedestrian");

  // Physical trajectory attribute features
  pt.setString("acceleration_pedestrian/featureClass", FeatureSingletonIntegratedAcceleration::CLASS_NAME);
  setCommonAttributes(pt, "acceleration_pedestrian");

  pt.setString("rotation_rate_pedestrian/featureClass", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
  setCommonAttributes(pt, "rotation_rate_pedestrian");

  pt.setString("velocity_pedestrian/featureClass", FeatureSingletonIntegratedVelocity::CLASS_NAME);
  setCommonAttributes(pt, "velocity_pedestrian");

  pt.setString("velocity_difference_1_pedestrian/featureClass", FeatureSingletonIntegratedVelocityDifference::CLASS_NAME);
  setCommonAttributes(pt, "velocity_difference_1_pedestrian");
  pt.setDouble("velocity_difference_1_pedestrian/internal_parameters/desiredVelocity", 1.0);

  pt.setString("direction_of_motion/featureClass", FeatureSingletonIntegratedDirectionOfMotion::CLASS_NAME);
  setCommonAttributes(pt, "direction_of_motion");

  pt.setString("absolute_velocity_measurement/featureClass", FeatureSingletonObservationAbsoluteVelocity::CLASS_NAME);
  setCommonAttributes(pt, "absolute_velocity_measurement", false, false);

  // Observation features
  pt.setString("measurement2d_pedestrian/featureClass", FeatureSingletonObservationPosition2d::CLASS_NAME);
  setCommonAttributes(pt, "measurement2d_pedestrian", false, false);

  pt.setString("measurement_heading_pedestrian/featureClass", FeatureSingletonObservationHeading::CLASS_NAME);
  setCommonAttributes(pt, "measurement_heading_pedestrian", false, false);

  pt.setString("measurement_velocity_vector/featureClass", FeatureSingletonObservationVelocityVector::CLASS_NAME);
  setCommonAttributes(pt, "measurement_velocity_vector", false, false);

  // Barrier functions / soft constraints
  pt.setString("barrier_velocity/featureClass", FeatureSingletonIntegratedBarrierVelocity::CLASS_NAME);
  setCommonAttributes(pt, "barrier_velocity");
  pt.setDouble("barrier_velocity/internal_parameters/maximumVelocity", 1.0);
  pt.setDouble("barrier_velocity/internal_parameters/factor", 1000.0);

  // Target features
  pt.setString("target_position/featureClass", FeatureSingletonRobotTarget::CLASS_NAME);
  setCommonAttributes(pt, "target_position");

  pt.setString("target_velocity/featureClass", FeatureSingletonRobotTarget::CLASS_NAME);
  setCommonAttributes(pt, "target_velocity");
  pt.setDouble("target_velocity/internal_parameters/velTransRef", 1.0);

  // Grid features
  pt.setString("static_obstacle_distance_pedestrian/featureClass", FeatureSingletonIntegratedStaticObstacleDistance::CLASS_NAME);
  setCommonAttributes(pt, "static_obstacle_distance_pedestrian");
  pt.setDouble("static_obstacle_distance_pedestrian/internal_parameters/sigmoidHeight", 1.0);
  pt.setDouble("static_obstacle_distance_pedestrian/internal_parameters/sigmoidScale", 1.0);
  pt.setDouble("static_obstacle_distance_pedestrian/internal_parameters/sigmoidShift", 1.0);
  pt.setString("static_obstacle_distance_pedestrian/internal_parameters/extrapolationMethod", "CONSTANT");

  // Serialize
  boost::filesystem::path out( boost::filesystem::current_path() );
  out /= "propertyTreeFeatures.example.xml";
  pt.saveXml(out);
  SM_INFO_STREAM("Wrote " << out);

  return EXIT_SUCCESS;

}
