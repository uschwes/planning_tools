/*
 * testPlanner.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: pfmark
 */

#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include <sm/logging.hpp>
#include <sm/BoostPropertyTree.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

#include <fcl/shape/geometric_shapes.h>

#include <aslam/backend/OptimizerBFGS.hpp>

#include <planner_interfaces/Time.hpp>

#include <common_agents/DifferentialDriveAgent.hpp>
#include <common_agents/PedestrianAgent.hpp>

#include <probabilistic_planner/ProbabilisticPlanner.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp>
#include "Support.hpp"

using namespace std;
using namespace common_agents;
using namespace planning2d;
using namespace prob_planner;

TEST(probabilistic_planner_TESTSUITE, DISABLED_testCallbacks) {

  try {
    sm::BoostPropertyTree pt;
    sm::value_store::PropertyTreeValueStore vpt(pt);

    pt.setString("rotation_rate_pedestrian/featureClass", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
    pt.setString("rotation_rate_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("rotation_rate_pedestrian/weight",1.0);
    pt.setString("velocity_pedestrian/featureClass", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("velocity_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("velocity_pedestrian/weight", 1.0);

    ProbabilisticPlanner planner(vpt);
    EXPECT_EQ(2, planner.getFeatureContainer().getContainer().size());

    // Robot state and target
    DifferentialDriveAgent::StateStamped robotState(DifferentialDriveAgent::State(0, 0, Pose2d(0, 0, 0)), Time(1.0));
    Pose2d robotTarget(5, 5, 0);

    // Dynamic objects
    std::vector<Agent::ConstPtr> dynamicObjects;
    CollisionGeometryPtr geom(new fcl::Cylinder(0.5, 2));
    PedestrianAgent::Ptr agent1(new PedestrianAgent(false, 1, PedestrianAgent::StateStamped(PedestrianAgent::State(-1, -1, Pose2d(4, 4, 0)), Time(1.0)), geom));
    dynamicObjects.push_back(agent1);
    PedestrianAgent::Ptr agent2(new PedestrianAgent(false, 2, PedestrianAgent::StateStamped(PedestrianAgent::State(1, 0, Pose2d(-1, -3, 0)), Time(1.0)), geom));
    dynamicObjects.push_back(agent2);

    // Occupancy grid
    OccupancyGridStamped grid(Pose2d(0., 0., 0.), 0.1, OccupancyGridStamped::Size2d(100,100), OccupancyValue::FREE, Time(1.0));

    // run callbacks
    planner.callbackSetTarget(robotTarget, true);
    planner.callbackCurrentState(robotState);
    planner.callbackDynamicObjects(dynamicObjects);
    planner.callbackOccupancyGrid(grid);
    EXPECT_EQ(Time(1.0), planner.getScene().getMinTime());
    EXPECT_EQ(Time(1.0) + planner.getParameters().planningHorizon, planner.getScene().getMaxTime());
    EXPECT_EQ(3, planner.getScene().numberOfAgents()); // ego + 2 dynamic objects
    EXPECT_EQ(4, planner.getScene().getObservations().size()); // grid + ego + 2 dynamic objects

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}


TEST(probabilistic_planner_TESTSUITE, DISABLED_testComputePlan) {

  try {
    sm::BoostPropertyTree pt;
    sm::value_store::PropertyTreeValueStore vpt(pt);

    pt.setString("acceleration_pedestrian/featureClass", FeatureSingletonIntegratedAcceleration::CLASS_NAME);
    pt.setString("acceleration_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("acceleration_pedestrian/weight",1.0);
    pt.setString("velocity_pedestrian/featureClass", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("velocity_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("velocity_pedestrian/weight", 1.0);

    ProbabilisticPlanner::Optimizer::Options options;
    options.maxIterations = 10;
    options.numThreadsJacobian = 1;

    ProbabilisticPlanner planner(vpt, options);

    // Robot state and target
    DifferentialDriveAgent::StateStamped robotState(DifferentialDriveAgent::State(1., 1., Pose2d(0., 0., 0.)), Time(1.0));
    Pose2d robotTarget(5, 5, 0);
    planner.callbackSetTarget(robotTarget, true);
    planner.callbackCurrentState(robotState);

    DifferentialDriveAgent::StateStamped robotState_3(DifferentialDriveAgent::State(1., 1., Pose2d(0., 0., 0.)), Time(3.0));
    planner.callbackCurrentState(robotState_3);

    DifferentialDriveAgent::StateStamped robotState_7(DifferentialDriveAgent::State(1., 1., Pose2d(0., 0., 0.)), Time(7.0));
    planner.callbackCurrentState(robotState_7);

    // Dynamic objects
    std::vector<Agent::ConstPtr> dynamicObjects;
    CollisionGeometryPtr geom(new fcl::Cylinder(0.5, 2.0));
    PedestrianAgent::Ptr agent1(new PedestrianAgent(false, 1, PedestrianAgent::StateStamped(PedestrianAgent::State(-1., -1., Pose2d(4., 4., 0.)), Time(1.0)), geom));
    dynamicObjects.push_back(agent1);
    PedestrianAgent::Ptr agent2(new PedestrianAgent(false, 2, PedestrianAgent::StateStamped(PedestrianAgent::State(1., 0., Pose2d(-1., -3., 0.)), Time(1.0)), geom));
    dynamicObjects.push_back(agent2);
    planner.callbackDynamicObjects(dynamicObjects);

    std::vector<Agent::ConstPtr> dynamicObjects_3;
    PedestrianAgent::Ptr agent1_3(new PedestrianAgent(false, 1, PedestrianAgent::StateStamped(PedestrianAgent::State(-1., -1., Pose2d(4., 4., 0.)), Time(3.0)), geom));
    dynamicObjects_3.push_back(agent1_3);
    PedestrianAgent::Ptr agent2_3(new PedestrianAgent(false, 2, PedestrianAgent::StateStamped(PedestrianAgent::State(1., 0., Pose2d(-1., -3., 0.)), Time(3.0)), geom));
    dynamicObjects_3.push_back(agent2_3);
    planner.callbackDynamicObjects(dynamicObjects_3);

    std::vector<Agent::ConstPtr> dynamicObjects_7;
    PedestrianAgent::Ptr agent1_7(new PedestrianAgent(false, 1, PedestrianAgent::StateStamped(PedestrianAgent::State(-1., -1., Pose2d(4., 4., 0.)), Time(7.0)), geom));
    dynamicObjects_7.push_back(agent1_7);
    PedestrianAgent::Ptr agent2_7(new PedestrianAgent(false, 2, PedestrianAgent::StateStamped(PedestrianAgent::State(1., 0., Pose2d(-1., -3., 0.)), Time(7.0)), geom));
    dynamicObjects_7.push_back(agent2_7);
    planner.callbackDynamicObjects(dynamicObjects_7);

    // run tests
    EXPECT_EQ(3, planner.getScene().numberOfAgents()); // ids: ego, 1, 2

    StateInputTrajectory plannedTrajectory;
    planner.computePlan(Time(7.0), plannedTrajectory);

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}
