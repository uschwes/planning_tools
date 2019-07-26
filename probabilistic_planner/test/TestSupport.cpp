/*
 * TestSupport.cpp
 *
 *  Created on: 31.08.2015
 *      Author: Ulrich Schwesinger
 */

// gtest
#include <gtest/gtest.h>

// FCL includes
#include <fcl/shape/geometric_shapes.h>

// self includes
#include "Support.hpp"

// other package includes
#include <common_agents/PedestrianAgent.hpp>


using namespace prob_planner;
using namespace planning2d;
using namespace common_agents;

TEST(probabilistic_planner_TESTSUITE, populateScene) {

  try {
    ContinuousScene::Ptr scene(new ContinuousScene());
    const std::size_t nAgents = 4;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    populateScene(*scene, false, nAgents, t0, timeHorizon);
    EXPECT_EQ(nAgents, scene->numberOfAgents());

    auto agentContainer = scene->getOptAgentContainer();

    for (auto& it : agentContainer) {
      EXPECT_EQ(OptAgentType::UNKNOWN, it.second.getType());
      EXPECT_EQ(Time(0.0), it.second.trajectory().getStartTime());
      EXPECT_EQ(Time(10.0), it.second.trajectory().getFinalTime());
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(probabilistic_planner_TESTSUITE, populateSceneCoordinates) {

  try {
    ContinuousScene scene;
    std::map<planning2d::Id, StateTrajectory> trajCoords;
    trajCoords[0] = StateTrajectory( {PedestrianAgent::StateStamped(PedestrianAgent::State(0.0, 0.0, 0.0, 0.0, 0.0), Time(0.0)),
                                      PedestrianAgent::StateStamped(PedestrianAgent::State(4.0, 4.0, 0.0, 0.0, 0.0), Time(10.0))} );
    trajCoords[1] = StateTrajectory( {PedestrianAgent::StateStamped(PedestrianAgent::State(1.0, 1.0, 0.0, 0.0, 0.0), Time(1.0)),
                                      PedestrianAgent::StateStamped(PedestrianAgent::State(5.0, 5.0, 0.0, 0.0, 0.0), Time(11.0))} );
    trajCoords[2] = StateTrajectory( {PedestrianAgent::StateStamped(PedestrianAgent::State(2.0, 2.0, 0.0, 0.0, 0.0), Time(2.0)),
                                      PedestrianAgent::StateStamped(PedestrianAgent::State(6.0, 6.0, 0.0, 0.0, 0.0), Time(12.0))} );

    populateSceneCoordinates(scene, trajCoords);
    EXPECT_EQ(3, scene.numberOfAgents());

    auto optA = scene.getOptAgent(0);
    EXPECT_EQ(OptAgentType::UNKNOWN, optA.getType());

    EXPECT_EQ(Time(0.0), optA.trajectory().getStartTime());
    EXPECT_EQ(Time(10.0), optA.trajectory().getFinalTime());
    EXPECT_LT( (trajCoords[0].front().pose().position() - optA.trajectory().getPosition2d(Time(0.0))).norm(), 1e-3);
    EXPECT_LT( (trajCoords[0].back().pose().position() - optA.trajectory().getPosition2d(Time(10.0))).norm(), 1e-3);

    optA = scene.getOptAgent(1);
    EXPECT_EQ(OptAgentType::UNKNOWN, optA.getType());
    EXPECT_EQ(Time(1.0), optA.trajectory().getStartTime());
    EXPECT_EQ(Time(11.0), optA.trajectory().getFinalTime());
    EXPECT_LT( (trajCoords[1].front().pose().position() - optA.trajectory().getPosition2d(Time(1.0))).norm(), 1e-3);
    EXPECT_LT( (trajCoords[1].back().pose().position() - optA.trajectory().getPosition2d(Time(11.0))).norm(), 1e-3);

    optA = scene.getOptAgent(2);
    EXPECT_EQ(OptAgentType::UNKNOWN, optA.getType());
    EXPECT_EQ(Time(2.0), optA.trajectory().getStartTime());
    EXPECT_EQ(Time(12.0), optA.trajectory().getFinalTime());
    EXPECT_LT( (trajCoords[2].front().pose().position() - optA.trajectory().getPosition2d(Time(2.0))).norm(), 1e-3);
    EXPECT_LT( (trajCoords[2].back().pose().position() - optA.trajectory().getPosition2d(Time(12.0))).norm(), 1e-3);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(probabilistic_planner_TESTSUITE, createStateTrajectoryRandomWalk) {

  const double sigma = 1.0;
  const StateStamped s0(State(Eigen::Vector2d(0.0, 0.0), Pose2d(Position2d(0.0, 0.0), 0.0)), Time((double)0.0));
  const Duration dt(0.1);

  StateTrajectory trajectory = createStateTrajectoryRandomWalk(10, dt, s0, sigma);
  ASSERT_TRUE( trajectory.front().stamp() == s0.stamp());

  for (size_t cnt = 1; cnt<trajectory.size(); ++cnt) {
    ASSERT_LT( (trajectory[cnt].pose().position() - trajectory[cnt-1].pose().position()).norm() , 2.0*4.0*sigma);
    ASSERT_LT( std::remainder(trajectory[cnt].pose().yaw() - trajectory[cnt-1].pose().yaw(), planning2d::TWOPI) , 4.0*sigma);
    ASSERT_LT( (trajectory[cnt].state() - trajectory[cnt-1].state()).norm() , 2.0*4.0*sigma);
    ASSERT_TRUE( trajectory[cnt].stamp() == trajectory[cnt-1].stamp() + dt) <<
        "stamp[" << cnt << "] = " << trajectory[cnt].stamp() <<
        ", stamp[" << cnt-1 << "]= " << trajectory[cnt-1].stamp();
  }
}
