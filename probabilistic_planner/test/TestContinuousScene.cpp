#include <gtest/gtest.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/TestAgent.hpp>

#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include "Support.hpp"

using namespace std;
using namespace planning2d;
using namespace prob_planner;


TEST(ContinuousScene, ContinuousScene) {

  try {

    sm::logging::Level logLevel = sm::logging::getLevel();
    sm::logging::setLevel(sm::logging::Level::Error);

    ContinuousScene scene;
    EXPECT_EQ(true, scene.empty());
    EXPECT_EQ(0, scene.numberOfAgents());

    int numberOfAgents = 1;

    // Data memory
    std::map< int, std::pair<double,double> > startPositions;
    std::map< int, std::pair<double,double> > goalPositions;
    std::map< int, double> timestampContainer;
//    std::map< int, Trajectory> trajectoryContainer;

    // Generate agents and add to scene
    for (int i=0; i<numberOfAgents; i++) {
      TestAgent::Ptr agent = TestAgent::Ptr(new TestAgent());
      Trajectory::Ptr trajectory(new Trajectory());
      agent->setId(i);

      // Generate random coordinates
      double xStart = (std::rand() % 1000) / 100;
      double yStart = (std::rand() % 1000) / 100;
      double xGoal = (std::rand() % 1000) / 100;
      double yGoal = (std::rand() % 1000) / 100;
      double yaw = (std::rand() % 360) * M_PI/180;
      double timeDouble = 22.345;//static_cast<double>((std::rand() % 1000) / 10);

      // Assign to agent
//      auto agentState = agent->stateStamped();
      agent->stateStamped().state() = Eigen::Vector2d(1., 2.);
      agent->stateStamped().stamp() = Time(timeDouble);
      agent->stateStamped().pose() = Pose2d(xStart, yStart, yaw);

      // Generate trajectory starting at current state heading towards the goal state
      trajectory->initStraightSpline( agent->stateStamped(),
                                     StateStamped(State(Eigen::Vector2d(0.,0.), Pose2d(xGoal, yGoal, yaw)), Time(timeDouble + 10.0)),
                                     10,
                                     1.0);

      // Fuse agent and trajectory in OptAgent
      OptAgent optAgent(agent, trajectory, OptAgentType::PEDESTRIAN);
      scene.addOptAgent(optAgent);

      // Save data to be able to compare afterwards
      startPositions[i] = std::pair<double,double>(xStart, yStart);
      goalPositions[i] = std::pair<double,double>(xGoal, yGoal);
      timestampContainer[i] = timeDouble;

//      trajectory.getPosition(Time(timestampContainer[i])).x();

    }

    // Tests
    EXPECT_EQ(numberOfAgents, scene.numberOfAgents());
    EXPECT_FALSE(scene.empty());

    // Test copy method
    {
      ContinuousScene sc;
      scene.copy(sc);
      EXPECT_EQ(scene.numberOfAgents(), sc.numberOfAgents());
    }

    // Test agent removal
    {
      ContinuousScene sc;
      scene.copy(sc);
      for (int i=0; i<numberOfAgents; i++) {
        sc.removeOptAgent(Id(i));
      }
      EXPECT_EQ(0, sc.numberOfAgents());
    }

    // Test parameter getter
    {
      EXPECT_EQ(0, scene.numActiveSplineParameters()); // Not activated yet
      scene.activateAllDesignVariables(true);
      Eigen::VectorXd p = scene.activeSplineParameters();
      EXPECT_GT(scene.numActiveSplineParameters(), 0);
      EXPECT_EQ(p.size(), scene.numActiveSplineParameters()) << "p = " << p.transpose();
    }

    for (int i=0; i!=numberOfAgents; i++) {

      //***** Agent Tests *****//
      // Id
      EXPECT_EQ(i, scene.getOptAgent(i).getAgent()->getId());
      EXPECT_EQ(i, scene.getOptAgentContainer().at(i).getAgent()->getId());

      // Timestamp
      EXPECT_EQ(Time(timestampContainer[i]), scene.getOptAgent(i).getAgent()->stateStamped().stamp());
      EXPECT_EQ(Time(timestampContainer[i]), scene.getOptAgentContainer().at(i).getAgent()->stateStamped().stamp());

      // Position
      EXPECT_DOUBLE_EQ(startPositions[i].first, scene.getOptAgent(i).getAgent()->stateStamped().pose().position().x());
      EXPECT_DOUBLE_EQ(startPositions[i].second, scene.getOptAgent(i).getAgent()->stateStamped().pose().position().y());
      EXPECT_DOUBLE_EQ(startPositions[i].first, scene.getOptAgentContainer().at(i).getAgent()->stateStamped().pose().position().x());
      EXPECT_DOUBLE_EQ(startPositions[i].second, scene.getOptAgentContainer().at(i).getAgent()->stateStamped().pose().position().y());

      // State
      EXPECT_EQ(1, scene.getOptAgent(i).getAgent()->stateStamped().state()(0));
      EXPECT_EQ(2, scene.getOptAgent(i).getAgent()->stateStamped().state()(1));
      EXPECT_EQ(1, scene.getOptAgentContainer().at(i).getAgent()->stateStamped().state()(0));
      EXPECT_EQ(2, scene.getOptAgentContainer().at(i).getAgent()->stateStamped().state()(1));

      //***** Trajectory Tests *****//
      // starting positions
      EXPECT_NEAR(startPositions[i].first, scene.getOptAgent(i).trajectory().getPosition(Time(timestampContainer[i])).x(), 1e-12);
      EXPECT_NEAR(startPositions[i].second, scene.getOptAgent(i).trajectory().getPosition(Time(timestampContainer[i])).y(), 1e-12);
      EXPECT_NEAR(startPositions[i].first, scene.getOptAgentContainer().at(i).trajectory().getPosition(Time(timestampContainer[i])).x(), 1e-12);
      EXPECT_NEAR(startPositions[i].second, scene.getOptAgentContainer().at(i).trajectory().getPosition(Time(timestampContainer[i])).y(), 1e-12);

      // goal positions
      EXPECT_NEAR(goalPositions[i].first, scene.getOptAgent(i).trajectory().getPosition(Time(timestampContainer[i] + 10.0)).x(), 1e-12);
      EXPECT_NEAR(goalPositions[i].second, scene.getOptAgent(i).trajectory().getPosition(Time(timestampContainer[i] + 10.0)).y(), 1e-12);
      EXPECT_NEAR(goalPositions[i].first, scene.getOptAgentContainer().at(i).trajectory().getPosition(Time(timestampContainer[i] + 10.0)).x(), 1e-12);
      EXPECT_NEAR(goalPositions[i].second, scene.getOptAgentContainer().at(i).trajectory().getPosition(Time(timestampContainer[i] + 10.0)).y(), 1e-12);


      // Deactivate design variables of spline
      auto traj = scene.getOptAgent(0).trajectory();
      EXPECT_EQ(traj.numDesignVariables(), traj.numActiveDesignVariables());
      traj.deactiveDesignVariablesAtTime(traj.getStartTime());
      EXPECT_GT(traj.numDesignVariables(), traj.numActiveDesignVariables());

      // Timing
      double minTime = timestampContainer.begin()->second;
      for (auto stamp : timestampContainer)
        minTime = std::min(minTime, stamp.second);

      double maxTime = timestampContainer.begin()->second;
      for (auto stamp : timestampContainer)
        maxTime = std::max(maxTime, stamp.second);

      maxTime+=10.0;

      EXPECT_EQ(Time(minTime), scene.getMinTime());
      EXPECT_EQ(Time(maxTime), scene.getMaxTime());
    }

    // Test add design variables optimization problem
    {
      aslam::backend::OptimizationProblem problem;
      scene.activateAllDesignVariables(false);
      scene.addDesignVariables(problem, false);
      EXPECT_EQ(0, problem.countActiveDesignVariables());
      scene.activateAllDesignVariables(true);
      EXPECT_GT(problem.countActiveDesignVariables(), 0);
    }

    // Test updating function
    TestAgent::Ptr newAgent = TestAgent::Ptr(new TestAgent(2));
    Trajectory::Ptr newTrajectory(new Trajectory());

    double xStart = -1;
    double yStart = -1;
    double xGoal = -2;
    double yGoal = -2;
    double yaw = -180 * M_PI/180;
    double timeDouble = static_cast<double>((std::rand() % 1000) / 10);

    newAgent->stateStamped().state() = Eigen::Vector2d(1, 2);
    newAgent->stateStamped().stamp() = Time(timeDouble);
    newAgent->stateStamped().pose() = Pose2d(xStart, yStart, yaw);

    newTrajectory->initStraightSpline( newAgent->stateStamped(),
                                   StateStamped(State(Eigen::Vector2d(0,0), Pose2d(xGoal, yGoal, yaw)), Time(timeDouble + 10.0)),
                                   10,
                                   1.0);

    // update existing agent in existing scene
    OptAgent newOptAgent(newAgent, newTrajectory, OptAgentType::PEDESTRIAN);
    scene.updateOptAgent(newOptAgent);

    // Testing
    EXPECT_DOUBLE_EQ(timeDouble, scene.getOptAgent(2).getAgent()->stateStamped().stamp().toSec());
    EXPECT_DOUBLE_EQ(xStart, scene.getOptAgent(2).getAgent()->stateStamped().pose().position().x());
    EXPECT_DOUBLE_EQ(yStart, scene.getOptAgent(2).getAgent()->stateStamped().pose().position().y());
    EXPECT_DOUBLE_EQ(yaw, scene.getOptAgent(2).getAgent()->stateStamped().pose().yaw());
    EXPECT_NEAR(xStart, scene.getOptAgent(2).trajectory().getStateStamped(Time(timeDouble)).pose().position().x(), 1e-12);
    EXPECT_NEAR(yStart, scene.getOptAgent(2).trajectory().getStateStamped(Time(timeDouble)).pose().position().y(), 1e-12);
    EXPECT_NEAR(xGoal, scene.getOptAgent(2).trajectory().getStateStamped(Time(timeDouble + 10)).pose().position().x(), 1e-12);
    EXPECT_NEAR(yGoal, scene.getOptAgent(2).trajectory().getStateStamped(Time(timeDouble + 10)).pose().position().y(), 1e-12);

    // test clearing
    scene.clear();
    EXPECT_EQ(true, scene.empty());

    sm::logging::setLevel(logLevel);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(ContinuousScene, Cleanup) {

  try{
    ContinuousScene scene;
    populateScene(scene, false, 4, Time(0.0), Duration(5.0), OptAgentType::PEDESTRIAN);

    EXPECT_EQ(4, scene.numberOfAgents());
    auto snap1 = boost::make_shared<SceneSnapshot>(Time(1.0));
    auto snap2 = boost::make_shared<SceneSnapshot>(Time(2.0));
    snap1->addObject(1, StateWithUncertainty());
    snap2->addObject(2, StateWithUncertainty());
    scene.addObservation(snap1);
    scene.addObservation(snap2);
    scene.removeUnobservedAgents();
    EXPECT_EQ(2, scene.numberOfAgents());

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(ContinuousScene, SortObservations) {

  try {

    auto snap1 = boost::make_shared<SceneSnapshot>(Time(1.0));
    auto snap2 = boost::make_shared<SceneSnapshot>(Time(2.0));
    auto snap3 = boost::make_shared<SceneSnapshot>(Time(3.0));

    ContinuousScene scene;
    scene.addObservation(snap2);
    scene.addObservation(snap3);
    scene.addObservation(snap1);

    scene.sortObservationsByAscendingTime();

    Time tPrev(0.0);
    for (auto snapshot : scene.getObservations()) {
      EXPECT_GT(snapshot->stamp(), tPrev);
      tPrev = snapshot->stamp();
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(ContinuousScene, TrimObservations) {

  try {
    const Id id1 = 1;
    const Id id2 = 2;
    State s(0);
    s.pose() = Pose2d(1.0, 2.0, 3.0);
    StateWithUncertainty swu(s, Eigen::Matrix3d::Identity());

    auto snap1 = boost::make_shared<SceneSnapshot>(Time(1.0));
    snap1->addObject(id1, swu);
    snap1->addObject(id2, swu);
    auto snap2 = boost::make_shared<SceneSnapshot>(Time(2.0));
    snap2->addObject(id1, swu);
    auto snap3 = boost::make_shared<SceneSnapshot>(Time(3.0));
    snap3->addObject(id1, swu);

    ContinuousScene scene;
    scene.addObservation(snap2);
    scene.addObservation(snap3);
    scene.addObservation(snap1);

    scene.keepLatestObservationPerAgent();

    EXPECT_EQ(1, scene.getObservations()[0]->objectContainer().size());
    EXPECT_EQ(0, scene.getObservations()[1]->objectContainer().size());
    EXPECT_EQ(1, scene.getObservations()[2]->objectContainer().size());

    scene.trimObservations(Time(2.5));
    EXPECT_EQ(1, scene.getObservations().size());
    EXPECT_EQ(Time(3.0), scene.getObservations().back()->stamp());

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
