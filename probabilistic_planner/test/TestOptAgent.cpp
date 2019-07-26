#include <gtest/gtest.h>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/TestAgent.hpp>

#include <probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp>
#include <probabilistic_planner/state_representation/OptAgent.hpp>

#include <aslam/backend/OptimizationProblem.hpp>

using namespace std;
using namespace planning2d;
using namespace prob_planner;


TEST(OptAgent, OptAgent) {

  try {

    sm::logging::Level logLevel = sm::logging::getLevel();
    sm::logging::setLevel(sm::logging::Level::Error);


    TestAgent::Ptr agent = TestAgent::Ptr(new TestAgent());
    agent->setId(0);
    Trajectory::Ptr trajectory(new Trajectory());

    double xStart = -1;
    double yStart = -1;
    double xGoal = -2;
    double yGoal = -2;
    double yaw = -180 * M_PI/180;
    double timeDouble = static_cast<double>((std::rand() % 1000) / 10);

    agent->stateStamped().state() = Eigen::Vector2d(1., 2.);
    agent->stateStamped().stamp() = Time(timeDouble);
    agent->stateStamped().pose() = Pose2d(xStart, yStart, yaw);

    trajectory->initStraightSpline( agent->stateStamped(),
                                   StateStamped(State(Eigen::Vector2d(0.,0.), Pose2d(xGoal, yGoal, yaw)), Time(timeDouble + 10.0)),
                                   10,
                                   1.0);

    OptAgent optAgent(agent, trajectory, OptAgentType::UNKNOWN);

    // Test copy method
    {
      OptAgent oac = optAgent.copy();
      EXPECT_EQ(optAgent.getId(), oac.getId());
      EXPECT_EQ(optAgent.getType(), oac.getType());
      EXPECT_NE(optAgent.getAgent(), oac.getAgent()); // compare pointers
      EXPECT_NE(&optAgent.trajectory(), &oac.trajectory()); // compare pointers
      EXPECT_NE(&optAgent.trajectory().getSpline(), &oac.trajectory().getSpline());
      const auto& dvs = optAgent.trajectory().getDesignVariables();
      const auto& dvsc = oac.trajectory().getDesignVariables();
      ASSERT_EQ(dvs.size(), dvsc.size());
      for (size_t i=0; i<dvs.size(); i++)
        EXPECT_NE(&dvs[i], &dvsc[i]);
    }

    // Testing
    EXPECT_EQ(OptAgentType::UNKNOWN, optAgent.getType());
    EXPECT_DOUBLE_EQ(timeDouble, optAgent.getAgent()->stateStamped().stamp().toSec());
    EXPECT_DOUBLE_EQ(xStart, optAgent.getAgent()->stateStamped().pose().position().x());
    EXPECT_DOUBLE_EQ(yStart, optAgent.getAgent()->stateStamped().pose().position().y());
    EXPECT_NEAR(xGoal, optAgent.trajectory().getPosition(optAgent.trajectory().getFinalTime()).x(), 1e-12);
    EXPECT_NEAR(yGoal, optAgent.trajectory().getPosition(optAgent.trajectory().getFinalTime()).y(), 1e-12);


    OptAgent optAgent2(agent, trajectory, OptAgentType::CAR);

    // Testing
    EXPECT_EQ(OptAgentType::CAR, optAgent2.getType());
    EXPECT_DOUBLE_EQ(timeDouble, optAgent2.getAgent()->stateStamped().stamp().toSec());
    EXPECT_DOUBLE_EQ(xStart, optAgent2.getAgent()->stateStamped().pose().position().x());
    EXPECT_DOUBLE_EQ(yStart, optAgent2.getAgent()->stateStamped().pose().position().y());
    EXPECT_NEAR(xGoal, optAgent2.trajectory().getPosition(optAgent2.trajectory().getFinalTime()).x(), 1e-12);
    EXPECT_NEAR(yGoal, optAgent2.trajectory().getPosition(optAgent2.trajectory().getFinalTime()).y(), 1e-12);

    optAgent2.setType(OptAgentType::PEDESTRIAN);
    EXPECT_EQ(OptAgentType::PEDESTRIAN, optAgent2.getType());

    sm::logging::setLevel(logLevel);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(OptAgent, isActive) {

  try {

    Trajectory::Ptr trajectory(new Trajectory());
    trajectory->initZeroSpline(Time(0.0), Time(10.0), 10);
    OptAgent optAgent(TestAgent::Ptr(new TestAgent(0)), trajectory, OptAgentType::UNKNOWN);
    EXPECT_TRUE(optAgent.isActive());
    auto getNumActiveDv = [&](const aslam::backend::OptimizationProblem& problem) {
      std::size_t numActive = 0;
      for (std::size_t i=0; i < problem.numDesignVariables(); ++i)
        numActive += problem.designVariable(i)->isActive() ? 1 : 0;
      return numActive;
    };

    { // Test that design variables are correctly activated
      aslam::backend::OptimizationProblem problem;
      optAgent.addDesignVariables(problem, true);
      EXPECT_GT(getNumActiveDv(problem), 0);
    }

    { // Test that design variables are correctly deactivated
      trajectory->activateAllDesignVariables(true);
      optAgent.setActive(false);
      EXPECT_FALSE(optAgent.isActive());
      aslam::backend::OptimizationProblem problem;
      optAgent.addDesignVariables(problem, true);
      EXPECT_EQ(getNumActiveDv(problem), 0);
    }

  } catch (const exception& e) {
   FAIL() << e.what();
 }
}


TEST(OptAgentTypeRegistry, OptAgentTypeRegistry) {

  try {
    OptAgentTypeRegistry registry;

    // mapping from agent type to string
    EXPECT_EQ(registry.toString(OptAgentType::UNKNOWN), "UNKNOWN");
    EXPECT_EQ(registry.toString(OptAgentType::PEDESTRIAN), "PEDESTRIAN");
    EXPECT_EQ(registry.toString(OptAgentType::CAR), "CAR");
    EXPECT_EQ(registry.toString(OptAgentType::ROBOT), "ROBOT");

    // mapping from string to agent type
    EXPECT_EQ(registry.fromString("UNKNOWN"), OptAgentType::UNKNOWN);
    EXPECT_EQ(registry.fromString("PEDESTRIAN"), OptAgentType::PEDESTRIAN);
    EXPECT_EQ(registry.fromString("CAR"), OptAgentType::CAR);
    EXPECT_EQ(registry.fromString("ROBOT"), OptAgentType::ROBOT);

    std::stringstream os;
    os << OptAgentType::UNKNOWN;
    EXPECT_EQ("UNKNOWN", os.str());

    OptAgentType type;
    std::stringstream is;
    is << "UNKNOWN";
    is >> type;
    EXPECT_EQ(OptAgentType::UNKNOWN, type);


  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
