#include <gtest/gtest.h>

#include "../include/probabilistic_planner/models/GaussianMeasurementModel.hpp"
#include "../include/probabilistic_planner/state_representation/ContinuousScene.hpp"
#include "../include/probabilistic_planner/state_representation/SceneSnapshot.hpp"
#include "../include/probabilistic_planner/optimization/OptimizationProblem.hpp"

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>

#include <common_agents/DifferentialDriveAgent.hpp>

#include <sm/logging.hpp>

using namespace std;
using namespace planning2d;
using namespace prob_planner;

TEST(probabilistic_planner_TESTSUITE, GaussianMeasurementModel) {

  try {

//    ContinuousScene scene;
//    Eigen::Matrix2d cov = 0.1*Eigen::Matrix2d::Identity();
//    GaussianMeasurementModel meas(scene, cov);
//    meas.negativeLogDensity(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0));

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(probabilistic_planner_TESTSUITE, Scene) {

  try {

    ContinuousScene scene;

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(probabilistic_planner_TESTSUITE, Optimization) {

  try {

    ContinuousScene scene;

    // populate scene
    DifferentialDriveAgent::Ptr agent = DifferentialDriveAgent::Ptr(new DifferentialDriveAgent());
    agent->setId(0);
    Trajectory trajectory;
    OptAgent a(agent, trajectory);
    scene.addAgent(a);

    aslam::backend::Optimizer optimizer;
    aslam::backend::OptimizationProblem problem;

    // add all spline coefficients of agents as design variables
    for (auto &agent : scene.getAgentContainer()) {
      auto dvar = agent.second.getTrajectory().getDesignVariables();
      SM_INFO_STREAM("Adding " << dvar.size() << " design variables.");
      for (auto &v: dvar)
        problem.addDesignVariable(v, false);
    }

    // Create a model
    Eigen::Matrix2d cov = 0.1*Eigen::Matrix2d::Identity();
    GaussianMeasurementModel meas(scene, cov);

    // add all error terms from the probabilistic models as error terms
    auto errs = meas.getErrorTerms();
    SM_INFO_STREAM("Adding " << errs.size() << " error terms for gaussian measurement model.");
    for (auto &err : errs)
      problem.addErrorTerm(err);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(probabilistic_planner_TESTSUITE, SceneSnapshot) {

  try {

    Time stamp(0.0);
    SceneSnapshot meas(Time(0.0));
    EXPECT_EQ(stamp, meas.stamp());

    State state(0);
    state.pose() = Pose2d(0.0, 0.0, 0.0);
    meas.addObject(0, state);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
