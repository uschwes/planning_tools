/*
 * testSplineFit.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: pfmark
 */
#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <sm/random.hpp>
#include <sm/logging.hpp>
#include <sm/boost/null_deleter.hpp>

#include <aslam/backend/ExpressionErrorTerm.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer.hpp>
#include <aslam/backend/OptimizerRprop.hpp>

#include <planner_interfaces/Time.hpp>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>
#include "Support.hpp"


using namespace std;
using namespace planning2d;
using namespace prob_planner;
using namespace aslam::backend;

TEST(probabilistic_planner_TESTSUITE, SplineFit) {

  try {

    // Create a discretized random walk trajectory
    const int nPoints = 10;
    const int nSplineSegments = 100;
    const Duration dt(1.0);
    const StateStamped s0(State(Eigen::Vector2d(0.0, 0.0), Pose2d(Position2d(0.0, 0.0), 0.0)), Time((double)0.0));
    StateTrajectory stateTrajectory = createStateTrajectoryRandomWalk(nPoints, dt, s0, 1.0);

    // Fit a spline trough the discretized random walk trajectory
    Trajectory::Ptr trajectory(new Trajectory());
    trajectory->initFromDiscretizedTrajectory(stateTrajectory, nSplineSegments, 0.0001);

    // Check that the fitted spline is close to the discrete positions
    for (size_t i=0; i<stateTrajectory.size(); ++i) {
      const Time& stamp = stateTrajectory[i].stamp();
      EXPECT_EQ(trajectory->getStateStamped(stamp).stamp(), stamp);
      EXPECT_LT( (trajectory->getStateStamped(stamp).pose().position() - stateTrajectory[i].pose().position()).norm(), 0.1);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(probabilistic_planner_TESTSUITE, SplineOptimize) {

  try {

    // Create a discretized random walk trajectory
    const int nPoints = 10;
    const int nSplineSegments = 100;
    const Duration dt(1.0);
    const StateStamped s0(State(Eigen::Vector2d(0.0, 0.0), Pose2d(Position2d(0.0, 0.0), 0.0)), Time((double)0.0));
    StateTrajectory stateTrajectory = createStateTrajectoryRandomWalk(nPoints, dt, s0, 1.0);

    // Fit a spline trough the discretized random walk trajectory
    Trajectory::Ptr trajectorySplineFit(new Trajectory);
    trajectorySplineFit->initFromDiscretizedTrajectory(stateTrajectory, nSplineSegments, 0.1);

    // Initialize a straight spline from start to goal position
    Trajectory::Ptr trajectoryOptimized(new Trajectory());
    trajectoryOptimized->initStraightSpline(stateTrajectory.front(), stateTrajectory.back(), nSplineSegments, 1.0);

    // Set up optimization Problem
    boost::shared_ptr<aslam::backend::OptimizationProblem> problem(new aslam::backend::OptimizationProblem);

    // Add spline design variables to optimization problem
    trajectoryOptimized->addDesignVariables(*problem, true);

    const Eigen::Matrix2d invCov = 100.0*Eigen::Matrix2d::Identity();
    for (size_t i=0; i<nPoints; ++i) {
      const Time& stamp = stateTrajectory[i].stamp();
      const Eigen::Vector2d splineFitPos = trajectorySplineFit->getPosition(stamp);
      auto deviation = toErrorTerm(trajectoryOptimized->getExpressionFromSplinePos2d(stamp) -
                                   Trajectory::TrajectoryValueExpression(splineFitPos),    // deviation from measured point
                                   invCov);
      problem->addErrorTerm(deviation);
    }

    // Set options and optimize
    OptimizerOptionsRprop options;
    options.maxIterations = 500;
    options.numThreadsJacobian = 1;
    OptimizerRprop optimizer(options);
    optimizer.setProblem(problem);

    EXPECT_NO_THROW(optimizer.checkProblemSetup());

    optimizer.optimize();

    EXPECT_LT(optimizer.getStatus().gradientNorm, 1e-3);

    // Check that the optimized spline is close to the fitted spline
    for (size_t i=0; i<nPoints; ++i) {
      const Time& stamp = stateTrajectory[i].stamp();
      EXPECT_NEAR(trajectoryOptimized->getPosition(stamp).x(), trajectorySplineFit->getPosition(stamp).x(), 0.2);
      EXPECT_NEAR(trajectoryOptimized->getPosition(stamp).y(), trajectorySplineFit->getPosition(stamp).y(), 0.2);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
