#include <gtest/gtest.h>
#include <sm/random.hpp>
#include <probabilistic_planner/state_representation/Trajectory.hpp>

using namespace std;
using namespace planning2d;
using namespace prob_planner;


TEST(Trajectory, shiftTime) {

  try {

    Trajectory trajectory;
    PositionTrajectory points;
    for (size_t i=0; i<10; i++)
      points.emplace_back(Position2d(sm::random::randLU(0., 0.1) + static_cast<double>(i), sm::random::randLU(0., 0.3)), Time(sm::random::randLU(0.0, 0.1) + static_cast<double>(i)));
    trajectory.initFromDiscretizedTrajectory(points, (points.back().stamp() - points.front().stamp()).toSec() * 10, 1e-12);

    { // shift forward
      const Duration shift(sm::random::randLU(0.5, 1.0));
      Trajectory original = trajectory;
      trajectory.shiftTime(shift);
      EXPECT_EQ(original.getStartTime() + shift, trajectory.getStartTime());
      EXPECT_EQ(original.getFinalTime() + shift, trajectory.getFinalTime());
      EXPECT_EQ(original.getSpline().getNumControlVertices(), trajectory.getSpline().getNumControlVertices());
      for (Time stamp = trajectory.getStartTime(); stamp <= original.getFinalTime(); stamp += Duration(0.1))
        EXPECT_LT( (original.getPosition2d(stamp) - trajectory.getPosition2d(stamp)).norm(), 1e-2);
      EXPECT_LT( (original.getPosition2d(original.getFinalTime()) + Position2d(original.getVelocityXY(original.getFinalTime())*shift.toSec()) -
          trajectory.getPosition2d(trajectory.getFinalTime())).norm(), 1e-4);
    }

    { // adjust start time backwards
      const Duration shift(sm::random::randLU(1.5, 2.0));
      Trajectory original = trajectory;
      const Time oldEnd = trajectory.getFinalTime();
      const Time newStart = trajectory.getStartTime() - shift;

      trajectory.adjustTime(newStart, oldEnd, 1e-12);
      EXPECT_EQ(newStart, trajectory.getStartTime());
      EXPECT_EQ(oldEnd, trajectory.getFinalTime());
      EXPECT_LT( ((original.getPosition2d(original.getStartTime()) - Position2d(original.getVelocityXY(original.getStartTime())*shift.toSec()) -
          trajectory.getPosition2d(trajectory.getStartTime()))).norm(), 1e-3);
      for (Time stamp = original.getStartTime(); stamp <= original.getFinalTime(); stamp += Duration(0.1))
        EXPECT_LT( (original.getPosition2d(stamp) - trajectory.getPosition2d(stamp)).norm(), 1e-4);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Trajectory, Trajectory) {

  try {

    // Initialize zero spline
    {
      Trajectory trajectory;
      trajectory.initZeroSpline(Time(0.0), Time(1.0), 10);
      for (Time t = 0.0; t < 1.0; t += 0.1)
        EXPECT_EQ(trajectory.getPosition2d(t), Position2d(0.0, 0.0));
    }

    // Initialize trajectory with random numbers
    Trajectory trajectory;
    double x1 = (rand() % 1000) / 100;
    double y1 = (rand() % 1000) / 100;
    double x2 = (rand() % 1000) / 100;
    double y2 = (rand() % 1000) / 100;
    double heading = atan2( y2-y1, x2-x1 );
    Pose2d startPose(x1, y1, heading);
    Time startTime(2.0);
    StateStamped startState(planning2d::State(Eigen::Vector2d(0,0), startPose), startTime);
    Pose2d endPose(x2, y2, heading);
    Time finalTime(10.0);
    StateStamped endState(planning2d::State(Eigen::Vector2d(0,0), endPose), finalTime);

    int numberOfSegments = rand() % 10;

    // generate spline between two timestamped points
    trajectory.initStraightSpline(startState, endState, numberOfSegments, 1.0);
//    auto designVariables = trajectory.getDesignVariables();
//    auto jacobian = trajectory.getJacobian((startTime.toSec() + finalTime.toSec()) / 2);

    EXPECT_TRUE(trajectory.contains(startState.stamp()));
    EXPECT_FALSE(trajectory.contains(startState.stamp() - planning2d::Duration(static_cast<planning2d::time::T>(1))));
    EXPECT_TRUE(trajectory.contains(endState.stamp()));
    EXPECT_FALSE(trajectory.contains(endState.stamp() + planning2d::Duration(static_cast<planning2d::time::T>(1))));

    SM_DEBUG_STREAM("Straight p2p spline:");
    SM_DEBUG_STREAM("Starting point = (" << startState.pose().position().x() <<
                    ", " << startState.pose().position().y() << ")");
    SM_DEBUG_STREAM("Goal point = (" << endState.pose().position().x() <<
                    ", " << endState.pose().position().y() << ")");

    for (double t=startTime.toSec(); t<=finalTime.toSec(); t+=1.0) {
      Time time(t);
      SM_DEBUG_STREAM("Point " << t << ": time = " << time << " (" << trajectory.getPosition(time).x() <<
                      ", " << trajectory.getPosition(time).y() << ")");
    }

    // Test copy constructor
    {
      Trajectory tc(trajectory);
      auto& dvs = trajectory.getDesignVariables();
      auto& dvsc = tc.getDesignVariables();
      for (std::size_t i=0; i<dvsc.size(); i++) {
        Eigen::MatrixXd pc;
        dvsc[i]->getParameters(pc);
        Eigen::MatrixXd pcn = pc*2.0;
        dvsc[i]->setParameters(pcn);
        Eigen::MatrixXd p;
        dvs[i]->getParameters(p);
        ASSERT_FALSE(p.isApprox(pcn)) <<
            "p: " << p.transpose() << endl << "pcn: " << pcn.transpose() << endl <<
            "pc should be " << pc.transpose() << endl;
      }
    }


    // Checks
    // Straight spline
    EXPECT_NE(startState.pose().position().x(), endState.pose().position().x());
    EXPECT_NE(startState.pose().position().y(), endState.pose().position().y());
    EXPECT_NEAR(trajectory.getStateStamped(startTime).pose().position().x(), startState.pose().position().x(), 1e-12);
    EXPECT_NEAR(trajectory.getStateStamped(startTime).pose().position().y(), startState.pose().position().y(), 1e-12);
    EXPECT_NEAR(trajectory.getStateStamped(startTime).pose().yaw(), startState.pose().yaw(), 1e-12);
    EXPECT_EQ(trajectory.getStartTime(), startTime);
    EXPECT_EQ(trajectory.getFinalTime(), finalTime);
    EXPECT_EQ(trajectory.getStateStamped(startTime).stamp(), startTime);
    EXPECT_EQ(trajectory.getStateStamped(finalTime).stamp(), finalTime);

    // Discretize trajectory
    Trajectory trajectory2;
    Trajectory trajectoryTrimmed;
    const Duration discretizationDt(0.5);
    auto discreteTrajectory = trajectory.discretizeTrajectory(1.0);
    auto discreteTrajectory2 = trajectory.discretizeTrajectoryForPeriod(discretizationDt, Time(0.0), Time(12.0));

    // Initialize spline from discrete trajectory
    trajectory2.initFromDiscretizedTrajectory(discreteTrajectory, 10, 1.0);

    // Test whether discretization function works correctly and only discretizes for values within the defined range
    // (no extrapolation)
    trajectoryTrimmed.initFromDiscretizedTrajectory(discreteTrajectory2, 10, 1.0);
    EXPECT_EQ(startTime.toSec(), trajectoryTrimmed.getStartTime().toSec());
    EXPECT_EQ(finalTime.toSec(), trajectoryTrimmed.getFinalTime().toSec());

    // Check for each point
    for (const auto& ss : discreteTrajectory){
      EXPECT_EQ(startTime.toSec(), trajectory2.getStartTime().toSec());
      EXPECT_EQ(finalTime.toSec(), trajectory2.getFinalTime().toSec());
      EXPECT_NEAR(ss.pose().position().x(), trajectory2.getStateStamped(ss.stamp()).pose().position().x(), 1e-12);
      EXPECT_NEAR(ss.pose().position().y(), trajectory2.getStateStamped(ss.stamp()).pose().position().y(), 1e-12);
      EXPECT_NEAR(ss.pose().position().x(), trajectory2.getPose2d(ss.stamp()).position().x(), 1e-12);
      EXPECT_NEAR(ss.pose().position().y(), trajectory2.getPose2d(ss.stamp()).position().y(), 1e-12);
      EXPECT_NEAR(ss.pose().position().x(), trajectory2.getPosition(ss.stamp()).x(), 1e-12);
      EXPECT_NEAR(ss.pose().position().y(), trajectory2.getPosition(ss.stamp()).y(), 1e-12);
      EXPECT_EQ(ss.stamp(), trajectory2.getStateStamped(ss.stamp()).stamp());
    }

    // Theoretical check (going from (0,0) to (1,1) within 1 second)
    Trajectory trajectory3;
    StateStamped startState2(State(Eigen::Vector2d(0.,0.), Pose2d(0., 0., 0.)), Time(0.0));
    StateStamped endState2(State(Eigen::Vector2d(1.,1.), Pose2d(1., 1., 0.)), Time(1.0));
    trajectory3.initStraightSpline(startState2, endState2, 4, 1.0);
    EXPECT_NEAR(trajectory3.getVelocityXY(Time(0.0))(0), 1.0, 1e-12);
    EXPECT_NEAR(trajectory3.getVelocityXY(Time(0.0))(1), 1.0, 1e-12);
    EXPECT_NEAR(trajectory3.getVelocityTransRot(Time(0.0))(0), sqrt(2.0), 1e-12);
    EXPECT_NEAR(trajectory3.getVelocityTransRot(Time(0.0))(1), 0.0, 1e-12);

    // Check expressions
    auto expr = trajectory3.getExpressionFromSplinePos2d(Time(0.0));
    auto posExpr = expr.evaluate();
    auto posEval = trajectory3.getPosition(Time(0.0));
    EXPECT_TRUE(posExpr.isApprox(posEval)) <<
        "Position from expression: " << posExpr.transpose() <<
        ", position from evaluator: " << posEval;

    expr = trajectory3.getExpressionFromSplinePos2d(Time(1.0));
    posExpr = expr.evaluate();
    posEval = trajectory3.getPosition(Time(1.0));
    EXPECT_TRUE(posExpr.isApprox(posEval)) <<
        "Position from expression: " << posExpr.transpose() <<
        ", position from evaluator: " << posEval;

    expr = trajectory3.getExpressionFromSplineVelXY(Time(0.5));
    auto velExpr = expr.evaluate();
    auto velEval = trajectory3.getVelocityXY(Time(0.5));
    EXPECT_TRUE(velExpr.isApprox(velEval)) <<
        "Velocity from expression: " << velExpr.transpose() <<
        ", velocity from evaluator: " << velEval.transpose();

    auto exprR = trajectory3.getExpressionFromSplineRotationRateSquared(Time(0.5));
    auto rotRateExpr = exprR.evaluate();
    auto rotRateEval = trajectory3.getVelocityTransRot(Time(0.5))[1];
    rotRateEval *= rotRateEval;
    EXPECT_NEAR(rotRateExpr(0,0), rotRateEval, 1e-3) <<
        "Rotational velocity squared from expression: " << rotRateExpr <<
        ", rotational velocity squared from evaluator: " << rotRateEval;

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
