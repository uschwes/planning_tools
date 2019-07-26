/*
 * Trajectory.cpp
 *
 *  Created on: May 28, 2015
 *      Author: pfmark
 */

#include <math.h>

#include <iostream>

#include <sm/logging.hpp>

#include <bsplines/DiffManifoldBSpline.hpp>
#include <bsplines/BSplineFitter.hpp>

#include <probabilistic_planner/state_representation/Trajectory.hpp>

#include <aslam/backend/VectorExpressionToGenericMatrixTraits.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

using namespace planning2d;

namespace prob_planner {

/**
 * Extract vector of 2d positions from discrete Trajectory
 * @param trajectory discrete time stamped positions of a trajectory
 * @return extracted time vector
 */
template <typename TrajectoryT>
void getTimeVector(const TrajectoryT& trajectory, std::vector<Trajectory::SplineTime>& times) {
  times.clear();
  times.reserve(trajectory.size());
  for (const auto& s : trajectory)
    times.push_back(s.stamp().nanosec);
}

/**
 * Extract vector of 2d positions from discrete Trajectory
 * @param trajectory discrete time stamped positions of a trajectory
 * @return extracted vector with 2d positions
 */
template <typename TrajectoryT>
void getPositionVector(const TrajectoryT& trajectory, std::vector<Eigen::Vector2d>& points) {
  points.clear();
  points.reserve(trajectory.size());
  for (const auto& s : trajectory)
    points.push_back(((const Position2d&)s).asVector());
}


void Trajectory::initFromDiscretizedTrajectory(const StateTrajectory& trajectory,
                                               const int numberOfSegments,
                                               const double lambda,
                                               const planning2d::Time* minTime /*= nullptr*/,
                                               const planning2d::Time* maxTime /*= nullptr*/) {
  PositionTrajectory trajectory2;
  trajectory2.reserve(trajectory.size());
  for (const auto& s : trajectory)
    trajectory2.push_back((Position2dStamped)s); // TODO: Remove aweful conversion
  this->initFromDiscretizedTrajectory(trajectory2, numberOfSegments, lambda, minTime, maxTime);
}

void Trajectory::initFromDiscretizedTrajectory(const PositionTrajectory& trajectory,
                                               const int numberOfSegments,
                                               const double lambda,
                                               const planning2d::Time* minTime /*= nullptr*/,
                                               const planning2d::Time* maxTime /*= nullptr*/) {
  SM_ASSERT_GT(RuntimeException, trajectory.size(), 0, "");
  std::vector<Trajectory::SplineTime> times;
  std::vector<Eigen::Vector2d> points; // TODO: what about aligned allocation?
  getTimeVector(trajectory, times);
  getPositionVector(trajectory, points);

  const auto& min = minTime != nullptr ? minTime->nanosec : trajectory.front().stamp().nanosec;
  const auto& max = maxTime != nullptr ? maxTime->nanosec : trajectory.back().stamp().nanosec;

  if (this->isInitialized()) // Reconstruct if already initialized, since initialization twice is not supported
    _spline = Spline();

  bsplines::BSplineFitter<Spline> fitter;
  fitter.initUniformSpline(_spline, min, max, times, points, numberOfSegments, lambda);   // lambda: regularization weight
}

std::size_t Trajectory::numActiveDesignVariables() {
  std::size_t numActiveVariables = 0;
  for (const auto& dv : getDesignVariables()) {
    if (dv->isActive())
      numActiveVariables++;
  }
  return numActiveVariables;
}

void Trajectory::activateAllDesignVariables(bool activate) {
  for (auto& dv : getDesignVariables())
    dv->setActive(activate);
}

void Trajectory::addDesignVariables(aslam::backend::OptimizationProblem& problem, bool autoActivate) {
  if (autoActivate) this->activateAllDesignVariables(true);
  for (auto& dv : getDesignVariables())
    problem.addDesignVariable(dv, false);
}

void Trajectory::deactiveDesignVariablesAtTime(const planning2d::Time& timestamp) {
  for (auto& dv : _spline.getDesignVariables(timestamp.nanosec)) {
    dv->setActive(false);
  }
}

void Trajectory::initZeroSpline(const Time& start, const Time& end, const std::size_t numSegments) {
  _spline.initConstantUniformSpline(start.nanosec, end.nanosec, numSegments, Eigen::Vector2d(0.0, 0.0));
}

void Trajectory::initStraightSpline(const StateStamped& startingPoint,
                                    const StateStamped& goalPoint,
                                    const int numberOfSegments,
                                    const double lambda) {
  initStraightSpline(planning2d::Position2dStamped(startingPoint.position(), startingPoint.stamp()),
                     planning2d::Position2dStamped(goalPoint.position(), goalPoint.stamp()),
                     numberOfSegments, lambda);
}

void Trajectory::initStraightSpline(const planning2d::Position2dStamped& start,
                                    const planning2d::Position2dStamped& goal,
                                    const int numberOfSegments,
                                    const double lambda) {
  initFromDiscretizedTrajectory( PositionTrajectory( {start, goal} ), numberOfSegments, lambda);
}

Eigen::MatrixXd Trajectory::getJacobian(const Time& timestamp) const {
  Spline::full_jacobian_t jacobian;
  _spline.getEvaluatorAt<1>(timestamp.nanosec).evalJacobian(1, jacobian);
  return static_cast<Eigen::MatrixXd>(jacobian);
}

Eigen::MatrixXd Trajectory::getSplineControlVertices() const {
  Eigen::MatrixXd vertices(_spline.getNumControlVertices(), (int)_spline.getDimension());
  typename Spline::SegmentConstIterator it = _spline.getAbsoluteBegin();
  for(int c = 0, end = vertices.rows(); c != end; c++, it++) {
    vertices.row(c) = it->getControlVertex();
  }
  return vertices;
}

Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSplinePos2d(const Time& timestamp) const {
  return getExpressionFromSplinePos2d(_spline.getExpressionFactoryAt<0>(timestamp.nanosec));
}

Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSplineVelXY(const Time& timestamp) const {
  return getExpressionFromSplineVelXY(_spline.getExpressionFactoryAt<1>(timestamp.nanosec));
}

Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSplineAccXY(const Time& timestamp) const {
  return getExpressionFromSplineAccXY(_spline.getExpressionFactoryAt<2>(timestamp.nanosec));
}

aslam::backend::GenericMatrixExpression<1, 1> Trajectory::getExpressionFromSplineRotationRateSquared(const Time& timestamp) const {
  return getExpressionFromSplineRotationRateSquared(_spline.getExpressionFactoryAt<2>(timestamp.nanosec));
}

StateStamped Trajectory::getStateStamped(const Time& timestamp) const {
  auto evaluator = _spline.getEvaluatorAt<2>(timestamp.nanosec);
  return StateStamped(State(getVelocityTransRot(evaluator), Pose2d(getPose(evaluator))), timestamp);
}

Pose2d Trajectory::getPose2d(const Time& timestamp) const {
  auto evaluator = _spline.getEvaluatorAt<1>(timestamp.nanosec);
  auto velocity = getVelocityXY(evaluator);
  return Pose2d(getPosition(evaluator), atan2(velocity(1), velocity(0)));
}

Position2d Trajectory::getPosition2d(const Time& timestamp) const {
  return Position2d(getPosition(timestamp));
}

Eigen::Vector2d Trajectory::getPosition(const Time& timestamp) const {
  return getPosition(_spline.getEvaluatorAt<0>(timestamp.nanosec));
}

Eigen::Vector3d Trajectory::getPose(const Time& timestamp) const {
  return getPose(_spline.getEvaluatorAt<1>(timestamp.nanosec));
}

Eigen::Vector2d Trajectory::getVelocityXY(const Time& timestamp) const {
  return getVelocityXY(_spline.getEvaluatorAt<1>(timestamp.nanosec));
}

Eigen::Vector2d Trajectory::getAccelerationXY(const Time& timestamp) const {
  return getAccelerationXY(_spline.getEvaluatorAt<2>(timestamp.nanosec));
}

Eigen::Vector2d Trajectory::getVelocityTransRot(const Time& timestamp) const {
  return getVelocityTransRot(_spline.getEvaluatorAt<2>(timestamp.nanosec));
}

Eigen::Matrix<double,5,1> Trajectory::getPoseAndVelocityXY(const planning2d::Time& timestamp) const {
  return getPoseAndVelocityXY(_spline.getEvaluatorAt<1>(timestamp.nanosec));
}

double Trajectory::getRotationRateSquared(const Time& timestamp) const {
  const Eigen::Vector2d velTransRot = getVelocityTransRot(timestamp);
  return velTransRot[1]*velTransRot[1];
}

StateTrajectory Trajectory::discretizeTrajectory(const Duration& dt) const {
  return discretizeTrajectoryForPeriod(dt, getStartTime(), getFinalTime());
}

StateTrajectory Trajectory::discretizeTrajectoryForPeriod(const Duration& dt,
                                                          const Time& startTime,
                                                          const Time& endTime) const {

  SM_ASSERT_GT(RuntimeException, dt, Duration((time::T)0), "");
  SM_ASSERT_GE(RuntimeException, endTime, startTime, "");

  StateTrajectory trajectory;
  // time interval has to be limited in order to avoid extrapolation
  const planning2d::Time start(std::max(startTime.nanosec, _spline.getMinTime()));
  const planning2d::Time end(std::min(endTime.nanosec, _spline.getMaxTime()));

  trajectory.reserve(static_cast<size_t>(std::ceil((end - start)/dt)));
  for (planning2d::Time t = start; t <= end; t += dt)
    trajectory.push_back(getStateStamped(t));

  return trajectory;
}

}
