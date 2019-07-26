/*
 * TrajectoryImpl.hpp
 *
 *  Created on: 19.10.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_TRAJECTORYIMPL_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_TRAJECTORYIMPL_HPP_

#include <planner_interfaces/Exceptions.hpp>
#include <planner_interfaces/MathSupport.hpp>

namespace prob_planner {

namespace details {

struct ConstantVelocityExtrapolationFunctor {
  bool operator()(const Trajectory& trajectory, const planning2d::Time& stamp, planning2d::Position2d& extrapolated) {
    const planning2d::Time tEval = stamp > trajectory.getFinalTime() ? trajectory.getFinalTime() : trajectory.getStartTime();
    extrapolated = trajectory.getPosition2d(tEval) + planning2d::Position2d((stamp - tEval).toSec()*trajectory.getVelocityXY(tEval));
    return true;
  }
};

} /* namespace details */

template <typename ExtrapolationFunctor>
void Trajectory::adjustTime(const planning2d::Time& start, const planning2d::Time& end,
                            ExtrapolationFunctor extrapolateFcn, const double lambda /*=1e-3*/,
                            const planning2d::Duration& samplingResolution /*= 0.1*/) {

  SM_ASSERT_GT(planning2d::FunctionInputException, end, start, "");
  const auto previousStartTime = this->getStartTime();
  const auto previousEndTime = this->getFinalTime();
  const planning2d::Duration segmentDuration = (previousEndTime - previousStartTime)/_spline.getNumValidTimeSegments();
  const int numSegments = std::max(static_cast<int>((end - start)/segmentDuration), 1);
  int numDiscretePoints = std::max(static_cast<std::size_t>((end - start).toSec()/samplingResolution.toSec()), 10UL);
  std::vector<planning2d::Time> stamps;
  stamps.reserve(numDiscretePoints);
  planning2d::math::linspace(start, end, numDiscretePoints, stamps, true);
  planning2d::PositionTrajectory discreteTrajectory; // discrete points to refit the spline
  auto stamp = stamps.begin();
  planning2d::Position2dStamped extrapolated;

  // extrapolate positions before old trajectory start
  for ( ; stamp != stamps.end() && *stamp<previousStartTime; ++stamp) {
    if (extrapolateFcn(*this, *stamp, extrapolated)) {
      extrapolated.stamp() = *stamp;
      discreteTrajectory.emplace_back(extrapolated);
    }
  }

  // interpolate
  for ( ; stamp != stamps.end() && *stamp<=previousEndTime; ++stamp)
    discreteTrajectory.emplace_back(this->getPosition2d(*stamp), *stamp);

  // extrapolate remaining positions
  for ( ; stamp != stamps.end() && *stamp<=end; ++stamp) {
    if (extrapolateFcn(*this, *stamp, extrapolated)) {
      extrapolated.stamp() = *stamp;
      discreteTrajectory.emplace_back(extrapolated);
    }
  }

  this->_spline = Spline();
  this->initFromDiscretizedTrajectory(discreteTrajectory, numSegments, lambda, &start, &end);
}

void Trajectory::adjustTime(const planning2d::Time& start, const planning2d::Time& end,
                            const double lambda /*=1e-3*/, const planning2d::Duration& samplingResolution /*= 0.1*/) {
  details::ConstantVelocityExtrapolationFunctor functor;
  this->adjustTime(start, end, functor, lambda, samplingResolution);
}

template <typename ExtrapolationFunctor>
void Trajectory::shiftTime(const planning2d::Duration& shift, ExtrapolationFunctor extrapolateFcn,
                           const double lambda /*=1e-3*/, const planning2d::Duration& samplingResolution /*= 0.1*/) {
  this->adjustTime(this->getStartTime() + shift, this->getFinalTime() + shift, extrapolateFcn, lambda, samplingResolution);
}

void Trajectory::shiftTime(const planning2d::Duration& shift, const double lambda /*=1e-3*/,
                           const planning2d::Duration& samplingResolution /*= 0.1*/) {
  details::ConstantVelocityExtrapolationFunctor functor;
  this->shiftTime(shift, functor, lambda, samplingResolution);
}

template <int IMaximalDerivativeOrder>
Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSpline(const planning2d::Time& timestamp) const {
  return convertToGME(_spline.getExpressionFactoryAt<IMaximalDerivativeOrder>(timestamp.toSec()).getValueExpression(IMaximalDerivativeOrder));
}

planning2d::Time Trajectory::getStartTime() const {
  SM_ASSERT_TRUE_DBG( planning2d::RuntimeException, isInitialized(), "");
  return planning2d::Time(_spline.getMinTime());
}
planning2d::Time Trajectory::getFinalTime() const {
  SM_ASSERT_TRUE_DBG( planning2d::RuntimeException, isInitialized(), "");
  return planning2d::Time(_spline.getMaxTime());
}
bool Trajectory::contains(const planning2d::Time& stamp) const {
  return stamp >= getStartTime() && stamp <= getFinalTime();
}

template <int IMaximalDerivativeOrder>
Trajectory::Spline::Evaluator<IMaximalDerivativeOrder> Trajectory::getEvaluatorAt(const planning2d::Time& timestamp) const {
  return _spline.getEvaluatorAt<IMaximalDerivativeOrder>(timestamp);
}


template <int IMaximalDerivativeOrder>
Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSplinePos2d(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const {
  return convertToGME(factory.getValueExpression(0));
}
template <int IMaximalDerivativeOrder>
Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSplineVelXY(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const {
  static_assert(IMaximalDerivativeOrder >= 1, "");
  return convertToGME(factory.getValueExpression(1));
}
template <int IMaximalDerivativeOrder>
Trajectory::TrajectoryValueExpression Trajectory::getExpressionFromSplineAccXY(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const {
  static_assert(IMaximalDerivativeOrder >= 2, "");
  return convertToGME(factory.getValueExpression(2));
}
template <int IMaximalDerivativeOrder>
aslam::backend::GenericMatrixExpression<1, 1> Trajectory::getExpressionFromSplineRotationRateSquared(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const {
  Trajectory::TrajectoryValueExpression vel = getExpressionFromSplineVelXY(factory);
  Trajectory::TrajectoryValueExpression acc = getExpressionFromSplineAccXY(factory);
  auto velSquaredNorm = vel.squaredNorm();
  auto nom = (vel.toScalarExpression<0,0>()*acc.toScalarExpression<1,0>()) - (vel.toScalarExpression<1,0>()*acc.toScalarExpression<0,0>());
  auto rate = nom/velSquaredNorm;
  return rate*rate;
}



template <int IMaximalDerivativeOrder>
Eigen::Vector2d Trajectory::getPosition(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const {
  return evaluator.eval();
}
template <int IMaximalDerivativeOrder>
Eigen::Vector3d Trajectory::getPose(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const {
  Eigen::Vector3d pose;
  auto velocity = getVelocityXY(evaluator);
  SM_ASSERT_FALSE(planning2d::RuntimeException, velocity.norm() < 1e-12, "Robot too slow. Cannot compute correct heading.");
  pose.segment(0,2) = getPosition(evaluator);
  pose(2) = atan2(velocity(1), velocity(0));
  return pose;
}
template <int IMaximalDerivativeOrder>
Eigen::Vector2d Trajectory::getVelocityXY(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const {
  static_assert(IMaximalDerivativeOrder >= 1, "");
  return evaluator.evalD(1);
}
template <int IMaximalDerivativeOrder>
Eigen::Vector2d Trajectory::getAccelerationXY(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const {
  static_assert(IMaximalDerivativeOrder >= 2, "");
  return evaluator.evalD(2);
}
template <int IMaximalDerivativeOrder>
Eigen::Vector2d Trajectory::getVelocityTransRot(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const {

  static_assert(IMaximalDerivativeOrder >= 2, "");

  // omega = ||v|| / R
  // R = 1 / kappa = (x_dot^2 + y_dot^2)^3/2 / (x_dot*y_dotdot - y_dot*x_dotdot) =
  // ||v||^3 / (x_dot*y_dotdot - y_dot*x_dotdot)
  // => omega = (x_dot*y_dotdot - y_dot*x_dotdot)/||v||^2

  Eigen::Vector2d vel = getVelocityXY(evaluator);
  Eigen::Vector2d acc = getAccelerationXY(evaluator);

  // Velocity and acceleration have to be larger than 0 to allow for correct discretization
  // TODO fit this with velocity spline or additional spline for heading

  const double velSquaredNorm = vel.squaredNorm();
  const double vn = sqrt(velSquaredNorm);
  double omega = ( vel(0)*acc(1) - vel(1)*acc(0) )/velSquaredNorm;
  if(!std::isfinite(omega)) {
    SM_WARN_STREAM("Trajectory::getVelocityTransRot(): Correct computation of rotation rate impossible, setting to zero. Velocity norm: " << vn);
    omega = 0.0;
  }
  Eigen::Vector2d transRotVel(vn, omega);

  return transRotVel;
}

template <int IMaximalDerivativeOrder>
inline Eigen::Matrix<double,5,1> Trajectory::getPoseAndVelocityXY(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const {
  Eigen::Matrix<double,5,1> poseAndVelXY;
  poseAndVelXY.segment(0,3) = getPose(evaluator);
  poseAndVelXY.segment(3,2) = getVelocityXY(evaluator);
  return poseAndVelXY;
}

template<class Archive>
void Trajectory::save(Archive & ar, const unsigned int /*version*/) const
{
    std::size_t numSegments = _spline.getNumValidTimeSegments();
    planning2d::Time start = this->getStartTime();
    planning2d::Time end = this->getFinalTime();

    ar << numSegments;
    ar << start;
    ar << end;

    Eigen::MatrixXd parameters;
    int blockIndex, columnBase;
    bool isActive, isMarginalized;
    double scaling;

    prob_planner::Trajectory::Spline& spline = const_cast<prob_planner::Trajectory::Spline&>(this->getSpline());

    for (const auto dv : spline.getDesignVariables())
    {
      blockIndex = dv->blockIndex();
      columnBase = dv->columnBase();
      isActive = dv->isActive();
      isMarginalized = dv->isMarginalized();
      scaling = dv->scaling();
      dv->getParameters(parameters);

      ar << blockIndex;
      ar << columnBase;
      ar << isActive;
      ar << isMarginalized;
      ar << scaling;
      ar << parameters;
    }
}

template<class Archive>
void Trajectory::load(Archive & ar, const unsigned int /*version*/)
{
  std::size_t numSegments;
  planning2d::Time start, end;
  Eigen::MatrixXd parameters;
  int blockIndex, columnBase;
  bool isActive, isMarginalized;
  double scaling;

  ar >> numSegments;
  ar >> start;
  ar >> end;

  this->initZeroSpline(start, end, numSegments);

  prob_planner::Trajectory::Spline& spline = const_cast<prob_planner::Trajectory::Spline&>(this->getSpline());
  for (auto& dv : spline.getDesignVariables()) {

    ar >> blockIndex;
    ar >> columnBase;
    ar >> isActive;
    ar >> isMarginalized;
    ar >> scaling;
    ar >> parameters;

    dv->setBlockIndex(blockIndex);
    dv->setColumnBase(columnBase);
    dv->setActive(isActive);
    dv->setMarginalized(isMarginalized);
    dv->setScaling(scaling);
    dv->setParameters(parameters);
  }
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_STATE_REPRESENTATION_IMPL_TRAJECTORYIMPL_HPP_ */
