/*
 * Trajectory.hpp
 *
 *  Created on: Apr 10, 2015
 *      Author: pfmark
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <Eigen/Core>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/State.hpp>
#include <planner_interfaces/Trajectory.hpp>

#include <aslam/splines/OPTBSpline.hpp>
#include <aslam/splines/OPTEuclideanBSpline.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/GenericMatrixExpression.hpp>

#include <bsplines/NsecTimePolicy.hpp>

namespace aslam {
  namespace backend {
    class OptimizationProblem;
  }
}

namespace prob_planner {

class ContinuousScene; // forward declaration

class Trajectory {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(Trajectory);
  friend class boost::serialization::access;

 public:
  friend class ContinuousScene;
  typedef typename aslam::splines::OPTBSpline<typename ::bsplines::EuclideanBSpline<4, 2, bsplines::NsecTimePolicy>::CONF>::BSpline Spline;
  typedef Spline::dv_t DesignVariable;
  typedef aslam::backend::GenericMatrixExpression<2, 1> TrajectoryValueExpression;
  typedef bsplines::NsecTimePolicy::time_t SplineTime;

  Trajectory() = default;
  Trajectory(const Trajectory& t) = default;
  ~Trajectory() { }

  /**
   * Reset the trajectory to initial condition
   */
  void reset() { }

  /**
   * Is the trajectory initialized?
   * @return True iff initialized
   */
  bool isInitialized() const { return _spline.isInitialized(); }

  /**
   * Initializes a spline with all positions at zero
   * @param start start time
   * @param end end time
   * @param numSegments number of segments
   */
  void initZeroSpline(const planning2d::Time& start, const planning2d::Time& end, const std::size_t numSegments);

  /**
   * Initialize spline from discretized state trajectory. To this point, only the positions are used for fitting.
   * Optionally the time range can be specified.
   * The spline will be unconstrained in the fir in domain regions without discrete points.
   * @param[in] trajectory discrete time stamped states of a trajectory
   * @param[in] numberOfSegments Number of spline segments
   * @param[in] lambda weight for the regularization term
   * @param[in] minTime Optionally specify a minimum time for the spline different than the first stamp in \p trajectory
   * @param[in] maxTime Optionally specify a maximum time for the spline different than the first stamp in \p trajectory
   */
  void initFromDiscretizedTrajectory(const planning2d::StateTrajectory& trajectory,
                                     const int numberOfSegments,
                                     const double lambda,
                                     const planning2d::Time* minTime = nullptr,
                                     const planning2d::Time* maxTime = nullptr);

  /**
   * Initialize spline from discretized position trajectory. Optionally the time range can be specified.
   * The spline will be unconstrained in the fir in domain regions without discrete points.
   * @param[in] trajectory discrete time stamped states of a trajectory
   * @param[in] numberOfSegments Number of spline segments
   * @param[in] lambda weight for the regularization term
   * @param[in] minTime Optionally specify a minimum time for the spline different than the first stamp in \p trajectory
   * @param[in] maxTime Optionally specify a maximum time for the spline different than the first stamp in \p trajectory
   */
  void initFromDiscretizedTrajectory(const planning2d::PositionTrajectory& trajectory,
                                     const int numberOfSegments,
                                     const double lambda,
                                     const planning2d::Time* minTime = nullptr,
                                     const planning2d::Time* maxTime = nullptr);

  /**
   * Initialize straight spline from start and goal states. To this point, only the positions are used for fitting.
   * @param[in] startingPoint starting point for the spline
   * @param[in] goalPoint goal point for the spline
   * @param[in] numberOfSegments Number of spline segments
   * @param[in] lambda weight for the regularization term
   */
  void initStraightSpline(const planning2d::StateStamped& startingPoint,
                          const planning2d::StateStamped& goalPoint,
                          const int numberOfSegments,
                          const double lambda);

  /**
   * Initialize straight spline from start and goal positions
   * @param[in] start starting position for the spline
   * @param[in] goal goal position for the spline
   * @param[in] numberOfSegments Number of spline segments
   * @param[in] lambda weight for the regularization term
   */
  void initStraightSpline(const planning2d::Position2dStamped& start,
                          const planning2d::Position2dStamped& goal,
                          const int numberOfSegments,
                          const double lambda);

  /**
   * Shift the time interval. Only forward shifts are allowed.
   * @param minTime New start time
   * @param maxTime New end time
   * @param extrapolateFcn Functor with signature
   *        Position2d pos = (const planning2d::Position2dStamped& pos, const Eigen::Vector2d& velXY, const planning2d::Time& stamp),
   *        that is called whenever a position has to be extrapolated. Default implementation uses
   *        constant linear velocity extrapolation.
   * @param lambda Lambda parameter for the new spline fit
   */
  template <typename ExtrapolationFunctor>
  inline void adjustTime(const planning2d::Time& minTime, const planning2d::Time& maxTime,
                        ExtrapolationFunctor extrapolateFcn, const double lambda = 1e-3,
                        const planning2d::Duration& samplingResolution = 0.1);
  inline void adjustTime(const planning2d::Time& minTime, const planning2d::Time& maxTime,
                        const double lambda = 1e-3, const planning2d::Duration& samplingResolution = 0.1);

  /**
   * Shift the time interval. Only forward shifts are allowed.
   * @param shift Shift by this amount forward in time
   * @param extrapolateFcn Functor with signature
   *        Position2d pos = (const planning2d::Position2dStamped& pos, const Eigen::Vector2d& velXY, const planning2d::Time& stamp),
   *        that is called whenever a position has to be extrapolated. Default implementation uses
   *        constant linear velocity extrapolation.
   * @param lambda Lambda parameter for the new spline fit
   */
  template <typename ExtrapolationFunctor>
  inline void shiftTime(const planning2d::Duration& shift, ExtrapolationFunctor extrapolateFcn,
                        const double lambda = 1e-3, const planning2d::Duration& samplingResolution = 0.1);
  inline void shiftTime(const planning2d::Duration& shift, const double lambda = 1e-3,
                        const planning2d::Duration& samplingResolution = 0.1);

  /**
   * extract spline parameters/design variables
   */
  std::size_t numDesignVariables() const { return _spline.numDesignVariables(); }
  std::size_t numActiveDesignVariables();
  const std::vector<DesignVariable*>& getDesignVariables() { return _spline.getDesignVariables(); }
  const DesignVariable* designVariable(const std::size_t i) const { return _spline.designVariable(i); }
  void activateAllDesignVariables(bool activate);

  /**
   * Adds all design variables of this spline to the optimization problem
   * Note: Don't forget to activate the desired ones
   * @param problem Optimization problem
   * @param activate Activate the design variables iff true. If false, will leave the active state of the dv as is.
   */
  void addDesignVariables(aslam::backend::OptimizationProblem& problem, bool activate);

  /**
   * Deactivate all design variables that influence a certain timestamp
   * @param timestamp point in time for which the relevant design variables will be deactivated
   */
  void deactiveDesignVariablesAtTime(const planning2d::Time& timestamp);

  /**
   * get derivative of spline w.r.t spline parameters
   * @param timestamp time of derivative
   * @return jacobian matrix cJacobianomprising first-order derivative w.r.t. design variables of spline
   */
  Eigen::MatrixXd getJacobian(const planning2d::Time& timestamp) const;

  /**
   * Get spline itself in order to make it easier to analyze from the outside
   * @return Euclidean B-spline
   */
  const Spline& getSpline() const { return _spline; }

  /**
   * Get spline control vertices
   * @return Euclidean B-spline control vertices
   */
  Eigen::MatrixXd getSplineControlVertices() const;

  /**
   * Get vector expression of Spline at timestamp
   * @param timestamp time at which the spline has to be evaluated
   * @tparam D derivative order
   * @return vector expression (size 2) of the 2D point
   */
  template <int D>
  TrajectoryValueExpression getExpressionFromSpline(const planning2d::Time& timestamp) const;

  /**
   * Get vector expression of Spline for position at timestamp
   * @param timestamp time at which the spline has to be evaluated
   * @return vector expression (size 2) of the 2D point
   */
  TrajectoryValueExpression getExpressionFromSplinePos2d(const planning2d::Time& timestamp) const;

  /**
   * Get vector expression of Spline for velocity at timestamp
   * @param timestamp time at which the spline has to be evaluated
   * @return vector expression (size 2) of the velocity
   */
  TrajectoryValueExpression getExpressionFromSplineVelXY(const planning2d::Time& timestamp) const;

  /**
   * Get vector expression of Spline for acceleration at timestamp
   * @param timestamp time at which the spline has to be evaluated
   * @return vector expression (size 2) of the acceleration
   */
  TrajectoryValueExpression getExpressionFromSplineAccXY(const planning2d::Time& timestamp) const;

  /**
   * Get vector expression of Spline for rotation rate at timestamp
   * @param timestamp time at which the spline has to be evaluated
   * @return vector expression (size 2) of the rotation rate
   */
  aslam::backend::GenericMatrixExpression<1, 1> getExpressionFromSplineRotationRateSquared(const planning2d::Time& timestamp) const;

  /**
   * query methods, evaluate spline at certain timestep
   * (evaluate spline and time derivative at certain
   * timestep in order to get state information)
   * @param timestamp time at which spline has to be evaluated
   */
  planning2d::StateStamped getStateStamped(const planning2d::Time& timestamp) const;
  planning2d::Pose2d getPose2d(const planning2d::Time& timestamp) const;
  planning2d::Position2d getPosition2d(const planning2d::Time& timestamp) const;
  Eigen::Vector2d getPosition(const planning2d::Time& timestamp) const;
  Eigen::Vector3d getPose(const planning2d::Time& timestamp) const;
  Eigen::Vector2d getVelocityXY(const planning2d::Time& timestamp) const;
  Eigen::Vector2d getAccelerationXY(const planning2d::Time& timestamp) const;
  Eigen::Vector2d getVelocityTransRot(const planning2d::Time& timestamp) const;
  Eigen::Matrix<double,5,1> getPoseAndVelocityXY(const planning2d::Time& timestamp) const;
  double getRotationRateSquared(const planning2d::Time& timestamp) const;

  template <int IMaximalDerivativeOrder>
  Spline::Evaluator<IMaximalDerivativeOrder> getEvaluatorAt(const planning2d::Time& timestamp) const;

  /**
   * timing methods
   */
  inline planning2d::Time getStartTime() const;
  inline planning2d::Time getFinalTime() const;
  inline bool contains(const planning2d::Time& stamp) const;

  /**
   * get trajectory as discrete points in time, spline has to be evaluated at
   * certain timesteps to get the trajectory
   * @param dt discretization time of the desired discrete trajectory
   * @param startTime timestamp at which discrete trajectory should start
   * @param endTime timestamp at which discrete trajectory should end
   */
  planning2d::StateTrajectory discretizeTrajectory(const planning2d::Duration& dt) const;
  planning2d::StateTrajectory discretizeTrajectoryForPeriod(const planning2d::Duration& dt,
                                                            const planning2d::Time& startTime,
                                                            const planning2d::Time& endTime) const;


  //! Serialization methods
  template<class Archive>
  inline void save(Archive & ar, const unsigned int version) const;
  template<class Archive>
  inline void load(Archive & ar, const unsigned int version);
  BOOST_SERIALIZATION_SPLIT_MEMBER();

 private:
  /// \brief Mutable getter for design variable
  DesignVariable* designVariable(const std::size_t i) { return _spline.designVariable(i); }

  template <int IMaximalDerivativeOrder>
  TrajectoryValueExpression getExpressionFromSplinePos2d(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const;
  template <int IMaximalDerivativeOrder>
  TrajectoryValueExpression getExpressionFromSplineVelXY(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const;
  template <int IMaximalDerivativeOrder>
  TrajectoryValueExpression getExpressionFromSplineAccXY(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const;
  template <int IMaximalDerivativeOrder>
  aslam::backend::GenericMatrixExpression<1, 1> getExpressionFromSplineRotationRateSquared(const Spline::ExpressionFactory< Spline::ConstTimeFactoryData<IMaximalDerivativeOrder> >& factory) const;

  template <int IMaximalDerivativeOrder>
  inline Eigen::Vector2d getPosition(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const;
  template <int IMaximalDerivativeOrder>
  inline Eigen::Vector3d getPose(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const;
  template <int IMaximalDerivativeOrder>
  inline Eigen::Vector2d getVelocityXY(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const;
  template <int IMaximalDerivativeOrder>
  inline Eigen::Vector2d getAccelerationXY(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const;
  template <int IMaximalDerivativeOrder>
  inline Eigen::Vector2d getVelocityTransRot(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const;
  template <int IMaximalDerivativeOrder>
  inline Eigen::Matrix<double,5,1> getPoseAndVelocityXY(const Spline::Evaluator<IMaximalDerivativeOrder>& evaluator) const;

 private:
  Spline _spline;

};


} // end namespace prob_planner

#include "impl/TrajectoryImpl.hpp"

#endif /* TRAJECTORY_HPP_ */
