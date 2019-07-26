/*
 * ProbabilisticPlanner.hpp
 *
 *  Created on: Apr 10, 2015
 *      Author: pfmark
 */

#ifndef PROBABILISTICPLANNER_HPP_
#define PROBABILISTICPLANNER_HPP_

#include <boost/shared_ptr.hpp>

#include <fcl/shape/geometric_shapes.h>

#include <sm/BoostPropertyTree.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/OptimizerBFGS.hpp>
#include <aslam/backend/OptimizerRprop.hpp>

#include <common_agents/DifferentialDriveAgent.hpp>

#include <planner_interfaces/TargetPlannerInterface.hpp>
#include <planner_interfaces/Trajectory.hpp>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>
#include <probabilistic_planner/features/FeatureContainer.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTarget.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>

namespace prob_planner {

class ProbabilisticPlanner : public planning2d::TargetPlannerInterface {

 public:

  PLANNING_2D_POINTER_TYPEDEFS(ProbabilisticPlanner);
//  typedef aslam::backend::OptimizerRprop Optimizer;
  typedef aslam::backend::OptimizerBFGS Optimizer;

  struct Parameters {
    Parameters() { }
    planning2d::Duration planningHorizon = 7.0; /// Planning horizon [s]
    int segmentsPerSecond = 1; /// Spline segments per second
    planning2d::Duration observationHistoryLength = 1.0; /// determines the length of the measurement history which is taken into account [s]
    double splineLambda = 1e-2;
    bool agentMemory = false;
    double finalTargetScaling = 1.0;
  };

 public:
  // From property tree
  ProbabilisticPlanner(sm::value_store::ValueStore& vpt);
  ProbabilisticPlanner(sm::value_store::ValueStore& vpt, const Optimizer::Options& options);
  // From file (property tree stored as xml)
  ProbabilisticPlanner(const std::string& xmlPath);
  ProbabilisticPlanner(const std::string& xmlPath, const Optimizer::Options& options);
  ~ProbabilisticPlanner() { }

  void reset() override;

  void init();

  /**********************
   *  pure virtual functions inherited from TargetPlannerInterface class
   **********************/

  planning2d::ReturnCode computePlan(const planning2d::Time& currentTime,
                                     planning2d::StateInputTrajectory& stateInputSequence) override;

  planning2d::ReturnCode callbackOccupancyGrid(const planning2d::OccupancyGridStamped& grid) override;

  planning2d::ReturnCode callbackCurrentState(const planning2d::StateStamped& stateStamped) override;

  planning2d::ReturnCode callbackSetTarget(const planning2d::Pose2d& target, bool isFinal) override;

  planning2d::ReturnCode callbackDynamicObjects(const std::vector<planning2d::Agent::ConstPtr>& dynamicObjects) override;

  void setDesiredVelocity(const double desiredVel) { _desiredVelocity = desiredVel; }
  double getDesiredVelocity() const { return _desiredVelocity; }

  // helper functions for test purposes
  const FeatureContainer& getFeatureContainer() const { return _featureContainer; }
  const ContinuousScene& getScene() const { return _scene; }
  const aslam::backend::OptimizationProblem& getOptimizationProblem() { return *_problem; }
  void addDynamicObjectToScene(planning2d::Agent::ConstPtr dynamicObject);
  void addEgoAgentToScene(const planning2d::StateStamped& stateStamped);

  const Parameters& getParameters() const { return _params; }
  void setMaximumVelocity(const double barrierVel);
  void initializeEgoTrajectory();

  void enableInteractionAwareness(const bool doEnable) {
    _isInteractionAware = doEnable;
    SM_INFO_STREAM("Interaction awareness: " << _isInteractionAware);
  }

 protected:
  // Features
  void assignTargetFeature();

 private:
  planning2d::Pose2dStamped adjustTargetToPlanningHorizon(const planning2d::StateStamped& currentState);
  void getTrajectoryFromScene(planning2d::StateInputTrajectory& stateInputSequence, planning2d::Id id) const;
  planning2d::StateInputTrajectory discretizeStateInputTrajectory(const Trajectory& trajectory, const planning2d::Id& id, const double dt) const;
  void adjustTrajectoryToNewTime(planning2d::Agent::ConstPtr agent);
  void addOrAdaptDynamicObject(planning2d::Agent::ConstPtr dynamicObject);

  Parameters _params;
  bool _isEgoTargetValid = false;
  bool _targetIsFinal = false;
  double _targetWeight;
  bool _isInteractionAware = true;
  double _maxProjectedTargetDistance = 10.0;
  double _closeTargetRange = 3.0;
  planning2d::Pose2d _robotTarget;
  Eigen::MatrixXd _invCovMatrix = Eigen::MatrixXd::Zero(5,5);
  double _desiredVelocity = 1.0;    /// desired robot velocity [m/s]
  boost::shared_ptr<aslam::backend::OptimizationProblem> _problem;
  FeatureContainer _featureContainer;
  boost::shared_ptr<FeatureSingletonRobotTarget> _targetFeature = nullptr;
  boost::shared_ptr<FeatureSingletonIntegratedBarrierVelocity> _barrierVelocityFeature = nullptr;
  ContinuousScene _scene;
  Optimizer _optimizer;
  common_agents::DifferentialDriveAgent::Ptr _egoAgent = common_agents::DifferentialDriveAgent::Ptr(new common_agents::DifferentialDriveAgent(false, ID_EGO, common_agents::DifferentialDriveAgent::StateStamped(), planning2d::CollisionGeometryPtr(new fcl::Cylinder(0.4, 2.0))));

};


} // end namespace prob_planner

#endif /* PROBABILISTICPLANNER_HPP_ */
