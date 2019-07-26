/*
 * ProbabilisticPlanner.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: pfmark
 */

#include "../include/probabilistic_planner/ProbabilisticPlanner.hpp"
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>
#include <probabilistic_planner/Support.hpp>

#include <common_agents/DifferentialDriveAgent.hpp>
#include <common_agents/PedestrianAgent.hpp>

#include <sm/timing/Timer.hpp>
#include <sm/logging.hpp>


namespace prob_planner {

ProbabilisticPlanner::ProbabilisticPlanner(sm::value_store::ValueStore& vpt)
    : _problem(new aslam::backend::OptimizationProblem()),
      _featureContainer(vpt)
{
  _optimizer.setProblem(_problem);
  this->init();
}

ProbabilisticPlanner::ProbabilisticPlanner(sm::value_store::ValueStore& vpt, const Optimizer::Options& options)
    : ProbabilisticPlanner(vpt)
{
  _optimizer.setOptions(options);
}

ProbabilisticPlanner::ProbabilisticPlanner(const std::string& xmlPath)
    : _problem(new aslam::backend::OptimizationProblem()),
      _featureContainer(xmlPath)
{
  _optimizer.setProblem(_problem);
  this->init();
}

ProbabilisticPlanner::ProbabilisticPlanner(const std::string& xmlPath, const Optimizer::Options& options)
    : ProbabilisticPlanner(xmlPath)
{
  _optimizer.setOptions(options);
}

void ProbabilisticPlanner::reset() {
  _scene.clear();
  _isEgoTargetValid = false;
  _problem->clear();
}

void prob_planner::ProbabilisticPlanner::init() {

  _invCovMatrix(0,0) = 100;
  _invCovMatrix(1,1) = 100;
  _invCovMatrix(2,2) = 100;
  _invCovMatrix(3,3) = 100;
  _invCovMatrix(4,4) = 100;

  // Features
  assignTargetFeature();
  setMaximumVelocity(2.0);

}

/**********************
 *  pure virtual functions inherited from TargetPlannerInterface class
 **********************/

planning2d::ReturnCode ProbabilisticPlanner::computePlan(const planning2d::Time& currentTime,
                                                         planning2d::StateInputTrajectory& stateInputSequence) {

  using namespace planning2d::time;
  ::prob_planner::Timer timer(std::string("ProbabilisticPlanner: ") + __FUNCTION__, false);

  // Shift the interval of the scene
  _scene.trimObservations(currentTime - _params.observationHistoryLength);
  _problem->clear();

  SM_FINE_STREAM_NAMED("probabilistic_planner", "Scene has " << _scene.numberOfAgents() << " agents and " << _scene.numberOfObservations() << " observations.");

  // get rid of the agents we did not observe any more
  _scene.removeUnobservedAgents();

  if (_scene.numberOfAgents() > 0) {

    // add the design variables of the agent's trajectory splines to the problem
    // inlcude the information, whether agent should be interacting with others (otherwise constant velocity predictions)
    for (auto& oa : _scene.getOptAgentContainer())
      oa.second.addDesignVariables(*_problem, _isInteractionAware || oa.second.getId() == ID_EGO);

    // add feature based error terms to optimization problem
    _featureContainer.addErrorTerms(_scene, *_problem);

    // initialize the optimizer with the new problem
    _optimizer.initialize();

    // Run optimization
    SM_FINE_STREAM_NAMED("probabilistic_planner", "Starting optimization.");

    auto timeStart = planning2d::time::getCurrentTime();
    _optimizer.optimize();
    auto returnValueOptimizer = _optimizer.getStatus();
    planning2d::Duration computationTime = (planning2d::time::getCurrentTime() - timeStart);

    SM_WARN_STREAM_COND(computationTime > planning2d::Duration(0.5), "Computation time was " << computationTime.format(Formatter(MILLISEC)) << " ms.");

    SM_DEBUG_STREAM_NAMED("probabilistic_planner", returnValueOptimizer << std::endl <<
                          "\tcomputation time: " << computationTime.format(Formatter(MILLISEC)) << std::endl <<
                          "\tnumber of agents: " << _scene.numberOfAgents());

    if (!returnValueOptimizer.success())
      SM_WARN_STREAM("Optimizer did not converge. Convergence criterion status is " << returnValueOptimizer.convergence);

    // Extract robot trajectory / planned path for the robot
    this->getTrajectoryFromScene(stateInputSequence, ID_EGO);

  } else {
    SM_WARN_STREAM("Scene is empty. Cannot plan with empty scene.");
  }

  return planning2d::RETCODE_OK;
}


planning2d::ReturnCode ProbabilisticPlanner::callbackOccupancyGrid(const planning2d::OccupancyGridStamped& grid) {
  ::prob_planner::Timer timer(std::string("ProbabilisticPlanner: ") + __FUNCTION__, false);
  auto snPtr = boost::make_shared<SceneSnapshot>(grid.stamp());
  snPtr->setOccupancyGrid(grid);
  _scene.addObservation(snPtr);

  return planning2d::RETCODE_OK;
}

planning2d::ReturnCode ProbabilisticPlanner::callbackCurrentState(const planning2d::StateStamped& stateStamped) {
  ::prob_planner::Timer timer(std::string("ProbabilisticPlanner: ") + __FUNCTION__, false);

  // make sure the current state of the robot is a differential drive state, will throw
  auto egoState = dynamic_cast<const common_agents::DifferentialDriveAgent::StateStamped&>(stateStamped);

  common_agents::HolonomicAgent::StateStamped measurementState;
  measurementState.stamp() = stateStamped.stamp();
  measurementState.pose() = stateStamped.pose();
  measurementState.setVelX(egoState.getVelX());
  measurementState.setVelY(egoState.getVelY());
  _egoAgent->stateStamped() = measurementState;
  double distanceToGoal = (measurementState.pose().position().asVector()-_robotTarget.position().asVector()).norm();

  bool trajectoryAdjusted = true;
  bool withinTargetRange = (distanceToGoal < _closeTargetRange);
  if (_scene.hasAgent(ID_EGO)) {
    if ( !withinTargetRange ) {
      try {
        adjustTrajectoryToNewTime(_egoAgent);
      } catch (const std::exception& e) {
        trajectoryAdjusted = false;
      }
    } else if (withinTargetRange || !trajectoryAdjusted) {
      _scene.removeOptAgent(ID_EGO);
      addEgoAgentToScene(measurementState);
    }
  }
  else {
    addEgoAgentToScene(measurementState);
  }

  auto snPtr = boost::make_shared<SceneSnapshot>(stateStamped.stamp());
  snPtr->addObject(ID_EGO, StateWithUncertainty(measurementState, 10.0*_invCovMatrix));
  _scene.addObservation(snPtr);

  return planning2d::RETCODE_OK;
}

planning2d::ReturnCode ProbabilisticPlanner::callbackSetTarget(const planning2d::Pose2d& target, bool isFinal) {
  ::prob_planner::Timer timer(std::string("ProbabilisticPlanner: ") + __FUNCTION__, false);

  _robotTarget = target;
  _isEgoTargetValid = true;
  _targetIsFinal = isFinal;
  if (_scene.hasAgent(ID_EGO)) {
    SM_FINE_STREAM_NAMED("probabilistic_planner", "Target was originally set to " << _robotTarget << " with distance " << (_scene.getOptAgent(ID_EGO).getAgent()->stateStamped().pose().position().asVector() - _robotTarget.position().asVector()).norm());
    _robotTarget = adjustTargetToPlanningHorizon(_scene.getOptAgent(ID_EGO).getAgent()->stateStamped());
    SM_FINE_STREAM_NAMED("probabilistic_planner", "Target was adjusted to " << _robotTarget << " with distance " << (_scene.getOptAgent(ID_EGO).getAgent()->stateStamped().pose().position().asVector() - _robotTarget.position().asVector()).norm());
  }
  if (_targetFeature != nullptr) {
    _targetFeature->setTarget(target);
    // Scale weight if target is final one
    if (_targetIsFinal)
      _targetFeature->setWeight(0,_params.finalTargetScaling * _targetWeight);
    else
      _targetFeature->setWeight(0, _targetWeight);
    SM_FINE_STREAM_NAMED("probabilistic_planner", "Target in targetFeature was set to " << _targetFeature->getTarget());
  }
  return planning2d::RETCODE_OK;
}


planning2d::ReturnCode ProbabilisticPlanner::callbackDynamicObjects(const std::vector<planning2d::Agent::ConstPtr>& dynamicObjects) {
  ::prob_planner::Timer timer(std::string("ProbabilisticPlanner: ") + __FUNCTION__, false);

  std::set<planning2d::Id> observedAgents;
  observedAgents.insert(ID_EGO);

  for (auto& agent : dynamicObjects) {
    this->addOrAdaptDynamicObject(agent);
    observedAgents.insert(agent->getId());
    auto snPtr = boost::make_shared<SceneSnapshot>(agent->stateStamped().stamp());
    snPtr->addObject(agent->getId(), StateWithUncertainty(agent->stateStamped(), _invCovMatrix));
    _scene.addObservation(snPtr);
  }

  if (!_params.agentMemory)
    _scene.removeAllAgentsExcept(observedAgents);

  return planning2d::RETCODE_OK;
}


planning2d::Pose2dStamped ProbabilisticPlanner::adjustTargetToPlanningHorizon(const planning2d::StateStamped& currentState) {

  if(!_isEgoTargetValid) {
    SM_WARN_STREAM_THROTTLE(5.0, "Target for ego robot not specified yet, setting target to current pose " << currentState.pose());
    return planning2d::Pose2dStamped(currentState.pose(), currentState.stamp() + _params.planningHorizon);
  }

  planning2d::Position2d distanceToTarget = _robotTarget.position() - currentState.pose().position();
  double estimatedTimeToTarget(distanceToTarget.norm() / std::max((_desiredVelocity + currentState.state().norm())/2.0, 0.1));
  planning2d::Pose2dStamped adjustedTarget(_robotTarget, currentState.stamp() + planning2d::Duration(estimatedTimeToTarget));


  if (distanceToTarget.norm() > _maxProjectedTargetDistance) {
    adjustedTarget.stamp() = currentState.stamp() + _params.planningHorizon;
    adjustedTarget.position() = currentState.pose().position() + distanceToTarget.cwiseProduct(_maxProjectedTargetDistance / distanceToTarget.asVector().norm());
  }

  return adjustedTarget;
}


void ProbabilisticPlanner::getTrajectoryFromScene(planning2d::StateInputTrajectory& stateInputSequence, planning2d::Id id) const {
  SM_ASSERT_TRUE(planning2d::RuntimeException, _scene.hasAgent(id), "Agent with id " << id << " you are trying to query is not in scene.");
  stateInputSequence = this->discretizeStateInputTrajectory(_scene.getOptAgent(id).trajectory(), id, 0.05);
}


void ProbabilisticPlanner::addDynamicObjectToScene(planning2d::Agent::ConstPtr dynamicObject) {

  // For now, assure that every other agent is a pedestrian
  common_agents::PedestrianAgent::ConstPtr pedestrian = boost::dynamic_pointer_cast<const common_agents::PedestrianAgent>(dynamicObject);
  SM_ASSERT_TRUE( planning2d::FunctionInputException, pedestrian != nullptr, "All dynamic have to be pedestrians");

  // Estimate Target
  double xTarget = pedestrian->stateStamped().pose().position().x() + _params.planningHorizon.toSec()*pedestrian->stateStamped().getVelX();
  double yTarget = pedestrian->stateStamped().pose().position().y() + _params.planningHorizon.toSec()*pedestrian->stateStamped().getVelY();
  double headingTarget = pedestrian->stateStamped().pose().yaw();
  double vxTarget = pedestrian->stateStamped().getVelX();
  double vyTarget = pedestrian->stateStamped().getVelY();
  planning2d::Time timeTarget = pedestrian->stateStamped().stamp() + _params.planningHorizon;

  common_agents::PedestrianAgent::StateStamped target(common_agents::PedestrianAgent::State(xTarget, yTarget, headingTarget, vxTarget, vyTarget), timeTarget);

  // Initialize Trajectory
  Trajectory::Ptr trajectory(new Trajectory());
  trajectory->initStraightSpline(pedestrian->stateStamped(), target, std::round(_params.planningHorizon.toSec()/_params.segmentsPerSecond), _params.splineLambda);

  // Create OptAgent
  OptAgent optAgent(pedestrian, trajectory, OptAgentType::PEDESTRIAN);

  // Add to scene
  _scene.addOptAgent(optAgent);
}

void ProbabilisticPlanner::addOrAdaptDynamicObject(planning2d::Agent::ConstPtr dynamicObject) {

  if (_isInteractionAware && dynamicObject->interactionAware()) {
    bool trajectoryAdjusted = true;
    if (_scene.hasAgent(dynamicObject->getId())) {
      try {
        adjustTrajectoryToNewTime(dynamicObject);
      } catch (const std::exception& e) {
        trajectoryAdjusted = false;
      }
    } else {
      addDynamicObjectToScene(dynamicObject);
    }

    // in case the adjustment failed, initialize trajectory as a new straight one
    if (!trajectoryAdjusted) {
      _scene.removeOptAgent(dynamicObject->getId());
      addDynamicObjectToScene(dynamicObject);
    }
  } else {
    if (_scene.hasAgent(dynamicObject->getId())) {
      _scene.removeOptAgent(dynamicObject->getId());
    }
    addDynamicObjectToScene(dynamicObject);
  }
}

void ProbabilisticPlanner::addEgoAgentToScene(const planning2d::StateStamped& stateStamped) {

  _egoAgent->stateStamped() = stateStamped;

  // Target state
  auto targetPose = adjustTargetToPlanningHorizon(stateStamped);
  common_agents::PedestrianAgent::StateStamped targetStateStamped(common_agents::PedestrianAgent::State(0.0, 0.0, targetPose), targetPose.stamp());

  // Create initial trajectory
  Trajectory::Ptr trajectory(new Trajectory());
  trajectory->initStraightSpline(stateStamped, targetStateStamped, std::max(5, static_cast<int>(std::round((targetStateStamped.stamp() - stateStamped.stamp()).toSec()/_params.segmentsPerSecond))), 1e-2);

  // Add to scene
  OptAgent optAgent(_egoAgent, trajectory, OptAgentType::ROBOT);
  _scene.addOptAgent(optAgent);
}

planning2d::StateInputTrajectory ProbabilisticPlanner::discretizeStateInputTrajectory(const Trajectory& trajectory, const planning2d::Id& id, const double dt) const {
  planning2d::StateInputTrajectory stateInputTrajectory;
  // time interval has to be limited in order to avoid extrapolation
  const double start = trajectory.getStartTime().toSec();
  const double end = trajectory.getFinalTime().toSec();
  stateInputTrajectory.reserve(static_cast<size_t>((end - start)/dt));
  for (double t = start; t <= end; t += dt){
    planning2d::Time timestamp(t);

    Eigen::Vector2d velTransRot;
    planning2d::Pose2d pose;
    pose.position() = trajectory.getPosition2d(timestamp);

    try {
      velTransRot = trajectory.getVelocityTransRot(timestamp);
      pose.yaw() = trajectory.getPose2d(timestamp).yaw();
    } catch (const std::exception& e) {
      if (!stateInputTrajectory.empty()) {
        velTransRot = stateInputTrajectory.back().state().state();
        pose.yaw() = stateInputTrajectory.back().state().pose().yaw();
      } else {
        common_agents::DifferentialDriveState ddState(_scene.getOptAgent(id).getAgent()->stateStamped());
        velTransRot[0] = ddState.getVelTrans();
        velTransRot[1] = ddState.getVelRot();
        pose.yaw() = ddState.yaw();
        SM_WARN_STREAM_THROTTLE(1, "Approximating heading and velocity with current yaw " << pose.yaw() << " and velocity " << velTransRot.transpose() << ".");
      }
    }

    stateInputTrajectory.push_back(planning2d::StateInputPairStamped(planning2d::State(velTransRot, pose),
                                                                     planning2d::SystemInput(velTransRot),
                                                                     timestamp));
  }
  return stateInputTrajectory;
}

void ProbabilisticPlanner::adjustTrajectoryToNewTime(planning2d::Agent::ConstPtr agent) {

  Trajectory agentTrajectory = _scene.getOptAgentContainer().at(agent->getId()).trajectory();
  planning2d::StateTrajectory observedStates = _scene.getObservedAgentStates(agent->getId());
  OptAgentType optAgentType = _scene.getOptAgentContainer().at(agent->getId()).getType();
  planning2d::Time startTime = agent->stateStamped().stamp();
  planning2d::Time endTime = startTime + _params.planningHorizon;
  SM_ASSERT_TRUE(planning2d::RuntimeException, agentTrajectory.contains(startTime), "Start time of new trajectory has to lie within the valid spline interval of the previous trajectory.")
  _scene.removeOptAgent(agent->getId());

  planning2d::Duration dt(0.1);
  planning2d::StateTrajectory discreteTrajectory;  // don't use observations if ego agent
  if (agent->getId()!=ID_EGO)
    planning2d::StateTrajectory discreteTrajectory = observedStates;

  auto finalTimeSpline = agentTrajectory.getFinalTime();

  // Discretize previous trajectory and extrapolate if necessary
  for (planning2d::Time t=startTime; t<=endTime; t += dt) {
    if (t < finalTimeSpline && (_isInteractionAware || agent->getId() == ID_EGO)) {
      planning2d::StateStamped stateStamped;
      try {
        stateStamped = agentTrajectory.getStateStamped(t);
      } catch (const std::exception& e) {
        stateStamped.stamp() = t;
        stateStamped.pose().position() = agentTrajectory.getPosition2d(t);
        if (!discreteTrajectory.empty()) {
          stateStamped.state() = discreteTrajectory.back().state();
          stateStamped.pose().yaw() = discreteTrajectory.back().pose().yaw();
        } else {
          stateStamped.state() = agent->stateStamped().state();
          stateStamped.pose().yaw() = agent->stateStamped().pose().yaw();
          SM_WARN_STREAM_THROTTLE(1.0, "Approximating heading and velocity with current yaw " << stateStamped.pose().yaw() << " and velocity " << stateStamped.state().transpose() << ".");
        }
      }
      discreteTrajectory.push_back(stateStamped);
    } else if (discreteTrajectory.size() >= 2) {
      auto s1 = discreteTrajectory[discreteTrajectory.size()-2];
      auto s2 = discreteTrajectory.back();
      planning2d::StateStamped extrapolatedState;
      extrapolatedState.pose().position().setVector(s1.pose().position().asVector() +
                                                   (s2.pose().position().asVector() - s1.pose().position().asVector()) /
                                                   (s2.stamp() - s1.stamp()).toSec() * (t - s1.stamp()).toSec());
      extrapolatedState.pose().yaw() = s2.pose().yaw();
      extrapolatedState.state() = s2.state();
      extrapolatedState.stamp() = t;
      discreteTrajectory.push_back(extrapolatedState);
    } else {
      SM_ERROR_STREAM("Position out of scope of previous trajectory.");
    }
  }

  SM_FINE_STREAM_NAMED("probabilistic_planner", "Discrete trajectory duration is " << discreteTrajectory.back().stamp().toSec()-discreteTrajectory.front().stamp().toSec() << "s");

  if (discreteTrajectory.size()>2) {
    double distanceMeasuremntToTrajectoryStart = (agent->stateStamped().pose().position().asVector()- discreteTrajectory.front().pose().position().asVector()).norm();
    SM_ASSERT_LE(planning2d::RuntimeException, distanceMeasuremntToTrajectoryStart, 1.0, "trajectory has separated from the agent");
    // Initialize scene
    Trajectory::Ptr trajectory(new Trajectory());
    trajectory->initFromDiscretizedTrajectory(discreteTrajectory, std::max(static_cast<int>(std::round(_params.planningHorizon.toSec()/_params.segmentsPerSecond)), 1), _params.splineLambda);

    // Create OptAgent
    OptAgent optAgent(agent, trajectory, optAgentType);

    // Add to scene
    _scene.addOptAgent(optAgent);
  }
}

void ProbabilisticPlanner::assignTargetFeature(){
  auto targetFeature = _featureContainer.getFeature(FeatureSingletonRobotTarget::CLASS_NAME);
  if (targetFeature != nullptr) {
    _targetFeature = boost::dynamic_pointer_cast<FeatureSingletonRobotTarget>(targetFeature);
    _targetWeight = _targetFeature->getWeight();
    SM_INFO_STREAM("standard target weight was set to " << _targetWeight);
  }
  else
    SM_DEBUG_STREAM_NAMED("probabilistic_planner", "Cannot assign target feature since it is not part of feature container");
}

void ProbabilisticPlanner::setMaximumVelocity(const double barrierVel){
  auto barrierVelocityFeature = _featureContainer.getFeature(FeatureSingletonIntegratedBarrierVelocity::CLASS_NAME);
  if (barrierVelocityFeature != nullptr) {
    _barrierVelocityFeature = boost::dynamic_pointer_cast<FeatureSingletonIntegratedBarrierVelocity>(barrierVelocityFeature);
    _barrierVelocityFeature->setMaximumVelocity(barrierVel);
  } else {
    SM_DEBUG_STREAM_NAMED("probabilistic_planner", "Barrier velocity could not be set to " << barrierVel << " m/s. Feature is not part of the feature container.");
  }
}

void ProbabilisticPlanner::initializeEgoTrajectory() {

}

} /* namespace prob_planner */
