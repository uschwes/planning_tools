/*
 * ContinuousScene.cpp
 *
 *  Created on: 05.05.2015
 *      Author: sculrich
 */

#include "../include/probabilistic_planner/state_representation/ContinuousScene.hpp"

#include "sm/logging.hpp"

namespace prob_planner {

using namespace aslam::backend;

void ContinuousScene::copy(ContinuousScene& scene) const {
  scene.clear();
  scene._observations = _observations;
  scene._agentContainer.clear();
  for (auto& oa : _agentContainer) {
    OptAgent oac = oa.second.copy();
    scene.addOptAgent(oac);
  }
}

void ContinuousScene::clear() {
  clearAgentContainer();
  _observations.clear();
}

void ContinuousScene::clearAgentContainer() {
  _agentContainer.clear();
}

bool ContinuousScene::empty() const {
  return _agentContainer.empty() & _observations.empty();
}

std::size_t ContinuousScene::numberOfAgents() const {
  return _agentContainer.size();
}

std::size_t ContinuousScene::numberOfObservations() const {
  return _observations.size();
}

bool ContinuousScene::hasAgent(const planning2d::Id id) const {
  AgentContainer::const_iterator it = _agentContainer.find(id);
  return it != _agentContainer.end();
}

void ContinuousScene::addOptAgent(const OptAgent& agent) {
  SM_ASSERT_FALSE(planning2d::FunctionInputException , this->hasAgent(agent.getAgent()->getId()),
                  "Agent with id " << agent.getAgent()->getId() << " already exists.");
  SM_ASSERT_TRUE(planning2d::ParameterException, agent.trajectory().isInitialized(),
                 "It is illegal to add an OptAgent whose trajectory is not initialized. Please " <<
                 "initialize the trajectory before adding the OptAgent to the scene.");
  _agentContainer.insert(AgentContainer::value_type(agent.getAgent()->getId(), agent));
}

void ContinuousScene::updateOptAgent(const OptAgent& agent) {
  auto it = _agentContainer.find(agent.getId());
  if (it == _agentContainer.end())
    _agentContainer.emplace(agent.getId(), agent);
  else
    it->second = agent;
}

const OptAgent& ContinuousScene::getOptAgent(const planning2d::Id& id) const {
  try {
    return _agentContainer.at(id);
  } catch (const std::out_of_range& e) {
    SM_THROW(planning2d::OutOfBoundAccessException, "Agent with id " << id << " does not exist.")
  } catch(...) {
    throw;
  }
}

OptAgent& ContinuousScene::getOptAgent(const planning2d::Id& id) {
  SM_ASSERT_TRUE_DBG(planning2d::FunctionInputException , this->hasAgent(id),
                  "Agent with id " << id << " does not exist.");
  return _agentContainer.at(id);
}

void ContinuousScene::removeOptAgent(const planning2d::Id& id) {
  if(hasAgent(id))
    _agentContainer.erase(id);
  else
    SM_ERROR_STREAM("Agent " << id << " cannot be removed because it's not part of the scene. ");
}

void ContinuousScene::removeUnobservedAgents() {
  auto observedAgents = getObservedAgents();
  removeAllAgentsExcept(observedAgents);
}

std::set<planning2d::Id> ContinuousScene::getObservedAgents() const {
  std::set<planning2d::Id> observedAgents;
  for (const auto& snapshot : _observations) {
    for (const auto& object : snapshot->objectContainer())
      observedAgents.insert(object.first);
  }
  return observedAgents;
}

void ContinuousScene::trimObservations(const planning2d::Time& cutoffTime) {
  for (auto it=_observations.begin(); it!=_observations.end(); ) {
    if ((*it)->stamp() < cutoffTime)
      it = _observations.erase(it);
    else
      ++it;
  }
}

void ContinuousScene::sortObservationsByAscendingTime() {
  std::sort(_observations.begin(), _observations.end(), [](SceneSnapshot::Ptr snap1, SceneSnapshot::Ptr snap2){ return (snap1->stamp() < snap2->stamp()); });
}

void ContinuousScene::keepLatestObservationPerAgent() {
  const auto observedAgents = getObservedAgents();
  sortObservationsByAscendingTime();
  for (const auto& id : observedAgents) {
    int cnt = 0;
    for (auto rit=_observations.rbegin(); rit!=_observations.rend(); ++rit) {
      auto itAgent = (*rit)->objectContainer().find(id);
      if (itAgent != (*rit)->objectContainer().end()) {
        cnt += 1;
        if (cnt > 1)
          (*rit)->objectContainer().erase(itAgent);
      }
    }
  }
}

planning2d::StateTrajectory ContinuousScene::getObservedAgentStates(const planning2d::Id& id) const {
  planning2d::StateTrajectory observedStates;
  for (auto& snap : _observations) {
    if (snap->hasObject(id)) {
      planning2d::State state = snap->getObject(id);
      observedStates.push_back(planning2d::StateStamped(state, snap->stamp()));
    }
  }
  // Sort vector
  std::sort(observedStates.begin(), observedStates.end(), [](const planning2d::StateStamped& state1, const planning2d::StateStamped& state2){return (state1.stamp()<state2.stamp());});
  return observedStates;
}

void ContinuousScene::addObservation(const SceneSnapshot::Ptr& observation) {
  _observations.push_back(observation);
  if (_observations.size() > _maxBufferSize) {
    const auto diffSize = _observations.size() - _maxBufferSize;
    this->sortObservationsByAscendingTime();
    _observations.erase(_observations.begin(), _observations.begin() + diffSize);
    SM_WARN_STREAM("Observation buffer in scene is full. Clearing oldest " << diffSize << " observation(s).");
  }
}

const std::vector<SceneSnapshot::ConstPtr>& ContinuousScene::getObservations() const {
  return reinterpret_cast<const std::vector<SceneSnapshot::ConstPtr>&>(_observations);
}

planning2d::Time ContinuousScene::getMinTime() const {
  SM_ASSERT_FALSE( planning2d::RuntimeException, _agentContainer.empty(), "You must not call this function without any agents inserted");
  planning2d::Time minTime = _agentContainer.begin()->second.trajectory().getStartTime();
  for (auto it=++_agentContainer.begin(); it!=_agentContainer.end(); ++it)
    minTime = std::min(it->second.trajectory().getStartTime(), minTime);

  return minTime;
}

planning2d::Time ContinuousScene::getMaxTime() const {
  SM_ASSERT_FALSE( planning2d::RuntimeException, _agentContainer.empty(), "You must not call this function without any agents inserted");
  planning2d::Time maxTime = _agentContainer.begin()->second.trajectory().getFinalTime();
  for (auto it=++_agentContainer.begin(); it!=_agentContainer.end(); ++it)
    maxTime = std::max(it->second.trajectory().getFinalTime(), maxTime);

  return maxTime;
}

void ContinuousScene::activateAllDesignVariables(bool activate) {
  for (auto& oa : _agentContainer)
    oa.second.trajectory().activateAllDesignVariables(activate);
}

std::size_t ContinuousScene::numActiveSplineParameters() const {
  std::size_t numParameters = 0;
  for (auto& oa : _agentContainer) {
    for (size_t i=0; i<oa.second.trajectory().numDesignVariables(); i++)
      if (oa.second.trajectory().designVariable(i)->isActive()) {
        // Note: Here we assume vector-space design variables
        numParameters += oa.second.trajectory().designVariable(i)->minimalDimensions();
      }
  }
  return numParameters;
}

Eigen::VectorXd ContinuousScene::activeSplineParameters() const {
  Eigen::VectorXd parameters(this->numActiveSplineParameters());
  std::size_t cnt = 0;
  Eigen::MatrixXd p;
  for (auto& oa : _agentContainer) {
    for (size_t i=0; i<oa.second.trajectory().numDesignVariables(); i++) {
      if (oa.second.trajectory().designVariable(i)->isActive()) {
        const DesignVariable* dv = oa.second.trajectory().designVariable(i);
        dv->getParameters(p);
        const int dim = p.size();
        parameters.segment(cnt, dim) = Eigen::VectorXd(Eigen::Map<Eigen::VectorXd>(p.data(), dim));
        cnt += dim;
      }
    }
  }
  return parameters;
}

void ContinuousScene::addDesignVariables(aslam::backend::OptimizationProblem& problem, bool autoActivate) {
  for (auto& oa : _agentContainer) {
    oa.second.addDesignVariables(problem, autoActivate);
  }
}

void ContinuousScene::setActiveSplineParameters(const Eigen::VectorXd& parameters) {
  std::size_t cnt = 0;
  for (auto& oa : _agentContainer) {
    for (size_t i=0; i<oa.second.trajectory().numDesignVariables(); i++) {
      if (oa.second.trajectory().designVariable(i)->isActive()) {
        DesignVariable* dv = oa.second.trajectory().designVariable(i);
        const int dim = dv->minimalDimensions();
//        Eigen::MatrixXd p(dv->minimalDimensions(), 1);
//        p.col(0) = parameters.segment(cnt, dv->minimalDimensions());
        dv->setParameters(parameters.segment(cnt, dim));
        cnt += dim;
      }
    }
  }
}

}  /* namespace prob_planner */
