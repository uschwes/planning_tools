/*
 * ContinuousScene.hpp
 *
 *  Created on: Apr 10, 2015
 *      Author: pfmark
 */

#ifndef PROBABILISTIC_PLANNER_STATE_REPRESENTATION_CONTINUOUSSCENE_HPP_
#define PROBABILISTIC_PLANNER_STATE_REPRESENTATION_CONTINUOUSSCENE_HPP_

#include <iostream>
#include <map>

#include <Eigen/Core>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/OccupancyGrid.hpp>
#include <planner_interfaces/Agent.hpp>

#include "../include/probabilistic_planner/state_representation/OptAgent.hpp"
#include "../include/probabilistic_planner/state_representation/SceneSnapshot.hpp"

namespace prob_planner {

class ContinuousScene {

 public:
  typedef std::map<planning2d::Id, OptAgent> AgentContainer;
  PLANNING_2D_POINTER_TYPEDEFS(ContinuousScene);
  friend class boost::serialization::access;

 public:
  ContinuousScene() { }
  ContinuousScene(const ContinuousScene&) = delete;
  ~ContinuousScene() { }

  /// \brief Perform deep copy
  void copy(ContinuousScene& scene) const;

  void clear();
  void clearAgentContainer();
  bool empty() const;
  std::size_t numberOfAgents() const;
  std::size_t numberOfObservations() const;

  // Agents
  void setOptAgentContainer(const AgentContainer& agentMap) { _agentContainer = agentMap; }
  const AgentContainer& getOptAgentContainer() const { return _agentContainer; }
  AgentContainer& getOptAgentContainer() { return _agentContainer; }

  bool hasAgent(const planning2d::Id id) const;
  void addOptAgent(const OptAgent& agent);
  void updateOptAgent(const OptAgent& agent);
  const OptAgent& getOptAgent(const planning2d::Id& id) const;
  OptAgent& getOptAgent(const planning2d::Id& id);
  void removeOptAgent(const planning2d::Id& id);
  void removeUnobservedAgents();
  template <template <typename, typename...> class Container>
  void removeAllAgentsExcept(const Container<planning2d::Id>& agentsToKeep);

  /// \brief Const getter for the observations
  const std::vector<SceneSnapshot::ConstPtr>& getObservations() const;

  /// \brief getter for a list of observed agents (specified by agent ID)
  std::set<planning2d::Id> getObservedAgents() const;
  /// \brief get trajectory of observed agent states in the past for specific agent (agent ID)
  planning2d::StateTrajectory getObservedAgentStates(const planning2d::Id& id) const;
  /// \brief Add an observation
  void addObservation(const SceneSnapshot::Ptr& observation);
  /// \brief Trims the observations by time. All observations that are older than the specified cutoff time get removed.
  void trimObservations(const planning2d::Time& cutoffTime);
  void keepLatestObservationPerAgent();
  void sortObservationsByAscendingTime();

  // Timing
  planning2d::Time getMinTime() const;
  planning2d::Time getMaxTime() const;

  //  Helper functions
  /// \brief Activates/Deactivates all spline design variables
  void activateAllDesignVariables(bool activate);
  /// \brief total number of active spline parameters (number of design variables * dimensionality)
  std::size_t numActiveSplineParameters() const;
  /// \brief Get the value of all active spline parameters in a flattened array
  Eigen::VectorXd activeSplineParameters() const;
  /// \brief Adds all design variables of this scene to the optimization problem
  void addDesignVariables(aslam::backend::OptimizationProblem& problem, bool autoActivate);

  std::size_t getMaxBufferSize() const { return _maxBufferSize; }
  void setMaxBufferSize(std::size_t maxBufferSize) { _maxBufferSize = maxBufferSize; }

  // Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 protected:
  /// \brief Set the values of all active spline parameters from a flattened array
  void setActiveSplineParameters(const Eigen::VectorXd& parameters);

 private:
  AgentContainer _agentContainer;
  std::vector<SceneSnapshot::Ptr> _observations;
  std::size_t _maxBufferSize = 10000;

};

} /* namespace prob_planner */

#include "impl/ContinuousSceneImpl.hpp"

#endif /* PROBABILISTIC_PLANNER_STATE_REPRESENTATION_CONTINUOUSSCENE_HPP_ */
