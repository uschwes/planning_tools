/*
 * OptAgent.hpp
 *
 *  Created on: Apr 29, 2015
 *      Author: pfmark
 */

#ifndef OPTAGENT_HPP_
#define OPTAGENT_HPP_

#include <functional>

#include <planner_interfaces/Agent.hpp>
#include <planner_interfaces/Support.hpp>

#include <probabilistic_planner/state_representation/Trajectory.hpp>
#include <probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp>

namespace prob_planner {

class OptAgent {

 public:
  PLANNING_2D_POINTER_TYPEDEFS(OptAgent);
  typedef std::reference_wrapper<OptAgent> Ref;
  typedef std::reference_wrapper<const OptAgent> ConstRef;
  friend class boost::serialization::access;
  template<typename V0, typename V1> friend class std::pair; // hack for serialization

 public:
  OptAgent(const OptAgent &) = default;

  /// \brief Constructor
  OptAgent(planning2d::Agent::ConstPtr agent, Trajectory::Ptr trajectory, const OptAgentType& type) :
    _agent(agent),
    _trajectory(trajectory),
    _type(type) {
    SM_ASSERT_FALSE(planning2d::InitializationException, agent==nullptr, " ");
    SM_ASSERT_FALSE(planning2d::InitializationException, trajectory==nullptr, " ");
  }

  /// \brief destructor
  ~OptAgent() { }

  /// \brief perform deep copy
  inline OptAgent copy() const;

  /**
   * @brief Adds all design variables from this agent to the optimization problem. If the agent is not active,
   *        the design variables will be deactivated after a call to this method.
   * @param problem Optimization problem
   * @param autoActivate Activates all design variables iff true. If false, leaves the state untouched.
   */
  inline void addDesignVariables(aslam::backend::OptimizationProblem& problem, bool autoActivate);

  /// \brief Setter for agent
  void setAgent(planning2d::Agent::ConstPtr agentPtr) { _agent = agentPtr; }
  /// \brief Const getter for agent
  planning2d::Agent::ConstPtr getAgent() const { return _agent; }

  /// \brief Const getter for trajectory
  const Trajectory& trajectory() const { return *_trajectory; }
  /// \brief Mutable getter for trajectory
  Trajectory& trajectory() { return *_trajectory; }
  /// \brief Setter for trajectory
  void setTrajectory(Trajectory::Ptr trajectory) { _trajectory = trajectory; }

  /// \brief Getter for agent type
  OptAgentType getType() const { return _type; }
  /// \brief Setter for agent type
  void setType(const OptAgentType type) { _type = type; }

  /// \brief Getter for agent id
  planning2d::Id getId() const { return _agent->getId(); }


  /// \brief Setter/getter for active flag
  bool isActive() const { return _isActive; }
  void setActive(const bool active) { _isActive = active; }

  /// \brief Serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int version);

 private:
  OptAgent() { } /// \brief Private default constructor

 private:
  bool _isActive = true;              /// \brief Is active for optimization
  planning2d::Agent::ConstPtr _agent; /// \brief agent
  Trajectory::Ptr _trajectory;        /// \brief optimizable trajectory for agent
  OptAgentType _type = UNKNOWN;       /// \brief opt agent type
};


inline OptAgent OptAgent::copy() const {
  return OptAgent (_agent->clone(), Trajectory::Ptr(new Trajectory(*_trajectory)), _type);
}

} /* namespace prob_planner */


#include "impl/OptAgentImpl.hpp"

#endif /* OPTAGENT_HPP_ */
