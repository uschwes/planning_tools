#ifndef PLANNER_INTERFACES_AGENT_WITH_TARGET_HPP_
#define PLANNER_INTERFACES_AGENT_WITH_TARGET_HPP_

#include "Agent.hpp"

namespace planning2d {

  /**
   * @class Agent An agent with target is an agent with an getTarget method returning a stamped state.
   */
  class AgentWithTarget : virtual public Agent {
    public:
      PLANNING_2D_POINTER_TYPEDEFS(AgentWithTarget);

      //! Default constructor
      inline AgentWithTarget() = default;

      /** Constructor taking care of all member variable initializations
       *  @param isControllable Whether or not we can control this agent
       */
      inline AgentWithTarget(bool isControllable) : Agent(isControllable, ID_INVALID) { }

      //! destructor
      inline virtual ~AgentWithTarget() { }

      virtual const StateStamped& getTarget() const = 0;
  }; /* class AgentWithTarget */
  
} /* namespace planning2d */

#endif /* PLANNER_INTERFACES_AGENT_WITH_TARGET_HPP_ */
