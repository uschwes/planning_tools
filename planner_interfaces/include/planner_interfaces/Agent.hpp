#ifndef PLANNER_INTERFACES_AGENT_HPP_
#define PLANNER_INTERFACES_AGENT_HPP_

#include <limits>

#include <Eigen/Dense>

#include <sm/assert_macros.hpp>

#include "Support.hpp"
#include "Exceptions.hpp"
#include "Pose2d.hpp"
#include "State.hpp"
#include "SystemInput.hpp"
#include "Trajectory.hpp"
#include "HolonomicVelocity.hpp"
#include "Shape.hpp"
#include "Time.hpp"

namespace planning2d {

  using namespace std;

  /**
   * @class Agent An agent is comprised of its system model, its geometric properties and some control laws.
   */
  class Agent {
  
    public:
      PLANNING_2D_POINTER_TYPEDEFS(Agent);

    public:

      //! Default constructor
      inline Agent() : _isControllable(false), _id(ID_INVALID), _isInteractionAware(true) { }

      /** Constructor taking care of all member variable initializations
       *  @param isControllable Whether or not we can control this agent
       *  @param id The id of the agent
       */
      inline Agent(bool isControllable, Id id) : _isControllable(isControllable), _id(id), _isInteractionAware(true) { }

      //! destructor
      inline virtual ~Agent() { }

      //! Call after using the default constructor in order to initialize the agent properly
      inline void initialize(bool isControllable, Id id) { _isControllable = isControllable; _id = id; }

      //! Set whether or not we can control this agent
      inline void setControllable(bool isControllable) { _isControllable = isControllable; }

      //! Returns reference to interaction awareness of agent
      const bool& interactionAware() const { return _isInteractionAware; }
      void setInteractionAware(bool interactionAware) {_isInteractionAware = interactionAware; }

      //! Returns const reference to the current stamped state of the agent (stamp + pose + state variables)
      virtual const StateStamped& stateStamped() const = 0;

      //! Returns mutable reference to the stamped state of the agent (stamp + pose + state variables)
      virtual StateStamped& stateStamped() = 0;

      /** *************************** **/
      /** System model implementation **/
      /** *************************** **/

      /** Apply system input for duration dt to system. The agent should change its internal state based on its system model.
       * @param input System input
       * @param dt Duration over which the system input shall by applied
       */
      virtual void applyInput(const SystemInput& input, Duration dt) = 0;
      
      /** ************************** **/
      /** Controller implementations **/
      /** ************************** **/
      virtual SystemInput computeControlOutput(const StateStamped& referenceState) const = 0;
      virtual SystemInput computeControlOutput(const Pose2d& referencePose) const = 0;
      virtual SystemInput computeControlOutput(const StateTrajectory& referenceTrajectory) const = 0;
      virtual SystemInput computeControlOutput(const HolonomicVelocity& referenceSpeed) const = 0;
      
      /** *********************************** **/
      /** Collision detection implementations **/
      /** *********************************** **/

      /** Return a collision object in x,y,time space.
       * The collision object encodes an object in 2D workspace (x,y) plus time. The time dimension has to be in seconds!
       * @return Collision object
       */
      virtual CollisionObjectPtr getWorkspaceTimeCollisionObject() const = 0;

      /** Return a collision geometry in 3D workspace (x,y,z).
       * The collision object encodes an object in 3D workspace (x,y,z).
       * We utilize this type due to the solely 3D collision shapes defined in FCL.
       * @return Collision geometry
       */
      virtual CollisionGeometryConstPtr getCollisionGeometry() const = 0;

      /** Return approximation of the agent's 2D shape as a collection of discs
       * @param nMaxCircles Return not more than \var nMaxCircles in the approximation
       * @return Disc Approximation object
       */
      virtual const DiscApproximation& getDiscApproximation(std::size_t nMaxCircles) const = 0;
      
      //! deep copy
      virtual Agent::Ptr clone() const = 0;

      //! getter for agent id
      Id getId() const {
        SM_ASSERT_NE(InitializationException, _id, ID_INVALID,
                   "Specify the id first");
        return _id;
      }

      //! setter for agent id
      void setId(const Id id) { _id = id; }

      //! getter
      bool isControllable() const { return _isControllable; }

      //! serialization method
      template<class Archive>
      inline void serialize(Archive & ar, const unsigned int /*version*/) { ar & _isControllable & _id & _isInteractionAware; }

    private:

      bool _isControllable; /// @brief Whether or not this agent can be controlled by us
      Id _id; /// @brief The agent's unique identifier
      bool _isInteractionAware; /// @brief Whether the agent interacts with others or will behave according to a constant velocity model

  }; /* class Agent */
  
} /* namespace planning2d */

#endif /* PLANNER_INTERFACES_AGENT_HPP_ */
