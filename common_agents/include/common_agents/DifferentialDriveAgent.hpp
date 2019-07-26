/*
 * DifferentialDriveAgent.hpp
 *
 *  Created on: Apr 30, 2015
 *      Author: pfmark
 */

#ifndef COMMON_AGENTS_DIFFERENTIALDRIVEAGENT_HPP_
#define COMMON_AGENTS_DIFFERENTIALDRIVEAGENT_HPP_

#include <planner_interfaces/Agent.hpp>
#include <planner_interfaces/FclSupport.hpp>

#include "DifferentialDriveState.hpp"
#include "DifferentialDriveSystemInput.hpp"

namespace common_agents {

class DifferentialDriveAgent : public planning2d::Agent {

 public:
  typedef DifferentialDriveState State;
  typedef DifferentialDriveSystemInput SystemInput;
  typedef DifferentialDriveStateStamped StateStamped;
  typedef DifferentialDriveSystemInputStamped SystemInputStamped;

  PLANNING_2D_POINTER_TYPEDEFS(DifferentialDriveAgent);

 public:
  DifferentialDriveAgent();
  DifferentialDriveAgent(bool isControllable,
                         const planning2d::Id id,
                         const DifferentialDriveStateStamped& stateStamped,
                         planning2d::CollisionGeometryConstPtr geometry);
  virtual ~DifferentialDriveAgent();

  /** Apply system input for duration dt to system. The agent should change its internal state based on its system model.
   * @param input System input
   * @param dt Duration over which the system input shall by applied
   */
  void applyInput(const planning2d::SystemInput& input, planning2d::Duration dt);

  //! Returns const reference to the current stamped state of the agent (stamp + pose + state variables)
  const DifferentialDriveStateStamped& stateStamped() const override { return _stateStamped; }

  //! Returns mutable reference to the stamped state of the agent (stamp + pose + state variables)
  DifferentialDriveStateStamped& stateStamped() override { return _stateStamped; }

  /** ************************** **/
  /** Controller implementations **/
  /** ************************** **/
  planning2d::SystemInput computeControlOutput(const planning2d::StateStamped& referenceState) const override;
  planning2d::SystemInput computeControlOutput(const planning2d::Pose2d& referencePose) const override;
  planning2d::SystemInput computeControlOutput(const planning2d::StateTrajectory& referenceTrajectory) const override;
  planning2d::SystemInput computeControlOutput(const planning2d::HolonomicVelocity& referenceSpeed) const override;

  /** *********************************** **/
  /** Collision detection implementations **/
  /** *********************************** **/

  /** Return a collision object in x,y,time space.
   * The collision object encodes an object in 2D workspace (x,y) plus time. The time dimension has to be in seconds!
   * @return Collision object
   */
  planning2d::CollisionObjectPtr getWorkspaceTimeCollisionObject() const override;

  /** Return a collision geometry in 3D workspace (x,y,z).
   * The collision object encodes an object in 3D workspace (x,y,z).
   * We utilize this type due to the solely 3D collision shapes defined in FCL.
   * @return Collision geometry
   */
  planning2d::CollisionGeometryConstPtr getCollisionGeometry() const override { return _geometry; }
  void setCollisionGeometry(planning2d::CollisionGeometryConstPtr geom);

  /** Return approximation of the agent's 2D shape as a collection of discs
   * @param nMaxCircles Return not more than \var nMaxCircles in the approximation
   * @return Disc Approximation object
   */
  const planning2d::DiscApproximation& getDiscApproximation(std::size_t nMaxCircles) const override;

  //! deep copy
  planning2d::Agent::Ptr clone() const override { return planning2d::Agent::Ptr(new DifferentialDriveAgent(*this)); }

  //! serialization method
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) {
    REGISTER_FCL_TYPES_SERIALIZATION(ar);
    ar & boost::serialization::base_object<planning2d::Agent>(*this);
    ar & _stateStamped;
    ar & _geometry;
  }

 private:
  DifferentialDriveStateStamped _stateStamped; // TODO: DifferentialDriveStateStamped needs StateStamped as parent
  planning2d::CollisionGeometryConstPtr _geometry;
  mutable planning2d::DiscApproximation _diskApproximation;
  planning2d::CollisionObjectPtr _wtCollObj;

};

}



#endif /* COMMON_AGENTS_DIFFERENTIALDRIVEAGENT_HPP_ */
