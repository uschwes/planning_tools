/*
 * TestAgent.cpp
 *
 *  Created on: Sep 30, 2014
 *      Author: sculrich
 */

// FCL includes
#include <fcl/shape/geometric_shapes.h>

#include <planner_interfaces/TestAgent.hpp>


namespace planning2d {

using namespace fcl;

TestAgent::TestAgent() : Agent() { }


TestAgent::TestAgent(Id id) :
  Agent(true /*isControllable*/, id) {

  _currentState.pose() = Pose2d(0.0, 0.0, 0.0);
  _currentState.linearVelocity() = 1.0;
  _currentState.stamp() = Time((double)0.0);

  const double extentTimeDimensionSec = 2.0;
  const double radius = 1.0;

  // Insert a disc with radius 1.0 at the robot's center point
  _diskApproximation.insertDisc(radius, Position2d(0., 0.));

  // create FCL collision object
  const boost::shared_ptr<CollisionGeometry> cylinderShape(new Cylinder(radius, extentTimeDimensionSec));
  _collisionObject.reset(new CollisionObject(cylinderShape));

}

TestAgent::~TestAgent() {

}

// MMP and V-Charge planner rely on the ability to forward simulate a system model
void TestAgent::applyInput(const planning2d::SystemInput& input, Duration dt) {
  const double dtSec = dt.toSec();
  _currentState.pose().position().x() = _currentState.pose().position().x() + _currentState.linearVelocity()*dtSec;
//  _currentState.pose().position().y() = _currentState.pose().position().y();
  _currentState.pose().yaw() = _currentState.pose().yaw() + dtSec;
  _currentState.linearVelocity() = TestAgent::SystemInput(input).linearVelocity();
  _currentState.stamp() = (_currentState.stamp() + dt);
}

// Controller implementations
::planning2d::SystemInput TestAgent::computeControlOutput(const ::planning2d::StateStamped& /* referenceState */) const {
  TestAgent::SystemInput input;
  input.linearVelocity() = 0.1;
  return input;
}

::planning2d::SystemInput TestAgent::computeControlOutput(const Pose2d& /* referencePose */) const {
  TestAgent::SystemInput input;
  input.linearVelocity() = 0.1;
  return input;
}

::planning2d::SystemInput TestAgent::computeControlOutput(const StateTrajectory& /* referenceTrajectory */) const {
  TestAgent::SystemInput input;
  input.linearVelocity() = 0.1;
  return input;
}

::planning2d::SystemInput TestAgent::computeControlOutput(const HolonomicVelocity& /* referenceSpeed */) const {
  TestAgent::SystemInput input;
  input.linearVelocity() = 0.1;
  return input;
}

CollisionObjectPtr TestAgent::getWorkspaceTimeCollisionObject() const {
  _collisionObject->setTranslation(fcl::Vec3f(_currentState.pose().position().x(), _currentState.pose().position().y(), _currentState.pose().yaw()));
  fcl::Matrix3f mat;
  mat.setEulerYPR(_currentState.pose().yaw(), 0., 0.);
  _collisionObject->setRotation(mat);
  return _collisionObject;
}

CollisionGeometryConstPtr TestAgent::getCollisionGeometry() const {
  return CollisionGeometryConstPtr(_collisionObject->collisionGeometry());
}

const DiscApproximation& TestAgent::getDiscApproximation(std::size_t nMaxCircles) const {
  SM_ASSERT_LE(planning2d::RuntimeException, this->_diskApproximation.getNumDiscs(), nMaxCircles, "");
  return this->_diskApproximation;
}

Agent::Ptr TestAgent::clone() const {
  return Agent::Ptr(new TestAgent(*this));
}


}
