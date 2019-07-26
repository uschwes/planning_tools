/*
 * DifferentialDriveAgent.cpp
 *
 *  Created on: May 4, 2015
 *      Author: pfmark
 */

// FCL includes
#include <fcl/shape/geometric_shapes.h>

#include <common_agents/DifferentialDriveAgent.hpp>

namespace common_agents {

using namespace planning2d;

DifferentialDriveAgent::DifferentialDriveAgent()
{

}


DifferentialDriveAgent::DifferentialDriveAgent(bool isControllable,
                                               const planning2d::Id id,
                                               const DifferentialDriveStateStamped& stateStamped,
                                               CollisionGeometryConstPtr geometry) :
        Agent(isControllable, id),
        _stateStamped(stateStamped),
        _geometry(geometry)
{
  CollisionGeometryPtr geom = boost::const_pointer_cast<CollisionGeometry>(_geometry);
  _wtCollObj = CollisionObjectPtr(new planning2d::CollisionObject(geom));
}


DifferentialDriveAgent::~DifferentialDriveAgent()
{
}

void DifferentialDriveAgent::applyInput(const planning2d::SystemInput& /*input*/,
                                        Duration /*dt*/)
{
  SM_THROW(planning2d::NoImplementationException, "Not implemented");
}

planning2d::SystemInput DifferentialDriveAgent::computeControlOutput(
    const planning2d::StateStamped& /*referenceState*/) const
{
  SM_THROW(planning2d::NoImplementationException, "Not implemented");
  return planning2d::SystemInput();
}

planning2d::SystemInput DifferentialDriveAgent::computeControlOutput(
    const Pose2d& /*referencePose*/) const
{
  SM_THROW(planning2d::NoImplementationException, "Not implemented");
  return planning2d::SystemInput();
}

planning2d::SystemInput DifferentialDriveAgent::computeControlOutput(
    const StateTrajectory& /*referenceTrajectory*/) const
{
  SM_THROW(planning2d::NoImplementationException, "Not implemented");
  return planning2d::SystemInput();
}

planning2d::SystemInput DifferentialDriveAgent::computeControlOutput(
    const HolonomicVelocity& /*referenceSpeed*/) const
{
  SM_THROW(planning2d::NoImplementationException, "Not implemented");
  return planning2d::SystemInput();
}

planning2d::CollisionObjectPtr DifferentialDriveAgent::getWorkspaceTimeCollisionObject() const
{
  SM_ASSERT_TRUE( planning2d::InitializationException, _wtCollObj != nullptr, "");
  fcl::Transform3f tf;
  fcl::Quaternion3f q;
  q.fromEuler(0., 0., _stateStamped.pose().yaw());
  tf.setTranslation(fcl::Vec3f(_stateStamped.pose().position().x(), _stateStamped.pose().position().y(), _stateStamped.stamp().toSec()));
  tf.setQuatRotation(q);
  _wtCollObj->setTransform(tf);
  return _wtCollObj;
}

void DifferentialDriveAgent::setCollisionGeometry(planning2d::CollisionGeometryConstPtr geom) {
  _geometry = geom;
  CollisionGeometryPtr g = boost::const_pointer_cast<CollisionGeometry>(_geometry);
  _wtCollObj = CollisionObjectPtr(new planning2d::CollisionObject(g));
}

const planning2d::DiscApproximation& DifferentialDriveAgent::getDiscApproximation(
    std::size_t nMaxCircles) const
{
  SM_ASSERT_TRUE( planning2d::InitializationException, _geometry != nullptr, "Set the collision geometry first.");
  SM_ASSERT_EQ(planning2d::NoImplementationException, nMaxCircles, 1, "No disk approximation for box shapes with more than one disk implemented so far.");

  _diskApproximation = planning2d::DiscApproximation();
  boost::shared_ptr<const fcl::Box> shape = boost::dynamic_pointer_cast<const fcl::Box>(_geometry);
  if (shape != nullptr) {

    _diskApproximation.insertDisc( hypot(shape->side[0]/2., shape->side[1]/2.), _stateStamped.pose().position());

  } else {

    boost::shared_ptr<const fcl::Cylinder> shape = boost::dynamic_pointer_cast<const fcl::Cylinder>(_geometry);
    if (shape != nullptr) {

      _diskApproximation.insertDisc( shape->radius, _stateStamped.pose().position());

    } else {
      SM_THROW(planning2d::NoImplementationException, "No disk approximation for other shapes implemented so far.");
    }
  }
  return _diskApproximation;
}

}

