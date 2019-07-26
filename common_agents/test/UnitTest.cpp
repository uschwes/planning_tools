#include <gtest/gtest.h>

// interface includes
#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/State.hpp>
#include <planner_interfaces/Shape.hpp>

// FCL includes
#include <fcl/shape/geometric_shapes.h>

#include "../include/common_agents/DifferentialDriveAgent.hpp"
#include "../include/common_agents/HolonomicAgent.hpp"
#include "../include/common_agents/PedestrianAgent.hpp"

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

using namespace std;
using namespace planning2d;
using namespace common_agents;

template <class T>
string serialize(const T& data) {
  ostringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa.register_type<DifferentialDriveAgent::StateStamped>();
  oa.register_type<HolonomicAgent::StateStamped>();
  oa.register_type<DifferentialDriveAgent>();
  oa.register_type<HolonomicAgent>();
  oa << data;
  return ss.str();
}

template <class T>
T deserialize(const std::string& str) {
  stringstream s;
  s << str;

  boost::archive::binary_iarchive ia(s);
  ia.register_type<DifferentialDriveAgent::StateStamped>();
  ia.register_type<HolonomicAgent::StateStamped>();
  ia.register_type<DifferentialDriveAgent>();
  ia.register_type<HolonomicAgent>();
  T obj;
  ia >> obj;

  return obj;
}

template <typename AgentType>
void testSerialization(const AgentType& agent, const ::fcl::Box& shape) {
    const auto deserialized = deserialize<AgentType*>(serialize(&agent));
    ASSERT_FALSE(deserialized == nullptr);
    EXPECT_EQ(typeid(agent).name(), typeid(*deserialized).name());
    EXPECT_EQ(agent.getId(), deserialized->getId());
    ASSERT_FALSE(deserialized->getCollisionGeometry() == nullptr);
    EXPECT_EQ(typeid(*(agent.getCollisionGeometry())).name(), typeid(*(deserialized->getCollisionGeometry())).name());
    boost::shared_ptr<const ::fcl::Box> boxDeserialized = boost::dynamic_pointer_cast<const ::fcl::Box>(deserialized->getCollisionGeometry());
    ASSERT_NE(nullptr, boxDeserialized);
    EXPECT_DOUBLE_EQ(shape.side[0], boxDeserialized->side[0]);
    EXPECT_DOUBLE_EQ(shape.side[1], boxDeserialized->side[1]);
    EXPECT_DOUBLE_EQ(shape.side[2], boxDeserialized->side[2]);
}


TEST(common_agents_TESTSUITE, DifferentialDriveState) {

  try {

    // State
    const double transVel = 1.0;
    const double rotVel = 2.0;
    const double x = 0.1;
    const double y = 0.2;
    const double theta = 0.3;

    // Standard constructor
    {
      DifferentialDriveState s;
      EXPECT_EQ(2, s.dimension());
    }

    {
      // Constructor 1
      DifferentialDriveState s(transVel, rotVel, Pose2d(x, y, theta));
      EXPECT_DOUBLE_EQ(transVel, s.getVelTrans());
      EXPECT_DOUBLE_EQ(rotVel, s.getVelRot());
      EXPECT_DOUBLE_EQ(transVel, s.velocity()(0));
      EXPECT_DOUBLE_EQ(rotVel, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }

    {
      // Constructor 2
      Eigen::Vector2d velocity(transVel, rotVel);
      DifferentialDriveState s(velocity, Pose2d(x, y, theta));
      EXPECT_DOUBLE_EQ(transVel, s.getVelTrans());
      EXPECT_DOUBLE_EQ(rotVel, s.getVelRot());
      EXPECT_DOUBLE_EQ(transVel, s.velocity()(0));
      EXPECT_DOUBLE_EQ(rotVel, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }

    {
      // Constructor 3
      State stateParent(Eigen::Vector2d(transVel, rotVel), Pose2d(x, y, theta));
      DifferentialDriveState s(stateParent);
      EXPECT_DOUBLE_EQ(transVel, s.getVelTrans());
      EXPECT_DOUBLE_EQ(rotVel, s.getVelRot());
      EXPECT_DOUBLE_EQ(transVel, s.velocity()(0));
      EXPECT_DOUBLE_EQ(rotVel, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }


  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, HolonomicState) {

  try {

    // State
    const double vx = 1.0;
    const double vy = 2.0;
    const double x = 0.1;
    const double y = 0.2;
    const double theta = 0.3;

    // Standard constructor
    {
      HolonomicState s;
      EXPECT_EQ(2, s.dimension());
    }

    {
      // Constructor 1
      HolonomicState s(vx, vy, Pose2d(x, y, theta));
      EXPECT_DOUBLE_EQ(vx, s.getVelX());
      EXPECT_DOUBLE_EQ(vy, s.getVelY());
      EXPECT_DOUBLE_EQ(vx, s.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }

    {
      // Constructor 2
      Eigen::Vector2d velocity(vx, vy);
      HolonomicState s(velocity, Pose2d(x, y, theta));
      EXPECT_DOUBLE_EQ(vx, s.getVelX());
      EXPECT_DOUBLE_EQ(vy, s.getVelY());
      EXPECT_DOUBLE_EQ(vx, s.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }

    {
      // Constructor 3
      State stateParent(Eigen::Vector2d(vx, vy), Pose2d(x, y, theta));
      HolonomicState s(stateParent);
      EXPECT_DOUBLE_EQ(vx, s.getVelX());
      EXPECT_DOUBLE_EQ(vy, s.getVelY());
      EXPECT_DOUBLE_EQ(vx, s.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }

    {
      // Constructor 4
      HolonomicState s(x, y, theta, vx, vy);
      EXPECT_DOUBLE_EQ(vx, s.getVelX());
      EXPECT_DOUBLE_EQ(vy, s.getVelY());
      EXPECT_DOUBLE_EQ(vx, s.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, s.velocity()(1));
      EXPECT_DOUBLE_EQ(x, s.pose().position().x());
      EXPECT_DOUBLE_EQ(y, s.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, s.pose().yaw());
    }


  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, DifferentialDriveSystemInput) {

  try {

    const double transVel = 1.0;
    const double rotVel = 2.0;

    {
      // Standard constructor
      DifferentialDriveSystemInput u;
      EXPECT_EQ(2, u.dimension());
    }

    {
      // Constructor 1
      DifferentialDriveSystemInput u(transVel, rotVel);
      EXPECT_DOUBLE_EQ(transVel, u.getVelTrans());
      EXPECT_DOUBLE_EQ(rotVel, u.getVelRot());
      EXPECT_DOUBLE_EQ(transVel, u.velocity()(0));
      EXPECT_DOUBLE_EQ(rotVel, u.velocity()(1));
    }

    {
      // Constructor 2
      Eigen::Vector2d velocity(transVel, rotVel);
      DifferentialDriveSystemInput u(velocity);
      EXPECT_DOUBLE_EQ(transVel, u.getVelTrans());
      EXPECT_DOUBLE_EQ(rotVel, u.getVelRot());
      EXPECT_DOUBLE_EQ(transVel, u.velocity()(0));
      EXPECT_DOUBLE_EQ(rotVel, u.velocity()(1));
    }

    {
      // Constructor 3
      SystemInput stateParent(Eigen::Vector2d(transVel, rotVel));
      DifferentialDriveSystemInput u(stateParent);
      EXPECT_DOUBLE_EQ(transVel, u.getVelTrans());
      EXPECT_DOUBLE_EQ(rotVel, u.getVelRot());
      EXPECT_DOUBLE_EQ(transVel, u.velocity()(0));
      EXPECT_DOUBLE_EQ(rotVel, u.velocity()(1));
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, HolonomicSystemInput) {

  try {

    const double vx = 1.0;
    const double vy = 2.0;

    {
      // Standard constructor
      HolonomicSystemInput u;
      EXPECT_EQ(2, u.dimension());
    }

    {
      // Constructor 1
      HolonomicSystemInput u(vx, vy);
      EXPECT_DOUBLE_EQ(vx, u.getVelX());
      EXPECT_DOUBLE_EQ(vy, u.getVelY());
      EXPECT_DOUBLE_EQ(vx, u.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, u.velocity()(1));
    }

    {
      // Constructor 2
      Eigen::Vector2d velocity(vx, vy);
      HolonomicSystemInput u(velocity);
      EXPECT_DOUBLE_EQ(vx, u.getVelX());
      EXPECT_DOUBLE_EQ(vy, u.getVelY());
      EXPECT_DOUBLE_EQ(vx, u.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, u.velocity()(1));
    }

    {
      // Constructor 3
      SystemInput stateParent(Eigen::Vector2d(vx, vy));
      HolonomicSystemInput u(stateParent);
      EXPECT_DOUBLE_EQ(vx, u.getVelX());
      EXPECT_DOUBLE_EQ(vy, u.getVelY());
      EXPECT_DOUBLE_EQ(vx, u.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, u.velocity()(1));
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, DifferentialDriveStateStamped) {

  try {

    // State
    const double transVel = 1.0;
    const double rotVel = 2.0;
    const double x = 0.1;
    const double y = 0.2;
    const double theta = 0.3;

    const Time time(10.0);
    DifferentialDriveState state(transVel, rotVel, Pose2d(x, y, theta));
    DifferentialDriveStateStamped stateStamped(state, time);

    // Test state
    EXPECT_DOUBLE_EQ(transVel, stateStamped.getVelTrans());
    EXPECT_DOUBLE_EQ(rotVel, stateStamped.getVelRot());
    EXPECT_DOUBLE_EQ(transVel, stateStamped.velocity()(0));
    EXPECT_DOUBLE_EQ(rotVel, stateStamped.velocity()(1));
    EXPECT_DOUBLE_EQ(x, stateStamped.pose().position().x());
    EXPECT_DOUBLE_EQ(y, stateStamped.pose().position().y());
    EXPECT_DOUBLE_EQ(theta, stateStamped.pose().yaw());

    // Test time
    EXPECT_EQ(time, stateStamped.stamp());
    EXPECT_DOUBLE_EQ(10.0, stateStamped.stamp().toSec());

    // Test serialization
    StateStamped* p = &stateStamped;
    const auto deserialized = deserialize<StateStamped*>(serialize(p));
    EXPECT_FALSE(deserialized == nullptr);
    EXPECT_EQ(stateStamped, *deserialized);
    EXPECT_EQ(typeid(stateStamped).name(), typeid(*deserialized).name());

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, HolonomicStateStamped) {

  try {

    // State
    const double vx = 1.0;
    const double vy = 2.0;
    const double x = 0.1;
    const double y = 0.2;
    const double theta = 0.3;

    const Time time(10.0);
    HolonomicState state(vx, vy, Pose2d(x, y, theta));
    HolonomicStateStamped stateStamped(state, time);

    // Test state
    EXPECT_DOUBLE_EQ(vx, stateStamped.getVelX());
    EXPECT_DOUBLE_EQ(vy, stateStamped.getVelY());
    EXPECT_DOUBLE_EQ(vx, stateStamped.velocity()(0));
    EXPECT_DOUBLE_EQ(vy, stateStamped.velocity()(1));
    EXPECT_DOUBLE_EQ(x, stateStamped.pose().position().x());
    EXPECT_DOUBLE_EQ(y, stateStamped.pose().position().y());
    EXPECT_DOUBLE_EQ(theta, stateStamped.pose().yaw());

    // Test time
    EXPECT_EQ(time, stateStamped.stamp());
    EXPECT_DOUBLE_EQ(10.0, stateStamped.stamp().toSec());

    // Test other constructors
    {
      StateStamped s = static_cast<StateStamped>(stateStamped);
      stateStamped = HolonomicStateStamped(s);
      EXPECT_DOUBLE_EQ(vx, stateStamped.getVelX());
      EXPECT_DOUBLE_EQ(vy, stateStamped.getVelY());
      EXPECT_DOUBLE_EQ(vx, stateStamped.velocity()(0));
      EXPECT_DOUBLE_EQ(vy, stateStamped.velocity()(1));
      EXPECT_DOUBLE_EQ(x, stateStamped.pose().position().x());
      EXPECT_DOUBLE_EQ(y, stateStamped.pose().position().y());
      EXPECT_DOUBLE_EQ(theta, stateStamped.pose().yaw());
      EXPECT_EQ(time, stateStamped.stamp());
      EXPECT_DOUBLE_EQ(10.0, stateStamped.stamp().toSec());
    }

    // Test serialization
    {
      StateStamped* p = &stateStamped;
      const auto deserialized = deserialize<StateStamped*>(serialize(p));
      EXPECT_FALSE(deserialized == nullptr);
      EXPECT_EQ(stateStamped, *deserialized);
      EXPECT_EQ(typeid(stateStamped).name(), typeid(*deserialized).name());
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, DifferentialDriveSystemInputStamped) {

  try {

    // State
    const double transVel = 1.0;
    const double rotVel = 2.0;

    const Time time(10.0);
    DifferentialDriveSystemInput u(transVel, rotVel);
    DifferentialDriveSystemInputStamped us(u, time);

    // Test state
    EXPECT_DOUBLE_EQ(transVel, us.getVelTrans());
    EXPECT_DOUBLE_EQ(rotVel, us.getVelRot());
    EXPECT_DOUBLE_EQ(transVel, us.velocity()(0));
    EXPECT_DOUBLE_EQ(rotVel, us.velocity()(1));

    // Test time
    EXPECT_EQ(time, us.stamp());
    EXPECT_DOUBLE_EQ(10.0, us.stamp().toSec());

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, DifferentialDriveAgent) {

  try {

    const double transVel = 1.0;
    const double rotVel = 2.0;
    const double x = 0.1;
    const double y = 0.2;
    const double theta = 0.3;

    DifferentialDriveAgent::State state(transVel, rotVel, Pose2d(x, y, theta));
    const Time time(10.0);
    DifferentialDriveAgent::StateStamped ss(state, time);

    DifferentialDriveAgent agent;
    agent.setId(111);
    DifferentialDriveAgent::StateStamped& s = agent.stateStamped();
    s = ss;

    boost::shared_ptr<const ::fcl::Box> shape(new ::fcl::Box(::fcl::Vec3f(1.0, 2.0, 3.0)));
    agent.setCollisionGeometry(shape);

    EXPECT_DOUBLE_EQ(x, agent.stateStamped().pose().position().x());
    EXPECT_DOUBLE_EQ(y, agent.stateStamped().pose().position().y());
    EXPECT_DOUBLE_EQ(theta, agent.stateStamped().pose().yaw());
    EXPECT_DOUBLE_EQ(transVel, agent.stateStamped().getVelTrans());
    EXPECT_DOUBLE_EQ(rotVel, agent.stateStamped().getVelRot());

    EXPECT_EQ(time, agent.stateStamped().stamp());
    EXPECT_DOUBLE_EQ(10.0, agent.stateStamped().stamp().toSec());

    EXPECT_EQ(shape, agent.getCollisionGeometry());
    EXPECT_EQ(agent.stateStamped().stamp().toSec(), agent.getWorkspaceTimeCollisionObject()->getTranslation()[2]);

    // Test serialization
    testSerialization(agent, *shape);

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(common_agents_TESTSUITE, HolonomicAgent) {

  try {

    const double vx = 1.0;
    const double vy = 2.0;
    const double x = 0.1;
    const double y = 0.2;
    const double theta = 0.3;

    HolonomicAgent::State state(vx, vy, Pose2d(x, y, theta));
    const Time time(10.0);
    HolonomicAgent::StateStamped ss(state, time);

    HolonomicAgent agent;
    agent.setId(111);
    HolonomicAgent::StateStamped& s = agent.stateStamped();
    s = ss;

    boost::shared_ptr<const ::fcl::Box> shape(new ::fcl::Box(::fcl::Vec3f(1.0, 2.0, 3.0)));
    agent.setCollisionGeometry(shape);

    EXPECT_DOUBLE_EQ(x, agent.stateStamped().pose().position().x());
    EXPECT_DOUBLE_EQ(y, agent.stateStamped().pose().position().y());
    EXPECT_DOUBLE_EQ(theta, agent.stateStamped().pose().yaw());
    EXPECT_DOUBLE_EQ(vx, agent.stateStamped().getVelX());
    EXPECT_DOUBLE_EQ(vy, agent.stateStamped().getVelY());

    EXPECT_EQ(time, agent.stateStamped().stamp());
    EXPECT_DOUBLE_EQ(10.0, agent.stateStamped().stamp().toSec());

    EXPECT_EQ(shape, agent.getCollisionGeometry());
    EXPECT_EQ(agent.stateStamped().stamp().toSec(), agent.getWorkspaceTimeCollisionObject()->getTranslation()[2]);

    const planning2d::DiscApproximation& da = agent.getDiscApproximation(1);
    EXPECT_EQ(1, da.getNumDiscs());
    EXPECT_DOUBLE_EQ(da.getRadius(0), hypot(0.5, 1.0));
    EXPECT_EQ(da.getPosition(0), state.pose().position());

    // Test serialization
    testSerialization(agent, *shape);

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}
