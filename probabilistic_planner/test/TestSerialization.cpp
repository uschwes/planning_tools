#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/make_shared.hpp>

#include <sm/logging.hpp>

#include <planner_interfaces/Time.hpp>
#include <planner_interfaces/TestAgent.hpp>

#include <probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp>
#include <probabilistic_planner/state_representation/Trajectory.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>
#include "Support.hpp"

using namespace std;
using namespace planning2d;
using namespace prob_planner;

template <typename Archive>
void registerTypes(Archive & /*a*/) { }

template <typename Archive, typename MyType, typename ... MyTypes>
void registerTypes(Archive & a) {
  a.template register_type<MyType>();
  registerTypes<Archive, MyTypes...>(a);
}

template <class T, typename ... Types>
string serialize(const T& data) {
  ostringstream ss;
  boost::archive::binary_oarchive oa(ss);
  registerTypes< boost::archive::binary_oarchive, Types...>(oa);
  oa << data;
  return ss.str();
}

template <class T, typename ... Types>
void deserialize(const std::string& str, T& obj) {
    stringstream s;
    s << str;
    boost::archive::binary_iarchive ia(s);
    registerTypes< boost::archive::binary_iarchive, Types...>(ia);
    ia >> obj;
}

template <class T, typename ... Types>
T deserialize(const std::string& str) {
    T obj;
    deserialize<T, Types...>(str, obj);
    return obj;
}

template <class T, typename ... Types>
T serializeDeserialize(const T& obj) {
  return deserialize<T, Types...>(serialize<T, Types...>(obj));
}

template <class T, typename ... Types>
void serializeDeserialize(const T& in, T& out) {
  deserialize<T, Types...>(serialize<T, Types...>(in), out);
}


struct TestCacheEntry : public cache::CacheEntryInterface, public StampedType
{
  TestCacheEntry() : StampedType(Time(100.0)) { }
  virtual ~TestCacheEntry() { }
  bool isValid() const { return true; }
  template<class Archive>
  inline void serialize(Archive & ar, const unsigned int /*version*/) {
    boost::serialization::void_cast_register<TestCacheEntry, cache::CacheEntryInterface>();
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(cache::CacheEntryInterface);
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(StampedType);
  }
};

void expectEqual(const Trajectory& t0, const Trajectory& t1) {
  ASSERT_EQ(t0.getSpline().numDesignVariables(), t1.getSpline().numDesignVariables());
  for (size_t i=0; i<t0.getSpline().numDesignVariables(); i++) {
    const auto& dv0 = *(t0.getSpline().designVariable(i));
    const auto& dv1 = *(t1.getSpline().designVariable(i));
    EXPECT_EQ(dv0.blockIndex(), dv1.blockIndex());
    EXPECT_EQ(dv0.columnBase(), dv1.columnBase());
    EXPECT_EQ(dv0.isActive(), dv1.isActive());
    EXPECT_EQ(dv0.isMarginalized(), dv1.isMarginalized());
    EXPECT_DOUBLE_EQ(dv0.scaling(), dv1.scaling());
  }
  for (Time t = t0.getStartTime(); t <= t0.getFinalTime(); t += Duration(0.1))
    EXPECT_EQ(t0.getPosition2d(t), t1.getPosition2d(t));
}

void expectEqual(const Agent& a0, const Agent& a1) {
  EXPECT_EQ(a0.getId(), a1.getId());
  EXPECT_EQ(a0.isControllable(), a1.isControllable());
}

void expectEqual(const SceneSnapshot& snap0, const SceneSnapshot& snap1) {
  EXPECT_EQ(snap0.stamp(), snap1.stamp());
  EXPECT_EQ(snap0.getOccupancyGrid(), snap1.getOccupancyGrid());
  EXPECT_EQ(snap0.objectContainer(), snap1.objectContainer());
}

void expectEqual(const OptAgent& a0, const OptAgent& a1) {
  expectEqual(a0.trajectory(), a1.trajectory());
  ASSERT_NE(a0.getAgent(), nullptr);
  ASSERT_NE(a1.getAgent(), nullptr);
  expectEqual(*a0.getAgent(), *a1.getAgent());
}

void expectEqual(const ContinuousScene& s0, const ContinuousScene& s1) {
  EXPECT_EQ(s0.numberOfAgents(), s1.numberOfAgents());
  EXPECT_EQ(s0.numberOfObservations(), s1.numberOfObservations());
  ContinuousScene::AgentContainer::const_iterator it0 = s0.getOptAgentContainer().begin();
  ContinuousScene::AgentContainer::const_iterator it1 = s1.getOptAgentContainer().begin();
  for (; it0 != s0.getOptAgentContainer().end(); ++it0, ++it1) {
    EXPECT_EQ(it0->first, it1->first);
    expectEqual(it0->second, it1->second);
  }
  for (std::size_t i=0; i<s0.numberOfObservations(); ++i)
    expectEqual(*s0.getObservations()[i], *s1.getObservations()[i]);
}


TEST(probabilistic_planner_TESTSUITE, Serialization) {

  try {

    Trajectory::Ptr trajectory(new Trajectory());
    trajectory->initZeroSpline(Time(0.0), Time(1.0), 10);
    trajectory->activateAllDesignVariables(true);

    TestAgent::Ptr agent = TestAgent::Ptr(new TestAgent(1));
    agent->stateStamped().state() = Eigen::Vector2d(1., 2.);
    agent->stateStamped().stamp() = Time(10.0);
    agent->stateStamped().pose() = Pose2d(0.1, 0.2, 0.3);

    auto snapshot = boost::make_shared<SceneSnapshot>(Time(2.0));
    snapshot->addObject(agent->getId(), StateWithUncertainty(agent->stateStamped(), Eigen::Matrix<double,5,5>::Identity()));
    snapshot->cache().get("test")->second = boost::shared_ptr<TestCacheEntry>(new TestCacheEntry());

    OptAgent optAgent(agent, trajectory, OptAgentType::UNKNOWN);

    ContinuousScene scene;
    scene.addOptAgent(optAgent);
    scene.addObservation(snapshot);

    // Test trajectory
    {
      Trajectory::Ptr deserialized = serializeDeserialize(trajectory);
      ASSERT_NE(deserialized, nullptr);
      expectEqual(*trajectory, *deserialized);
    }

    // Test SceneSnapshot
    {
      SceneSnapshot deserialized(Time(0.0));
      serializeDeserialize<SceneSnapshot, TestCacheEntry>(*snapshot, deserialized);
      expectEqual(*snapshot, deserialized);
    }

    // Test Agent
    {
      TestAgent::Ptr deserialized = serializeDeserialize<TestAgent::Ptr, TestAgent>(agent);
      ASSERT_NE(deserialized, nullptr);
    }

    // Test OptAgent
    {
      OptAgent deserialized(planning2d::Agent::ConstPtr(new planning2d::TestAgent()), Trajectory::Ptr(new Trajectory()), OptAgentType::UNKNOWN);
      serializeDeserialize<OptAgent, planning2d::TestAgent>(optAgent, deserialized);
      EXPECT_EQ(optAgent.getType(), deserialized.getType());
      ASSERT_NE(deserialized.getAgent(), nullptr);
      expectEqual(*optAgent.getAgent(), *deserialized.getAgent());
      expectEqual(optAgent.trajectory(), deserialized.trajectory());
    }

    // Test Scene
    {
      ContinuousScene deserialized;
      serializeDeserialize<ContinuousScene, planning2d::TestAgent, TestCacheEntry>(scene, deserialized);
      expectEqual(scene, deserialized);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
