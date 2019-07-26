#include <gtest/gtest.h>

#include <fstream>
#include <memory>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

#include <sm/BoostPropertyTree.hpp>
#include <sm/logging.hpp>

#include "planner_interfaces/PlannerInterfaceDebugWrapper.hpp"
#include "planner_interfaces/TestAgent.hpp"
#include "TestPlanner.hpp"

using namespace std;
using namespace planning2d;

template <class T>
string serialize(const T& data) {
  ostringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa.register_type<TestAgent::StateStamped>();
  oa << data;
  return ss.str();
}

template <class T>
T deserialize(const std::string& str) {
    stringstream s;
    s << str;

    boost::archive::binary_iarchive ia(s);
    ia.register_type<TestAgent::StateStamped>();
    T obj;
    ia >> obj;

    return obj;
}

TEST(planner_interfaces_TESTSUITE, SerializePolymorph) {

  Pose2d pose(1.0,2.0,3.0);
  TestAgent::StateStamped testStateStamped(planning2d::StateStamped(State(State::T::Constant(3, 1, 1.0), pose), Time(100L)));
  StateStamped* p = &testStateStamped;

  auto deserialized = deserialize<StateStamped*>(serialize(p));

  EXPECT_FALSE(NULL == deserialized);
  EXPECT_EQ(testStateStamped, *deserialized);

}


TEST(planner_interfaces_TESTSUITE, PlannerInterfaceDebugWrapperTest) {

  class WrapperTestPlanner : public TargetPlannerInterface {
   public:
    ReturnCode computePlan(const Time& currentTime, StateInputTrajectory& /* stateInputSequence */) override {
      _computePlanLastCurrentTime = currentTime;
      return RETCODE_OK;
    }

    ReturnCode callbackOccupancyGrid(const OccupancyGridStamped& grid) override {
      _occupancyGridForCallback = grid;
      return RETCODE_OK;
    }
    ReturnCode callbackCurrentState(const planning2d::StateStamped & state) override {
      _currentStateForCallback = state;
      return RETCODE_OK;
    }
//    virtual ReturnCode callbackReferencePath(const Path& path);
    ReturnCode callbackDynamicObjects(const std::vector<Agent::ConstPtr>& dynamicObjects) override {
      _dynamicObjectsForCallback = dynamicObjects;
      return RETCODE_OK;
    }

    ReturnCode callbackSetTarget(const planning2d::Pose2d& target, bool isFinal)  {
      _targetForCallback = target;
      _isFinalForCallback = isFinal;
      return RETCODE_OK;
    }

    Time _computePlanLastCurrentTime;
    OccupancyGridStamped _occupancyGridForCallback;
    TestAgent::StateStamped _currentStateForCallback;
    std::vector<Agent::ConstPtr> _dynamicObjectsForCallback;
    planning2d::Pose2d _targetForCallback;
    bool _isFinalForCallback;
  };

  std::string folderPath = "tmp_" + planning2d::time::getCurrentTime().toDateString("%Y%m%d_%H%M%S%F");
  // Make sure we have a new or empty folder
  SM_ASSERT_TRUE(planning2d::RuntimeException, !boost::filesystem::exists(folderPath) || boost::filesystem::is_empty(folderPath), "");

  try {
    auto planner = boost::make_shared<WrapperTestPlanner>();

    PlannerInterfaceDebugWrapper<TestAgent> plannerWrapper(planner, folderPath);
    plannerWrapper.registerTypes<TestAgent::StateStamped, TestAgent>();

    TestAgent::StateStamped egoState;
    egoState.pose() = Pose2d(10.0, 10.0, 0.0);
    egoState.linearVelocity() = 0.0;
    egoState.stamp() = time::getCurrentTime();

    TestAgent dynamicAgentTemplate(0);

    StateStamped s(State(Eigen::MatrixXd::Ones(1,1), Pose2d(3.0, 4.0, 0.5)), Time((double)2.0));
    StateStamped t(State(Eigen::MatrixXd::Ones(1,1), Pose2d(6.0, 4.0, 0.5)), Time((double)4.0));
    dynamicAgentTemplate.stateStamped() = s;

    std::vector<Agent::ConstPtr> dynamicObjects;
    for (int i=0; i<10; ++i)
      dynamicObjects.push_back(boost::make_shared<TestAgent>(dynamicAgentTemplate));

    Path referencePath(1);
    referencePath[0] = Pose2d(20.0, 20.0, 0.0);

    // create occupancy grid
    OccupancyGridStamped grid(Pose2d(5.0, 5.0, 0.0), 0.1, OccupancyGrid::Matrix::Constant(100,100, OccupancyValue::FREE), Time(0L));

    // target value
    Pose2d targetForEgoAgent(10.0, 10.0, 0.0);

    // call all the callbacks
    plannerWrapper.callbackCurrentState(egoState);
    plannerWrapper.callbackOccupancyGrid(grid);
    plannerWrapper.callbackDynamicObjects(dynamicObjects);
    plannerWrapper.callbackSetTarget(targetForEgoAgent, true);

    // compute solution trajectory
    StateInputTrajectory solutionTrajectory;
    planning2d::Time expectedComputePlanCurrentTime = time::getCurrentTime();
    plannerWrapper.computePlan(expectedComputePlanCurrentTime, solutionTrajectory);

    // initialize to dummy values
    planner->_computePlanLastCurrentTime = -1.0;
    planner->_occupancyGridForCallback = OccupancyGridStamped(Pose2d(1000.0, 1000.0, 0.0), 1000.0, Time(0L));
    planner->_currentStateForCallback = StateStamped();
    planner->_isFinalForCallback = false;
    planner->_targetForCallback = Pose2d(-1000.0, -1000.0, -1000.0);

    plannerWrapper.playback(0, 1);
    EXPECT_EQ(expectedComputePlanCurrentTime, planner->_computePlanLastCurrentTime);
    EXPECT_EQ(grid, planner->_occupancyGridForCallback);
    EXPECT_EQ(egoState, planner->_currentStateForCallback);
    EXPECT_EQ(true, planner->_isFinalForCallback);
    EXPECT_EQ(targetForEgoAgent, planner->_targetForCallback);

    int i = 0;
    for(auto a : planner->_dynamicObjectsForCallback){
      EXPECT_EQ(dynamicObjects[i]->getId(), a->getId());
      EXPECT_EQ(dynamicObjects[i]->stateStamped(), a->stateStamped());
      i++;
    }

    // cleanup
    if (boost::filesystem::exists(folderPath))
      boost::filesystem::remove_all(folderPath);

  } catch (const std::exception& e) {
    // cleanup
    if (boost::filesystem::exists(folderPath))
      boost::filesystem::remove_all(folderPath);
    FAIL() << e.what();
  }

}
