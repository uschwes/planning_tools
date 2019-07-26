#include <gtest/gtest.h>

#include <planner_interfaces/Time.hpp>

#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>

using namespace std;
using namespace planning2d;
using namespace prob_planner;


TEST(SceneSnapshot, SceneSnapshot) {

  typedef std::map<planning2d::Id, StateWithUncertainty> StateMap;

  try {

    // Test constructors
    {
      Time stamp( (double)3.013570234 );
      {
        SceneSnapshot meas(stamp);
        EXPECT_EQ(stamp, meas.stamp());
        EXPECT_EQ(0, meas.objectContainer().size());
        EXPECT_FALSE(meas.getOccupancyGrid());
      }
      {
        SceneSnapshot meas(stamp, 0, StateWithUncertainty());
        EXPECT_EQ(stamp, meas.stamp());
        EXPECT_EQ(1, meas.objectContainer().size());
        EXPECT_NO_THROW(meas.getObject(0));
        EXPECT_FALSE(meas.getOccupancyGrid());
      }
      {
        SceneSnapshot meas(stamp, OccupancyGrid());
        EXPECT_EQ(stamp, meas.stamp());
        EXPECT_EQ(0, meas.objectContainer().size());
        EXPECT_TRUE(meas.getOccupancyGrid());
      }
      {
        SceneSnapshot meas(stamp, {std::make_pair(0, StateWithUncertainty()), std::make_pair(1, StateWithUncertainty())});
        EXPECT_EQ(stamp, meas.stamp());
        EXPECT_EQ(2, meas.objectContainer().size());
        EXPECT_NO_THROW(meas.getObject(0));
        EXPECT_NO_THROW(meas.getObject(1));
        EXPECT_FALSE(meas.getOccupancyGrid());
      }
    }

    Time stamp( (double)3.013570234 );
    SceneSnapshot meas(stamp);
    EXPECT_EQ(stamp, meas.stamp());

    StateMap stateMap;
    {
      const Id id = 0;
      State s(0);
      s.pose() = Pose2d(1.0, 2.0, 3.0);
      EXPECT_ANY_THROW(StateWithUncertainty su(s, Eigen::Matrix2d::Identity()*0.3)); // because dimensions do not match
      StateWithUncertainty su(s, Eigen::Matrix3d::Identity()*0.3);
      stateMap[id] = su;
    }

    {
      const Id id = 4;
      State s(0);
      s.pose() = Pose2d(4.0, 5.0, 6.0);
      StateWithUncertainty su(s, Eigen::Matrix3d::Identity()*0.2);
      stateMap[id] = su;
    }

    meas.objectContainer() = stateMap;
    EXPECT_EQ(2UL, meas.objectContainer().size());

    {
      const Id id = 8;
      State s(0);
      s.pose() = Pose2d(4.0, 5.0, 6.0);
      StateWithUncertainty su(s, Eigen::Matrix3d::Identity()*0.5);
      stateMap[id] = su;
      meas.addObject(id, su);
    }

    OccupancyGrid grid(Pose2d(10.0, 15.0, 0.0), 3.0);
    grid.matrix() = OccupancyGrid::Matrix::Constant(100, 100, OccupancyValue::FREE);
    meas.setOccupancyGrid(grid);

    // Tests
    EXPECT_EQ(3, meas.objectContainer().size());
    EXPECT_DOUBLE_EQ(4.0, meas.objectContainer()[4].x());
    EXPECT_DOUBLE_EQ(5.0, meas.objectContainer()[4].y());
    EXPECT_DOUBLE_EQ(6.0, meas.objectContainer()[4].yaw());
    EXPECT_EQ(Eigen::Matrix3d::Identity()*0.2, meas.objectContainer()[4].invCov());
    EXPECT_EQ(Eigen::Matrix3d::Identity()*0.2, meas.getObject(4).invCov());
    EXPECT_TRUE(meas.getOccupancyGrid());
    EXPECT_DOUBLE_EQ(10.0, meas.getOccupancyGrid().get().getOrigin().x());
    EXPECT_DOUBLE_EQ(15.0, meas.getOccupancyGrid().get().getOrigin().y());
    EXPECT_DOUBLE_EQ(0.0, meas.getOccupancyGrid().get().getOrigin().yaw());
    EXPECT_DOUBLE_EQ(3.0, meas.getOccupancyGrid().get().resolution());

    // test cacheable version
    {

      struct TestCacheEntryDistanceTransform : public cache::CacheEntryInterface
      {
        TestCacheEntryDistanceTransform() { }
        TestCacheEntryDistanceTransform(const OccupancyGrid& grid)
            : distanceTransform(grid.getOrigin(), grid.resolution(), grid.sizeInCells())
        {
        }
        virtual ~TestCacheEntryDistanceTransform() { }
        Map<float> distanceTransform;
        bool isValid() const override { return true; }
      };

      auto& entry = meas.cache().get("distance_transform_occupance_grid")->second;
      EXPECT_TRUE(entry == nullptr);
      entry = boost::shared_ptr<TestCacheEntryDistanceTransform>(new TestCacheEntryDistanceTransform(meas.getOccupancyGrid().get()));

      EXPECT_TRUE(meas.cache().get("distance_transform_occupance_grid")->second != nullptr);
      const auto& it = meas.cache().getConst("distance_transform_occupance_grid");
      ASSERT_TRUE(meas.cache().isValid(it));
      const auto& dtCache = boost::dynamic_pointer_cast<TestCacheEntryDistanceTransform>(it->second);
      ASSERT_TRUE(dtCache != nullptr);
      const auto& grid = meas.getOccupancyGrid().get();
      EXPECT_EQ(grid.getOrigin(), dtCache->distanceTransform.getOrigin());
      EXPECT_EQ(grid.resolution(), dtCache->distanceTransform.resolution());
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
