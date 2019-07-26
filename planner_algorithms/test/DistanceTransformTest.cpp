/*
 * DistanceTransformTest.cpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <gtest/gtest.h>

#include <planner_algorithms/DistanceTransform.hpp>

using namespace std;
using namespace planning2d;
using namespace planning2d::algorithms;


TEST(planner_algorithms_TESTSUITE, OccupancyGridDistanceTransform)
{
  try
  {
    OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(100, 100), OccupancyValue::FREE);

    // Test distance transform
    OccupancyGrid::Index oidx(0,80);
    grid(oidx) = OccupancyValue::OCCUPIED;

    Map<float> dt(grid.getOrigin(), grid.resolution(), grid.sizeInCells());
    distanceTransform(grid, dt);

    // we have one occupied pixel oidx
    Position2d op = dt.toPosition(oidx);
    for (size_t ix = 0; ix < dt.sizeInCells().x(); ++ix) {
      for (size_t iy = 0; iy < dt.sizeInCells().y(); ++iy) {
        Map<float>::Index idx(ix, iy);
        Position2d pos = dt.toPosition(idx);
        EXPECT_NEAR(dt.at(idx), (pos - op).norm(), 5e-2) <<
            "Failure at index " << idx << " and position " << pos;
      }
    }
  }
  catch (const exception& e)
  {
    FAIL() << e.what();
  }
}

TEST(planner_algorithms_TESTSUITE, OccupancyGridSignedDistanceTransform)
{
  try
  {
    OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(100, 100), OccupancyValue::FREE);
    grid.block<3,3>(OccupancyGrid::Index(80,80)).array().setConstant(OccupancyValue::OCCUPIED);

    Map<float> dt(grid.getOrigin(), grid.resolution(), grid.sizeInCells());
    signedDistanceTransform(grid, dt);

    for (size_t ix = 80; ix < 83; ix++) {
      for (size_t iy = 80; iy < 83; iy++) {
        Map<float>::Index idx(ix, iy);
        Position2d pos = dt.toPosition(idx);
        if (idx != OccupancyGrid::Index(81,81))
          EXPECT_NEAR(dt.at(idx), 0.0, 1e-6) <<
                      "Failure at index " << idx << " and position " << pos;
        else
          EXPECT_NEAR(dt.at(idx), -dt.resolution(), 1e-6) <<
                      "Failure at index " << idx << " and position " << pos;
      }
    }
  }
  catch (const exception& e)
  {
    FAIL() << e.what();
  }
}
