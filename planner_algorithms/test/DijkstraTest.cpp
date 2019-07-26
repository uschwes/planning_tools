#include <gtest/gtest.h>

#include <planner_interfaces/OccupancyGrid.hpp>
#include <planner_algorithms/Dijkstra.hpp>

TEST(planner_algorithms_TESTSUITE, Dijkstra)
{
  using namespace std;
  using namespace planning2d;
  using namespace planning2d::algorithms;

  try
  {
    typedef double Scalar;

    OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(10, 10), OccupancyValue::FREE);

    OccupancyGrid::Index oidx(5,5);
    grid(oidx) = OccupancyValue::OCCUPIED;

    dijkstra::OccupancyGridConnectivity4d<Scalar> neighbors(grid);

    OccupancyGrid::Index source(0,0);

    Map<Scalar> costs(grid.getOrigin(), grid.resolution(), grid.sizeInCells(), std::numeric_limits<Scalar>::infinity());

    auto djkstr = make_dijkstra(source, neighbors, costs);
    while(!djkstr.step()) { } // run until finished
    EXPECT_TRUE(djkstr.step());
    EXPECT_TRUE(djkstr.run());

    // Check costmap
    for (auto it = costs.begin(); it != costs.end(); ++it) {
      auto index = costs.toIndex(it);
      if (index == oidx)
        EXPECT_DOUBLE_EQ(std::numeric_limits<Scalar>::infinity(), *it); // occupied cells shall have infinite costs
      else
        EXPECT_DOUBLE_EQ(grid.resolution() * index.sum(), *it); // Manhattan distance
    }

    // Extract path
    OccupancyGrid::Index goal(9,9);
    std::vector<OccupancyGrid::Index> path;
    dijkstra::extractBestPath(neighbors, costs, goal, path);
    EXPECT_EQ(19, path.size());
    for (auto& idx : path) {
      const auto pos = grid.toPosition(idx);
      EXPECT_DOUBLE_EQ(pos.x() + pos.y(), costs(idx));
    }
  }
  catch (const exception& e)
  {
    FAIL() << e.what();
  }
}
