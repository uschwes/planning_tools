#include <gtest/gtest.h>
#include <sm/eigen/gtest.hpp>

#include <planner_interfaces/OccupancyGrid.hpp>
#include <planner_algorithms/Dijkstra.hpp>

TEST(DijkstraProfilingTests, DISABLED_ProfileDijkstra)
{
  using namespace std;
  using namespace planning2d;
  using namespace planning2d::algorithms;

  try
  {
    typedef double Scalar;
    OccupancyGrid::Index source(0,0);
    OccupancyGrid grid(Pose2d(0.0, 0.0, 0.0), 0.1, OccupancyGrid::Size2d(250, 250), OccupancyValue::FREE);

    // 4-Connectivity
    {
      dijkstra::OccupancyGridConnectivity4d<Scalar> neighbors(grid);
      Map<Scalar> costs(grid.getOrigin(), grid.resolution(), grid.sizeInCells(), std::numeric_limits<Scalar>::infinity());

      auto djkstr = make_dijkstra<Scalar, std::set>(source, neighbors, costs);
      Time start = time::getCurrentTime();
      djkstr.run();
      SM_INFO_STREAM("4-Connectivity: Dijkstra algorithm on " << grid.sizeInCells() << " took " << (time::getCurrentTime() - start).format(time::Formatter(time::MILLISEC)));
    }
    // 8-Connectivity
    {
      dijkstra::OccupancyGridConnectivity8d<Scalar> neighbors(grid);
      Map<Scalar> costs(grid.getOrigin(), grid.resolution(), grid.sizeInCells(), std::numeric_limits<Scalar>::infinity());

      auto djkstr = make_dijkstra<Scalar, std::set>(source, neighbors, costs);
      Time start = time::getCurrentTime();
      djkstr.run();
      SM_INFO_STREAM("8-Connectivity: Dijkstra algorithm on " << grid.sizeInCells() << " took " << (time::getCurrentTime() - start).format(time::Formatter(time::MILLISEC)));
    }
  }
  catch (const exception& e)
  {
    FAIL() << e.what();
  }
}
