#include <gtest/gtest.h>

#include <planner_algorithms/MapChangeDetection.hpp>

using namespace std;
using namespace planning2d;
using namespace planning2d::algorithms;

void testMapChangeDetectionResult(const Map<bool>& binary, const bool tagBoundary)
{
  for (auto it = ((const Map<bool>&)binary).begin(); it != binary.end(); ++it)
  {
    Map<bool>::Index index = binary.toIndex(it);
    if ((tagBoundary && binary.isMapBoundary(index)) || (1 <= index.x() && index.x() <= 3 && index.y() == 2))
      EXPECT_TRUE(*it) << "Failure at " << index;
    else
      EXPECT_FALSE(*it) << "Failure at " << index;
  }
}

TEST(planner_algorithms_TESTSUITE, MapChangeDetection)
{
  try
  {
    Map<int32_t> map(Pose2d(0.0, 0.0, 0.0), 0.1, Map<int32_t>::Size2d(5, 5), 0);
    map.block<5,3>(Map<int32_t>::Index(0,0)).array() += 1; // set lower 3 rows to 1

    Map<bool> binary(map.getOrigin(), map.resolution(), map.sizeInCells());
    changeDetection(map, binary, true);
    testMapChangeDetectionResult(binary, true);
    changeDetection(map, binary, false);
    testMapChangeDetectionResult(binary, false);
  }
  catch (const exception& e)
  {
    FAIL() << e.what();
  }
}

