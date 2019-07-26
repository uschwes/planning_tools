#include <gtest/gtest.h>
#include <sm/random.hpp>
#include <planner_algorithms/Voronoi.hpp>


TEST(planner_algorithms_TESTSUITE, PointsVoronoi)
{
  using namespace std;
  using namespace planning2d;
  using namespace planning2d::algorithms;

  try
  {
    const float distFromCenter = 1.f;
    const Point2d<float> center(Eigen::Vector2f::Random());

    // sample random points on a circle at distFromCenter
    const std::size_t numPoints = 3;
    vector< Point2d<float> > points;
    points.reserve(numPoints);
    for (std::size_t i=0; i<numPoints; ++i)
    {
      const float angle = sm::random::randLU(0., planning2d::PI);
      points.emplace_back(center + Point2d<float>(cos(angle), sin(angle)).cwiseProduct(distFromCenter));
    }

    std::vector< std::vector<Point2d<float> > > facets;
    voronoi(points, facets);

    ASSERT_EQ(numPoints, facets.size());

    for (std::size_t i=0; i<numPoints; ++i)
    {
      ASSERT_EQ(4, facets[i].size());

      // One of the vertices of the facet should be the center
      bool centerFound = false;
      for (const auto& p : facets[i])
        centerFound |= (p - center).squaredNorm() < 1e-3;

      ASSERT_TRUE(centerFound);
    }
  }
  catch (const exception& e)
  {
    FAIL() << e.what();
  }
}
