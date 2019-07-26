/*
 * VoronoiImpl.hpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_VORONOIIMPL_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_VORONOIIMPL_HPP_

// standard
#include <vector>

// Eigen
#include <Eigen/Dense>

// opencv2
#include <opencv2/imgproc/imgproc.hpp>

// planner_interfaces
#include <planner_interfaces/Position2d.hpp>

// self
#include <planner_algorithms/Support.hpp>

namespace planning2d
{
namespace algorithms
{

template <typename ContainerIn, typename ContainerOut>
void voronoi(const ContainerIn& points, ContainerOut& facets)
{
  Timer timer("voronoi -- point list", false);

  // Create an instance of Subdiv2D
  cv::Rect rect(std::numeric_limits<int>::min()/2, std::numeric_limits<int>::min()/2, std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
  cv::Subdiv2D subdiv(rect);

  // Insert points into subdiv
  for (const auto& p : points)
    subdiv.insert(cv::Point2f((float)p.x(), (float)p.y()));

  std::vector< std::vector<cv::Point2f> > facetsTmp;
  std::vector<cv::Point2f> centers;
  facets.reserve(points.size());
  centers.reserve(points.size());

  subdiv.getVoronoiFacetList(std::vector<int>(), facetsTmp, centers);

  // copy to output
  facets.clear();
  facets.reserve(facetsTmp.size());
  for (const auto& facet : facetsTmp)
  {
    facets.resize(facets.size() + 1);
    for (const auto& p : facet)
      facets.back().push_back(Point2d<float>(p.x, p.y));
  }
}

extern template void voronoi< std::vector<Point2d<int64_t> >, std::vector< std::vector< Point2d<float> > > >(const std::vector<Point2d<int64_t> >& points,
                                                                                                             std::vector< std::vector< Point2d<float> > >& facets);
extern template void voronoi< std::vector<Point2d<float> >, std::vector< std::vector< Point2d<float> > > >(const std::vector<Point2d<float> >& points,
                                                                                                           std::vector< std::vector< Point2d<float> > >& facets);
extern template void voronoi< std::vector<Point2d<double> >, std::vector< std::vector< Point2d<float> > > >(const std::vector<Point2d<double> >& points,
                                                                                                            std::vector< std::vector< Point2d<float> > >& facets);

} /* namespace algorithms */
} /* planning2d */


#endif /* INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_VORONOIIMPL_HPP_ */
