/*
 * Voronoi.cpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

// self
#include <planner_algorithms/Voronoi.hpp>

namespace planning2d
{
namespace algorithms
{

template void voronoi< std::vector<Point2d<int64_t> >, std::vector< std::vector< Point2d<float> > > >(const std::vector<Point2d<int64_t> >& points,
                                                                                                      std::vector< std::vector< Point2d<float> > >& facets);
template void voronoi< std::vector<Point2d<float> >, std::vector< std::vector< Point2d<float> > > >(const std::vector<Point2d<float> >& points,
                                                                                                    std::vector< std::vector< Point2d<float> > >& facets);
template void voronoi< std::vector<Point2d<double> >, std::vector< std::vector< Point2d<float> > > >(const std::vector<Point2d<double> >& points,
                                                                                                     std::vector< std::vector< Point2d<float> > >& facets);

} /* namespace algorithms */
} /* namespace planning2d */
