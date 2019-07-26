/*
 * VoronoiPy.cpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <boost/python.hpp>

#include <planner_algorithms/Voronoi.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace planning2d::algorithms;

boost::python::list voronoiPointsWrapper(const boost::python::list& ll)
{
  std::vector< Point2d<double> > points;
  points.reserve(len(ll));

  for (int i=0; i<len(ll); i++)
    points.push_back( boost::python::extract< Point2d<double> >(ll[i]) );

  std::vector< std::vector<Point2d<float>> > facets;
  voronoi(points, facets);

  // copy to out list
  boost::python::list out;
  for (const auto& facet : facets)
  {
    boost::python::list points;
    for (const auto& p : facet)
      points.append(p);
    out.append(points);
  }
  return out;
}


void exportVoronoi()
{
  def("voronoi", &voronoiPointsWrapper, "facets = voronoi(list points): Computes the Voronoi tesselation from a set of 2D points");
} /* void exportVoronoi() */
