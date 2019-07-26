/*
 * MapChangeDetectionPy.cpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <boost/python.hpp>

#include <planner_algorithms/MapChangeDetection.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace planning2d::algorithms;

template <typename T>
Map<bool> mapChangeDetectionWrapper(const Map<T>& map, const bool tagBoundary = true)
{
  Map<bool> binary(map.getOrigin(), map.resolution(), map.sizeInCells());
  changeDetection(map, binary, tagBoundary);
  return binary;
}
BOOST_PYTHON_FUNCTION_OVERLOADS(changeDetectionWrapper_overloads, mapChangeDetectionWrapper, 1, 2);


void exportMapChangeDetection()
{
  def("mapChangeDetection", &mapChangeDetectionWrapper<uint8_t>, changeDetectionWrapper_overloads());
  def("mapChangeDetection", &mapChangeDetectionWrapper<float>, changeDetectionWrapper_overloads());
  def("mapChangeDetection", &mapChangeDetectionWrapper<double>, changeDetectionWrapper_overloads());
} /* void exportDijkstra() */
