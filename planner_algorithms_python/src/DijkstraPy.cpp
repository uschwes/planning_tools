/*
 * DijkstraPy.cpp
 *
 *  Created on: April 7, 2016
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <planner_interfaces/OccupancyGrid.hpp>
#include <planner_algorithms/Dijkstra.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace planning2d::algorithms;

template <typename Connectivity, typename Scalar>
boost::python::list extractBestPathWrapper(Connectivity neighborsFcn, const Map<Scalar>& costs, const OccupancyGrid::Index& source) {
  std::vector<OccupancyGrid::Index> path;
  dijkstra::extractBestPath(neighborsFcn, costs, source, path);
  boost::python::list ll;
  for (const auto& p : path) { ll.append(p); }
  return ll;
}

template <typename Connectivity, typename Scalar>
void exportDijkstraAndConnectivity(const std::string& connectivityName, const std::string& dijkstraName)
{
  class_<Connectivity>(connectivityName.c_str(), init<const OccupancyGrid&>( (connectivityName + "(OccupancyGrid grid): Constructor").c_str()))
  ;

  typedef Dijkstra<OccupancyGrid::Index, Map<Scalar>, Connectivity > DJKSTR;
  class_<DJKSTR>(dijkstraName.c_str(), init<OccupancyGrid::Index, Connectivity&, Map<Scalar>&, optional<OccupancyGrid::Index const * const> >(
      (dijkstraName + "(MapIndex start, " + connectivityName + " connectivity, Map& costOut, MapIndex goal[optional]): Constructor").c_str()))
    .def("step", &DJKSTR::step, "Execute one step meaning expand the lowest cost node on the heap")
    .def("run", &DJKSTR::run, "Run until end")
  ;

  def("extractBestPath", &extractBestPathWrapper<Connectivity, Scalar>);
}

void exportDijkstra()
{
  typedef double Scalar;
  typedef dijkstra::OccupancyGridConnectivity4d<Scalar> Connectivity4d;
  typedef dijkstra::OccupancyGridConnectivity8d<Scalar> Connectivity8d;

  exportDijkstraAndConnectivity<Connectivity4d, Scalar>("Connectivity4d", "DijkstraC4D");
  exportDijkstraAndConnectivity<Connectivity8d, Scalar>("Connectivity8d", "DijkstraC8D");

} /* void exportDijkstra() */
