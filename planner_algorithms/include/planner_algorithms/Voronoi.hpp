/*
 * Voronoi.hpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_VORONOI_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_VORONOI_HPP_


namespace planning2d
{
namespace algorithms
{
/**
 * Computes the Voronoi tesselation of the map
 * @param[in] points List of points (Voronoi centers)
 * @param[out] facets List of polygons describing the Voronoi facets
 */
template <typename ContainerIn, typename ContainerOut>
void voronoi(const ContainerIn& points, ContainerOut& facets);

} /* namespace algorithms */
} /* planning2d */

#include "implementation/VoronoiImpl.hpp"

#endif /* INCLUDE_PLANNER_ALGORITHMS_IMPLEMENTATION_VORONOI_HPP_ */
