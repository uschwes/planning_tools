/*
 * MapChangeDetection.hpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_MAPCHANGEDETECTION_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_MAPCHANGEDETECTION_HPP_


#include <cstdint>
#include <planner_interfaces/OccupancyGrid.hpp>


namespace planning2d
{
namespace algorithms
{

/**
 * Computes a binary representation of the input map, labeling cells with at least one differing cell
 * (4-connected) as true.
 * @param[in] map Input map
 * @param[out] binary Output map
 * @param[in] tagBoundary Specifies whether to set map boundary cells to true or not
 * @tparam Scalar Scalar type of data, has to support != operator
 */
template <typename Scalar>
void changeDetection(const Map<Scalar>& map, Map<bool>& binary, bool tagBoundary = true);

} /* namespace algorithms */
} /* namespace planning2d */

#include "implementation/MapChangeDetectionImpl.hpp"

#endif /* INCLUDE_PLANNER_ALGORITHMS_MAPCHANGEDETECTION_HPP_ */
