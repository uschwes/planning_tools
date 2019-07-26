/*
 * DistanceTransform.hpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PLANNER_ALGORITHMS_DISTANCETRANSFORM_HPP_
#define INCLUDE_PLANNER_ALGORITHMS_DISTANCETRANSFORM_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <planner_interfaces/OccupancyGrid.hpp>

namespace planning2d
{
namespace algorithms
{

/**
 * Computes a distance transform map. UNKNOWN cells will be considered free.
 * @param[in] in The original map. Occupied cells should have value 0.
 *               Note that the map data will be casted to unsigned 8-bit integer, so check your data range.
 * @param[out] out The distance transformed map. The map is of type float since the opencv algorithm used operates with float accuracy.
 * @param[out] labels Optional discretized Voronoi regions.
 * @param[in] distanceType Type of the distance measure
 * @param[in] maskSize Size of the distance computation kernel
 */
template <typename T>
void distanceTransform(const Map<T>& in, Map<float>& out, Map<int32_t>* labels = nullptr, int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE);

/**
 * Computes a signed distance transform map. UNKNOWN cells will be considered free.
 * @param[in] in The original map. Occupied cells should have value 0.
 *               Note that the map data will be casted to unsigned 8-bit integer, so check your data range.
 * @param[out] out The signed distance transformed map. The map is of type float since the opencv algorithm used operates with float accuracy.
 * @param[out] labels Optional discretized Voronoi regions.
 * @param[in] distanceType Type of the distance measure
 * @param[in] maskSize Size of the distance computation kernel
 */
template <typename T>
void signedDistanceTransform(const Map<T>& in, Map<float>& out, Map<int32_t>* labels = nullptr, int distanceType = CV_DIST_L2, int maskSize = CV_DIST_MASK_PRECISE);


} /* namespace algorithms */
} /* planning2d */


#include "implementation/DistanceTransformImpl.hpp"

#endif /* INCLUDE_PLANNER_ALGORITHMS_DISTANCETRANSFORM_HPP_ */
