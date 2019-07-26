/*
 * DistanceTransform.cpp
 *
 *  Created on: 19.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <planner_algorithms/DistanceTransform.hpp>

namespace planning2d
{
namespace algorithms
{

template void distanceTransform(const Map<OccupancyValue>& in, Map<float>& out, Map<int32_t>* labels, int distanceType, int maskSize);
template void distanceTransform(const Map<uint8_t>& in, Map<float>& out, Map<int32_t>* labels, int distanceType, int maskSize);

} /* namespace algorithms */
} /* planning2d */
