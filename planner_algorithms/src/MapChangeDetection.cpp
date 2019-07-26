/*
 * ChangeDetection.cpp
 *
 *  Created on: 18.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <planner_algorithms/MapChangeDetection.hpp>

namespace planning2d
{
namespace algorithms
{

// Explicit template instantiation
template void changeDetection<uint8_t>(const Map<uint8_t>& map, Map<bool>& binary, bool tagBoundary);
template void changeDetection<float>(const Map<float>& map, Map<bool>& binary, bool tagBoundary);
template void changeDetection<double>(const Map<double>& map, Map<bool>& binary, bool tagBoundary);

} /* namespace algorithms */
} /* namespace planning2d */
