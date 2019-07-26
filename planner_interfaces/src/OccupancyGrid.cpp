/*
 * OccupancyGrid.cpp
 *
 *  Created on: 05.08.2015
 *      Author: sculrich
 */

#include <planner_interfaces/OccupancyGrid.hpp>

namespace planning2d
{

template class Map<uint8_t, Eigen::RowMajor>;
template class Map<uint16_t, Eigen::RowMajor>;
template class Map<uint32_t, Eigen::RowMajor>;
template class Map<uint64_t, Eigen::RowMajor>;
template class Map<int8_t, Eigen::RowMajor>;
template class Map<int16_t, Eigen::RowMajor>;
template class Map<int32_t, Eigen::RowMajor>;
template class Map<int64_t, Eigen::RowMajor>;
template class Map<OccupancyValue, Eigen::RowMajor>;
template class Map<float, Eigen::RowMajor>;
template class Map<double, Eigen::RowMajor>;

} /* namespace planning2d */
