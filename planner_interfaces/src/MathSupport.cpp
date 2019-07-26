/*
 * MathSupport.cpp
 *
 *  Created on: 27.04.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <planner_interfaces/MathSupport.hpp>

namespace planning2d {
namespace math {

template void linspace(const double start, const double end, const std::size_t numVals, std::vector<double>& vals, const bool makeUnique /*= true*/);
template void linspace(const int64_t start, const int64_t end, const std::size_t numVals, std::vector<int64_t>& vals, const bool makeUnique /*= true*/);

} /* namespace planning2d */
} /* namespace math */
