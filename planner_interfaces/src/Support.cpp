/*
 * Support.cpp
 *
 *  Created on: Oct 21, 2014
 *      Author: sculrich
 */

// Boost includes
#include <boost/math/constants/constants.hpp>

// self includes
#include "../include/planner_interfaces/Support.hpp"

namespace planning2d {

const double PI = boost::math::constants::pi<double>();
const double TWOPI = 2.*PI;
const double LOG_TWOPI = log(TWOPI);

} /* namespace planning2d */
