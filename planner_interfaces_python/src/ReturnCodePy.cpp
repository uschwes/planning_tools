/*
 * ReturnCodePy.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <planner_interfaces/ReturnCodes.hpp>

using namespace boost::python;
using namespace planning2d;

void exportReturnCode() {

  enum_<ReturnCode>("ReturnCode")
      .value("RETCODE_OK", ReturnCode::RETCODE_OK)
      .value("RETCODE_EXCEPTION_OCCURED", ReturnCode::RETCODE_EXCEPTION_OCCURED)
      .value("RETCODE_GENERAL_ERROR", ReturnCode::RETCODE_GENERAL_ERROR)
      .value("RETCODE_INVALID_INPUTS", ReturnCode::RETCODE_INVALID_INPUTS)
      .value("RETCODE_ROBOT_INEVITABLE_COLLISION_IN_FUTURE", ReturnCode::RETCODE_ROBOT_INEVITABLE_COLLISION_IN_FUTURE)
      .value("RETCODE_ROBOT_IN_COLLISION_NOW", ReturnCode::RETCODE_ROBOT_IN_COLLISION_NOW)
  ;

} /* void exportReturnCode() */
