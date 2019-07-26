/*
 * TargetPlannerInterfacePy.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/TargetPlannerInterface.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;


void exportTargetPlannerInterface() {

  class_<TargetPlannerInterface, TargetPlannerInterface::Ptr, boost::noncopyable, bases<PlannerInterface> >("TargetPlannerInterface", no_init)
      .def("callbackSetTarget", pure_virtual(&TargetPlannerInterface::callbackSetTarget))
      .def_pickle(BoostSerializationBinary_pickle_suite<TargetPlannerInterface>())
  ;

} /* void exportTargetPlannerInterfacePy() */
