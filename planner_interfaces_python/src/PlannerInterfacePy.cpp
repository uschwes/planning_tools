/*
 * PlannerInterfacePy.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/PlannerInterface.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;


void exportPlannerInterface() {

  class_<PlannerInterface, PlannerInterface::Ptr, boost::noncopyable>("PlannerInterface", no_init)
      .def("reset", pure_virtual(&PlannerInterface::reset))
      .def("setPlanningFrame", &PlannerInterface::setPlanningFrame)
      .def("computePlan", pure_virtual(&PlannerInterface::computePlan))
      .def("callbackOccupancyGrid", pure_virtual(&PlannerInterface::callbackOccupancyGrid))
      .def("callbackCurrentState", pure_virtual(&PlannerInterface::callbackCurrentState))
      .def("callbackDynamicObjects", pure_virtual(&PlannerInterface::callbackDynamicObjects))
      .def_pickle(BoostSerializationBinary_pickle_suite<PlannerInterface>())
  ;

} /* void exportPlannerInterface() */
