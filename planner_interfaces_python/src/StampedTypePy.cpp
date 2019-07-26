/*
 * StampedTypePy.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/StampedType.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;

void setStamp(StampedType& s, const Time& t) { s.stamp() = t; }
const Time& getStamp(const StampedType& s) { return s.stamp(); }


void exportStampedType() {

  // TODO: docstrings
  class_<StampedType>("StampedType", init<>("Default constructor"))
      .def(init<Time>("StampedType(Time stamp): Constructor creates a StampedType from a Time object."))
      .add_property("stamp", make_function(&getStamp, return_internal_reference<>()), &setStamp)
      .def_pickle(BoostSerializationBinary_pickle_suite<StampedType>())
  ;

} /* void exportStampedType() */
