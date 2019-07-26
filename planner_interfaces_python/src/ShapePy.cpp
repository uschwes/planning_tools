/*
 * ShapePy.cpp
 *
 *  Created on: Jul 5, 2016
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/register_ptr_to_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/Shape.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>
#include <boost/serialization/shared_ptr.hpp>

using namespace boost::python;
using namespace planning2d;


void exportShape() {

  class_<DiscApproximation>("DiscApproximation", init<>("DiscApproximation(): Default constructor"))
      .def(init<std::size_t>("DiscApproximation(int numDisks): Constructor"))
      .def("getRadius", &DiscApproximation::getRadius, "Get radius of disc number i")
      .def("getPosition", make_function(&DiscApproximation::getPosition, return_internal_reference<>()), "Get position of disc number i")
      .add_property("numDiscs", &DiscApproximation::getNumDiscs, "The number of discs in the data structure")
      .def("insertDisc", &DiscApproximation::insertDisc, "Inserts a disc into the data structure")
      .def("transformFrom", make_function(&DiscApproximation::transformFrom, return_internal_reference<>()), "Transform all the contained discs from the frame given by the pose argument")
      .def_pickle(BoostSerializationBinary_pickle_suite<DiscApproximation>())
  ;

} /* void exportAgent() */
