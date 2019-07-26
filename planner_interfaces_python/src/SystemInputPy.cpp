/*
 * SystemInputPy.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/SystemInput.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;

inline void setDataWrapper(SystemInput& u, const SystemInput::T& v) {
  u.data() = v;
}


void exportSystemInput() {

  class_<SystemInput, SystemInput::Ptr>("SystemInput", init<>("Default constructor"))
      .def(init<size_t>("SystemInput(int nInputs): Constructor creates SystemInput with nInputs inputs."))
      .def(init<SystemInput::T>("SystemInput(np.arraynd inputs): Constructor creates SystemInput from array."))
      .add_property("data", make_function((const SystemInput::T& (SystemInput::*) (void) const)&SystemInput::data, return_value_policy<copy_const_reference>()), &setDataWrapper)
      .add_property("dimension", &SystemInput::dimension)
//      .def("item", &getItem, &setItem)
      .def("__eq__", &SystemInput::operator==)
      .def("__ne__", &SystemInput::operator!=)
      .def_pickle(BoostSerializationBinary_pickle_suite<SystemInput>())
  ;

  class_<SystemInputStamped, SystemInputStamped::Ptr, bases<SystemInput, StampedType> >("SystemInputStamped", init<>("Default constructor"))
      .def(init<size_t>("SystemInputStamped(int nInputs): Constructor creates SystemInputStamped with nInputs inputs."))
      .def(init<SystemInput, Time>("SystemInputStamped(SystemInput input, Time stamp): Constructor"))
      .def("__eq__", &SystemInputStamped::operator==)
      .def("__ne__", &SystemInputStamped::operator!=)
      .def_pickle(BoostSerializationBinary_pickle_suite<SystemInputStamped>())
  ;

} /* void exportSystemInput() */
