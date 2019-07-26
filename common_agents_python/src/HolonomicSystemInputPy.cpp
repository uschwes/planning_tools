/*
 * HolonomicSystemInputPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/HolonomicSystemInput.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace common_agents;

inline void setVelocityWrapper(HolonomicSystemInput& u, const Eigen::Vector2d& v) {
  u.velocity() = v;
}

void exportHolonomicSystemInput() {

  class_<HolonomicSystemInput, HolonomicSystemInput::Ptr, bases<SystemInput> >("HolonomicSystemInput", init<>("HolonomicSystemInput(): Default constructor"))
    .def(init<const planning2d::SystemInput&>("HolonomicSystemInput(SystemInput input): Takes a planning2d::SystemInput object"))
    .def(init<double, double>("HolonomicSystemInput(double vx, double vy)"))
    .def(init<const Eigen::Vector2d&>("HolonomicSystemInput(np.array vel)"))
    .add_property("velocity", make_function((const HolonomicSystemInput::T& (HolonomicSystemInput::*) (void) const)&HolonomicSystemInput::velocity, return_value_policy<copy_const_reference>()), &setVelocityWrapper)
    .add_property("vx", make_function((const double& (HolonomicSystemInput::*) (void) const)&HolonomicSystemInput::getVelX, return_value_policy<copy_const_reference>()), &HolonomicSystemInput::setVelX)
    .add_property("vy", make_function((const double& (HolonomicSystemInput::*) (void) const)&HolonomicSystemInput::getVelY, return_value_policy<copy_const_reference>()), &HolonomicSystemInput::setVelY)
    .def_pickle(BoostSerializationBinary_pickle_suite<HolonomicSystemInput>())
  ;

  class_<HolonomicSystemInputStamped, HolonomicSystemInputStamped::Ptr, bases<HolonomicSystemInput, SystemInputStamped> >("HolonomicSystemInputStamped", init<>("HolonomicSystemInputStamped(): Default constructor"))
    .def(init<const planning2d::SystemInput&, const planning2d::Time&>("HolonomicSystemInputStamped(SystemInput input, Time stamp): Takes a planning2d::SystemInput object and a stamp"))
    .def(init<const HolonomicSystemInput&, const planning2d::Time&>("HolonomicSystemInputStamped(HolonomicSystemInput input, Time stamp)"))
    .def_pickle(BoostSerializationBinary_pickle_suite<HolonomicSystemInputStamped>())
  ;

  implicitly_convertible<HolonomicSystemInput::Ptr, HolonomicSystemInput::ConstPtr >();
  implicitly_convertible<HolonomicSystemInputStamped::Ptr, HolonomicSystemInputStamped::ConstPtr >();

} /* void exportHolonomicState() */
