/*
 * DifferentialDriveSystemInputPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/DifferentialDriveSystemInput.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace common_agents;

inline void setVelocityWrapper(DifferentialDriveSystemInput& u, const Eigen::Vector2d& v) {
  u.velocity() = v;
}

void exportDifferentialDriveSystemInput() {

  class_<DifferentialDriveSystemInput, DifferentialDriveSystemInput::Ptr, bases<SystemInput>, boost::noncopyable>("DifferentialDriveSystemInput", init<>("DifferentialDriveSystemInput(): Default constructor"))
     .def(init<const planning2d::SystemInput&>("DifferentialDriveSystemInput(SystemInput input): Takes a planning2d::SystemInput object"))
     .def(init<double, double>("DifferentialDriveSystemInput(double vtrans, double vrot)"))
     .def(init<const Eigen::Vector2d&>("DifferentialDriveSystemInput(np.array vel)"))
     .add_property("velocity", make_function((const DifferentialDriveSystemInput::T& (DifferentialDriveSystemInput::*) (void) const)&DifferentialDriveSystemInput::velocity, return_value_policy<copy_const_reference>()), &setVelocityWrapper)
     .add_property("vt", make_function((const double& (DifferentialDriveSystemInput::*) (void) const)&DifferentialDriveSystemInput::getVelTrans, return_value_policy<copy_const_reference>()), &DifferentialDriveSystemInput::setVelTrans, "translational velocity")
     .add_property("vr", make_function((const double& (DifferentialDriveSystemInput::*) (void) const)&DifferentialDriveSystemInput::getVelRot, return_value_policy<copy_const_reference>()), &DifferentialDriveSystemInput::setVelRot, "rotational velocity")
     .def_pickle(BoostSerializationBinary_pickle_suite<DifferentialDriveSystemInput>())
  ;

  class_<DifferentialDriveSystemInputStamped, DifferentialDriveSystemInputStamped::Ptr, bases<DifferentialDriveSystemInput, SystemInputStamped>, boost::noncopyable>("DifferentialDriveSystemInputStamped", init<>("DifferentialDriveSystemInputStamped(): Default constructor"))
    .def(init<const planning2d::SystemInput&, const planning2d::Time&>("DifferentialDriveSystemInputStamped(SystemInput input, Time stamp): Takes a planning2d::SystemInput object and a stamp"))
    .def(init<const DifferentialDriveSystemInput&, const planning2d::Time&>("DifferentialDriveSystemInputStamped(DifferentialDriveSystemInput input, Time stamp)"))
    .def_pickle(BoostSerializationBinary_pickle_suite<DifferentialDriveSystemInputStamped>())
  ;

  implicitly_convertible<DifferentialDriveSystemInput::Ptr, DifferentialDriveSystemInput::ConstPtr >();
  implicitly_convertible<DifferentialDriveSystemInputStamped::Ptr, DifferentialDriveSystemInputStamped::ConstPtr >();

} /* void exportHolonomicState() */
