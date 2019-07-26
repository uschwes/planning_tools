/*
 * DifferentialDriveStatePy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/DifferentialDriveState.hpp>

#include <planner_interfaces_python/SupportPy.hpp>
#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace common_agents;

inline void setVelocityWrapper(DifferentialDriveState& state, const Eigen::Vector2d& v) {
  state.velocity() = v;
}

void exportDifferentialDriveState() {

  class_<DifferentialDriveState, DifferentialDriveState::Ptr, bases<State> >("DifferentialDriveState", init<>("DifferentialDriveState(): Default constructor"))
     .def(init<const planning2d::State&>("DifferentialDriveState(State state): Takes a planning2d::State object"))
     .def(init<const DifferentialDriveState&>("DifferentialDriveState(DifferentialDriveState state): Copy constructor"))
     .def(init<const double, const double, const planning2d::Pose2d&>("DifferentialDriveState(double vtrans, double vrot, Pose2d pose)"))
     .def(init<const Eigen::Vector2d&, const planning2d::Pose2d&>("DifferentialDriveState(np.array vel, Pose2d pose)"))
     .def(init<const double, const double, const double, const double, const double>("DifferentialDriveState(x, y, yaw, vtrans, vrot)"))
     .add_property("velocity", make_function((const DifferentialDriveState::T& (DifferentialDriveState::*) (void) const)&DifferentialDriveState::velocity, return_value_policy<copy_const_reference>()), &setVelocityWrapper)
     .add_property("vt", make_function((const double& (DifferentialDriveState::*) (void) const)&DifferentialDriveState::getVelTrans, return_value_policy<copy_const_reference>()), &DifferentialDriveState::setVelTrans)
     .add_property("vr", make_function((const double& (DifferentialDriveState::*) (void) const)&DifferentialDriveState::getVelRot, return_value_policy<copy_const_reference>()), &DifferentialDriveState::setVelRot)
     .def("__str__", &toString<DifferentialDriveState>)
     .def_pickle(BoostSerializationBinary_pickle_suite<DifferentialDriveState>())
  ;

  class_<DifferentialDriveStateStamped, DifferentialDriveStateStamped::Ptr, bases<DifferentialDriveState, StateStamped> >("DifferentialDriveStateStamped",
    init<>("DifferentialDriveStateStamped(): Default constructor"))
    .def(init<const planning2d::State&, const planning2d::Time&>("DifferentialDriveState(State state, Time stamp): Takes a planning2d::State object and a stamp"))
    .def(init<const DifferentialDriveState&, const planning2d::Time&>("DifferentialDriveState(DifferentialDriveState state, Time stamp)"))
    .def(init<const planning2d::StateStamped&>("DifferentialDriveStateStamped(stateStamped): Takes a planning2d::StateStamped object."))
    .def("__str__", &toString<DifferentialDriveStateStamped>)
    .def_pickle(BoostSerializationBinary_pickle_suite<DifferentialDriveStateStamped>())
  ;

  implicitly_convertible<DifferentialDriveState::Ptr, DifferentialDriveState::ConstPtr >();
  implicitly_convertible<DifferentialDriveStateStamped::Ptr, DifferentialDriveStateStamped::ConstPtr >();

} /* void exportDifferentialDriveState() */
