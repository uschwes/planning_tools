/*
 * HolonomicStatePy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/HolonomicState.hpp>

#include <planner_interfaces_python/SupportPy.hpp>
#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace common_agents;

inline void setVelocityWrapper(HolonomicState& state, const Eigen::Vector2d& v) {
  state.velocity() = v;
}

void exportHolonomicState() {

  class_<HolonomicState, HolonomicState::Ptr, bases<State>, boost::noncopyable>("HolonomicState", init<>("HolonomicState(): Default constructor"))
    .def(init<const planning2d::State&>("HolonomicState(State state): Takes a planning2d::State object"))
    .def(init<const HolonomicState&>("HolonomicState(HolonomicState state): Copy constructor"))
    .def(init<double, double, const planning2d::Pose2d&>("HolonomicState(double vx, double vy, Pose2d pose)"))
    .def(init<double, double, double, const planning2d::Position2d&>("HolonomicState(double vx, double vy, double yaw, Position2d pos)"))
    .def(init<const Eigen::Vector2d&, const planning2d::Pose2d&>("HolonomicState(np.array vel, Pose2d pose)"))
    .def(init<const double, const double, const double, const double, const double>("HolonomicState(x, y, yaw, vx, vy)"))
    .add_property("velocity", make_function((const HolonomicState::T& (HolonomicState::*) (void) const)&HolonomicState::velocity, return_value_policy<copy_const_reference>()), &setVelocityWrapper)
    .add_property("vx", make_function((const double& (HolonomicState::*) (void) const)&HolonomicState::getVelX, return_value_policy<copy_const_reference>()), &HolonomicState::setVelX)
    .add_property("vy", make_function((const double& (HolonomicState::*) (void) const)&HolonomicState::getVelY, return_value_policy<copy_const_reference>()), &HolonomicState::setVelY)
    .def("__str__", &toString<HolonomicState>)
    .def_pickle(BoostSerializationBinary_pickle_suite<HolonomicState>())
  ;

  class_<HolonomicStateStamped, HolonomicStateStamped::Ptr, bases<HolonomicState, StateStamped>, boost::noncopyable>("HolonomicStateStamped", init<>("HolonomicStateStamped(): Default constructor"))
    .def(init<const planning2d::State&, const planning2d::Time&>("HolonomicStateStamped(State state, Time stamp): Takes a planning2d::State object and a stamp"))
    .def(init<const HolonomicState&, const planning2d::Time&>("HolonomicStateStamped(HolonomicState state, Time stamp)"))
    .def(init<const planning2d::StateStamped&>("HolonomicStateStamped(stateStamped): Takes a planning2d::StateStamped object."))
    .def("__str__", &toString<HolonomicStateStamped>)
    .def_pickle(BoostSerializationBinary_pickle_suite<HolonomicStateStamped>())
  ;

  implicitly_convertible<HolonomicState::Ptr, HolonomicState::ConstPtr >();
  implicitly_convertible<HolonomicStateStamped::Ptr, HolonomicStateStamped::ConstPtr >();

} /* void exportHolonomicState() */
