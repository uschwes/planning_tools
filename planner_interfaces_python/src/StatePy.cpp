/*
 * StatePy.cpp
 *
 *  Created on: Feb 19, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/State.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;

void setPose(State& s, const Pose2d& p) { s.pose() = p; }
const Pose2d& getPose(const State& s) { return s.pose(); }

void setState(State& state, const State::T& s) { state.state() = s; }
const State::T& getState(const State& state) { return state.state(); }

double getItem(State& state, const size_t i) { return state(i); }
void setItem(State& state, const size_t i, double val) { state(i) = val; }


void exportState() {

  class_<State, State::Ptr>("State", init<>("Default constructor"))
      .def(init<size_t>("State(int nStates): Constructor creates State with nStates states."))
      .def(init<State::T, Pose2d>("State(np.arraynd states, Pose2d pose): Constructor creates State from array and a pose object."))
      .add_property("pose", make_function(&getPose, return_internal_reference<>()), &setPose)
      .add_property("state", make_function(&getState, return_value_policy<copy_const_reference>()), &setState)
      .add_property("dimension", &State::dimension)
//      .def("item", &getItem, &setItem)
      .def("__eq__", &State::operator==)
      .def("__ne__", &State::operator!=)
      .def_pickle(BoostSerializationBinary_pickle_suite<State>())
  ;

  class_<StateStamped, StateStamped::Ptr, bases<State, StampedType> >("StateStamped", init<>("Default constructor"))
      .def(init<size_t>("StateStamped(int nStates): Constructor creates StateStamped with nStates states."))
      .def(init<State, Time>("StateStamped(State state, Time stamp): Constructor"))
      .def("__eq__", &StateStamped::operator==)
      .def("__ne__", &StateStamped::operator!=)
      .def_pickle(BoostSerializationBinary_pickle_suite<StateStamped>())
  ;

} /* void exportState() */
