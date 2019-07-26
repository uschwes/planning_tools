/*
 * AgentPy.cpp
 *
 *  Created on: Feb 23, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/register_ptr_to_python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/Agent.hpp>

#include <planner_interfaces_python/SupportPy.hpp>
#include <planner_interfaces_python/PythonPickleSupport.hpp>
#include <boost/serialization/shared_ptr.hpp>

using namespace boost::python;
using namespace planning2d;

inline const StateStamped& getStateStampedWrapper(const Agent::Ptr agent) {
  return agent->stateStamped();
}
inline void setStateStampedWrapper(const Agent::Ptr agent, const StateStamped& s) {
  agent->stateStamped() = s;
}
inline void setControllableTrue(const Agent::Ptr agent, const bool isControllable) {
  agent->setControllable(isControllable);
}

const Agent& get_item(const std::vector<Agent::ConstPtr>& agentVector, int index) {
  SM_ASSERT_GE_LT(planning2d::OutOfBoundAccessException, index, 0, static_cast<int>(agentVector.size()), "");
  return *agentVector[index];
}


void exportAgent() {

  class_< std::vector<Agent::ConstPtr> >("AgentVector")
      .def(boost::python::vector_indexing_suite<std::vector<Agent::ConstPtr> >())
      .def("__init__", make_constructor(&vector_shared_ptr_from_list<Agent::ConstPtr>))
      .def("__init__", make_constructor(&vector_shared_ptr_from_object<Agent::ConstPtr>))
      .def("__getitem__", &get_item, return_internal_reference<>())
      .def_pickle(BoostSerializationBinary_pickle_suite< std::vector<Agent::ConstPtr> >())
  ;

  class_<Agent, Agent::Ptr, boost::noncopyable>("Agent", no_init)
      .def("initialize", &Agent::initialize)
      .def("setControllable", &Agent::setControllable)
      .add_property("isControllable", &Agent::isControllable, &setControllableTrue)
      .add_property("stateStamped", make_function(&getStateStampedWrapper, return_internal_reference<>()), &setStateStampedWrapper)
      .def("applyInput", pure_virtual(&Agent::applyInput))
      .add_property("id", &Agent::getId, &Agent::setId)
      .def("computeControlOutput", pure_virtual((SystemInput (Agent::*)(const StateStamped&) const)&Agent::computeControlOutput))
      .def("computeControlOutput", pure_virtual((SystemInput (Agent::*)(const Pose2d&) const)&Agent::computeControlOutput))
      .def("computeControlOutput", pure_virtual((SystemInput (Agent::*)(const StateTrajectory&) const)&Agent::computeControlOutput))
      .def("computeControlOutput", pure_virtual((SystemInput (Agent::*)(const HolonomicVelocity&) const)&Agent::computeControlOutput))
      .def("getDiscApproximation", pure_virtual(&Agent::getDiscApproximation), return_internal_reference<>())
      .def_pickle(BoostSerializationBinary_pickle_suite<Agent>())
  ;
  register_ptr_to_python< Agent::ConstPtr >();
  implicitly_convertible<Agent::Ptr, Agent::ConstPtr >();

} /* void exportAgent() */
