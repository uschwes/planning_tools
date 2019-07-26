/*
 * HolonomicAgentPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/HolonomicAgent.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>
#include <boost/serialization/shared_ptr.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace common_agents;

void exportHolonomicAgent() {

  class_<HolonomicAgent, HolonomicAgent::Ptr, bases<Agent>, boost::noncopyable>("HolonomicAgent", init<>("HolonomicAgent(): Default constructor"))
    .def(init<bool, const planning2d::Id, const HolonomicStateStamped&, planning2d::CollisionGeometryConstPtr>(
         "HolonomicAgent(): bool isControllable, "
         "planning2d::Id id, "
         "HolonomicStateStamped stateStamped, "
         "CollisionGeometry geometry)")
     )
     .def_pickle(BoostSerializationBinary_pickle_suite<HolonomicAgent>())
  ;

  implicitly_convertible<HolonomicAgent::Ptr, HolonomicAgent::ConstPtr >();

} /* void exportPedestrianAgent() */
