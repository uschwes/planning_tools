/*
 * DifferentialDriveAgentPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/DifferentialDriveAgent.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>
#include <boost/serialization/shared_ptr.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace common_agents;

void exportDifferentialDriveAgent() {

  class_<DifferentialDriveAgent, DifferentialDriveAgent::Ptr, bases<planning2d::Agent>, boost::noncopyable>("DifferentialDriveAgent", init<>("DifferentialDriveAgent(): Default constructor"))
    .def(init<bool, const planning2d::Id, const DifferentialDriveStateStamped&, planning2d::CollisionGeometryConstPtr>(
         "DifferentialDriveAgent(): bool isControllable, "
         "planning2d::Id id, "
         "DifferentialDriveStateStamped stateStamped, "
         "CollisionGeometry geometry)")
         )
    .def_pickle(BoostSerializationBinary_pickle_suite<DifferentialDriveAgent>())
  ;

//   implicitly_convertible<DifferentialDriveAgent::Ptr, DifferentialDriveAgent::ConstPtr >();

} /* void exportPedestrianAgent() */
