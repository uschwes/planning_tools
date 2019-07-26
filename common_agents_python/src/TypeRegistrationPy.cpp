/*
 * TypeRegistrationPy.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <common_agents/HolonomicAgent.hpp>
#include <common_agents/DifferentialDriveAgent.hpp>

#include <planner_interfaces_python/ArchivePy.hpp>

#include <boost/serialization/shared_ptr.hpp>

using namespace boost::python;
using namespace common_agents;

template <typename Archive>
void registerTypes(Archive& ar) {
  registerType<Archive, HolonomicSystemInput>(ar);
  registerType<Archive, HolonomicSystemInputStamped>(ar);
  registerType<Archive, HolonomicState>(ar);
  registerType<Archive, HolonomicStateStamped>(ar);
  registerType<Archive, HolonomicAgent>(ar);
  registerType<Archive, DifferentialDriveSystemInput>(ar);
  registerType<Archive, DifferentialDriveSystemInputStamped>(ar);
  registerType<Archive, DifferentialDriveState>(ar);
  registerType<Archive, DifferentialDriveStateStamped>(ar);
  registerType<Archive, DifferentialDriveAgent>(ar);
}

void exportTypeRegistration() {

  def("registerTypes", &registerTypes<OArchive>);
  def("registerTypes", &registerTypes<IArchive>);

} /* void exportPedestrianAgent() */
