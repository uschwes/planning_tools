/*
 * ArchivePy.cpp
 *
 *  Created on: Mar 3, 2016
 *      Author: uschwes
 */

#include <boost/python.hpp>

#include <planner_interfaces_python/ArchivePy.hpp>

using namespace boost::python;

void exportArchive() {

  class_<OArchive, boost::noncopyable>("OArchive", init<const std::string&>("OArchive(string path): Constructor"))
  ;
  class_<IArchive, boost::noncopyable>("IArchive", init<const std::string&>("IArchive(string path): Constructor"))
  ;

} /* void exportArchive() */
