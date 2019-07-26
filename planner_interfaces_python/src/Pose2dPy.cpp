/*
 * Pose2dPy.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/Pose2d.hpp>
#include <planner_interfaces_python/SupportPy.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;

// we cannot pass references to doubles through boost python, or can we?
void setX(Pose2d& pose, double x) { pose.x() = x; }
void setY(Pose2d& pose, double y) { pose.y() = y; }
void setYaw(Pose2d& pose, double yaw) { pose.yaw() = yaw; }

void setPosition(Pose2d& pose, const Position2d& p) { pose.position() = p; }


void exportPose2d() {

  // TODO: docstrings
  class_<Pose2d>("Pose2d", init<>("Default constructor"))
      .def(init<Pose2d::Vector>("Pose2d(numpy.arraynd pose): Constructor creates Pose2d from a 3d array (x/y/yaw)."))
      .def(init<double,double,double>("Pose2d(double x, double y, double yaw): Constructor creates Pose2d from x, y and yaw value."))
      .def(init<const Position2d&,double>("Pose2d(Position pos, double yaw): Constructor creates Pose2d from Position2d and yaw value."))
      .def("asVector", &Pose2d::asVector)
      .def("setVector", &Pose2d::setVector)
      .add_property("x", make_function((const double& (Pose2d::*) (void) const)&Pose2d::x, return_value_policy<copy_const_reference>()), &setX)
      .add_property("y", make_function((const double& (Pose2d::*) (void) const)&Pose2d::y, return_value_policy<copy_const_reference>()), &setX)
      .add_property("yaw", make_function((const double& (Pose2d::*) (void) const)&Pose2d::yaw, return_value_policy<copy_const_reference>()), &setYaw)
      .def("normalizeYaw", make_function(&Pose2d::normalizeYaw, return_internal_reference<>()))
      .add_property("position", make_function((const Position2d& (Pose2d::*) (void) const)&Pose2d::position, return_internal_reference<>()), &setPosition)
      .def("asVector", &Pose2d::asVector)
      .def("transformTo", &Pose2d::transformTo)
      .def("transformFrom", &Pose2d::transformFrom)
      .def("normalizeMinusPiPlusPi", &Pose2d::normalizeMinusPiPlusPi)
      .staticmethod("normalizeMinusPiPlusPi")
      .def("__eq__", &Pose2d::operator==)
      .def("__ne__", &Pose2d::operator!=)
      .def(self + Pose2d())
      .def(self - Pose2d())
      .def("__str__", &toString<Pose2d>)
      .def_pickle(BoostSerializationBinary_pickle_suite<Pose2d>())
  ;

  class_<Pose2dStamped, Pose2dStamped::Ptr, bases<Pose2d, StampedType> >("Pose2dStamped", init<>("Default constructor"))
      .def(init<const Pose2dStamped::Vector&, const Time&>("Pose2dStamped(numpy.arraynd pose, Time stamp): Constructor from 3d array and Time object"))
      .def(init<const Position2d&, double, const Time&>("Pose2dStamped(numpy.arraynd pose, double yaw, Time stamp): Constructor from a Position2d, the yaw angle and a Time object"))
      .def(init<double, double, double, const Time&>("Pose2dStamped(double x, double y, double yaw, Time stamp): Constructor from a Position2d, the yaw angle and a Time object"))
      .def(init<const Pose2d&, const Time&>("Pose2dStamped(Pose2d pose, Time stamp): Constructor from a Pose2d and a Time object"))
      .def("__eq__", &Pose2dStamped::operator==)
      .def("__ne__", &Pose2dStamped::operator!=)
      .def_pickle(BoostSerializationBinary_pickle_suite<Pose2dStamped>())
  ;

} /* void exportPose2d() */
