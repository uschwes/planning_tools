/*
 * TimePy.cpp
 *
 *  Created on: Oct 23, 2014
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/Time.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>
#include <planner_interfaces_python/SupportPy.hpp>

using namespace boost::python;
using namespace planning2d;


template <typename T>
std::string formatWFormatter(const T& t, const time::Formatter& fmt) {
  return toString(t.format(fmt));
}


void exportTime() {
  
  enum_<time::TimeUnit>("TimeUnit")
    .value("SEC", time::TimeUnit::SEC)
    .value("MILLISEC", time::TimeUnit::MILLISEC)
    .value("MICROSEC", time::TimeUnit::MICROSEC)
    .value("NANOSEC", time::TimeUnit::NANOSEC)
  ;

  class_<time::Formatter>("TimeFormatter",
    init< optional<time::TimeUnit, int, const std::string&, const std::string&> >
      ("TimeFormatter(TimeUnit unit, int precision, string bracketLeft, string bracketRight)"))
  ;

  class_<Time>("Time", init<>("Default constructor"))
    .def(init<double>("Time(double seconds): Constructs time object from a stamp given in seconds"))
    .def(init<time::T>("Time(int nanoseconds): Constructs time object from a stamp given in nanoseconds"))
    .def(self + Duration())
    .def(self - Duration())
    .def(self += Duration())
    .def(self -= Duration())
    .def(self - Time())
    .def("__eq__", &Time::operator==)
    .def("__ne__", &Time::operator!=)
    .def(self > Time())
    .def(self < Time())
    .def(self >= Time())
    .def(self <= Time())
    .def("isValid", &Time::isValid, "bool isValid(): Returns true iff time object was properly initialized.")
    .def("isZero", &Time::isZero, "bool isZero(): Returns true iff timestamp is 0.")
    .def("toSec", &Time::toSec, "double toSec(): Returns time in seconds.")
    .def("toDateString", (std::string (Time::*) (void) const)&Time::toDateString, "string toDateString(): Returns time as date string.")
    .def("toDateStringFormatted", (std::string (Time::*) (const std::string&) const)&Time::toDateString, "string toDateString(string format): Returns time as formatted date string.")
    .def("__str__", &toString<Time> )
    .def("format", &formatWFormatter<Time>)
    .def_readwrite("nanosec", &Time::nanosec, "Nanosecond member")
    .def_pickle(BoostSerializationBinary_pickle_suite<Time>())
  ;

  class_<Duration>("Duration", init<>("Default constructor initializes duration to zero"))
    .def(init<double>("Duration(double seconds): Constructs duration object from a stamp given in seconds"))
    .def(init<time::T>("Duration(int nanoseconds): Constructs duration object from a stamp given in nanoseconds"))
    .def(self + Duration())
    .def(self - Duration())
    .def(self += Duration())
    .def(self -= Duration())
    .def(self / Duration())
    .def(self /= double())
    .def(self / double())
    .def(self *= double())
    .def(self * double())
    .def("__eq__", &Duration::operator==)
    .def("__ne__", &Duration::operator!=)
    .def(self > Duration())
    .def(self < Duration())
    .def(self >= Duration())
    .def(self <= Duration())
    .def("toSec", &Duration::toSec, "double toSec(): Returns duration in seconds.")
    .def("__str__", &toString<Duration> )
    .def("format", &formatWFormatter<Duration>, "string format(TimeFormatter format): Returns a formatted string")
    .def("format", (std::string (Duration::*) (const std::string&) const)&Duration::format, "string format(string format): Returns a formatted string")
    .def_readwrite("nanosec", &Duration::nanosec, "Nanosecond member")
    .def_pickle(BoostSerializationBinary_pickle_suite<Duration>())
  ;

} /* void exportTime() */
