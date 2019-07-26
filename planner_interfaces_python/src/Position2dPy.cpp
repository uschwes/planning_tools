/*
 * Pose2dPy.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/Position2d.hpp>
#include <planner_interfaces_python/SupportPy.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;

// we cannot pass references to doubles through boost python, or can we?
template <typename T> void setX(Point2d<T>& point, T x) { point.x() = x; }
template <typename T> void setY(Point2d<T>& point, T y) { point.y() = y; }
template <typename T> T getX(const Point2d<T>& point) { return point.x(); }
template <typename T> T getY(const Point2d<T>& point) { return point.y(); }


template <typename T>
void exportPoint2d(const std::string& name) {


  typedef Point2d<T> Point;
  class_<Point>(name.c_str(), init<>("Default constructor"))
      .def(init<typename Point::Vector>(""))
      .def(init<T,T>(""))
      .def(init<Point>(""))
      .def("asVector", make_function( (const typename Point::Vector& (Point::*) (void) const)&Point::asVector, return_value_policy<copy_const_reference>()), "Returns the coordinate as a numpy array")
      .def("setVector", &Point::setVector, "Sets the coordinates from a numpy array")
      .add_property("x", &getX<T>, &setX<T>, "x-coordinate")
      .add_property("y", &getY<T>, &setY<T>, "y-coordinate")
      .def("cwiseProduct", &Point::cwiseProduct, "Multiplication with scalar")
      .def("__eq__", &Point::operator==)
      .def("__ne__", &Point::operator!=)
      .def("__lt__", &Point::operator<)
      .def("__le__", &Point::operator<=)
      .def("__gt__", &Point::operator>)
      .def("__ge__", &Point::operator>=)
      .def(self + Point())
      .def(self += Point())
      .def(self - Point())
      .def(self -= Point())
      .def("norm", &Point::norm, "Computes Euclidean L2 norm")
      .def("squaredNorm", &Point::squaredNorm, "Computes squared Euclidean L2 norm")
      .def("sum", &Point::sum, "Computes sum of x- and y-coordinate")
      .def("floor", (Point2d<T> (Point::*) (void) const)&Point::floor, "For both x and y, computes the largest integer value not greater than the entry")
      .def("ceil", (Point2d<T> (Point::*) (void) const)&Point::ceil, "For both x and y, computes the largest integer value not less than the entry")
      .def("__str__", &toString<Point>)
      .def_pickle(BoostSerializationBinary_pickle_suite<Point>())
  ;


  typedef Point2dStamped<T> PointStamped;
  class_<PointStamped, typename PointStamped::Ptr, bases<Point, StampedType> >((name + "Stamped").c_str(), init<>("Point2dStamped(): Default constructor"))
      .def(init<const typename PointStamped::Vector&, const Time&>("Point2dStamped(Vector xy, Time stamp)"))
      .def(init<double, double, const Time&>("Point2dStamped(double x,double y, Time stamp)"))
      .def(init<const Point&, const Time&>(("Point2dStamped(" + name + ", Time stamp)").c_str()))
      .def("__eq__", &PointStamped::operator==)
      .def("__ne__", &PointStamped::operator!=)
      .def_pickle(BoostSerializationBinary_pickle_suite<PointStamped>())
  ;

}

void exportPosition2d() {

  exportPoint2d<float>("PointFloat");
  exportPoint2d<double>("PointDouble");
  exportPoint2d<int>("PointInt");
  exportPoint2d<int64_t>("PointInt64");
  exportPoint2d<std::size_t>("PointUnsignedInt");

} /* void exportPosition2d() */
