/*
 * TrajectoryPy.cpp
 *
 *  Created on: Jul 17, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <planner_interfaces/Trajectory.hpp>

#include <planner_interfaces_python/PythonPickleSupport.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;


template <typename T>
T extractRangeWrapper(const T& trajectory, const Time& start, const Time& end)
{
  return extractRange(start, end, trajectory);
}

template <typename T>
bool isEqual(const T& val0, const T& val1) { return val0 == val1; }

template <typename T>
bool isNotEqual(const T& val0, const T& val1) { return val0 == val1; }

template <typename T>
void exportStampedTrajectoryT(const std::string& name)
{

  class_< T >(name.c_str())
    .def(vector_indexing_suite< T >())
    .def("__iter__", boost::python::iterator< T >())
    .def("__len__", &T::size)
    .def("clear", &T::clear)
    .def("extractRange", &extractRangeWrapper<T>)
    .def("__eq__", &isEqual<T>)
    .def("__ne__", &isNotEqual<T>)
    .def_pickle(BoostSerializationBinary_pickle_suite<T>())
  ;
}

template <typename T>
void exportTrajectoryT(const std::string& name)
{

  class_< T >(name.c_str())
    .def(vector_indexing_suite< T >())
    .def("__iter__", boost::python::iterator< T >())
    .def("__len__", &T::size)
    .def("clear", &T::clear)
    .def("__eq__", &isEqual<T>)
    .def("__ne__", &isNotEqual<T>)
    .def_pickle(BoostSerializationBinary_pickle_suite<T>())
  ;
}


void exportTrajectory() {

  exportStampedTrajectoryT<PositionTrajectory>("PositionTrajectory");
  exportStampedTrajectoryT<PoseTrajectory>("PoseTrajectory");
  exportStampedTrajectoryT<StateTrajectory>("StateTrajectory");
  exportStampedTrajectoryT<SystemInputTrajectory>("SystemInputTrajectory");
  exportStampedTrajectoryT<StateInputTrajectory>("StateInputTrajectory");
  exportTrajectoryT<Path>("Path");

} /* void exportTrajectory() */
