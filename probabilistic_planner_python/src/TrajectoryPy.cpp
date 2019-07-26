/*
 * TrajectoryPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <probabilistic_planner/state_representation/Trajectory.hpp>
#include <aslam/backend/OptimizationProblem.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

void exportTrajectory() {

  class_< std::vector<aslam::backend::DesignVariable*> >("DesignVariableVector")
    .def(vector_indexing_suite< std::vector<aslam::backend::DesignVariable*> >())
    .def("__iter__", boost::python::iterator< std::vector<aslam::backend::DesignVariable*> >())
  ;

  class_<Trajectory, Trajectory::Ptr>("Trajectory", init<>("Trajectory(): Default constructor"))
    .def("reset", &Trajectory::reset)
    .add_property("isInitialized", &Trajectory::isInitialized)
    .def("initFromDiscretizedTrajectory", (void (Trajectory::*) (const PositionTrajectory&, const int, const double, const Time*, const Time*))&Trajectory::initFromDiscretizedTrajectory,
         (boost::python::arg("minTime") = boost::python::object(), boost::python::arg("maxTime") = boost::python::object()),
         "void initFromDiscretizedTrajectory(PositionTrajectory trajectory, int numberOfSegments, double lambda, Time minTime=None, Time maxTime=None): "
         "Computes the continuous trajectory from a sampled version. Default values for minTime and maxTime are taken from discrete trajectory")
    .def("initFromDiscretizedTrajectory", (void (Trajectory::*) (const StateTrajectory&, const int, const double, const Time*, const Time*))&Trajectory::initFromDiscretizedTrajectory,
         (boost::python::arg("minTime") = boost::python::object(), boost::python::arg("maxTime") = boost::python::object()),
         "void initFromDiscretizedTrajectory(StateTrajectory trajectory, int numberOfSegments, double lambda, Time minTime=None, Time maxTime=None): "
         "Computes the continuous trajectory from a sampled version. Default values for minTime and maxTime are taken from discrete trajectory")
    .def("initStraightSpline", (void (Trajectory::*) (const Position2dStamped&, const Position2dStamped&, const int, const double))&Trajectory::initStraightSpline,
         "initStraightSpline(Position2dStamped startingPoint, Position2dStamped goalPoint, int numberOfSegments, double lambda)")
    .def("initStraightSpline", (void (Trajectory::*) (const StateStamped&, const StateStamped&, const int, const double))&Trajectory::initStraightSpline,
         "initStraightSpline(StateStamped startingPoint, StateStamped goalPoint, int numberOfSegments, double lambda)")
    .def("initZeroSpline", &Trajectory::initZeroSpline,
         "initZeroSpline(Time start, Time end, int numberOfSegments)")
    .def("adjustTime", (void (Trajectory::*) (const Time&, const Time&, const double, const Duration&))&Trajectory::adjustTime)
    .def("shiftTime", (void (Trajectory::*) (const Duration&, const double, const Duration&))&Trajectory::shiftTime)
    .def("activateAllDesignVariables", &Trajectory::activateAllDesignVariables, "activateAllDesignVariables(bool activate): Activates/Deactivates all design variables in the trajectory")
    .def("getDesignVariables", make_function(&Trajectory::getDesignVariables, return_internal_reference<>()), "Returns the design variables attached to the spline")
    .def("addDesignVariables", &Trajectory::addDesignVariables)
    .def("getJacobian", &Trajectory::getJacobian, "")
    .def("getStateStamped", &Trajectory::getStateStamped, "")
    .def("getPose2d", &Trajectory::getPose2d, "") // TODO: Does this cause memory misalignment?
    .def("getPosition", (Eigen::Vector2d (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getPosition, "")
    .def("getPose", (Eigen::Vector3d (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getPose, "")
    .def("getPoseAndVelocityXY", (Eigen::Matrix<double,5,1> (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getPoseAndVelocityXY, "")
    .def("getPosition2d", &Trajectory::getPosition2d, "")
    .def("getVelocityXY", (Eigen::Vector2d (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getVelocityXY, "")
    .def("getAccelerationXY", (Eigen::Vector2d (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getAccelerationXY, "")
    .def("getVelocityTransRot", (Eigen::Vector2d (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getVelocityTransRot, "")
    .def("getRotationRateSquared", (double (Trajectory::*)(const planning2d::Time&) const)&Trajectory::getRotationRateSquared, "")
    .add_property("startTime", &Trajectory::getStartTime, "")
    .add_property("finalTime", &Trajectory::getFinalTime, "")
    .add_property("spline", make_function(&Trajectory::getSpline, return_internal_reference<>()), "Aslam Euclidean B-Spline object")
    .add_property("controlVertices", &Trajectory::getSplineControlVertices, "Control vertices of the spline")
    .def("contains", &Trajectory::contains, "bool contains(Time stamp): Whether or not the timestamp stamp is contained in the time interval of the trajectory")
    .def("discretizeTrajectory", &Trajectory::discretizeTrajectory, "")
    .def("discretizeTrajectoryForPeriod", &Trajectory::discretizeTrajectoryForPeriod, "")
  ;

  implicitly_convertible<Trajectory::Ptr, Trajectory::ConstPtr >();

} /* void exportOptAgent() */
