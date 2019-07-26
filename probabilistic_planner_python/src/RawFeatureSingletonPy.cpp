/*
 * RawFeatureSingletonPy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/features/RawFeatureSingleton.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

inline boost::python::list getSuitableAgentsWrapper(RawFeatureSingleton* f, const planning2d::Time& t, const ContinuousScene& scene) {
  boost::python::list l;
  for (auto a : f->getSuitableAgents(t, scene))
    l.append(a.get());
  return l;
}

void exportRawFeatureSingleton() {

  class_<RawFeatureSingleton, bases<RawFeature>, boost::noncopyable >("RawFeatureSingleton", no_init)
    .def("getSuitableAgents", &getSuitableAgentsWrapper)
  ;

  implicitly_convertible<RawFeatureSingleton::Ptr, RawFeatureSingleton::ConstPtr >();

} /* void exportRawFeatureSingleton() */
