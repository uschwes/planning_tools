/*
 * RawFeaturePairwisePy.cpp.cpp
 *
 *  Created on: Jul 16, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/features/RawFeaturePairwise.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

inline boost::python::list getSuitableAgentPairsWrapper(RawFeaturePairwise* f, const planning2d::Time& t, const ContinuousScene& scene) {
  boost::python::list l;
  for (auto agentPair : f->getSuitableAgentPairs(t, scene))
    l.append(make_tuple(agentPair.first.get(), agentPair.second.get()));
  return l;
}

void exportRawFeaturePairwise() {

  class_<RawFeaturePairwise, bases<RawFeature>, boost::noncopyable >("RawFeaturePairwise", no_init)
    .def("getSuitableAgentPairs", &getSuitableAgentPairsWrapper)
  ;

  implicitly_convertible<RawFeaturePairwise::Ptr, RawFeaturePairwise::ConstPtr >();

} /* void exportRawFeaturePairwise() */
