/*
 * RawFeaturePy.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: sculrich
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/features/RawFeature.hpp>
#include <probabilistic_planner/features/RawFeatureErrorTerm.hpp>

using namespace std;
using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;
using namespace aslam::backend;

void setNameWrapper(RawFeature& f, const std::string& name) {
  f.name() = name;
}

template <int Dim>
void exportRawFeatureErrorTerm() {

  typedef RawFeatureErrorTerm<Dim> RawFeatureErrorTermDim;

  class_<RawFeatureErrorTermDim, typename RawFeatureErrorTermDim::Ptr, bases<ScalarNonSquaredErrorTerm>, boost::noncopyable>(("RawFeatureErrorTerm" + to_string(Dim)).c_str(), no_init)
    .add_property("feature", make_function(&RawFeatureErrorTermDim::feature, return_internal_reference<>()))
  ;
  implicitly_convertible<typename RawFeatureErrorTermDim::Ptr, typename RawFeatureErrorTermDim::ConstPtr >();

}

void exportRawFeature() {

  exportRawFeatureErrorTerm<1>();
  exportRawFeatureErrorTerm<2>();

  class_<RawFeature, RawFeature::Ptr, boost::noncopyable>("RawFeature", no_init)
    .add_property("name", make_function((const std::string& (RawFeature::*) (void) const)&RawFeature::name, return_value_policy<copy_const_reference>()), &setNameWrapper, "The feature name")
    .add_property("nameWithOptAgentType", &RawFeature::nameWithOptAgentType, "The feature name with OptAgent type concatenated")
    .def("addErrorTerms", pure_virtual(&RawFeature::addErrorTerms))
    .add_property("optAgentType", &RawFeature::getOptAgentType, make_function((void (RawFeature::*) (const OptAgentType&))&RawFeature::setOptAgentType), "OptAgentType for which the feature is defined")
    .def("setOptAgentType", make_function((void (RawFeature::*) (const std::string&))&RawFeature::setOptAgentType), "set type for which the feature is defined (string input)")
    .def("numWeights", &RawFeature::numWeights, "number of feature weights / feature dimension")
    .def("getWeight", &RawFeature::getWeight)
    .def("setWeight", &RawFeature::setWeight)
    .def("getWeightAsDesignVariable", make_function(&RawFeature::getWeightAsDesignVariable, return_internal_reference<>()), "specific feature weight as design variable")
    .def("getCurrentWeightsVector", make_function(&RawFeature::getCurrentWeightsVector, return_value_policy<copy_const_reference>()), "vector of feature weights")
    .def("activateForLearning", &RawFeature::activateForLearning, "activate this feature for learning")
    .add_property("numActiveWeights", &RawFeature::numActiveWeights, "number of active weights of this feature")
    .def("forbidNegativeWeights", &RawFeature::forbidNegativeWeights, "allow/disallow negative feature weights")
    .def("evaluate", &RawFeature::evaluate, "evaluate feature for given scene")
    .def("scale", (void (RawFeature::*) (const Eigen::VectorXd&))&RawFeature::scale)
    .def("scaleDim", (void (RawFeature::*) (const size_t, double))&RawFeature::scale)
    .add_property("scalingFactor", make_function((const Eigen::VectorXd& (RawFeature::*) (void) const)&RawFeature::getScalingFactor, return_value_policy<copy_const_reference>()),
                  "Current scaling factor for feature")
  ;

  implicitly_convertible<RawFeature::Ptr, RawFeature::ConstPtr >();


  class_<FeatureWeightScalarDesignVariable, FeatureWeightScalarDesignVariable*, bases<aslam::backend::Scalar>, boost::noncopyable>("FeatureWeightScalarDesignVariable", no_init)
  ;

} /* void exportRawFeature() */
