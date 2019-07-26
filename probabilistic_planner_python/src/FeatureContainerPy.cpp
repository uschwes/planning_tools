/*
 * FeatureContainerPy.cpp
 *
 *  Created on: Aug 13, 2015
 *      Author: pfmark
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner_python/FeatureContainerModifiable.hpp>

using namespace boost::python;
using namespace prob_planner;

boost::python::list getContainerWrapper(FeatureContainer& featureContainer) {
  boost::python::list container;
  for (const auto& feature : featureContainer.getContainer())
    container.append(feature);
  return container;
}

void exportFeatureContainer() {

  class_<FeatureContainer, boost::noncopyable>("FeatureContainer", init<const sm::value_store::ValueStore&>("FeatureContainer(ValueStore vpt): Constructor"))
      .def(init<const std::string&>("FeatureContainer(string xmlPath): Constructor"))
      .def("getContainer", &getContainerWrapper, "Get feature container itself")
      .def("getFeature", &FeatureContainer::getFeature, "Feature f = getFeature(string name): Get feature from container")
      .def("addErrorTerms", &FeatureContainerModifiable::addErrorTerms, "Add error terms of all features contained in the feature container to the optimization problem")
      .def("save", &FeatureContainer::save, "save(string xmlPathOut): Save feature container to xml file")
      .add_property("numFeaturesActiveForLearning", &FeatureContainer::numFeaturesActiveForLearning)
  ;

  class_<FeatureContainerModifiable, bases<FeatureContainer>, boost::noncopyable>("FeatureContainerModifiable", init<>("Default constructor"))
      .def(init<const sm::value_store::ValueStore&>("FeatureContainerModifiable(ValueStore vpt): Constructor"))
      .def(init<const std::string&>("FeatureContainerModifiable(string xmlPath): Constructor"))
      .def("push_back", (void (FeatureContainerModifiable::*) (const std::string&, const OptAgentType&, const double)) &FeatureContainerModifiable::push_back, "Add feature to container")
  ;

} /* void exportFeatureContainer() */
