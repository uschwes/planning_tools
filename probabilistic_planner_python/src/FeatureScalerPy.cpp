/*
 * FeatureScalerPy.cpp
 *
 *  Created on: 23.09.2015
 *      Author: Ulrich Schwesinger
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/FeatureScaler.hpp>

using namespace boost::python;
using namespace planning2d;
using namespace prob_planner;

void exportFeatureScaler() {

  def("scaleSingleFeature", (void (*) (RawFeature::Ptr, const std::vector<ContinuousScene::Ptr>&))&scale);
  def("scaleAllFeatures", (void (*) (FeatureContainer&, const std::vector<ContinuousScene::Ptr>&))&scale);

} /* void exportOptAgent() */


