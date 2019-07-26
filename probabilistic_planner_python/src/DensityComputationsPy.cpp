/*
 * SupportPy.cpp
 *
 *  Created on: 12.07.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/support/DensityComputations.hpp>

using namespace prob_planner;
using namespace aslam::backend;
using namespace boost::python;

void exportDensityComputations() {

  def("computeNegativeLogDensity", (double (*) (const RawFeature::ConstPtr&, const ContinuousScene&, const size_t))&computeNegativeLogDensity, (boost::python::arg("numThreads") = 1, boost::python::arg("applyDvScaling") = false),
      "Computes the gradient of the feature wrt. the spline control vertices");

  def("computeNegativeLogDensity", (double (*) (const FeatureContainer&, const ContinuousScene&, const size_t))&computeNegativeLogDensity, (boost::python::arg("numThreads") = 1, boost::python::arg("applyDvScaling") = false),
      "Computes the gradient of the feature wrt. the spline control vertices");

  def("computeGradientNegativeLogDensity", (RowVectorType (*) (const RawFeature::ConstPtr&, const ContinuousScene&, const size_t, const bool))&computeGradientNegativeLogDensity, (boost::python::arg("numThreads") = 1, boost::python::arg("applyDvScaling") = false),
      "Computes the gradient of the feature wrt. the spline control vertices");

  def("computeGradientNegativeLogDensity", (RowVectorType (*) (const FeatureContainer&, const ContinuousScene&, const size_t, const bool))&computeGradientNegativeLogDensity, (boost::python::arg("numThreads") = 1, boost::python::arg("applyDvScaling") = false),
      "Computes the gradient of the feature wrt. the spline control vertices");

}
