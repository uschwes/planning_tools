/*
 * AutocorrelationPy.cpp
 *
 *  Created on: 29.11.2015
 *      Author: Ulrich Schwesinger
 */

#include <boost/python.hpp>
#include <numpy_eigen/boost_python_headers.hpp>

#include <probabilistic_planner/sampling/Autocorrelation.hpp>

using namespace std;
using namespace boost::python;
using namespace prob_planner::sampling;

void exportAutocorrelation()
{
  def("computeAutocorrelation", (Eigen::MatrixXd (*) (const Eigen::MatrixXd&))&computeAutocorrelation);
  def("computeAutocorrelation", (Eigen::MatrixXd (*) (const vector<prob_planner::ContinuousScene::Ptr>&))&computeAutocorrelation);
  def("computeIntegratedAutocorrelationTime", (Eigen::RowVectorXd (*) (const Eigen::MatrixXd&))&computeIntegratedAutocorrelationTime);
  def("computeIntegratedAutocorrelationTime", (Eigen::RowVectorXd (*) (const vector<prob_planner::ContinuousScene::Ptr>&))&computeIntegratedAutocorrelationTime);
  def("computeEffectiveSampleSize", (double (*) (const Eigen::MatrixXd&))&computeEffectiveSampleSize);
  def("computeEffectiveSampleSize", (double (*) (const Eigen::MatrixXd&, double&))&computeEffectiveSampleSize);
  def("computeEffectiveSampleSize", (double (*) (const vector<prob_planner::ContinuousScene::Ptr>&))&computeEffectiveSampleSize);
  def("computeEffectiveSampleSize", (double (*) (const vector<prob_planner::ContinuousScene::Ptr>&, double&))&computeEffectiveSampleSize);
}
