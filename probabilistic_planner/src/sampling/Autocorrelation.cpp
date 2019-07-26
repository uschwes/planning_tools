/*
 * Autocorrelation.cpp
 *
 *  Created on: 25.11.2015
 *      Author: Ulrich Schwesinger
 */

// standard
#include <cmath>

// self
#include <probabilistic_planner/sampling/Autocorrelation.hpp>

// planner interfaces
#include <planner_interfaces/Exceptions.hpp>

// Eigen
#include <unsupported/Eigen/FFT>

using namespace std;

namespace prob_planner {
namespace sampling {

namespace details {

Eigen::MatrixXd toMatrix(const vector<ContinuousScene::Ptr>& samples)
{
  SM_ASSERT_FALSE(planning2d::FunctionInputException, samples.empty(), "Cannot determine dimensionality without any samples");
  auto s0 = samples.front()->activeSplineParameters();
  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(samples.size(), s0.size());
  size_t cnt = 0;
  for (const auto& sample : samples)
    p.row(cnt++) = sample->activeSplineParameters();
  return p;
}

} /* namespace details */

Eigen::MatrixXd computeAutocorrelation(const Eigen::MatrixXd& samples)
{
  Eigen::FFT<double> fft;

  // normalize
  Eigen::MatrixXd samplesNormed = samples;
  for (int k=0; k<samples.cols(); ++k) // TODO: should be possible as column-wise operation
    samplesNormed.col(k) -= Eigen::VectorXd::Constant(samples.rows(), 1, samples.col(k).mean());

  // zero-padding
  const size_t sz = samplesNormed.rows();
  const size_t szPadded2 = 2*pow(2, ceil(log(sz)/log(2)));

  Eigen::MatrixXd samplesPadded = Eigen::MatrixXd::Zero(szPadded2, samplesNormed.cols());
  samplesPadded.topRows(samplesNormed.rows()) = samplesNormed;

  // forward FFT
  Eigen::MatrixXcd freqc(samplesPadded.rows(), samplesPadded.cols());
  for (int k=0; k<samplesPadded.cols(); ++k) // it works to iterate over the columns
    fft.fwd(freqc.col(k).data() /*dst*/, samplesPadded.col(k).data() /*src*/, samplesPadded.rows());

  // Norm of the complex signal
  Eigen::MatrixXcd freq = freqc.cwiseProduct(freqc.conjugate());

  // inverse FFT
  Eigen::MatrixXd acorr(freq.rows(), freq.cols());
  for (int k=0; k<freq.cols(); ++k) // it works to iterate over the columns
    fft.inv(acorr.col(k).data() /*dst*/, freq.col(k).data() /*src*/, freq.rows());

  // Remove zero-padded values
  acorr.conservativeResize(sz, Eigen::NoChange);

  return acorr;
}


Eigen::RowVectorXd computeIntegratedAutocorrelationTime(const Eigen::MatrixXd& samples)
{
  Eigen::MatrixXd corr = computeAutocorrelation(samples);

  Eigen::RowVectorXd t = Eigen::RowVectorXd::Ones(1, samples.cols());
  for (int col=0; col<corr.cols(); ++col) {
    for (int row=1; row<corr.rows(); ++row) {
//      if (corr(row-1,col) + corr(row,col) < 0)
//        break;
      if (corr(row,col) < 0 && corr(row-1,col) > 0)
        break;
      t[col] += 2*corr(row-1,col)/corr(0, col);
    }
  }

  return t;
}

double computeEffectiveSampleSize(const Eigen::MatrixXd& samples)
{
  double integratedAutocorrelationTime;
  return computeEffectiveSampleSize(samples, integratedAutocorrelationTime);
}

double computeEffectiveSampleSize(const Eigen::MatrixXd& samples, double& integratedAutocorrelationTime)
{
  integratedAutocorrelationTime = computeIntegratedAutocorrelationTime(samples).maxCoeff();
  return samples.rows()/integratedAutocorrelationTime;
}

Eigen::MatrixXd computeAutocorrelation(const vector<ContinuousScene::Ptr>& samples)
{
  return computeAutocorrelation(details::toMatrix(samples));
}

Eigen::RowVectorXd computeIntegratedAutocorrelationTime(const vector<ContinuousScene::Ptr>& samples)
{
  return computeIntegratedAutocorrelationTime(details::toMatrix(samples));
}

double computeEffectiveSampleSize(const vector<ContinuousScene::Ptr>& samples)
{
  return computeEffectiveSampleSize(details::toMatrix(samples));
}

double computeEffectiveSampleSize(const vector<ContinuousScene::Ptr>& samples, double& integratedAutocorrelationTime)
{
  return computeEffectiveSampleSize(details::toMatrix(samples), integratedAutocorrelationTime);
}

} /* namespace sampling*/
} /* namespace prob_planner */
