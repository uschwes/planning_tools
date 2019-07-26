/*
 * DensityComputations.cpp
 *
 *  Created on: 13.07.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#include <probabilistic_planner/support/DensityComputations.hpp>

#include <boost/shared_ptr.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/util/ProblemManager.hpp>

namespace prob_planner {

double computeNegativeLogDensity(
    const RawFeature::ConstPtr& feature,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/) {

  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
  const_cast<ContinuousScene&>(scene).addDesignVariables(*problem, false);
  feature->addErrorTerms(scene, *problem);
  double negloglik = 0.0;

  if (problem->numTotalErrorTerms() > 0) {
    ProblemManager pm;
    pm.setProblem(problem);
    pm.initialize();
    negloglik = pm.evaluateError(numThreads);
  }

  return negloglik;
}

double computeNegativeLogDensity(
    const FeatureContainer& features,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/) {

  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
  const_cast<ContinuousScene&>(scene).addDesignVariables(*problem, false);
  for (const auto& feature : features.getContainer()) {
    feature->addErrorTerms(scene, *problem);
  }
  double negloglik = 0.0;

  if (problem->numTotalErrorTerms() > 0) {
    ProblemManager pm;
    pm.setProblem(problem);
    pm.initialize();
    negloglik = pm.evaluateError(numThreads);
  }

  return negloglik;
}

RowVectorType computeGradientNegativeLogDensity(
    const RawFeature::ConstPtr& feature,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/,
    const bool applyDvScaling /*= false*/) {

  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
  const_cast<ContinuousScene&>(scene).addDesignVariables(*problem, false);
  feature->addErrorTerms(scene, *problem);
  RowVectorType J = RowVectorType::Zero(1, scene.numActiveSplineParameters());

  if (problem->numTotalErrorTerms() > 0) {
    ProblemManager pm;
    pm.setProblem(problem);
    pm.initialize();
    pm.computeGradient(J, numThreads, false, applyDvScaling, true);
  }

  return J;
}

RowVectorType computeGradientNegativeLogDensity(
    const FeatureContainer& features,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/,
    const bool applyDvScaling /*= false*/) {

  boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
  const_cast<ContinuousScene&>(scene).addDesignVariables(*problem, false);
  for (const auto& feature : features.getContainer()) {
    feature->addErrorTerms(scene, *problem);
  }
  RowVectorType J = RowVectorType::Zero(1, scene.numActiveSplineParameters());

  if (problem->numTotalErrorTerms() > 0) {
    ProblemManager pm;
    pm.setProblem(problem);
    pm.initialize();
    pm.computeGradient(J, numThreads, false, applyDvScaling, true);
  }

  return J;
}

} /* namespace prob_planner */
