/*
 * ProbabilisticLearner.cpp
 *
 *  Created on: 29.07.2015
 *      Author: sculrich
 */

// aslam includes
#include <aslam/backend/OptimizationProblem.hpp>

// self includes
#include <probabilistic_planner/Support.hpp>
#include <probabilistic_planner/ProbabilisticLearner.hpp>
#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/features/FeatureContainer.hpp>

namespace prob_planner {

using namespace std;
using namespace planning2d;
using namespace aslam::backend;

ProbabilisticLearner::Options::Options() {
  check();
}

ProbabilisticLearner::Options::Options(const sm::value_store::PropertyTreeValueStore& vpt) :
    rpropOptions(aslam::backend::OptimizerOptionsRprop(vpt)),
    etOptions(ErrorTermLearning::Options(vpt)),
    hybridMonteCarloOptions(aslam::backend::SamplerHybridMcmcOptions(vpt)),
    metropolisHastingsOptions(aslam::backend::SamplerMetropolisHastingsOptions(vpt)) {

  check();

}

void ProbabilisticLearner::Options::check() const {
  rpropOptions.check();
  metropolisHastingsOptions.check();
  hybridMonteCarloOptions.check();
}



ProbabilisticLearner::ProbabilisticLearner(FeatureContainer& features, const Options& options /*= Options()*/) :
    _features(features),
    _options(options) {

}

ProbabilisticLearner::~ProbabilisticLearner() {

}

void ProbabilisticLearner::run() {

  // Run the RPROP optimizer
  Timer t("ProbabilisticLearner: run", false);
  SM_INFO_STREAM("ProbabilisticLearner: Optimizing feature weights with " << _options.etOptions.nMcmcSamplesForMean << " samples.");
  _optimizer->optimize();
}

void ProbabilisticLearner::addDemonstrations(std::vector<ContinuousScene::Ptr>& demonstrations) {
  this->initialize(demonstrations);
}

void ProbabilisticLearner::setErrorTermOptions(const ErrorTermLearning::Options& options) {
  for (auto& et : _errorTerms)
    et->setOptions(options);
}

void ProbabilisticLearner::setOptimizerRpropOptions(const OptimizerOptionsRprop& options) {
  _optimizer->setOptions(options);
}

void ProbabilisticLearner::initialize(std::vector<ContinuousScene::Ptr>& demonstrations) {

  Timer t("ProbabilisticLearner: initialize", false);

  // Create an optimization problem for the RPROP algorithm.
  boost::shared_ptr<OptimizationProblem> negLogLikelihood(new OptimizationProblem());

  // The RPROP optimization problem must have the feature weights as design variables.
  size_t nActiveDv = 0;
  for (auto feature : _features.getContainer()) {
    for (size_t i=0; i<feature->numWeights(); i++) {
      Scalar* dv = feature->getWeightAsDesignVariable(i);
      // Add all active and inactive feature weight design variables.
      // The optimizer will only tune the active ones
      negLogLikelihood->addDesignVariable(dv, false);
      nActiveDv += dv->isActive() ? 1 : 0;
    }
  }
  SM_ASSERT_GT( InitializationException, nActiveDv, 0, "None of the feature weights is activated for learning.");

  // Create samplers
  _samplers.clear();
  for (size_t i=0; i<demonstrations.size(); i++) {
    switch (_options.samplerType) {
      case Options::METROPOLIS_HASTINGS:
        _samplers.emplace_back(new SamplerMetropolisHastings(_options.metropolisHastingsOptions));
        break;
      case Options::HYBRID_MONTE_CARLO:
        _samplers.emplace_back(new SamplerHybridMcmc(_options.hybridMonteCarloOptions));
        break;
    }
  }

  // Create a learning error term for each demonstration. Since we sum over demonstrations, we can
  // simply create one error term per demonstration scene
  size_t cnt = 0;
  _errorTerms.clear();
  for (auto scenePtr : demonstrations) {
    SM_ASSERT_TRUE( NullPointerException, scenePtr != nullptr, "");
    _errorTerms.emplace_back(new ErrorTermLearning(_features, *scenePtr, "Scene" + boost::lexical_cast<string>(cnt), _options.etOptions, _samplers[cnt]));
    negLogLikelihood->addErrorTerm(_errorTerms.back());
    cnt++;
  }

  // setup optimizer
  _optimizer.reset(new OptimizerRprop(_options.rpropOptions));
  _optimizer->setProblem(negLogLikelihood);
  _optimizer->checkProblemSetup();
  _optimizer->initialize();
}

const std::vector<ContinuousScene::Ptr>& ProbabilisticLearner::getSamples(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _errorTerms.size(), "");
  SM_WARN_STREAM_COND(!_errorTerms[i]->getOptions().storeSamples, "ProbabilisticLearner: In order to store samples, set the options \"storeSamples\" in ErrorTermlearning::Options to true!");
  return _errorTerms[i]->getSamples();
}

ErrorTermLearning::ConstPtr ProbabilisticLearner::getErrorTerm(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _errorTerms.size(), "");
  return _errorTerms[i];
}

const std::vector<ErrorTermLearning::FeatureInfo>& ProbabilisticLearner::getFeatureInfo(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _errorTerms.size(), "");
  return _errorTerms[i]->featureInfo();
}

double ProbabilisticLearner::getEffectiveSampleSize(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _errorTerms.size(), "");
  return _errorTerms[i]->getEffectiveSampleSize();
}

double ProbabilisticLearner::getIntegratedAutocorrelationTime(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _errorTerms.size(), "");
  return _errorTerms[i]->getIntegratedAutocorrelationTime();
}

} /* namespace prob_planner  */
