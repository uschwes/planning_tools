/*
 * ErrorTermLearning.cpp
 *
 *  Created on: 10.08.2015
 *      Author: Ulrich Schwesinger
 */

#include <probabilistic_planner/features/ErrorTermLearning.hpp>
#include <probabilistic_planner/sampling/Autocorrelation.hpp>

#include <aslam/backend/util/ThreadedRangeProcessor.hpp>
#include <aslam/backend/SamplerMetropolisHastings.hpp>
#include <aslam/backend/SamplerHybridMcmc.hpp>

#include <aslam/backend/OptimizerRprop.hpp>
#include <aslam/backend/OptimizerBFGS.hpp>

using namespace std;
using namespace planning2d;
using namespace aslam::backend;

namespace prob_planner {

ErrorTermLearning::Options::Options(const sm::value_store::ValueStore& vpt)
{
  nMcmcStepsBurnIn = vpt.getInt("nMcmcStepsBurnIn", nMcmcStepsBurnIn);
  nMcmcSamplesForMean = vpt.getInt("nMcmcSamplesForMean", nMcmcSamplesForMean);
  nThin = vpt.getInt("nThin", nThin);
  storeSamples = vpt.getBool("storeSamples", storeSamples);
  initializeAtMode = vpt.getBool("initializeAtMode", initializeAtMode);
  this->validate();
}

void ErrorTermLearning::Options::validate() const
{
  SM_ASSERT_GT(planning2d::ParameterException, nThin, 0, "Thinning interval has to be larger or equal to one");
}

/**
 * Constructor
 * @param negativeLogDensity The negative log density formulation
 * @param features The collection of features
 * @param demonstration The demonstrations
 * @param fValDemonstration
 */
ErrorTermLearning::ErrorTermLearning(FeatureContainer& features,
                                     ContinuousScene& scene,
                                     const std::string& name,
                                     const Options& options /*= Options()*/,
                                     boost::shared_ptr<aslam::backend::SamplerBase> sampler /*= nullptr*/) :
  ScalarNonSquaredErrorTerm(),
  _sampler(sampler),
  _featureContainer(features),
  _scene(scene),
  _options(options),
  _name("_" + name) {

  _options.validate();

  // copy scene
  for (const auto& oa : scene.getOptAgentContainer()) {
    _demonstration.save(const_cast<Trajectory::Spline&>(oa.second.trajectory().getSpline()).getDesignVariables());
  }

  // No additional scaling
  this->setWeight(1.0);

  // Add the feature weights of all features as design variables of this error term
  // Also build an index from feature to its demonstration
  vector<DesignVariable*> dvs;
  dvs.reserve(features.getContainer().size());
  _featureInfo.reserve(features.getContainer().size());
  for (auto feature : _featureContainer.getContainer()) {

    // determine whether any of the feature's design variables is active
    std::size_t numActiveDesignVariables = 0;
    for (const auto& dv : feature->getWeights()) {
      if (dv.isActive())
        ++numActiveDesignVariables;
    }

    // Compute the feature values from the demonstration, afterwards we can modify the scene
    if (numActiveDesignVariables > 0) { // only compute the demonstrations if any design variable is active for this feature
      FeatureInfo fi;
      fi.numActiveDesignVariables = numActiveDesignVariables;
      fi.name = feature->nameWithOptAgentType();
      fi.feature = feature;
      fi.demonstration = feature->evaluate(_scene);
      _featureInfo.push_back(fi); // only keep the active ones
      SM_ASSERT_TRUE( RuntimeException, fi.demonstration.allFinite(), "fDem(" << feature->nameWithOptAgentType() << "): " << fi.demonstration.transpose());
      SM_DEBUG_STREAM_NAMED("sampling", "ErrorTermLearning" << _name << ": Demonstration for feature \"" << feature->nameWithOptAgentType() << "\": " << fi.demonstration.transpose());
    } else {
      SM_DEBUG_STREAM_NAMED("sampling", "ErrorTermLearning" << _name << ": \"" << feature->nameWithOptAgentType() << "\" (deactivated)");
    }

    for (size_t i=0; i<feature->numWeights(); i++)
      dvs.push_back(feature->getWeightAsDesignVariable(i));

  }
  this->setDesignVariables(dvs);

  // Create the negative log density formulation
  RawFeature::OptimizationProblemPtr negativeLogDensity(new OptimizationProblem());

  // Add the spline design variables to the probability distribution. Do not activate here,
  // has to be done externally by the user
  for (auto& agent : _scene.getOptAgentContainer())
    agent.second.trajectory().addDesignVariables(*negativeLogDensity, false);

  // Configure the negative log density formulation
  for (auto feature : _featureContainer.getContainer())
    feature->addErrorTerms(_scene, *negativeLogDensity);

  // Construct a default sampler if none is given
  if (sampler == nullptr) {

    aslam::backend::SamplerMetropolisHastingsOptions options;
    options.transitionKernelSigma = 0.02;
    _sampler.reset(new aslam::backend::SamplerMetropolisHastings(options));

    // Create the MCMC sampler
  //  aslam::backend::SamplerHybridMcmcOptions options;
  //  options.delta = 0.01;
  //  options.nHamiltonianSteps = 100;
  //  options.nThreads = 4;
  //  _sampler.reset(new aslam::backend::SamplerHybridMcmc(options));

  }

  _sampler->setNegativeLogDensity(negativeLogDensity);
  _sampler->checkNegativeLogDensitySetup();
  _sampler->initialize();

}


/// \brief Destructor
ErrorTermLearning::~ErrorTermLearning() {

}


/// \brief Evaluate the error term and return the scalar error \f$ e \f$
double ErrorTermLearning::evaluateErrorImplementation() {
  SM_THROW(NoImplementationException, "method not intended to be used");
}


/// \brief Evaluate the Jacobians for \f$ e \f$
void ErrorTermLearning::evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) {

  // Compute the feature expectations from the current scene
  this->computeFeatureExpectations();

  // Assemble the gradient
  Timer timer("ErrorTermLearning: Compute---Gradient", false);
  for (auto& fi : _featureInfo) {
    fi.gradient = fi.demonstration - fi.expectation;
    for (int i = 0; i < fi.gradient.rows(); i++)
      outJacobians.add(fi.feature->getWeightAsDesignVariable(i), fi.gradient.row(i));
  }
  SM_INFO_STREAM("ErrorTermLearning" << _name << ": Jacobian computation complete" << endl << _featureInfo);

}


/**
 * Compute the feature expectations from the current scene
 * @param[out] featureExpectations Vector of multidimensional feature expectations.
 *                                 Each entry i holds the feature expectation for feature i stored in \var _featureDemonstrationPairs
 */
void ErrorTermLearning::computeFeatureExpectations() {

  Timer timer("ErrorTermLearning: Compute---Feature Expectations", false);
  SM_INFO_STREAM("ErrorTermLearning" << _name << ": Computing feature expectations...");

  // Reset scene to demonstration
  if (_options.resetToDemonstration) { _demonstration.restore(); }

  // initialize feature expectations
  for (auto& fi : _featureInfo) {
    fi.expectation.resize(fi.feature->numWeights());
    fi.expectation.setZero();
    fi.weights = fi.feature->getCurrentWeightsVector();
  }

  // Do not use cached values in the sampler here, since the feature weights might have changed
  _sampler->forceRecomputationNegLogDensity();

  // initialize the sampler at the maximum likelihood state
  _mode.reset();
  if (_options.initializeAtMode) {
    SM_DEBUG_STREAM_NAMED("sampling", "ErrorTermLearning" << _name << ": Initializing MCMC chain at mode of distribution");
    OptimizerOptionsBFGS options;
    options.maxIterations = 1000;
    options.convergenceGradientNorm = 1e-3;
    options.convergenceDeltaX = 1e-12;
    options.convergenceDeltaError = 1e-6;
    OptimizerBFGS optimizer(options);
    boost::shared_ptr<OptimizationProblem> problem(new OptimizationProblem());
    _scene.addDesignVariables(*problem, false);
    _featureContainer.addErrorTerms(_scene, *problem);
    optimizer.setProblem(problem);
    optimizer.optimize();
    for (auto& fi : _featureInfo) {
      SM_ASSERT_NOTNULL_DBG(planning2d::RuntimeException, fi.feature, "");
      fi.mode = fi.feature->evaluate(_scene);
    }
    _mode.reset(new ContinuousScene());
    _scene.copy(*_mode);
  }

  // Burn in the MCMC sampler
  SM_INFO_STREAM("ErrorTermLearning" << _name << ": Running burn-in of MCMC sampler with " << _options.nMcmcStepsBurnIn << " samples...");
  _sampler->run(_options.nMcmcStepsBurnIn); // burn-in phase

  // Run the sampler until we get a non-zero acceptance rate
  if (_options.nMcmcSamplesForMean > 1) {
    size_t cnt = 0;
    while(_sampler->statistics().getWeightedMeanAcceptanceProbability() < 1e-3 && cnt < _options.nMcmcSamplesForMean) {
      SM_INFO_STREAM_THROTTLE(10.0, "ErrorTermLearning" << _name << ": Tuning acceptance rate of MCMC sampler "
          "(actual: " << _sampler->statistics().getWeightedMeanAcceptanceProbability() << ")...");
      _sampler->run(1);
      cnt++;
    }
  }

  // Draw samples
  _samples.clear();
  _effectiveSampleSize = 0.0;
  if (_options.storeSamples) { _samples.reserve(_options.nMcmcSamplesForMean); }

  size_t nSamplesGenerated = 0;
  _effectiveSampleSize = std::numeric_limits<double>::signaling_NaN();
  _integratedAutocorrelationTime = std::numeric_limits<double>::signaling_NaN();
  while (true) {

    // store sample if option is activated
    if (_options.storeSamples) {
      _samples.emplace_back(new ContinuousScene());
      _scene.copy(*_samples.back());
      _effectiveSampleSize = sampling::computeEffectiveSampleSize(_samples, _integratedAutocorrelationTime);
    }

    nSamplesGenerated++;

    // Note: Threading does not save a lot here
//    boost::function<void(size_t, size_t, size_t, Eigen::VectorXd&)> job(boost::bind(&ErrorTermLearning::addFeatureValues, this, _1, _2, _3, _4));
//    util::runThreadedFunction(job, featureExpectations.size(), featureExpectations);

    for (auto& fi : _featureInfo) {
      auto fval = fi.feature->evaluate(_scene);
      SM_FINE_STREAM_NAMED("sampling", "ErrorTermLearning" << _name << ": value of \"" << fi.feature->nameWithOptAgentType() << "\" for sample number " << nSamplesGenerated << ": " << fval.transpose());
      fi.expectation += (fval - fi.expectation) / nSamplesGenerated;   // recursive mean computation: mu_k = mu_k-1 + 1/k * (x_k - mu_k-1)
      SM_ASSERT_TRUE( RuntimeException, fi.expectation.allFinite(), "Expectation for feature " << fi.feature->nameWithOptAgentType() << ": " << fi.expectation.transpose() );
    }

    {
      std::ostringstream os;
      os << std::fixed << setprecision(2) << "ErrorTermLearning" << _name << ": " << static_cast<double>(nSamplesGenerated)/static_cast<double>(_options.nMcmcSamplesForMean)*100.0 <<
          "% (" << nSamplesGenerated << ", " << _options.nMcmcSamplesForMean << ")" << endl <<
          "EMA acceptance rate: " << _sampler->statistics().getWeightedMeanAcceptanceProbability();
      if (_options.storeSamples)
        os << endl << "ACT: " << _integratedAutocorrelationTime << endl << "ESS: " << _effectiveSampleSize;
      else
        os << endl << "ACT/ESS: activate options \"storeSamples\"";
      os << endl << _featureInfo;
      SM_INFO_STREAM_THROTTLE(10.0, os.str());
    }

    if (nSamplesGenerated >= _options.nMcmcSamplesForMean)
      break;

    // Note: This is the slowest part
    _sampler->run(_options.nThin);

  }

}


ostream& operator<<(ostream& os, const vector<ErrorTermLearning::FeatureInfo>& features) {

  using namespace Eigen;
  static IOFormat fmt(StreamPrecision, DontAlignCols, ",","\n","","","","");

  size_t maxLenName = 0;
  for (auto& fi : features)
    maxLenName = fi.feature->nameWithOptAgentType().size() > maxLenName ? fi.feature->nameWithOptAgentType().size() : maxLenName;

  maxLenName += 5;
  os << setw(maxLenName + 65) << setfill('-') << "" << setfill(' ') << endl;
  os << fixed << setprecision(4) << setw(maxLenName) << left <<
      "Feature" << setw(15) << "Expectation" << setw(15) << "Demonstration" << setw(15) <<
      "Mode" << setw(12) << "Weight(s)" << "Gradient" << endl;
  os << setw(maxLenName + 65) << setfill('-') << "" << setfill(' ') << endl;

  string sep = "\n";
  size_t cnt = 0;
  std::ostringstream ss; // Note: Something goes wrong with stream formatting when Eigen stream operator is used directly
  for (auto& fi : features) {
    if (cnt == features.size() - 1) sep.clear();
    os << setw(maxLenName) << fi.feature->nameWithOptAgentType();
    ss.str("");
    ss << fi.expectation.transpose().format(fmt);
    os << setw(15) << ss.str();
    ss.str("");
    ss << fi.demonstration.transpose().format(fmt);
    os << setw(15) << ss.str();
    ss.str("");
    if (fi.mode)
      ss << fi.mode.get().transpose().format(fmt);
    else
      ss << "";
    os << setw(15) << ss.str();
    ss.str("");
    ss << fi.weights.transpose().format(fmt);
    os << setw(12) << ss.str();
    ss.str("");
    ss << fi.gradient.transpose().format(fmt);
    os << ss.str();
    os << sep;
    cnt++;
  }
  return os;
}

} /* namespace prob_planner */
