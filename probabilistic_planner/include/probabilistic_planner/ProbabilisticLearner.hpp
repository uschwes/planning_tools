/*
 * ProbabilisticLearner.hpp
 *
 *  Created on: 29.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_PROBABILISTICLEARNER_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_PROBABILISTICLEARNER_HPP_

#include <planner_interfaces/Support.hpp>

#include <aslam/backend/OptimizerRprop.hpp>
#include <aslam/backend/SamplerMetropolisHastings.hpp>
#include <aslam/backend/SamplerHybridMcmc.hpp>

#include <probabilistic_planner/features/ErrorTermLearning.hpp>

// Forward declarations
namespace prob_planner {
  class FeatureContainer;
  class ContinuousScene;
}

namespace prob_planner {

/**
 * @class ProbabilisticLearner
 * Class optimizing the log likelihood of the demonstrations by adjusting the feature weights
 */
class ProbabilisticLearner {
 public:
  PLANNING_2D_POINTER_TYPEDEFS(ProbabilisticLearner);

  struct Options {
    Options();
    Options(const sm::value_store::PropertyTreeValueStore& vpt);
    enum SamplerType { METROPOLIS_HASTINGS, HYBRID_MONTE_CARLO };

    aslam::backend::OptimizerOptionsRprop rpropOptions;
    ErrorTermLearning::Options etOptions;
    aslam::backend::SamplerHybridMcmcOptions hybridMonteCarloOptions;
    aslam::backend::SamplerMetropolisHastingsOptions metropolisHastingsOptions;
    SamplerType samplerType = HYBRID_MONTE_CARLO;

    void check() const;
  };

 public:
  /**
   * Constructor
   * @param features Feature container, you have to activate the desired features for learning before
   * @param options Options for the learner
   */
  ProbabilisticLearner(FeatureContainer& features, const Options& options = Options());

  /// @brief Destructor
  ~ProbabilisticLearner();

  /// @brief Add demonstrations to the problem
  void addDemonstrations(std::vector<ContinuousScene::Ptr>& demonstrations);

  /// @brief Run the learning procedure on the demonstrations previously supplied via addDemonstrations()
  void run();

  /// @brief The number of samplers, equal to the number of error terms and demonstrations
  std::size_t nSamplers() const { return _samplers.size(); }

  /// @brief Get the valid samples generated during the last RPROP optimization step of the i-th sampler.
  ///        In order to generate samples, the options ErrorTermLearning::Options::storeSamples must be set to true.
  const std::vector<ContinuousScene::Ptr>& getSamples(const std::size_t i) const;

  ///@brief Getter for the \var i-th error term
  ErrorTermLearning::ConstPtr getErrorTerm(const std::size_t i) const;

  /// @brief Get the samplers
  const std::vector< boost::shared_ptr<aslam::backend::SamplerBase> >& getSamplers() const { return _samplers; }

  /// @brief Getter for the optimizer
  const aslam::backend::OptimizerRprop& optimizer() const { return *_optimizer; }

  ///@brief Getter for the feature information for the i-th demonstration
  const std::vector<ErrorTermLearning::FeatureInfo>& getFeatureInfo(const std::size_t i) const;
  ///@brief Getter for the effective sample size for the i-th demonstration
  double getEffectiveSampleSize(const std::size_t i) const;
  ///@brief Getter for the integrated autocorrelation time for the i-th demonstration
  double getIntegratedAutocorrelationTime(const std::size_t i) const;

  ///@brief Const getter for options
  const Options& getOptions() const { return _options; }

  ///@brief Setter for options
  void setErrorTermOptions(const ErrorTermLearning::Options& options);
  void setOptimizerRpropOptions(const aslam::backend::OptimizerOptionsRprop& options);

 private:
  /// @brief initialization everything regarding the demonstrations
  void initialize(std::vector<ContinuousScene::Ptr>& demonstrations);

 private:
  FeatureContainer& _features;
  boost::shared_ptr<aslam::backend::OptimizerRprop> _optimizer; /// @brief the optimizer adjusting the feature weights
  std::vector< boost::shared_ptr<aslam::backend::SamplerBase> > _samplers; /// @brief Create a separate sampler instance for each error term
  std::vector<ErrorTermLearning::Ptr> _errorTerms; /// @brief one error term per demonstration

  Options _options;

}; /* class ProbabilisticLearner */

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_PROBABILISTICLEARNER_HPP_ */
