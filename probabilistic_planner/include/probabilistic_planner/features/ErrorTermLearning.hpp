/*
 * ErrorTermLearning.hpp
 *
 *  Created on: 27.07.2015
 *      Author: sculrich
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_ERRORTERMLEARNING_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_ERRORTERMLEARNING_HPP_

// standard includes
#include <vector>
#include <utility> // std::pair
#include <memory> // std::shared_ptr

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>

// self includes
#include "FeatureContainer.hpp"
#include "../Support.hpp"

// other package includes
#include <planner_interfaces/Support.hpp>

// aslam includes
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/JacobianContainer.hpp>
#include <aslam/backend/util/CommonDefinitions.hpp>
#include <aslam/backend/util/utils.hpp>

namespace aslam {
  namespace backend {
    class SamplerBase;
  }
}

namespace prob_planner {

class ProbabilisticLearner;

class ErrorTermLearning : public aslam::backend::ScalarNonSquaredErrorTerm {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PLANNING_2D_POINTER_TYPEDEFS(ErrorTermLearning);
  friend class ProbabilisticLearner;

  struct Options {
    Options() { }
    Options(const sm::value_store::ValueStore& vpt);

    /// \brief Test the sanity of the parameters
    void validate() const;

    /// \brief How many iterations for burn-in (reaching stationary distribution)
    std::size_t nMcmcStepsBurnIn = 0;

    /// \brief How many samples to take in the statistical mean.
    ///        Note that you need more samples depending on the autocorrelation
    ///        time of the Markov chain. E.g. if the autocorrelation time is 100
    ///        samples, you should take 100 times more samples for mean computation.
    std::size_t nMcmcSamplesForMean = 200;
    /// \brief Thinning interval, only each \p nThin sample will be kept
    std::size_t nThin = 1;
    bool storeSamples = false;
    bool initializeAtMode = true;
    bool resetToDemonstration = true; /// \brief Reset to demonstration before computing feature expectations
  };

  struct FeatureInfo {
    FeatureInfo() { demonstration.resize(0,1); expectation.resize(0,1); gradient.resize(0,1); }
    std::string name;
    Eigen::VectorXd weights;
    Eigen::VectorXd demonstration;
    Eigen::VectorXd expectation;
    boost::optional<Eigen::VectorXd> mode;
    Eigen::VectorXd gradient;
    std::size_t numActiveDesignVariables = 0;
    RawFeature::Ptr feature; // pointer to corresponding feature, will not be serialized!
    template<class Archive>
    inline void serialize(Archive & ar, const unsigned int version);
  };

 public:

  ErrorTermLearning(FeatureContainer& features,
                    ContinuousScene& scene,
                    const std::string& name,
                    const Options& options = Options(),
                    boost::shared_ptr<aslam::backend::SamplerBase> sampler = nullptr);
  virtual ~ErrorTermLearning();

  void setName(const std::string& name) { if (!name.empty()) _name = "_" + name; }
  const Options& getOptions() const { return _options; }
  void setOptions(const Options& options) { _options.validate(); _options = options; }

  const std::vector<ContinuousScene::Ptr>& getSamples() const { return _samples; }
  double getEffectiveSampleSize() const { return _effectiveSampleSize; }
  double getIntegratedAutocorrelationTime() const { return _integratedAutocorrelationTime; }

  const std::vector<FeatureInfo>& featureInfo() const { return _featureInfo; }

  ContinuousScene::ConstPtr getMode() const { return _mode; }

 protected:
  virtual double evaluateErrorImplementation() override;
  virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & outJacobians) override;

 private:
  void computeFeatureExpectations();
  void addFeatureValues(size_t /* threadId */, size_t startIdx, size_t endIdx, Eigen::VectorXd& fval);

 private:
  boost::shared_ptr<aslam::backend::SamplerBase> _sampler;

  /// \brief debug data
  std::vector<ContinuousScene::Ptr> _samples;
  ContinuousScene::Ptr _mode = nullptr;
  double _effectiveSampleSize = std::numeric_limits<double>::signaling_NaN();
  double _integratedAutocorrelationTime = std::numeric_limits<double>::signaling_NaN();

  const FeatureContainer& _featureContainer;
  std::vector<FeatureInfo> _featureInfo; /// \brief Association of active features, demonstration and expectation values.
  ContinuousScene& _scene; /// \brief The scene passed into the constructor, this will be modified during sampling
  aslam::backend::utils::DesignVariableState _demonstration; /// \brief The initial design variables of the scene passed into the constructor

  Options _options; /// \brief options of this class

  std::string _name; /// \brief The name of the error term, used for printouts

};



template<class Archive>
inline void ErrorTermLearning::FeatureInfo::serialize(Archive & ar, const unsigned int /*version*/) {
  ar & name;
  ar & weights;
  ar & demonstration;
  ar & expectation;
  ar & mode;
  ar & gradient;
  ar & numActiveDesignVariables;
}
std::ostream& operator<<(std::ostream& os, const std::vector<ErrorTermLearning::FeatureInfo>& features);

}

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_ERRORTERMLEARNING_HPP_ */
