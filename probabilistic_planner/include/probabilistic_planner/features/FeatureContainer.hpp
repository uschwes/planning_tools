/*
 * FeatureContainer.hpp
 *
 *  Created on: Jul 28, 2015
 *      Author: pfmark
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURECONTAINER_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURECONTAINER_HPP_

#include <memory>

#include <sm/logging.hpp>

#include <probabilistic_planner/features/RawFeature.hpp>

namespace prob_planner {

class FeatureContainer;
class FeatureSingletonProduct;

class FeatureFactoryBase {
 public:
  virtual RawFeature::Ptr create(const sm::value_store::ValueStore& vpt) = 0;
  virtual RawFeature::Ptr create(const OptAgentType& optAgentType, const double weight) = 0;
  virtual ~FeatureFactoryBase() { }
};

template<typename Feature>
class FeatureFactory : public FeatureFactoryBase {
 public:
  FeatureFactory() { }
  RawFeature::Ptr create(const sm::value_store::ValueStore& vpt) override {
    return RawFeature::Ptr(new Feature(vpt));
  }
  RawFeature::Ptr create(const OptAgentType& optAgentType, const double weight) override {
    return RawFeature::Ptr(new Feature(optAgentType, weight));
  }
  virtual ~FeatureFactory() { }
};

class FeatureProductFactory : public FeatureFactoryBase {
 public:
  FeatureProductFactory(const FeatureContainer& container) : _container(container) { }
  RawFeature::Ptr create(const sm::value_store::ValueStore& vpt) override;
  RawFeature::Ptr create(const OptAgentType& optAgentType, const double weight) override;
  virtual ~FeatureProductFactory() { }
 private:
  const FeatureContainer& _container;
};


class FeatureContainer {

 public:
  FeatureContainer(const sm::value_store::ValueStore& vpt);
  FeatureContainer(const std::string& xmlPath);
  FeatureContainer(const FeatureContainer&) = delete;
  ~FeatureContainer() {
    for(auto & v : _featureLookUp) {
      delete v.second;
    }
  }

  void save(const std::string& xmlPath) const;

  const std::vector<RawFeature::Ptr>& getContainer() const { return _container; }

  /// \brief Gets the feature with a given name \p featureClass
  /// \note Returns nullptr if feature not in container
  RawFeature::Ptr getFeature(const std::string& featureClass) const;

  FeatureContainer& operator = (const FeatureContainer&) = delete;

  void addErrorTerms(const ContinuousScene& scene, aslam::backend::OptimizationProblem& optimizationProblem) const;

  std::size_t numFeaturesActiveForLearning() const;

 protected:
  FeatureContainer();
  void push_back(const RawFeature::Ptr& f);
  void push_back(const std::string& featureClass, const OptAgentType& optAgentType, const double weight);

 private:
  bool isValidFeature(const std::string& featureClass) const;
  void createFeatureMap();
  void loadFeatures(const sm::value_store::ValueStore& vpt);
  std::unordered_map<std::string, FeatureFactoryBase*> _featureLookUp;
  std::vector<RawFeature::Ptr> _container;

};


} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_FEATURECONTAINER_HPP_ */
