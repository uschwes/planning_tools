/*
 * FeatureContainer.cpp
 *
 *  Created on: Jul 28, 2015
 *      Author: pfmark
 */

#include <boost/algorithm/string/trim.hpp>
#include <probabilistic_planner/features/FeatureContainer.hpp>

#include <sm/BoostPropertyTree.hpp>
#include <sm/random.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

// Pairwise features
#include <probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedSigmoidDistance.hpp>

// Physical trajectory attribute features
#include <probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedDirectionOfMotion.hpp>

// Observation features
#include <probabilistic_planner/features/FeatureSingletonObservationAbsoluteVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationVelocityVector.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationHeading.hpp>

// Barrier functions / soft constraints
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocityDifference.hpp>

// Target features
#include <probabilistic_planner/features/FeatureSingletonRobotTarget.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTargetVelocity.hpp>

// Grid features
#include <probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp>

// Product feature
#include <probabilistic_planner/features/FeatureProduct.hpp>

namespace prob_planner {

RawFeature::Ptr FeatureProductFactory::create(const sm::value_store::ValueStore& vpt) {
  return RawFeature::Ptr(new FeatureSingletonProduct(vpt, _container));
}
RawFeature::Ptr FeatureProductFactory::create(const OptAgentType& /*optAgentType*/, const double /*weight*/) {
  SM_THROW(planning2d::NoImplementationException, "Not implemented");
}

/**
 * Constructor
 */
FeatureContainer::FeatureContainer() {
  this->createFeatureMap();
}

FeatureContainer::FeatureContainer(const sm::value_store::ValueStore& vpt) {
  this->createFeatureMap();
  this->loadFeatures(vpt);
}

FeatureContainer::FeatureContainer(const std::string& xmlPath) {
  sm::BoostPropertyTree pt;
  pt.loadXml(xmlPath);
  sm::value_store::PropertyTreeValueStore vpt(pt);
  this->createFeatureMap();
  this->loadFeatures(vpt);
}

void FeatureContainer::save(const std::string& xmlPath) const {
  sm::BoostPropertyTree pt;
  sm::value_store::PropertyTreeValueStore vpt(pt);
  for (const auto& feature : getContainer()) {
    //TODO C this should be much simpler with .getChild of ValueStore
    sm::PropertyTree childPt(pt, feature->name() + "_" + OptAgentTypeRegistry::getRegistry().toString(feature->getOptAgentType()));
    sm::value_store::PropertyTreeValueStore child(childPt);
    feature->save(child);
  }
  pt.setHumanReadableInputOutput(true);
  pt.saveXml(xmlPath);
}

bool FeatureContainer::isValidFeature(const std::string& featureClass) const {
  return _featureLookUp.find(featureClass) != _featureLookUp.end();
}

RawFeature::Ptr FeatureContainer::getFeature(const std::string& featureClass) const {
  // Returns nullptr if feature not in container
  RawFeature::Ptr f;
  for (auto& feature : _container) {
      if (feature->name() == featureClass)
        f = feature;
    }
  return f;
}

void FeatureContainer::loadFeatures(const sm::value_store::ValueStore& vpt) {
  _container.clear();
  std::vector<sm::value_store::KeyValueStorePair> productFeatures;
  for (auto & c : vpt.getChildren()) {
    std::string featureClass = c.getValueStore().getString("featureClass");
    boost::trim(featureClass);
    if (this->isValidFeature(featureClass)) {
      if (featureClass != FeatureSingletonProduct::CLASS_NAME) { // delay creation until all singleton features are constructed
        _container.push_back(_featureLookUp.at(featureClass)->create(c.getValueStore()));
        SM_DEBUG_STREAM_NAMED("feature_computation", __FUNCTION__ << ": creating feature " << _container.back()->name());
      } else {
        productFeatures.push_back(c);
      }
    } else {
      SM_THROW(std::runtime_error, "The feature \"" << c.getKey() << "\" is not among the list of valid features. Please enter a valid feature name.");
    }
  }

  // construct product features
  for (auto & c : productFeatures) {
    std::string featureClass = c.getValueStore().getString("featureClass");
    boost::trim(featureClass);
    if (featureClass == FeatureSingletonProduct::CLASS_NAME) {
      if (this->isValidFeature(featureClass)) {
        _container.push_back(_featureLookUp.at(featureClass)->create(c.getValueStore()));
        SM_DEBUG_STREAM_NAMED("feature_computation", __FUNCTION__ << ": creating feature " << _container.back()->name());
      } else {
        SM_THROW(std::runtime_error, "The feature \"" << c.getKey() << "\" is not among the list of valid features. Please enter a valid feature name.");
      }
    }
  }
}

void FeatureContainer::push_back(const RawFeature::Ptr& f) {
  _container.push_back(f);
}

void FeatureContainer::push_back(const std::string& featureClass, const OptAgentType& optAgentType, const double weight) {
  SM_ASSERT_TRUE(planning2d::LookupException, this->isValidFeature(featureClass), "Feature not implemented.");
  _container.push_back(_featureLookUp.at(featureClass)->create(optAgentType, weight));
}

void FeatureContainer::addErrorTerms(const ContinuousScene& scene, aslam::backend::OptimizationProblem& optimizationProblem) const {
  for (auto& feature : _container) {
    SM_DEBUG_STREAM_NAMED("feature_computation", __FUNCTION__ << ": Add error terms of feature \"" << feature->name() << "\"");
    feature->addErrorTerms(scene, optimizationProblem);
  }
}

std::size_t FeatureContainer::numFeaturesActiveForLearning() const {

  std::size_t ret = 0;
  for (const auto& feature : _container)
    ret += feature->numActiveWeights() > 0 ? 1 : 0;
  return ret;
}

void FeatureContainer::createFeatureMap() {
  // Pairwise features
  _featureLookUp.insert(std::make_pair(FeaturePairwiseIntegratedInverseDistance::CLASS_NAME, new FeatureFactory<FeaturePairwiseIntegratedInverseDistance>()));
  _featureLookUp.insert(std::make_pair(FeaturePairwiseIntegratedDistance::CLASS_NAME, new FeatureFactory<FeaturePairwiseIntegratedDistance>()));
  _featureLookUp.insert(std::make_pair(FeaturePairwiseIntegratedSigmoidDistance::CLASS_NAME, new FeatureFactory<FeaturePairwiseIntegratedSigmoidDistance>()));

  // Motion features
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedAcceleration::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedAcceleration>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedRotationRate::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedRotationRate>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedVelocity::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedVelocity>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedVelocityDifference::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedVelocityDifference>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedDirectionOfMotion::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedDirectionOfMotion>()));

  //Observation features
  _featureLookUp.insert(std::make_pair(FeatureSingletonObservationPosition2d::CLASS_NAME, new FeatureFactory<FeatureSingletonObservationPosition2d>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonObservationHeading::CLASS_NAME, new FeatureFactory<FeatureSingletonObservationHeading>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonObservationVelocityVector::CLASS_NAME, new FeatureFactory<FeatureSingletonObservationVelocityVector>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonObservationAbsoluteVelocity::CLASS_NAME, new FeatureFactory<FeatureSingletonObservationAbsoluteVelocity>()));

  // Barrier features
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedBarrierVelocity::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedBarrierVelocity>()));

  // Target features
  _featureLookUp.insert(std::make_pair(FeatureSingletonRobotTarget::CLASS_NAME, new FeatureFactory<FeatureSingletonRobotTarget>()));
  _featureLookUp.insert(std::make_pair(FeatureSingletonRobotTargetVelocity::CLASS_NAME, new FeatureFactory<FeatureSingletonRobotTargetVelocity>()));

  // Grid features
  _featureLookUp.insert(std::make_pair(FeatureSingletonIntegratedStaticObstacleDistance::CLASS_NAME, new FeatureFactory<FeatureSingletonIntegratedStaticObstacleDistance>()));

  // product features
  _featureLookUp.insert(std::make_pair(FeatureSingletonProduct::CLASS_NAME, new FeatureProductFactory(*this)));
}

} /* namespace prob_planner */
