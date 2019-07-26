/*
 * RawFeature.cpp
 *
 *  Created on: 18.03.2016
 *      Author: Ulrich Schwesinger
 */

#include <probabilistic_planner/features/RawFeature.hpp>

namespace prob_planner {

RawFeature::RawFeature(
    const std::string& name,
    const OptAgentType& optAgentType,
    const std::size_t dimension,
    const double scalingFactor /*= 1.0*/,
    const bool activateScaling /*= true*/)
    : _name(name),
      _optAgentType(optAgentType),
      _scalingFactor(Eigen::VectorXd::Constant(dimension, scalingFactor)),
      _isScalingActive(activateScaling)
{
  _weights.resize(dimension, DesignVariable(0.0));
  _weightsVector.resize(dimension, 1);
}


RawFeature::RawFeature(
    const std::string& name,
    const sm::value_store::ValueStore& vpt,
    const std::size_t dimension)
    : _name(name)
{
  _weights.resize(dimension, RawFeature::DesignVariable(0.0));
  _weightsVector.resize(dimension, 1);
  activateScaling(vpt.getBool("scaling/active", true).get());
  setOptAgentType(vpt.getString("agentType").get());
  setWeightsFromValueStore(vpt);
  setScalingFactorFromValueStore(vpt);
  try { activateForLearning(vpt.getBool("activeForLearning").get()); } // key may be missing
  catch (...) { }
}


void RawFeature::setWeightsFromValueStore(const sm::value_store::ValueStore& vpt) {
  if (numWeights() == 1)
    setWeight(0, vpt.getDouble("weight").get());
  else
    for (std::size_t i=0; i<numWeights(); ++i)
      setWeight(i, vpt.getDouble("weight/" + std::to_string(i)).get());
}

void RawFeature::setScalingFactor(const std::size_t i, const double s) {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, numWeights(), "");
  SM_FINE_STREAM_NAMED("feature_computation", name() << ": Setting scaling factor #" << i << " to value " << s);
  _scalingFactor[i] = s;
}

void RawFeature::scale(const Eigen::VectorXd& s) {
  if(_isScalingActive)
    _scalingFactor = _scalingFactor.cwiseProduct(s);
  SM_FINE_STREAM_NAMED("feature_computation", name() << ": Setting scaling factor to value " << _scalingFactor.transpose());
}

void RawFeature::scale(const std::size_t i, const double s) {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, numWeights(), "");
  if(_isScalingActive) {
    SM_FINE_STREAM_NAMED("feature_computation", name() << ": Setting scaling factor #" << i << " to value " << s);
    _scalingFactor[i] *= s;
  }
}

void RawFeature::setScalingFactorFromValueStore(const sm::value_store::ValueStore& vpt) {
  // Default value 1.0: no scaling
  SM_ASSERT_GT( planning2d::InitializationException, numWeights(), 0, "");
  _scalingFactor.resize(numWeights());
  if (numWeights() == 1)
    setScalingFactor(0, vpt.getDouble("scaling/factor", 1.0).get());
  else
    for (std::size_t i=0; i<numWeights(); ++i)
      setScalingFactor(i, vpt.getDouble("scaling/factor/" + std::to_string(i), 1.0).get());
}


void RawFeature::save(sm::value_store::ExtendibleValueStore& vpt) const {
  vpt.addString("featureClass", name());
  vpt.addString("agentType", OptAgentTypeRegistry::getRegistry().toString(_optAgentType));
  if (numWeights() == 1) {
    vpt.addDouble("weight", getWeight(0));
    vpt.addDouble("scaling/factor", getScalingFactor(0));
  } else {
    for (std::size_t i=0; i<numWeights(); ++i) {
      vpt.addDouble("weight/" + std::to_string(i), getWeight(i));
      vpt.addDouble("scaling/factor/" + std::to_string(i), getScalingFactor(0));
    }
  }
  vpt.addBool("scaling/active", _isScalingActive);
  vpt.addBool("activeForLearning", this->isActiveForLearning());
  saveImpl(vpt);
}

void RawFeature::activateForLearning(const bool activate) {
    for (auto& dv : _weights)
      dv.setActive(activate);
    if (activate)
      SM_FINE_STREAM_NAMED("feature_computation", name() << ": Activated for learning");
    else
      SM_FINE_STREAM_NAMED("feature_computation", name() << ": Deactivated for learning");
}

bool RawFeature::isActiveForLearning() const {
  bool isActiveForLearning = false;
  for (auto& dv : _weights)
    isActiveForLearning |= dv.isActive();
  return isActiveForLearning;
}

void RawFeature::forbidNegativeWeights(bool f) {
  for (auto& w : _weights)
    w.forbidNegative(f);
}

} /* namespace prob_planner */
