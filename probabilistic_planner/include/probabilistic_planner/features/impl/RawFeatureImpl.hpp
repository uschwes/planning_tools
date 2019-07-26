/*
 * RawFeatureImpl.hpp
 *
 *  Created on: 18.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURES_IMPL_RAWFEATUREIMPL_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURES_IMPL_RAWFEATUREIMPL_HPP_

namespace prob_planner {

inline std::size_t RawFeature::numActiveWeights() const {
  std::size_t nActive = 0;
  for (auto& w : _weights)
    nActive += static_cast<std::size_t>(w.isActive());
  return nActive;
}

inline double RawFeature::getWeight(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _weights.size(), "");
  return _weights.at(i).toScalar();
}

inline RawFeature::DesignVariable* RawFeature::getWeightAsDesignVariable(const std::size_t i) {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _weights.size(), "");
  return &_weights.at(i);
}

inline void RawFeature::setWeight(const std::size_t i, const double w) {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, _weights.size(), "");
  SM_FINE_STREAM_NAMED("feature_computation", name() << ": Setting weight #" << i << " with value " << w);
  _weights.at(i).setParameters( (Eigen::MatrixXd(1,1) << w).finished() );
}

inline const Eigen::VectorXd& RawFeature::getCurrentWeightsVector() const {
  SM_ASSERT_EQ_DBG(planning2d::RuntimeException, static_cast<std::size_t>(_weightsVector.size()), _weights.size(), "This is a bug in the code");
  size_t i = 0;
  for(auto & w : _weights) _weightsVector(i++) = w.toScalar();
  return _weightsVector;
}

inline double RawFeature::getScalingFactor(const std::size_t i) const {
  SM_ASSERT_LT( planning2d::OutOfBoundAccessException, i, static_cast<std::size_t>(_scalingFactor.size()), "");
  return _scalingFactor[i];
}

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURES_IMPL_RAWFEATUREIMPL_HPP_ */
