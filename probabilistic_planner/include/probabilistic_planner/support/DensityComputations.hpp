/*
 * DensityComputations.hpp
 *
 *  Created on: 13.07.2016
 *      Author: Ulrich Schwesinger (ulrich.schwesinger@mavt.ethz.ch)
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_SUPPORT_DENSITYCOMPUTATIONS_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_SUPPORT_DENSITYCOMPUTATIONS_HPP_

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/features/RawFeature.hpp>
#include <probabilistic_planner/features/FeatureContainer.hpp>

using namespace std;
using namespace prob_planner;
using namespace aslam::backend;

namespace prob_planner {

double computeNegativeLogDensity(
    const RawFeature::ConstPtr& feature,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/);

double computeNegativeLogDensity(
    const FeatureContainer& features,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/);

RowVectorType computeGradientNegativeLogDensity(
    const RawFeature::ConstPtr& feature,
    const ContinuousScene& scene,
    const size_t numThreads = 1,
    const bool applyDvScaling = false);


RowVectorType computeGradientNegativeLogDensity(
    const FeatureContainer& features,
    const ContinuousScene& scene,
    const size_t numThreads /*= 1*/,
    const bool applyDvScaling /*= false*/);

} /* namespace prob_planner */

#endif /* INCLUDE_PROBABILISTIC_PLANNER_SUPPORT_DENSITYCOMPUTATIONS_HPP_ */
