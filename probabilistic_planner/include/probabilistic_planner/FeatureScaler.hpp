/*
 * FeatureScaler.hpp
 *
 *  Created on: 23.09.2015
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_PROBABILISTIC_PLANNER_FEATURESCALER_HPP_
#define INCLUDE_PROBABILISTIC_PLANNER_FEATURESCALER_HPP_

#include <vector>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/features/FeatureContainer.hpp>

namespace prob_planner {

  void scale(RawFeature::Ptr feature,
             const std::vector<ContinuousScene::Ptr>& demonstrations);

  void scale(FeatureContainer& features,
             const std::vector<ContinuousScene::Ptr>& demonstrations);

}

#endif /* INCLUDE_PROBABILISTIC_PLANNER_FEATURESCALER_HPP_ */
