/*
 * TestFeatureScaler.cpp
 *
 *  Created on: 23.09.2015
 *      Author: Ulrich Schwesinger
 */

#include <gtest/gtest.h>

// self includes
#include "../include/probabilistic_planner/FeatureScaler.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp"

#include "Support.hpp"

using namespace std;
using namespace planning2d;
using namespace prob_planner;

double computeMaxFeatureValue(RawFeature::ConstPtr feature, const std::vector<ContinuousScene::Ptr>& scenes) {
  double max = std::numeric_limits<double>::min();
  for (const auto &scene : scenes) {
    const double fval = feature->evaluate(*scene)[0];
    max = fval > max ? fval : max;
  }
  return max;
}

TEST(probabilistic_planner_TESTSUITE, TestFeatureScaler) {

  const size_t nScenes = 5;
  const size_t nAgents = 1;
  const Time t0(0.0);
  const Duration timeHorizon(10.0);
  std::vector<ContinuousScene::Ptr> scenes(nScenes);

  for (auto& scene : scenes) {
    scene.reset(new ContinuousScene());
    populateScene(*scene, true, nAgents, t0, timeHorizon, OptAgentType::PEDESTRIAN);
  }

  const OptAgentType type = OptAgentType::PEDESTRIAN;
  FeatureSingletonIntegratedVelocity::Ptr f(new FeatureSingletonIntegratedVelocity(type, 1.0));
  EXPECT_DOUBLE_EQ(1.0, f->getScalingFactor(0));

  f->activateScaling(false);
  EXPECT_FALSE(f->isScalingActive());
  EXPECT_DOUBLE_EQ(1.0, f->getScalingFactor()[0]);
  f->scale(0, 2.0);
  EXPECT_DOUBLE_EQ(1.0, f->getScalingFactor()[0]); // scaling not active

  double max = computeMaxFeatureValue(f, scenes);

  f->activateScaling(true);
  scale(f, scenes);
  EXPECT_DOUBLE_EQ(1./max, f->getScalingFactor(0));

  max = computeMaxFeatureValue(f, scenes);
  EXPECT_DOUBLE_EQ(max, 1.0); // after scaling the maximum value should be 1.0
}
