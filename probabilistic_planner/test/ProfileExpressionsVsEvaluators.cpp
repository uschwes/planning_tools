/*
 * ProfileExpressionsVsEvaluators.cpp
 *
 *  Created on: 31.08.2015
 *      Author: Ulrich Schwesinger
 */

#include <sm/logging.hpp>
#include <sm/timing/Timer.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/util/ProblemManager.hpp>

#include "../include/probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp"

#include "Support.hpp"

using namespace std;
using namespace planning2d;
using namespace aslam::backend;
using namespace prob_planner;

int main(int /*argc*/, char** /*argv*/) {

  try {

    const size_t nIterations = static_cast<size_t>(3e4);

    ContinuousScene scene;
    const size_t nAgents = 4;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType agentType = OptAgentType::PEDESTRIAN;
    populateScene(scene, false, nAgents, t0, timeHorizon, agentType);

    // Create features
    vector<RawFeature::Ptr> features;
    features.emplace_back( new FeaturePairwiseIntegratedDistance(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedVelocity(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedAcceleration(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedRotationRate(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonObservationPosition2d(agentType, 1.0) );

    // add all error terms from the feature to the optimization problem
    boost::shared_ptr<OptimizationProblem> negLogDensity(new OptimizationProblem());
    for (auto& oa : scene.getOptAgentContainer())
      oa.second.trajectory().addDesignVariables(*negLogDensity, true);
    for (auto feature : features)
      feature->addErrorTerms(scene, *negLogDensity);


    {
      SM_INFO_STREAM("Timing evaluator...");
      sm::timing::Timer tim0("FeatureEvaluation: Evaluators", false);
      for (auto feature : features) {
        sm::timing::Timer tim1(feature->name() + ": Evaluator", false);
        for (size_t i=0; i<nIterations; ++i)
          feature->evaluate(scene);
        tim1.stop();
      }
      tim0.stop();
    }

    {
      ProblemManager pm;
      pm.setProblem(negLogDensity);
      pm.initialize();

      SM_INFO_STREAM("Timing expressions...");
      sm::timing::Timer tim("FeatureEvaluation: Expressions", false);
      for (size_t i=0; i<nIterations; ++i)
        pm.evaluateError(1);
      tim.stop();
    }

    sm::timing::Timing::print(cout, sm::timing::SORT_BY_TOTAL);

    return EXIT_SUCCESS;
  } catch (exception& e) {
    SM_FATAL_STREAM(e.what());
    return EXIT_FAILURE;
  }

}
