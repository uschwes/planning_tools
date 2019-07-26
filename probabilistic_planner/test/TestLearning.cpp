#include <gtest/gtest.h>

// standard includes
#include <math.h>
#include <stdlib.h>

// boost includes
#include <boost/make_shared.hpp>

// Schweizer Messer includes
#include <sm/logging.hpp>
#include <sm/random.hpp>
#include <sm/timing/Timer.hpp>

// self includes
#include "../include/probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp"
#include "../include/probabilistic_planner/features/FeatureContainer.hpp"
#include "../include/probabilistic_planner/state_representation/ContinuousScene.hpp"
#include "../include/probabilistic_planner/state_representation/SceneSnapshot.hpp"
#include "../include/probabilistic_planner/ProbabilisticLearner.hpp"
#include "Support.hpp"

// other package includes
#include <common_agents/PedestrianAgent.hpp>

using namespace std;
using namespace planning2d;
using namespace prob_planner;
using namespace common_agents;
using namespace aslam::backend;

class TestFeatureContainer: public FeatureContainer {
 public:
  TestFeatureContainer() { }
  void push_back(const RawFeature::Ptr& f) { FeatureContainer::push_back(f); }
};


TEST(probabilistic_planner_TESTSUITE, DISABLED_TestLearning) {

  try {

    sm::random::seed(666); // deterministic seed for scene creation to make successive runs comparable

    std::vector<ContinuousScene::Ptr> scenes(2);
    for (auto& scene : scenes) scene.reset(new ContinuousScene());

    const size_t nAgents = 2;
    const bool random = true;
    const OptAgentType agentType = PEDESTRIAN;
    for (auto& scene : scenes) { // 2x nAgents per scene
      populateScene(*scene, random, nAgents, 0.0, 10.0, agentType);
//      populateScene(*scene, random, nAgents, 5.0, 15.0, agentType);
    }


    // Deform the trajectories in the scene a little to get non-zero accelerations
    {
      auto dist = [&] (double) { return 0.1*sm::random::randn(); };
      for (auto& scene : scenes) {
        for (auto& agent : scene->getOptAgentContainer()) {
          auto dvs = agent.second.trajectory().getDesignVariables();
          Eigen::MatrixXd p;
          for (auto dv : dvs) {
            const int dbd = dv->minimalDimensions();
            dv->update(Eigen::VectorXd::NullaryExpr(dbd, 1, dist).eval().data(), dbd);
          }
        }
      }
    }

    // generate a single measurements at the start point for all agents with a specified noise
    {
      const double std = 0.5;
      const Duration dt(0.2);
      auto dist = [&] (double) { return std*sm::random::randn(); };
      for (auto& scene : scenes) {
        Time minTime = scene->getMinTime();
        auto sn = boost::make_shared<SceneSnapshot>(minTime);
        for (auto& agent : scene->getOptAgentContainer()) {
          if (minTime >= agent.second.trajectory().getStartTime() && minTime <= agent.second.trajectory().getFinalTime()) {
            const Position2d noise(Eigen::VectorXd::NullaryExpr(2, dist));
            const Position2d pos = agent.second.trajectory().getPosition2d(minTime) + noise;
            PedestrianAgent::State s(0.0, 0.0, Pose2d(pos, 0.0));
            sn->addObject(agent.first, StateWithUncertainty(s, 1./(std*std)*Eigen::MatrixXd::Identity(s.fullDimension(),s.fullDimension())));
          }
        }
        scene->addObservation(sn);
      }
    }

    // Activate all spline design variables in all scenes
    for (auto& scene : scenes)
      scene->activateAllDesignVariables(true);

    // Create features
    TestFeatureContainer features;
    features.push_back( boost::make_shared<FeaturePairwiseIntegratedDistance>(agentType, 1.0) );
    features.push_back( boost::make_shared<FeatureSingletonIntegratedVelocity>(agentType, 2.0) );
    features.push_back( boost::make_shared<FeatureSingletonIntegratedAcceleration>(agentType, 3.0) );
//    features.push_back( boost::make_shared<FeatureSingletonIntegratedRotationRate>(agentType, 4.0) );
    features.push_back( boost::make_shared<FeatureSingletonObservationPosition2d>(agentType, 1.0) );  // Create a measurement model

    // activate some features for learning
    for (auto feature : features.getContainer()) {
      if (feature->name() != "measurement_position2d")
        feature->activateForLearning(true);
    }

    for (auto feature : features.getContainer())
      SM_INFO_STREAM("The initial weight(s) for feature " << feature->name() << " are " << feature->getCurrentWeightsVector().transpose());

    // learner options
    ProbabilisticLearner::Options options;
    options.samplerType = ProbabilisticLearner::Options::METROPOLIS_HASTINGS;
    options.rpropOptions.maxIterations = 1;
    options.rpropOptions.numThreadsJacobian = 2; // more threads does not seem to help
  //  options.rpropOptions.convergenceGradientNorm = features.getContainer().size()*1e-1;
    options.rpropOptions.convergenceGradientNorm = 0.1;
    options.rpropOptions.initialDelta = 0.5;
    options.etOptions.nMcmcSamplesForMean = 100000;
    options.etOptions.nMcmcStepsBurnIn = 200;
    options.etOptions.storeSamples = true;
    options.metropolisHastingsOptions.nThreadsEvaluateLogDensity = 2;
    options.metropolisHastingsOptions.transitionKernelSigma = 0.02;

    // Train the model
    sm::random::seed(std::time(nullptr)); // random seed now
//    sm::logging::setLevel(sm::logging::Level::All);
    sm::logging::enableNamedStream("sampling");
    sm::logging::enableNamedStream("optimization");
    SCOPED_TRACE("");
    ProbabilisticLearner learner(features, options);

    SCOPED_TRACE("");
    learner.addDemonstrations(scenes);

#ifdef NDEBUG

    std::vector< std::vector<ErrorTermLearning::FeatureInfo> > featureInfos;
    for (size_t j=0; j<scenes.size(); ++j)
      featureInfos.push_back(learner.getFeatureInfo(j));

    const size_t numMaxIterations = 500;
    Eigen::VectorXd demonstration;
    for (size_t i=0; i<numMaxIterations; ++i) {

      SCOPED_TRACE("");
      learner.run(); // do one step

      for (std::size_t j=0; j<learner.nSamplers(); j++)
        EXPECT_EQ(options.etOptions.nMcmcSamplesForMean, learner.getSamples(j).size());

      for (size_t j=0; j<scenes.size(); ++j) {
        auto& fi = learner.getFeatureInfo(j);
        for (size_t k=0; k<fi.size(); ++k)
          EXPECT_TRUE( fi[k].demonstration.isApprox( featureInfos[j][k].demonstration ) );
      }

    }

    EXPECT_NE(1.0, features.getContainer()[0]->getWeight(0));
    EXPECT_NE(2.0, features.getContainer()[1]->getWeight(0));
    EXPECT_NE(3.0, features.getContainer()[2]->getWeight(0));
    EXPECT_NE(4.0, features.getContainer()[3]->getWeight(0));
    EXPECT_DOUBLE_EQ(1.0, features.getContainer().back()->getWeight(0)); // This feature was not activated for learning
#else
    SM_WARN_STREAM("Learning will only run when you compile in Release mode.");
#endif

    for (auto feature : features.getContainer())
      SM_INFO_STREAM("The learned weight(s) for feature " << feature->name() << " are " << feature->getCurrentWeightsVector().transpose());

#ifdef probabilistic_planner_ENABLE_TIMING
    sm::timing::Timing::print(cout, sm::timing::SORT_BY_TOTAL);
#endif

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}
