#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include <boost/make_shared.hpp>

#include <sm/logging.hpp>
#include <sm/random.hpp>
#include <sm/timing/Timer.hpp>

#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/OptimizerRprop.hpp>
#include <aslam/backend/OptimizerBFGS.hpp>

#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedVelocityDifference.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonIntegratedDirectionOfMotion.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp"
#include "../include/probabilistic_planner/features/FeatureSingletonObservationVelocityVector.hpp"
#include "../include/probabilistic_planner/state_representation/ContinuousScene.hpp"
#include "../include/probabilistic_planner/state_representation/SceneSnapshot.hpp"
#include "../include/probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp"
#include "Support.hpp"

#include <common_agents/PedestrianAgent.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp>


using namespace std;
using namespace planning2d;
using namespace prob_planner;
using namespace common_agents;
using namespace aslam::backend;

template <typename Optimizer>
struct OptimizerTests : public ::testing::Test  {
  virtual ~OptimizerTests() { }
  void fillOptions() { SM_THROW(planning2d::NoImplementationException, "not implemented"); }
  boost::shared_ptr<Optimizer> getOptimizer() {
    this->fillOptions();
    return boost::shared_ptr<Optimizer>(new Optimizer(_options));
  }
  typename Optimizer::Options& getOptions() { return _options; }
  typename Optimizer::Options _options;
};

template <>
void OptimizerTests<OptimizerRprop>::fillOptions() {
  _options.maxIterations = 10000;
  _options.numThreadsJacobian = 2;
  _options.convergenceGradientNorm = 0.3;
  _options.initialDelta = 0.5;
}

template <>
void OptimizerTests<OptimizerBFGS>::fillOptions() {
  _options.maxIterations = 10000;
  _options.numThreadsJacobian = 2;
  _options.convergenceGradientNorm = 0.3;
}

typedef ::testing::Types<
    OptimizerRprop,
    OptimizerBFGS
> OptimizerTypes;

TYPED_TEST_CASE(OptimizerTests, OptimizerTypes);


TYPED_TEST(OptimizerTests, Optimization) {

  try {

    ContinuousScene scene;

    const size_t nAgents = 2;
    const bool random = true;
    const OptAgentType agentType = OptAgentType::PEDESTRIAN;
    populateScene(scene, random, nAgents, 0.0, 5.0, agentType);

    // add measurements
    std::map<planning2d::Id, StateWithUncertainty> agentObservations;
    Time measStamp = scene.getMinTime();
    {
      auto sn = boost::make_shared<SceneSnapshot>(measStamp);
      size_t cnt = 0;
      for (auto& agent : scene.getOptAgentContainer()) {
        const auto pos = agent.second.trajectory().getPosition2d(agent.second.trajectory().getStartTime());
        PedestrianAgent::State s(pos.x() + 1., pos.y() + 1., 0.0, 0.0, 0.0); // sqrt(2) meters offset from first spline position
        const int stateDim = s.dimension() + 3;
        Eigen::MatrixXd invCov = 10.*Eigen::MatrixXd::Identity(stateDim, stateDim);
        StateWithUncertainty su(s, invCov);
        agentObservations[agent.first] = su;
        sn->addObject(agent.first, su);
        cnt++;
      }

      // add a grid
      OccupancyGrid grid(Pose2d(0., 0., 0.), 0.1, OccupancyGrid::Size2d(400,400), OccupancyValue::FREE);
      grid.set(OccupancyGrid::Index(0,0), OccupancyValue::OCCUPIED);
      sn->setOccupancyGrid(grid);

      scene.addObservation(sn);
    }

    boost::shared_ptr<OptimizationProblem> negLogDensity(new OptimizationProblem());

    // add the design variables of the agents and activate them
    scene.addDesignVariables(*negLogDensity, true);

    // Create features
    vector<RawFeature::Ptr> features;
    features.emplace_back( new FeaturePairwiseIntegratedInverseDistance(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedVelocity(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedBarrierVelocity(OptAgentType::ROBOT, 1.0, 1.0, 1000.0) );
    features.emplace_back( new FeatureSingletonIntegratedVelocityDifference(agentType, 1.0, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedAcceleration(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedDirectionOfMotion(agentType, 1.0) );
    features.emplace_back( new FeatureSingletonIntegratedStaticObstacleDistance(agentType, 10.0) );
    boost::dynamic_pointer_cast<FeatureSingletonIntegratedStaticObstacleDistance>(features.back())->setSigmoidParameters(50.0, 20.0, 0.0);
    features.emplace_back( new FeatureSingletonObservationPosition2d(agentType, 1.0) );    // Create a measurement model


    // add all error terms from the features to the optimization problem
    for (auto feature : features)
      feature->addErrorTerms(scene, *negLogDensity);

    // optimize
    boost::shared_ptr<TypeParam> optimizerPtr = this->getOptimizer();
    TypeParam& optimizer = *optimizerPtr;

    optimizer.setProblem(negLogDensity);
    optimizer.initialize();

    sm::timing::Timer optTimer("probabilistic_planner_TESTSUITE: optimization", false);
    optimizer.optimize();
    const auto ret = optimizer.getStatus();
    optTimer.stop();
    SM_DEBUG_STREAM("RProp optimizer ran for " << ret.numIterations << " iteration(s)");

    // Checks
    {
      EXPECT_TRUE(ret.success());
      EXPECT_GT(ret.numIterations, 0);
      EXPECT_GT(ret.numJacobianEvaluations, 0);
      EXPECT_GE(ret.numErrorEvaluations, 0);
      EXPECT_LE(ret.gradientNorm, this->getOptions().convergenceGradientNorm);
      EXPECT_TRUE(std::isnan(ret.deltaError) || ret.deltaError < 1e-3);

      size_t cnt = 0;
      for (auto& agent : scene.getOptAgentContainer()) {
        const auto& su = agentObservations[agent.first];
        const double dp = (agent.second.trajectory().getPosition2d(measStamp) - su.position()).norm();
        SM_VERBOSE_STREAM("Position offset of spline of agent " << agent.second.getId() <<
          ": " << dp);
        EXPECT_LT(dp, 1e0); // this is very specific to which features we use
        cnt++;
      }
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
