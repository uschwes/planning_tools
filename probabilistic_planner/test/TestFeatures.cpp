#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <fcl/shape/geometric_shapes.h>

#include <sm/BoostPropertyTree.hpp>
#include <sm/random.hpp>
#include <sm/value_store/PropertyTreeValueStore.hpp>

#include <common_agents/DifferentialDriveAgent.hpp>
#include <common_agents/PedestrianAgent.hpp>

#include <aslam/backend/DesignVariableGenericVector.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>
#include <aslam/backend/test/GenericScalarExpressionTests.hpp>
#include <aslam/backend/test/ExpressionTests.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/OptimizerRprop.hpp>

#include <planner_algorithms/DistanceTransform.hpp>

#include <probabilistic_planner/state_representation/ContinuousScene.hpp>
#include <probabilistic_planner/state_representation/SceneSnapshot.hpp>
#include <probabilistic_planner/features/FeatureContainer.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedInverseDistance.hpp>
#include <probabilistic_planner/features/FeaturePairwiseIntegratedDistance.hpp>
#include <probabilistic_planner/features/FeatureStatistics.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedBarrierVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedAcceleration.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedRotationRate.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedDirectionOfMotion.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedVelocityDifference.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationAbsoluteVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationVelocityVector.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationPosition2d.hpp>
#include <probabilistic_planner/features/FeatureSingletonObservationHeading.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTarget.hpp>
#include <probabilistic_planner/features/FeatureSingletonRobotTargetVelocity.hpp>
#include <probabilistic_planner/features/FeatureSingletonIntegratedStaticObstacleDistance.hpp>
#include <probabilistic_planner/features/FeatureProduct.hpp>

#include "Support.hpp"

#include "PosFeature.hpp"
#include "OverlappingTimeAgentsPairwiseFeature.hpp"

using namespace std;
using namespace planning2d;
using namespace prob_planner;
using namespace common_agents;
using namespace aslam::backend;

TEST(Features, featureWeightDesignVariable) {

  {
    FeatureWeightScalarDesignVariable dv(-1.0);
    EXPECT_DOUBLE_EQ(0.0, dv.getParameters()(0,0));
  }
  {
    FeatureWeightScalarDesignVariable dv(0.0);
    double dx[] = {1.0};
    dv.update(dx, 1);
    EXPECT_DOUBLE_EQ(1.0, dv.getParameters()(0,0));
  }
  {
    FeatureWeightScalarDesignVariable dv(0.0);
    double dx[] = {-1.0};
    dv.update(dx, 1);
    EXPECT_DOUBLE_EQ(0.0, dv.getParameters()(0,0));
  }
  {
    FeatureWeightScalarDesignVariable dv(0.0, false);
    double dx[] = {-1.0};
    dv.update(dx, 1);
    EXPECT_DOUBLE_EQ(-1.0, dv.getParameters()(0,0));
  }
}

TEST(Features, scaling) {
  
  const OptAgentType type = OptAgentType::PEDESTRIAN;
  FeatureSingletonIntegratedVelocity f(type, 1.0);
  EXPECT_TRUE(f.getScalingFactor().isApprox(Eigen::VectorXd::Constant(1, 1.0)));
  
  f.scale(Eigen::VectorXd::Constant(1, 2.0));
  EXPECT_TRUE(f.getScalingFactor().isApprox(Eigen::VectorXd::Constant(1, 2.0)));
  
  f.scale(0, 2.0);
  EXPECT_DOUBLE_EQ(4.0, f.getScalingFactor(0));
}

TEST(Features, testGetSuitableAgents) {

  try {
    // Set up feature container
    sm::BoostPropertyTree pt;
    sm::value_store::PropertyTreeValueStore vpt(pt);

    pt.setString("rotation_rate_pedestrian/featureClass", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
    pt.setString("rotation_rate_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("rotation_rate_pedestrian/weight", 2.0);
    pt.setString("velocity_robot/featureClass", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("velocity_robot/agentType", "ROBOT");
    pt.setDouble("velocity_robot/weight", 3.0);
    pt.setString("pairwise_dist/featureClass", FeaturePairwiseIntegratedInverseDistance::CLASS_NAME);
    pt.setString("pairwise_dist/agentType", "ALL");
    pt.setDouble("pairwise_dist/weight", 3.0);

    FeatureSingletonIntegratedRotationRate rrFeature(OptAgentType::PEDESTRIAN, 1.0);
    FeatureSingletonIntegratedVelocity velFeature(OptAgentType::ROBOT, 1.0);
    FeaturePairwiseIntegratedInverseDistance distFeature(OptAgentType::ALL, 1.0);

    FeatureContainer featureContainer(vpt);

    // Set up scene
    ContinuousScene scene;
    populateScene(scene, 1, 2, Time(0.0), Duration(3.0), OptAgentType::PEDESTRIAN);
    populateScene(scene, 1, 2, Time(1.5), Duration(3.0), OptAgentType::PEDESTRIAN);
    populateScene(scene, 1, 3, Time(0.0), Duration(3.0), OptAgentType::ROBOT);

    auto agentsRR1 = rrFeature.getSuitableAgents(Time(1.0), scene);
    auto agentsRR2 = rrFeature.getSuitableAgents(Time(2.0), scene);
    auto agentsVel1 = velFeature.getSuitableAgents(Time(1.0), scene);
    auto agentPairs1 = distFeature.getSuitableAgentPairs(Time(1.0), scene);
    auto agentPairs2 = distFeature.getSuitableAgentPairs(Time(2.0), scene);

    EXPECT_EQ(2, agentsRR1.size());
    EXPECT_EQ(4, agentsRR2.size());
    EXPECT_EQ(3, agentsVel1.size());
    EXPECT_EQ((5*5-5)/2, agentPairs1.size());
    EXPECT_EQ((7*7-7)/2, agentPairs2.size());

    std::vector<planning2d::Id> agentIds;
    std::vector<Id> expectedIds = {0, 1};

    for (auto& agent : agentsRR1)
      agentIds.push_back(agent.get().getId());

    EXPECT_EQ(expectedIds, agentIds);

    expectedIds.push_back(2);
    expectedIds.push_back(3);

    agentIds.clear();
    for (auto& agent : agentsRR2)
      agentIds.push_back(agent.get().getId());

    EXPECT_EQ(expectedIds, agentIds);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, errorTerms) {

  try {

    const double w = 1.0;
    FeaturePairwiseIntegratedInverseDistance f(OptAgentType::PEDESTRIAN, w);
    ASSERT_DOUBLE_EQ(w, f.getWeight(0));

    const int nAgents = 2;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, OptAgentType::PEDESTRIAN);

    OptimizationProblem negLogDensity;

    for (auto agent : scene.getOptAgentContainer())
      agent.second.trajectory().addDesignVariables(negLogDensity, true);

    f.addErrorTerms(scene, negLogDensity);

    // test the error terms
    for (size_t i=0; i<negLogDensity.numNonSquaredErrorTerms(); i++)
      testErrorTerm(*negLogDensity.nonSquaredErrorTerm(i));
    for (size_t i=0; i<negLogDensity.numErrorTerms(); i++)
      testErrorTerm(*negLogDensity.errorTerm(i));

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureStatisticsEvaluateAndErrorTerm) {

  try {

    const double w = 4.0;
    PosFeature f(OptAgentType::PEDESTRIAN, w, 1);
    ASSERT_DOUBLE_EQ(w, f.getWeight(0));

    const int nAgents = 4;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, OptAgentType::PEDESTRIAN);

    // test evaluation at certain point
    {
      double fEval = f.evaluate<double, OptAgent>(Time(0.0), scene.getOptAgent(0));
      EXPECT_DOUBLE_EQ(1.0, fEval);
      fEval = f.evaluate<double, OptAgent>(Time(0.0), scene.getOptAgent(1));
      EXPECT_DOUBLE_EQ(1., fEval);
    }

    // Test evaluation over full time period
    {
      Eigen::VectorXd fEval = f.evaluate(scene);
      ASSERT_EQ(1, fEval.size());
      // We use Euler forward integration, so the integral cannot be exact
      EXPECT_NEAR(1.0*timeHorizon.toSec()*nAgents, fEval[0], 1e-6);

      // Test that error term evaluation produces same result
      const double fErr = evaluateFromErrorTerms(f, scene);
      EXPECT_NEAR(w*fEval[0], fErr, 1e-6);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(Features, featureStatisticsIntegration) {

  try {

    OptimizationProblem problem;

    // Generate Scene
    Time timeStart(0.0);
    PedestrianAgent::StateStamped stateStampedStart(PedestrianAgent::State(2, 2, 0, 0, 0), timeStart);

    Time timeGoal(10.0);
    PedestrianAgent::StateStamped stateStampedGoal(PedestrianAgent::State(4, 4, 0, 1, 1), timeGoal);

    Trajectory::Ptr trajectory(new Trajectory());
    trajectory->initStraightSpline(stateStampedStart, stateStampedGoal, 10, 0.1);

    ContinuousScene scene;
    CollisionGeometryPtr geom(new fcl::Cylinder(0.8, 1.4));
    PedestrianAgent::Ptr agent(new PedestrianAgent(false, 0, stateStampedStart, geom));
    OptAgent a(agent, trajectory, OptAgentType::PEDESTRIAN);
    scene.addOptAgent(a);

    // Get design variables from scene
    scene.getOptAgent(0).addDesignVariables(problem, true);

    // Integration via spline method
//    auto& spline = trajectory->getSpline();
//    Eigen::Vector2d integral = spline.evalIntegral(timeStart.nanosec, timeGoal.nanosec)/1e9; //TODO: fix in bsplines
    EXPECT_DOUBLE_EQ(10.0, (timeGoal - timeStart).toSec());
    EXPECT_DOUBLE_EQ(6.0, (stateStampedGoal.pose().position() + stateStampedStart.pose().position()).x());
    Eigen::Vector2d intManual = (timeGoal - timeStart).toSec() * (stateStampedGoal.pose().position() + stateStampedStart.pose().position()).asVector() * 0.5;
//    EXPECT_TRUE(intManual.isApprox(integral, 1e-8)) << "intManual = " << intManual.transpose() << ", intSpline = " << integral;

    // Integration via FeatureStatistics function
    const double w = 1.0;
    PosFeature f(OptAgentType::PEDESTRIAN, w, 0.0);
    const double fErr = evaluateFromErrorTerms(f, scene);
    SM_FINE_STREAM("The error sum is " << fErr);
    EXPECT_NEAR(intManual(0), fErr/w, 2e-1);
//    EXPECT_NEAR(integral(0), fErr/w, 1e-1);

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(Features, featurePairwiseIntegratedInverseDistance) {

  try {
    size_t nAgents = 2;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    const double w = 1.0;
    FeaturePairwiseIntegratedInverseDistance f(type, w);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    for (Time stamp = Time(0.0); stamp<=Time(10.0); stamp+=Duration(0.1)) {
      auto agentPairs = f.getSuitableAgentPairs(stamp, scene);
      ASSERT_FALSE(agentPairs.empty());

      const Eigen::Vector2d dpos = agentPairs[0].first.get().trajectory().getPosition(stamp) - agentPairs[0].second.get().trajectory().getPosition(stamp);
      auto distSquared = dpos.squaredNorm();
      const double expectedVal = 1.0/(1.0 + distSquared);

      double fEval = f.evaluate<double, OptAgent>(stamp, agentPairs[0].first, agentPairs[0].second);
      EXPECT_DOUBLE_EQ(expectedVal, fEval);
      GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp,
                                                                                                                           collectors::OptAgentExpressionWrapper(agentPairs[0].first),
                                                                                                                           collectors::OptAgentExpressionWrapper(agentPairs[0].second));
      EXPECT_NEAR(expectedVal, fExpr.evaluate()[0], 1e-6);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featurePairwiseIntegratedDistance) {

  try {
    size_t nAgents = 2;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    const double w = 1.0;
    FeaturePairwiseIntegratedDistance f(type, w);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    for (Time stamp = Time(0.0); stamp<=Time(10.0); stamp+=Duration(0.1)) {
      auto agentPairs = f.getSuitableAgentPairs(stamp, scene);
      ASSERT_FALSE(agentPairs.empty());

      const Eigen::Vector2d dpos = agentPairs[0].first.get().trajectory().getPosition(stamp) - agentPairs[0].second.get().trajectory().getPosition(stamp);
      const double expectedVal = dpos.squaredNorm();

      double fEval = f.evaluate<double, OptAgent>(stamp, agentPairs[0].first, agentPairs[0].second);
      EXPECT_DOUBLE_EQ(expectedVal, fEval);
      GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp,
                                                                                                                           collectors::OptAgentExpressionWrapper(agentPairs[0].first),
                                                                                                                           collectors::OptAgentExpressionWrapper(agentPairs[0].second));
      EXPECT_DOUBLE_EQ(expectedVal, fExpr.evaluate()[0]);
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureSingletonIntegratedVelocity) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    FeatureSingletonIntegratedVelocity f(type, 1.0);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    for (Time stamp = Time(0.0); stamp<=Time(10.0); stamp+=Duration(0.1)) {
      auto agents = f.getSuitableAgents(stamp, scene);
      ASSERT_EQ(1, agents.size());

      const double expectedVal = agents[0].get().trajectory().getVelocityXY(stamp).squaredNorm();

      double fEval = f.evaluate<double, OptAgent>(stamp, agents[0]);
      EXPECT_NEAR(expectedVal, fEval, 1e-6);
      GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp,
                                                                                                                           collectors::OptAgentExpressionWrapper(agents[0]));
      EXPECT_NEAR(expectedVal, fExpr.evaluate()[0], 1e-6);
    }
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureSingletonIntegratedVelocityDifference) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    FeatureSingletonIntegratedVelocityDifference f(type, 1.0);
    double desiredVel = 0.5;
    f.setDesiredVelocity(desiredVel);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    for (Time stamp = Time(0.0); stamp<=Time(10.0); stamp+=Duration(0.1)) {
      auto agents = f.getSuitableAgents(stamp, scene);
      ASSERT_EQ(1, agents.size());

      const double expectedVal = pow(sqrt(agents[0].get().trajectory().getVelocityXY(stamp).squaredNorm() + 0.1*0.1) - (desiredVel + 0.1), 2);

      double fEval = f.evaluate<double, OptAgent>(stamp, agents[0]);
      EXPECT_NEAR(expectedVal, fEval, 1e-6);
      GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp,
                                                                                                                           collectors::OptAgentExpressionWrapper(agents[0]));
      EXPECT_NEAR(expectedVal, fExpr.evaluate()[0], 1e-6);
    }
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureSingletonIntegratedRotationRate) {

  size_t nAgents = 1;
  const Time t0(0.0);
  const Duration timeHorizon(10.0);
  const OptAgentType type = OptAgentType::PEDESTRIAN;

  FeatureSingletonIntegratedRotationRate f(type, 1.0);
  ContinuousScene scene;

  populateScene(scene, false, nAgents, t0, timeHorizon, type);

  // Deform the splines a bit to get a non-zero rotation rate
  for (auto& agent : scene.getOptAgentContainer()) {
    for (auto dv : agent.second.trajectory().getDesignVariables()) {
      const int dim = dv->minimalDimensions();
      Eigen::VectorXd dvDx(dim);
      for (int i=0; i<dvDx.rows(); ++i)
        dvDx[i] = 0.2*sm::random::randn();
      dvDx *= dv->scaling();
      dv->update(&dvDx(0), dim);
    }
  }

  for (Time stamp = Time(0.0); stamp<=Time(10.0); stamp+=Duration(0.1)) {
    auto agents = f.getSuitableAgents(stamp, scene);
    ASSERT_EQ(1, agents.size());

    const double expectedVal = agents[0].get().trajectory().getRotationRateSquared(stamp);

    double fEval = f.evaluate<double, OptAgent>(stamp, agents[0]);
    GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp,
                                                                                                                         collectors::OptAgentExpressionWrapper(agents[0]));
    EXPECT_NEAR(fEval, fExpr.evaluate()[0], 1e-6) << "Evaluator and expression mismatch";
    EXPECT_NEAR(expectedVal, fEval, 1e-6);
    EXPECT_NEAR(expectedVal, fExpr.evaluate()[0], 1e-6);

  }
}

TEST(Features, featureObservationPosition2d) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    ContinuousScene scene;
    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    // create some measurements
    OptAgent& agent = scene.getOptAgent(0);
    size_t cnt = 0;
    for (Time stamp = agent.trajectory().getStartTime(); stamp <= agent.trajectory().getFinalTime(); stamp += Duration(1.0), cnt++) {
      Position2d pos = agent.trajectory().getPosition2d(stamp);
      PedestrianAgent::State s(pos.x() + 1.0, pos.y() + 1.0, 0.0, 0.0, 0.0); // sqrt(2) meters offset from the true position
      auto sn = boost::make_shared<SceneSnapshot>(stamp);
      sn->addObject(0, StateWithUncertainty(s, 1.0*Eigen::MatrixXd::Identity(5, 5)));
      scene.addObservation(sn);
    }

    FeatureSingletonObservationPosition2d f(type, 1.0);

    Eigen::VectorXd fEval = f.evaluate(scene);
    EXPECT_DOUBLE_EQ(cnt*1.0, fEval[0]);
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureObservationHeading) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;
    const double invVarMeas = 2.0;

    ContinuousScene scene;
    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    // create some measurements
    OptAgent& agent = scene.getOptAgent(0);

    // the heading of the trajectories generated by populateScene() is constant 0.0
    size_t cnt = 0;
    for (Time stamp = agent.trajectory().getStartTime(); stamp <= agent.trajectory().getFinalTime(); stamp += Duration(1.0), cnt++) {
      Position2d pos = agent.trajectory().getPosition2d(stamp);
      PedestrianAgent::State s(pos.x(), pos.y(), (cnt%2==0) ? 1.0 : 2.*M_PI - 1.0, 0.0, 0.0); // 1.0 [rad] constant offset, switching sides
      auto sn = boost::make_shared<SceneSnapshot>(stamp);
      sn->addObject(0, StateWithUncertainty(s, invVarMeas*Eigen::MatrixXd::Identity(5, 5)));
      scene.addObservation(sn);
    }

    FeatureSingletonObservationHeading f(type, 1.0);
    testErrorTerms(f, scene);

    Eigen::VectorXd fEval = f.evaluate(scene);
    EXPECT_NEAR(0.5*scene.getObservations().size()*invVarMeas, fEval[0], 1e-6);
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureObservationVelocity) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(2.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;
    const double invVarMeas = 2.0;
    double expectedVal = 0;

    ContinuousScene scene;
    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    // create some measurements
    OptAgent& agent = scene.getOptAgent(0);

    // the heading of the trajectories generated by populateScene() is constant 0.0
    size_t cnt = 0;
    for (Time stamp = agent.trajectory().getStartTime(); stamp <= agent.trajectory().getFinalTime(); stamp += Duration(1.0), cnt++) {
      Position2d pos = agent.trajectory().getPosition2d(stamp);
      auto heading = (cnt%2==0) ? 1.0 : 2.*M_PI - 1.0; // 1.0 [rad] constant offset, switching sides
      PedestrianAgent::State s(pos.x(), pos.y(), heading, cos(heading), sin(heading));
      expectedVal += pow((agent.trajectory().getVelocityXY(stamp).norm() - 1.0),2) * 0.5 * invVarMeas;
      auto sn = boost::make_shared<SceneSnapshot>(stamp);
      sn->addObject(0, StateWithUncertainty(s, invVarMeas*Eigen::MatrixXd::Identity(5, 5)));
      scene.addObservation(sn);
    }

    FeatureSingletonObservationAbsoluteVelocity f(type, 1.0);
    testErrorTerms(f, scene);
    Eigen::VectorXd fEval = f.evaluate(scene);
    EXPECT_NEAR(expectedVal, fEval[0], 1e-6);
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureObservationVelocityVector) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(2.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;
    const double invVarMeas = 2.0;
    double expectedVal = 0;

    ContinuousScene scene;
    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    // create some measurements
    OptAgent& agent = scene.getOptAgent(0);

    // the heading of the trajectories generated by populateScene() is constant 0.0
    size_t cnt = 0;
    for (Time stamp = agent.trajectory().getStartTime(); stamp <= agent.trajectory().getFinalTime(); stamp += Duration(1.0), cnt++) {
      Position2d pos = agent.trajectory().getPosition2d(stamp);
      auto heading = (cnt%2==0) ? 1.0 : 2.*M_PI - 1.0; // 1.0 [rad] constant offset, switching sides
      PedestrianAgent::State s(pos.x(), pos.y(), heading, cos(heading), sin(heading));
      expectedVal += (agent.trajectory().getVelocityXY(stamp) - Eigen::Vector2d(cos(heading), sin(heading))).squaredNorm() * 0.5 * invVarMeas;
      auto sn = boost::make_shared<SceneSnapshot>(stamp);
      sn->addObject(0, StateWithUncertainty(s, invVarMeas*Eigen::MatrixXd::Identity(5, 5)));
      scene.addObservation(sn);
    }

    FeatureSingletonObservationVelocityVector f(type, 1.0);
    testErrorTerms(f, scene);
    Eigen::VectorXd fEval = f.evaluate(scene);
    EXPECT_NEAR(expectedVal, fEval[0], 1e-6);
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureDirectionOfMotion) {

  try {
    const Time t0(0.0);
    const Duration timeHorizon(2.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;
    double expectedVal = 0;

    // Prepare scene
    ContinuousScene scene;
    StateTrajectory stateTraj;
    std::map<Id, StateTrajectory> coordMap;
    StateStamped startState(State(Eigen::Vector2d(1.0, 0.0), Pose2d(0.0, 0.0, 0.0)), t0);
    stateTraj.push_back(startState);
    StateStamped goalState(State(Eigen::Vector2d(1.0, 0.0), Pose2d(2.0, 0.0, 0.0)), t0 + timeHorizon);
    stateTraj.push_back(goalState);
    coordMap[0] = stateTraj;
    populateSceneCoordinates(scene, coordMap, type);

    // What to expect from the feature value
    Duration timestep(0.1); // TODO: where to get this from?
    Time tmin = scene.getMinTime();
    Time tmax = scene.getMaxTime();
    auto robot = scene.getOptAgent(0);
    for (auto t=tmin; t<tmax; t+=timestep) {
      expectedVal += acos((robot.trajectory().getVelocityXY(t).transpose() * startState.state())[0] / (robot.trajectory().getVelocityXY(t).norm() * startState.state().norm()));
    }
    expectedVal = expectedVal * timestep.toSec();

    FeatureSingletonIntegratedDirectionOfMotion f(type, 1.0);
    testErrorTerms(f, scene);
    auto fEval = f.evaluate(scene);
    EXPECT_NEAR(expectedVal, fEval[0], 1e-6);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureSingletonBarrierVelocity) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const Duration timestep(0.1);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    FeatureSingletonIntegratedBarrierVelocity f(type, 1.0);
    double maxVel = 2.0;
    f.setMaximumVelocity(maxVel);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, type);

    for (Time stamp = t0; stamp<=t0+timeHorizon; stamp+=timestep) {
      auto agents = f.getSuitableAgents(stamp, scene);
      ASSERT_EQ(1, agents.size());

      const double velNorm = agents[0].get().trajectory().getVelocityXY(stamp).norm();
      const double expectedVal = velNorm > maxVel ? planning2d::math::square(velNorm) : 0.0;

      double fEval = f.evaluate<double, OptAgent>(stamp, agents[0]);
      EXPECT_NEAR(expectedVal, fEval, 1e-6);
      GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp,
                                                                                                                           collectors::OptAgentExpressionWrapper(agents[0]));
      EXPECT_NEAR(expectedVal, fExpr.evaluate()[0], 1e-6);
    }
  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureSingletonRobotTarget) {

  try {
    size_t nAgents = 1;
    const Time t0(0.0);
    const Duration timeHorizon(10.0);
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    FeatureSingletonRobotTarget f(type, 1.0);
    const Pose2d target(12.0, 0.0, 0.0);
    f.setTarget(target);
    ContinuousScene scene;

    populateScene(scene, false, nAgents, t0, timeHorizon, type);
    const auto& agent = scene.getOptAgent(0);
    const auto& trajectory = agent.trajectory();
    const auto stamp = trajectory.getFinalTime();
    const Position2d& finalPos = trajectory.getPose2d(stamp).position();
    const double expectedVal = (finalPos - target.position()).squaredNorm();

    GenericMatrixExpression<1,1> fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(stamp, agent);
    EXPECT_NEAR(expectedVal, fExpr.evaluate()[0], 1e-6);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureSingletonRobotTargetVelocity) {

  try {
    const OptAgentType type = OptAgentType::PEDESTRIAN;

    const double vTransRef = 2.0;
    const Pose2d target(12.0, 0.0, 0.0);
    FeatureSingletonRobotTargetVelocity f(type, 1.0, target, vTransRef);
    EXPECT_EQ(target, f.getTarget());
    f.setTarget(Pose2d(0., 0., 0.));
    EXPECT_NE(target, f.getTarget());
    f.setTarget(target);
    EXPECT_EQ(target, f.getTarget());

    Trajectory::Ptr trajectory(new Trajectory());
    const Pose2d endPose(1.0, 0.0, 0.0);
    trajectory->initStraightSpline(StateStamped(HolonomicState(0., 0., Pose2d(0.0, 0.0, 0.0)), Time(0.0)), StateStamped(HolonomicState(0., 0., endPose), Time(1.0)), 10, 1e-2);
    auto pos = trajectory->getPosition2d(trajectory->getFinalTime());
    auto vel = trajectory->getVelocityXY(trajectory->getFinalTime());
    EXPECT_NEAR(pos.x(), endPose.x(), 1e-6);
    EXPECT_NEAR(pos.y(), endPose.y(), 1e-6);
    EXPECT_NEAR(vel.x(), 1.0, 1e-6);
    EXPECT_NEAR(vel.y(), 0.0, 1e-6);
    OptAgent oa(Agent::Ptr(new PedestrianAgent()), trajectory, type);

    auto vref = Eigen::Vector2d(2., 0.);
    auto fExpr = f.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(Time(0.0) /* irrelevant */, oa);
    EXPECT_NEAR((vel - vref).norm(), fExpr.evaluate()[0], 1e-6);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureSingletonStaticObstacleDistance) {

  try {

    const double gridResolution = 0.1;
    OccupancyGrid::Size2d gridSizeCells(200, 200);
    planning2d::OccupancyGrid grid( planning2d::Pose2d(0., 0., 0.), gridResolution, gridSizeCells, OccupancyValue::FREE);
    grid( Position2d(0., 0.) ) = OccupancyValue::OCCUPIED;

    // Test grid expression
    {
      Map<float>::Ptr floatMap(new Map<float>(grid.getOrigin(), grid.resolution(), grid.sizeInCells()));
      algorithms::signedDistanceTransform(grid, *floatMap);

      Map<float>::InterpolatedIndex idxp;
      for (idxp.y() = 0.0; idxp.y() < floatMap->sizeInCellsY(); idxp.y() += 0.6789) {
        for (idxp.x() = 0.0; idxp.x() < floatMap->sizeInCellsX(); idxp.x() += 0.6789) {

          Position2d pos = floatMap->toPosition(idxp);
          aslam::backend::DesignVariableGenericVector<2> dv(pos.asVector());
          dv.setActive(true);
          dv.setBlockIndex(0);
          GenericMatrixExpression<2, 1> abscissaExpr(&dv);

          ASSERT_TRUE(floatMap->isInsideMap(pos)) << "Position " << pos << " not in map";
          GridExpressionNode<float,grid::InterpolationMethod::CUBIC_CATMULL_ROM,grid::ExtrapolationMethod::CONSTANT> gridExpr(abscissaExpr, floatMap);
          {
            SCOPED_TRACE(testing::Message() << "Testing expression at position " << pos);
            testExpression(ScalarExpression(&gridExpr), -1, false);
          }
        }
      }
    } // Test grid expression

    // Test feature
    {
      const Time t0(0.0);
      const Duration timeHorizon(10.0);
      const OptAgentType type = OptAgentType::PEDESTRIAN;

      std::map<planning2d::Id, StateTrajectory> coords;
      coords[0] = StateTrajectory( { StateStamped(State(Eigen::Vector2d(0.0, 0.0), Pose2d(0.0, 0.0, 0.0)), t0),
                                     StateStamped(State(Eigen::Vector2d(10.0, 0.0), Pose2d(10.0, 0.0, 0.0)), t0 + timeHorizon) } );
      ContinuousScene scene;
      populateSceneCoordinates(scene, coords, OptAgentType::PEDESTRIAN);

      // add one occupancy grid observation
      auto& trajectory = scene.getOptAgent(0).trajectory();
      auto stamp = t0 + timeHorizon/2.;

      auto sn = boost::make_shared<SceneSnapshot>(stamp);
      sn->setOccupancyGrid(grid);
      scene.addObservation(sn);

      FeatureSingletonIntegratedStaticObstacleDistance f(type, 1.0);

      OptimizationProblem problem;
      trajectory.addDesignVariables(problem, true);
      SCOPED_TRACE("");
      f.addErrorTerms(scene, problem);

      auto fval = f.evaluate(scene);
      ASSERT_EQ(1, fval.size());
      EXPECT_FALSE(std::isnan(fval[0]));
      EXPECT_GT(fval[0], 0.0);
    } // Test feature

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureSingletonProduct) {

  try {
    FeatureSingletonIntegratedVelocity::Ptr lhs(new FeatureSingletonIntegratedVelocity(OptAgentType::PEDESTRIAN, 1.0));
    FeatureSingletonIntegratedRotationRate::Ptr rhs(new FeatureSingletonIntegratedRotationRate(OptAgentType::PEDESTRIAN, 1.0));
    FeatureSingletonProduct crossproduct(lhs, rhs, OptAgentType::PEDESTRIAN, 1.0);
    EXPECT_EQ(string("singleton_cross_product.") + FeatureSingletonIntegratedVelocity::CLASS_NAME + "." + FeatureSingletonIntegratedRotationRate::CLASS_NAME, crossproduct.name());
    FeatureSingletonProduct autoproduct(lhs, lhs, OptAgentType::PEDESTRIAN, 1.0);
    EXPECT_EQ(string("singleton_auto_product.") + FeatureSingletonIntegratedVelocity::CLASS_NAME, autoproduct.name());

    Trajectory::Ptr trajectory(new Trajectory());
    trajectory->initStraightSpline(Position2dStamped(0.0, 0.0, Time(0.0)), Position2dStamped(2.0, 0.0, Time(1.0)), 10, 1e-2); // ~ 2m/s
    OptAgent oa(Agent::Ptr(new PedestrianAgent()), trajectory, OptAgentType::PEDESTRIAN);

    auto exprLhs = lhs->evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(Time(0.0), oa);
    auto exprRhs = rhs->evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(Time(0.0), oa);
    auto exprProduct = crossproduct.evaluate<GenericMatrixExpression<1,1>, collectors::OptAgentExpressionWrapper>(Time(0.0), oa);
    const double expected = exprLhs.evaluate()[0] * exprRhs.evaluate()[0];
    const double nexprVal = crossproduct.evaluate<double,OptAgent>(Time(0.0), oa);
    EXPECT_DOUBLE_EQ(expected, exprProduct.evaluate()[0]);
    EXPECT_DOUBLE_EQ(expected, nexprVal);

    auto productMult = multiply(lhs, rhs, OptAgentType::PEDESTRIAN, 1.0);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, compareFeatureObservationPosition2dWithSplineFit) {

  try {
    // Create set of discrete points
    const int nPoints = 10;
    const int nSplineSegments = 100;
    const Id agentId = 0;
    const OptAgentType agentType = OptAgentType::PEDESTRIAN;
    const Duration dt(1.0);
    const StateStamped s0(State(Eigen::Vector2d(0.0, 0.0), Pose2d(Position2d(0.0, 0.0), 0.0)), Time(0.0));

    StateTrajectory stateTrajectory = createStateTrajectoryRandomWalk(nPoints, dt, s0, 1.0);

    // Spline fit
    Trajectory::Ptr trajectorySplineFit(new Trajectory());
    trajectorySplineFit->initFromDiscretizedTrajectory(stateTrajectory, nSplineSegments, 0.1);

    // Create Agent and scene
    ContinuousScene scene;
    populateScene(scene, false, 1, Time(0.0), (stateTrajectory.back().stamp() - stateTrajectory.front().stamp()), agentType);

    // Add observations to scene
    std::vector<SceneSnapshot::Ptr> snapshots = toSceneSnapshots(stateTrajectory, agentId, 1.0*Eigen::MatrixXd::Identity(5, 5));
    for (auto& obs : snapshots)
      scene.addObservation(obs);

    // Set up optimization Problem
    boost::shared_ptr<aslam::backend::OptimizationProblem> problem( new aslam::backend::OptimizationProblem());

    // Add spline design variables to optimization problem and activate them
    scene.getOptAgent(0).addDesignVariables(*problem, true);

    // Add measurement feature error terms to problem
    FeatureSingletonObservationPosition2d feature(agentType, 1.0);
    feature.addErrorTerms(scene, *problem);

    // Set options and optimize
    OptimizerOptionsRprop options;
    options.maxIterations = 500;
    options.numThreadsJacobian = 1;
    OptimizerRprop optimizer(options);
    optimizer.setProblem(problem);

    ASSERT_NO_THROW(optimizer.checkProblemSetup());

    optimizer.optimize();

    EXPECT_LT(optimizer.getStatus().gradientNorm, 1e-3);

    // Testing
    for (size_t i=0; i<stateTrajectory.size(); ++i) {
      auto& agent = scene.getOptAgent(agentId);
      const Time& stamp = stateTrajectory[i].stamp();
      EXPECT_NEAR( agent.trajectory().getPosition(stamp).x(), trajectorySplineFit->getPosition(stamp).x(), 2.0) <<
          "x-coordinate of positions at time " << stamp << " not within error bounds";
      EXPECT_NEAR( agent.trajectory().getPosition(stamp).y(), trajectorySplineFit->getPosition(stamp).y(), 2.0) <<
          "y-coordinate of positions at time " << stamp << " not within error bounds";;
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}


TEST(Features, featureStatisticsIntegrationPairwise) {

  try {

    OptimizationProblem problem;

    // Create scene
    ContinuousScene scene;
    std::map<planning2d::Id, StateTrajectory> trajCoords;
    // Three straight lines starting at different x-coordinates
    PedestrianAgent::State s(0.0, 0.0, 0.0, 0.0, 0.0);
    trajCoords[0] = StateTrajectory( {PedestrianAgent::StateStamped(s, Time(0.0)),
                                      PedestrianAgent::StateStamped(s, Time(5.0))} );
    trajCoords[1] = StateTrajectory( {PedestrianAgent::StateStamped(s, Time(1.0)),
                                      PedestrianAgent::StateStamped(s, Time(6.0))} );
    trajCoords[2] = StateTrajectory( {PedestrianAgent::StateStamped(s, Time(2.0)),
                                      PedestrianAgent::StateStamped(s, Time(7.0))} );
    populateSceneCoordinates(scene, trajCoords, OptAgentType::PEDESTRIAN);
    ASSERT_EQ(trajCoords.size(), scene.numberOfAgents());


    // Feature
    const double w = 1.0;
    OverlappingTimeAgentsPairwiseFeature f(OptAgentType::PEDESTRIAN, w);

    Eigen::VectorXd fEval = f.evaluate(scene);
    ASSERT_EQ(1, fEval.size());

    // quickly check that feature value for two agents is really 1.0
    const double fExpected = 1.0;
    {
      const Time stamp(1.0);
      auto agentPairs = f.getSuitableAgentPairs(stamp, scene);

      double fEval = f.evaluate<double, OptAgent>(stamp, agentPairs[0].first, agentPairs[0].second);
      EXPECT_DOUBLE_EQ(fExpected, fEval);
    }

    // Agents 0 and 1 overlap in time in the interval [1.0, 5.0]
    // Agents 0 and 2 overlap in time in the interval [2.0, 5.0]
    // Agents 1 and 2 overlap in time in the interval [2.0, 6.0]
    // The distance between the agents is constant (1.0, 0.0), so the feature value
    // is constant fExpected.
    // integrating fExpected over the time spans 4.0s, 3.0s and 4.0s
    const double overlapManual = fExpected*(4. + 3. + 4.);
    EXPECT_NEAR(overlapManual, fEval[0], 0.1*scene.numberOfAgents());

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}


TEST(Features, propertyTree) {

  try {

    sm::BoostPropertyTree pt;
    sm::value_store::PropertyTreeValueStore vpt(pt);

    // 1 dim feature
    pt.setString("acceleration_pedestrian/featureClass", FeatureSingletonIntegratedAcceleration::CLASS_NAME);
    pt.setString("acceleration_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("acceleration_pedestrian/weight", 2.0);
    pt.setDouble("acceleration_pedestrian/scaling/factor", 2.0);
    pt.setBool("acceleration_pedestrian/scaling/active", false);
    SCOPED_TRACE("");
    FeatureSingletonIntegratedAcceleration feature(vpt.getChildren()[0].getValueStore());
    EXPECT_DOUBLE_EQ(2.0, feature.getWeight(0));
    EXPECT_EQ(OptAgentType::PEDESTRIAN, feature.getOptAgentType());
    EXPECT_EQ(FeatureSingletonIntegratedAcceleration::CLASS_NAME, feature.name());
    EXPECT_FALSE(feature.isScalingActive());
    EXPECT_DOUBLE_EQ(2.0, feature.getScalingFactor(0));

    // Multi dim feature
    class TestMultiDimFeature : public RawFeature {
     public:
      TestMultiDimFeature(const sm::value_store::ValueStore& vpt) : RawFeature("multi_dim", vpt, 2) { }
      void addErrorTerms(const ContinuousScene& , OptimizationProblem& ) const { }
    };

    pt.setString("multi_dim_feature/featureClass", "multi_dim");
    pt.setString("multi_dim_feature/agentType", "PEDESTRIAN");
    pt.setDouble("multi_dim_feature/weight/0", 4.0);
    pt.setDouble("multi_dim_feature/weight/1", 5.0);
    pt.setDouble("multi_dim_feature/scaling/factor/0", 1.0);
    pt.setDouble("multi_dim_feature/scaling/factor/1", 2.0);
    pt.setBool("multi_dim_feature/scaling/active", true);
    SCOPED_TRACE("");
    TestMultiDimFeature featureMulti(vpt.getChildren()[1].getValueStore());
    EXPECT_DOUBLE_EQ(4.0, featureMulti.getWeight(0));
    EXPECT_DOUBLE_EQ(5.0, featureMulti.getWeight(1));
    EXPECT_DOUBLE_EQ(1.0, featureMulti.getScalingFactor(0));
    EXPECT_DOUBLE_EQ(2.0, featureMulti.getScalingFactor(1));
    EXPECT_TRUE(featureMulti.isScalingActive());
    EXPECT_EQ(OptAgentType::PEDESTRIAN, featureMulti.getOptAgentType());
    EXPECT_EQ("multi_dim", featureMulti.name());


  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(Features, featureContainer) {

  try {
    sm::BoostPropertyTree pt;
    sm::value_store::PropertyTreeValueStore vpt(pt);

    pt.setString("rotation_rate_pedestrian/featureClass", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
    pt.setString("rotation_rate_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("rotation_rate_pedestrian/weight", 2.0);
    pt.setDouble("rotation_rate_pedestrian/scaling/factor", 3.0);
    pt.setBool  ("rotation_rate_pedestrian/scaling/active", false);
    pt.setString("velocity_car/featureClass", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("velocity_car/agentType", "CAR");
    pt.setDouble("velocity_car/weight", 3.0);
    pt.setString("crossproduct_velocity_rotation_rate_robot/featureClass", FeatureSingletonProduct::CLASS_NAME);
    pt.setString("crossproduct_velocity_rotation_rate_robot/lhs", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("crossproduct_velocity_rotation_rate_robot/rhs", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
    pt.setString("crossproduct_velocity_rotation_rate_robot/agentType", "ROBOT");
    pt.setDouble("crossproduct_velocity_rotation_rate_robot/weight", 4.0);
    pt.setDouble("crossproduct_velocity_rotation_rate_robot/scaling/factor", 5.0);
    pt.setBool  ("crossproduct_velocity_rotation_rate_robot/scaling/active", true);
    pt.setString("autoproduct_velocity_robot/featureClass", FeatureSingletonProduct::CLASS_NAME);
    pt.setString("autoproduct_velocity_robot/lhs", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("autoproduct_velocity_robot/rhs", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("autoproduct_velocity_robot/agentType", "CAR");
    pt.setDouble("autoproduct_velocity_robot/weight", 1.2);
    pt.setDouble("autoproduct_velocity_robot/scaling/factor", 0.1);
    pt.setBool  ("autoproduct_velocity_robot/scaling/active", false);

    FeatureContainer featureContainer(vpt);
    auto featureVector = featureContainer.getContainer();
    SCOPED_TRACE("Testing feature count");
    EXPECT_EQ(4, featureVector.size());

    { // Test rotation rate feature
      SCOPED_TRACE("Testing rotation rate feature");
      const auto f = featureContainer.getFeature(FeatureSingletonIntegratedRotationRate::CLASS_NAME);
      ASSERT_TRUE(f != nullptr);
      EXPECT_EQ(FeatureSingletonIntegratedRotationRate::CLASS_NAME, f->name());
      EXPECT_EQ(OptAgentType::PEDESTRIAN, f->getOptAgentType());
      EXPECT_DOUBLE_EQ(2.0, f->getWeight());
      EXPECT_DOUBLE_EQ(3.0, f->getScalingFactor(0));
      EXPECT_FALSE(f->isScalingActive());
    }

    { // Test velocity feature
      SCOPED_TRACE("Testing velocity feature");
      const auto f = featureContainer.getFeature(FeatureSingletonIntegratedVelocity::CLASS_NAME);
      ASSERT_TRUE(f != nullptr);
      EXPECT_EQ(FeatureSingletonIntegratedVelocity::CLASS_NAME, f->name());
      EXPECT_EQ(OptAgentType::CAR, f->getOptAgentType());
      EXPECT_DOUBLE_EQ(3.0, f->getWeight());
    }

    { // Test cross product feature
      SCOPED_TRACE("Testing cross product feature");
      const string fname = string("singleton_cross_product.") + FeatureSingletonIntegratedVelocity::CLASS_NAME + "." + FeatureSingletonIntegratedRotationRate::CLASS_NAME;
      const auto f = featureContainer.getFeature(fname);
      ASSERT_TRUE(f != nullptr);
      EXPECT_EQ(fname, f->name());
      EXPECT_EQ(OptAgentType::ROBOT, f->getOptAgentType());
      EXPECT_DOUBLE_EQ(4.0, f->getWeight());
      EXPECT_DOUBLE_EQ(5.0, f->getScalingFactor(0));
      EXPECT_TRUE(f->isScalingActive());
    }

    { // Test auto product feature
      SCOPED_TRACE("Testing auto product feature");
      const string fname = string("singleton_auto_product.") + FeatureSingletonIntegratedVelocity::CLASS_NAME;
      const auto f = featureContainer.getFeature(fname);
      ASSERT_TRUE(f != nullptr);
      EXPECT_EQ(fname, f->name());
      EXPECT_EQ(OptAgentType::CAR, f->getOptAgentType());
      EXPECT_DOUBLE_EQ(1.2, f->getWeight());
      EXPECT_DOUBLE_EQ(0.1, f->getScalingFactor(0));
      EXPECT_FALSE(f->isScalingActive());
    }

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}

TEST(Features, featureContainerFromFile) {

  try {
    sm::BoostPropertyTree pt;
    sm::BoostPropertyTree ptFromFile;

    pt.setString("rotation_rate_pedestrian/featureClass", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
    pt.setString("rotation_rate_pedestrian/agentType", "PEDESTRIAN");
    pt.setDouble("rotation_rate_pedestrian/weight", 2.0);
    pt.setDouble("rotation_rate_pedestrian/scaling/factor", 3.0);
    pt.setString("velocity_car/featureClass", FeatureSingletonIntegratedVelocity::CLASS_NAME);
    pt.setString("velocity_car/agentType", "CAR");
    pt.setDouble("velocity_car/weight", 3.0);
    pt.setDouble("velocity_car/scaling/factor", 4.0);

    pt.setHumanReadableInputOutput(true);

    const string filename = "test" + time::getCurrentTime().format(time::Formatter(time::NANOSEC,0,"","")).toString(true) + ".xml";
    pt.saveXml(filename);
    ptFromFile.loadXml(filename);
    boost::filesystem::remove(filename);

    sm::value_store::PropertyTreeValueStore vpt(pt);

    FeatureContainer featureContainer(vpt);
    auto featureVector = featureContainer.getContainer();

    EXPECT_EQ(2, featureVector.size());
    EXPECT_DOUBLE_EQ(2.0, featureVector[0]->getWeight());
    EXPECT_DOUBLE_EQ(2.0, featureVector[0]->getWeight(0));
    EXPECT_TRUE(featureVector[0]->getScalingFactor().isApprox(Eigen::VectorXd::Constant(1, 3.0)));
    EXPECT_EQ(OptAgentType::PEDESTRIAN, featureVector[0]->getOptAgentType());
    EXPECT_DOUBLE_EQ(3.0, featureVector[1]->getWeight());
    EXPECT_TRUE(featureVector[1]->getScalingFactor().isApprox(Eigen::VectorXd::Constant(1, 4.0)));
    EXPECT_EQ(OptAgentType::CAR, featureVector[1]->getOptAgentType());

  } catch (const exception& e) {
    FAIL() << e.what();
  }

}

TEST(Features, featureContainerSave) {

  try {

    const string filename = "test" + time::getCurrentTime().format(time::Formatter(time::NANOSEC,0,"","")).toString(true) + ".xml";

    class FeatureContainerTest : public FeatureContainer {
     public:
      FeatureContainerTest() { }
      using FeatureContainer::FeatureContainer;
      using FeatureContainer::push_back;
    };

    { // save
      sm::BoostPropertyTree pt;
      sm::value_store::PropertyTreeValueStore vs(pt);
      pt.setString("rotation_rate_pedestrian/featureClass", FeatureSingletonIntegratedRotationRate::CLASS_NAME);
      pt.setString("rotation_rate_pedestrian/agentType", "PEDESTRIAN");
      pt.setDouble("rotation_rate_pedestrian/weight", 2.0);
      pt.setDouble("rotation_rate_pedestrian/scaling/factor", 3.0);
      pt.setBool("rotation_rate_pedestrian/scaling/active", false);
      pt.setBool("rotation_rate_pedestrian/activeForLearning", true);
      pt.setString("barrier_velocity_car/featureClass", FeatureSingletonIntegratedBarrierVelocity::CLASS_NAME);
      pt.setString("barrier_velocity_car/agentType", "CAR");
      pt.setDouble("barrier_velocity_car/weight", 3.0);
      pt.setDouble("barrier_velocity_car/scaling/factor", 4.0);
      pt.setBool("barrier_velocity_car/scaling/active", true);
      pt.setBool("barrier_velocity_car/activeForLearning", false);
      pt.setDouble("barrier_velocity_car/internal_parameters/maximumVelocity", 5.0);
      pt.setDouble("barrier_velocity_car/internal_parameters/factor", 6.0);

      FeatureContainerTest container(vs);
      container.save(filename);
      container.save(filename); // save twice to check that this is overwrites
    }

    { // load & check
      SCOPED_TRACE(testing::Message() << "Loading xml file from " << filename);
      FeatureContainer container(filename);

      {
        const auto f = container.getFeature(FeatureSingletonIntegratedRotationRate::CLASS_NAME);
        ASSERT_TRUE(f != nullptr);
        EXPECT_EQ(1, f->numWeights());
        EXPECT_DOUBLE_EQ(2.0, f->getWeight(0));
        EXPECT_EQ(OptAgentType::PEDESTRIAN, f->getOptAgentType());
        EXPECT_DOUBLE_EQ(3.0, f->getScalingFactor(0));
        EXPECT_FALSE(f->isScalingActive());
        EXPECT_TRUE(f->isActiveForLearning());
      }
      {
        const auto f = boost::dynamic_pointer_cast<FeatureSingletonIntegratedBarrierVelocity>(container.getFeature(FeatureSingletonIntegratedBarrierVelocity::CLASS_NAME));
        ASSERT_TRUE(f != nullptr);
        EXPECT_EQ(1, f->numWeights());
        EXPECT_DOUBLE_EQ(3.0, f->getWeight(0));
        EXPECT_EQ(OptAgentType::CAR, f->getOptAgentType());
        EXPECT_DOUBLE_EQ(4.0, f->getScalingFactor(0));
        EXPECT_TRUE(f->isScalingActive());
        EXPECT_FALSE(f->isActiveForLearning());
        EXPECT_DOUBLE_EQ(5.0, f->getMaximumVelocity());
        EXPECT_DOUBLE_EQ(6.0, f->getFactor());
      }
    }

    boost::filesystem::remove(filename);

  } catch (const exception& e) {
    FAIL() << e.what();
  }
}
