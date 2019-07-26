/*
 * testSupport.hpp
 *
 *  Created on: 07.07.2015
 *      Author: sculrich
 */

#ifndef TEST_TESTSUPPORT_HPP_
#define TEST_TESTSUPPORT_HPP_

// boost
#include <boost/make_shared.hpp>

//aslam includes
#include <aslam/backend/ScalarNonSquaredErrorTerm.hpp>
#include <aslam/backend/test/ErrorTermTester.hpp>

// self includes
#include "../include/probabilistic_planner/features/RawFeature.hpp"
#include "../include/probabilistic_planner/state_representation/OptAgentTypeRegistry.hpp"
#include "../include/probabilistic_planner/state_representation/OptAgent.hpp"
#include "../include/probabilistic_planner/state_representation/SceneSnapshot.hpp"
#include "../include/probabilistic_planner/state_representation/ContinuousScene.hpp"

namespace prob_planner {

//#define VERBOSE_DEBUG_OUTPUT = 1;

void populateScene(ContinuousScene& scene,
                   const bool random = false,
                   const std::size_t nAgents = 1,
                   const planning2d::Time startTime = planning2d::Time(0.0),
                   const planning2d::Duration timeHorizon = planning2d::Duration(10.0),
                   const OptAgentType type = OptAgentType::UNKNOWN,
                   const bool activateAllDesignVariables = true);

void populateSceneCoordinates(ContinuousScene& scene,
                              const std::map<planning2d::Id, planning2d::StateTrajectory >& trajCoords,
                              const OptAgentType type = OptAgentType::UNKNOWN,
                              const bool activateAllDesignVariables = true);

planning2d::StateTrajectory createStateTrajectoryRandomWalk(const size_t nPoints,
                                                            const planning2d::Duration& dt,
                                                            const planning2d::StateStamped& s0,
                                                            const double sigma);

std::vector<SceneSnapshot::Ptr> toSceneSnapshots(const planning2d::StateTrajectory& trajectory,
                                                 const planning2d::Id agentId,
                                                 const Eigen::MatrixXd& invCov);

template<typename Feature>
double evaluateFromErrorTerms(const Feature& f, ContinuousScene& scene);

template<typename Feature>
void testErrorTerms(const Feature& f, ContinuousScene& scene);

} // namespace prob_planner








template<typename Feature>
double prob_planner::evaluateFromErrorTerms(const Feature& f, ContinuousScene& scene) {

  aslam::backend::OptimizationProblem problem;

  for (auto& agent : scene.getOptAgentContainer())
    agent.second.trajectory().addDesignVariables(problem, true);

  f.addErrorTerms(scene, problem);
  double fErr = 0.0;
  for (size_t i=0; i<problem.numNonSquaredErrorTerms(); i++)
    fErr += problem.nonSquaredErrorTerm(i)->evaluateError();
  for (size_t i=0; i<problem.numErrorTerms(); i++)
    fErr += problem.errorTerm(i)->evaluateError();
  return fErr;
}


template<typename Feature>
void prob_planner::testErrorTerms(const Feature& f, ContinuousScene& scene) {

  aslam::backend::OptimizationProblem problem;

  for (auto& agent : scene.getOptAgentContainer())
    agent.second.trajectory().addDesignVariables(problem, true);

  f.addErrorTerms(scene, problem);
  for (size_t i=0; i<problem.numNonSquaredErrorTerms(); i++)
    aslam::backend::testErrorTerm(*problem.nonSquaredErrorTerm(i));
  for (size_t i=0; i<problem.numErrorTerms(); i++)
    aslam::backend::testErrorTerm(*problem.errorTerm(i));
}

#endif /* TEST_TESTSUPPORT_HPP_ */
