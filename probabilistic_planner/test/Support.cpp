/*
 * testSupport.cpp
 *
 *  Created on: 17 Jul, 2015
 *      Author: sculrich
 */


// FCL includes
#include <fcl/shape/geometric_shapes.h>

// self includes
#include "Support.hpp"

// other package includes
#include <common_agents/PedestrianAgent.hpp>

// Schweizer Messer includes
#include <sm/random.hpp>

namespace prob_planner {

using namespace std;
using namespace common_agents;
using namespace planning2d;

/**
 * Populates a scene with a given number of agents. If \random is false, the agents' trajectories
 * will be straight lines with y-position always 1.0.
 * @param scene The scene to populate
 * @param random Set to true to create random start and goal locations
 * @param nAgents Number of agents to place into the scene
 * @param timeHorizon Time horizon
 * @param type Type of agents to drop into the scene
 */
void populateScene(ContinuousScene& scene,
                   const bool random /*= false*/,
                   const std::size_t nAgents /*= 1*/,
                   const planning2d::Time startTime /*= planning2d::Time(0.0)*/,
                   const Duration timeHorizon /*= Duration(10.0)*/,
                   const OptAgentType type /*= OptAgentType::UNKNOWN*/,
                   const bool activateAllDesignVariables /*= true*/) {

  // populate scene
  const double agentRadius = 0.4;
  const double agentHeight = 1.8;
  const int numSplineSegments = 10;

  const size_t startId = scene.numberOfAgents();
  for (size_t i=0; i<nAgents; ++i) {

    const double sx = random ? sm::random::randLU(0., 10.) : (double)i;
    const double sy = random ? sm::random::randLU(0., 10.) : 1.0;
    const double sh = random ? sm::random::randLU(0., 10.) : 0.0;
    const double svx = random ? sm::random::randLU(0., 2.) : 0.0;
    const double svy = random ? sm::random::randLU(0., 2.) : 0.0;

    const double gx = random ? sm::random::randLU(0., 10.) : (double)i + 10.;
    const double gy = random ? sm::random::randLU(0., 10.) : 1.0;
    const double gh = random ? sm::random::randLU(0., 10.) : 0.0;
    const double gvx = random ? sm::random::randLU(0., 2.) : 0.0;
    const double gvy = random ? sm::random::randLU(0., 2.) : 0.0;

    PedestrianAgent::StateStamped s(PedestrianAgent::State(sx, sy, sh, svx, svy), startTime);
    PedestrianAgent::StateStamped g(PedestrianAgent::State(gx, gy, gh, gvx, gvy), startTime + timeHorizon);

    s.pose().normalizeYaw();
    g.pose().normalizeYaw();

    CollisionGeometryPtr geom(new fcl::Cylinder(agentRadius, agentHeight));
    PedestrianAgent::Ptr agent(new PedestrianAgent(false, startId + i, s, geom));
    Trajectory::Ptr trajectory(new Trajectory());

    trajectory->initStraightSpline(s, g, numSplineSegments, 1e-3);
    OptAgent a(agent, trajectory, type);
    scene.addOptAgent(a);
  }

  scene.activateAllDesignVariables(activateAllDesignVariables);

}


void populateSceneCoordinates(
    ContinuousScene& scene,
    const std::map<planning2d::Id, planning2d::StateTrajectory>& trajCoords,
    const OptAgentType type,
    const bool activateAllDesignVariables /*= true*/) {

  const double agentRadius = 0.4;
  const double agentHeight = 1.8;
  const int numSplineSegments = 10;

  for (auto& idTrajPair : trajCoords) {

    const planning2d::Id id = idTrajPair.first;
    const planning2d::StateTrajectory& trajectoryDiscrete = idTrajPair.second;

    CollisionGeometryPtr geom(new fcl::Cylinder(agentRadius, agentHeight));
    PedestrianAgent::Ptr agent(new PedestrianAgent(false, id, trajectoryDiscrete.front(), geom));
    Trajectory::Ptr trajectory(new Trajectory());

    trajectory->initStraightSpline(trajectoryDiscrete.front(), trajectoryDiscrete.back(), numSplineSegments, 1e-3);
    OptAgent a(agent, trajectory, type);
    scene.addOptAgent(a);
  }

  scene.activateAllDesignVariables(activateAllDesignVariables);
}

StateTrajectory createStateTrajectoryRandomWalk(const size_t nPoints,
                                                const Duration& dt,
                                                const StateStamped& s0,
                                                const double sigma) {

  StateTrajectory st(nPoints);

  StateStamped ss = s0;

  for (size_t i=0; i<nPoints; ++i) {
    st[i] = ss;
    ss.pose().position().x() += sigma*sm::random::randn();
    ss.pose().position().y() += sigma*sm::random::randn();
    ss.pose().yaw() += sigma*sm::random::randn();
    ss.pose().normalizeYaw();
    for (std::size_t j=0; j<ss.dimension(); ++j)
      ss.state()[j] += sigma*sm::random::randn();

    ss.stamp() += dt;
  }
  return st;
}

std::vector<SceneSnapshot::Ptr> toSceneSnapshots(const StateTrajectory& trajectory,
                                                 const Id agentId,
                                                 const Eigen::MatrixXd& invCov) {
  std::vector<SceneSnapshot::Ptr> snapshots;
  for (auto& ss : trajectory) {
    auto snapshot = boost::make_shared<SceneSnapshot>(ss.stamp());
    snapshot->addObject(agentId, StateWithUncertainty(ss, invCov));
    snapshots.push_back(snapshot);
  }
  return snapshots;
}

} // namespace prob_planner
