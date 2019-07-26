#!/usr/bin/env python

import numpy as np
import pylab as pl
from collections import Iterable, defaultdict
from matplotlib.patches import Ellipse

import probabilistic_planner as planner
import common_agents_python as agents
from aslam_backend import *
import fcl_python as fcl
from probabilistic_planner.libprobabilistic_planner_python import ContinuousScene


def populateScene(nAgents, var=5.0, startTime=planner.Time(0.0), timeHorizon=planner.Duration(10.0), \
                  splineResolution=planner.Duration(1.0), splineCoeffNoise=0.0, startPositions=None, \
                  endPositions=None, agentTypes=None, radius=0.4, splineLambda=1e-3):
  """ Creates a scene with some agents in it
  """
  scene = planner.ContinuousScene()

  startTime = startTime
  endTime = startTime + timeHorizon
  
  for i in range(nAgents):
    
    posStart = planner.Position2d(np.random.rand(2,)*var) if startPositions is None or i >= len(startPositions) else startPositions[i]
    posEnd = planner.Position2d(np.random.rand(2,)*var) if endPositions is None or i >= len(endPositions) else endPositions[i]
    vel = (posEnd - posStart).cwiseProduct(1./timeHorizon.toSec())
    yaw = np.arctan2(vel.y, vel.x)
      
    s = agents.HolonomicStateStamped(agents.HolonomicState(vel.x, vel.y, yaw, posStart), startTime)
    g = agents.HolonomicStateStamped(agents.HolonomicState(vel.x, vel.y, yaw, posEnd), endTime)
    s.pose.normalizeYaw()
    g.pose.normalizeYaw()

    sm.logInfo("populateScene: Creating agent {0} at {1} with velocity {2}".format(i, s.pose, s.velocity))
    
    geom = fcl.Cylinder(radius, 1.8)
    a = agents.HolonomicAgent(False, i, s, geom)
    
    t = planner.Trajectory()
    numSegments = int(np.ceil(timeHorizon.toSec()/splineResolution.toSec()))
    t.initStraightSpline(s, g, numSegments, splineLambda)
    agent = planner.OptAgent(a, t, planner.OptAgentType.PEDESTRIAN if agentTypes is None or i >= len(agentTypes) else agentTypes[i])
    scene.addOptAgent(agent)
    
  for id,optAgent in scene.optAgents.iteritems():
    dvs = optAgent.trajectory.getDesignVariables()
    for i in range(len(dvs)):
      dv = dvs[i]
      md = dv.minimalDimensions()
      dv.update(splineCoeffNoise*np.random.randn(md,))
    
  return scene

def addStateObservations(scene, stampsPerAgent, covTrue=0.1*np.eye(5), invCovAssumed=None):
  """ Note: ContinuousScene uses a pointer underneath, so we can do in-place modification
  """
  if invCovAssumed is None: invCovAssumed = np.linalg.inv(covTrue)
  for agentId,stamps in stampsPerAgent.iteritems():
    stamps = [stamps] if not isinstance(stamps, Iterable) else stamps
    for stamp in stamps:
      obs = planner.SceneSnapshot(stamp)
      optAgent = scene.optAgents[agentId]
      if not (stamp >= optAgent.trajectory.startTime and stamp <= optAgent.trajectory.finalTime):
        raise RuntimeError("Invalid timestamp {0} for agent {1} supplied".format(stamp, agentId)) 
      tpos = optAgent.trajectory.getPosition(stamp) + np.random.multivariate_normal(np.zeros(2,), covTrue[0:2, 0:2])
      tvel = optAgent.trajectory.getVelocityXY(stamp) + np.random.multivariate_normal(np.zeros(2,), covTrue[3:5, 3:5])
      state = agents.HolonomicStateStamped(agents.HolonomicState(tvel, planner.Pose2d(planner.Position2d(tpos), covTrue[3,3])), stamp)
      su = planner.StateWithUncertainty(state, invCovAssumed)
      obs.addObject(optAgent.id, su)
      scene.addObservation(obs)
    
def addGridObservations(scene, gridsStamped):
  """ Note: ContinuousScene uses a pointer underneath, so we can do in-place modification
  """
  for grid in gridsStamped:
    obs = planner.SceneSnapshot(grid.stamp)
    obs.occupancyGrid = grid
    scene.addObservation(obs)
    
def numObservationsPerAgent(scene):
  observations = defaultdict(lambda: 0, {})
  for agentId,optAgent in scene.optAgents.iteritems():
    for snapshots in scene.getObservations():
      for obsAgentId,_ in snapshots.objectContainer.iteritems():
        if agentId == obsAgentId: observations[agentId] += 1
  return observations
    
def getTrajectory(scene, agentId, resolution=planner.Duration(0.1)):
  """ Extracts a discretized trajectory for a specific agent from the scene
  """
  trajectory = scene.optAgents[agentId].trajectory
  if not trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
  stamps = [planner.Time(d) for d in np.arange(trajectory.startTime.nanosec, trajectory.finalTime.nanosec, resolution.nanosec)]
  discretized = [trajectory.getPosition(stamp) for stamp in stamps]
  return np.asarray(discretized).T,stamps
    
def flatten(ll):
  """ Flattens a nested list of lists
  """
  flattened = []
  if isinstance(ll, Iterable):
    for l in ll:
      flattened.extend(flatten(l))
  else:
    flattened.append(ll)
  return flattened

def getSplineParametersFromSamples(samples):
  """ Returns a MxN matrix of spline parameters, where M is the number of samples and N the dimensionality of the scene
  """
  parameters = []
  for sample in samples:
    parameters.append(sample.activeSplineParameters)
  return np.asarray(parameters)

def computeMode(scene, featureContainer, method="ASLAM-BFGS", options={'disp': False, 'maxiter': 500, 'xtol': 1e-6, 'ftol': 1e-3, 'gtol': 0.1}):
  """ Compute the mode of the maxent distribution
  """
  from probabilistic_planner.util import Optimization as popt
  isEnabled = sm.isNamedLoggingStreamEnabled("optimization")
  sm.disableNamedLoggingStream("optimization")
  sceneOpt = popt.SceneOptimizable(scene, featureContainer)
  res = popt.optimize(sceneOpt, method=method, options=options)
  if 'success' in res.keys() and not res['success']: sm.logError("{0}".format(res))
  if isEnabled: sm.enableNamedLoggingStream("optimization")
  return sceneOpt.scene
  
def computeMean(samples):
  """ Compute the mean spline from a given set of samples
  """
  assert len(samples) > 0
  sceneMean = planner.ContinuousSceneModifiable(samples[0])
  parameters = getSplineParametersFromSamples(samples)
  mean = np.mean(parameters, axis=0)
  sceneMean.setActiveSplineParameters(mean)
  return sceneMean
  
def computeFeatureValues(samples, feature):
  """ Compute the feature values for a given set of samples.
  """
  return np.array( [feature.evaluate(sample) for sample in samples] )

def autoCorr(x, axis=0, fast=False):
  """ Compute the autocorrelation function of x.
  """
  x = np.atleast_1d(x)
  m = [slice(None), ] * len(x.shape)

  # For computational efficiency, crop the chain to the largest power of two if requested.
  if fast:
      n = int(2**np.floor(np.log2(x.shape[axis])))
      m[axis] = slice(0, n)
      x = x
  else:
      n = x.shape[axis]

  # Compute the FFT and then (from that) the auto-correlation function.
  f = np.fft.fft(x-np.mean(x, axis=axis), n=2*n, axis=axis)
  m[axis] = slice(0, n)
  acf = np.fft.ifft(f * np.conjugate(f), axis=axis)[m].real
  m[axis] = 0
  return acf / acf[m]

def autoCorrSplineParameters(samples, fast=False):
  """ Compute the autocorrelation function of the spline parameters.
  """
  x = getSplineParametersFromSamples(samples)
  return autoCorr(x, axis=0, fast=fast)

def autoCorrFeatureValues(samples, featureContainer, fast=False):
  """ Compute the autocorrelation function of the feature values.
  """
  acorr = {}
  for feature in featureContainer.getContainer():
    x = computeFeatureValues(samples, feature)
    acorr[feature.name] = autoCorr(x, axis=0, fast=fast)
  return acorr

def negloglik(featureContainer, scene):
  from aslam_backend import OptimizationProblem
  problem = OptimizationProblem()
  scene.addDesignVariables(problem, False)
  featureContainer.addErrorTerms(scene, problem)
  return problem.evaluateError()
