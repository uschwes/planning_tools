#!/usr/bin/env python

import numpy as np
import pylab as pl

import probabilistic_planner as planner
import probabilistic_planner.util as putil
import common_agents_python as agents
import aslam_backend as opt
import fcl_python as fcl
import sm

# Setup
plotCircles = False

sm.setLoggingLevel(sm.LoggingLevel.Debug)
scene = putil.populateScene(2, 7.0)

fig = pl.figure()
ax = fig.add_subplot(111)
ax.grid('on')
putil.plotScene(scene, ax, plotCircles=plotCircles)

# Aslam optimizer optimization problem
problem = opt.OptimizationProblem()

for id, optAgent in scene.optAgents.iteritems():
    optAgent.trajectory.addDesignVariables(problem)
    print "Added {0} design varibales of agent {1}".format(len(optAgent.trajectory.getDesignVariables()), id)

featureContainer = planner.FeatureContainer()
featureContainer.push_back('singleton_integrated_acceleration', planner.OptAgentType.PEDESTRIAN, 1.0)
featureContainer.push_back('singleton_integrated_velocity', planner.OptAgentType.PEDESTRIAN, 1.0)
featureContainer.push_back('singleton_integrated_rotation_rate', planner.OptAgentType.PEDESTRIAN, 1.0)
featureContainer.push_back('pairwise_integrated_distance', planner.OptAgentType.PEDESTRIAN, 0.0)

featureContainer.addErrorTerms(scene, problem)

# Optimization options
options = opt.OptimizerRpropOptions()
options.verbose = False;
options.maxIterations = 1000;
options.nThreads = 1;

# Optimizer 
optimizer = opt.OptimizerRprop(options)

# Optimize
optimizer.setProblem(problem)
optimizer.checkProblemSetup()
optimizer.initialize()
optimizer.optimize()

putil.plotScene(scene, ax, plotCircles=plotCircles)

pl.show(block=False)