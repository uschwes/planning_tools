#!/usr/bin/env python

import numpy as np
import pylab as pl
import datetime

import fcl_python
import probabilistic_planner as pp
import probabilistic_planner.util as putil
import probabilistic_planner.util.Plotting as pplot
import probabilistic_planner.util.Optimization as popt
import sm

# Setup
saveToFolder = "plots/optimizer_comparison/{0}".format(datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
pl.matplotlib.rcParams.update({'font.size': 16, 'font.family': 'serif'})
# saveToFolder = None
np.random.seed(666) # use for deterministic behavior in scene creation
usedFeatures = {
#                 'pairwise_integrated_distance': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'pairwise_integrated_inverse_distance': (True, pp.OptAgentType.PEDESTRIAN, 10.0),

                'singleton_integrated_acceleration': (True, pp.OptAgentType.PEDESTRIAN, 10.0),
                'singleton_integrated_velocity': (True, pp.OptAgentType.PEDESTRIAN, 10.0),
                'singleton_integrated_rotation_rate': (True, pp.OptAgentType.PEDESTRIAN, 0.2),
#                 'singleton_integrated_velocity_difference': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'singleton_integrated_direction_of_motion': (True, pp.OptAgentType.PEDESTRIAN, 1.0),

                'measurement_position2d': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
#                 'observation_heading': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
                'measurement_velocity_vector': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
#                 'measurement_absolute_velocity': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
                
#                 'barrier_velocity': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
                }

#Logging
sm.setLoggingLevel(sm.LoggingLevel.Info)
sm.enableNamedLoggingStream("optimization")

# Create scene
scene = putil.populateScene(2, var=7.0, startTime=pp.Time(0.0), timeHorizon=pp.Duration(10.0), splineResolution=pp.Duration(1.0), splineCoeffNoise=0.00, \
                            startPositions=[pp.Position2d(0.0,0.01), pp.Position2d(10.0,0.0)], endPositions=[pp.Position2d(10.0,0.01), pp.Position2d(0.0,0.0)])

scene.activateAllDesignVariables(True) #Activate all design variables
putil.addObservations(scene, [scene.minTime], covTrue=0.1**2*np.eye(5,5)) # generate observations for the first position
scene.activateAllDesignVariables(True) # activate all bspline design variables

# generate and activate features
featureContainer = pp.FeatureContainerModifiable()
for fname,info in usedFeatures.iteritems():
  featureContainer.push_back(fname, info[1], info[2])
  if fname == 'pairwise_integrated_inverse_distance': featureContainer.getContainer()[-1].cutoffDistance = 100.0

# Optimization options
valid_methods = ['ASLAM-RProp+','ASLAM-RProp-','ASLAM-iRProp-','ASLAM-iRProp+','ASLAM-BFGS', 'Nelder-Mead','Powell','CG','BFGS','Newton-CG','L-BFGS-B','TNC','COBYLA','SLSQP']
options = {'disp': True, 'maxiter': 500, 'xtol': 1e-6, 'ftol': 1e-3, 'gtol': 1e-3}
res = {}

# Setup plot
colors,_,markers = pplot.generateUniqueLinestyles(len(valid_methods), colormap='brg')
fig = pl.figure(figsize=(20,12))
fig.canvas.set_window_title('optimizer_comparison')
ax = fig.add_subplot(111)
ax.grid('on')
ax.set_yscale('log')
ax.tick_params(axis='y', which='both')
ax.yaxis.set_major_formatter(pl.FormatStrFormatter("%.1f"))
ax.yaxis.set_minor_formatter(pl.FormatStrFormatter("%.1f"))
ax.set_xlabel('number of gradient evaluations')
ax.set_ylabel('objective fcn')
pl.show(block=False)
callback = lambda x: popt.plot_convergence_callback(sceneOpt, lgrad, abscissa_type=ax.get_xlabel(), ordinate_type=ax.get_ylabel())

# Optimize
for cnt,method in enumerate(valid_methods):
  
  sm.logInfo("Runnig optimization with method {0}".format(method))
  lgrad, = ax.plot([],[], color=colors[cnt], marker=markers[cnt], linestyle='-', label='{0}'.format(method))
  sceneOpt = popt.SceneOptimizable(scene, featureContainer)
  
  res[method] = popt.optimize(sceneOpt, method=method, options=options, callback=callback)
  ax.legend(loc = 'upper right', numpoints = 1, fancybox = True, framealpha = 0.5)

# Save figures
if not saveToFolder is None:
  import os
  if not os.path.isdir(saveToFolder): os.makedirs(saveToFolder) # Create log folder if not existant
  for filetype in ['png', 'pdf']:
    fig.savefig(os.path.join(saveToFolder, "{0}.{1}".format(fig.canvas.get_window_title(), filetype)), bbox_inches='tight')
    sm.logInfo("Saved figures to {0}".format(saveToFolder))
