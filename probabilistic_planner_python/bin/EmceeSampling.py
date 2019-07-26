#!/usr/bin/env python

try:
  import emcee
except ImportError,ex:
  raise ImportError(ex.message + ", try 'sudo pip install emcee'")

import numpy as np
import pylab as pl
import probabilistic_planner as pp
import probabilistic_planner.util as pputil
from probabilistic_planner.util import Sampling

np.random.seed(997) # use for deterministic behavior in scene creation

s = pputil.populateScene(2, spline_coeff_noise = 0.002)
scene = Sampling.ContinuousSceneWrapper(s)

usedFeatures = {
                'singleton_integrated_acceleration': (True, pp.OptAgentType.PEDESTRIAN, 20.0),
                'singleton_integrated_velocity': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'singleton_integrated_rotation_rate': (True, pp.OptAgentType.PEDESTRIAN, 10.0),
                'pairwise_integrated_distance': (True, pp.OptAgentType.PEDESTRIAN, 10.0),
                }
features = pp.FeatureContainerModifiable()
for fname,info in usedFeatures.iteritems():
  features.push_back(fname, info[1], info[2])

# setup
ndim = scene.parameters.size
nwalkers = 4*ndim
niterations = 1000
nstepsPerIteration = 1
nstepsBurnIn = 2000
nsamplesVisualize = 1
pos = np.tile(scene.parameters, (nwalkers, 1)) + 5.0*np.random.randn(nwalkers, ndim)
# pos = 1.0*np.random.rand(nwalkers, ndim)
# pos = np.random.rand(ndim * nwalkers).reshape((nwalkers, ndim))
ll_fcn = lambda x: Sampling.log_likelihood(x, features, scene)

# Plot setup
fig = pl.figure(figsize=(20.,12.))
gs = pl.matplotlib.gridspec.GridSpec(4, 1)
ax0 = pl.subplot(gs[0:3, :])
ax0.grid('on')
ax1 = pl.subplot(gs[3, :])
ax1.grid('on')
cm = pputil.generateColormap(scene.base, colormap=pl.get_cmap('winter')) # http://matplotlib.org/users/colormaps.html
axes = [ax0, ax1]
# l,_,_ = pputil.plotScene(scene.base, axes[0], colormap=cm, alpha_traj=1.0, traj_args={'linewidth': 2.0})
pl.show(block=False)

# Sampling
sampler = emcee.EnsembleSampler(nwalkers, ndim, ll_fcn)
print "Burning in..."
pos, prob, state = sampler.run_mcmc(pos, nstepsBurnIn)
drawers = []
expectations = dict([(f.name, []) for f in features.getContainer()])
for i in range(niterations):

  pos, prob, state = sampler.run_mcmc(pos, nstepsPerIteration)
  
  e = dict([(f.name, 0.0) for f in features.getContainer()])
  
  # Delete old artists
  for d in drawers: d.remove()
  drawers = []
  
  # Randomly pick nsamples to visualize
  iVis = np.random.randint(0, nWalkers, nsamplesVisualize)
  
  for iwalker in range(nwalkers):
    scene.parameters = pos[iwalker,:]
    if iwalker in iVis:
      l,_,_ = pputil.plotScene(scene.base, axes[0], colormap=cm, alpha_traj=0.2)
      drawers.extend(l)
    
    for f in features.getContainer(): # incremental mean
      e[f.name] += f.evaluate(scene.base)/nwalkers
    
  # Update axes limits
  lines = [ d for d in drawers if isinstance(d, pl.matplotlib.lines.Line2D) ]
  xmin,ymin,xmax,ymax = 1e40,1e40,-1e40,-1e40
  for l in lines: 
    xmin,xmax = np.min( np.minimum(l.get_data()[0], xmin) ), np.max( np.maximum(l.get_data()[0], xmax) )
    ymin,ymax = np.min( np.minimum(l.get_data()[1], ymin) ), np.max( np.maximum(l.get_data()[1], ymax) )
  offs = 1.0
  axes[0].set_xlim((xmin - offs, xmax + offs))
  axes[0].set_ylim((ymin - offs, ymax + offs))
  
  # Autocorrelation
  tau = sampler.get_autocorr_time(64)
  print "Max. autocorrelation: {0}.".format(np.max(tau))
      
  print "++++++++ Expectations #{0} +++++++++".format(i)
  axes[1].set_color_cycle(None)
  for fname,exp_val in e.iteritems():
    expectations[fname].append(exp_val[0])
    expectations[fname][-1] = np.mean(np.array(expectations[fname])) # Compute as mean over all samples
    print "{0}: {1}".format(fname, exp_val[0])
    l, = axes[1].plot(expectations[fname], label=fname)
    drawers.append(l)
  if axes[1].get_legend() is None: axes[1].legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)

  
  fig.canvas.draw()
#   raw_input("Press any key for next sample...")

  
# sampler.reset()