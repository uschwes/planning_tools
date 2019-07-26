#!/usr/bin/env python

""" Compare the performance of different sampling strategies for the MaxEnt formulation
"""

try:
  import emcee
except ImportError,ex:
  raise ImportError(ex.message + ", try 'sudo pip install emcee'")

import numpy as np
import pylab as pl
import sm
import time
import aslam_backend as opt
import probabilistic_planner as pp
import probabilistic_planner.util as pputil
from probabilistic_planner.util import Sampling

  
def movingAverage(x, windowSize, axis=0):
  windowSize = min(windowSize, x.shape[axis])
  ret = np.cumsum(x, axis=axis)
  ret[windowSize:] = ret[windowSize:] - ret[:-windowSize]
  return ret[windowSize - 1:] / windowSize

def updateLine(hl, x, y):
    hl.set_xdata(np.append(hl.get_xdata(), x))
    hl.set_ydata(np.append(hl.get_ydata(), y))
    
    
def autocorr(x, axis=0, fast=False):
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

def autocorrIntegrated(x, axis=0, window=50, fast=False):
  
  f = autocorr(x, axis=axis, fast=fast)

  # Special case 1D for simplicity.
  if len(f.shape) == 1:
    return 1 + 2*np.sum(f[1:window])

  # N-dimensional case.
  m = [slice(None), ] * len(f.shape)
  m[axis] = slice(1, window)
  tau = 1 + 2*np.sum(f[m], axis=axis)

  return tau


class SamplerHelper(object):
  def __init__(self, name, sampler, features, scene = None, allocateNSamples = 1000, nStepsBurnIn = 0, nSamplesMean=100):
    x = scene.parameters
    ndim = x.size
    self._name = name
    self._nsamples = 0
    self._nSamplesMean = nSamplesMean
    self._nStepsBurnIn = nStepsBurnIn
    self._allocateNSamples = allocateNSamples
    self._sampler = sampler
    self._samples = np.zeros(shape=(allocateNSamples, ndim))
    self._scene = scene
    self._features = features
    self._featureValues = np.zeros(shape=(allocateNSamples, len(features.getContainer())))
        
    # initialize sampler state
    if isinstance(self._sampler, emcee.EnsembleSampler):
#           self._state = np.tile(x, (sampler.k, 1)) + np.random.uniform(-50.0, 50.0, size=(sampler.k, ndim))
          self._state = np.tile(x, (sampler.k, 1)) + 0.01*np.random.randn(sampler.k, ndim)
    else:
          self._state = x
  @property
  def sampler(self): return self._sampler
  @property 
  def samples(self): return self._samples[:self._nsamples]
  @property
  def name(self): return self._name
  @property 
  def nsamples(self): return self._nsamples
  @property
  def featureValues(self): return self._featureValues[:self._nsamples]
  @property
  def acceptanceRate(self):
    if isinstance(self._sampler, emcee.EnsembleSampler):
      return np.mean(self._sampler.acceptance_fraction)
    else:
      return self._sampler.statistics.getWeightedMeanAcceptanceProbability()
    
  @property
  def featureExpectations(self):
    return np.mean(self.featureValues[-self._nSamplesMean::, :], axis=0)

  def allocate(self):
    if self._nsamples >= self._samples.shape[0]: # allocate more memory
      self._samples = np.vstack( (self._samples, np.zeros(shape=(self._allocateNSamples, self._samples.shape[1]))) )
      self._featureValues = np.vstack( (self._featureValues, np.zeros(shape=(self._allocateNSamples, self._featureValues.shape[1]))) )
  
  def __computeFeatureValues(self, x):
    p = self._scene.parameters
    self._scene.parameters = x
    for idx,f in enumerate(self._features.getContainer()):
      self._featureValues[self._nsamples-1,idx] = f.evaluate(self._scene.base)
    self._scene.parameters = p # restore
    
  def addSample(self, sample):
    self.allocate()
    self._samples[self._nsamples,:] = sample
    self._nsamples += 1
    self.__computeFeatureValues(sample)
    
  def tracePlot(self, dims, axes=None, fig=None, **kwargs):
    assert not (axes is None and fig is None), "Either specify list of axes or figure"
    
    if axes is None:
      nH = int(np.ceil(np.sqrt(len(dims))))
      nV = int(np.floor(np.sqrt(len(dims))))
      gs = pl.matplotlib.gridspec.GridSpec(nV, nH)
      axes = []
      for idx,dim in enumerate(dims):
        sub = np.unravel_index([idx], (nV,nH))
        ax = fig.add_subplot(gs[sub[0][0], sub[1][0]])
        axes.append(ax)

    lines = []
    for dim,ax in zip(dims, axes):
      ax.grid('on')
      ax.set_title('dim {0}'.format(dim))
      l,=ax.plot(self.samples[:,dim], label = self.name, **kwargs)
      lines.append(l)
      
    return axes,lines
    
  def autocorr(self, fast=False):
    return autocorr(self.samples, axis=0, fast=fast)
  
  def autocorrIntegrated(self, window=50, fast=False):
    return autocorrIntegrated(self.samples, axis=0, window=window, fast=fast)
  
  def burnIn(self):
    self.generateSamples(self._nStepsBurnIn, store=False)
    
  def generateSamples(self, nSamples, store=True):
    if isinstance(self._sampler, emcee.EnsembleSampler):
      ns = 0
      while ns < nSamples:
        self._state,_,_ = self._sampler.run_mcmc(self._state, 1)
        ns += self._state.shape[0]
        if store:
          for i in range(self._state.shape[0]):
            self.addSample(self._state[i,:])
    else:
      x = self._scene.parameters
      self._scene.parameters = self._state
      for i in range(nSamples):
        self._sampler.run(1)
        self._state = self._scene.parameters
        if store: self.addSample(self._state)
      self._scene.parameters = x # restore

np.random.seed(997) # use for deterministic behavior in scene creation
sm.seed(int(time.time()*1000000)) # required to add randomness to the sampling procedure

s = pputil.populateScene(2, spline_coeff_noise = 0.000)
scene = Sampling.ContinuousSceneWrapper(s)

usedFeatures = {
                'singleton_integrated_acceleration': (True, pp.OptAgentType.PEDESTRIAN, 2.0),
#                 'singleton_integrated_velocity': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'singleton_integrated_rotation_rate': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'pairwise_integrated_distance': (True, pp.OptAgentType.PEDESTRIAN, 10.0),
                }
features = pp.FeatureContainerModifiable()
for fname,info in usedFeatures.iteritems():
  features.push_back(fname, info[1], info[2])
  
negLogDensity = opt.OptimizationProblem()

for id,oa in scene.base.optAgents.iteritems():
  oa.trajectory.addDesignVariables(negLogDensity)

for f in features.getContainer():
  f.addErrorTerms(scene.base, negLogDensity)
  
# Bring the system to it's maximum likelihood state as an alternative to burn in
# nRpropIterations = 1000
# options = opt.OptimizerRpropOptions()
# options.convergenceGradientNorm
# options.verbose=False
# options.initialDelta=0.5
# options.maxIterations=1
# options.nThreads=2
# optimizer = opt.OptimizerRprop(options)
# optimizer.setProblem(negLogDensity)
# gradNorm = np.zeros(shape=(nRpropIterations,))
# ax = pl.figure().gca()
# pl.show(block=False)
# for i in range(nRpropIterations):
# #   if i > 0: 
# #     for l in lines: l.remove()
# #   lines,_,_=pputil.plotScene(scene.base, ax)
# #   ax.figure.canvas.draw()
# #   time.sleep(0.1)
#   optimizer.optimize()
#   gradNorm[i] = optimizer.gradientNorm
# 
# fig = pl.figure()
# pl.show(block=False)
# ax = fig.gca()
# ax.plot(gradNorm)
# ax.grid('on')
# ax.set_xlabel('iteration')
# ax.set_ylabel('gradient norm')


# setup the different samplers
nDim = scene.parameters.size
nWalkers = 10*nDim
nIterations = 1000
nSamplesStaticsticalMean = 100
plotFeatureTrace = True
# nSamplesVisualizeHistory = nSamplesStaticsticalMean*3
nSamplesVisualizeHistory = nIterations*nWalkers
ll_fcn = lambda x: Sampling.log_likelihood(x, features, scene)

# Sampling
samplers = []
samplers.append(SamplerHelper("emcee", emcee.EnsembleSampler(nWalkers, nDim, ll_fcn, a=2.0, live_dangerously=False), features, scene, nIterations*nWalkers, nStepsBurnIn=0, nSamplesMean = nSamplesStaticsticalMean*2))
 
options = opt.SamplerMetropolisHastingsOptions()
options.transitionKernelSigma = 0.02
samplers.append(SamplerHelper("metropolis", opt.SamplerMetropolisHastings(options), features, scene, nIterations*nWalkers, nStepsBurnIn=0, nSamplesMean = nSamplesStaticsticalMean*100))
samplers[-1].sampler.setNegativeLogDensity(negLogDensity)
samplers[-1].sampler.initialize()
 
# options = opt.SamplerHybridMcmcOptions()
# options.leapFrogStepSize = 1.2
# options.nLeapFrogSteps = 50
# options.nThreads = 2
# samplers.append(SamplerHelper("hamiltonian", opt.SamplerHybridMcmc(options), features, scene, nIterations*nWalkers, nStepsBurnIn=0, nSamplesMean = nSamplesStaticsticalMean*100))
# samplers[-1].sampler.setNegativeLogDensity(negLogDensity)
# samplers[-1].sampler.initialize()


# Plot setup
fig = pl.figure(figsize=(20.,12.))
gs = pl.matplotlib.gridspec.GridSpec(len(features.getContainer()), 1)
axes = []
malines = []
sampler_colors = [pl.get_cmap('gist_rainbow')(float(i)/len(samplers)) for i in range(len(samplers))]
for idx,feature in enumerate(features.getContainer()):
  ax = pl.subplot(gs[idx, :], sharex=axes[-1] if len(axes) > 0 else None)
  ax.grid('on')
  ax.set_ylabel(feature.name)
  ax.set_xlabel("iteration" if idx == len(features.getContainer())-1 else '')
  ax.get_yaxis().set_major_locator(pl.MaxNLocator(integer=True))
  ll = []
  for j in range(len(samplers)):
    l,=ax.plot([], [], lw=2, ls='-', color=sampler_colors[j], label=samplers[j].name, marker='x')
    ll.append(l)
  malines.append(ll)
  ax.legend(loc='upper right', numpoints=1, fancybox = True, framealpha = 0.5)
  axes.append(ax)
pl.show(block=False)

# Run the samplers and plot their progress
artists_to_delete = []
np.random.seed(int(time.time()*1000000))

for sampler in samplers:
  print "Burning in sampler {0}".format(sampler.name)
  sampler.burnIn()

for cnt in range(nIterations):
  
  minf = [1e40]*len(features.getContainer())
  maxf = [-1e40]*len(features.getContainer())
  for a in artists_to_delete: a.remove()
  artists_to_delete = []
    
  for ns,(sampler,color) in enumerate(zip(samplers,sampler_colors)):
    
    sampler.generateSamples(nWalkers)
    featureExpectations = sampler.featureExpectations
    print "Aceptance rate {0}: {1}".format(sampler.name, sampler.acceptanceRate)
    
    for nf in range(sampler.featureValues.shape[1]):
      xmin = max(sampler.nsamples-nSamplesVisualizeHistory, 0)
      x = np.array(range(xmin, sampler.nsamples))
      featureValues = sampler.featureValues[-nSamplesVisualizeHistory:,nf]
      expectation = featureExpectations[nf]
      if plotFeatureTrace:
        lfval,=axes[nf].plot(x, featureValues, c=color, ls='--')
        artists_to_delete.append(lfval)
      updateLine(malines[nf][ns], sampler.nsamples, expectation)
      axes[nf].set_xlim( (x[0], x[-1]) )
      minf[nf] = np.min(np.append(malines[nf][ns].get_ydata()[-round(nSamplesVisualizeHistory/nWalkers):], minf[nf]))
      maxf[nf] = np.max(np.append(malines[nf][ns].get_ydata()[-round(nSamplesVisualizeHistory/nWalkers):], maxf[nf]))
  for ax,vmi,vma in zip(axes, minf, maxf):
    ax.set_ylim( (vmi-1.0, vma+1.0) )
  fig.canvas.draw()