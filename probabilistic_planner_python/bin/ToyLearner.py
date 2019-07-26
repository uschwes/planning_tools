#!/usr/bin/env python

import numpy as np
from collections import defaultdict, Iterable
import datetime, os, shutil, argparse, itertools

import fcl_python
import probabilistic_planner as pp
import probabilistic_planner.util as putil
from probabilistic_planner.util import Plotting as pplot
import common_agents_python as agents
import aslam_backend as opt
import fcl_python as fcl
import sm

def checkArgs(args):
  assert argconf.minNumSamples >= 0
  assert argconf.maxNumSamples >= argconf.minNumSamples
  assert argconf.numOccupiedGridCells >= 0
  assert argconf.numSamplesBurnIn >= 0
  assert argconf.replayFolder is None or os.path.exists(argconf.replayFolder)
  assert argconf.replayIteration is None or argconf.replayIteration >= 0
  
# Parse command line
parser = argparse.ArgumentParser(description="Learn feature weights from artifical or real data")
parser.add_argument("-f", "--log-folders", nargs='+', type=str, dest="logFolders", default=None, help="Moos logfolder or empty for artifical data")
parser.add_argument("--feature-file", type=str, dest="featureFile", default=None, help="Xml property tree file path for features")
parser.add_argument("-v", "--verbosity", type=str, dest="verbosity", default="Info", choices=['All', 'Finest', 'Verbose', 'Finer', 'Trace', 'Fine', 'Debug', 'Info', 'Warn', 'Error', 'Fatal'], help="Verbosity level")
parser.add_argument("--enable-named-streams", nargs='+', type=str, default = [], dest="enabledStreams")
parser.add_argument("--no-plots", dest = "noPlots", action="store_true", default=False, help="Don't show plots")
parser.add_argument("--plot-initial-scene", dest = "plotInitialScene", action="store_true", default=False, help="Show the scene without samples")
parser.add_argument("-q", "--quiet", dest = "quiet", action="store_true", default=False, help="Be quiet. Don't show the plots. Just save a pdf")
parser.add_argument("--scale-features", dest = "scaleFeatures", action="store_true", default=False, help="Apply scaling to features")
parser.add_argument("--num-samples-min", dest = "minNumSamples", type=int, default=100, help="Minimum number of samples to use for expectations")
parser.add_argument("--num-samples-max", dest = "maxNumSamples", type=int, default=2000, help="Maximum number of samples to use for expectations")
parser.add_argument("--num-iterations-max", dest = "maxNumIterations", type=int, default=1000, help="Maximum number of iterations")
parser.add_argument("--save-folder", dest = "saveFolder", type=str, default="generated/sampling/", help="Save figures/samples to this location")
parser.add_argument("--replay-folder", dest = "replayFolder", type=str, default=None, help="Replay previously saved samples from this directory")
parser.add_argument("--replay-iteration", dest = "replayIteration", type=int, default=None, help="Replay this iteration from the replay folder")
parser.add_argument("--dont-save-figures", dest = "dontSaveFigures", action="store_true", default=False, help="Do not save figures")
parser.add_argument("--dont-save-samples", dest = "dontSaveSamples", action="store_true", default=False, help="Do not save samples")
parser.add_argument("--wait-after-iteration", dest = "waitAfterIteration", action="store_true", default=False, help="Require user input to proceed after each iteration")
parser.add_argument("--sampler-initialize-at-mode", dest = "initializeAtMode", action="store_true", default=True, help="Initialize the sampler at the mode of the distribution")
parser.add_argument("--sampler-num-samples-burnin", dest = "numSamplesBurnIn", type=int, default=10, help="Number of samples to burn in the sampler")
parser.add_argument("--sampler-seed", dest = "samplerSeed", type=int, default=None, help="Deterministic seed for the sampler")
parser.add_argument("--scene-seed", dest = "sceneSeed", type=int, default=None, help="Deterministic seed for the scene creation")
parser.add_argument("--use-scaling", dest = "useScaling", action="store_true", default=False, help="Activate feature scaling")
parser.add_argument("--num-occupied-grid-cells", dest = "numOccupiedGridCells", type=int, default=10, help="Number of randomly occupied grid cells in the grid")
argconf = parser.parse_args()
checkArgs(argconf)

import matplotlib
if argconf.quiet: matplotlib.use('Agg', warn=False)
import matplotlib.pyplot as pl
from matplotlib.backends.backend_pdf import PdfPages

# Setup
saveToFolder = os.path.join(argconf.saveFolder, "{0}".format(datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
pl.matplotlib.rcParams.update({'font.size': 16, 'font.family': 'serif'})
np.random.seed(argconf.sceneSeed) # use for deterministic behavior in scene creation
sm.seed(int(datetime.datetime.now().microsecond) if argconf.samplerSeed is None else argconf.samplerSeed) # required to add randomness to the sampling procedure

# plots = set(['scene', 'expectations_vs_demonstrations', 'feature_weights_progress', 'occupancy', \
#              'feature_histogram', 'spline_parameter_distribution', 'spline_parameter_autocorrelation', \
#              'feature_value_autocorrelation', 'parameter_trace', 'feature_expectation_trace'])
# plots = set(['scene', 'feature_weights_progress'])
plots = set(['scene'])

#Logging setup
sm.setLoggingLevel(sm.loggingLevelFromString(argconf.verbosity))
for stream in argconf.enabledStreams: sm.enableNamedLoggingStream(stream)

scenes = pp.ContinuousSceneList()
if argconf.logFolders is None:
  # Create scene(s) with some noise on coefficients
  dvSigma = 0.1
  scene = pp.ContinuousSceneModifiable(putil.populateScene(2, var=7.0, startTime=pp.Time(0.0), timeHorizon=pp.Duration(10.0), splineResolution=pp.Duration(1.0), splineCoeffNoise=0.00, \
                              startPositions=[pp.Position2d(0.0,0.01), pp.Position2d(10.0,-1.0)], endPositions=[pp.Position2d(10.0,0.01), pp.Position2d(0.0,-3.0)]))
  scene.activateAllDesignVariables(True)
  
  #Add some noise to coefficients
  p = scene.activeSplineParameters
  scene.setActiveSplineParameters(p + dvSigma*np.random.randn(p.shape[0]))
  
  # Create list of scenes
  scenes.append(scene)
  
  # generate observations for the first positions
  for scene in scenes: putil.addStateObservations(scene, dict(zip(scene.optAgents.keys(), [oa.trajectory.startTime for oa in scene.optAgents.values()])), \
                                                  covTrue=0.001**2*np.eye(5,5), invCovAssumed=1./0.1**2*np.eye(5,5))
  
  # Add a grid measurement
  xlim,ylim = pplot.computeBoundingBox(scenes)
  gridBorder = 3.0
  gridSize = np.array([int((xlim[1] - xlim[0] + 2.*gridBorder)/0.1), int((ylim[1] - ylim[0] + 2.*gridBorder)/0.1)])
  grid = pp.OccupancyGridStamped(pp.Pose2d(xlim[0]-gridBorder,ylim[0]-gridBorder,0.0), 0.1, \
                                 pp.OccupancyValue.FREE*np.ones(shape=(gridSize[1],gridSize[0]), dtype=np.ubyte), pp.Time(0.0))
  # Random obstacles
  for cnt in range(0, argconf.numOccupiedGridCells):
    grid.setAtIndex(pp.MapIndex( np.random.randint(0, grid.sizeInCellsX), np.random.randint(0, grid.sizeInCellsY)), pp.OccupancyValue.OCCUPIED)
  putil.addGridObservations(scene, [grid])

else:
  from local_planner_europa2.util import logFolder2Scenes
  _scenes = list(itertools.chain.from_iterable([ logFolder2Scenes(folder, sceneDuration=pp.Duration(10.0), staticGrid=True, staticAgentRadius=0.3) for folder in argconf.logFolders ]))
  for s in _scenes: scenes.append(s)
  
if len(scenes) == 0: 
  sm.logInfo("Nothing to process, exiting")
  import sys
  sys.exit()

# activate all bspline design variables (for sampling)
for scene in scenes: scene.activateAllDesignVariables(True)

# Make a backup copy since the learner will modify the scenes, these are the demonstrations
demonstrations = pp.ContinuousSceneList()
for scene in scenes: demonstrations.append( scene.copy() )

# get all agent ids from all scenes
agentIds = putil.flatten([ scene.optAgents.keys() for scene in scenes ])

# generate and activate features
if argconf.featureFile is None:
  usedFeatures = {
#                 'pairwise_integrated_distance': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'pairwise_integrated_inverse_distance': (True, pp.OptAgentType.PEDESTRIAN, 9.0),
                'singleton_integrated_acceleration': (True, pp.OptAgentType.PEDESTRIAN, 10.0),
                'singleton_integrated_velocity': (True, pp.OptAgentType.PEDESTRIAN, 11.0),
                'singleton_integrated_velocity_difference': (True, pp.OptAgentType.PEDESTRIAN, 2.0),
#                 'singleton_integrated_rotation_rate': (True, pp.OptAgentType.PEDESTRIAN, 0.2),
                'singleton_integrated_direction_of_motion': (True, pp.OptAgentType.PEDESTRIAN, 1.0),
                'measurement_position2d': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
#                 'observation_heading': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
                'measurement_velocity_vector': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
#                 'measurement_absolute_velocity': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
                'singleton_integrated_static_obstacle_distance': (True, pp.OptAgentType.PEDESTRIAN, 8.0),
#                 'barrier_velocity': (False, pp.OptAgentType.PEDESTRIAN, 1.0),
                }
  featureContainer = pp.FeatureContainerModifiable()
  for fname,info in usedFeatures.iteritems():
    featureContainer.push_back(fname, info[1], info[2])
  
  for f in featureContainer.getContainer():
    if f.name == 'pairwise_integrated_inverse_distance': f.cutoffDistance = 100.0
    if f.name == 'singleton_integrated_static_obstacle_distance': f.setSigmoidParameters(100.0, 10.0, 0.0)
    if f.name == 'singleton_integrated_velocity_difference': f.desiredVelocity = 2.0
else:
  featureContainer = pp.FeatureContainer(argconf.featureFile)
  
# Check that each agent has an observation
for scene in scenes:
  numObservationsPerAgent = putil.numObservationsPerAgent(scene)
  if np.min(numObservationsPerAgent.values()) == 0:
    raise RuntimeError("Not all agents have at least one observation attached")
  
deactivatedFeatures = set(['measurement_position2d', 'measurement_velocity_vector', 'barrier_velocity'])
for f in featureContainer.getContainer():
  f.activateForLearning(not f.name in deactivatedFeatures)
  f.forbidNegativeWeights(True)

# scale features
if argconf.scaleFeatures: pp.scaleAllFeatures(featureContainer, scenes)

activeFeatures = [f for f in featureContainer.getContainer() if f.numActiveWeights > 0]
activeFeatureNames = [f.name for f in activeFeatures]
numActiveFeatures = len(activeFeatures)

# generate colors (http://matplotlib.org/users/colormaps.html)
cmAgents = pplot.generateColormap(scenes, colormap=pl.get_cmap('rainbow'))
# cmAgents = {0: (0.4, 0.4, 0.4, 1.0), 1: (0.5, 0.0, 1.0, 1.0)}
cmFeatures = dict(zip(activeFeatureNames, pplot.generateColors(numActiveFeatures, colormap=pl.get_cmap('jet'))))
_,lsFeatures,msFeatures = pplot.generateUniqueLinestyles(numActiveFeatures)
lsFeatures = dict(zip(activeFeatureNames, lsFeatures))
msFeatures = dict(zip(activeFeatureNames, msFeatures))

# figure setup
def figure_setup(scenes, plots=set(['scene', 'expectations_vs_demonstrations', 'feature_weights_progress', 'occupancy', \
                                    'feature_histogram', 'spline_parameter_distribution', 'spline_parameter_autocorrelation', \
                                    'feature_value_autocorrelation', 'parameter_trace', 'feature_expectation_trace'])):
  figuresPerScene = []
  axesPerScene = []
  dec = int(np.floor(np.log10(len(scenes)))) + 1
  for cnt,scene in enumerate(scenes):
    
    figuresThisScene = []

    axes = {}

    if 'scene' in plots:
      figuresThisScene.append(pl.figure(figsize=(20.,12.))) # new figure per demonstration
      figuresThisScene[-1].canvas.set_window_title('scene_{0}'.format(str(cnt).zfill(dec)))
      ax = figuresThisScene[-1].add_subplot(111)
      ax.grid('on')
      ax.set_xlabel('x [m]')
      ax.set_ylabel('y [m]')
      grid = None
      for o in reversed(scene.getObservations()):
        grid = o.occupancyGrid
        if not grid is None: break
      if not grid is None:
        try:
          from planner_algorithms import distanceTransform
          dt = distanceTransform(grid)
          pplot.plotGrid(dt, ax, cmap='Greys_r', interpolation='none', mask_value=np.NAN, rotate_args={'mode': 'constant', 'order': 0, 'cval': np.NAN})
        except ImportError:
          sm.logWarn("Cannot import planner_algorithms, not showing distance transform")
      pplot.plotScene(scene, ax, show_radii=False, show_observations=True, show_observation_ellipses=False, show_gridmaps=True, colormap=cmAgents, alpha_obs_ellipses=0.05, traj_args={'linewidth': 2.0}, \
                      arrow_head_width=0.2, grid_args={'plot_border': True, 'plot_origin': True}, grid_rotate_args={'mode': 'constant', 'order': 0, 'cval': pp.OccupancyValue.FREE})
      ax.legend(loc = 'best', numpoints = 1, fancybox = True, framealpha = 0.5)
      ax.set_aspect('equal')
      axes['scene'] = ax
    
    if 'expectations_vs_demonstrations' in plots:
      figuresThisScene.append( pl.figure(figsize=(20.,12.)) ) # new figure per demonstration
      figuresThisScene[-1].canvas.set_window_title('expectations_vs_demonstrations_scene{0}'.format(str(cnt).zfill(dec)))
      ax = figuresThisScene[-1].add_subplot(111) # Expectations
      ax.grid('on')
      ax.set_ylabel('feature value')
      axes['expectations_vs_demonstrations'] = ax

    if 'feature_weights_progress' in plots and cnt == 0: # Note: This will be the same for all plots
      figuresThisScene.append( pl.figure(figsize=(20.,12.)) ) # new figure per demonstration
      figuresThisScene[-1].canvas.set_window_title('feature_weights_progress')
      ax = figuresThisScene[-1].add_subplot(111) # Gradient norm  
      ax.grid('on', which='both', axis='y')
      ax.grid('on', which='major', axis='x')
      ax.set_xlabel('iteration')
      ax.set_ylabel('gradient norm')
      ax.set_yscale('log')
      ax.get_xaxis().set_major_locator(pl.MaxNLocator(integer=True))
      ax.tick_params(axis='y', which='both')
      ax.yaxis.set_major_formatter(pl.FormatStrFormatter("%.1f"))
      ax.yaxis.set_minor_formatter(pl.FormatStrFormatter("%.1f"))
      ax2 = ax.twinx()
      ax2.set_ylabel('feature weight')
      ax2.grid('on', which='major', axis='y')
      axes['feature_weights_progress'] = [ax, ax2]

    if 'occupancy' in plots:
      figuresThisScene.append( pl.figure(figsize=(20.,12.)) )
      figuresThisScene[-1].canvas.set_window_title('occupancy')
      ax = figuresThisScene[-1].add_subplot(111)
      ax.grid('on')
      ax.set_xlabel('x')
      ax.set_ylabel('y')
      axes['occupancy'] = ax
     
    if 'feature_histogram' in plots:
      figuresThisScene.append( pl.figure(figsize=(20.,12.)) )
      figuresThisScene[-1].canvas.set_window_title('feature_histogram_scene{0}'.format(str(cnt).zfill(dec)))
      ax = figuresThisScene[-1].add_subplot(111)
      ax.grid('on')
      ax.set_xlabel('feature value')
      ax.set_ylabel('probability')
      axes['feature_histogram'] = ax

    if 'spline_parameter_distribution' in plots:
      nAxes = scene.numActiveSplineParameters
      fig1,axs = pplot.squareTileAxesLayout(nAxes, figsize=(20.,12.))
      figuresThisScene.append(fig1)
      figuresThisScene[-1].canvas.set_window_title('spline_parameter_distribution_scene{0}'.format(str(cnt).zfill(dec)))
      axs = putil.flatten(axs)
      axes['spline_parameter_distribution'] = axs
      figuresThisScene[-1].tight_layout()
    
    if 'spline_parameter_autocorrelation' in plots:
      nAxes = scene.numActiveSplineParameters
      fig2,axs = pplot.squareTileAxesLayout(nAxes, figsize=(20.,12.))
      figuresThisScene.append(fig2)
      axs = putil.flatten(axs)
      for ax in axs: ax.grid('on')
      figuresThisScene[-1].canvas.set_window_title('spline_parameter_autocorrelation_scene{0}'.format(str(cnt).zfill(dec)))
      axes['spline_parameter_autocorrelation'] = axs
      figuresThisScene[-1].tight_layout()
     
    if 'feature_value_autocorrelation' in plots:
      nAxes = len(featureContainer.getContainer())
      fig3,axs = pplot.squareTileAxesLayout(nAxes, figsize=(20.,12.))
      figuresThisScene.append(fig3)
      axs = putil.flatten(axs)
      for ax in axs: ax.grid('on')
      figuresThisScene[-1].canvas.set_window_title('feature_value_autocorrelation_scene{0}'.format(str(cnt).zfill(dec)))
      axes['feature_value_autocorrelation'] = axs
      figuresThisScene[-1].tight_layout()
      
    if 'parameter_trace' in plots:
      nAxes = scene.numActiveSplineParameters
      fig1,axs = pplot.squareTileAxesLayout(nAxes, figsize=(20.,12.))
      figuresThisScene.append(fig1)
      figuresThisScene[-1].canvas.set_window_title('parameter_trace_scene{0}'.format(str(cnt).zfill(dec)))
      axs = putil.flatten(axs)
      axes['parameter_trace'] = axs
      figuresThisScene[-1].tight_layout()
      
    if 'feature_expectation_trace' in plots:
      nAxes = len(featureContainer.getContainer())
      fig1,axs = pplot.squareTileAxesLayout(nAxes, figsize=(20.,12.))
      figuresThisScene.append(fig1)
      figuresThisScene[-1].canvas.set_window_title('feature_expectation_trace_scene{0}'.format(str(cnt).zfill(dec)))
      axs = putil.flatten(axs)
      for ax in axs: ax.grid('on')
      axes['feature_expectation_trace'] = axs
      figuresThisScene[-1].tight_layout()
    
    axesPerScene.append( axes )
    figuresPerScene.append(figuresThisScene)
    
  return figuresPerScene, axesPerScene

# Plot initial scene
if argconf.plotInitialScene:
  figure_setup(demonstrations, plots=['scene'])
  pl.show(block=False)
  if argconf.waitAfterIteration: raw_input("Press enter to continue...")

# Learner options
options = pp.ProbabilisticLearnerOptions()
# options.samplerType = pp.SamplerType.METROPOLIS_HASTINGS
options.samplerType = pp.SamplerType.HYBRID_MONTE_CARLO

options.rpropOptions = opt.OptimizerOptionsRprop()
options.rpropOptions.maxIterations = 1
options.rpropOptions.nThreads = 2
options.rpropOptions.initialDelta = 1.5
options.rpropOptions.maxDelta = 3.0
options.rpropOptions.convergenceGradientNorm = 0.5

# Create regularizer term
beta = 1.0
featureWeightDvs = opt.ScalarDesignVariableVector()
for feature in featureContainer.getContainer():
  for i in range(feature.numWeights()):
    featureWeightDvs.append(feature.getWeightAsDesignVariable(i))
options.rpropOptions.regularizer = opt.L1Regularizer(featureWeightDvs, beta)

plotSamplesSubsampleFactor = 1
options.etOptions = pp.ErrorTermLearningOptions()
if options.samplerType == pp.SamplerType.HYBRID_MONTE_CARLO:
  options.etOptions.nMcmcStepsBurnIn = argconf.numSamplesBurnIn
  options.etOptions.nMcmcSamplesForMean = 1
  options.etOptions.nThin = 1
  plotSamplesSubsampleFactor = 1
  options.etOptions.storeSamples = True
elif options.samplerType == pp.SamplerType.METROPOLIS_HASTINGS:
  options.etOptions.nMcmcStepsBurnIn = argconf.numSamplesBurnIn
  options.etOptions.nMcmcSamplesForMean = 5000
  plotSamplesSubsampleFactor = 5
  options.etOptions.storeSamples = True

options.etOptions.initializeAtMode = argconf.initializeAtMode
options.etOptions.resetToDemonstration = True

options.metropolisHastingsOptions.transitionKernelSigma = 0.05
options.metropolisHastingsOptions.nThreadsEvaluateLogDensity = 1

options.hybridMonteCarloOptions.initialLeapFrogStepSize = 0.01
options.hybridMonteCarloOptions.decFactorLeapFrogStepSize = 0.98
options.hybridMonteCarloOptions.incFactorLeapFrogStepSize = 1.02
options.hybridMonteCarloOptions.minLeapFrogStepSize = 0.0001
options.hybridMonteCarloOptions.maxLeapFrogStepSize = 0.1
options.hybridMonteCarloOptions.nLeapFrogSteps = 500
options.hybridMonteCarloOptions.targetAcceptanceRate = 0.69
options.hybridMonteCarloOptions.standardDeviationMomentum = 1.0
options.hybridMonteCarloOptions.nThreads = 2

# Learn 
learner = pp.ProbabilisticLearner(featureContainer, options)
learner.addDemonstrations(scenes)
sm.logInfo("Learning model...")
gradientNorm = []
yMaxBarPlot = None

def adaptNumberOfSamples(iteration, learner, nlow=50, nhigh=500, cntLow=10, cntHigh=20):
  errorTermOptions = learner.options.etOptions
  if iteration <= cntLow:
    errorTermOptions.nMcmcSamplesForMean = nlow
  elif iteration < cntHigh and iteration > cntLow:
    errorTermOptions.nMcmcSamplesForMean = nlow + (iteration-cntLow)*(nhigh-nlow)/(cntHigh-cntLow)
  else:
    errorTermOptions.nMcmcSamplesForMean = nhigh
  learner.setErrorTermOptions(errorTermOptions)
  sm.logDebug("Setting number of samples for expectation to {0}".format(learner.options.etOptions.nMcmcSamplesForMean))

startIdx = 0 if not argconf.replayFolder or argconf.replayIteration is None else argconf.replayIteration
endIdx = argconf.maxNumIterations if not argconf.replayFolder or argconf.replayIteration is None else argconf.replayIteration + 1
for cnt in range(startIdx, endIdx):

  # Compute analytic mode
  plotMode = False
  if plotMode:
    modeAnalytic = [ putil.computeMode(scene, featureContainer) for scene in scenes ]
  else:
    modeAnalytic = [None]*len(scenes)

  sm.logInfo("Learning iteration {0}".format(cnt))
  
  if argconf.replayFolder is None:
    adaptNumberOfSamples(cnt, learner, nlow=argconf.minNumSamples, nhigh=argconf.maxNumSamples, cntLow=10, cntHigh=50)
    options.rpropOptions.maxIterations = learner.optimizer.status.numIterations + 1
    learner.setOptimizerRpropOptions(options.rpropOptions)
    learner.run()
    samplesAllScenes = learner.samples
    featureInfosAllScenes = learner.featureInfo
    gradientNorm.append(learner.optimizer.status.gradientNorm)
    
  else:
    import pickle
    samplesAllScenes,gradientNorm,featureInfosAllScenes = pickle.load(open(os.path.join(argconf.replayFolder, "samples_iteration{0}.pc".format(cnt)), "rb"))
    sm.logInfo("Loaded information for {0} scenes for iteration {1} from {2}".format(len(samplesAllScenes), cnt, argconf.replayFolder))

  # Create output folder if not existant
  if (not argconf.dontSaveFigures or not argconf.dontSaveSamples or argconf.replayFolder is None) and not os.path.isdir(saveToFolder):  # Create log folder if not existant
    os.makedirs(saveToFolder)

  # Save samples
  if argconf.replayFolder is None and not argconf.dontSaveSamples:
    import pickle
    filename = os.path.join(saveToFolder, "samples_iteration{0}.pc".format(cnt))
    pickle.dump((samplesAllScenes, gradientNorm, featureInfosAllScenes), open(filename, "wb"))
    sm.logInfo("Saved information for {0} scenes for iteration {1} to {2}".format(len(samplesAllScenes), cnt, filename))
    

  if not argconf.noPlots:
    pl.close('all')
    figuresPerScene, axesPerScene = figure_setup(demonstrations, plots=plots)
    
    # Plot demonstrations vs. expectations
    for fiPerDem,samples,axes in zip(featureInfosAllScenes, samplesAllScenes, axesPerScene):
      if not 'expectations_vs_demonstrations' in axes: continue
      ax = axes['expectations_vs_demonstrations']
      bars = pplot.plotFeatureExpectationVsDemonstration(ax, fiPerDem, featureContainer, edgecolor='black')
      if ax.get_legend() is None: ax.legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
      if yMaxBarPlot is None: # make sure we have a fixed height
        heights = []
        for bb in bars:
          for b in bb:
            heights.append(b.get_height())
        yMaxBarPlot = 1.2*np.max( heights )
      ax.set_ylim( (0, yMaxBarPlot) )
    
    # Plot gradient norm
    _,linestyles,markerstyles = pplot.generateUniqueLinestyles(numActiveFeatures, colormap='brg')
    for axes,featureInfos in zip(axesPerScene, featureInfosAllScenes):
      if not 'feature_weights_progress' in axes: continue
      axs = axes['feature_weights_progress']
      l0,=axs[0].plot(gradientNorm, '-d', color='black', label='gradient_norm', markeredgecolor='black', linewidth=3, markersize=10, markeredgewidth=2)
      if axs[0].get_legend() is None: axs[0].legend(loc='upper left', numpoints=1, fancybox = True, framealpha = 0.5)
      axs[1].set_color_cycle(None)
      for fi in featureInfos:
        l1,=axs[1].plot(np.asarray(fi.weights).ravel(), color=cmFeatures[fi.name], markeredgecolor=cmFeatures[fi.name], ls=lsFeatures[fi.name], \
                       marker=msFeatures[fi.name], label=fi.name, linewidth=2, markersize=10, markeredgewidth=2)
      if axs[1].get_legend() is None: axs[1].legend(loc='upper right', numpoints=1, fancybox = True, framealpha = 0.5)
    
    # Plot new samples
    for samples,mode,axes in zip(samplesAllScenes, modeAnalytic, axesPerScene):
      if not 'scene' in axes: continue
      ax = axes['scene']
      t=pplot.plotSamples(samples, ax, featureContainer, subsample=plotSamplesSubsampleFactor, plot_mode=plotMode, plot_mean=False, mode=mode, \
                          show_radii=False, show_observations=False, colormap=cmAgents, traj_args={'linestyle': '-'}, alpha_traj=0.2)
      ax.legend(loc = 'upper right', numpoints = 1, fancybox = True, framealpha = 0.5)
      
    # plot autocorrelation of spline parameters
    for samples,axes in zip(samplesAllScenes, axesPerScene):
      if not 'spline_parameter_autocorrelation' in axes: continue
      axs = axes['spline_parameter_autocorrelation']
      l = pplot.plotAutocorrSplineParameters(axs, samples)
      for ax in axs:
        if ax.get_legend() is None: ax.legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
        
    # plot autocorrelation of feature values
    for samples,axes in zip(samplesAllScenes, axesPerScene):
      if not 'feature_value_autocorrelation' in axes: continue
      axs = axes['feature_value_autocorrelation']
      l = pplot.plotAutocorrFeatureValues(axs, samples, featureContainer)
      for ax in axs:
        if ax.get_legend() is None: ax.legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
        
    # plot parameter traces
    for samples,axes in zip(samplesAllScenes, axesPerScene):
      if not 'parameter_trace' in axes: continue
      axs = axes['parameter_trace']
      l = pplot.plotParameterTrace(axs, samples)
      for ax in axs:
        if ax.get_legend() is None: ax.legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
        
    # plot feature expectation traces
    for samples,axes in zip(samplesAllScenes, axesPerScene):
      if not 'feature_expectation_trace' in axes: continue
      axs = axes['feature_expectation_trace']
      l = pplot.plotFeatureExpectationTrace(axs, samples, featureContainer, batch_sizes=[50])
      for ax in axs:
        if ax.get_legend() is None: ax.legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
  
    # Plot occupancy probability map
    for samples,axes in zip(samplesAllScenes, axesPerScene):
      if not 'occupancy' in axes: continue
      a = pplot.plotOccupancyProbabilityMap(axes['occupancy'], samples, resolution=0.2, aspect='auto', interpolation='bilinear', alpha=1.0)
      
    # Plot feature histogram
    for samples,axes,featureInfos in zip(samplesAllScenes, axesPerScene, featureInfosAllScenes):
      if not 'feature_histogram' in axes: continue
      ax = axes['feature_histogram']
      ax.set_color_cycle(None)
      for fi in featureInfos:
        a = pplot.plotFeatureValueHistogram(ax, samples, featureContainer.getFeature(fi.name), featureInfo=fi, bins=max(10, int(len(samples)/100)), log_scale=True, color=cmFeatures[fi.name])
      if ax.get_legend() is None: ax.legend(loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
      
    # Plot design variable distributions
    for samples,mode,axes,fig in zip(samplesAllScenes, modeAnalytic, axesPerScene, figuresPerScene):
      if not 'spline_parameter_distribution' in axes: continue
      axs = axes['spline_parameter_distribution']
      for ax in axs: ax.set_color_cycle(None)
      p,me,mo,t=pplot.plotDesignVariableHistograms(axs, samples, bins=50, alpha=0.5, edgecolor=None, plot_mode=plotMode, mode=mode)
  #     for dim,ax in enumerate(axs): 
  #       if ax.get_legend() is None: ax.legend(tuple(p), ('dv{0}'.format(dim),), loc='best', numpoints=1, fancybox = True, framealpha = 0.5)
      fig[1].legend( (tuple(me),tuple(mo)) if plotMode else tuple(me), ('mean','mode') if plotMode else 'mean', loc='upper center', numpoints=1, fancybox = True, framealpha = 0.5, ncol=2)
      fig[1].tight_layout(rect=[0, 0.03, 1, 0.95])
        
    # Update canvases
    pl.show(block=False)
  #   pl.tight_layout()

    # Save figures
    if not argconf.dontSaveFigures:
      try:
        for figs in figuresPerScene:
          if not isinstance(figs, Iterable): figs = [figs]
          for fig in figs:
            for filetype in ['pdf', 'png']:
              fig.savefig(os.path.join(saveToFolder, "iteration_{0:03d}_{1}.{2}".format(cnt,fig.canvas.get_window_title(), filetype)), bbox_inches='tight' )
        sm.logInfo("Saved figures to {0}".format(saveToFolder))
      except Exception,e:
        sm.logError("{0}".format(e))
        shutil.rmtree(saveToFolder)
      
  # Update feature file if we are not in replay mode
  if argconf.replayFolder is None:
    featureContainer.save( os.path.join(saveToFolder, "learned_features.xml") )
      
  # Check stop criterion
  if learner.optimizer.status.success():
    sm.logInfo("Learning procedure converged, stopping")
    sm.logInfo("{0}".format(learner.optimizer.status))
    break
  
  if argconf.waitAfterIteration: raw_input("Press enter to continue...")

