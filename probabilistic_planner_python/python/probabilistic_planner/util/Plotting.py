import numpy as np
import pylab as pl
from itertools import cycle
from collections import defaultdict, Iterable
import matplotlib.patheffects as path_effects
from matplotlib.patches import Ellipse

from planner_interfaces_python.util import *
from probabilistic_planner import *
from probabilistic_planner import util
from operator import isCallable

def plot_point_cov(points, nstd=2, ax=None, **kwargs):
    """
    Plots an `nstd` sigma ellipse based on the mean and covariance of a point
    "cloud" (points, an Nx2 array).

    Parameters
    ----------
        points : An Nx2 array of the data points.
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    pos = points.mean(axis=0)
    cov = np.cov(points, rowvar=False)
    return plot_cov_ellipse(cov, pos, nstd, ax, **kwargs)

def plot_cov_ellipse(ax, cov, pos, nstd=2.0, **kwargs):
    """
    Plots an `nstd` sigma error ellipse based on the specified covariance
    matrix (`cov`). Additional keyword arguments are passed on to the 
    ellipse patch artist.

    Parameters
    ----------
        cov : The 2x2 covariance matrix to base the ellipse on
        pos : The location of the center of the ellipse. Expects a 2-element
            sequence of [x0, y0].
        nstd : The radius of the ellipse in numbers of standard deviations.
            Defaults to 2 standard deviations.
        ax : The axis that the ellipse will be plotted on. Defaults to the 
            current axis.
        Additional keyword arguments are pass on to the ellipse patch.

    Returns
    -------
        A matplotlib ellipse artist
    """
    def eigsorted(cov):
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        return vals[order], vecs[:,order]

    vals, vecs = eigsorted(cov)
    theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

    # Width and height are "full" widths, not radius
    width, height = 2.0 * nstd * np.sqrt(vals)
    ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

    ax.add_artist(ellip)
    return ellip


def generateColormap(scenes, colormap=None, shuffle=True):
  cm = {}
  if not isinstance(scenes, Iterable): scenes = [scenes]
  ids = set([])
  for scene in scenes: 
    ids |= set(scene.optAgents.keys())
    for obs in scene.getObservations():
      for id, obj in obs.objectContainer.iteritems():
        ids.add(id)
  if colormap is None: colormap = pl.get_cmap('gist_rainbow')
  colors = [colormap(float(i)/(len(ids) -1 if len(ids) > 1 else 1)) for i in range(len(ids))]
  if shuffle: np.random.shuffle(colors)
  for idx,id in enumerate(ids): cm[id] = colors[idx]
  return cm

def generateColors(numColors, colormap=None, shuffle=False):
  if colormap is None: colormap = pl.get_cmap('gist_rainbow')
  if isinstance(colormap, str): colormap = pl.get_cmap(colormap)
  if numColors == 0: return []
  if numColors == 1: return [colormap(0)]
  colors = [colormap(float(i)/(numColors-1)) for i in range(numColors)]
  if shuffle: np.random.shuffle(colors)
  return colors

def generateUniqueLinestyles(numStyles, colormap=None):
  ''' Generates various linestyles for a given number of colors
  '''
  colors = generateColors(numStyles, colormap)
  linestyles = cycle(["-","--","-.",":"])
  markerstyles = cycle(['s', 'd', '+', 'o', '*', 'v', '^', ',', '.'])
  return colors, [linestyles.next() for i in range(numStyles)], [markerstyles.next() for i in range(numStyles)]

def plotScene(scene, ax, time_resolution=Duration(0.1), colormap=None, show_radii=False, show_trajectories=True, show_observations=False, show_gridmaps = False, show_observation_ellipses=False, \
              show_observation_velocities = False, arrow_head_width=0.05, arrow_head_length=0.05, marker_obs='x', alpha_traj=1.0, alpha_obs_ellipses=0.01, alpha_obs_marker=1.0, traj_args={}, \
              traj_label=lambda x: 'agent {0}'.format(x), arrow_args={}, obs_marker_args={}, cov_args={}, grid_args={}, grid_rotate_args = {}, colormap_args = {}, radii_args=lambda x: {}):
  if colormap is None: colormap = generateColormap(scene, **colormap_args)
  traj_artists = []
  obs_ellipses_artists = []
  obs_marker_artists = []
  im = None
  
  get_color = lambda agentId: colormap(agentId) if hasattr(colormap, '__call__') else colormap[agentId]
  
  if show_trajectories:
    for id,optAgent in scene.optAgents.iteritems():
      color = get_color(id)
      radius = optAgent.agent.getDiscApproximation(1).getRadius(0)
      pos,_ = util.getTrajectory(scene, id, resolution=time_resolution)
      l, = ax.plot( pos[0,:], pos[1,:], color=color, label='{0}'.format(traj_label(optAgent.id)), alpha=alpha_traj, **traj_args)
      traj_artists.append(l)
      if pos.shape[1] > 0:
        delta = pos[:,-1] - pos[:,-2]
        a = ax.arrow(pos[0,-1], pos[1,-1], delta[0], delta[1], color=color, head_width=arrow_head_width, head_length=arrow_head_length, alpha=alpha_traj, **arrow_args)
        traj_artists.append(a)
      if show_radii:
        for p in pos.T:
          agent_circles = pl.Circle(tuple(p), radius, **radii_args(id))
          ax.add_artist(agent_circles)
  if show_observations:
    perAgentCounter = dict.fromkeys(scene.optAgents.keys(), 0)
    for snapshots in scene.getObservations():
      for id,snapshot in snapshots.objectContainer.iteritems():
        color = get_color(id)
        if not id in perAgentCounter: perAgentCounter[id] = 0
        o, = ax.plot(snapshot.pose.position.x, snapshot.pose.position.y, linestyle='none', marker=marker_obs, color=color, \
                     label='observation agent {0}'.format(id) if perAgentCounter[id] == 0 else '_nolegend_', alpha=alpha_obs_marker, **obs_marker_args )
        perAgentCounter[id] += 1
        if show_observation_ellipses:
          e = plot_cov_ellipse(ax, np.linalg.inv(snapshot.invCov[0:2, 0:2]), [snapshot.pose.position.x, snapshot.pose.position.y], facecolor=colormap[id], edgecolor='none', alpha=alpha_obs_ellipses, **cov_args)
          obs_marker_artists.append(o)
          obs_ellipses_artists.append(e)
        if show_observation_velocities:
          ax.arrow(snapshot.pose.x, snapshot.pose.y, snapshot.state[0], snapshot.state[1], color=color, alpha=0.5)
      if show_gridmaps and snapshots.occupancyGrid is not None:
        map = snapshots.occupancyGrid
        im = plotGrid(map, ax, mask_value = OccupancyValue.FREE, rotate_args = grid_rotate_args, **grid_args)
        
  return traj_artists,obs_marker_artists,obs_ellipses_artists,im


def plotSamples(samples, ax, featureContainer, colormap=None, subsample=1, plot_mode=True, plot_mean=True, mode=None, **kwargs):
  '''
  Plots all samples
  
  Parameters
  ----------
  samples : iterable of ContinuousScenes
  ax: matplotlib axis object
  featureContainer: feature container
      Used to compute the mode of the distribution. Required if plot_mode=True and mode=None
  colormap: dictionary
      Dictionary from agent id to either color or callable in the format of f(sample, sampleIndex)
  
  Returns
  -------
  err_code : int
    Non-zero value indicates error code, or zero on success.
 '''
  
  artists = []
  
  get_colormap = lambda sample, identifier: dict(zip(colormap.keys(), [f(sample, identifier) if hasattr(f, '__call__') else f for f in colormap.values()])) if not colormap is None else colormap
  
  if len(samples) == 0: return artists
  
  for cnt,sample in enumerate(samples):
    if cnt % subsample != 0: continue
    t,_,_,_=plotScene(sample, ax, colormap=get_colormap(sample,cnt), traj_label=lambda x: '_no_legend_', **kwargs)
    artists.extend(t)

  # Plot spline parameter mode and mean
  parameters = util.getSplineParametersFromSamples(samples)

  # Compute mean of samples
  if plot_mean:
    mean = util.computeMean(samples)
    
  # Compute analytic mode
  if plot_mode and mode is None:
    mode = util.computeMode(samples[0], featureContainer)
  
  if 'traj_args' in kwargs.keys():
    kwargs['traj_args']['linestyle'] = '-'
    kwargs['traj_args']['linewidth'] = 3
  kwargs.pop('show_observations', None)
  kwargs.pop('alpha_traj', None)

  # Plot mean
  if plot_mean:
    t,_,_,_=plotScene(mean, ax, colormap=get_colormap(mean, 'mean'), show_observations=False, alpha_traj=1.0, traj_label=lambda x: 'mean agent {0}'.format(x), **kwargs)
    artists.extend(t)
    for line in t: line.set_path_effects([path_effects.Stroke(linewidth=5, foreground='black'), path_effects.Normal()])

  # Plot analytic mode
  if plot_mode:
    t,_,_,_=plotScene(mode, ax, colormap=get_colormap(mode, 'mode'), show_observations=False, alpha_traj=1.0, traj_label=lambda x: 'mode agent {0}'.format(x), **kwargs)
    artists.extend(t)
    for line in t: line.set_path_effects([path_effects.Stroke(linewidth=5, foreground='white'), path_effects.Normal()])
  
  return artists


def plotFeatureExpectationVsDemonstration(ax, featureInfo, featureContainer, mode=None, samples=None, **kwargs):
  
  # TODO: make it work for multidimensional features
  
  bar_artists = []
  w = 0.3
  alpha = 0.7
  fdem = [fi.demonstration[0] for fi in featureInfo if fi.numActiveDesignVariables > 0]
  fexp = [fi.expectation[0] for fi in featureInfo if fi.numActiveDesignVariables > 0]
  fnames = [fi.name for fi in featureInfo if fi.numActiveDesignVariables > 0]
  x = np.arange(len(fexp))
  rects0 = ax.bar(x, fdem, width=w, color='b', alpha=alpha, label='demonstrations', align='center', **kwargs)
  rects1 = ax.bar(x+w, fexp, width=w, color='g', alpha=alpha, label='expectations', align='center', **kwargs)
  ax.set_xticks(x)
  ax.set_xticklabels(tuple(fnames))
  for label in ax.get_xticklabels(): label.set_rotation(15)
  bar_artists.extend( [rects0, rects1] )
  
  if not mode is None:
    fmode = [ fi.feature.evaluate(mode)[0] for fi in featureInfo if fi.feature.numActiveWeights > 0 ]
    rects = ax.bar(x+w/2., fmode, width=w/2., color='black', alpha=alpha, label='mode', align='center', **kwargs)
    bar_artists.append(rects)
     
  if not samples is None:
    mean = util.computeMean(samples)
    fmean = [ fi.feature.evaluate(mean)[0] for fi in featureInfo if fi.feature.numActiveWeights > 0 ]
    rects = ax.bar(x+3./2*w, fmean, width=w/2., color='gray', alpha=alpha, label='mean', align='center', **kwargs)
    bar_artists.append(rects)
  
  ax.autoscale(tight=True)
  return bar_artists


def plotFeatureValueHistogram(ax, samples, feature, log_scale=False, featureInfo=None, markersize=6, **kwargs):
  artists = []
  fvals = util.computeFeatureValues(samples, feature)
  means = np.mean(fvals, axis=0)
  for i,col in enumerate(fvals.T):
    y,x,h = ax.hist(col, normed=True, histtype='step', label=feature.name if feature.numWeights() == 1 else feature.name + '_dim' + i, **kwargs)
    color = h[0].get_edgecolor()
    meanx = np.searchsorted(x, means[i])
    idxmode = np.argmax(y) # maximum probability = mode
    l0, = ax.plot(means[i], y[meanx-1 if meanx > 0 else meanx], 'o', color=color, markersize=markersize)
    v0 = ax.axvline(means[i], linestyle='--', color=color)
    l1, = ax.plot(x[idxmode], y[idxmode], '*', color=color, markersize=markersize)
    artists.extend( [l0, l1, v0] )
    artists.extend(h)
    if not featureInfo is None and featureInfo.numActiveDesignVariables > 0:
      ld = ax.axvline(featureInfo.demonstration[i], linestyle=':', color=color)
      artists.append(ld)
  if log_scale: ax.set_yscale('log')
  return artists


def plotDesignVariableHistograms(axes, samples, markersize=6, plot_mode=True, mode=None, **kwargs):
  patch_artists = []
  mean_artists = []
  mode_artists = []
  text_artists = []
  if len(samples) == 0: return
  dims = samples[0].numActiveSplineParameters
  assert dims == len(axes), "Need {0} axis objects, got {1}".format(dims, len(axes))
  
  parameters = util.getSplineParametersFromSamples(samples)

  means = np.mean(parameters, axis=0)
  for dim,ax in enumerate(axes):
    y,x,patches = ax.hist(parameters[:, dim], normed=True, histtype='bar', label = 'dv{0}'.format(dim), **kwargs)
    color = patches[0].get_facecolor()
    v0 = ax.axvline(means[dim], linestyle='-', color='black', label='mean')
    t0 = ax.text(0.1, 0.9, 'dv{0}'.format(dim), ha="left", va="top", transform=ax.transAxes,
         bbox = dict(boxstyle="round",
                     ec='black',
                     fc=color,
                     alpha=0.2
                     )
             )
    patch_artists.extend(patches)
    mean_artists.append(v0)
    text_artists.append(t0)
    
  # Compute analytic mode
  if plot_mode and mode is None:
    mode,_ = util.computeMode(samples[0], featureContainer)
    
  if plot_mode:
    parametersMode = mode.activeSplineParameters
    for dim,ax in enumerate(axes):
       v0 = ax.axvline(parametersMode[dim], linestyle='-', color='red', label='mode')
       mode_artists.append(v0)
       
  return patch_artists,mean_artists,mode_artists,text_artists


def squareTileAxesLayout(numAxes, **kwargs):
  cols = int(np.ceil(np.sqrt(numAxes)))
  rows = int(np.floor(np.sqrt(numAxes)))
  if rows*cols < numAxes: rows += 1
  fig,axes = pl.subplots(nrows=rows, ncols=cols, **kwargs)
  unused = rows*cols-numAxes
  
  # Remove excess axes again
  cnt = 0
  axssub = []
  for r in range(rows):
    axsrow = []
    for c in range(cols):
      if cnt >= numAxes:
        fig.delaxes(axes[r,c])
      else:
        axsrow.append(axes[r,c])
      cnt += 1
    axssub.append(axsrow)
        
  return fig,axssub


def computeBoundingBox(samples):
  xmin = ymin = 1e40
  xmax = ymax = -1e40
  for scene in samples:
    for id,optAgent in scene.optAgents.iteritems():
      trajectory = optAgent.trajectory
      if not trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
      pos = np.asarray([trajectory.getPosition(Time(stamp)) for stamp in np.arange(trajectory.startTime.toSec(), trajectory.finalTime.toSec()+1e-12, 0.1)]).T
      xmin = min(xmin, np.min(pos[0,:]))
      xmax = max(xmax, np.max(pos[0,:]))
      ymin = min(ymin, np.min(pos[1,:]))
      ymax = max(ymax, np.max(pos[1,:]))
  return (xmin,xmax), (ymin,ymax)
    
def computeOccupancyProbabilityMap(samples, origin, resolution, sizeInCells, timestamps, agents=None):
  ''' 
  Computes a sampled version of the position probability at given timestamps
  
  Parameters
    ----------
      samples: Composite trajectory samples from the density
      map: Base for the discretized output densities
      timestamps: vector of timestamps to evaluate the density for
      agents: list of agents to consider
  '''
  
  maps = []

  for cnt,stamp in enumerate(timestamps):
    _maps = defaultdict(MapDouble)
    for scene in samples:
      for agentId,optAgent in scene.optAgents.iteritems():
        if not agents is None and not agentId in agents: continue
        if not agentId in _maps.keys(): _maps[agentId] = MapDouble(origin, resolution, sizeInCells, 0.0)
        if not optAgent.trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
        pos = optAgent.trajectory.getPosition2d(stamp)
        idx = _maps[agentId].toIndex(pos)
        if _maps[agentId].isInsideMapIndex(idx): _maps[agentId][idx] = _maps[agentId][idx]+1.0
    for map in _maps.values(): sum = np.sum(map.matrix); map.matrix = map.matrix / sum if sum > 0 else map.matrix
    maps.append(_maps)
          
  return maps

def plotOccupancyProbabilityMap(ax, samples, agents=None, xlim=None, ylim=None, resolution=0.1, mask_zero=False, **kwargs):
  
  # Compute area to plot
  if xlim is None or ylim is None:
    xlim,ylim = computeBoundingBox(samples)
    
  xrange = xlim[1] - xlim[0]
  yrange = ylim[1] - ylim[0]
  map = MapDouble(Pose2d(xlim[0], ylim[0], 0.0), resolution)
  map.matrix = np.zeros(shape=(np.ceil(yrange/resolution), np.ceil(xrange/resolution)))
  
  for scene in samples:
    for agentId,optAgent in scene.optAgents.iteritems():
      if not agents is None and not agentId in agents: continue
      trajectory = optAgent.trajectory
      if not trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
      pos = np.asarray([trajectory.getPosition(Time(stamp)) for stamp in np.arange(trajectory.startTime.toSec(), trajectory.finalTime.toSec()+1e-12, 0.1)]).T
      for p in pos.T:
        idx = map.toIndex(Position2d(p))
        map.setAtIndex(idx, map.getAtIndex(idx)+1.0)
        
  extent = [map.origin.x, map.origin.x+map.resolution*map.sizeInCells.x, map.origin.y, map.origin.y+map.resolution*map.sizeInCells.y]
  mat = np.ma.masked_where(map.matrix == 0, map.matrix) if mask_zero else map.matrix
  return ax.imshow(mat, extent=extent, origin='lower', **kwargs)

def _plotPerSplineParameter(ax, vals, idx, color, add_text=True, add_title=False, **kwargs):
  l, = ax.plot(vals, color=color, **kwargs)
  t = 'x' if idx % 2 == 0 else 'y'
  t = '{0}{1}'.format(t, idx/2)
  p = None
  if add_text:
    p = ax.text(0.1, 0.9, t, ha="left", va="top", transform=ax.transAxes, \
                 bbox = dict(boxstyle="round", ec='black', fc=l.get_color(), alpha=0.2))
  if add_title:
    ax.set_title(t)
  return l,p
    
def plotAutocorrSplineParameters(axes, samples, add_text=True, add_title=False, autocorr_args = {}, **kwargs):
  acorr = util.autoCorrSplineParameters(samples, **autocorr_args)
  if len(axes) != acorr.shape[1]:
   raise RuntimeError("You need to supply {0} axes".format(acorr.shape[1]))
  
  ll = []
  colorg = kwargs.pop('color', None)
  colorl = kwargs.pop('xcolor', colorg), kwargs.pop('ycolor', colorg)
  for dim,(iacorr,ax) in enumerate(zip(acorr.T, axes)):
    l,t = _plotPerSplineParameter(ax, iacorr, dim, colorl[dim % 2], add_text=add_text, add_title=add_title, **kwargs)
    ll.extend( (l,t) )
  return ll

def plotAutocorrFeatureValues(axes, samples, featureContainer, autocorr_args = {}, plot_args={}):
  acorr = util.autoCorrFeatureValues(samples, featureContainer, **autocorr_args)
  if len(axes) != len(acorr):
   raise RuntimeError("You need to supply {0} axes".format(len(acorr)))
  
  ll = []
  for (fname,facorr),ax in zip(acorr.iteritems(), axes):
    l, = ax.plot(facorr, label=fname, **plot_args)
    ll.append(l)
    
  return ll

def plotParameterTrace(axes, samples, add_text=True, add_title=False, **kwargs):
  parameters = util.getSplineParametersFromSamples(samples)
  if len(axes) != parameters.shape[1]:
   raise RuntimeError("You need to supply {0} axes".format(parameters.shape[1]))
  
  line_artists = []
  patch_artists = []
  colorg = kwargs.pop('color', None)
  colorl = kwargs.pop('xcolor', colorg), kwargs.pop('ycolor', colorg)
  for dim,(p,ax) in enumerate(zip(parameters.T, axes)):
    l,t = _plotPerSplineParameter(ax, p, dim, colorl[dim % 2], add_text=add_text, add_title=add_title, **kwargs)
    if not t is None: patch_artists.append(t)
  return line_artists,patch_artists

def plotFeatureExpectationTrace(axes, samples, featureContainer, batch_sizes=[50], **kwargs):
  
  class Item(object):
    def __init__(self):
      self.mean_full = []
      self.mean_batch = {}
      self.feature_values = []

  fvals = defaultdict(Item)
  for feature in featureContainer.getContainer():
    item = Item()
    item.feature_values = util.computeFeatureValues(samples, feature)
    item.mean_full = np.cumsum(item.feature_values)/np.arange(1,item.feature_values.size+1) # Running mean
    for sz in batch_sizes:
      item.mean_batch[sz] = np.array( [np.mean(item.feature_values[max(cnt-sz+1,0):cnt+1]) for cnt in range(len(item.feature_values))] )
    fvals[feature.name] = item
    
  if len(axes) != len(fvals):
    raise RuntimeError("You need to supply {0} axes".format(len(fvals)))

  line_artists = []
  for (fname,fval),ax in zip(fvals.iteritems(), axes):
    ax.set_title(fname)
    l, = ax.plot(fval.feature_values, label="values", **kwargs)
    line_artists.append(l)
    l, = ax.plot(fval.mean_full, label="mean_full", **kwargs)
    line_artists.append(l)
    for sz in batch_sizes:
      l, = ax.plot(fval.mean_batch[sz], label="mean_batch_{0}".format(sz), **kwargs)
      line_artists.append(l)
    
  return line_artists

def plotSceneVelocities(scene, ax, time_resolution=Duration(0.1), colormap=None, colormap_args={}):
  if colormap is None: colormap = generateColormap(scene, **colormap_args)
  artists = []
  for agentId,optAgent in scene.optAgents.iteritems():
      trajectory = optAgent.trajectory
      if not trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
      stamps = [Time(d) for d in np.arange(trajectory.startTime.nanosec, trajectory.finalTime.nanosec, time_resolution.nanosec)]
      l, = ax.plot( [stamp.toSec() for stamp in stamps], [np.linalg.norm(trajectory.getVelocityXY(stamp)) for stamp in stamps], c = colormap[agentId], label='agent {0}'.format(optAgent.id));
      artists.append(l)
  return artists;

def plotSceneDistances(scene, ax, time_resolution=Duration(0.1), colormap=None, colormap_args={}, dist_shape_centers=True):
  if colormap is None: colormap = generateColormap(scene, **colormap_args)
  ego = scene.getOptAgent(ID_EGO);
  artists = []
  for agentId, optAgent in scene.optAgents.iteritems():
    if agentId == ID_EGO: continue
    if not ego.trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
    if not optAgent.trajectory.isInitialized: raise RuntimeError("Trajectory for agent %d is not initialized" % agentId)
    tmin = min(ego.trajectory.startTime, optAgent.trajectory.startTime)
    tmax = min(ego.trajectory.finalTime, optAgent.trajectory.finalTime)
    stamps = [Time(d) for d in np.arange(tmin.nanosec, tmax.nanosec, time_resolution.nanosec)]
    d = np.array([np.linalg.norm(ego.trajectory.getPosition(t) - optAgent.trajectory.getPosition(t)) for t in stamps])
    if not dist_shape_centers:
      d -= optAgent.agent.getDiscApproximation(1).getRadius(0) + ego.agent.getDiscApproximation(1).getRadius(0)
    l, = ax.plot([t.toSec() for t in stamps], d, c = colormap[agentId], label='agent {0}'.format(agentId))
    artists.append(l)
  return artists;

def createVideo(inFolder, regexp, outFile, fps=1.0):
  ''' Creates a video from a sequence of png files
  '''
  import os
  inFolder = inFolder.strip("/")
  command = 'mencoder "mf://{0}/{1}" -mf fps={2}:type=png -ovc copy -ofps 5 -o {3}.avi'.format(inFolder, regexp, fps, outFile)
  print "Executing: {0}".format(command)
  rval = os.system(command)
  if rval != 0: raise RuntimeError("Video creating failed")