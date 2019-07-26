import numpy as np
from datetime import datetime
from scipy import optimize as scipyopt

import sm
from probabilistic_planner import ContinuousScene, ContinuousSceneModifiable
from probabilistic_planner.util import Plotting as pplot
from aslam_backend import OptimizationProblem
from aslam_backend import OptimizerRprop, OptimizerOptionsRprop, ConvergenceStatus, OptimizerStatusRprop, RpropMethod
from aslam_backend import OptimizerBFGS, OptimizerOptionsBFGS, OptimizerStatusBFGS

class SceneOptimizable(object):
  def __init__(self, scene, featureContainer):
    assert isinstance(scene, ContinuousScene)
    self.scene = ContinuousSceneModifiable(scene)
    self._featureContainer = featureContainer
    self.problem = OptimizationProblem()
    for id, optAgent in self.scene.optAgents.iteritems(): 
      optAgent.trajectory.addDesignVariables(self.problem, False)
    self._featureContainer.addErrorTerms(self.scene, self.problem)
    self.nGradientEvals = self.nObjectiveEvals = 0
    
  def negLogLikelihood(self, x = None):
    self.nObjectiveEvals += 1
    if not x is None: self.parameters = x
    return self.problem.evaluateError()
  
  def grad(self, x = None):
    self.nGradientEvals += 1
    if not x is None: self.parameters = x
    return self.problem.computeGradient()
  
  @property
  def parameters(self):
    return self.scene.activeSplineParameters
  
  @parameters.setter
  def parameters(self, x):
    self.scene.setActiveSplineParameters(x)
    
  def plot_scene_callback(self, x, ax, **kwargs):
    ax.clear()
    pplot.plotScene(self.scene, ax, **kwargs)
    ax.relim()
    ax.autoscale_view(tight=False)
    ax.figure.canvas.draw()
  
  def log_callback(self, x = None):
    sm.logInfo("x = {0}\nobj_fcn = {1}\ngrad_norm = {2}".format(self.parameters if x is None else x, self.negLogLikelihood(x), np.linalg.norm(self.grad(x))))
    
    
def plot_convergence_callback(sceneOpt, line, ordinate_type='gradient_norm', abscissa_type='iteration'):
  if ordinate_type == 'gradient norm':
    y = np.linalg.norm(sceneOpt.grad())
  elif ordinate_type == 'objective fcn':
    y = sceneOpt.negLogLikelihood()
  else: 
    raise RuntimeError("wrong ordinate_type supplied")
  
  if abscissa_type == 'iteration':
    x = len(line.get_xdata()) + 1
  elif abscissa_type == 'number of objective evaluations':
    x = sceneOpt.nObjectiveEvals
  elif abscissa_type == 'number of gradient evaluations':
    x = sceneOpt.nGradientEvals
  else:
    raise RuntimeError("wrong abscissa_type supplied")
    
  line.set_xdata( np.append(line.get_xdata(), x) )
  line.set_ydata( np.append(line.get_ydata(), y) )
  line.axes.relim()
  line.axes.autoscale_view(tight=False)
  line.axes.figure.canvas.draw()

def optimize(scene, options, method, callback=lambda x: None, **kwargs):
  ''' 
  Optimize the scene.
    Example: res = optimize(sceneOpt, method='BFGS', options={'disp': True, 'maxiter':1000})'''
  
  start = datetime.now()
  
  callback(scene.parameters)
  
  try:
    methodlc = method.lower()
    if 'aslam' in methodlc:
      if 'rprop' in methodlc:
        optimizerOptions = OptimizerOptionsRprop()
        if 'rprop+' in methodlc: optimizerOptions.method = RpropMethod.RPROP_PLUS
        elif 'rprop-' in methodlc: optimizerOptions.method = RpropMethod.RPROP_MINUS
        elif 'irprop-' in methodlc: optimizerOptions.method = RpropMethod.IRPROP_MINUS
        elif 'irprop+' in methodlc: optimizerOptions.method = RpropMethod.IRPROP_PLUS
      elif 'bfgs' in methodlc: optimizerOptions = OptimizerOptionsBFGS()
      else: raise RuntimeError("Unsupported method '{0}'".format(method))
        
      if 'maxiter' in options: optimizerOptions.maxIterations = options['maxiter']
      if 'gtol' in options: optimizerOptions.convergenceGradientNorm = options['gtol']
      if 'xtol' in options: optimizerOptions.convergenceDeltaX = options['xtol']
      if 'ftol' in options: optimizerOptions.convergenceDeltaObjective = options['ftol']
      maxIterations = optimizerOptions.maxIterations
      optimizerOptions.maxIterations = 1
        
      if 'rprop' in methodlc: optimizer = OptimizerRprop(optimizerOptions)
      elif 'bfgs' in methodlc:  optimizer = OptimizerBFGS(optimizerOptions)
        
      optimizer.setProblem(scene.problem)
      optimizer.checkProblemSetup()
      optimizer.initialize()
            
      for cnt in range(maxIterations):
        optimizer.optimize()
        ret = optimizer.status
        scene.nObjectiveEvals = ret.numObjectiveEvaluations
        scene.nGradientEvals = ret.numDerivativeEvaluations
        callback(scene.parameters)

        if ret.success() or ret.failure(): break
        
      res = {'status': ret.convergence,
             'success': ret.success(), 
             'nfev': ret.numObjectiveEvaluations,
             'njev': ret.numDerivativeEvaluations, 
             'fun': scene.negLogLikelihood(),
             'jac': scene.grad(),
             'jacNorm': ret.gradientNorm, 
             'x': scene.parameters, 
             'message': ret.convergence, 
             'nit': ret.numIterations}

    else:
      x0 = scene.parameters
      objective_fcn = lambda x: scene.negLogLikelihood(x)
      jacobian_fcn = lambda x: scene.grad(x)
      res = scipyopt.minimize(objective_fcn, x0, jac=jacobian_fcn, options=options, callback=callback, method=method, **kwargs)

  except Exception,e:
    sm.logError("{0}: Exception '{1}' encountered".format(method, e))
    res = {}

  sm.logDebugNamed("optimization", "Optimization with method {0} took {1}".format(method, datetime.now() - start))
  return res
