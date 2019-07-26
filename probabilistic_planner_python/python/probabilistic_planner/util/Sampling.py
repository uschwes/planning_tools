import numpy as np
from probabilistic_planner import ContinuousScene

class ContinuousSceneWrapper(object):
  """ 
  Adds parameters access to the ContinuousScene object, needed purely for 
  interfacing 3rd party sampling routines from python
  """
  
  def __init__(self, scene):
    assert isinstance(scene, ContinuousScene), "Invalid type {0}".format(type(scene))
    self._scene = scene
    
  @property
  def base(self):
    return self._scene
  @base.setter
  def base(self, scene):
    self._scene = scene
    
  @property
  def parameters(self):
    x = np.empty(shape=(0,))
    for id,optAgent in self._scene.optAgents.iteritems():
      dvs = optAgent.trajectory.getDesignVariables()
      for i in range(len(dvs)):
        dv = dvs[i]
        p = dv.getParameters()
        x = np.concatenate( [x, p.reshape((p.size,))] )
    return x
  
  @parameters.setter
  def parameters(self, x):
    cnt = 0
    for id,optAgent in self._scene.optAgents.iteritems():
      dvs = optAgent.trajectory.getDesignVariables()
      for i in range(len(dvs)):
        dv = dvs[i]
        dim = dv.getParameters().size
        p = dv.setParameters(x[cnt:cnt+dim])
        cnt += dim

def log_likelihood(x, features, scene):
  """
  Wrapper to be used in combination with emcee Sampler
  """
  
  # Update the scene with the parameter vector
  if not isinstance(scene, ContinuousSceneWrapper):
    scene = ContinuousSceneWrapper(scene)

  p = scene.parameters
  
  scene.parameters = x.squeeze()
  
  ll = 0.0
  for f in features.getContainer():
    ll += -f.getCurrentWeightsVector().dot(f.evaluate(scene.base))
  
  scene.parameters = p # Restore parameters
  return ll