import numpy as np
from scipy import ndimage
from planner_interfaces_python import *

def plotGrid(grid, ax, image=None, mask_value = None, cmap='Greys_r', interpolation='nearest', rotate_args={}, plot_border=False, plot_origin=False, plot_contour=False, contour_args={}, \
             plot_gradient=False, gradient_args={}, **kwargs):
  
  data = ndimage.rotate(grid.matrix, -grid.origin.yaw*180./np.pi, **rotate_args) if grid.origin.yaw != 0.0 else grid.matrix
  image = ndimage.rotate(image, -grid.origin.yaw*180./np.pi, **rotate_args) if grid.origin.yaw != 0.0 and image is not None else image
  
    # Transform the four corner points
  corners = [grid.toPosition(MapIndex(0,0)), grid.toPosition(MapIndex(int(grid.sizeInCells.x),0)), \
             grid.toPosition(MapIndex(int(grid.sizeInCells.x),int(grid.sizeInCells.y))), grid.toPosition(MapIndex(0,int(grid.sizeInCells.y)))]
  if plot_border: 
    for cnt,(c0,c1) in enumerate(zip(corners, corners[1:]+corners[0:1])):
      ax.plot( [c0.x, c1.x], [c0.y, c1.y], linestyle='-', color='black', label='grid border' if cnt == 0 else '_nolegend_')
  if plot_origin: ax.plot(corners[0].x, corners[0].y, linestyle='None', marker='d', color='black', label='grid origin')
  corners = np.asarray( [c.asVector() for c in corners] ).T
  extent = [np.min(corners[0,:]), np.max(corners[0,:]), np.min(corners[1,:]), np.max(corners[1,:])]
  mat = np.ma.masked_where(data == mask_value, data) if not mask_value is None else data
  cim = ax.imshow(mat if image is None else image, extent=extent, origin='lower', cmap=cmap, interpolation=interpolation, **kwargs)
  
  ccont = None
  if plot_contour:
    ccont = ax.contour(data, extent=extent, origin='lower', cmap=cmap if not 'colors' in contour_args.keys() else None, **contour_args)
  
  cgrad = None
  if plot_gradient:
    gradY, gradX = np.gradient(data)
    if 'inverted' in gradient_args.keys() and gradient_args['inverted']:
      gradX = -gradX
      gradY = -gradY
      gradient_args.pop('inverted')
    x = np.linspace(extent[0], extent[1], data.shape[1])
    y = np.linspace(extent[2], extent[3], data.shape[0])
    cgrad = ax.streamplot(x, y, gradX, gradY, **gradient_args)
  
  return cim,ccont,cgrad

def plotGridGradient(gradX, gradY, ax, color_by_gradient_norm=True, density=[1.0, 1.0], rotate_args={}, **kwargs):
 
  if gradX.sizeInCells.x != gradY.sizeInCells.x - 1 or gradX.sizeInCells.y == gradY.sizeInCells.y - 1:
    raise RuntimeError("Invalid shapes of gradient matrixes. gradX: {0}, gradY: {1}".format(gradX.sizeInCells, gradY.sizeInCells))
 
  if gradX.origin.yaw != 0.0:
    dataX = ndimage.rotate(gradX.matrix, gradX.origin.yaw*180./np.pi, **rotate_args)
  else:
    dataX = gradX.matrix
  if gradY.origin.yaw != 0.0:
    dataY = ndimage.rotate(gradY.matrix, gradY.origin.yaw*180./np.pi, **rotate_args)
  else:
    dataY = gradY.matrix

  x = np.linspace(gradX.origin.x, gradX.origin.x+gradX.resolution*(gradX.sizeInCells.x), gradX.sizeInCells.x)
  y = np.linspace(gradY.origin.y, gradY.origin.y+gradX.resolution*(gradY.sizeInCells.y), gradY.sizeInCells.y)
  strm = ax.streamplot(x, y, dataX[:-1,:], dataY[:,:-1], color = np.sqrt(dataX[:-1,:]**2 + dataY[:,:-1]**2) if color_by_gradient_norm else None, density=density, **kwargs)
  if color_by_gradient_norm: cb = ax.figure.colorbar(strm.lines)
  else: cb = None
  return strm,cb

  
class GridEraser(object):
  def __init__(self, ax, grid, callback=lambda g: None):
    self._ax = ax
    self._grid = grid
    self._callback = callback
    self._rect = AlignedBox2dDouble()
    self._delete = True
    self._ax.figure.canvas.mpl_connect('button_press_event', self.on_press)
    self._ax.figure.canvas.mpl_connect('button_release_event', self.on_release)
    self._ax.figure.canvas.mpl_connect('key_press_event', self.on_key_press)
  def __enter__(self): return self
  def __exit__(self, exc_type, exc_value, traceback):
    self._ax.figure.canvas.mpl_disconnect('button_press_event', self.on_press)
    self._ax.figure.canvas.mpl_disconnect('button_release_event', self.on_release)
  def on_key_press(self, event):
    if event.key == "t":
      self._delete = not self._delete
      print "Deletion: {0}".format("ON" if self._delete else "OFF")
  def on_press(self, event):
    p = np.array([event.xdata, event.ydata])
    self._rect = AlignedBox2dDouble()
    if not p.tolist().count(None) > 0:
      self._rect.extend(p)
  def on_release(self, event):
    p = np.array([event.xdata, event.ydata])
    if not p.tolist().count(None) > 0:
      self._rect.extend(p)
      for px in np.linspace(self._rect.min[0], self._rect.max[0], np.floor(2*self._rect.sizes[0]/self._grid.resolution)+1):
        for py in np.linspace(self._rect.min[1], self._rect.max[1], np.floor(2*self._rect.sizes[1]/self._grid.resolution)+1):
          pos = Position2d(px, py)
          if self._grid.isInsideMapPosition(pos):
            self._grid.setAtPosition(pos, OccupancyValue.FREE if self._delete else OccupancyValue.OCCUPIED)
      self._callback(self._grid)
  @staticmethod
  def default_callback(grid, ax):
    ax.cla()
    plotGrid(grid, ax, mask_value = OccupancyValue.FREE, rotate_args={'order':0,'cval':OccupancyValue.FREE})
    ax.figure.canvas.draw()
