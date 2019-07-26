#!/usr/bin/env python
from planner_interfaces_python import *
from planner_algorithms import *
import numpy as np

import unittest

class TestPlannerAlgorithms(unittest.TestCase):
      
    def test_dijkstra(self):
      grid = OccupancyGrid(Pose2d(0.,0.,0.), 0.1, MapSize(200,200), OccupancyValue.FREE)
      costs = MapDouble(Pose2d(0.0, 0.0, 0.0), 0.1, np.inf*np.ones(shape=(200,200), dtype=np.double))
      connectivity = Connectivity4d(grid)
      start = MapIndex(0, 0)
      dijkstra = DijkstraC4D(MapIndex(0, 0), connectivity, costs)
      dijkstra.run()
      self.assertEqual(costs[start], 0.0)
      self.assertEqual(costs[start + MapIndex(1,0)], grid.resolution)
      
    def test_mapChangeDetection(self):
      map = MapUint8(Pose2d(0.0, 0.0, 0.0), 0.1, MapSize(200,200), 0)
      mapChangeDetection(map)
      map = MapFloat(Pose2d(0.0, 0.0, 0.0), 0.1, MapSize(200,200), 0.0)
      mapChangeDetection(map)
      map = MapDouble(Pose2d(0.0, 0.0, 0.0), 0.1, MapSize(200,200), 0.0)
      mapChangeDetection(map)
      
    def test_voronoiPoints(self):
      points = [ PointDouble(0,0), PointDouble(1,0), PointDouble(1,1)]
      facets = voronoi(points)
      self.assertEqual(len(facets), 3)
      for points in facets:
        self.assertEqual(len(points), 4)
      
    def test_distanceTransform(self):
      map = MapBool(Pose2d(-10., -10., 0.), 0.1, MapSize(100,100), True)
      map[0,0] = False
      dt = distanceTransform(map)
      dt = distanceTransformSigned(map)
      dt,labels = distanceTransformLabeled(map)
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun('planner_algorithms_python', 'PlannerAlgorithms', TestPlannerAlgorithms)
