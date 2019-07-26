#!/usr/bin/env python
from planner_interfaces_python import *
import numpy as np
import pickle
import os

import unittest

class TestPlannerInterfaces(unittest.TestCase):
  
    def _pickle_test(self, msg, comparator = None):
      if comparator is None:
        comparator = lambda self,x,y: self.assertEqual(x, y, "")
      pfile = "test.pickle"
      pickle.dump( msg, open( pfile, "wb" ) )
      msgPickled = pickle.load( open( pfile, "rb" ) )
      os.remove(pfile)
      comparator(self, msg, msgPickled)
      
    def test_time(self):
      
        tim = Time()
        self.assertFalse(tim.isValid(), "")
        self.assertFalse(tim.isZero(), "")
        
        tim = Time(0.0)
        self.assertTrue(tim.isValid(), "")
        self.assertTrue(tim.isZero(), "")
        self.assertEqual(tim.nanosec, 0, "")
        self.assertEqual(tim.toSec(), 0.0, "")
        self.assertEqual(tim.toDateString(), "1970-Jan-01 00:00:00", "")
        self.assertTrue(tim == Time(0.0), "")
        self.assertFalse(tim == Time(0.1), "")
        self.assertTrue(tim < Time(0.1), "")
        self.assertFalse(tim > Time(0.1), "")
        
        tim = Time(1000000000) # 1e-9 nanoseconds = 1.0 seconds
        self.assertTrue(tim.isValid(), "")
        self.assertFalse(tim.isZero(), "")
        self.assertEqual(tim.nanosec, 1000000000, "")
        self.assertEqual(tim.toSec(), 1.0, "")
        self.assertEqual(tim.toDateString(), "1970-Jan-01 00:00:01", "")
        
        tim = Time(0.0) + Duration(1.0)
        self.assertEqual(tim.nanosec, 1000000000, "")
        self.assertEqual(tim.toSec(), 1.0, "")
        tim-=Duration(0.6)
        self.assertEqual(tim.toSec(), 0.4, "")

        self._pickle_test(tim)

    def test_duration(self):
      
        dur = Duration()

        self.assertEqual(dur.nanosec, 0, "")
        self.assertEqual(dur.toSec(), 0.0, "")
        self.assertTrue(dur == Duration(0.0), "")
        self.assertFalse(dur == Duration(0.1), "")
        self.assertTrue(dur < Duration(0.1), "")
        self.assertFalse(dur > Duration(0.1), "")
        
        dur = Duration(0.0) + Duration(1.0)
        self.assertEqual(dur.nanosec, 1000000000, "")
        self.assertEqual(dur.toSec(), 1.0, "")
        dur-=Duration(0.6)
        self.assertEqual(dur.toSec(), 0.4, "")
        
        dur = Duration(1.0)
        dur /= 0.1
        self.assertEqual(dur.toSec(), 10.0, "")
        dur *= 0.1
        self.assertEqual(dur.toSec(), 1.0, "")
        dur = Duration(1.0)/0.1
        self.assertEqual(dur.toSec(), 10.0, "")
        dur = Duration(10.0)*0.1
        self.assertEqual(dur.toSec(), 1.0, "")

        self._pickle_test(dur)
        
    def test_position2d(self):
      pos = Position2d(0.0, 0.1)
      self.assertEqual(pos.x, 0.0)
      self.assertEqual(pos.y, 0.1)
      self.assertEqual(pos.norm(), 0.1)
      self._pickle_test(pos)
      
      pos = Position2d(np.array([0.0, 0.1]))
      self.assertEqual(pos.x, 0.0)
      self.assertEqual(pos.y, 0.1)
      
      posPlus = pos + Position2d(0.1, 0.2)
      self.assertAlmostEqual(posPlus.x, 0.1, delta=1e-12)
      self.assertAlmostEqual(posPlus.y, 0.3, delta=1e-12)
      posPlus += Position2d(0.1, 0.2)
      self.assertAlmostEqual(posPlus.x, 0.2, delta=1e-12)
      self.assertAlmostEqual(posPlus.y, 0.5, delta=1e-12)
      posPlus = posPlus - Position2d(0.1, 0.2)
      self.assertAlmostEqual(posPlus.x, 0.1, delta=1e-12)
      self.assertAlmostEqual(posPlus.y, 0.3, delta=1e-12)
      posPlus -= Position2d(0.1, 0.2)
      self.assertAlmostEqual(posPlus.x, 0.0, delta=1e-12)
      self.assertAlmostEqual(posPlus.y, 0.1, delta=1e-12)
      
      pos = Position2dStamped(0.0, 0.1, Time(1.0))
      self.assertEqual(pos.x, 0.0, "")
      self.assertEqual(pos.y, 0.1, "")
      self.assertEqual(pos.stamp, Time(1.0), "")
      self._pickle_test(pos)
      
    def test_pose2d(self):
      pose = Pose2d(0.0, 0.1, 0.2)
      self.assertEqual(pose.position, Position2d(0.0, 0.1), "pose: {0}".format(pose))
      pose = Pose2d(1.0, 2.0, np.pi)
      self._pickle_test(pose)
      
      pose = pose + Pose2d(0.0, 0.1, 0.2)
      pose = pose - Pose2d(0.0, 0.1, 0.2)
      
      pose = Pose2dStamped(1.0, 2.0, np.pi, Time(0.1))
      self.assertEqual(pose.stamp, Time(0.1), "pose: {0}".format(pose))
      self._pickle_test(pose)

    def test_state(self):
      state = State(2)
      state = State(np.array([3.0, 4.0]), Pose2d(0.0, 1.0, 2.0))
      stateStamped = StateStamped(state, Time(0.1))
      self.assertTrue( (stateStamped.state == np.array([3.0, 4.0])).all() )
      self._pickle_test(state)
      self._pickle_test(stateStamped)
      
    def test_map(self):
      origin = Pose2d(1.0, 2.0, 0.0)
      map = MapDouble(origin, 0.1)
      self.assertEqual(map.origin, origin)
      self.assertEqual(map.resolution, 0.1)
      self.assertEqual(map.sizeInCells, MapSize(0, 0))
      self.assertFalse(map.isInsideMapIxIy(0,0))
      self.assertFalse(map.isInsideMapIndex(MapIndex(0,0)))
      self.assertFalse(map.isInsideMapPosition(Position2d(0.0,0.0)))
      
      arr = np.linspace(0,200*200,200*200).reshape(200,200).T
      map.matrix = np.linspace(0,200*200,200*200).reshape(200,200).T
      self.assertEqual(map.sizeInCells, MapSize(200, 200))
      self.assertEqual(map.sizeInCellsX, 200)
      self.assertEqual(map.sizeInCellsY, 200)
      self.assertEqual(map.sizeInMeters, PointDouble(20.0, 20.0))
      self.assertEqual(map.sizeInMetersX, 20.0)
      self.assertEqual(map.sizeInMetersY, 20.0)
      self.assertTrue(map.isInsideMapIxIy(0,0))
      for i in range(200):
        for j in range(200):
          self.assertEqual(map.getAtIxIy(i, j), arr[j,i])
      self.assertEqual(map.getAtPosition(Position2d(1.1, 2.1)), arr[1,1])
      
      map.resolution = 0.2
      self.assertEqual(map.resolution, 0.2)
      self.assertEqual(map.reciprocalResolution, 1./0.2)

      self._pickle_test(map)
      
    def test_trajectory(self):
      traj = PositionTrajectory()
      traj.append(Position2dStamped(0., 1., Time(2.0)))
      self._pickle_test(traj)
      
      traj = PoseTrajectory()
      traj.append(Pose2dStamped(0., 1., 2.0, Time(2.0)))
      self._pickle_test(traj)
      
      traj = StateTrajectory()
      traj.append(StateStamped(State(np.array([3.0, 4.0]), Pose2d(0.0, 1.0, 2.0)), Time(0.1)))
      self._pickle_test(traj)
      
      traj = SystemInputTrajectory()
      traj.append(SystemInputStamped(SystemInput(np.array([3.0, 4.0])), Time(0.1)))
      self._pickle_test(traj)
      
      path = Path()
      path.append(Pose2d(0.0, 1.0, 2.0))
      self._pickle_test(path)
        
if __name__ == '__main__':
    import rostest
    rostest.rosrun('planner_interfaces_python', 'PlannerInterfaces', TestPlannerInterfaces)
