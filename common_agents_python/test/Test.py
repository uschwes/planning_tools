#!/usr/bin/env python
import common_agents_python as cap
import numpy as np
import pickle, os

import unittest

class TestCommonAgents(unittest.TestCase):
    def _pickle_test(self, msg, comparator = None):
      if comparator is None:
        comparator = lambda self,x,y: self.assertEqual(x, y, "")
      pfile = "test.pickle"
      pickle.dump( msg, open( pfile, "wb" ) )
      msgPickled = pickle.load( open( pfile, "rb" ) )
      os.remove(pfile)
      comparator(self, msg, msgPickled)
      
    def testHolonomicState(self):
      s0 = cap.HolonomicState()
      self.assertEqual(s0.dimension, 2)
      
      s0 = cap.HolonomicState(cap.State(np.array([3., 4.]), cap.Pose2d(0.0, 1.0, 2.0)))
      self.assertEqual(s0.pose.position.x, 0.0)
      self.assertEqual(s0.pose.position.y, 1.0)
      self.assertEqual(s0.pose.yaw, 2.0)
      self.assertEqual(s0.vx, 3.0)
      self.assertEqual(s0.vy, 4.0)
      self.assertTrue( (s0.velocity == np.array([3.0, 4.0])).all() )
      
      s1 = cap.HolonomicState(s0)
      self.assertEqual(s1.pose.position.x, 0.0)
      self.assertEqual(s1.pose.position.y, 1.0)
      self.assertEqual(s1.pose.yaw, 2.0)
      self.assertEqual(s1.vx, 3.0)
      self.assertEqual(s1.vy, 4.0)
      self.assertTrue( (s1.velocity == np.array([3.0, 4.0])).all() )
      
      s2 = cap.HolonomicState(3.0, 4.0, cap.Pose2d(0.0, 1.0, 2.0))
      self.assertEqual(s2.pose.position.x, 0.0)
      self.assertEqual(s2.pose.position.y, 1.0)
      self.assertEqual(s2.pose.yaw, 2.0)
      self.assertEqual(s2.vx, 3.0)
      self.assertEqual(s2.vy, 4.0)
      self.assertTrue( (s2.velocity == np.array([3.0, 4.0])).all() )
      
      s3 = cap.HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0)
      self.assertEqual(s3.pose.position.x, 0.0)
      self.assertEqual(s3.pose.position.y, 1.0)
      self.assertEqual(s3.pose.yaw, 2.0)
      self.assertEqual(s3.vx, 3.0)
      self.assertEqual(s3.vy, 4.0)
      self.assertTrue( (s3.velocity == np.array([3.0, 4.0])).all() )
      
      self._pickle_test(s3)
        
    def testHolonomicSystemInput(self):
      
      u = cap.HolonomicSystemInput()
      u.vx = 0.0
      u.vy = 1.0
      self.assertEqual(u.vx, 0.0)
      self.assertEqual(u.vy, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      u = cap.HolonomicSystemInput(cap.SystemInput(np.array([0., 1.])))
      self.assertEqual(u.vx, 0.0)
      self.assertEqual(u.vy, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      u = cap.HolonomicSystemInput(0., 1.0)
      self.assertEqual(u.vx, 0.0)
      self.assertEqual(u.vy, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      u = cap.HolonomicSystemInput(np.array([0., 1.]))
      self.assertEqual(u.vx, 0.0)
      self.assertEqual(u.vy, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      self._pickle_test(u)
    
    def testHolonomicAgent(self):
      a = cap.HolonomicAgent()
      self.assertRaises(RuntimeError, lambda:  a.id)
      a.id = 3
      self.assertEqual(a.id, 3)
      a.isControllable = True
      self.assertTrue(a.isControllable)
      a.isControllable = False
      self.assertFalse(a.isControllable)
      
      ss = cap.HolonomicStateStamped(cap.HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0), cap.Time(0.1))
      a.stateStamped = ss
      self.assertEqual(a.stateStamped.stamp, cap.Time(0.1))
      self.assertEqual(a.stateStamped.pose.position.x, 0.0)
      self.assertEqual(a.stateStamped.pose.position.y, 1.0)
      self.assertEqual(a.stateStamped.pose.yaw, 2.0)
      self.assertEqual(cap.HolonomicState(a.stateStamped).vx, 3.0)
      self.assertEqual(cap.HolonomicState(a.stateStamped).vy, 4.0)
      self.assertTrue( (cap.HolonomicState(a.stateStamped).velocity == np.array([3.0, 4.0])).all() )
      
      self._pickle_test(a, comparator = lambda self,x,y: self.assertEqual(x.stateStamped.stamp, y.stateStamped.stamp))
      
    def testDifferentialDriveState(self):
      s0 = cap.DifferentialDriveState()
      self.assertEqual(s0.dimension, 2)
      
      s0 = cap.DifferentialDriveState(cap.State(np.array([3., 4.]), cap.Pose2d(0.0, 1.0, 2.0)))
      self.assertEqual(s0.pose.position.x, 0.0)
      self.assertEqual(s0.pose.position.y, 1.0)
      self.assertEqual(s0.pose.yaw, 2.0)
      self.assertEqual(s0.vt, 3.0)
      self.assertEqual(s0.vr, 4.0)
      self.assertTrue( (s0.velocity == np.array([3.0, 4.0])).all() )
      
      s1 = cap.DifferentialDriveState(s0)
      self.assertEqual(s1.pose.position.x, 0.0)
      self.assertEqual(s1.pose.position.y, 1.0)
      self.assertEqual(s1.pose.yaw, 2.0)
      self.assertEqual(s1.vt, 3.0)
      self.assertEqual(s1.vr, 4.0)
      self.assertTrue( (s1.velocity == np.array([3.0, 4.0])).all() )
      
      s2 = cap.DifferentialDriveState(3.0, 4.0, cap.Pose2d(0.0, 1.0, 2.0))
      self.assertEqual(s2.pose.position.x, 0.0)
      self.assertEqual(s2.pose.position.y, 1.0)
      self.assertEqual(s2.pose.yaw, 2.0)
      self.assertEqual(s2.vt, 3.0)
      self.assertEqual(s2.vr, 4.0)
      self.assertTrue( (s2.velocity == np.array([3.0, 4.0])).all() )
      
      s3 = cap.DifferentialDriveState(0.0, 1.0, 2.0, 3.0, 4.0)
      self.assertEqual(s3.pose.position.x, 0.0)
      self.assertEqual(s3.pose.position.y, 1.0)
      self.assertEqual(s3.pose.yaw, 2.0)
      self.assertEqual(s3.vt, 3.0)
      self.assertEqual(s3.vr, 4.0)
      self.assertTrue( (s3.velocity == np.array([3.0, 4.0])).all() )
      
      self._pickle_test(s3)
        
    def testDifferentialDriveSystemInput(self):
      
      u = cap.DifferentialDriveSystemInput()
      u.vt = 0.0
      u.vr = 1.0
      self.assertEqual(u.vt, 0.0)
      self.assertEqual(u.vr, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      u = cap.DifferentialDriveSystemInput(cap.SystemInput(np.array([0., 1.])))
      self.assertEqual(u.vt, 0.0)
      self.assertEqual(u.vr, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      u = cap.DifferentialDriveSystemInput(0., 1.0)
      self.assertEqual(u.vt, 0.0)
      self.assertEqual(u.vr, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      u = cap.DifferentialDriveSystemInput(np.array([0., 1.]))
      self.assertEqual(u.vt, 0.0)
      self.assertEqual(u.vr, 1.0)
      self.assertEqual(u.velocity[0], 0.0)
      self.assertEqual(u.velocity[1], 1.0)
      
      self._pickle_test(u)
    
    def testDifferentialDriveAgent(self):
      a = cap.DifferentialDriveAgent()
      self.assertRaises(RuntimeError, lambda:  a.id)
      a.id = 3
      self.assertEqual(a.id, 3)
      a.isControllable = True
      self.assertTrue(a.isControllable)
      a.isControllable = False
      self.assertFalse(a.isControllable)
      
      ss = cap.DifferentialDriveStateStamped(cap.DifferentialDriveState(0.0, 1.0, 2.0, 3.0, 4.0), cap.Time(0.1))
      a.stateStamped = ss
      self.assertEqual(a.stateStamped.stamp, cap.Time(0.1))
      self.assertEqual(a.stateStamped.pose.position.x, 0.0)
      self.assertEqual(a.stateStamped.pose.position.y, 1.0)
      self.assertEqual(a.stateStamped.pose.yaw, 2.0)
      self.assertEqual(cap.DifferentialDriveState(a.stateStamped).vt, 3.0)
      self.assertEqual(cap.DifferentialDriveState(a.stateStamped).vr, 4.0)
      self.assertTrue( (cap.DifferentialDriveState(a.stateStamped).velocity == np.array([3.0, 4.0])).all() )
        
      self._pickle_test(a, comparator = lambda self,x,y: self.assertEqual(x.stateStamped.stamp, y.stateStamped.stamp))
      
if __name__ == '__main__':
    import rostest
    rostest.rosrun('common_agents_python', 'CommonAgents', TestCommonAgents)