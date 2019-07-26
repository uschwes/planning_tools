#!/usr/bin/env python
import fcl_python
from probabilistic_planner import *
from common_agents_python import *
import numpy as np

import unittest
from probabilistic_planner.libprobabilistic_planner_python import OptAgentType

class TestProbabilisticPlanner(unittest.TestCase):
  
    def testOptAgent(self):
      a = HolonomicAgent()
      a.stateStamped = HolonomicStateStamped(HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0), Time(0.1))
      a.id = 0
      agent = OptAgent(a, Trajectory(), OptAgentType.PEDESTRIAN)
      
    def testTrajectory(self):

      pt = PositionTrajectory()
      pt.append(Position2dStamped(Position2d(0.0, 0.0), Time(0.0)))
      pt.append(Position2dStamped(Position2d(1.0, 0.0), Time(1.0)))
      st = StateTrajectory()
      st.append(StateStamped(State(np.array([]), Pose2d(0., 0., 0.)), Time(0.0)))
      st.append(StateStamped(State(np.array([]), Pose2d(1., 0., 0.)), Time(10.0)))
      t = Trajectory()
      t.initFromDiscretizedTrajectory(st, 10, 1e-3)
      self.assertEqual(t.startTime.toSec(), 0.0)
      self.assertEqual(t.finalTime.toSec(), 10.0)
      t = Trajectory()
      t.initFromDiscretizedTrajectory(pt, 10, 1e-3, minTime=Time(-1.0), maxTime=Time(11.0))
      self.assertEqual(t.startTime.toSec(), -1.0)
      self.assertEqual(t.finalTime.toSec(), 11.0)
      t = Trajectory()
      t.initFromDiscretizedTrajectory(st, 10, 1e-3)
       
      t = Trajectory()
      self.assertFalse(t.isInitialized)
      t.initStraightSpline(HolonomicStateStamped(HolonomicState(0., 1., 0., 0., 0.), Time(0.0) ),
                           HolonomicStateStamped(HolonomicState(10., 11., 0., 0., 0.), Time(10.0) ),
                           10, 1e-3)
      self.assertTrue(t.isInitialized)
      self.assertEqual(t.startTime.toSec(), 0.0)
      self.assertEqual(t.finalTime.toSec(), 10.0)
      
      tolerance = 1e-3
      p = t.getPosition(Time(0.0))
      self.assertAlmostEqual(p[0], 0.0, delta=tolerance)
      self.assertAlmostEqual(p[1], 1.0, delta=tolerance)
      p = t.getPosition(Time(10.0))
      self.assertAlmostEqual(p[0], 10.0, delta=tolerance)
      self.assertAlmostEqual(p[1], 11.0, delta=tolerance)
      
    def testContinuousScene(self):
      scene = ContinuousScene()
      self.assertTrue(scene.empty())
      
      a = HolonomicAgent()
      a.stateStamped = HolonomicStateStamped(HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0), Time(0.1))
      a.id = 0
      
      t = Trajectory()
      t.initStraightSpline(HolonomicStateStamped(HolonomicState(0., 0., 0., 0., 0.), Time(0.0) ),
                           HolonomicStateStamped(HolonomicState(10., 10., 0., 0., 0.), Time(10.0) ),
                           10,
                           1e-3)
      
      agent = OptAgent(a, t, OptAgentType.PEDESTRIAN)
      
      scene.addOptAgent(agent)
      self.assertEqual(scene.minTime.toSec(), 0.0)
      self.assertEqual(scene.maxTime.toSec(), 10.0)
      
    def testSceneSnapshot(self):
      id = 0
      sn = SceneSnapshot(Time(0.0))
      state = HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0)
      uncertainty = np.eye(5,5)
      su = StateWithUncertainty(state, uncertainty)
      sn.addObject(id, su)
      self.assertRaises(RuntimeError, lambda: sn.getObject(id + 1))
      o = sn.getObject(id)
      self.assertEqual(o.pose, state.pose)
      self.assertTrue( (o.state == state.state).all() )
      self.assertTrue( (o.invCov == uncertainty).all() )

    def testMeasurementPosition2D(self):
      scene = ContinuousScene()
      
      a = HolonomicAgent()
      a.stateStamped = HolonomicStateStamped(HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0), Time(0.1))
      a.id = 0
      
      t = Trajectory()
      t.initStraightSpline(HolonomicStateStamped(HolonomicState(0., 0., 0., 0., 0.), Time(0.0) ),
                           HolonomicStateStamped(HolonomicState(10., 10., 0., 0., 0.), Time(10.0) ),
                           10,
                           1e-3)
      
      agent = OptAgent(a, t, OptAgentType.PEDESTRIAN)
      
      scene.addOptAgent(agent)
      
      id = 0
      sn = SceneSnapshot(Time(0.0))
      su = StateWithUncertainty(HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0), np.eye(5,5))
      sn.addObject(id, su)
      
      ssn = SnapshotContainer()
      ssn.append(sn)

    def testFeatures(self):
      fVel = FeatureSingletonIntegratedVelocity(OptAgentType.PEDESTRIAN, 1.0)
      fRot = FeatureSingletonIntegratedRotationRate(OptAgentType.PEDESTRIAN, 1.0)
      fProd = FeatureSingletonProduct(fVel, fRot, OptAgentType.PEDESTRIAN, 2.0)

      scene = ContinuousScene()
      a = HolonomicAgent()
      a.stateStamped = HolonomicStateStamped(HolonomicState(0.0, 1.0, 2.0, 3.0, 4.0), Time(0.1))
      a.id = 0
      t = Trajectory()
      positions = PositionTrajectory()
      positions.append(Position2dStamped(0.0, 0.0, Time(0.0)))
      positions.append(Position2dStamped(1.0, 0.0, Time(1.0)))
      positions.append(Position2dStamped(1.0, 1.0, Time(2.0)))
      t.initFromDiscretizedTrajectory(positions, 10, 1e-3)
      agent = OptAgent(a, t, OptAgentType.PEDESTRIAN)
      scene.addOptAgent(agent)

      self.assertGreater(fProd.evaluate(scene), 0.0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('probabilistic_planner_python', 'ProbabilisticPlanner', TestProbabilisticPlanner)