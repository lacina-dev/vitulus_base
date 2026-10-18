#!/usr/bin/env python3
"""drive_primitive logic, offline: no ROS, simulated clock and odometry.

    python3 -m pytest test/test_drive_primitive.py      (or plain python3)

The simulated robot integrates the node's own (v, w) output, so the closed
loop, the ramps and the stop at the target are exercised for real.
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_PATH = os.path.join(os.path.dirname(HERE), 'nodes', 'drive_primitive')
_loader = importlib.machinery.SourceFileLoader('drive_primitive', _PATH)
_spec = importlib.util.spec_from_loader('drive_primitive', _loader)
dp = importlib.util.module_from_spec(_spec)
_loader.exec_module(dp)

DT = 0.05


class Sim(object):
    def __init__(self, yaw=0.0, points=None, motor=True, mux='nav', sup='RUNNING',
                 slip=1.0, params=None):
        self.t = 100.0
        self.x = self.y = 0.0
        self.yaw = yaw
        self.slip = slip
        self.points = [(3.0, 3.0)] if points is None else points
        self.cancels = []
        self.drv = dp.Driver(params or {}, clock=lambda: self.t,
                             on_cancel_nav=lambda: self.cancels.append(self.t))
        self.out = []
        self.feed(motor=motor, mux=mux, sup=sup)

    def feed(self, motor=True, mux='nav', sup='RUNNING', odom=True, scan=True):
        if odom:
            self.drv.on_odom(self.x, self.y, self.yaw)
        if scan:
            self.drv.on_points(self.points)
        if motor is not None:
            self.drv.on_motor(motor)
        if mux is not None:
            self.drv.on_mux_active(mux)
        if sup is not None:
            self.drv.on_supervisor(sup)

    def run(self, seconds, odom=True, scan=True, sup='RUNNING', keepalive=None, hook=None):
        n = int(seconds / DT)
        for i in range(n):
            res = self.drv.step()
            self.out.append(res)
            if res is not None:
                v, w = res
                self.x += v * self.slip * DT * math.cos(self.yaw)
                self.y += v * self.slip * DT * math.sin(self.yaw)
                self.yaw = dp.wrap(self.yaw + w * self.slip * DT)
            self.t += DT
            if keepalive:
                self.drv.on_keepalive(keepalive)
            if hook:
                hook(self, i)
            self.feed(motor=None, mux=None, sup=sup, odom=odom, scan=scan)
            if not self.drv.running() and self.drv.zeros_left == 0:
                break
        return self.drv.status

    def cmd(self, **kw):
        kw.setdefault('id', 'c1')
        return self.drv.command(json.dumps(kw))


class TestDrive(unittest.TestCase):
    def test_straight_forward(self):
        s = Sim()
        st = s.cmd(kind='straight', distance_m=0.5, speed=0.2)
        self.assertEqual(st['state'], 'RUNNING')
        st = s.run(10)
        self.assertEqual(st['state'], 'SUCCEEDED', st)
        self.assertAlmostEqual(s.x, 0.5, delta=0.03)
        vs = [o[0] for o in s.out if o]
        self.assertLessEqual(max(vs), 0.2 + 1e-9)
        # ramp: first commands are small
        self.assertLess(vs[0], 0.05)
        self.assertEqual(s.out[-1], (0.0, 0.0))
        self.assertEqual(sum(1 for o in s.out if o == (0.0, 0.0)), 3)

    def test_straight_reverse_obstacle_behind(self):
        s = Sim(points=[(-0.55, 0.0)])     # 0.26 m behind the rear edge
        st = s.cmd(kind='straight', distance_m=-0.5, speed=0.15)
        self.assertEqual(st['state'], 'REJECTED')
        self.assertIn('behind', st['reason'])
        s2 = Sim(points=[(-1.5, 0.0)])
        self.assertEqual(s2.cmd(kind='straight', distance_m=-0.4, speed=0.15)['state'], 'RUNNING')
        self.assertEqual(s2.run(10)['state'], 'SUCCEEDED')
        self.assertAlmostEqual(s2.x, -0.4, delta=0.03)

    def test_obstacle_ahead_rejected_and_aborted(self):
        s = Sim(points=[(0.4, 0.1)])
        st = s.cmd(kind='straight', distance_m=1.0)
        self.assertEqual(st['state'], 'REJECTED')
        self.assertIn('ahead', st['reason'])
        s = Sim(points=[(1.2, 0.0)])
        s.cmd(kind='straight', distance_m=1.5, speed=0.2)

        def hook(sim, i):
            # the static point in base_link gets closer as the robot drives
            sim.points = [(1.2 - sim.x, 0.0)]
        st = s.run(15, hook=hook)
        self.assertEqual(st['state'], 'ABORTED')
        self.assertIn('obstacle ahead', st['reason'])
        self.assertLess(s.x, 0.8)
        # obstacle beside the corridor does not stop it
        s = Sim(points=[(0.5, 0.6)])
        self.assertEqual(s.cmd(kind='straight', distance_m=0.3)['state'], 'RUNNING')
        # self return inside the box is ignored
        s = Sim(points=[(0.1, 0.1)])
        self.assertEqual(s.cmd(kind='straight', distance_m=0.3)['state'], 'RUNNING')

    def test_rotate_negative_across_pi(self):
        s = Sim(yaw=-3.0)
        st = s.cmd(kind='rotate', angle_deg=-90, speed=0.5)
        self.assertEqual(st['state'], 'RUNNING')
        st = s.run(15)
        self.assertEqual(st['state'], 'SUCCEEDED', st)
        turned = dp.wrap(s.yaw - (-3.0))
        self.assertAlmostEqual(math.degrees(turned), -90, delta=3)
        s = Sim(yaw=3.0)
        s.cmd(kind='rotate', angle_deg=120)
        self.assertEqual(s.run(20)['state'], 'SUCCEEDED')
        self.assertAlmostEqual(math.degrees(dp.wrap(s.yaw - 3.0)), 120, delta=3)

    def test_rotate_turning_circle(self):
        s = Sim(points=[(0.3, 0.3)])   # outside box, inside the circle
        st = s.cmd(kind='rotate', angle_deg=45)
        self.assertEqual(st['state'], 'REJECTED')
        self.assertIn('turning circle', st['reason'])

    def test_velocity_duration(self):
        s = Sim()
        s.cmd(kind='velocity', v=0.1, w=0.0, duration_s=2.0)
        st = s.run(10)
        self.assertEqual(st['state'], 'SUCCEEDED')
        self.assertAlmostEqual(st['elapsed_s'], 2.0, delta=0.1)
        self.assertEqual(s.cmd(kind='velocity', v=0.1, duration_s=6)['state'], 'REJECTED')

    def test_takeover(self):
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0)
        s.run(0.5)
        s.drv.on_mux_active('teleop')
        st = s.run(1)
        self.assertEqual(st['state'], 'ABORTED')
        self.assertIn('takeover', st['reason'])
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0)
        s.drv.on_takeover('ds4')
        self.assertEqual(s.drv.status['state'], 'ABORTED')
        s = Sim(mux='ds4')
        self.assertEqual(s.cmd(kind='straight', distance_m=1.0)['state'], 'REJECTED')

    def test_stop(self):
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0)
        s.run(0.5)
        s.drv.on_stop(True)
        self.assertEqual(s.drv.status['state'], 'ABORTED')
        self.assertEqual(s.drv.status['reason'], 'stop')
        self.assertEqual(s.drv.step(), (0.0, 0.0))

    def test_keepalive(self):
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1, keepalive_s=0.6)
        st = s.run(3, keepalive='c1')
        self.assertEqual(st['state'], 'RUNNING')
        st = s.run(2)
        self.assertEqual(st['state'], 'ABORTED')
        self.assertIn('keepalive', st['reason'])
        # a keepalive of another id does not count
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1, keepalive_s=0.6)
        self.assertEqual(s.run(2, keepalive='other')['state'], 'ABORTED')

    def test_supervisor(self):
        s = Sim(sup='READY')
        st = s.cmd(kind='straight', distance_m=0.5, require_supervisor=True)
        self.assertEqual(st['state'], 'REJECTED')
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1, require_supervisor=True)
        s.run(0.5)
        st = s.run(3, sup=None)       # status stops arriving
        self.assertEqual(st['state'], 'ABORTED')
        self.assertIn('stale', st['reason'])
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1, require_supervisor=True)
        st = s.run(1, sup='FAULT')
        self.assertIn('FAULT', st['reason'])
        # without require_supervisor the supervisor does not matter
        s = Sim(sup='FAULT')
        self.assertEqual(s.cmd(kind='straight', distance_m=0.3)['state'], 'RUNNING')

    def test_motors_off(self):
        s = Sim(motor=False)
        self.assertEqual(s.cmd(kind='straight', distance_m=0.5)['reason'], 'motors off')
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1)
        s.run(0.5)
        s.drv.on_motor(False)
        self.assertEqual(s.run(0.5)['reason'], 'motors off')

    def test_stale_odom_and_scan(self):
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1)
        st = s.run(2, odom=False)
        self.assertIn('odometry', st['reason'])
        s = Sim()
        s.cmd(kind='straight', distance_m=1.0, speed=0.1)
        st = s.run(2, scan=False)
        self.assertIn('lidar', st['reason'])

    def test_stuck(self):
        s = Sim(slip=0.0)
        s.cmd(kind='straight', distance_m=1.0, speed=0.2)
        st = s.run(10)
        self.assertEqual(st['state'], 'ABORTED')
        self.assertIn('no progress', st['reason'])

    def test_invalid_and_superseded(self):
        s = Sim()
        self.assertEqual(s.cmd(kind='fly')['state'], 'REJECTED')
        self.assertEqual(s.cmd(kind='straight', distance_m=5)['state'], 'REJECTED')
        self.assertEqual(s.cmd(kind='straight', distance_m=0.5, speed=0.9)['state'], 'REJECTED')
        self.assertEqual(s.drv.command('not json')['state'], 'REJECTED')
        self.assertEqual(s.drv.command(json.dumps({'kind': 'straight'}))['state'], 'REJECTED')
        s.cmd(id='a', kind='straight', distance_m=1.0)
        s.cmd(id='b', kind='rotate', angle_deg=30, cancel_nav=True)
        self.assertEqual(s.drv.status['id'], 'b')
        self.assertEqual(len(s.cancels), 1)

    def test_timeout(self):
        s = Sim(slip=0.5)
        s.cmd(kind='straight', distance_m=1.0, speed=0.2, max_duration_s=2.0)
        st = s.run(10)
        self.assertEqual(st['state'], 'ABORTED')
        self.assertIn('timeout', st['reason'])


if __name__ == '__main__':
    sys.exit(unittest.main())
