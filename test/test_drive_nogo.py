#!/usr/bin/env python3
"""drive_primitive refuses a straight line into a nogo, offline.

    python3 -m pytest test/test_drive_nogo.py      (or plain python3)
"""
import importlib.machinery
import importlib.util
import json
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_PATH = os.path.join(os.path.dirname(HERE), 'nodes', 'drive_primitive')
_loader = importlib.machinery.SourceFileLoader('drive_primitive', _PATH)
_spec = importlib.util.spec_from_loader('drive_primitive', _loader)
dp = importlib.util.module_from_spec(_spec)
_loader.exec_module(dp)


def box(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def rules(*polys):
    return json.dumps({'site': 'A', 'rules': [{'type': 'nogo', 'name': 'Bazen', 'points': p}
                                              for p in polys]})


class Refusal(unittest.TestCase):
    def setUp(self):
        self.t = 100.0
        self.drv = dp.Driver(clock=lambda: self.t)
        self.drv.on_odom(0.0, 0.0, 0.0)
        self.drv.on_points([(3.0, 3.0)])
        self.drv.on_motor(True)
        self.drv.on_mux_active('nav')

    def go(self, dist=1.0, **kw):
        cmd = {'id': 'a', 'kind': 'straight', 'distance_m': dist}
        cmd.update(kw)
        return self.drv.command(json.dumps(cmd))

    def test_no_rules_needs_no_pose(self):
        self.assertFalse(self.drv.wants_map_pose())
        self.assertEqual(self.go()['state'], 'RUNNING')

    def test_crossing_is_refused_with_a_reason(self):
        self.drv.on_rules(rules(box(0.8, -1, 1.5, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        st = self.go()
        self.assertEqual(st['state'], 'REJECTED')
        self.assertEqual(st['reason'], 'target crosses nogo Bazen')
        self.assertIsNone(self.drv.cmd)

    def test_short_of_it_runs(self):
        self.drv.on_rules(rules(box(1.5, -1, 2.5, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go()['state'], 'RUNNING')

    def test_away_from_it_runs(self):
        self.drv.on_rules(rules(box(0.3, -1, 1.5, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go(-1.0)['state'], 'RUNNING')

    def test_reverse_into_it_is_refused(self):
        self.drv.on_rules(rules(box(-1.5, -1, -0.8, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go(-1.0)['state'], 'REJECTED')

    def test_body_width_counts(self):
        self.drv.on_rules(rules(box(0.8, 0.2, 1.5, 1)))    # beside the centre line
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go()['state'], 'REJECTED')

    def test_map_heading_counts(self):
        self.drv.on_rules(rules(box(-0.5, 0.8, 0.5, 1.5)))
        self.drv.on_map_pose(0.0, 0.0, 1.5708)
        self.assertEqual(self.go()['state'], 'REJECTED')

    def test_inside_may_leave(self):
        self.drv.on_rules(rules(box(-1, -1, 1, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go(-1.5)['state'], 'RUNNING')

    def test_no_map_pose_is_refused(self):
        self.drv.on_rules(rules(box(5, 5, 6, 6)))
        self.assertEqual(self.go()['reason'], 'nogo check: no map pose')
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.t += 2.0
        self.drv.on_odom(0.0, 0.0, 0.0)
        self.drv.on_points([(3.0, 3.0)])
        self.assertEqual(self.go()['reason'], 'nogo check: no map pose')

    def test_rotation_is_not_a_line(self):
        self.drv.on_rules(rules(box(0.3, -1, 1.5, 1)))
        st = self.drv.command(json.dumps({'id': 'r', 'kind': 'rotate', 'angle_deg': 30}))
        self.assertEqual(st['state'], 'RUNNING')

    def test_velocity_kind(self):
        self.drv.on_rules(rules(box(0.8, -1, 1.5, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        st = self.drv.command(json.dumps({'id': 'v', 'kind': 'velocity', 'v': 0.3,
                                          'duration_s': 4.0}))
        self.assertEqual(st['state'], 'REJECTED')

    def test_bad_rules_keep_the_last_good(self):
        self.drv.on_rules(rules(box(0.8, -1, 1.5, 1)))
        self.drv.on_rules('garbage')
        self.assertTrue(self.drv.wants_map_pose())


class Review(Refusal):
    def test_target_within_the_margin_is_refused(self):
        # Front ends at 1.238, the nogo 10 cm further: the mux would stop it short.
        self.drv.on_rules(rules(box(1.34, -1, 2.0, 1)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go()['state'], 'REJECTED')

    def arc(self, v, w, dur):
        return self.drv.command(json.dumps({'id': 'v', 'kind': 'velocity', 'v': v, 'w': w,
                                            'duration_s': dur}))

    def test_an_arc_is_checked_as_an_arc(self):
        # 0.3 m/s at 0.6 rad/s for 4 s bends left, the straight line misses this.
        self.drv.on_rules(rules(box(0.2, 0.5, 1.0, 1.0)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.arc(0.3, 0.6, 4.0)['state'], 'REJECTED')

    def test_an_arc_away_runs(self):
        self.drv.on_rules(rules(box(1.0, -0.3, 1.8, 0.3)))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.arc(0.3, 0.6, 5.0)['state'], 'RUNNING')

    def test_another_sites_rules_do_not_apply(self):
        self.drv.on_rules(rules(box(0.8, -1, 1.5, 1)))
        self.drv.on_status(json.dumps({'serving': {'site': 'B'}}))
        self.drv.on_map_pose(0.0, 0.0, 0.0)
        self.assertEqual(self.go()['state'], 'RUNNING')
        self.drv.on_status(json.dumps({'serving': {'site': 'A'}}))
        self.assertEqual(self.go()['state'], 'REJECTED')

    def test_site_null_keeps_the_rules(self):
        self.drv.on_rules(rules(box(0.8, -1, 1.5, 1)))
        self.drv.on_rules(json.dumps({'site': None, 'rules': []}))
        self.assertTrue(self.drv.wants_map_pose())

    def test_no_rules_key_keeps_the_rules(self):
        self.drv.on_rules(rules(box(0.8, -1, 1.5, 1)))
        self.drv.on_rules(json.dumps({'site': 'A'}))
        self.assertTrue(self.drv.wants_map_pose())


if __name__ == '__main__':
    unittest.main()
