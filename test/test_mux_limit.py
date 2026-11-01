#!/usr/bin/env python3
"""cmd_vel_mux /cmd_vel_mux/limit, offline: no master, the real _cb with a fake output.

    python3 -m pytest test/test_mux_limit.py      (or plain python3)
"""
import importlib.machinery
import importlib.util
import json
import os
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

from geometry_msgs.msg import Twist  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
_PATH = os.path.join(os.path.dirname(HERE), 'nodes', 'cmd_vel_mux')
_loader = importlib.machinery.SourceFileLoader('cmd_vel_mux', _PATH)
_spec = importlib.util.spec_from_loader('cmd_vel_mux', _loader)
mx = importlib.util.module_from_spec(_spec)
_loader.exec_module(mx)

NAV = {'name': 'nav', 'topic': '/cmd_vel', 'priority': 10, 'human': False}
PRIM = {'name': 'primitive', 'topic': '/p', 'priority': 50, 'human': False}
TELEOP = {'name': 'teleop', 'topic': '/teleop/cmd_vel', 'priority': 90, 'human': True}
DS4 = {'name': 'ds4', 'topic': '/ds4/cmd_vel', 'priority': 100, 'human': True}


class Out(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Sink(object):
    def publish(self, msg):
        pass


class Str(object):
    def __init__(self, data):
        self.data = data


def twist(v, w):
    t = Twist()
    t.linear.x, t.angular.z = v, w
    return t


def limit(**kw):
    d = {'source': 'map_rules', 'stamp': 1.0, 'v_fwd': 0.4, 'v_back': 0.25, 'w': 0.9,
         'rules': 2, 'reason': 'nogo:Bazen'}
    d.update(kw)
    return Str(json.dumps(d))


def make_mux():
    m = mx.Mux.__new__(mx.Mux)
    m.now = 100.0
    m.clock = lambda: m.now
    m.lock = mx.threading.RLock()
    m.dz_lin, m.dz_ang = 0.02, 0.05
    m.hold_s, m.rearm_quiet_s, m.software_hold_s = 1.5, 1.0, 0.5
    m.sw_dropped, m.sw_last_msg = {}, {}
    m.cancel_on_takeover = False
    m.inputs = [DS4, TELEOP, PRIM, NAV]
    m.out = Out()
    m.pub_takeover = Sink()
    m.holds, m.hold_blocked_last = {}, None
    m.human_last_deflect = {}
    m.takeover_active, m.takeover_count = False, 0
    m.last_takeover, m.session_end = None, None
    m.nav_last_nonzero, m.nav_admitted, m.nav_blocked_now = {}, set(), {}
    m.last_forwarded = None
    m.limit, m.limit_t, m.limit_warn_t = None, None, None
    m.limit_stale_said = False
    m.limit_said_t = None
    m.pub_warn, m.pub_log = Out(), Out()
    m.saved = []
    m._save_limit_engaged = m.saved.append
    m._publish_status = lambda force=False: None
    m._start_takeover = lambda source, now: None
    return m


class Clamp(unittest.TestCase):
    LIM = {'v_fwd': 0.2, 'v_back': 0.1, 'w': 0.9, 'w_left': 0.9, 'w_right': 0.9}

    def test_directions(self):
        self.assertEqual(mx.clamp_cmd(0.5, 0.0, self.LIM), (0.2, 0.0))
        self.assertEqual(mx.clamp_cmd(-0.5, 0.0, self.LIM), (-0.1, 0.0))
        self.assertEqual(mx.clamp_cmd(0.1, 0.3, self.LIM), (0.1, 0.3))

    def test_arc_is_kept(self):
        v, w = mx.clamp_cmd(0.4, 0.4, self.LIM)
        self.assertAlmostEqual(v, 0.2)
        self.assertAlmostEqual(w, 0.2)
        self.assertAlmostEqual(v / w, 0.4 / 0.4)

    def test_arc_is_kept_reversing(self):
        v, w = mx.clamp_cmd(-0.3, 0.3, self.LIM)
        self.assertAlmostEqual(v / w, -1.0)

    def test_stopped_by_a_rule_still_turns(self):
        lim = dict(self.LIM, v_fwd=0.0)
        self.assertEqual(mx.clamp_cmd(0.1, 0.4, lim), (0.0, 0.4))

    def test_pure_rotation_is_not_scaled(self):
        lim = dict(self.LIM, v_fwd=0.0, v_back=0.0)
        self.assertEqual(mx.clamp_cmd(0.0, 0.4, lim), (0.0, 0.4))

    def test_directional_turn(self):
        lim = dict(self.LIM, w_left=0.0, w_right=0.5)
        self.assertEqual(mx.clamp_cmd(0.0, 0.4, lim), (0.0, 0.0))
        self.assertEqual(mx.clamp_cmd(0.0, -0.4, lim), (0.0, -0.4))
        self.assertEqual(mx.clamp_cmd(0.0, -0.9, lim), (0.0, -0.5))


class Parse(unittest.TestCase):
    def test_missing_directional_fields_use_w(self):
        lim = mx.parse_limit(limit().data)
        self.assertEqual((lim['w_left'], lim['w_right']), (0.9, 0.9))

    def test_no_rules(self):
        self.assertIsNone(mx.parse_limit(limit(rules=0).data))

    def test_refused(self):
        for raw in ('x', '[]', limit(source='other').data, limit(v_fwd='a').data,
                    limit(v_fwd=-1).data, limit(w=float('nan')).data,
                    '{"source": "map_rules"}'):
            with self.assertRaises(ValueError):
                mx.parse_limit(raw)


class Forward(unittest.TestCase):
    def test_no_message_ever_no_limit(self):
        m = make_mux()
        msg = twist(1.0, 1.0)
        m._cb(msg, NAV)
        self.assertIs(m.out.sent[-1], msg)

    def test_rules_zero_is_no_limit_and_never_stale(self):
        m = make_mux()
        m._limit(limit(rules=0))
        m.now += 100.0
        msg = twist(1.0, 1.0)
        m._cb(msg, NAV)
        self.assertIs(m.out.sent[-1], msg)

    def test_software_is_clamped(self):
        m = make_mux()
        m._limit(limit(v_fwd=0.1))
        m._cb(twist(0.4, 0.4), NAV)
        self.assertAlmostEqual(m.out.sent[-1].linear.x, 0.1)
        self.assertAlmostEqual(m.out.sent[-1].angular.z, 0.1)
        m.now += 2.0
        m._limit(limit(v_fwd=0.1))
        m._cb(twist(0.4, 0.0), PRIM)
        self.assertAlmostEqual(m.out.sent[-1].linear.x, 0.1)

    def test_under_the_limit_goes_out_as_it_came(self):
        m = make_mux()
        m._limit(limit())
        msg = twist(0.1, 0.1)
        m._cb(msg, NAV)
        self.assertIs(m.out.sent[-1], msg)

    def test_leaving_stays_open(self):
        m = make_mux()
        m._limit(limit(v_fwd=0.0, v_back=0.15))
        m._cb(twist(0.3, 0.0), NAV)
        self.assertEqual(m.out.sent[-1].linear.x, 0.0)
        m._cb(twist(-0.1, 0.0), NAV)
        self.assertAlmostEqual(m.out.sent[-1].linear.x, -0.1)

    def test_stale_stops_software(self):
        m = make_mux()
        m._limit(limit())
        m.now += 1.01
        m._cb(twist(0.1, 0.1), NAV)
        self.assertEqual(m.out.sent[-1], Twist())
        self.assertTrue(m._limit_status(m.now)['stale'])
        m._limit(limit())
        m._cb(twist(0.1, 0.1), NAV)
        self.assertEqual(m.out.sent[-1].linear.x, 0.1)

    def test_not_stale_at_the_contract_age(self):
        m = make_mux()
        m._limit(limit())
        m.now += 0.99
        m._cb(twist(0.1, 0.0), NAV)
        self.assertEqual(m.out.sent[-1].linear.x, 0.1)

    def test_bad_message_goes_stale_rather_than_open(self):
        m = make_mux()
        m._limit(limit())
        m.now += 2.0
        m._limit(Str('garbage'))
        m._cb(twist(0.1, 0.0), NAV)
        self.assertEqual(m.out.sent[-1], Twist())

    def test_restart_under_rules_starts_stale(self):
        m = make_mux()
        m.limit = {'stale_start': True}        # __init__ with ~limit_engaged
        m._cb(twist(0.1, 0.0), NAV)
        self.assertEqual(m.out.sent[-1], Twist())
        m._limit(limit(rules=0))
        self.assertEqual(m.saved, [False])
        m._cb(twist(0.1, 0.0), NAV)
        self.assertEqual(m.out.sent[-1].linear.x, 0.1)

    def test_engaged_flag_on_change_only(self):
        m = make_mux()
        m._limit(limit())
        m._limit(limit())
        m._limit(limit(rules=0))
        self.assertEqual(m.saved, [True, False])

    def test_human_is_never_clamped(self):
        for inp in (TELEOP, DS4):
            m = make_mux()
            m._limit(limit(v_fwd=0.0, v_back=0.0, w=0.0, w_left=0.0, w_right=0.0))
            msg = twist(1.0, 1.0)
            m._cb(msg, inp)
            self.assertIs(m.out.sent[-1], msg)
            m.now += 1.2                       # limit stale, still inside hold_s
            m._cb(msg, inp)
            self.assertIs(m.out.sent[-1], msg)


class Frozen(unittest.TestCase):
    ZERO = dict(v_fwd=0.0, v_back=0.0, w=0.0)

    def test_m2_fresh_zero_for_tf_or_a_bad_nogo_reads_as_stale(self):
        for reason in ('tf_stale', 'nogo_unparsable:Bazen'):
            m = make_mux()
            m._limit(limit(reason=reason, **self.ZERO))
            self.assertTrue(m._limit_status(m.now)['stale'], reason)
            self.assertTrue(m._limit_watch(m.now))
            self.assertEqual(len(m.pub_warn.sent), 1)

    def test_m2_a_nogo_ahead_is_not_stale(self):
        m = make_mux()
        m._limit(limit(reason='nogo:Bazen', v_fwd=0.0))
        self.assertFalse(m._limit_status(m.now)['stale'])
        self.assertFalse(m._limit_watch(m.now))

    def test_m2_it_ends_with_the_reason(self):
        m = make_mux()
        m._limit(limit(reason='tf_stale', **self.ZERO))
        m._limit_watch(m.now)
        m._limit(limit(reason=''))
        self.assertTrue(m._limit_watch(m.now))
        self.assertFalse(m._limit_status(m.now)['stale'])


class Repeat(unittest.TestCase):
    def test_every_60_s_while_it_stands(self):
        m = make_mux()
        m._limit(limit(reason='tf_stale', v_fwd=0.0, v_back=0.0, w=0.0))
        m._limit_watch(m.now)
        for dt in (30.0, 59.9):
            m.now = 100.0 + dt
            m._limit(limit(reason='tf_stale', v_fwd=0.0, v_back=0.0, w=0.0))
            m._limit_watch(m.now)
        self.assertEqual(len(m.pub_warn.sent), 1)
        m.now = 160.1
        m._limit(limit(reason='tf_stale', v_fwd=0.0, v_back=0.0, w=0.0))
        self.assertFalse(m._limit_watch(m.now), 'no status storm')
        self.assertEqual(len(m.pub_warn.sent), 2)
        self.assertEqual(len(m.pub_log.sent), 2)


class StaleIsReported(unittest.TestCase):
    def test_once_on_both_channels_and_in_status(self):
        m = make_mux()
        m._limit(limit())
        self.assertFalse(m._limit_watch(m.now))
        m.now += 1.5
        self.assertTrue(m._limit_watch(m.now), 'status goes out at once')
        self.assertFalse(m._limit_watch(m.now))
        self.assertEqual(json.loads(m.pub_warn.sent[0].data)['type'], 'limit')
        self.assertEqual(len(m.pub_warn.sent), 1)
        self.assertEqual(len(m.pub_log.sent), 1)

    def test_back_again(self):
        m = make_mux()
        m._limit(limit())
        m.now += 1.5
        m._limit_watch(m.now)
        m._limit(limit())
        self.assertTrue(m._limit_watch(m.now))
        self.assertFalse(m._limit_status(m.now)['stale'])

    def test_no_rules_says_nothing(self):
        m = make_mux()
        m.now += 100.0
        self.assertFalse(m._limit_watch(m.now))
        self.assertEqual(m.pub_warn.sent, [])


if __name__ == '__main__':
    unittest.main()
