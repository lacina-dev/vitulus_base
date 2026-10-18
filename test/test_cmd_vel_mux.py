#!/usr/bin/env python3
"""cmd_vel_mux in isolation: its own roscore on a private port, fake publishers.

    python3 test/test_cmd_vel_mux.py          # starts roscore on :11977 itself

Never talks to the robot's master: it refuses to run when ROS_MASTER_URI would
point at :11311 and always starts (and stops) its own roscore.  Nothing here
reaches the wheels - the base controller is not on that master.

Cases
  1  nav 10 Hz + DS4 zeros 250 Hz          -> nav passes, zeros dropped, no cancel
  2  DS4 deflected                         -> DS4 wins, cancels sent once, takeover
  3  DS4 released, nav obeyed the cancel   -> zeros during hold, then silence;
                                              the robot stays (no goal restart)
  4  nav that ignores the cancel           -> stays blocked
  4b another software node meanwhile       -> passes (blocking is per callerid)
  5  nav quiet, then a new goal            -> passes again
  5b Go right after release (obeyed node)  -> passes at once
  6  DS4 PS button                         -> takeover without moving a stick
  7  web joystick vs DS4                   -> DS4 ranks higher; web alone wins over nav
  9  mower_smach idle / mowing             -> /mower_smach/stop only while mowing
  10 drive_primitive vs nav               -> primitive owns the wheels while it talks
                                              (zeros too); nav again 0.5 s after it
                                              stops; teleop beats primitive and the
                                              takeover sends /drive_primitive/stop
  8  mux stopped                           -> nothing reaches the base
"""
import json
import os
import subprocess
import sys
import threading
import time

PORT = int(os.environ.get('MUX_TEST_PORT', '11977'))
URI = 'http://127.0.0.1:%d' % PORT
if PORT == 11311:
    sys.exit('refusing: 11311 is the robot master')
os.environ['ROS_MASTER_URI'] = URI
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)

HERE = os.path.dirname(os.path.abspath(__file__))
MUX = os.path.join(os.path.dirname(HERE), 'nodes', 'cmd_vel_mux')

FAIL = []


def ok(label, cond, detail=''):
    print('  %-62s %s%s' % (label, 'ok' if cond else 'FAIL', '' if cond else ' (%s)' % detail))
    if not cond:
        FAIL.append(label)


def main():
    core = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    mux = None
    try:
        time.sleep(3.0)
        import rospy
        from actionlib_msgs.msg import GoalID
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool, String
        from ds4_driver.msg import Status
        assert rospy.get_master().getUri()[2].rstrip('/').endswith(':%d' % PORT)
        rospy.init_node('mux_test', anonymous=True, disable_signals=True)

        mux = subprocess.Popen([sys.executable, MUX], stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
        out, cancels, stops, takeovers, status = [], [], [], [], []
        lk = threading.Lock()

        def rec(lst):
            def cb(m):
                with lk:
                    lst.append((time.monotonic(), m))
            return cb
        rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, rec(out), queue_size=1000)
        for t in ('/move_base_flex/exe_path/cancel', '/move_base_flex/move_base/cancel',
                  '/move_base_flex/recovery/cancel'):
            rospy.Subscriber(t, GoalID, rec(cancels), queue_size=100)
        for t in ('/smach_goal/cancel', '/dock_smach/stop', '/mower_smach/stop'):
            rospy.Subscriber(t, Bool, rec(stops), queue_size=100)
        mower_stops = []
        rospy.Subscriber('/mower_smach/stop', Bool, rec(mower_stops), queue_size=100)
        rospy.Subscriber('/cmd_vel_mux/takeover', String, rec(takeovers), queue_size=100)
        rospy.Subscriber('/cmd_vel_mux/status', String,
                         lambda m: status.append(json.loads(m.data)), queue_size=100)

        nav_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        ds4_pub = rospy.Publisher('/ds4/cmd_vel', Twist, queue_size=1)
        web_pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=1)
        st_pub = rospy.Publisher('/ds4/status', Status, queue_size=1)

        prim_pub = rospy.Publisher('/drive_primitive/cmd_vel', Twist, queue_size=1)
        prim_stops = []
        rospy.Subscriber('/drive_primitive/stop', Bool, rec(prim_stops), queue_size=100)
        drive = {'nav': None, 'ds4': 0.0, 'web': None, 'prim': None}   # None = silent

        def tw(lin, ang=0.0):
            m = Twist()
            m.linear.x = lin
            m.angular.z = ang
            return m

        stop_ev = threading.Event()

        def pump(pub, key, hz):
            period = 1.0 / hz
            while not stop_ev.is_set():
                v = drive[key]
                if v is not None:
                    pub.publish(tw(v))
                time.sleep(period)
        threads = [threading.Thread(target=pump, args=(nav_pub, 'nav', 10), daemon=True),
                   threading.Thread(target=pump, args=(ds4_pub, 'ds4', 250), daemon=True),
                   threading.Thread(target=pump, args=(web_pub, 'web', 20), daemon=True),
                   threading.Thread(target=pump, args=(prim_pub, 'prim', 20), daemon=True)]
        for t in threads:
            t.start()
        # wait for the mux to be connected
        deadline = time.time() + 15
        while time.time() < deadline and not status:
            time.sleep(0.2)
        ok('mux is up (status latched)', bool(status))
        time.sleep(1.5)

        def window(sec):
            with lk:
                n0 = len(out), len(cancels), len(stops), len(takeovers)
            time.sleep(sec)
            with lk:
                return (out[n0[0]:], cancels[n0[1]:], stops[n0[2]:], takeovers[n0[3]:])

        print('1  nav 10 Hz + DS4 centred zeros 250 Hz')
        drive['nav'] = 0.3
        o, c, s, t = window(3.0)
        vals = [m.linear.x for _, m in o]
        ok('nav passes (~30 msgs at 0.3)', len(vals) >= 20 and all(abs(v - 0.3) < 1e-6 for v in vals),
           '%d msgs %s' % (len(vals), sorted(set(vals))))
        ok('no DS4 zeros forwarded', 0.0 not in vals)
        ok('no cancel, no takeover', not c and not s and not t, '%d %d %d' % (len(c), len(s), len(t)))

        print('2  DS4 stick deflected')
        drive['ds4'] = 0.4
        o, c, s, t = window(2.0)
        vals = [m.linear.x for _, m in o]
        # a nav message already on its way before the first DS4 one may pass
        first = next((i for i, v in enumerate(vals) if abs(v - 0.4) < 1e-6), len(vals))
        vals = vals[first:]
        ok('DS4 wins (only 0.4 after its first message)',
           vals and all(abs(v - 0.4) < 1e-6 for v in vals) and first <= 1,
           (first, sorted(set(vals))))
        ok('DS4 forwarded at its rate (> 200 msgs / 2 s)', len(vals) > 200, len(vals))
        ok('3 goal cancels, once each', len(c) == 3, len(c))
        ok('3 stops (smach_goal, dock_smach, mower_smach), once each',
           len(s) == 3 and all(m.data for _, m in s), len(s))
        ok('one takeover named ds4', len(t) == 1 and t[0][1].data == 'ds4',
           [m.data for _, m in t])

        print('3  DS4 released, nav obeyed the cancel (stops publishing)')
        drive['nav'] = None
        drive['ds4'] = 0.0
        t_rel = time.monotonic()
        o, c, s, t = window(4.0)
        vals = [(ts - t_rel, m.linear.x) for ts, m in o]
        vals = [x for x in vals if x[0] > 0.03]   # a stick message still in flight
        ok('only zeros after release', all(v == 0.0 for _, v in vals), vals[:3])
        ok('zeros forwarded while holding (~1.5 s), then silence',
           vals and max(ts for ts, _ in vals) < 1.8 and len(vals) > 100,
           '%d msgs, last %.2f s' % (len(vals), max([ts for ts, _ in vals] or [0])))
        ok('no new cancel / takeover on release', not c and not s and not t)
        ok('status back to nav (rearmed)', status[-1]['active'] == 'nav', status[-1])

        print('4  nav ignoring the cancel stays blocked')
        drive['ds4'] = 0.5
        time.sleep(0.3)
        drive['ds4'] = 0.0
        drive['nav'] = 0.3          # keeps driving after the cancel
        with lk:
            c = list(cancels[-3:])
        o, _c, s, t = window(4.0)
        nonzero = [m.linear.x for _, m in o if m.linear.x != 0.0]
        ok('the new takeover cancelled again', len(cancels) == 6, len(cancels))
        ok('no software motion while it ignores the cancel', not [v for v in nonzero if abs(v - 0.3) < 1e-6],
           nonzero[:5])
        ok('status nav_blocked', status[-1]['active'] == 'nav_blocked', status[-1])

        print('4b another software node starts a new goal meanwhile')
        other = subprocess.Popen(['rostopic', 'pub', '-r', '10', '/cmd_vel', 'geometry_msgs/Twist',
                                  '{linear: {x: 0.15}}'], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
        time.sleep(1.5)
        o, _c, _s, _t = window(2.0)
        other.terminate()
        other.wait(5)
        vals = [round(m.linear.x, 3) for _, m in o]
        ok('the new node drives (0.15 passes)', vals.count(0.15) >= 10, '%d' % vals.count(0.15))
        ok('the ignoring node is still blocked (no 0.3)', 0.3 not in vals)
        ok('status lists the blocked caller', any('mux_test' in c for c in status[-1]['nav_blocked']),
           status[-1])
        time.sleep(0.5)

        print('5  nav quiet for rearm, then a new goal')
        drive['nav'] = None
        time.sleep(1.5)
        drive['nav'] = 0.2
        o, c, s, t = window(2.0)
        vals = [m.linear.x for _, m in o]
        ok('new goal passes', len(vals) >= 12 and all(abs(v - 0.2) < 1e-6 for v in vals),
           '%d %s' % (len(vals), sorted(set(vals))))

        print('5b a goal right after release from a node that obeyed the cancel')
        drive['ds4'] = 0.5
        time.sleep(0.3)
        drive['ds4'] = 0.0
        drive['nav'] = None          # obeys the cancel
        time.sleep(1.6)              # hold is over, rearm window would still run
        drive['nav'] = 0.2           # Go pressed at once
        o, c, s, t = window(1.5)
        vals = [m.linear.x for _, m in o]
        ok('the new goal drives at once', len(vals) >= 10 and all(abs(v - 0.2) < 1e-6 for v in vals),
           '%d %s' % (len(vals), sorted(set(vals))))

        print('6  DS4 PS button, sticks centred')
        st = Status()

        def press():
            time.sleep(0.1)
            st.button_ps = 1
            st_pub.publish(st)
            time.sleep(0.05)
            st.button_ps = 0
            st_pub.publish(st)
        threading.Thread(target=press, daemon=True).start()
        o, c, s, t = window(1.0)
        o = [x for x in o if x[0] > o[0][0] + 0.15] if o else o
        nav_vals = [m.linear.x for _, m in o if abs(m.linear.x - 0.2) < 1e-6]
        ok('PS button = takeover (cancels, takeover ds4)',
           len(c) == 3 and len(t) == 1 and t[0][1].data == 'ds4', '%d %d' % (len(c), len(t)))
        ok('software blocked right after the button', not nav_vals, len(nav_vals))
        drive['nav'] = None
        time.sleep(3.0)

        print('7  web joystick vs DS4')
        drive['nav'] = 0.1
        time.sleep(0.5)
        drive['web'] = 0.25
        o, c, s, t = window(1.5)
        vals = set(round(m.linear.x, 3) for _, m in o[3:])
        ok('web joystick beats nav, takeover teleop', vals == {0.25} and
           [m.data for _, m in t] == ['teleop'], (vals, [m.data for _, m in t]))
        drive['ds4'] = -0.3
        o, c, s, t = window(1.5)
        vals = set(round(m.linear.x, 3) for _, m in o[3:])
        ok('DS4 beats the web joystick', vals == {-0.3}, vals)
        ok('no second takeover while a human already has it', not t, [m.data for _, m in t])
        drive['ds4'] = 0.0
        drive['web'] = None
        drive['nav'] = None
        time.sleep(3.0)

        print('9  mower_smach idle (busy false, no program) -> no /mower_smach/stop')
        busy_pub = rospy.Publisher('/mower_smach/busy', Bool, queue_size=1, latch=True)
        prog_pub = rospy.Publisher('/mower_smach/active_program', String, queue_size=1,
                                   latch=True)
        busy_pub.publish(Bool(False))
        prog_pub.publish(String(' '))
        time.sleep(1.5)
        with lk:
            m0 = len(mower_stops)
        drive['ds4'] = 0.4
        o, c, s, t = window(1.0)
        drive['ds4'] = 0.0
        with lk:
            m1 = len(mower_stops)
        ok('idle mower: takeover still cancels goals and stops dock/smach_goal',
           len(c) == 3 and len(s) == 2 and len(t) == 1, '%d %d %d' % (len(c), len(s), len(t)))
        ok('idle mower: no /mower_smach/stop', m1 == m0, m1 - m0)
        time.sleep(3.0)
        print('9b mowing program running -> /mower_smach/stop at once')
        busy_pub.publish(Bool(True))
        prog_pub.publish(String('garden'))
        time.sleep(1.0)
        drive['ds4'] = 0.4
        o, c, s, t = window(1.0)
        drive['ds4'] = 0.0
        with lk:
            m2 = len(mower_stops)
        ok('program running: 3 stops incl. /mower_smach/stop', len(s) == 3 and m2 == m1 + 1,
           '%d %d' % (len(s), m2 - m1))
        time.sleep(3.0)

        print('10 drive_primitive vs nav')
        drive['nav'] = 0.3
        time.sleep(1.0)
        drive['prim'] = 0.12
        time.sleep(0.3)
        o, c, s, t = window(2.0)
        vals = set(round(m.linear.x, 3) for _, m in o)
        ok('primitive beats nav (only 0.12)', vals == {0.12}, vals)
        ok('status primitive', status[-1]['active'] == 'primitive', status[-1])
        ok('the dropped nav command is listed', bool(status[-1]['nav_blocked']), status[-1])
        ok('no takeover, no cancel for software priority', not t and not c, (len(t), len(c)))
        drive['prim'] = 0.0
        time.sleep(0.2)
        o, c, s, t = window(1.5)
        vals = set(round(m.linear.x, 3) for _, m in o)
        ok('primitive zeros hold nav down', vals == {0.0}, vals)
        drive['prim'] = None
        t_q = time.monotonic()
        o, c, s, t = window(2.0)
        late = [round(m.linear.x, 3) for ts, m in o if ts - t_q > 0.7]
        early = [round(m.linear.x, 3) for ts, m in o if ts - t_q < 0.4]
        ok('nav drives again after 0.5 s of primitive silence',
           len(late) >= 8 and set(late) == {0.3}, late[:5])
        ok('nav still dropped right after primitive stopped', 0.3 not in early, early)
        drive['prim'] = 0.12
        time.sleep(0.5)
        with lk:
            p0 = len(prim_stops)
        drive['web'] = 0.2
        o, c, s, t = window(1.5)
        vals = set(round(m.linear.x, 3) for _, m in o[3:])
        ok('teleop beats primitive', vals == {0.2}, vals)
        ok('takeover teleop', [m.data for _, m in t] == ['teleop'], [m.data for _, m in t])
        with lk:
            p1 = len(prim_stops)
        ok('takeover sends /drive_primitive/stop', p1 == p0 + 1 and prim_stops[-1][1].data,
           p1 - p0)
        drive['web'] = None
        drive['prim'] = None
        drive['nav'] = None
        time.sleep(3.0)

        print('8  mux stopped -> nothing reaches the base')
        mux.terminate()
        mux.wait(5)
        mux = None
        drive['nav'] = 0.3
        o, _c, _s, _t = window(1.5)
        ok('no message on the base input', not o, len(o))
        drive['nav'] = None
        stop_ev.set()
    finally:
        if mux is not None:
            mux.terminate()
        core.terminate()
        try:
            core.wait(10)
        except Exception:
            core.kill()
    print()
    print('%d FAILURE(S): %s' % (len(FAIL), FAIL) if FAIL else 'all cmd_vel_mux checks passed')
    return 1 if FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
