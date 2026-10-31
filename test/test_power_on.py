#!/usr/bin/env python3
"""Zapnutí motorů proti power modulu, který příkaz ztratí nebo neodpoví.

Mise #262 (31. 10. 2026): zpráva ON se ztratila, base_controller čekal navždy
a přestal publikovat.  Test spouští SKUTEČNÝ uzel na soukromém roscore
(SIM_PORT, výchozí 11794) s podvrženým moteusem a modulem ve stavu CHARGED.
"""
import json
import os
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, '..', 'nodes', 'base_controller')
PORT = os.environ.get('SIM_PORT', '11794')
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%s' % PORT
os.environ['ROS_HOSTNAME'] = '127.0.0.1'
os.environ['ROS_HOME'] = tempfile.mkdtemp(prefix='power_on_')

import rospy  # noqa: E402
from std_msgs.msg import Bool, Float32  # noqa: E402
from vitulus_msgs.msg import Power_status  # noqa: E402

FAILS = []


def ok(label, cond, detail=''):
    if not cond:
        FAILS.append(label)
    print('  %-66s %s%s' % (label, 'ok' if cond else 'FAIL',
                            '' if cond else '  (%s)' % (detail,)))


class Module(object):
    """Power modul: zahodí prvních `lose` ON, pak zapíná hned jako firmware."""

    def __init__(self):
        self.lose = 0
        self.dead = False
        self.on = False
        self.ons = []
        self.offs = 0
        self.off_at = 0.0
        self.pub = rospy.Publisher('/pm/power_status', Power_status, queue_size=1)
        rospy.Subscriber('/pm/set_motor_switch', Bool, self.cmd, queue_size=20)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def cmd(self, msg):
        if not msg.data:
            self.offs += 1
            if self.on:
                self.off_at = time.time()
            self.on = False
            return
        self.ons.append(time.time())
        if self.dead:
            return
        if self.lose > 0:
            self.lose -= 1
            return
        self.on = True

    def tick(self, _event):
        self.pub.publish(Power_status(charger_status='CHARGED', supply_status='ONLINE',
                                      module_status='RUNNING', battery_capacity=100,
                                      motor_out_switch=self.on))


class State(object):
    def __init__(self):
        self.stamps = []
        self.value = None
        rospy.Subscriber('/base/motor_power_state', Bool, self.cb, queue_size=200)

    def cb(self, msg):
        self.stamps.append(time.time())
        self.value = msg.data

    def wait(self, want, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if self.value is want:
                return True
            time.sleep(0.02)
        return False

    def rate(self, seconds):
        start = len(self.stamps)
        time.sleep(seconds)
        return (len(self.stamps) - start) / seconds


def off_confirmed(module, timeout=2.0):
    """The state goes False first, the power module follows within one status."""
    end = time.time() + timeout
    while time.time() < end and module.on:
        time.sleep(0.02)
    return not module.on


def main():
    ctrl, notes = tempfile.mktemp(prefix='moteus_ctrl_'), tempfile.mktemp(prefix='moteus_notes_')
    env = dict(os.environ, STUB_MOTEUS_CTRL=ctrl, STUB_MOTEUS_NOTES=notes,
               PYTHONPATH=os.path.join(HERE, 'stub_moteus') + os.pathsep
               + os.environ.get('PYTHONPATH', ''))

    def drive(**wishes):
        with open(ctrl, 'w') as f:
            json.dump(wishes, f)

    def noted(key):
        if not os.path.exists(notes):
            return []
        with open(notes) as f:
            rows = [json.loads(line) for line in f]
        return [v for k, v in rows if k == key]

    core = subprocess.Popen(['roscore', '-p', PORT], env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    node = None
    try:
        time.sleep(3.0)
        rospy.init_node('power_on_test', disable_signals=True)
        module, state = Module(), State()
        power = rospy.Publisher('/base/motor_power', Bool, queue_size=5)
        torque = rospy.Publisher('/base/set_torque', Float32, queue_size=1, latch=True)
        torque.publish(Float32(20.0))
        node = subprocess.Popen([sys.executable, NODE, '_min_off_s:=0.6', '_boot_timeout_s:=1.5',
                                 '_boot_settle_s:=0.1'], env=env,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ok('uzel po startu publikuje stav', state.wait(False, 15.0), state.value)
        time.sleep(6.0)              # STALE_LATCH_IGNORE_S
        ok('50 Hz v klidu', state.rate(1.0) > 30)

        print('ztracený ON (mise #262)')
        module.lose = 1
        t0 = time.time()
        power.publish(Bool(True))
        up = state.wait(True, 5.0)
        took = time.time() - t0
        ok('druhý ON zapnul motory ze stavu CHARGED', up, module.ons)
        ok('… do 2 s', up and took < 2.5, took)
        ok('… ON poslán dvakrát', len(module.ons) == 2, module.ons)
        power.publish(Bool(False))
        ok('vypnutí potvrzeno', state.wait(False, 5.0) and off_confirmed(module))
        time.sleep(1.0)

        print('rychlé vypnutí a zapnutí')
        power.publish(Bool(True))
        ok('zapnuto', state.wait(True, 6.0), state.value)
        del module.ons[:]
        power.publish(Bool(False))
        time.sleep(0.05)
        power.publish(Bool(True))
        ok('vypnutí a pak zase zapnuto', state.wait(False, 3.0) and state.wait(True, 6.0), state.value)
        ok('ON až po min_off_s od vypnutí', module.ons and module.ons[0] - module.off_at >= 0.55,
           module.ons and module.ons[0] - module.off_at)
        print('série příkazů: platí poslední')
        for value in (False, True, False, True, False):
            power.publish(Bool(value))
            time.sleep(0.02)
        ok('skončí vypnuté', state.wait(False, 3.0))
        time.sleep(2.0)
        ok('… a modul zůstal vypnutý', not module.on)

        print('stará rychlost nesmí po zapnutí roztočit kola')
        vel = rospy.Publisher('/base/front_left_wheel_vel', Float32, queue_size=1)
        time.sleep(0.5)
        vel.publish(Float32(5.0))
        time.sleep(0.2)
        os.path.exists(notes) and os.remove(notes)
        power.publish(Bool(True))
        ok('zapnuto', state.wait(True, 6.0), state.value)
        time.sleep(0.3)
        ok('první příkazy nulové', noted('velocity') and not any(noted('velocity')), noted('velocity')[:4])

        print('zapnutí mimo /base/motor_power (panel, e-stop)')
        power.publish(Bool(False))
        ok('vypnuto', state.wait(False, 3.0))
        time.sleep(1.0)
        module.on = True             # ON that did not come through this node
        time.sleep(1.5)
        ok('samovolně sepnutý výstup se vypne', not module.on)
        pm = rospy.Publisher('/pm/set_motor_switch', Bool, queue_size=1)
        time.sleep(0.5)
        pm.publish(Bool(True))
        ok('ON z panelu spustí motory', state.wait(True, 6.0), state.value)
        pm.publish(Bool(False))
        ok('OFF z panelu je vypne', state.wait(False, 3.0) and off_confirmed(module))
        time.sleep(1.0)

        print('motory po zapnutí mlčí: celý cyklus vypnout-zapnout')
        opened = max(noted('open') or [0])
        drive(hang_until_open=opened + 4)
        offs = module.offs
        del module.ons[:]
        power.publish(Bool(True))
        ok('naběhly po dalším pokusu', state.wait(True, 12.0), state.value)
        ok('… po skutečném vypnutí v modulu', module.offs > offs, module.offs)
        power.publish(Bool(False))
        ok('vypnuto', state.wait(False, 3.0))
        drive()
        time.sleep(1.0)

        print('porucha v motoru se při startu smaže nebo start selže')
        drive(fault=33, hang_all=False)
        offs = module.offs
        power.publish(Bool(True))
        ok('s trvalou poruchou nezapne', not state.wait(True, 9.0), state.value)
        time.sleep(1.5)
        ok('po pokusech výstup vypnutý', not module.on, module.offs - offs)
        ok('… a stav False', state.value is False)
        ok('stav dál chodí', state.rate(1.0) > 30)
        drive()
        time.sleep(0.7)
        power.publish(Bool(True))
        ok('po odstranění poruchy nový pokyn zapne', state.wait(True, 6.0), state.value)

        print('watchdog drivu za běhu (23. 8. 2026: dvě kola v TIMEOUT)')
        os.path.exists(notes) and os.remove(notes)
        drive(watchdog=True)
        time.sleep(0.5)
        ok('watchdog nastal', noted('watchdog'))
        ok('… a stop ho odblokoval', state.wait(True, 2.0), state.value)
        drive()

        print('mlčící motory a OFF uprostřed startu')
        power.publish(Bool(False))
        ok('vypnuto', state.wait(False, 3.0))
        drive(hang_all=True)
        time.sleep(0.8)
        power.publish(Bool(True))
        time.sleep(0.8)
        power.publish(Bool(False))
        time.sleep(1.5)
        ok('OFF přebije start', not module.on and state.value is False, module.on)
        drive()
        time.sleep(0.7)

        print('modul neodpovídá')
        module.dead = True
        del module.ons[:]
        offs = module.offs
        t0 = time.time()
        power.publish(Bool(True))
        rate = state.rate(4.0)
        ok('během čekání stav chodí dál (mise #262: nic)', rate > 30, rate)
        ok('… a říká False', state.value is False)
        ok('… ON se opakuje', len(module.ons) >= 3, module.ons)
        time.sleep(7.5)
        sent = len(module.ons)
        ok('po 10 s konec opakování', 8 <= sent <= 12, sent)
        ok('… s pojistkou OFF', module.offs > offs, module.offs)
        time.sleep(2.0)
        ok('… už se nic neposílá', len(module.ons) == sent, len(module.ons))

        print('další výjezd po vzdání')
        module.dead = False
        power.publish(Bool(True))
        ok('nový pokyn zapne bez restartu uzlu', state.wait(True, 3.0), state.value)
        power.publish(Bool(False))
        ok('vypnutí potvrzeno', state.wait(False, 5.0))
        time.sleep(1.0)

        print('OFF během zapínání')
        module.dead = True
        del module.ons[:]
        offs = module.offs
        power.publish(Bool(True))
        time.sleep(1.5)
        power.publish(Bool(False))
        time.sleep(0.5)
        sent = len(module.ons)
        time.sleep(2.5)
        ok('OFF zastaví opakování ON', len(module.ons) == sent and sent >= 1, module.ons)
        ok('… a jde OFF do modulu', module.offs > offs, module.offs)
        module.dead = False
        ok('motory zůstaly vypnuté', state.value is False and not module.on)
        print('uzel běžel celou dobu' if node.poll() is None else 'UZEL SPADL')
        ok('uzel nespadl', node.poll() is None)
    finally:
        for proc in (node, core):
            if proc is not None:
                proc.terminate()
        for proc in (node, core):
            if proc is not None:
                try:
                    proc.wait(10)
                except subprocess.TimeoutExpired:
                    proc.kill()
    print('%d selhání' % len(FAILS) if FAILS else 'vše prošlo')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
