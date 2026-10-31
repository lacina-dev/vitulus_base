"""Podvržený moteus pro test_power_on: čtyři kola bez hardwaru."""
import asyncio
import enum
import json
import os
import types


class Register(enum.Enum):
    MODE = 0
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3
    VOLTAGE = 4
    TEMPERATURE = 5
    FAULT = 6


def _ctrl():
    """Test script's wishes, read at every call (JSON file in STUB_MOTEUS_CTRL)."""
    try:
        with open(os.environ['STUB_MOTEUS_CTRL']) as f:
            return json.load(f)
    except (KeyError, ValueError, OSError):
        return {}


def _note(key, value):
    path = os.environ.get('STUB_MOTEUS_NOTES')
    if path:
        with open(path, 'a') as f:
            f.write(json.dumps([key, value]) + '\n')


class _State(object):
    def __init__(self, fault=0, voltage=24.0):
        self.values = {Register.MODE: 10, Register.POSITION: 0.0, Register.VELOCITY: 0.0,
                       Register.TORQUE: 0.0, Register.VOLTAGE: voltage,
                       Register.TEMPERATURE: 25.0, Register.FAULT: fault}


class Fdcanusb(object):
    """A real port's shape: _close_transport removes the reader, then closes."""
    opens = 0

    def __init__(self, *args, **kwargs):
        Fdcanusb.opens += 1
        _note('open', Fdcanusb.opens)
        r, w = os.pipe()
        os.close(w)
        self._serial = types.SimpleNamespace(
            fd=r, serial=types.SimpleNamespace(close=lambda: os.close(r)))


class Controller(object):
    watchdog = False
    fired = False

    def __init__(self, *args, **kwargs):
        pass

    def make_set_output_nearest(self, **kwargs):
        return None

    async def set_stop(self):
        Controller.watchdog = False
        return None

    async def set_position(self, **kwargs):
        _note('velocity', kwargs.get('velocity'))
        if _ctrl().get('watchdog') and not Controller.fired:
            Controller.fired = Controller.watchdog = True
        return None

    async def query(self):
        c = _ctrl()
        # Never answered, like moteus on a dead bus: no timeout of its own.
        if c.get('hang_all') or Fdcanusb.opens <= c.get('hang_until_open', 0):
            await asyncio.get_event_loop().create_future()
        state = _State(fault=c.get('fault', 0), voltage=c.get('voltage', 24.0))
        if Controller.watchdog:
            _note('watchdog', 1)
            state.values[Register.MODE] = 11
        return state
