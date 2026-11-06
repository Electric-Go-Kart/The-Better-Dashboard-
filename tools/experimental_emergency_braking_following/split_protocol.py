"""Helpers for the experimental CARLA/Pi split; no vehicle I/O."""
import math
import time
import uuid


class CommandGate:
    """Accept only a new command tied to a recent locally sent ego sample."""

    def __init__(self, timeout_s, clock=time.monotonic):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout must be positive and finite")
        self.timeout_s = timeout_s
        self.clock = clock
        self.session = uuid.uuid4().hex
        self.sent = {}
        self.last_command_seq = -1
        self.instance = None
        self.retired_instances = set()
        self.last_fresh = None
        self.last_ego_time = None
        self.command = None

    def record_ego(self, seq):
        now = self.clock()
        self.sent[seq] = now
        self.sent = {s: t for s, t in self.sent.items() if now - t <= self.timeout_s}

    def accept(self, message):
        now = self.clock()
        try:
            if not isinstance(message, dict) or message.get("session") != self.session:
                return False
            seq, ego_seq, instance = message["seq"], message["ego_seq"], message["instance"]
            if (not isinstance(instance, str) or not instance or instance in self.retired_instances
                    or type(seq) is not int or seq < 0 or type(ego_seq) is not int
                    or (instance == self.instance and seq <= self.last_command_seq)):
                return False
            if ego_seq not in self.sent or not 0 <= now - self.sent[ego_seq] <= self.timeout_s:
                return False
            values = [message[k] for k in ("steer", "throttle", "brake")]
            if any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
                return False
            steer, throttle, brake = values
            if not (-1 <= steer <= 1 and 0 <= throttle <= 1 and 0 <= brake <= 1):
                return False
        except (KeyError, TypeError):
            return False
        if instance != self.instance:
            if self.instance is not None:
                self.retired_instances.add(self.instance)
            self.instance = instance
        self.last_command_seq = seq
        self.last_fresh = now
        self.last_ego_time = self.sent[ego_seq]
        self.command = message
        return True

    def fresh(self):
        return (self.last_ego_time is not None
                and 0 <= self.clock() - self.last_ego_time <= self.timeout_s)


class ControlMode:
    """Choose between CARLA autopilot and fresh remote control; shadow never writes."""

    def __init__(self, mode, hero, traffic_manager_port, make_control):
        self.mode = mode
        self.hero = hero
        self.port = traffic_manager_port
        self.make_control = make_control
        self.autopilot = False

    def update(self, gate):
        if self.mode == "shadow":
            return
        if gate.fresh():
            if self.autopilot:
                self.hero.set_autopilot(False, self.port)
                self.autopilot = False
                print("Fresh Pi commands: resumed simulated control", flush=True)
            self.hero.apply_control(self.make_control(**{k: gate.command[k] for k in ("steer", "throttle", "brake")}))
        elif not self.autopilot:
            self.hero.set_autopilot(True, self.port)
            self.autopilot = True
            print("Pi command timeout: CARLA autopilot enabled", flush=True)

    def close(self):
        if self.mode == "simulated" and not self.autopilot:
            self.hero.set_autopilot(True, self.port)
            self.autopilot = True
