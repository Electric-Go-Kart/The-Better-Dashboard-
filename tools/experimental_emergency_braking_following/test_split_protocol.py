import unittest

from split_protocol import CommandGate, ControlMode


class CommandGateTest(unittest.TestCase):
    def test_freshness_echo_order_and_range(self):
        now = [10.0]
        gate = CommandGate(0.5, lambda: now[0])
        gate.record_ego(5)
        command = {"session": gate.session, "instance": "pi-1", "seq": 0, "ego_seq": 5,
                   "steer": 0.1, "throttle": 0.2, "brake": 0.0}
        self.assertTrue(gate.accept(command))
        self.assertTrue(gate.fresh())
        self.assertFalse(gate.accept(command))
        self.assertFalse(gate.accept({**command, "seq": 1, "brake": float("nan")}))
        self.assertFalse(gate.accept({**command, "seq": 1, "session": "old"}))
        now[0] += 0.6
        self.assertFalse(gate.fresh())
        self.assertFalse(gate.accept({**command, "seq": 1}))
        gate.record_ego(6)
        self.assertTrue(gate.accept({**command, "seq": 1, "ego_seq": 6}))
        self.assertTrue(gate.fresh())

    def test_echo_age_not_receive_age(self):
        now = [10.0]
        gate = CommandGate(0.5, lambda: now[0])
        gate.record_ego(1)
        now[0] = 10.49
        command = {"session": gate.session, "instance": "pi-1", "seq": 0, "ego_seq": 1,
                   "steer": 0, "throttle": 0, "brake": 0}
        self.assertTrue(gate.accept(command))
        now[0] = 10.51
        self.assertFalse(gate.fresh())

    def test_control_transitions_and_shadow(self):
        class Hero:
            def __init__(self):
                self.calls = []
            def set_autopilot(self, enabled, port):
                self.calls.append(("autopilot", enabled, port))
            def apply_control(self, control):
                self.calls.append(("control", control))

        now = [0.0]
        gate = CommandGate(0.5, lambda: now[0])
        hero = Hero()
        shadow = ControlMode("shadow", hero, 8000, dict)
        shadow.update(gate)
        shadow.close()
        self.assertEqual(hero.calls, [])
        sim = ControlMode("simulated", hero, 8000, dict)
        sim.update(gate)
        gate.record_ego(1)
        gate.accept({"session": gate.session, "instance": "pi-1", "seq": 1, "ego_seq": 1,
                     "steer": 0.1, "throttle": 0.2, "brake": 0.0})
        sim.update(gate)
        now[0] = 0.6
        sim.update(gate)
        self.assertEqual(hero.calls, [("autopilot", True, 8000), ("autopilot", False, 8000),
                                     ("control", {"steer": 0.1, "throttle": 0.2, "brake": 0.0}),
                                     ("autopilot", True, 8000)])
        shadow.update(gate)
        self.assertEqual(len(hero.calls), 4)
        gate.record_ego(2)
        self.assertTrue(gate.accept({"session": gate.session, "instance": "pi-1", "seq": 2, "ego_seq": 2,
                                     "steer": 0.0, "throttle": 0.1, "brake": 0.0}))
        sim.update(gate)
        self.assertEqual(hero.calls[-2], ("autopilot", False, 8000))
        self.assertEqual(hero.calls[-1][0], "control")
        sim.close()
        self.assertEqual(hero.calls[-1], ("autopilot", True, 8000))

    def test_planner_restart_and_delayed_old_instance(self):
        gate = CommandGate(0.5, lambda: 10.0)
        gate.record_ego(1)
        command = {"session": gate.session, "instance": "old", "seq": 100, "ego_seq": 1,
                   "steer": 0, "throttle": 0, "brake": 0}
        self.assertTrue(gate.accept(command))
        self.assertTrue(gate.accept({**command, "instance": "new", "seq": 1}))
        self.assertFalse(gate.accept({**command, "seq": 101}))


if __name__ == "__main__":
    unittest.main()
