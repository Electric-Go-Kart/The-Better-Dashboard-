"""Steering controllers for the lane-drive CARLA harness."""
import math


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, value))


class PurePursuit:
    def __init__(self, gain=1.0):
        self.gain = gain

    def update(self, offset, heading, lookahead, curvature, speed, wheelbase, max_steer, dt):
        target = offset + math.tan(heading) * lookahead + 0.5 * curvature * lookahead**2
        alpha = math.atan2(target, lookahead)
        angle = self.gain * math.atan2(
            2 * wheelbase * math.sin(alpha), math.hypot(lookahead, target)
        )
        return clamp(angle / max_steer)


class Stanley:
    def __init__(self, gain=1.0, softening=0.5):
        self.gain = gain
        self.softening = softening

    def update(self, offset, heading, lookahead, curvature, speed, wheelbase, max_steer, dt):
        angle = heading + math.atan2(self.gain * offset, max(speed, 0.0) + self.softening)
        angle += math.atan(wheelbase * curvature)
        return clamp(angle / max_steer)


class PID:
    def __init__(self, kp=0.7, ki=0.03, kd=0.15):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.previous = None

    def reset(self):
        self.integral = 0.0
        self.previous = None

    def update(self, offset, heading, lookahead, curvature, speed, wheelbase, max_steer, dt):
        error = offset + math.tan(heading) * lookahead + 0.5 * curvature * lookahead**2
        derivative = 0.0 if self.previous is None or dt <= 0 else (error - self.previous) / dt
        self.integral = clamp(self.integral + error * dt, -2.0, 2.0)
        self.previous = error
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        angle += math.atan(wheelbase * curvature)
        return clamp(angle / max_steer)


class MPC:
    """Short-horizon kinematic bicycle MPC using a bounded beam search."""

    def __init__(self, horizon=8, step_s=0.2, beam_width=24):
        self.horizon = horizon
        self.step_s = step_s
        self.beam_width = beam_width
        self.actions = (-1.0, -0.5, 0.0, 0.5, 1.0)

    def update(self, offset, heading, lookahead, curvature, speed, wheelbase, max_steer, dt):
        del lookahead, dt
        v = max(0.0, speed)
        # State is kart lateral position and yaw relative to the lane centreline.
        beam = [(0.0, -offset, -heading, 0.0, None)]
        for _ in range(self.horizon):
            expanded = []
            for cost, lateral, yaw, previous_steer, first_steer in beam:
                for steer in self.actions:
                    steer_angle = steer * max_steer
                    next_lateral = lateral + v * math.sin(yaw) * self.step_s
                    next_yaw = yaw + (
                        v / max(wheelbase, 0.1) * math.tan(steer_angle) - v * curvature
                    ) * self.step_s
                    next_cost = cost + (
                        2.0 * next_lateral**2
                        + 0.8 * next_yaw**2
                        + 0.02 * steer**2
                        + 0.08 * (steer - previous_steer) ** 2
                    )
                    expanded.append((
                        next_cost, next_lateral, next_yaw, steer,
                        steer if first_steer is None else first_steer,
                    ))
            beam = sorted(expanded, key=lambda state: state[0])[:self.beam_width]
        return clamp(min(beam, key=lambda state: state[0])[4])


def make_controller(name, args):
    if name == "pure-pursuit":
        return PurePursuit(args.kp)
    if name == "stanley":
        return Stanley(args.stanley_k, args.stanley_softening)
    if name == "pid":
        return PID(args.pid_kp, args.pid_ki, args.pid_kd)
    if name == "mpc":
        return MPC(args.mpc_horizon, args.mpc_step, args.mpc_beam)
    raise ValueError(f"unknown lane controller: {name}")
