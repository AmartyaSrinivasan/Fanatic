from fanatic.interfaces import Observation, FanCommand
from fanatic.pid import PID
import numpy as np
from fanatic.estimator import MassEstimator
import random

STANDOFF = 0.35
LEAD_TIME = 0.15
ORBIT_RADIUS = 0.6
ORBIT_SPEED = 1.5
DODGE_SPEED = 3.0
DODGE_SPRING = 20.0
DODGE_DAMPING = 5.0
GRUMPY_AFTER = 30.0
CLEARANCE = 0.25
seed = 0


def _plane_basis(normal):
    n = normal / np.linalg.norm(normal)
    helper = np.array([0.0, 1.0, 0.0])
    if abs(np.dot(n, helper)) > 0.99:
        helper = np.array([1.0, 0.0, 0.0])
    u = np.cross(n, helper)
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


def _safe_target(fan_pos, target, ball_pos):
    away = fan_pos - ball_pos
    if np.linalg.norm(away) < CLEARANCE:
        if np.linalg.norm(away) < 1e-9:
            away = np.array([1.0, 0.0, 0.0])
        return ball_pos + 1.5 * CLEARANCE * away / np.linalg.norm(away)

    path = target - fan_pos
    length_sq = np.dot(path, path)
    if length_sq < 1e-12:
        return target
    s = np.clip(np.dot(ball_pos - fan_pos, path) / length_sq, 0.0, 1.0)
    closest = fan_pos + s * path
    gap = closest - ball_pos
    if np.linalg.norm(gap) >= CLEARANCE:
        return target

    if np.linalg.norm(gap) < 1e-9:
        gap = np.cross(path, [0.0, 1.0, 0.0])
        if np.linalg.norm(gap) < 1e-9:
            gap = np.cross(path, [1.0, 0.0, 0.0])
    return ball_pos + 1.5 * CLEARANCE * gap / np.linalg.norm(gap)


class Controller:
    def __init__(self, jet_model, ball_radius: float):
        self.jet_model = jet_model
        self.ball_radius = ball_radius
        self.estimator = MassEstimator()
        self.prev_obs = None
        self.pids = [PID(kp=20.0, ki=0.0, kd=10.0, out_min=-15.0, out_max=15.0) for _ in range(3)]
        self.held_time = 0.0

        self.rng = random.Random(seed)
        self.orbit_angle = None
        self.dodge_offset = np.zeros(3)
        self.dodge_velocity = np.zeros(3)
        self.next_dodge = self.rng.uniform(1.5, 4.0)

    @property
    def is_grumpy(self):
        return self.held_time > GRUMPY_AFTER

    def update(self, obs, dt):
        if obs.held:
            self.reset()

            self.held_time += dt
            if self.is_grumpy:
                toward_user = -obs.view_direction / np.linalg.norm(obs.view_direction)
                tilt = np.arccos(np.clip(toward_user[1], -1.0, 1.0))
                turn = np.arctan2(toward_user[2], toward_user[0])
                grumpy_target = obs.ball_position + ORBIT_RADIUS * toward_user
                grumpy_target = _safe_target(obs.fan_position, grumpy_target, obs.ball_position)
                return FanCommand(grumpy_target, tilt=tilt, turn=turn, airflow=30.0)

            u, v = _plane_basis(obs.view_direction)
            if self.orbit_angle is None:
                offset = obs.fan_position - obs.ball_position
                self.orbit_angle = np.arctan2(np.dot(offset, v), np.dot(offset, u))

            self.orbit_angle += ORBIT_SPEED * dt
            outward = np.cos(self.orbit_angle) * u + np.sin(self.orbit_angle) * v

            self.next_dodge -= dt
            if self.next_dodge <= 0:
                kick = np.array([self.rng.gauss(0, 1) for _ in range(3)])
                kick = kick / np.linalg.norm(kick)
                if np.dot(kick, outward) < 0:      # never dodge toward the ball
                    kick = -kick
                self.dodge_velocity = DODGE_SPEED * kick
                self.next_dodge = self.rng.uniform(1.5, 4.0)

            spring = -DODGE_SPRING * self.dodge_offset - DODGE_DAMPING * self.dodge_velocity
            self.dodge_velocity += spring * dt
            self.dodge_offset += self.dodge_velocity * dt

            facing = -outward
            tilt = np.arccos(np.clip(facing[1], -1.0, 1.0))
            turn = np.arctan2(facing[2], facing[0])
            orbit_target = obs.ball_position + ORBIT_RADIUS * outward + self.dodge_offset
            orbit_target = _safe_target(obs.fan_position, orbit_target, obs.ball_position)
            return FanCommand(orbit_target, tilt=tilt, turn=turn, airflow=0.0)

        self.orbit_angle = None
        self.dodge_offset = np.zeros(3)
        self.dodge_velocity = np.zeros(3)
        self.held_time = 0.0

        if (self.prev_obs is not None and not obs.held and not self.prev_obs.held
                and not obs.in_contact and not self.prev_obs.in_contact):
            measured_accel = (obs.ball_velocity - self.prev_obs.ball_velocity) / dt
            measured_force = self.jet_model.force_on_ball(
                self.prev_obs.ball_position, self.prev_obs.ball_velocity, self.ball_radius,
                obs.fan_position, obs.fan_direction, obs.fan_airflow)
            self.estimator.update(measured_accel - obs.gravity, measured_force)
        self.prev_obs = obs

        error = obs.target - obs.ball_position
        accel = np.array([pid.update(error[i], dt) for i, pid in enumerate(self.pids)])

        force = self.estimator.mass * (accel - obs.gravity)
        force_mag = np.linalg.norm(force)
        if force_mag < 1e-9:
            direction = np.array([0.0, 1.0, 0.0])   # nothing needed: just point up
        else:
            direction = force / force_mag

        tilt = np.arccos(np.clip(direction[1], -1, 1))
        turn = np.arctan2(direction[2], direction[0])

        predicted = (obs.ball_position + obs.ball_velocity * LEAD_TIME
                     + 0.5 * obs.gravity * LEAD_TIME ** 2)
        fan_target = predicted - STANDOFF * direction

        area = np.pi * (self.ball_radius ** 2)
        rho = self.jet_model.air_density
        cd = self.jet_model.drag_coeff

        if force_mag > 0:
            u = np.sqrt(2 * force_mag / (rho * cd * area))
        else:
            u = 0.0

        z = np.linalg.norm(obs.ball_position - obs.fan_position)
        k_decay = self.jet_model.k_decay
        nozzle_d = self.jet_model.nozzle_diameter

        attenuation = min(1.0, k_decay * nozzle_d / z) if z > 1e-6 else 1.0
        airflow = u / attenuation if attenuation > 1e-6 else u

        fan_target = _safe_target(obs.fan_position, fan_target, obs.ball_position)

        return FanCommand(
            target_position=fan_target,
            tilt=float(tilt),
            turn=float(turn),
            airflow=float(airflow),
        )

    def reset(self):
        for pid in self.pids:
            pid.reset()
        self.prev_obs = None