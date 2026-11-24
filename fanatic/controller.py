from fanatic.interfaces import Observation, FanCommand
from fanatic.pid import PID
import numpy as np
from fanatic.estimator import MassEstimator

STANDOFF = 0.35
LEAD_TIME = 0.15   # seconds to look ahead



class Controller:
    def __init__(self, jet_model, ball_radius: float):
        self.jet_model = jet_model
        self.ball_radius = ball_radius
        self.estimator = MassEstimator()
        self.prev_obs = None
        self.pids = [PID(kp=20.0, ki=0.0, kd=10.0, out_min=-15.0, out_max=15.0) for _ in range(3)]
        
    def update(self, obs, dt):
        if self.prev_obs is not None and not obs.held and not self.prev_obs.held and not obs.in_contact and not self.prev_obs.in_contact:
            measured_accel = (obs.ball_velocity - self.prev_obs.ball_velocity) / dt
            measured_force = self.jet_model.force_on_ball(
                self.prev_obs.ball_position, self.prev_obs.ball_velocity, self.ball_radius,
                obs.fan_position, obs.fan_direction, obs.fan_airflow)
            self.estimator.update(measured_accel - obs.gravity, measured_force)
        self.prev_obs = obs
        error=obs.target - obs.ball_position
        accel = np.array([pid.update(error[i], dt) for i, pid in enumerate(self.pids)])

        force = self.estimator.mass*(accel - obs.gravity)
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

        return FanCommand(
            target_position=fan_target,
            tilt=float(tilt),
            turn=float(turn),
            airflow=float(airflow)
        )

    def reset(self):
        for pid in self.pids:
            pid.reset()
        self.prev_obs = None