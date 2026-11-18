from fanatic.jet import JetModel
from fanatic.fan import Fan
from fanatic.interfaces import Observation, FanCommand
from fanatic.pid import PID
import numpy as np

STANDOFF = 0.35

class Controller:
    def __init__(self, jet_model, ball_radius: float, mass: float = 0.05):
        self.jet_model = jet_model
        self.ball_radius = ball_radius
        self.mass = mass
        self.pids = [PID(kp=10.0, ki=2.0, kd=6.0, out_min=-15.0, out_max=15.0) for _ in range(3)]
        
    def update(self, obs, dt):
        error=obs.target - obs.ball_position
        accel = np.array([pid.update(error[i], dt) for i, pid in enumerate(self.pids)])

        force = self.mass*(accel - obs.gravity)
        force_mag = np.linalg.norm(force)
        if force_mag < 1e-9:
            direction = np.array([0.0, 1.0, 0.0])   # nothing needed: just point up
        else:
            direction = force / force_mag
            direction = force/np.linalg.norm(force)

        tilt = np.arccos(np.clip(direction[1], -1, 1))
        turn = np.arctan2(direction[2], direction[0])
        
        fan_target = obs.ball_position - STANDOFF * direction

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