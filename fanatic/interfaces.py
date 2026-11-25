from dataclasses import dataclass
import numpy as np


@dataclass
class FanCommand:
    target_position: np.ndarray
    tilt: float  # radians from straight up (+y)
    turn: float  # radians from +x, toward +z
    airflow: float  # nozzle speed, m/s

@dataclass
class Observation:
    ball_position: np.ndarray
    ball_velocity: np.ndarray
    held: bool
    gravity: np.ndarray
    target: np.ndarray
    fan_position: np.ndarray
    fan_direction: np.ndarray
    fan_airflow: float
    in_contact: bool
    view_direction: np.ndarray