from dataclasses import dataclass
import numpy as np


@dataclass
class FanCommand:
    target_position: np.ndarray
    tilt: float  # radians from straight up (+y)
    turn: float  # radians from +x, toward +z
    airflow: float  # nozzle speed, m/s
