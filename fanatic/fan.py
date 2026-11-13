import numpy as np
from fanatic.interfaces import FanCommand


def _cap_vector(v: np.ndarray, max_len: float) -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm > max_len and norm > 0:
        return v * (max_len / norm)
    return v


class Fan:
    def __init__(
        self,
        position: np.ndarray = None,
        velocity: np.ndarray = None,
        tilt: float = 0.0,
        turn: float = 0.0,
        airflow: float = 0.0,
        max_speed: float = 10.0,
        max_accel: float = 5.0,
        airflow_min: float = 0.0,
        airflow_max: float = 20.0,
    ):
        self.position = (
            np.array(position, dtype=float) if position is not None else np.zeros(3)
        )
        self.velocity = (
            np.array(velocity, dtype=float) if velocity is not None else np.zeros(3)
        )
        self.tilt = tilt
        self.turn = turn
        self.airflow = airflow

        self.max_speed = max_speed
        self.max_accel = max_accel
        self.airflow_min = airflow_min
        self.airflow_max = airflow_max

    def direction(self) -> np.ndarray:
        return np.array(
            [
                np.sin(self.tilt) * np.cos(self.turn),
                np.cos(self.tilt),
                np.sin(self.tilt) * np.sin(self.turn),
            ]
        )

    def apply(self, command: FanCommand, dt: float):
        offset = np.asarray(command.target_position) - self.position
        distance = np.linalg.norm(offset)
        if distance > 0:
            speed = min(
                self.max_speed, 4.0 * distance, np.sqrt(2 * self.max_accel * distance)
            )
            desired = offset / distance * speed
        else:
            desired = np.zeros(3)

        accel_change = desired - self.velocity
        max_change = self.max_accel * dt
        accel_change = _cap_vector(accel_change, max_change)
        self.velocity += accel_change

        self.position += self.velocity * dt

        self.tilt = command.tilt
        self.turn = command.turn
        self.airflow = float(
            np.clip(command.airflow, self.airflow_min, self.airflow_max)
        )
