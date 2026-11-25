import numpy as np
from fanatic.interfaces import FanCommand


def _cap_vector(v: np.ndarray, max_len: float) -> np.ndarray:
    """Shorten v to max_len if it is longer, keeping its direction."""
    norm = np.linalg.norm(v)
    if norm > max_len and norm > 0:
        return v * (max_len / norm)
    return v


def _angles_to_dir(tilt: float, turn: float) -> np.ndarray:
    """Unit vector from tilt (radians from +y) and turn (radians from +x toward +z)."""
    return np.array([
        np.sin(tilt) * np.cos(turn),
        np.cos(tilt),
        np.sin(tilt) * np.sin(turn),
    ])


def _rotate_toward(a: np.ndarray, b: np.ndarray, step: float) -> np.ndarray:
    """Rotate unit vector a toward unit vector b by `step` radians."""
    perp = b - np.dot(a, b) * a
    n = np.linalg.norm(perp)
    if n < 1e-9:  # a and b parallel or opposite: pick any perpendicular
        perp = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(perp) < 1e-9:
            perp = np.cross(a, [0.0, 0.0, 1.0])
        n = np.linalg.norm(perp)
    perp = perp / n
    return np.cos(step) * a + np.sin(step) * perp


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
        max_ang_speed: float = 6.0,
        max_ang_accel: float = 20.0,
        airflow_min: float = 0.0,
        airflow_max: float = 40.0,
    ):
        self.position = (
            np.array(position, dtype=float) if position is not None else np.zeros(3)
        )
        self.velocity = (
            np.array(velocity, dtype=float) if velocity is not None else np.zeros(3)
        )
        self.dir = _angles_to_dir(tilt, turn)
        self.ang_speed = 0.0
        self.airflow = airflow

        self.max_speed = max_speed
        self.max_accel = max_accel
        self.max_ang_speed = max_ang_speed
        self.max_ang_accel = max_ang_accel
        self.airflow_min = airflow_min
        self.airflow_max = airflow_max

    @property
    def tilt(self) -> float:
        return np.arccos(np.clip(self.dir[1], -1.0, 1.0))

    @property
    def turn(self) -> float:
        return np.arctan2(self.dir[2], self.dir[0])

    def direction(self) -> np.ndarray:
        return self.dir

    def apply(self, command: FanCommand, dt: float):
        target = np.asarray(command.target_position, dtype=float)
        offset = target - self.position
        distance = np.linalg.norm(offset)
        if distance > 0:
            speed = min(
                self.max_speed,
                4.0 * distance,
                np.sqrt(2 * self.max_accel * distance),
            )
            desired = offset / distance * speed
        else:
            desired = np.zeros(3)

        change = _cap_vector(desired - self.velocity, self.max_accel * dt)
        self.velocity += change
        self.position += self.velocity * dt

        # Rotate toward the commanded aim within angular limits
        target_dir = _angles_to_dir(command.tilt, command.turn)
        angle = np.arccos(np.clip(np.dot(self.dir, target_dir), -1.0, 1.0))

        desired_ang_speed = min(
            self.max_ang_speed,
            4.0 * angle,
            np.sqrt(2 * self.max_ang_accel * angle),
        )
        ang_change = desired_ang_speed - self.ang_speed
        max_ang_change = self.max_ang_accel * dt
        if abs(ang_change) > max_ang_change:
            ang_change = np.sign(ang_change) * max_ang_change
        self.ang_speed += ang_change

        step = min(self.ang_speed * dt, angle)
        if angle > 1e-9 and step > 0:
            self.dir = _rotate_toward(self.dir, target_dir, step)
            self.dir = self.dir / np.linalg.norm(self.dir)

        # Airflow within its limits
        self.airflow = float(np.clip(command.airflow, self.airflow_min, self.airflow_max))