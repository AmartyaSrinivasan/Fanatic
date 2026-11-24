from dataclasses import dataclass
import numpy as np


@dataclass
class Ball:
    position: np.ndarray
    velocity: np.ndarray
    radius: float = 0.1
    mass: float = 0.05


class World:
    def __init__(self, ball, box_min, box_max, gravity, restitution=0.8, fan=None, jet=None):
        self.ball = ball
        self.box_min = box_min
        self.box_max = box_max
        self.gravity = gravity
        self.restitution = restitution
        self.held = False
        self.fan = fan
        self.jet = jet

    def step(self, dt):
        if self.held:
            return

        accel = self.gravity.copy()
        if self.fan is not None and self.jet is not None:
            force = self.jet.force_on_ball(
                self.ball.position, self.ball.velocity, self.ball.radius,
                self.fan.position, self.fan.direction(), self.fan.airflow)
            accel += force / self.ball.mass
        self.ball.velocity += accel * dt
        self.ball.position += self.ball.velocity * dt

        for i in range(3):
            if self.ball.velocity[i] < 0:
                if self.ball.position[i] - self.ball.radius < self.box_min[i]:
                    self.ball.position[i] = self.box_min[i] + self.ball.radius
                    self.ball.velocity[i] = -self.restitution * self.ball.velocity[i]
            if self.ball.velocity[i] > 0:
                if self.ball.position[i] + self.ball.radius > self.box_max[i]:
                    self.ball.position[i] = self.box_max[i] - self.ball.radius
                    self.ball.velocity[i] = -self.restitution * self.ball.velocity[i]

    def grab(self):
        self.held = True
        self.ball.velocity = np.zeros(3)

    def move_held(self, position):
        if not self.held:
            return
        position = np.array(position, dtype=float)
        self.ball.position = np.clip(
            position, self.box_min + self.ball.radius, self.box_max - self.ball.radius
        )

    def release(self, velocity):
        self.held = False
        self.ball.velocity = np.array(velocity, dtype=float)

    def in_contact(self, tol=0.001):
        low = self.box_min + self.ball.radius
        high = self.box_max - self.ball.radius
        pos = self.ball.position
        return bool(np.any((pos - low <= tol) | (high - pos <= tol)))