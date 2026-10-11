from dataclasses import dataclass
import numpy as np

@dataclass
class Ball:
    position: np.ndarray
    velocity: np.ndarray
    radius: float = 0.1

class World:
    def __init__(self, ball, box_min, box_max, gravity, restitution=0.8):
        self.ball = ball
        self.box_min = box_min
        self.box_max = box_max
        self.gravity = gravity
        self.restitution = restitution

    def step(self, dt):
        self.ball.velocity += self.gravity * dt
        self.ball.position += self.ball.velocity * dt

        for i in range(3):
            if self.ball.velocity[i] < 0:
                if self.ball.position[i] - self.ball.radius <self.box_min[i]:
                    self.ball.position[i] = self.box_min[i] + self.ball.radius
                    self.ball.velocity[i] = -self.restitution * self.ball.velocity[i]
            if self.ball.velocity[i] > 0:
                if self.ball.position[i] + self.ball.radius > self.box_max[i]:
                    self.ball.position[i] = self.box_max[i] - self.ball.radius
                    self.ball.velocity[i] = -self.restitution * self.ball.velocity[i]