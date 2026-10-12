import numpy as np
from vpython import box, sphere, vector, rate, color
from fanatic.physics import Ball, World

fps = 60
dt = 1 / 240
steps = 4 

def to_vec(arr):
    return vector(arr[0], arr[1], arr[2])

def run():
    box_min = np.array([-2.0, 0.0, -1.5])
    box_max = np.array([2.0, 3.0, 1.5])
    gravity = np.array([0.0, -9.81, 0.0])
    ball = Ball(position=np.array([-1.0, 3.0, 1.4]),
                velocity=np.array([6.5, 3.0, -2.0]))
    world = World(ball, box_min, box_max, gravity)
    centre = (box_min + box_max) / 2
    size = box_max - box_min
    box(pos=to_vec(centre), size=to_vec(size), opacity=0.15)
    ball_visual = sphere(pos=to_vec(ball.position),
                         radius=ball.radius, color=color.orange)
    while True:
        rate(fps)
        for _ in range(steps):
            world.step(dt)
        ball_visual.pos = to_vec(world.ball.position)