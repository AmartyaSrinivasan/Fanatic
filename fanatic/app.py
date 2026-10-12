import numpy as np
from vpython import box, sphere, vector, rate, color, scene, mag
from fanatic.physics import Ball, World
import time

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
    box(pos=to_vec(centre), size=to_vec(size), opacity=0.15, pickable=False)
    ball_visual = sphere(pos=to_vec(ball.position),
                         radius=ball.radius, color=color.orange)
    history = []

    def record():
        history.append((time.perf_counter(), world.ball.position.copy()))

    def on_down(evt):
        p = scene.mouse.project(normal=scene.forward, point=ball_visual.pos)
        if p is None:
            return
        if mag(p - ball_visual.pos) < 4 * ball_visual.radius:
            world.grab()
            history.clear()
            record()

    def on_move(evt):
        if not world.held:
            return
        p = scene.mouse.project(normal=scene.forward, point=ball_visual.pos)
        if p is None:
            return
        world.move_held([p.x, p.y, p.z])
        record()
        now = time.perf_counter()
        while history and now - history[0][0] > 0.1:
            history.pop(0)

    def on_up(evt):
        now = time.perf_counter()
        while history and now - history[0][0] > 0.1:
            history.pop(0)
        if not world.held:
            return
        if len(history)<2 or (history[-1][0]-history[0][0])==0:
            world.release(np.zeros(3))
        else:
            world.release((history[-1][1]-history[0][1])/(history[-1][0]-history[0][0]))

    scene.bind('mousedown', on_down)
    scene.bind('mousemove', on_move)
    scene.bind('mouseup', on_up)

    while True:
        rate(fps)
        for _ in range(steps):
            world.step(dt)
        ball_visual.pos = to_vec(world.ball.position)