import numpy as np
from vpython import box, sphere, vector, rate, color, scene, mag, cylinder, arrow, slider, wtext, label
from fanatic.physics import Ball, World
from fanatic.jet import JetModel
from fanatic.fan import Fan
from fanatic.interfaces import FanCommand, Observation
import time
from fanatic.controller import Controller

fps = 60
dt = 1 / 240
steps = 4


def to_vec(arr):
    return vector(arr[0], arr[1], arr[2])


def run():
    scene.width = 1200
    scene.height = 770
    scene.background = color.black
    box_min = np.array([-2.0, 0.0, -1.5])
    box_max = np.array([2.0, 3.0, 1.5])
    gravity = np.array([0.0, -9.81, 0.0])

    
    fan = Fan(position=[0.0, 0.5, 0.0], airflow=7.4, max_speed=40.0, max_accel=80.0,
          max_ang_speed=12.0, max_ang_accel=60.0)
    ball = Ball(position=np.array([0.0, 0.7, 0.0]), velocity=np.zeros(3))
    world = World(ball, box_min, box_max, gravity, fan=fan, jet=JetModel())
    controller = Controller(world.jet, ball.radius)
    fan_visual = cylinder(pos=to_vec(fan.position), axis=to_vec(0.08 * fan.direction()), radius=0.06, color=color.gray(0.6), pickable=False)
    jet_visual = arrow(pos=to_vec(fan.position), axis=to_vec(0.4 * fan.direction()), shaftwidth=0.02, color=color.cyan, opacity=0.5, pickable=False)
    ball_visual = sphere(
        pos=to_vec(ball.position), radius=ball.radius, color=color.orange
    )
    nag = label(text="Put the ball down.", height=24, color=color.red,
                box=False, visible=False, pos=to_vec(ball.position))
    

    def set_mass(s):
        world.ball.mass = s.value

    def set_gravity(s):
        world.gravity = np.array([0.0, -s.value, 0.0])

    scene.append_to_caption("\nBall mass (kg)  ")
    slider(bind=set_mass, min=0.02, max=0.15, value=0.05, length=300)
    scene.append_to_caption("\nGravity (m/s²)  ")
    slider(bind=set_gravity, min=2.0, max=15.0, value=9.81, length=300)
    scene.append_to_caption("\n\n")
    readout = wtext(text="")

    centre = (box_min + box_max) / 2
    scene.center = to_vec(centre)
    
    size = box_max - box_min
    box(pos=to_vec(centre), size=to_vec(size), opacity=0.15, pickable=False)

    history = []

    def record():
        history.append((time.perf_counter(), world.ball.position.copy()))

    def on_down(evt):
        p = scene.mouse.project(normal=scene.forward, point=ball_visual.pos)
        if p is None:
            return
        if mag(p - ball_visual.pos) < 4 * ball_visual.radius:
            world.grab()
            scene.userzoom = False
            scene.userspin = False
            scene.userpan = False
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
        scene.userzoom = True
        scene.userspin = True
        scene.userpan = True
        now = time.perf_counter()
        while history and now - history[0][0] > 0.1:
            history.pop(0)
        if not world.held:
            return
        if len(history) < 2 or (history[-1][0] - history[0][0]) == 0:
            world.release(np.zeros(3))
        else:
            world.release(
                (history[-1][1] - history[0][1]) / (history[-1][0] - history[0][0])
            )

    scene.bind("mousedown", on_down)
    scene.bind("mousemove", on_move)
    scene.bind("mouseup", on_up)

    target = np.array([0.0, 1.5, 0.0])
    sphere(pos=to_vec(target), radius=0.04, color=color.green, pickable=False)  

    while True:
        rate(fps)
        for _ in range(steps):
            obs = Observation(
                ball_position = world.ball.position.copy(), 
                ball_velocity = world.ball.velocity.copy(), 
                held = world.held, 
                gravity = world.gravity.copy(), 
                target = target, 
                fan_position = world.fan.position.copy(), 
                fan_direction = world.fan.direction().copy(),
                fan_airflow = world.fan.airflow,
                in_contact = world.in_contact(),
                view_direction=np.array([scene.forward.x, scene.forward.y, scene.forward.z]),
                )
            command = controller.update(obs, dt)
            world.fan.apply(command, dt)
            world.step(dt)
        readout.text = (f"True mass: {world.ball.mass:.3f} kg    "
                        f"Estimated: {controller.estimator.mass:.3f} kg")
        
        ball_visual.pos = to_vec(world.ball.position)
        fan_visual.pos = to_vec(world.fan.position)
        fan_visual.axis = to_vec(0.08 * world.fan.direction())
        jet_visual.pos = to_vec(world.fan.position)
        jet_visual.axis = to_vec(0.4 * world.fan.direction())
        nag.visible = controller.is_grumpy
        nag.pos = to_vec(world.fan.position + np.array([0.0, 0.4, 0.0]))
        