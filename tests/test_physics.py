import numpy as np
from fanatic.physics import Ball, World
import random
def test_free_fall_matches_formula():
    ball = Ball(position=np.array([0.0, 50.0, 0.0]), velocity=np.zeros(3))
    world = World(ball, np.array([-10.0, 0.0, -10.0]), np.array([10.0, 100.0, 10.0]),
                  gravity=np.array([0.0, -9.81, 0.0]))
    for _ in range(240):          # one second
        world.step(1 / 240)
    assert abs((50.0 - ball.position[1]) - 0.5 * 9.81 * 1.0**2) < 0.05

def test_bounces_lose_energy():
    ball = Ball(position=np.array([0.0, 50.0, 0.0]), velocity=np.zeros(3))
    world = World(ball, np.array([-10.0, 0.0, -10.0]), np.array([10.0, 100.0, 10.0]),
                  gravity=np.array([0.0, -9.81, 0.0]))
    dt = 1/60
    speed1= None
    speed2=None

    for i in range(1000):
        vbefore = ball.velocity[1]
        world.step(dt)
        vafter = ball.velocity[1]
        if vbefore<0 and vafter>0:
            speed1= abs(vbefore)
            speed2 = abs(vafter)
            break

    assert speed1 is not None, "Ball didnt bounce"
    assert speed2 < speed1



def test_ball_never_escape():
    random.seed(42)
    velocity = np.array([random.uniform(-20.0, 20.0) for _ in range(3)])
    ball = Ball(position=np.array([0.0, 50.0, 0.0]), velocity=velocity)
    ...
    box_min = np.array([-10.0, 0.0, -10.0])
    box_max = np.array([10.0, 100.0, 10.0])
    world = World(ball, box_min, box_max, gravity=np.array([0.0, -9.81, 0.0]))
    dt = 1/60

    for _ in range(5000):
        world.step(dt)
        assert np.all(ball.position >= box_min + ball.radius)
        assert np.all(ball.position <= box_max - ball.radius)