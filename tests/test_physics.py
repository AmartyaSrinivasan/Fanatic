import numpy as np
from fanatic.physics import Ball, World
import random

BOX_MIN = np.array([-10.0, 0.0, -10.0])
BOX_MAX = np.array([10.0, 100.0, 10.0])

def c(velocity=None):
    if velocity is None:
        velocity = np.zeros(3)
    ball = Ball(position=np.array([0.0, 50.0, 0.0]), velocity=velocity)
    world = World(ball, BOX_MIN.copy(), BOX_MAX.copy(),
                  gravity=np.array([0.0, -9.81, 0.0]))
    return world, ball

def test_free_fall_matches_formula():
    world, ball = c()
    for _ in range(240):
        world.step(1 / 240)
    assert abs((50.0 - ball.position[1]) - 0.5 * 9.81 * 1.0**2) < 0.05

def test_bounces_lose_energy():
    world, ball = c()
    dt = 1 / 240
    speed1 = None
    speed2 = None

    for _ in range(1000):
        vbefore = ball.velocity[1]
        world.step(dt)
        vafter = ball.velocity[1]
        if vbefore < 0 and vafter > 0:
            speed1 = abs(vbefore)
            speed2 = abs(vafter)
            break

    assert speed1 is not None, "Ball didnt bounce"
    assert speed2 < speed1

def test_ball_never_escape():
    random.seed(42)
    velocity = np.array([random.uniform(-20.0, 20.0) for _ in range(3)])
    world, ball = c(velocity)
    dt = 1 / 240

    for _ in range(5000):
        world.step(dt)
        assert np.all(ball.position >= BOX_MIN + ball.radius)
        assert np.all(ball.position <= BOX_MAX - ball.radius)

def test_ball_hold_and_release():
    world, ball = c()
    world.grab()
    pos1 = ball.position.copy()

    for _ in range(4):
        world.step(1 / 240)
    assert np.array_equal(pos1, ball.position)

    vel = np.array([10.0, 5.0, -2.0])
    world.release(vel)
    world.step(1 / 240)
    expected = vel + world.gravity * (1 / 240)
    assert np.allclose(ball.velocity, expected)

def test_move_held_clamps_inside_box():
    world, ball = c()
    world.grab()
    world.move_held([1000.0, 1000.0, 1000.0])
    assert np.array_equal(world.ball.position, BOX_MAX - world.ball.radius)