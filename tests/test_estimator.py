from fanatic.estimator import MassEstimator as me
import numpy as np
import random

def test_converges():
    m=me()
    for i in range(500):
        x=np.array([random.uniform(-5,5), random.uniform(-5,5), random.uniform(-5,5)])
        force = 0.08*x
        m.update(x, force)
    assert abs(m.mass-0.08) < 0.001

def test_track_change():
    m=me()
    for i in range(500):
        x=np.array([random.uniform(-5,5), random.uniform(-5,5), random.uniform(-5,5)])
        force = 0.08*x
        m.update(x, force)
    for i in range(500):
        x=np.array([random.uniform(-5,5), random.uniform(-5,5), random.uniform(-5,5)])
        force = 0.12*x
        m.update(x, force)
    assert abs(m.mass-0.12) < 0.005