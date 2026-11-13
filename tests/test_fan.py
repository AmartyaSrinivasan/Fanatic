from fanatic.fan import Fan
from fanatic.interfaces import FanCommand
import numpy as np


def test_straight_up():
    fan = Fan(tilt=0.0)
    expected = np.array([0, 1, 0])
    assert np.allclose(fan.direction(), expected)


def test_sideways():
    fan = Fan(tilt=np.pi / 2, turn=0.0)
    expected = np.array([1, 0, 0])
    assert np.allclose(fan.direction(), expected)


def test_airflow_clamp():
    fan = Fan(airflow_max=20.0)
    command = FanCommand(target_position=np.zeros(3), tilt=0.0, turn=0.0, airflow=999.0)
    fan.apply(command, dt=0.1)
    assert fan.airflow == fan.airflow_max
