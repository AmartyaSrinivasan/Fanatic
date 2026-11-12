import numpy as np
from fanatic.jet import JetModel

FAN_POS = np.zeros(3)
UP = np.array([0.0, 1.0, 0.0])

def test_behind_fan_gets_no_force():
    jet = JetModel()
    force = jet.force_on_ball(np.array([0.0, -0.5, 0.0]), np.zeros(3), 0.1,
                              FAN_POS, UP, airflow=8.0)
    assert np.allclose(force, 0.0)

def test_on_axis_force_points_along_jet():
    jet = JetModel()
    force = jet.force_on_ball(np.array([0.0, 0.3, 0.0]), np.zeros(3), 0.1,
                              FAN_POS, UP, airflow=8.0)
    assert abs(force[0]) < 1e-9
    assert abs(force[2]) < 1e-9
    assert force[1] > 0.0


def test_further_away_means_weaker():
    jet = JetModel()
    force_near = jet.force_on_ball(np.array([0.0, 0.3, 0.0]), np.zeros(3), 0.1,
                                 FAN_POS, UP, airflow=8.0)
    force_far = jet.force_on_ball(np.array([0.0, 1.0, 0.0]), np.zeros(3), 0.1,
                                  FAN_POS, UP, airflow=8.0)
    assert force_near[1] > force_far[1]


def test_off_axis_sideways_force_points_to_axis():
    jet = JetModel()
    force = jet.force_on_ball(np.array([0.05, 0.3, 0.0]), np.zeros(3), 0.1,
                              FAN_POS, UP, airflow=8.0)
    assert force[0] < 0.0


def test_no_airflow_plain_air_resistance():
    jet = JetModel()
    force = jet.force_on_ball(np.array([0.0, 0.3, 0.0]), np.array([2.0, 0.0, 0.0]), 0.1,
                              FAN_POS, UP, airflow=0.0)
    assert force[0] < 0.0
    assert abs(force[1]) < 1e-9
    assert abs(force[2]) < 1e-9