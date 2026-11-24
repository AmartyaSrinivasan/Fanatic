import numpy as np


class MassEstimator:
    def __init__(self, initial_guess=0.05, forgetting=0.99, max_accel=50.0):
        self.initial_guess = initial_guess
        self.forgetting = forgetting
        self.max_accel = max_accel
        self.sxx = 1.0
        self.sxy = initial_guess*1.0
        self.mass = initial_guess

    def update(self, x, force):
        if np.linalg.norm(x) > self.max_accel :
            return None
        
        self.sxx = self.forgetting * self.sxx + np.dot(x, x)
        self.sxy = self.forgetting * self.sxy + np.dot(x, force)
        self.mass = self.sxy/self.sxx