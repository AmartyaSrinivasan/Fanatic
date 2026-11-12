import numpy as np

class JetModel:
    def __init__(self, nozzle_diameter=0.05, k_decay=5.8, spread=0.1,
                 k_center=1.0, air_density=1.2, drag_coeff=0.47):
        self.nozzle_diameter = nozzle_diameter
        self.k_decay = k_decay
        self.spread = spread
        self.k_center = k_center
        self.air_density = air_density
        self.drag_coeff = drag_coeff

    def _geometry(self, point, fan_pos, fan_dir):
        fan_dir = fan_dir / np.linalg.norm(fan_dir)
        d = point - fan_pos
        z = np.dot(d, fan_dir)
        offset = d - z * fan_dir
        w = self.nozzle_diameter / 2 + self.spread * z
        return fan_dir, z, offset, w

    def air_velocity(self, point, fan_pos, fan_dir, airflow):
        fan_dir, z, offset, w = self._geometry(point, fan_pos, fan_dir)
        if z <= 0:
            return np.zeros(3)

        r = np.linalg.norm(offset)
        u_centre = airflow * min(1.0, self.k_decay * self.nozzle_diameter / z)
        u = u_centre * np.exp(-(r / w) ** 2)
        return u * fan_dir

    def force_on_ball(self, ball_pos, ball_vel, radius, fan_pos, fan_dir, airflow):
        fan_dir, z, offset, w = self._geometry(ball_pos, fan_pos, fan_dir)
        air = self.air_velocity(ball_pos, fan_pos, fan_dir, airflow)
        
        u = air - ball_vel
        area = np.pi * radius ** 2
        drag = 0.5 * self.air_density * self.drag_coeff * area * np.linalg.norm(u) * u
        
        if z > 0:
            u_local = np.linalg.norm(air)
            centring = -self.k_center * 0.5 * self.air_density * area * u_local**2 * offset / w
        else:
            centring = np.zeros(3)
        
        return drag + centring