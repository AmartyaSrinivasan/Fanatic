class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.total = 0.0
        self.old = None

    def update(self, error, dt):
        p = self.kp * error
        i = self.ki * self.total
        if self.old is None:
            d = 0.0
        else:
            d = (error - self.old) * self.kd / dt
        self.old = error

        output = p + i + d
        if not (
            (output >= self.out_max and error > 0)
            or (output <= self.out_min and error < 0)
        ):
            self.total += error * dt
            i = self.ki * self.total
            output = p + i + d

        clamped_output = max(self.out_min, min(self.out_max, output))

        return clamped_output

    def reset(self):
        self.total = 0
        self.old = None
