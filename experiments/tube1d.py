from fanatic.pid import PID

MASS = 0.05
G = 9.81
F_MAX = 1.5
TUBE_HEIGHT = 3.0
TARGET = 1.5
DT = 1 / 240
STEPS_PER_FRAME = 4
KP, KI, KD = 1.0, 0.0, 0.0
USE_FEEDFORWARD = False

ff = MASS * G if USE_FEEDFORWARD else 0.0
pid = PID(KP, KI, KD, out_min=-ff, out_max=F_MAX - ff)

def step(height, velocity):
    if height<0:
        height = 0
        if velocity<0:
            velocity*=-0.3
    if height>TUBE_HEIGHT:
        height = TUBE_HEIGHT
    error=TARGET-height
    force = ff + pid.update(error, DT)
    velocity +=((force - MASS*G)/MASS)*DT
    height +=velocity*DT
    return height, velocity

k,l=0.2,0.0
for i in range(120):
    for j in range(20):
        k,l=step(k,l)
    print(k,l)