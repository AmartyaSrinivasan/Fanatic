from fanatic.pid import PID
from vpython import box, sphere, vector, rate, color, graph, gcurve, scene

MASS = 0.05
G = 9.81
F_MAX = 1.5
TUBE_HEIGHT = 3.0
TARGET = 1.5
DT = 1 / 240
STEPS_PER_FRAME = 4
KP, KI, KD = 1.0, 0.5, 0.4
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

scene.title = f"KP={KP}  KI={KI}  KD={KD}  feedforward={USE_FEEDFORWARD}"
scene.center = vector(0, TUBE_HEIGHT / 2, 0)

box(pos=vector(0, TUBE_HEIGHT / 2, 0), size=vector(0.3, TUBE_HEIGHT, 0.3),
    opacity=0.15, pickable=False)
ball = sphere(pos=vector(0, 0.2, 0), radius=0.1, color=color.orange)
box(pos=vector(0, TARGET, 0), size=vector(0.5, 0.01, 0.5), color=color.green)

g = graph(title="Height vs time", xtitle="time (s)", ytitle="height (m)",
          ymin=0, ymax=TUBE_HEIGHT, width=600, height=300)
height_curve = gcurve(graph=g, color=color.orange)
target_curve = gcurve(graph=g, color=color.green)

height, velocity, t = 0.2, 0.0, 0.0
while t < 15:
    rate(60)
    for _ in range(STEPS_PER_FRAME):
        height, velocity = step(height, velocity)
        t += DT
    ball.pos.y = height
    height_curve.plot(t, height)
    target_curve.plot(t, TARGET)

while True:
    rate(30)