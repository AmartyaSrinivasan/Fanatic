## 1. Overview:
It’s a 3D Python simulation, built with VPython, where a computer-controlled fan keeps a ball floating at a target point inside a box, like a ball balanced above a hairdryer, and brings it back when the user drags, flings or moves it. The controller is told gravity but not the ball’s mass - both of which the user can change mid flight - so it has to estimate the mass from how the ball responds to the airflow. The fan can move, aim in any direction and vary its airflow, but it can only push and must never touch the ball. It’s mainly for me, to learn feedback control and online estimation from the physics up, and the same problems show up in drones carrying unknown loads and in contactless handling of objects.

## 2. Goals:
In the end, the project includes a ball of unknown mass that hovers at any target, where the flings are recovered, the mass estimate converges and tracks changes, the fan never touches the ball, and automated tests prove all four. In the end, I also hope to add several stats and graphs to make the project seem more cool and interactive to any user.

## 3. Architecture:

## 4. Class responsibilities

| Class | Owns | Allowed to know | Must never know |
|---|---|---|---|
| Ball | position, velocity, radius, mass | nothing; it's just data | n/a |
| World | the ball, box corners, gravity, restitution, held flag, the fan, the jet model | the true state of everything physical | the controller's internals or mass estimate; anything about VPython |
| JetModel | physical constants: air density, drag coefficient, jet decay, spread, centring strength | only the positions, velocities, fan aim and airflow passed in each call | the World, the Fan object, the ball's mass; it stores no scene state |
| Fan | position, velocity, aim angles, airflow, and its limits (max speed, max acceleration, airflow range) | the latest command from the controller | the ball and its mass |
| PID | gains, integral total, previous error, output limits | only the error and the timestep it's given | what it is controlling |
| MassEstimator | the current mass estimate, the forgetting factor, its running statistics | the ball's observed motion, gravity, the air force from the fan's commanded settings | the true mass |
| Controller | three PIDs, the mass estimator, the current mode, the target | an Observation: ball position and velocity, held flag, gravity, fan state and limits, target | the true mass; the World object |
| App | VPython objects, sliders, mouse handlers, drag history | everything, read-only, for drawing; it passes user actions (grab, release, slider changes) into the World | it must never contain physics or control logic |

## 5. Key interfaces

### Observation: what the controller receives each step

| Field | Type | Units | Meaning |
|---|---|---|---|
| ball_position | 3-vector | m | where the ball is |
| ball_velocity | 3-vector | m/s | how fast it is moving |
| held | bool | n/a | whether the user is holding the ball |
| gravity | 3-vector | m/s² | gravity, as set by the user |
| target | 3-vector | m | where the user wants the ball |
| fan_position | 3-vector | m | where the fan currently is |
| fan_direction | 3-vector | unit vector | which way the fan currently points |
| fan_airflow | float | m/s | current airflow speed |
| fan_limits | FanLimits | various | max speed, max acceleration, airflow min and max |

Deliberately absent: the ball's mass.

### FanCommand: what the controller outputs each step

| Field | Type | Units | Meaning |
|---|---|---|---|
| fan_target_position | 3-vector | m | where the fan is supposed to go |
| tilt | float | radian | the angle from the y axis to the vector, in which the fan needs to face |
| turn | float | radian | the angle from the x axis to the vector's projection on the xz plane, in which the fan needs to face |
| airflow | float | m/s | the speed of air the fan needs to supply to move the ball/keep it afloat |


## 6. Control Design 
how the controller moves from Observation to Fancommand

### 1. Error: 
Controller calculates target - ball_position. (3-vector)
### 2. Desired Acceleration:
PID.py is called in all three axises, each turning their respective errors into the acceleration the ball should have.
### 3. Desired Air Force:
The controller then calculates the amount of force the air supplied by the fan, which will be the estimated ball_mass x ((desired acceleration) - (gravity)). The feedforward in PID.py is the -gravity part here, the force known in advance.
### 4. Aim:
The controller then points the fan in that direction, and places it behind the ball in that line.
### 5. Airflow: 
The fan can't set the force on the ball directly. It can only set its nozzle airflow speed. So the controller inverts the jet model to find the airflow that produces the desired force. Because the fan is aimed straight at the ball, the ball sits on the jet's centre line, and the inversion is two rearranged formulas:
   - Air speed needed at the ball, from the drag equation: u = √(2F / (p · C_d · A))
   - Nozzle speed needed to deliver that air speed at distance z, from the jet's decay:
     U₀ = u / min(1, k·D / z)
For example, holding a 0.05 kg ball needs about 7.4 m/s of air at the ball; from 0.3 m away, that means a nozzle speed of about 7.6 m/s. The result is then clamped to the fan's airflow limits. A later refinement adds the ball's own velocity along the jet, since drag depends on the air's speed relative to the ball.
### 6. Limits and clamps:
The ball is clamed between the boundaries of the 3D box, and the fan is also clamped outside a certain radius around the ball, ensuring they never touch.

## 7. Modes:
There are three modes the fan follows.
- **Hover:** hold the ball steady at the target using the three PIDs and the mass estimate.
- **Orbit:** while the user holds the ball, circle it at the clearance radius with the jet pointed away; reset the PIDs so they start fresh on release.
- **Recover:** after release, or after a big disturbance, bring the ball back toward the target while keeping clear of it. 

## 8. Physics model and assumptions

- **Integration.** Physics advances in fixed steps of 1/240 s, four per frame, while the
  display runs at 60 frames per second. Each step uses semi-implicit Euler: velocity is
  updated from acceleration first, then position from the new velocity. This is barely
  more work than the basic method but much more stable.

- **Gravity.** A constant acceleration, set by the user through a slider. The controller
  is told its value.

- **Walls.** The ball bounces off all six walls of the box. On impact it is placed back
  exactly at the wall and the velocity component into the wall is reversed and scaled by
  a restitution of 0.8, so the ball keeps 64% of its kinetic energy per bounce.

- **Drag.** Moving air pushes the ball with F = ½ · ρ · C_d · A · |u| · u, where u is the
  air's velocity relative to the ball, ρ = 1.2 kg/m³, C_d = 0.47 for a sphere, and A is
  the ball's cross-section. The same formula slows a ball moving through still air.
  *Approximation: constant drag coefficient; no spin or wake effects.*

- **The air jet.** The jet's speed along its centre line stays at the nozzle speed for a
  short core of length k·D (about 5.8 nozzle diameters), then falls off roughly as
  1/distance. Across the stream the speed follows a bell-shaped profile whose width grows
  linearly with distance. *Approximation: an empirical round-jet model from the
  literature, not a fluid simulation; it ignores the air drawn in from around the jet.*

- **Centring force.** A force pulling the ball toward the jet's centre line, proportional
  to the local air speed and to how far off-centre the ball sits, with a tunable strength
  K_CENTER. This is what lets a tilted jet hold the ball. *Approximation: the least
  certain part of the model. Results that depend on it, such as how far the jet can tilt,
  should be read as qualitative, and their sensitivity to K_CENTER is tested.*

- **Holding.** While the user holds the ball, physics is paused for it; its position
  follows the mouse along a plane facing the camera and is clamped inside the box. On
  release it takes the velocity of the mouse over the last 0.1 s.

- **The fan.** The fan moves with limited speed and acceleration, aims with two angles,
  and its airflow is clamped to a minimum and maximum. It never touches the ball: the
  controller keeps it outside a clearance radius sized from the fling-speed cap and the
  fan's maximum acceleration, so no ball-fan collision is modelled.


## 9. Technology choices

- **Python 3.12.** Newer versions (3.14) broke VPython's dependencies, so the project
  is pinned to 3.12 in a virtual environment.
- **numpy.** All positions, velocities and forces are numpy arrays, so the same physics
  and control code works in any number of dimensions. This keeps a 2D fallback possible
  without rewriting the maths. Note: on this machine, Windows blocks `numpy.random`, so
  tests use Python's built-in `random` module instead.
- **VPython.** Chosen over Pygame after the move to 3D: it draws spheres and boxes in one
  line, and provides camera rotation, zoom and slider widgets out of the box. Its built-in
  object picking returned nothing in this setup, so grabbing uses a custom hit test.
- **pytest.** Automated unit and scenario tests, run with `python -m pytest`.
- **Git and GitHub.** Version control, with small, frequent commits.

## 10. Testing strategy

- **Headless by design.** Physics, PID, jet model, estimator and controller never import
  VPython, so all of them can be tested without opening a window.
- **Unit tests.** Each module's rules are tested in isolation: free fall matches ½gt²,
  bounces lose energy, the ball never escapes the box, held balls don't move, the PID's
  terms and limits behave correctly, and the jet's force points the right way and weakens
  with distance.
- **Scenario tests (later).** Scripted runs check whole behaviours: flings from every
  corner are recovered, the mass estimate tracks sudden changes, and the fan never comes
  closer than the clearance radius.
- **Test the tests.** New tests are run against a known bug first, to confirm they can
  fail, and the "collected N items" count is checked so no test is silently skipped.

## 11. Risks and fallbacks

| Risk | Fallback |
|---|---|
| Inverting the jet model is unstable near the jet's edge | Keep the fan aimed straight at the ball; solve numerically if needed |
| PID tuning in 3D takes longer than planned | Lean on feedforward; tune one axis at a time |
| The mass estimate is too noisy | Smooth the measured acceleration; adjust the forgetting factor |
| Mass and jet strength can't be told apart | Accept a small bias; the PID's integral term absorbs it, and it is explained in the presentation |
| The 3D display or interaction becomes too costly | Fall back to a 2D display over the same physics code |
| Schedule slips | Cut from the "should" tier at the Nov 15 review; mass estimation is protected |