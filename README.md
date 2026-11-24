# handtool_throw

Throwing objects with a Franka Panda that carries a pneumatic hand tool: a tube on the
flange with a suction cup at its tip. The tool grabs an object by suction and throws it
with a blow of compressed air. The robot aims the tube, and the throwing service decides
the launch angle (and, optionally, the valve time) that sends the object onto a target.

The same nodes drive the real robot and a Gazebo simulation. In simulation a Gazebo plugin
replaces the Arduino of the tool.

- [Overview](#overview)
- [Package layout](#package-layout)
- [Frames and conventions](#frames-and-conventions)
- [The throwing model](#the-throwing-model)
- [Configuration](#configuration)
- [Building](#building)
- [Running on the real robot](#running-on-the-real-robot)
- [Running in simulation](#running-in-simulation)
- [Offline tools](#offline-tools)
- [Ball experiments](#ball-experiments)
- [Known limitations](#known-limitations)

## Overview

```
                    handtool_throw_service (throwing_par.py)
                         ^ target from the throwing point, mass, energy
                         | v launch angle theta, valve time
  qualisys ------------> testing (testing.cpp) ------------> /backstepping_controller/command_cartesian
  (target box, base)        |        ^                          (minimum jerk cartesian trajectory)
                            |        +------------------------- /backstepping_controller/franka_pose
                            v
                    arduino/suctioning, arduino/blowing_off, arduino/valve, arduino/duty_cycle
                            |
                 real robot: Arduino of the tool (rosserial)
                 simulation: ball_throw_plugin (Gazebo)
```

- **testing** (`src/testing.cpp`) is the interactive node. It reads a choice from the
  keyboard, moves the robot through the backstepping controller, asks the service for the
  throwing parameters and drives the tool.
- **handtool_throw_service** (`scripts/throwing_par.py`) solves one throw. It only knows the
  ballistics: the target arrives already measured from the throwing point, and it answers
  with a launch angle over the horizon and a valve time.
- **throwing_model.py** holds the physics and the solver without any ROS, so
  `throwing_check.py` runs exactly the same code offline.
- **ball_throw_plugin** (`src/ball_throw_plugin.cpp`) is the simulated tool and the balls.
  It listens to the same `arduino/*` topics the real tool listens to, and publishes the
  balls and their landing points to RViz.

## Package layout

| Path | What it is |
|---|---|
| `src/testing.cpp` | Interactive node: menu, motions, calls to the service, tool commands |
| `src/ball_throw_plugin.cpp` | Gazebo world plugin: spawns the balls in their row, holds and releases them, RViz markers |
| `include/handtool_throw/yaml_helpers.h` | Readers of scalars, vectors and poses from the yaml, shared by the testing node and the plugin |
| `scripts/throwing_model.py` | Ballistic model and solver, no ROS |
| `scripts/throwing_par.py` | ROS service node around the model |
| `scripts/throwing_check.py` | Runs the model offline on a target, a sweep or several parameter sets |
| `scripts/generate_sim_objects.py` | Writes the ball list of a simulated session: balls in a row, random targets |
| `scripts/dummy_qualisys.py` | Publishes fixed base and target poses in place of the motion capture |
| `srv/throwing_par_srv.srv` | Request: mass, energy, target from the throwing point. Answer: valve time, theta |
| `launch/handtool_throw.launch` | testing node + service, with the parameters below |
| `launch/handtool_throw_sim.launch` | Gazebo + robot with tube + backstepping controller + `handtool_throw.launch` |
| `launch/handtool_throw_exp.launch` | Real experiment: robot + backstepping controller + Arduino + Qualisys + `handtool_throw.launch` + `exp_throw_objects.yaml` |
| `launch/run_arduino.launch` | rosserial node of the tool Arduino on `/dev/ttyACM0` |
| `launch/dummy_qualisys.launch` | `dummy_qualisys.py` with `configs/dummy_qualisys_params.yaml` |
| `urdf/panda_with_tube.urdf.xacro`, `urdf/tube.xacro` | Panda from `franka_description` with the tube bolted on the flange |
| `worlds/ball_throw.world` | Floor, sun and the hand tool plugin |
| `rviz/ball_throw.rviz` | Robot, targets, balls, landing points and floor |
| `configs/handtool_params.yaml` | Throwing pose and service parameters |
| `configs/throw_objects.yaml` | Default pose, motion times and object list of the sequence (choice 7) |
| `configs/exp_throw_objects.yaml` | Balls of the real experiment (grasp poses to fill) and the aimed point of the box |
| `configs/ball_throwing.txt` | Raw data of the ball experiments: frames and landing points |
| `configs/ball_landing_points.yaml` | Ball experiments reduced to the tool frame: landing points, mass, launch energy |
| `configs/throwing_sequence*.txt` | Notes of earlier throwing tests (tennis ball, wooden ball, cubes) |
| `notebooks/air_shot_model.ipynb`, `notebooks/throws.yaml` | Physical model of the air shot and identification of the valve energy model from vertical shots |

## Frames and conventions

- **Robot base frame** (`panda_link0`): every pose and every target in the yaml files and on
  the topics is expressed here, x forward, y left, z up, in metres.
- **Tube tip**: the end effector of the backstepping controller (thunder `EE` =
  flange + [0.14, 0, 0.02] m) is the free end of the tube (`tube_tip` in the URDF), so
  `franka_pose` and the throwing pose are poses of the point the object leaves from. The
  tube axis is the x axis of that frame.
- **Throwing point**: the testing node owns it.
  - Choice 7 throws every object from `testing/throw_position`.
  - Choices 5 and 6 throw from `testing/throw_pose` position.

  The testing node subtracts the throwing point from the target before calling the service,
  and builds the robot pose from the answer:

  ```
  R_throw = Rz(azimuth) * Ry(-theta) * throw_pose.orientation,   azimuth = atan2(target.y, target.x)
  ```

  So `throw_pose.orientation` must hold the tube horizontal along the base +x. It only sets
  the roll of the tool around the tube (which side the flange stays on), never the landing
  point. The testing node warns at start-up if it does not.
- **Target**: the point where the object should be when it comes down, in the base frame.
  For a ball on the floor it is the centre of the ball at touch-down, one radius above the
  floor. In the lab the base is 0.935 m above the floor, so a floor target has
  z = -0.935 + radius.
- **Orientations in yaml** are written either as `orientation: [x, y, z, w]` or as
  `rpy_deg: [r, p, y]` (extrinsic xyz, R = Rz*Ry*Rx).

## The throwing model

The service solves a planar, drag-free ballistic problem: the object leaves the throwing
point at speed v and angle theta, and must cover the horizontal distance R to the target
while falling by h. The speed comes from the launch energy, v = sqrt(2E/m).

Where E comes from is decided by a single parameter, `optimization/energy_source`.

### `energy_source: measured` (default)

- The energy is the one measured for the object. It comes with the request (`energy` field
  of each object), and the valve is always opened for `used_valve_us`, the valve time the
  energies were measured with.
- theta is the only unknown, solved in closed form:

  ```
  tan(theta) = (v^2 -+ sqrt(v^4 - g (g R^2 - 2 h v^2))) / (g R)
  ```

- For a target closer than the maximum range there are two solutions. The minus sign is the
  flat throw (`branch: low`) and the plus sign the lob (`branch: high`). With `branch: any`
  the one closer to `theta_ref_deg` is taken. A solution outside `bound_theta_deg` is
  rejected.
- A target beyond the maximum range is reported as out of reach, together with the maximum
  range at that energy. A target at the maximum range within `dist_tol` is accepted with
  the angle of maximum range.

### `energy_source: valve`

- The energy comes from the valve opening time through the identified model
  `E(t) = a1*log_b(a2*t + a3) - E(valve_0)` (`energy_a*`, `valve_0`).
- An SLSQP optimizer moves the variables listed in `free_vars` (`valve`, `theta` or both).
  The landing distance is an equality constraint, and the cost keeps the solution near
  `valve_ref` and `theta_ref_deg`.
- It is multi-start (`n_starts` per free variable) and filtered by `branch`.
- This was the original behaviour. In practice the valve is not deterministic enough to aim
  with, which is why `measured` became the default.

### Answer

The service answers with `result_theta` (launch angle over the horizon, rad) and
`result_valve_us`. It logs a report of every throw: distance, theta, branch, speed, energy,
apex height, flight time, and the sensitivity of the landing point to one degree of theta
(and to 100 us of valve time, in `valve` mode).

## Configuration

### `configs/handtool_params.yaml`

`testing` namespace:

| Parameter | Meaning |
|---|---|
| `pre_suct_time`, `suct_time` | [s] wait before suction, and time the cup needs to hold the object |
| `valve_us`, `duty_regulator` | Valve time and pressure regulator duty cycle of the manual choices 1–4 (255 bypasses the regulator) |
| `throw_pose` | Reference orientation of every throw, and throwing point of choices 5 and 6 (tube tip at the release), see [Frames](#frames-and-conventions). The position is the tool position of the ball experiments |

`optimization` namespace:

| Parameter | Meaning |
|---|---|
| `energy_source` | `measured` or `valve`, see [The throwing model](#the-throwing-model) |
| `used_valve_us` | [us] valve time of every `measured` throw (70000) |
| `bound_theta_deg` | [deg] allowed launch angles |
| `branch` | `low` (flat), `high` (lob) or `any` (closest to `theta_ref_deg`) |
| `theta_ref_deg` | [deg] preferred launch angle |
| `dist_min` | [m] a target closer than this to the throwing point (horizontally) has no direction and is rejected |
| `dist_tol` | [m] a solution missing the target by more than this is reported as failed |
| `g` | [m/s^2] |
| `valve_0`, `energy_a1..a3`, `energy_b`, `bound_valve`, `free_vars`, `n_starts`, `valve_ref`, `w_valve`, `w_theta`, `valve_scale`, `theta_scale_deg` | Used only with `energy_source: valve`, see the comments in the file |

### `configs/throw_objects.yaml`

This file is used by choice 7 and lives in the `testing` namespace:

- `default_pose`: where the sequence starts and returns to after every throw. Choice 7
  refuses to run without it.
- `approach_offset`: [m] offset of the approach pose, in the tip frame of the grasp pose.
  -x backs the tool away along the tube.
- `throw_position`: [m] throwing point of every object (tube tip at the blow), in the base
  frame. Choice 7 refuses to run without it. All the objects leave from the same point: thrown
  from above their own grasp, the balls on the +y side of the row put the tool against the
  robot. In the ball experiment it is 6 cm above the grasp position of `ball6cm`, see
  `exp_throw_objects.yaml`. Keep it clear of the objects still resting: with the tube almost
  vertical the wrist hangs about 0.4 m under the tip.
- `move_time`, `approach_time`, `pre_throw_time`: [s] durations of the long and short
  motions, and settling time on the throwing pose before the blow.
- `objects`: the list thrown one after the other. Each entry has:

  | Field | Required | Meaning |
  |---|---|---|
  | `name` | no | Used in the printouts and, in simulation, as the ball model name |
  | `mass` | yes | [kg] |
  | `grasp_pose` | yes | Pose the suction cup grabs the object from |
  | `energy` | with `energy_source: measured` | [J] measured launch energy, see `ball_landing_points.yaml` |
  | `target` | no | [m] fixed target. Without it, the box measured by qualisys is used |
  | `radius` | simulation only | [m] radius of the simulated ball, ignored by the testing node |
  | `sim_position` | simulation only | [m] centre of the resting ball, where the plugin spawns it, ignored by the testing node |

  An entry with a missing or malformed required field is skipped with an error. The list is
  re-read at every run of choice 7, so it can be reloaded with `rosparam load` in between.

## Building

```bash
catkin build handtool_throw
source devel/setup.bash
```

The Python nodes need numpy, scipy and yaml (`python3-scipy` is not in the base image of the
workspace container: `sudo apt install python3-scipy` or `rosdep install handtool_throw`).
After a change of `srv/throwing_par_srv.srv`, rebuild: the testing node and the service must
see the same message.

## Running on the real robot

The pieces to bring up, each from its own terminal:

```bash
# robot and backstepping controller (panda_controllers)
roslaunch panda_controllers backstepping_controller.launch robot_ip:=<robot ip>

# Arduino of the tool (rosserial, /dev/ttyACM0)
roslaunch handtool_throw run_arduino.launch

# target and robot base from the motion capture: the real one ...
roslaunch mocap_qualisys qualisys.launch
# ... or fixed poses, when the targets are written in the objects yaml or for testing
roslaunch handtool_throw dummy_qualisys.launch

# testing node + throwing service
roslaunch handtool_throw handtool_throw.launch
```

The testing node asks for a choice on the keyboard:

| Choice | What it does |
|---|---|
| 1 | Set `pre_suct_time` and `suct_time` (type both values) |
| 2 | Set the duty cycle of the pressure regulator |
| 3 | Set the valve time [us] |
| 4 | Suck and throw from where the robot is |
| 5 | Ask the service for the throw of `m_obj energy x y z` (target in the base frame) from `throw_pose`, print the result, no motion |
| 6 | Throw at the qualisys target from `throw_pose`: type `m_obj energy`, the robot goes to the throwing pose, sucks and throws |
| 7 | Objects sequence of `throw_objects.yaml`, see below |

For each object, choice 7 runs:

```
default pose -> approach pose -> grasp pose -> suction -> approach pose
             -> waypoint -> throwing pose (throw_position) -> wait pre_throw_time -> blow
             -> default pose -> press enter
```

- **Waypoint:** it sits on the throwing position, with the orientation halfway along the
  rotation from the grasp (tube pointing down) to the throw (tube aiming up). That rotation
  is 145–178°. Done in one segment, the backstepping lags more than 60° behind (its
  orientation law is singular at 90°), and near 180° the rotation axis is barely defined.
  With the waypoint, each segment turns at most about 90°.
- **`pre_throw_time` is 5 s:** in simulation, 1 s left the tube 4–11° short of the commanded
  pitch at the blow.

The throwing parameters are asked first, so an object the service cannot throw is skipped
before it is grabbed. The valve time of the answer is sent to the Arduino before the
motion.

The targets are published on `/throw_targets` (latched `MarkerArray`, frame `panda_link0`)
as a red disc and the object name, on the real robot too. Choice 7 publishes all the fixed
targets of the list at its start, and adds a qualisys target when it is measured. Choices 5
and 6 publish their single target.

> With `energy_source: measured` every object needs its `energy`. Copy it from
> `configs/ball_landing_points.yaml`. An object without one is rejected by the service.

### Ball experiment with the motion capture

`launch/handtool_throw_exp.launch` starts everything the real experiment needs:

- the robot and the backstepping controller (`robot_ip`, default 172.16.0.104);
- the Arduino;
- Qualisys;
- the testing node and the service;
- RViz, with `rviz:=true`.

It loads `configs/exp_throw_objects.yaml` after `throw_objects.yaml`, so only the object
list, `box_target_offset` and `throw_position` are replaced. The default pose and the
motion times stay as they are.

```bash
roslaunch handtool_throw handtool_throw_exp.launch robot_ip:=<robot ip>   # rviz:=true
# then choice 7 in the testing node
```

**Before the first run, fill `configs/exp_throw_objects.yaml`:**

- **`grasp_pose` of each ball:** the measured pose of the tube tip on the ball, in the base
  frame. A ball without one is skipped with an error, so the robot never moves to a pose
  nobody measured.
- **`box_target_offset`:** the point the ball centre aims at, in the frame of the
  `box_target` rigid body. Because it is in the box frame, the box can be moved and turned
  between throws.
- **`throw_position`:** where every ball is thrown from, 6 cm above the grasp position of
  `ball6cm` (the first ball, thrown first, so that spot is free by then). Thrown from above
  `ball7cm` or `ball8cm`, the tool hits the robot. Update it whenever the grasp pose of
  `ball6cm` is measured again.

**How a target is measured.** The balls have no `target`, so before each one is grabbed the
testing node waits for fresh poses of the two QTM rigid bodies:

- `mpc_franka` on `/qualisys/mpc_franka/pose`: its origin and axes must coincide with
  `panda_link0`;
- `box_target` on `/qualisys/box_target/pose`.

From those it computes the target in the base frame:

```
target = R_base^T * (p_box + R_box * box_target_offset - p_base)
```

It prints the target, publishes it to RViz, and asks the service for the angle. The
sequence then continues exactly as in simulation: grasp, waypoint, throw from
`throw_position`, default pose. If the box is out of reach, the object is skipped before it
is grabbed.

## Running in simulation

`launch/handtool_throw_sim.launch` starts:

- Gazebo with `worlds/ball_throw.world` (floor and the hand tool plugin);
- RViz with `rviz/ball_throw.rviz` (`rviz:=false` to skip it);
- the Panda with the tube, its base `base_z` = 0.935 m above the floor as in the lab;
- the backstepping controller (`panda_controllers.yaml`);
- `handtool_throw.launch`, the same testing node and service as on the real robot;
- the simulated object list `sim_objects`, loaded after `throw_objects.yaml` so that only
  `testing/objects` is replaced.

**1. Generate the balls and their random targets.** The default output is
`configs/sim_throw_objects.yaml`, the file the launch loads:

```bash
rosrun handtool_throw generate_sim_objects.py --n 2 --seed 0
```

The script takes the five balls of `configs/ball_landing_points.yaml` (mass, measured energy,
radius from the diameter) and writes each of them `--n` times:

- **Row:** the balls rest in a row in front of the robot, at x = `--row-x` (0.5 m) on a
  surface at z = `--row-z` (0 m, the height of the base). They are `--spacing` (0.15 m)
  apart along y, centred on y = 0, from the 5 cm ball on the -y side.
- **Grasp:** from above, with the tube pointing down (the orientation of the first object
  of `throw_objects.yaml`), and the tip `--grasp-gap` (5 mm) over the top of the ball.
- **Targets:** each ball gets a target drawn uniformly on the floor in `--x` × `--y`
  (default [1, 5] × [-1, 1] m, base frame). Reachability is checked from the
  `throw_position` of `throw_objects.yaml`, with the same model and the same
  `handtool_params.yaml` as the service. A target it cannot reach is drawn again, and the
  throw found is printed.

Other options: `--seed`, `--base-height` (must match `base_z`), `--out`.

**2. Launch and run the sequence.**

```bash
roslaunch handtool_throw handtool_throw_sim.launch          # headless:=true, rviz:=true, base_z:=...
# then choice 7 in the testing node
```

**What the plugin does:**

- At start-up every object of `/testing/objects` is spawned as a solid sphere at its
  `sim_position`, on a thin static shelf built under the row.
- `arduino/suctioning` attaches the resting ball closest to the tube tip, if it is within
  `grasp_dist` (0.1 m), so the list order does not matter. Its centre snaps onto the tip
  and is held there, with no contacts, while the robot moves.
- `arduino/blowing_off` releases it with the tip velocity plus sqrt(2E/m) along the tube
  axis. Contacts come back once the ball is clear of the tube.
- When the ball first touches the floor its landing point is logged in the base frame,
  with its target and the miss:

  ```
  ball_throw: ball7cm_0_0 released at 4.430 m/s, tube axis [0.841, 0.000, 0.542] (pitch 32.80 deg)
  ball_throw: ball7cm_0_0 landed at [3.173, -0.354] m, target [3.170, -0.354] m, miss 0.003 m (base frame)
  ```

In RViz (`/ball_throw/markers`, frame `panda_link0`) the plugin shows:

- the floor, 0.935 m under the base;
- the shelf;
- every ball where it is, resting, held or flying;
- a blue disc and the ball name where each ball first touched the floor, next to the red
  targets of the testing node.

The balls are spawned once per launch. To run the sequence again, relaunch.

The plugin reads the tube tip as `panda_link7` + [0.14, 0, 0.127] m. Gazebo merges the fixed
`panda_link8` and the tube into link 7, and that offset is the controller EE. Both are
parameters of the plugin in the world file. `arduino/valve` and `arduino/duty_cycle` have
no effect in simulation.

## Offline tools

`throwing_check.py` runs the service model without ROS, starting from the `optimization`
block of `handtool_params.yaml`. Any entry can be overridden with `--set key=value`:

```bash
# one ball, target measured FROM THE THROWING POINT
./scripts/throwing_check.py --mass 0.144 --energy 1.413 --target 3.0 0.0 -1.51

# the same target written in the robot base frame
./scripts/throwing_check.py --mass 0.144 --energy 1.413 --target 3.0 0.0 -0.9 \
    --from 0.0044 -0.354 0.608

# table over distances: reach, angle and sensitivity of one ball
./scripts/throwing_check.py --mass 0.235 --energy 1.156 --sweep 0.5 2.5 9 --target-z -1.51

# compare branches, or go back to the valve model
./scripts/throwing_check.py --mass 0.144 --energy 1.413 --target 2.5 0 -1.51 \
    --case "flat:branch=low" --case "lob:branch=high"
./scripts/throwing_check.py --mass 0.1 --target 2.7 0.0 -1.29 --set energy_source=valve
```

## Ball experiments

Five balls were thrown 20 times each from a fixed tool pose with the valve open for
70000 us:

| Ball | Mass | Marker | Mean landing [m from the tool] | Launch speed | Launch energy |
|---|---|---|---|---|---|
| 5 cm | 48 g | cross | 4.76 | 5.86 m/s | 0.824 J |
| 6 cm | 85 g | circle | 4.27 | 5.45 m/s | 1.261 J |
| 7 cm | 144 g | triangle | 3.17 | 4.43 m/s | 1.413 J |
| 8 cm | 188 g | square | 2.50 | 3.73 m/s | 1.308 J |
| 9 cm | 235 g | asterisk | 1.98 | 3.14 m/s | 1.156 J |

- **`configs/ball_throwing.txt`** holds the raw data:
  - the base position in the floor frame, (190, 310, 935) mm;
  - the tool pose in the base frame;
  - the landing points of each ball, in a local frame shifted by a whole number of 50 cm
    floor tiles.
- **`configs/ball_landing_points.yaml`** has the same data in the tool frame:
  - **Frame:** origin at the tool, z up. x is horizontal along the line fitted through all
    100 points, and y is the lateral distance from that line.
  - **Header:** the tool pitch (32.80°) and height (1.543 m), and the yaw and offset of the
    fitted line, to go back to the floor frame.
  - **Per ball:** mass, mean launch speed and energy, mean and standard deviation of the
    landing point, and all the points.
- **How the energy is computed:** for each throw, it is the drag-free launch speed that
  covers the measured horizontal distance. The ball leaves the tool position at the tool
  pitch and lands with its centre one radius above the floor. E = ½ m v² for each throw,
  averaged over the 20 throws.
- **Assumptions:** the base axes are parallel to the floor axes, and there is no air drag
  and no spin. So the energies are a lower bound, and the gap is largest for the light, fast
  5 and 6 cm balls.

## Known limitations

- **Short reach with measured energy.** 32.8° is almost exactly the angle of maximum range
  for these energies, so each ball can only be thrown up to about its experimental
  distance.
- **Flat throws cover a narrow band.** Shorter targets need either a lob (steep: up to about
  80° with `branch: any` for close targets) or a flat throw. The flat throws only cover a
  narrow band; for the 9 cm ball it is 1.74–2.0 m from the tool, the reach at theta = 0 being
  the lower end. Check that the robot can hold the tube at the angles the service answers;
  the simulation shows it.
- **No drag anywhere.** The model, the energy identification and Gazebo are all drag-free.
  They are consistent with one another, but the real light balls fall shorter at large
  angles.
- **Steep throws far to the side.** In simulation, a throw at 82° pitch and -51° azimuth
  stopped at about 70° pitch, probably at a joint limit, and missed by 1.3 m. If the arm
  cannot reach the commanded orientation, narrow `bound_theta_deg` or the targets.
- **Keyboard input.** The testing node reads the keyboard through the terminal of
  `roslaunch`.
