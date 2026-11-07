# Phase 1 report: the robot, its controllers and the first drive in Gazebo

- **Milestone:** M1 Core AMR (Phases 1-6; this is the first)
- **Dates:** 2026-11-05
- **Branch / tag:** `phase-1-robot-description`; tag `phase-1` after the pull request merges
  (not yet created as of 2026-11-05)
- **Result:** our own tote-carrying robot (revision B) drives in an empty Gazebo world through
  ros2_control, inside the `/robot1` namespace, with a 2D scanner, an IMU and EKF odometry. The
  same launch file runs it on mock hardware without a simulator, which CI tests on every pull
  request. Every automated acceptance check passed (the design change of 2026-11-05 was then
  checked by hand); the demo GIF is left to the maintainer
  ([exercise 7](#exercise-7-run-the-demo-with-windows-and-record-the-gif)).

> **Note, 2026-11-07:** the tests this report describes (pytest, the launch tests), the lint
> checks, CI and the pre-commit hooks were removed on 2026-11-07, and there are no branches or
> pull requests any more ([ADR-008](../decisions/ADR-008-develop-on-main-without-automated-tests.md)).
> This report stays as the record of Phase 1; exercises that run `colcon test`, pytest, a launch
> test, pre-commit or CI no longer apply.

This report is written for someone who has read the [Phase 0 report](phase-00-foundation.md)
and knows basic Linux, Python and C++, but is new to robot descriptions, ros2_control and
Gazebo. It explains what was built, why each piece exists, what went wrong on the way, and how
to try every piece yourself. Paths are relative to the repository root unless they start with
`~/warehouse_ws`. "The plan" is [docs/PROJECT_PLAN.md](../PROJECT_PLAN.md); "plan §2.4" means
section 2.4 there.

The files were written and verified with AI assistance (Claude Code agents). One agent built
the five packages; a review found four medium and four low issues, which a second agent fixed;
three independent verifiers then reproduced CI, measured the acceptance criteria in headless
Gazebo, and reviewed the code against the upstream Jazzy sources; a polish round fixed their
six low findings, and a second, smaller round fixed what a check of the first one found. Every
command in [Try it yourself](#try-it-yourself) was run again after the first polish round (the
second changed no exercise), in a clean shell, except the windowed parts of exercise 7 (Gazebo
and RViz on a desktop, teleop by hand, screen recording). Of exercise 7, the box-spawn command
was run against the headless simulation, and the GIF-conversion and size-check commands in a
Docker container (Ubuntu 24.04 with ffmpeg and gifsicle from `apt install`, which the laptop
does not have) on a generated test clip instead of a screen recording. The teleop command was
tested earlier with keys fed through a pseudo-terminal.

After the owner's design review (2026-11-05) one more agent gave the robot a 360° lidar puck on
its deck and a new look ([concept 14](#14-where-the-lidar-sits), timeline). It re-ran exercises
2 and 4 and the measurements behind exercise 1's scan check with the launch test's own set-up.
The test suite was not run with colcon, because checks are paused (ADR-004); only the
description tests ran, directly with pytest. The test counts in exercise 1 and in
[Measured results](#measured-results) follow from the tests that were added and removed, and are
confirmed when checks resume.

> **Words you will meet** (full list in the [Glossary](#glossary))
>
> - **URDF / xacro:** the XML file that describes a robot's parts and joints, and the macro
>   language that generates it.
> - **link / joint:** a rigid part of the robot, and the connection between two links (fixed,
>   or moving like a wheel axle).
> - **TF (tf2):** ROS's system of named coordinate frames and the transforms between them.
> - **ros2_control:** the framework between controllers (software that decides what the motors
>   do) and hardware (or a simulator, or a fake).
> - **gz-transport:** Gazebo's own messaging system, separate from ROS topics; a **bridge**
>   copies messages between the two.
> - **QoS (quality of service):** per-topic delivery rules, such as "reliable" or "best effort".
> - **odometry:** the robot's estimate of how far it has moved, from its wheels and sensors.
> - **EKF (extended Kalman filter):** an algorithm that fuses several noisy sensors into one
>   estimate, weighted by how much each is trusted.
> - **sim time:** the simulator's clock, published on `/clock`, which nodes use instead of the
>   wall clock.
> - **launch_testing:** ROS's framework for tests that start real nodes and check them while
>   they run.

## Contents

- [TL;DR](#tldr)
- [What was built](#what-was-built)
- [Concepts](#concepts)
  1. [URDF, xacro and one source of truth](#1-urdf-xacro-and-one-source-of-truth)
  2. [Links, joints, collisions and inertia](#2-links-joints-collisions-and-inertia)
  3. [TF and robot_state_publisher](#3-tf-and-robot_state_publisher)
  4. [ros2_control: hardware, controller manager, controllers](#4-ros2_control-hardware-controller-manager-controllers)
  5. [The controllers' configuration](#5-the-controllers-configuration)
  6. [Gazebo sensors and gz-transport](#6-gazebo-sensors-and-gz-transport)
  7. [The ros_gz bridge and QoS](#7-the-ros_gz-bridge-and-qos)
  8. [Differential-drive kinematics and wheel odometry](#8-differential-drive-kinematics-and-wheel-odometry)
  9. [The EKF, and why only the yaw rate from the IMU](#9-the-ekf-and-why-only-the-yaw-rate-from-the-imu)
  10. [Namespaces and the /tf remaps](#10-namespaces-and-the-tf-remaps)
  11. [Sim time and stamped commands](#11-sim-time-and-stamped-commands)
  12. [Launch files](#12-launch-files)
  13. [launch_testing](#13-launch_testing)
  14. [Where the lidar sits](#14-where-the-lidar-sits)
- [What happened](#what-happened)
- [Measured results](#measured-results)
- [Try it yourself](#try-it-yourself)
- [Pitfalls and lessons](#pitfalls-and-lessons)
- [Glossary](#glossary)
- [Further reading](#further-reading)
- [What comes next](#what-comes-next)

## TL;DR

Phase 1 built the robot every later phase drives:

- **A robot of our own**, described once in `warehouse_description/config/robot.yaml` and turned
  into a URDF by xacro: a 0.64 x 0.48 m, 35 kg differential-drive base in safety yellow and
  charcoal, with a 360° lidar puck on its deck (10 Hz) and an IMU (100 Hz). Revision A is the
  rolling chassis, revision B adds the sensors.
- **One description, three kinds of hardware** through ros2_control: Gazebo (`hardware:=gz`),
  a fake that needs no simulator (`hardware:=mock`), and a reserved `real` that the build
  refuses until a real driver exists.
- **Namespaced from day one:** every node, topic, TF tree and Gazebo topic of the robot lives
  under `/robot1`, so Phase 9 can add more robots without rewiring.
- **Odometry from an EKF** that fuses the wheels with the IMU's yaw rate.
- **Tests that start the real system:** a mock-hardware launch test runs in CI on every pull
  request; a Gazebo launch test checks every Phase 1 acceptance criterion locally. CI cannot
  render, so a second kind of test reads the Gazebo-side files without starting Gazebo.

Measured in Gazebo: sensor rates 10 / 100 / 50 Hz, 0 scanner self-hits, wheel odometry within
0.01 % of ground truth over 10 m, EKF within 0.09-0.42 % (the limit is 1 %).

The most important lesson: **check what a component actually does, not what its name or
description promises.** The "frictionless" casters had friction, the documented fallback of a
braking limit was wrong, a self-hit test passed for a blind scanner, and a launch file that
looked clean left Gazebo running after every test. Each was found by measuring.

## What was built

| Piece | Files | Purpose |
|---|---|---|
| Robot description | `warehouse_description/config/robot.yaml`, `urdf/*.xacro` | The robot's geometry, masses, sensors and limits from one YAML file; URDF for `gz` or `mock` hardware |
| Controller configuration | `warehouse_description/config/controllers.yaml` | Controller manager (100 Hz), `joint_state_broadcaster`, `diff_drive_controller`, `imu_sensor_broadcaster` |
| Description tests | `warehouse_description/test/test_description.py` | 19 pytest cases: URDF validity, frames, mass, friction, namespaced topics, the lidar's full circle and scan plane, every part inside the outline, one colour per link, `controllers.yaml` against `robot.yaml` |
| Gazebo world and spawning | `warehouse_gazebo/worlds/empty.sdf`, `launch/gazebo.launch.py`, `launch/spawn_robot.launch.py` | Starts Gazebo, spawns a robot, bridges `/clock` |
| Bridge configuration | `warehouse_gazebo/config/robot_bridge.yaml` | `scan` and `ground_truth/odom` from Gazebo to ROS, one file for every robot |
| Gazebo configuration tests | `warehouse_gazebo/test/test_gazebo_config.py` | 4 pytest cases without rendering, so CI runs them: the systems every world loads, the bridge file against the URDF, the caster friction after Gazebo's URDF conversion |
| Odometry EKF | `warehouse_navigation/config/ekf.yaml`, `launch/ekf.launch.py` | `robot_localization` EKF: `odom` and `odom -> base_footprint` |
| Bring-up | `warehouse_bringup/launch/sim.launch.py`, `launch/robot.launch.py`, `rviz/robot.rviz` | Entry point: world, robot and RViz; the robot scope on its own |
| System tests | `warehouse_system_tests/test/test_mock_bringup.py`, `test/test_gz_bringup.py`, `warehouse_system_tests/ros_helpers.py` | launch_testing: mock (CI) and Gazebo (local) |
| CI | `.github/workflows/ci.yml` | `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, `WAREHOUSE_SKIP_GZ_TESTS=1` |
| Decisions | `docs/decisions/ADR-005-custom-tote-amr.md`, ADR-004 addition | Why our own robot, ros2_control and an EKF; what CI skips and why |

Each package has a README that explains every file in it.

## Concepts

### 1. URDF, xacro and one source of truth

**What it is.** URDF (Unified Robot Description Format) is an XML file that describes a robot
as a tree of *links* (rigid parts) connected by *joints*. Every ROS tool that needs to know the
robot's shape reads it: `robot_state_publisher`, RViz, ros2_control, Gazebo. Plain URDF has no
variables or loops, so nobody writes it by hand; *xacro* (XML macros) adds properties, maths,
macros, conditionals and file includes, and generates the URDF.

**How it works here.** Open `warehouse_description/urdf/robot.urdf.xacro`:

- `<xacro:arg name="namespace">` and `<xacro:arg name="hardware">` are the two inputs, set on
  the command line (`xacro robot.urdf.xacro namespace:=robot1 hardware:=gz`) or by the launch
  file.
- `xacro.load_yaml(...)` reads `config/robot.yaml` into the property `robot`, so
  `${robot.drive.wheel_radius}` is 0.075. **No dimension in the URDF is typed twice.**
- `xacro.fatal(...)` stops the build with a message: `hardware:=real` is reserved for a future
  real driver, and an unknown value such as `gazebo` is refused.
- The robot is assembled from macros in separate files, one macro per part: `base.xacro`
  (frames, collision and inertia), `body.xacro` (chassis, yellow shell, bumpers), `deck.xacro`
  (tote tray, emergency stop, status light), `drive.xacro` (wheels and casters), `lidar.xacro`
  (the lidar puck, a macro a later sensor can reuse), `imu.xacro`, `ros2_control.xacro` and
  `gazebo.xacro` (ground truth). `inertia.xacro` holds the inertia macros and `materials.xacro`
  the colours.

The project rule is **"geometry is generated, tuning is validated"** (plan §2.6). Geometry flows
from `robot.yaml` into the URDF automatically. Files that cannot read `robot.yaml` themselves,
such as `controllers.yaml` (which needs the wheel radius and separation), are *checked* against
it by `test/test_description.py`. Change the wheel radius in one place and forget the other,
and the test fails ([exercise 6](#exercise-6-tuning-is-validated)).

**Why industry does it.** A robot's dimensions appear in many places: the model, the
controller, the navigation footprint, the safety fields. Copies drift apart, and a drive
controller with the wrong wheel radius reports wrong odometry without any error. One source plus
tests for the copies turns a silent bug into a red test.

### 2. Links, joints, collisions and inertia

**What it is.** Each link can have three descriptions: a *visual* (what you see in RViz and
Gazebo), a *collision* (what the physics engine touches) and an *inertial* (mass, centre of
mass and the inertia tensor, which says how hard the part is to rotate). Joints connect a parent
link to a child link: `fixed` (rigid), `continuous` (spins forever, like a wheel), `revolute`
(limited angle) or `prismatic` (slides).

**How it works here.**

- **Frames.** `base_footprint` is the root of the tree, on the floor under the robot centre
  (REP-105). It has no inertia: KDL, the kinematics library `robot_state_publisher` uses, does
  not support inertia on the root link. A fixed joint lifts `base_link` 0.075 m, onto the drive
  axle, which is also the geometric centre, so the robot turns in place inside its own circle.
- **Visuals vs collision.** The body is drawn from simple boxes, cylinders and a sphere (a skirt, yellow
  panels with a well for each wheel, bumpers that wrap round the corners, the deck top, a tote
  tray, a stop button, a status light) but collides as one box. Contacts do not care about these
  details, and one box is cheap for the physics engine.
- **One colour per link.** RViz draws every visual of a link in the colour of the link's first
  visual, so each part with a colour of its own is its own link: `shell_link` (yellow panels),
  `front_bumper_link`, `tote_tray_link` and so on, on fixed joints and without mass. Gazebo merges
  them into `base_footprint`, so the physics do not change.
  [Problem 15](#15-rviz-drew-the-whole-body-in-one-colour) tells how this was found.
- **Sphere wheels.** The wheels *look* like cylinders but *collide* as spheres. A cylinder
  touches the floor along a line; when the robot turns in place, the ends of that line drag,
  and in the prototype that skewed in-place yaw odometry by 8-9 %. A sphere touches in one point
  under the wheel centre (plan R13).
- **Casters** are frictionless spheres (`<mu1>0.0</mu1>` in a `<gazebo reference>` block): they
  carry load and never resist a turn. How that setting was silently lost at first is
  [problem 3](#3-the-frictionless-casters-had-friction).
- **Inertia** comes from three macros in `inertia.xacro` (box, cylinder, sphere of uniform
  density). The mass budget is in `warehouse_description/README.md`: body 32.6 kg with its
  centre of mass low (the battery), two 1 kg wheels, two 0.2 kg casters; 35.0 kg in total with
  the centre of mass 0.14 m above the floor. Sensor links have no mass.
- **Lumping.** Gazebo does not simulate fixed joints. When it converts the URDF to SDF (its
  own format), it merges every link on a fixed joint into its parent. `gz sdf -p` shows the
  result: one link `base_footprint` with 33.0 kg (body plus casters) and two wheel links of
  1 kg ([exercise 2](#exercise-2-the-robot-description-by-hand)).

**Why industry does it.** Simulation is only as good as its model. A wrong inertia changes how
the robot accelerates and turns; a tiny one makes the physics unstable. A collision shape that
is too detailed slows the simulation; one that is wrong (the cylinder wheels) produces errors
that look like a software bug in odometry.

### 3. TF and robot_state_publisher

**What it is.** TF (the `tf2` library) keeps track of coordinate frames over time. Every
transform is published as a message on the topic `/tf` (moving parts) or `/tf_static` (fixed
parts, sent once and kept for late subscribers). Any node can ask "where was `lidar_front_link`
relative to `odom` at time t?" and tf2 chains the transforms.

**How it works here.** The Phase 1 tree (plan §2.4). Each arrow names the node that publishes
the transform; RSP is `robot_state_publisher`:

```text
odom ──(EKF)──> base_footprint ──(RSP)──> base_link
                                          ├── lidar_front_link   (0.25, 0, 0.34 m: the scan plane)
                                          ├── imu_link
                                          ├── left_wheel_link, right_wheel_link  (joint_states)
                                          │   └── left_wheel_hub_link, right_wheel_hub_link
                                          ├── front_caster_link, rear_caster_link
                                          └── shell_link, front_bumper_link, rear_bumper_link,
                                              tote_tray_link, emergency_stop_link,
                                              status_light_link, lidar_front_mount_link
```

The last group and the two hubs are the visual-only parts of concept 2: fixed joints, so
`robot_state_publisher` sends them once on `tf_static`. RViz's TF display starts with them hidden
(`rviz/robot.rviz`); tick them under TF > Frames to see them.

- **`robot_state_publisher` (RSP)** reads the URDF from its `robot_description` parameter. It
  publishes the fixed joints once on `tf_static`, and the wheel joints on `tf` from the wheel
  angles `joint_state_broadcaster` reports on `joint_states` (at most 20 times a second, RSP's
  default `publish_frequency`). It also republishes the
  URDF on the topic `robot_description` with *transient local* durability, so a node that
  starts later (the controller manager, the Gazebo spawner, RViz) still receives it.
- **`odom -> base_footprint`** comes from the EKF (concept 9). The drive controller could
  publish it too, but two publishers of one transform make the robot jump between them, so the
  controller runs with `enable_odom_tf: false` (plan R5).
- **`map -> odom`** arrives in Phase 2 from AMCL. Gazebo's world frame is the `map` frame, which
  is why the ground-truth odometry is reported in `map`.
- Axes follow REP-103: x forward, y left, z up; angles are counter-clockwise positive.

**Why industry does it.** Every sensor reports data in its own frame. With TF, a navigation
stack can turn a scan point into a map position without knowing where the scanner is mounted;
moving the scanner means changing a few numbers. The design change of concept 14 moved it from a
corner of the body onto the deck, and nothing that uses the scan had to change: it looks the
lidar's pose up in TF.

### 4. ros2_control: hardware, controller manager, controllers

**What it is.** ros2_control separates *what to do* from *how to talk to the motors*:

```text
  controllers (plugins)         diff_drive_controller   joint_state_broadcaster   imu_sensor_broadcaster
        │ claim command interfaces, read state interfaces
  controller manager            one real-time loop: read() → update every controller → write()
        │
  resource manager + hardware   GazeboSimSystem (gz) │ GenericSystem (mock) │ your driver (real)
```

- A **hardware interface** (a plugin) exports *state interfaces* (values it can read: wheel
  position and velocity, ten IMU values) and *command interfaces* (values it accepts: wheel
  velocity). The `<ros2_control>` block in the URDF lists them, in `ros2_control.xacro`.
- The **controller manager** loads the hardware and runs the control loop at `update_rate`
  (100 Hz here). In every cycle it reads all states, updates every active controller, and
  writes the commands.
- **Controllers** are plugins too. `diff_drive_controller` claims the two wheel velocity
  command interfaces (only one controller may claim a command interface). *Broadcasters* claim
  nothing; they read states and publish them: `joint_state_broadcaster` on `joint_states`,
  `imu_sensor_broadcaster` on `imu`.
- A **spawner** (`ros2 run controller_manager spawner <name>`) is a short-lived process that
  asks the controller manager to load, configure and activate one controller, then exits. The
  launch log shows `spawner-…: process has finished cleanly` once each controller is active.

**How it works here.** `ros2_control.xacro` exports the same interfaces for every hardware
option; only the `<plugin>` changes:

| `hardware:=` | Plugin | Who runs the controller manager | Notes |
|---|---|---|---|
| `gz` | `gz_ros2_control/GazeboSimSystem` | The `gz_ros2_control-system` plugin, **inside the Gazebo process**, started when the robot is spawned | Gazebo moves the joints and fills in the IMU; the controller manager uses sim time |
| `mock` | `mock_components/GenericSystem` | `ros2_control_node`, a normal ROS node started by `robot.launch.py` | No physics: with `calculate_dynamics` the commanded wheel velocity is integrated into wheel position, so odometry moves. The IMU keeps its initial values (identity orientation, zero rates) |
| `real` | not yet (plan A1) | | xacro refuses it |

The launch path is the same for both; `robot.launch.py` only adds `ros2_control_node` for mock
hardware and the Gazebo spawn for `gz`.

**Why industry does it.** The controllers that drive a simulated robot drive the real one
unchanged. Mock hardware lets CI check the whole launch, controller and topic wiring in seconds,
without a simulator or a GPU. Writing a driver for a real robot means writing one hardware
plugin, not touching the controllers.

### 5. The controllers' configuration

**What it is.** `warehouse_description/config/controllers.yaml` holds the parameters of the
controller manager and of each controller. Both Gazebo's controller manager and
`ros2_control_node` load this one file.

**How it works here.** The details worth reading in the file:

- **`/**/` in every key.** A ROS parameter file is keyed by the node's full name. A plain
  `controller_manager:` key matches only `/controller_manager`; in `/robot1` the controllers
  would silently start with *default* parameters (plan R2). `/**/controller_manager` matches
  the node in any namespace.
- **Private topics, remapped by the spawner.** Controllers use private names: the drive
  controller subscribes to `~/cmd_vel`, which is `/robot1/diff_drive_controller/cmd_vel`.
  `robot.launch.py` passes `--controller-ros-args "-r ~/cmd_vel:=cmd_vel -r ~/odom:=odom/wheel"`
  to the spawner, so the robot's topics are `/robot1/cmd_vel` and `/robot1/odom/wheel`
  (plan §2.3). The older `<ros><remapping>` in the URDF plugin tag is deprecated in
  ros2_control 4.48.
- **Unprefixed frames.** `tf_frame_prefix_enable: false`. The default would publish
  `robot1/odom`, which matches nothing in our tree (plan D7: namespaces keep robots apart, not
  frame names).
- **Covariances.** `pose_covariance_diagonal` and `twist_covariance_diagonal` tell the EKF how
  much to trust the wheels; the IMU covariances are σ² of the simulated noise. All-zero
  defaults would claim perfect sensors.
- **Limits with signs.** Each acceleration limit carries the sign of the velocity change it
  limits: speeding up forward is +0.8 m/s², braking from forward -1.5 m/s², speeding up in
  reverse -0.8 m/s², braking from reverse +1.5 m/s². All four are set on purpose: see
  [problem 8](#8-braking-from-reverse-used-the-wrong-limit).
- **`enforce_command_limits: true`** on the controller manager clamps every wheel command to
  the URDF joint limit (22 rad/s). Jazzy's default is `false`, which ignores those limits.
- **`cmd_vel_timeout: 0.5`.** If no command arrived for 0.5 s, the controller brakes. How
  "arrived" is judged is concept 11. *Note, 2026-11-06:* the value is now 0.15 s, at the
  owner's request after driving by teleop (each key press kept the robot going for half a
  second). Every number measured in this report was measured with 0.5 s. With 0.15 s, every
  `cmd_vel` publisher must publish at 10 Hz or more while the robot should move
  ([log, 2026-11-06](../PROJECT_STATUS.md#ideas-and-decisions-log)).

**Why industry does it.** Controller parameters are safety-relevant. Hard limits in the
controller are the last line of defence below the navigation stack's own limits (Phase 3), so
they must be right in both directions of travel, and tested.

### 6. Gazebo sensors and gz-transport

**What it is.** Gazebo Harmonic (gz-sim 8) is a separate program with its own message system,
*gz-transport*. Its topics look like ROS topics (`/robot1/scan`) but are a different network:
`ros2 topic list` does not show them, and `gz topic -l` does not show ROS topics. A *system*
is a Gazebo plugin that adds a capability to the world, such as physics or sensor rendering.

**How it works here.**

- **The world loads only what it lists.** A world file that names any `<plugin>` gets no
  default systems, so `warehouse_gazebo/worlds/empty.sdf` lists all five the robot needs:
  physics (1 ms steps), user commands (the spawn service), scene broadcaster, sensors (rendered
  with ogre2) and IMU. The package README explains each.
- **The scanner is a `gpu_lidar`**: Gazebo renders the scene from the scanner's position and
  reads the depth. That is why it needs a GPU or EGL, why it sees *visuals* and not collisions,
  and why CI cannot run it. It also sees the robot's own visuals, so every part of the robot
  stays out of its scan plane except the lidar's own head, which `range_min` (0.10 m) hides
  (concept 14).
- **gz topics are global.** All robots in one world share one gz-transport network, so a fixed
  `<topic>scan</topic>` would merge every robot's scans into one stream. The xacro prefixes
  every gz topic with the namespace: `/robot1/scan`, `/robot1/imu`,
  `/robot1/ground_truth/odom` (plan R12).
- **`<gz_frame_id>`.** Without it, Gazebo stamps messages with a scoped name such as
  `robot1::base_footprint::lidar_front`, which is not a TF frame. Each sensor sets its URDF
  link name instead. sdformat prints a warning that the element is not in the SDF spec and
  copies it through; the sensors read it, as the tests confirm.
- **The IMU is not bridged.** `gz_ros2_control` reads it (matched by the sensor name
  `imu_sensor`) into the IMU state interfaces, and `imu_sensor_broadcaster` publishes it: the
  path a real IMU driver would use.
- **Ground truth.** `gazebo.xacro` adds Gazebo's `OdometryPublisher` system, which reports the
  robot's true world pose. Tests compare odometry against it; the robot stack never uses it.
- **`GZ_PARTITION`.** gz-transport discovers peers on the network like DDS does. Two
  simulations on one machine (or one LAN) see each other unless they use different partitions;
  the Gazebo test sets its own.

**Why industry does it.** Simulated sensors with realistic placement, noise and rendering limits
catch design errors (a scanner that sees its own body) long before hardware exists, and keeping
the simulator's transport separate from ROS means the robot stack cannot tell whether a message
came from a simulator or from hardware.

### 7. The ros_gz bridge and QoS

**What it is.** `ros_gz_bridge`'s `parameter_bridge` copies messages between gz-transport and
ROS 2, converting types (`gz.msgs.LaserScan` to `sensor_msgs/msg/LaserScan`). QoS settings
decide how a ROS topic delivers messages; a publisher and a subscriber connect only if their
QoS settings are compatible.

**How it works here.**

- **One file for every robot.** `warehouse_gazebo/config/robot_bridge.yaml` uses *relative*
  names on both sides (`scan`), and `spawn_robot.launch.py` runs the bridge in the robot's
  namespace with `expand_gz_topic_names: true`. The bridge then expands the gz name like a ROS
  name: `scan` becomes `/robot1/scan` on the gz side and on the ROS side. A second robot reuses
  the file unchanged.
- **Scan QoS: `SENSOR_DATA`** (best effort, keep the last 5, volatile). Best effort means a lost
  message is not resent: for a 10 Hz sensor, the next scan is more useful than a late copy of
  the last one. The rule to remember: a *reliable* subscriber does not connect to a *best
  effort* publisher (`ros2 topic info -v` shows the mismatch). RViz's scan display therefore
  uses Reliability Policy "Best Effort" (`rviz/robot.rviz`). The opposite direction works:
  `ros2 topic info -v /robot1/cmd_vel` shows the drive controller subscribing best effort, so
  any publisher can reach it.
- **`/clock` once per world.** `gazebo.launch.py` runs a second bridge, `clock_bridge`, for
  `/clock` without a namespace: every robot shares one clock.
- **`robot_description` is transient local**, as noted in concept 3: a late subscriber still
  gets the URDF.

**Why industry does it.** QoS is how ROS 2 trades latency against completeness per topic. Sensor
streams use best effort; commands, state and maps use reliable, and maps also transient local.
Picking the wrong one either drops important messages or leaves a subscriber silently
disconnected.

### 8. Differential-drive kinematics and wheel odometry

**What it is.** A differential drive has two independently driven wheels on one axle. With
wheel radius r = 0.075 m, wheel separation L = 0.40 m and wheel speeds ω_l, ω_r (rad/s):

```text
forward speed      v = r (ω_r + ω_l) / 2
turn rate          ω = r (ω_r − ω_l) / L
and back:        ω_l = (v − ω L / 2) / r      ω_r = (v + ω L / 2) / r
```

It cannot move sideways: its sideways speed is always zero (a *nonholonomic* constraint).

**How it works here.** `diff_drive_controller` does both directions:

- **Commands to wheels.** A `TwistStamped` on `cmd_vel` (v = `twist.linear.x`,
  ω = `twist.angular.z`) is limited (speed, acceleration, deceleration), converted to two wheel
  speeds, and written to the wheel command interfaces. The hard limits fit together: 1.2 m/s
  while turning at 2 rad/s needs (1.2 + 2 x 0.2) / 0.075 = 21.3 rad/s, just under the 22 rad/s
  wheel limit.
- **Wheels to odometry.** From the wheel positions it integrates the pose (x, y, yaw) and
  publishes `odom/wheel` (`nav_msgs/Odometry`, 50 Hz) with `frame_id: odom` and
  `child_frame_id: base_footprint`. This is *dead reckoning*: errors add up and are never
  corrected, which is why Phase 2 adds AMCL against a map.

In simulation there is no wheel slip and the sphere wheels touch the floor in exactly one
point, so wheel odometry is nearly perfect: 0.01 % over 10 m and 0.00 % over a 370° turn. A real
robot is worse, which is what the EKF and later AMCL are for.

**Why industry does it.** Wheel odometry is cheap, high-rate and smooth, so it is the backbone
of every wheeled robot's motion estimate, even though it drifts. Knowing the formulas helps you
spot a wrong wheel radius (distance off by a constant factor) or separation (turns off by a
constant factor) in a test result.

### 9. The EKF, and why only the yaw rate from the IMU

**What it is.** An extended Kalman filter keeps a state estimate (position, heading, speeds) and
its uncertainty (a covariance matrix). Every cycle it *predicts* the state forward with a motion
model, then *corrects* it with each measurement, weighting the measurement by its covariance:
a sensor with a small variance pulls the estimate harder.

**How it works here.** `robot_localization`'s `ekf_node`, configured in
`warehouse_navigation/config/ekf.yaml` (plan D3):

- **Inputs** are selected per sensor with a 15-entry list (x, y, z, roll, pitch, yaw, vx, vy,
  vz, vroll, vpitch, vyaw, ax, ay, az). From `odom/wheel` the EKF fuses vx, **vy** and the yaw
  rate; from `imu` only the yaw rate.
- **Why vy, which is always zero?** A diff drive cannot slide, so the controller reports vy = 0
  with a small variance. That zero is a real measurement. Without it nothing observes vy, and
  the y variance of the output grew without bound: about 4e4 m² after 30 s standing still in
  Gazebo, 7e5 m² after a minute. With it, y stays at 2-3 m², like x. robot_localization's
  configuration guide recommends exactly this ([problem 10](#10-the-ekfs-sideways-uncertainty-grew-without-bound)).
- **Why only the IMU's yaw rate?** The orientation from Gazebo's IMU is noise-free ground
  truth; fusing it would make localization look better than any real robot could manage, so it
  gets a covariance of 1000 and is not fused. Accelerations would have to be integrated twice
  (noise grows fast, and gravity leaks in through any tilt); a planar robot at warehouse speeds
  does not need them. The gyro's yaw rate, however, is exactly what wheels get wrong on a real
  robot (slip while turning), so it is the useful complement.
- **Weights.** Wheel yaw-rate variance 0.01 against IMU yaw-rate variance 4e-6 (the gyro noise
  σ of 0.002 rad/s, squared): the IMU dominates the heading, the wheels give the speed.
- **`two_d_mode: true`** keeps z, roll and pitch at zero. `world_frame: odom` makes the EKF
  publish `odom -> base_footprint`, and its output `odometry/filtered` is remapped to `odom`,
  the name Nav2 expects.

**What the numbers say.** In Gazebo the EKF's end point is 0.09-0.42 % off over 10 m, while the
wheel odometry alone is 0.01 % off; both are well inside the 1 % limit. The final review
measured why the EKF does worse: it integrates the *twist* the controller reports, and in
Gazebo that twist reads about 0.2 % high (1.002 m/s at a true 1.000 m/s). The controller
manager's update stamps jitter between 9 and 11 ms (they come from its ROS clock, which follows
`/clock`, not from Gazebo's 1 ms physics steps), and the controller computes speed as position
change divided by stamp interval. Each update covers exactly 10 ms of physics, so a 9 ms stamp
interval reads 11 % high and an 11 ms one only 9 % low: the jitter does not average out, it
pushes the mean up. The controller's own pose integrates positions and is unaffected. Removing
the IMU did not change the error, so the IMU is not the cause. On mock hardware the EKF ends
close to the wheel odometry once the robot stops (0.01-15 mm apart after eight short drives of
0.3-0.7 m). Phase 2's localization evaluation looks at this again.

**Mock caveat.** On mock hardware the IMU always reports a yaw rate of zero, and the EKF trusts
the IMU's yaw rate far more than the wheels'. In mock mode the EKF follows straight driving but
not turns: mock hardware tests wiring, not localization.

**Why industry does it.** Every deployed AMR fuses wheels and an IMU, because each fails
differently: wheels slip, gyros drift slowly. Choosing *which* fields to fuse matters as much as
the filter itself; fusing a ground-truth value in simulation is the classic way to fool
yourself.

### 10. Namespaces and the /tf remaps

**What it is.** A ROS namespace is a prefix for node and topic names. A *relative* name (`scan`)
is resolved inside the node's namespace (`/robot1/scan`); an *absolute* name (`/clock`) is not.
A *remap* (`-r from:=to`) renames a topic for one node.

**How it works here.**

- `robot.launch.py` wraps the whole robot in `PushROSNamespace('/robot1')`, and every name in
  the robot's files is relative. The nodes become `/robot1/robot_state_publisher`,
  `/robot1/ekf_filter_node`, `/robot1/controller_manager`, and the topics `/robot1/scan`,
  `/robot1/odom`, ...
- **tf2 uses the absolute `/tf` and `/tf_static`**, which a namespace does not move. So the
  launch file adds `SetRemap('/tf', 'tf')` and `SetRemap('/tf_static', 'tf_static')`: every
  robot node started after them publishes and listens on `/robot1/tf`. Each robot has its own
  TF tree with unprefixed frame names (`odom`, `base_footprint`), the Nav2 convention for
  multiple robots (plan D7).
- **Tools need the same remap.** RViz runs in `/robot1` with the remaps (`sim.launch.py`).
  `view_frames` and `rqt_tf_tree` take `--ros-args -r /tf:=/robot1/tf -r
  /tf_static:=/robot1/tf_static`. Teleop takes `-r __ns:=/robot1`, which moves the whole node
  into the namespace so its relative `cmd_vel` becomes `/robot1/cmd_vel`.
- **The namespace is absolute** (`'/' + namespace`), because the gz topics and the controller
  manager inside Gazebo get absolute names from the xacro. If someone includes
  `robot.launch.py` inside another namespace, the ROS side must stay at `/robot1` to meet them.
  `sim.launch.py` gives RViz the same absolute namespace.
- **One quirk, explained:** `ros2 topic info /tf` lists one publisher, `diff_drive_controller`.
  The controller always creates a `/tf` publisher and sends nothing on it while
  `enable_odom_tf` is false. The controller manager never sees the launch remaps (it runs inside
  Gazebo, or, for mock hardware, starts before them), so that idle publisher stays on the global
  `/tf`. Measured: 0 messages on `/tf` and `/tf_static` over 10 s.

**Why industry does it.** Namespacing from the first day costs a few remaps; adding it to a
working single-robot system later means hunting down every hard-coded `/scan` and `/odom`.
Separate TF trees mean one robot's transforms can never corrupt another's.

### 11. Sim time and stamped commands

**What it is.** With `use_sim_time: true`, a node's clock follows the `/clock` topic instead of
the computer's clock. Gazebo starts its clock at zero and may run slower or faster than real
time; every timestamp, timeout and rate in the system must then use sim time.

**How it works here.**

- `sim.launch.py` sets `use_sim_time` for every node with `SetParameter`; `robot.launch.py` does
  the same for its own nodes, so it also works on its own. The default is computed from the
  hardware: `true` for `gz`, `false` for `mock` (mock hardware has no simulator clock).
  `gz_ros2_control` sets it for the controllers inside Gazebo itself.
- **Jazzy's `diff_drive_controller` accepts only `TwistStamped`**, and it judges a command's age
  from the message's header stamp: if `now - stamp` exceeds `cmd_vel_timeout`, it brakes. A
  command stamped with the *wall clock* (about 1.79e9 s since 1970) looks like it comes from
  the far future in a simulation whose clock reads a few hundred seconds. Its age is negative,
  it never times out, and the robot keeps driving after you release the key (plan R1).
  [Exercise 5](#exercise-5-the-wall-clock-trap) shows it.
- That is why teleop runs with `-p stamped:=true -p use_sim_time:=true`, and why every test
  stamps its commands with the test node's own (sim-time) clock.
- Rates and durations in the tests come from message stamps, so they hold even if Gazebo runs
  slower than real time on a busy machine.

**Why industry does it.** A simulated robot that runs on two clocks behaves differently from the
real one in exactly the places that matter: timeouts, sensor fusion, and safety stops. Stamped
commands let the receiver reject stale commands, which a plain `Twist` cannot do.

### 12. Launch files

**What it is.** A Python launch file returns a `LaunchDescription`: a list of *actions* (start a
node, include another launch file, set a parameter for everything that follows) and
*substitutions* (values resolved when the launch runs, such as `LaunchConfiguration('namespace')`).

**How it works here.** Three files split the system into scopes:

- **`warehouse_gazebo/launch/gazebo.launch.py` (world scope)** starts Gazebo with
  `ExecuteProcess(cmd=['gz', 'sim', ...])` and the `/clock` bridge. `on_exit=Shutdown(...)`
  ends the whole launch when Gazebo exits (closing its window ends everything). It does not use
  `ros_gz_sim`'s `gz_sim.launch.py`; [problem 5](#5-orphaned-gazebo-servers-broke-later-runs)
  explains why.
- **`warehouse_bringup/launch/robot.launch.py` (robot scope)** builds its actions in an
  `OpaqueFunction`: plain Python that runs when the launch starts, with access to the resolved
  arguments. That allows ordinary `if hardware == 'mock':` code and running xacro in-process,
  so a bad `hardware` value fails the launch at once with xacro's message. Inside one
  `GroupAction`, the order matters: `SetParameter(use_sim_time)`, `PushROSNamespace`, then
  `ros2_control_node` (mock only), then the `/tf` remaps, then everything else. Actions such as
  `SetRemap` apply to the nodes that come *after* them in the group.
- **`warehouse_bringup/launch/sim.launch.py` (entry point)** is mostly declarative: arguments,
  `IncludeLaunchDescription` of the two files above, a computed default (`use_sim_time`
  defaults to `EqualsSubstitution(hardware, 'gz')`), and RViz in a `GroupAction` with
  `IfCondition`. Inside that group a small `OpaqueFunction` reads the namespace and pushes it as
  an absolute name, as `robot.launch.py` does (concept 10), so RViz stays with the robot even
  when `sim.launch.py` is itself included inside another namespace.

Two launch details tripped us up: a list of substitutions must be flat
([problem 4](#4-failed-to-normalize-given-item-of-type-list)), and `ros2 launch` refuses an
empty argument (`namespace:=`), so "no namespace" is written `namespace:=/`.

**Why industry does it.** Splitting world and robot scope is what makes Phase 9 small: a fleet
launch starts one world and includes `robot.launch.py` once per robot with a different
namespace and spawn pose.

### 13. launch_testing

**What it is.** `launch_testing` runs a launch description, then runs ordinary `unittest` test
cases against the running system, then shuts it down and runs a second set of tests on the
results (exit codes, logs).

**How it works here.** Open `warehouse_system_tests/test/test_mock_bringup.py`:

- `generate_test_description()` includes `sim.launch.py` with `hardware:=mock` and returns it
  with `ReadyToTest()`: the test cases start as soon as the processes are launched.
- `TestMockBringup` is a normal `unittest.TestCase`. `setUpClass` creates an `rclpy` node; each
  test subscribes, publishes and calls services like a user with `ros2` commands, and waits
  with `spin_until(...)` rather than fixed sleeps. Shared helpers live in the installed Python
  module `warehouse_system_tests.ros_helpers`, which also reads the robot's numbers from
  `robot.yaml` and `controllers.yaml`, so no test repeats a value.
- `@launch_testing.post_shutdown_test()` marks `TestShutdown`, which runs after one SIGINT:
  `assertExitCodes(proc_info)` requires every process to exit with code 0. The Gazebo test
  also checks that no process carrying its `GZ_PARTITION` survived.
- `CMakeLists.txt` registers each file with `add_launch_test(...)`, using
  `run_test_isolated.py` from `ament_cmake_ros` as the runner: each test file gets its own free
  `ROS_DOMAIN_ID`, so tests running in parallel never see each other.
- `test_gz_bringup.py` is decorated with `@unittest.skipIf(WAREHOUSE_SKIP_GZ_TESTS, ...)`. CI
  sets the variable, and every Gazebo case is reported as **skipped** with the reason, which
  `colcon test-result` counts. A CMake option would have hidden the tests completely.

Not every test needs a running system. Two pytest files check files instead of processes, in
seconds and in every pull request: `warehouse_description/test/test_description.py` (the URDF
and `controllers.yaml` against `robot.yaml`, concept 1) and
`warehouse_gazebo/test/test_gazebo_config.py`. CI cannot render, so the second one is CI's
only look at the Gazebo side. It checks that:

- every world loads the five systems the robot needs, with the right class names;
- every gz topic in the bridge file, expanded in `/robot1` by the bridge's own rule (`rclpy`'s
  `expand_topic_name` applies the same one), is a topic the robot description publishes, with
  the same message type. Ground truth counts only if the URDF loads its system under the exact
  filename;
- each bridge entry uses the same relative name on the ROS side, and the ROS type that matches
  its gz type;
- Gazebo's own URDF conversion (`gz sdf -p`) keeps the casters frictionless (problem 3).

Without it these mistakes would pass CI: `gz sdf -k` calls a world with a misspelt system name
`Valid.`, a bridge entry for a topic that nothing publishes stays silent, and the bridge skips
an entry with a misspelt ROS type after only a warning. Nineteen single mistakes were made on
purpose, among them a misspelt system, an absolute scan topic, a misspelt or wrong ROS type, a
misspelt plugin filename and a named caster collision: the test failed and named the problem
every time.

**Why industry does it.** Unit tests cannot catch a wrong remap, a missing `/**/`, or a node that
ignores SIGINT; those only show up when the real launch file runs. A launch test turns the
acceptance criteria into something CI checks on every pull request.

### 14. Where the lidar sits

**What it is.** A 2D lidar measures distances in one horizontal plane, its *scan plane*. It sees
only what crosses that plane: an obstacle below it, or behind something else, does not exist for
it. Where the lidar sits therefore decides what the robot can see.

**How it works here.** The first revision B did what industrial AMRs do: a scanner low in a
corner of the body (scan plane 0.20 m), looking out through a groove that ran round the body,
with a twin planned for the opposite corner in Rev C. Reviewing Phase 1, the owner chose a
TurtleBot-style puck instead: a 360° lidar standing on the deck near the front edge, scan plane
0.34 m, 720 rays 0.5° apart (`robot.yaml`, `lidar_front`). Three rules come with it:

- **Nothing else in the plane.** `gpu_lidar` sees visuals, so every other part on the deck stays
  at least 2 cm below the plane (tray rim 0.315 m, stop button 0.316 m, status light 0.318 m),
  and `range_min` (0.10 m) hides the puck's own head (radius 0.04 m).
  `test_only_the_lidar_head_reaches_the_scan_plane` checks the bounding box of every visual. In
  Gazebo, 100 scans with a box beside the robot, and 20 more with boxes beside, behind and in
  front of it, had 0 returns inside its outline.
- **A full circle, each direction once.** The rays run from -180° to +179.5°: one more step
  would point where the first ray does. 721 rays from -180° to +180° would measure straight back
  twice.
- **The warehouse follows the scan plane.** Shelves need a plinth of at least 0.40 m, so the
  lidar sees continuous faces rather than single uprights (plan §2.6, fit rules).

**What the puck cannot see.** Anything lower than 0.34 m: a pallet, forklift forks, a fallen
box, and the 0.30 m bodies of other robots like this one, of which only the pucks cross the plane
(Phase 9 runs three). And the rear, once a tote rides on the tray (no phase models one yet;
plan §2.6, Rev C): the tote's front face is only 0.13 m behind the puck, so it hides about 100°
behind the robot, and the scan then shows returns inside the robot's own outline that the
costmaps and the collision monitor must not take for an obstacle. The Rev E camera mast will
cross the plane too.

**Why industry does it differently.** Industrial AMRs usually carry certified safety scanners
close to the floor in two diagonally opposite corners: low, so they see small obstacles and
feet; in the corners, so together they see all round the robot whatever it carries (safety
scanners are sold for exactly this, about 270° each). A puck on top is the research and hobby robot's choice: one inexpensive sensor sees
all round, as long as nothing on the robot blocks it. For this robot that holds while the deck is
empty. Phase 3 (Rev C) revisits it: how the safety layer covers the rear behind a loaded tote and
the obstacles below the plane, for example with low scanners, and how the robot's own tote and
mast stay out of the scan.

## What happened

### Timeline

- **2026-11-04.** Plan v2 adopted: our own robot in revisions A-G, and the EKF moved into
  Phase 1.
- **2026-11-05, build.** One agent wrote the five packages, ADR-005 and the CI change, and
  verified them in a clean environment: 96 tests green on the host, 11 Gazebo cases skipped in
  a reproduction of the CI job, Gazebo acceptance numbers measured.
- **2026-11-05, review and fix round.** A review found four medium issues (braking from
  reverse, `robot.launch.py` on its own, the EKF's growing y covariance, a self-hit test that
  could not fail) and four low ones (test constants copied from config, collectors that never
  unsubscribed, tests reading data before it arrived, a relative namespace). All eight were
  fixed; the Gazebo test then ran 15 times (14 fully green, one hit an upstream shutdown bug).
- **2026-11-05, final verification.** Three verifiers worked in parallel on scratch copies: one
  reproduced the CI job in `ros:jazzy-ros-base` and ran `actionlint`, one measured every
  acceptance criterion in headless Gazebo with its own scripts, one reviewed the code against
  the Jazzy sources. No high or medium issues; six low ones.
- **2026-11-05, this report.** Written from the notes of the three rounds; every exercise below
  was run in a clean shell.
- **2026-11-05, polish round.** The six low findings and a fact-check of this report were fixed:
  the EKF explanation in `warehouse_navigation/README.md`, a log-format setting in
  `sim.launch.py` that changed nothing, RViz's namespace (now absolute, like the robot's), a
  discovery wait in the Gazebo test, a render-free test of the Gazebo-side files for CI, and
  the plan's references. The whole tree and every exercise were run again: 104 tests,
  0 failures, 0 skipped on the laptop; 11 skipped with the CI switch.
- **2026-11-05, polish check.** Two verifiers checked the polish round on scratch copies (the
  CI job, the full suite, three exercises, a fresh container, mistakes made on purpose) and
  found six low issues. A second, smaller round fixed five: the Gazebo configuration test now
  also checks the ROS side of each bridge entry and the ground-truth system's filename (5 of 19
  deliberate mistakes had still passed it), the README's status names the open GIF, concept 9
  explains why the jitter biases the speed upwards, and two wordings. The sixth, a note that
  `git restore` in exercises 2 and 6 fails before the files are committed, did not apply: they
  are committed. The full suite ran again: 104 tests, 0 failures, 0 skipped; 11 skipped with
  the CI switch.
- **2026-11-05, design review.** The owner reviewed the robot and chose a 360° lidar puck on the
  deck (scan plane 0.34 m) over the corner scanner in its groove (0.20 m, 264°), accepting that a
  loaded tote will block much of the rear view, and a modular look in safety yellow and
  charcoal. One agent rebuilt the description (one macro per part, one colour per link), rendered
  it headless in Gazebo and RViz to tune the colours, checked the scan, the frames and driving in
  headless Gazebo, and updated the tests and the documents (problems 15 and 16, concept 14).
  Only the description tests ran (with pytest); the rest of the suite waits, because checks are
  paused (ADR-004).

### The problems, and what they taught

Each story is told as symptom → cause → fix → lesson.

#### 1. The host could not build at all

- *Symptom:* every package failed at configure time:
  `Could not find a package configuration file provided by "ament_cmake_clang_format"`.
- *Cause:* `ros-jazzy-ament-cmake-clang-format` was not installed on the development laptop.
  Phase 0 makes it `REQUIRED` on purpose (Phase 0 bug 5), and `rosdep install` installs it,
  but that needs `sudo`, which the agents did not have.
- *Fix:* verification used the official `.deb` files unpacked into a private folder; the real
  fix is one `rosdep install` by the maintainer (see [Before you start](#before-you-start)).
- *Lesson:* the loud failure from Phase 0 worked exactly as designed: the build stopped instead
  of quietly dropping a linter. Run `rosdep install` after every pull that adds dependencies.

#### 2. pep257 rejected docstrings that pre-commit accepted

- *Symptom:* `colcon test` failed with pep257 `D213` on module docstrings; pre-commit had passed.
- *Cause:* `ament_pep257` wants the summary of a multi-line docstring on the line *after* the
  opening quotes. The repository's `.flake8` ignores the opposite rule (`D212`) but does not
  select `D213`, so the flake8 hook never checks it.
- *Fix:* every docstring now starts on the second line.
- *Lesson:* pre-commit is the fast subset (Phase 0, concept 4); `colcon test` is the reference.

#### 3. The frictionless casters had friction

- *Symptom:* `gz sdf -p` on the generated URDF showed no `<friction>` at all, not even on the
  wheels.
- *Cause:* sdformat's URDF-to-SDF converter applies `<gazebo reference="link">` surface settings
  only to the collision names it generates itself (`<link>_collision`). The first version gave
  each collision a `name=`, and the settings were silently dropped.
- *Fix:* every collision is unnamed. `warehouse_description`'s
  `test_casters_are_frictionless_in_gazebo` checks the URDF (friction 0 on the casters, no named
  collision), and `warehouse_gazebo`'s
  `test_casters_stay_frictionless_after_gazebo_converts_the_urdf` converts it with `gz sdf -p`
  and checks `mu` 0 on both casters in the result.
- *Lesson:* check the converted model (`gz sdf -p`), not the file you wrote. A setting that does
  nothing produces no error.

#### 4. "Failed to normalize given item of type list"

- *Symptom:* `ros2 launch` crashed at start with that message.
- *Cause:* the world file path was built as a list containing a list (the path substitution plus
  `'.sdf'`), inside the command list. Launch accepts a list of substitutions as one argument, but
  not nested lists.
- *Fix:* one flat list per argument (`gazebo.launch.py`, `world_file`).
- *Lesson:* a substitution list is joined into one string; keep it one level deep.

#### 5. Orphaned Gazebo servers broke later runs

- *Symptom:* later test runs failed in odd ways: spawners logged `Failed to configure
  controller`, and the mock test saw Gazebo IMU data and unexpected TF publishers.
- *Cause:* `ros_gz_sim`'s `gz_sim.launch.py` runs `gz sim` through a shell (`shell=True`).
  launch_testing sends SIGINT to the processes it started, which was the shell; the Gazebo
  server survived, with its own `/robot1/controller_manager`, in the same ROS domain. Manual
  runs hid it, because a terminal's Ctrl-C (and GNU `timeout`) signal the whole process group.
- *Fix:* `gazebo.launch.py` starts `gz sim` directly, and the Gazebo test checks after shutdown
  that no process with its private `GZ_PARTITION` is still running. The plugin path Gazebo
  needs comes from the ROS environment, so nothing was lost.
- *Lesson:* a test that passes when you press Ctrl-C can still leak processes under automation.
  Test shutdown the way automation does it, and check for leftovers.

#### 6. Remaps reached the controller manager

- *Symptom:* on mock hardware, one warning per controller: `The use of remapping arguments to
  the controller_manager node is deprecated`.
- *Cause:* the `/tf` remaps applied to `ros2_control_node`, which passes its remaps on to every
  controller it loads.
- *Fix:* `ros2_control_node` starts before the `SetRemap` actions in the group. It uses no TF,
  and Gazebo's controller manager never sees launch remaps either, so both modes now match.
- *Lesson:* scoped actions apply to what follows them; order inside a group is part of the
  design.

#### 7. A flaky mock drive test

- *Symptom:* under a full test run, the EKF travelled 1.05 m while the wheels reported 1.16 m.
- *Cause:* the test compared the latest message of two topics, and under load the EKF's messages
  arrived later. The EKF also has a start transient: its position dips a few millimetres
  negative, then trails the wheels by a few centimetres on a short drive.
- *Fix:* wait until both streams carry stamps from after the stop, and let the mock test check
  wiring (the EKF moves with the wheels) while Gazebo checks accuracy against ground truth.
  The test then passed 5 runs out of 5, 3 of them with every CPU busy.
- *Lesson:* compare messages by their stamps, not by arrival order.

#### 8. Braking from reverse used the wrong limit

- *Symptom (review):* braking from reverse ran at 0.8 m/s² instead of 1.5 m/s².
- *Cause:* `max_deceleration_reverse` was unset. In ros2_controllers 4.42.1 an unset value falls
  back to `max_acceleration` (0.8), not to `-max_deceleration` as the parameter's own
  description claims. The source (`control_toolbox`'s rate limiter) shows the real behaviour.
- *Fix:* all four linear and all four angular limits are explicit in `controllers.yaml`, with a
  comment on the sign convention. A new mock test measures braking at 1.50 m/s² from +0.5 m/s
  and from -0.3 m/s; with the reverse lines removed it fails at 0.80 m/s². It runs in CI.
- *Lesson:* a parameter's description is a claim; the source and a measurement are evidence.
  Phase 3's reverse safety field depends on this number.

#### 9. `robot.launch.py` on its own ran on the wrong clock

- *Symptom (review):* started next to `gazebo.launch.py`, the robot's nodes used the wall clock
  and the EKF published nothing.
- *Cause:* only `sim.launch.py` set `use_sim_time`.
- *Fix:* `robot.launch.py` has its own `use_sim_time` argument and sets it with `SetParameter`
  for its nodes; its namespace became absolute at the same time (concept 10).
- *Lesson:* a launch file that will be included by others must work on its own.

#### 10. The EKF's sideways uncertainty grew without bound

- *Symptom (review):* standing still in Gazebo, the y variance of `odom` grew to about 4e4 m²
  after 30 s and 7e5 m² after a minute.
- *Cause:* nothing measured vy, so the filter's uncertainty in y kept growing.
- *Fix:* fuse the diff drive's vy = 0 (concept 9). Plan D3 and ADR-005 were updated.
- *Lesson:* a covariance is part of the output. A position that looks right with a useless
  covariance will mislead every later consumer (AMCL, Nav2).

#### 11. The self-hit test could not fail

- *Symptom (review):* the test "no scan return inside the robot's outline" passed in the empty
  world, where every ray misses.
- *Cause:* with nothing to see, a blind, rotated or misplaced scanner also has zero self-hits.
- *Fix:* the test spawns a box 2 m to the left and checks every return in world coordinates:
  each run, 580 returns landed on the box in 20 scans of the first, corner-mounted scanner (520
  for the lidar puck, measured by hand with the same set-up), 0 inside the outline, 0 anywhere
  else.
- *Lesson:* the Phase 0 lesson again, in a new place: a check is only evidence if you have seen
  it fail.

#### 12. Test harness pitfalls

These cost time during verification and will cost you time too:

- *Background jobs ignore Ctrl-C.* A non-interactive `bash` script starts `&` jobs with SIGINT
  ignored, and `ros2 launch`, `gz` and the bridges inherited that, so probe sessions survived
  their SIGINT. Fix: `set -m` in such scripts.
- *Two signals look like a crash.* GNU `timeout` and a terminal's Ctrl-C signal the whole
  process group, so every node gets SIGINT from you *and* from launch. Some then exit with
  `-2` (`ekf_node` does). launch_testing sends one SIGINT, and everything exits with 0.
- *Domain cross-talk.* One run's `env -i` dropped the variable that carried its
  `ROS_DOMAIN_ID`, so a "separate" test ran in the same domain as a Gazebo session: the mock test
  saw an IMU orientation of w = 0.979 (Gazebo data) and spawners failed. Fix: pass the domain
  through, and re-run.
- *Stale CLI daemon.* `ros2 topic info` once said `Unknown topic` through a stale `ros2 daemon`;
  with `--no-daemon` it missed a subscription because the one-shot node did not wait for
  discovery. `ros2 daemon stop` before a session, give the new daemon a few seconds to discover
  the graph (its first answers can be `Unknown topic` or a node named `_NODE_NAME_UNKNOWN_`), and
  trust the launch tests over one-shot CLI calls.

#### 13. Gazebo sometimes hangs on shutdown (upstream)

- *Symptom:* about one Gazebo test run in 15 failed only `test_processes_exit_cleanly`:
  `gazebo-1` ignored SIGINT for 5 s and SIGTERM for 10 s and was killed (exit code -9).
- *Cause:* the open upstream bug
  [gazebosim/gz-sim#4173](https://github.com/gazebosim/gz-sim/issues/4173), on the same stack
  (Harmonic, `gpu_lidar`, headless, Intel GPU): gz-common's signal handler stops the Sensors
  system inside an interrupted gz-transport publish, and the render thread deadlocks.
- *Fix:* none in this repository; documented in `warehouse_system_tests/README.md`. CI is not
  affected (it skips the Gazebo test). If every other case passed, re-run. With the lidar puck,
  2 of 23 headless launches hung the same way when stopped, in line with the earlier rate.
- *Lesson:* when a failure is not yours, find the upstream issue, compare the stack, record it,
  and keep your own checks strict.

#### 14. Smaller snags

- `yamllint` rejected EKF matrices aligned with two spaces after commas; single spaces, with the
  order written in a comment above.
- `ros2 launch ... namespace:=` (empty) is a "malformed launch argument"; use `namespace:=/`.
- RViz cannot start with `QT_QPA_PLATFORM=offscreen` (OGRE needs GLX), so its configuration was
  checked under a virtual X server in the dev-container image. The LaserScan display subscribes
  only once the topic exists.
- `sim.launch.py` set `RCUTILS_CONSOLE_OUTPUT_FORMAT` to `[{severity}] [{time}] [{name}]:
  {message}`, which is rcutils' built-in default (`strings /opt/ros/jazzy/lib/librcutils.so`
  prints the same string), so the action changed nothing. The polish round removed it.
- The first explanation of the EKF's 0.1-0.4 % error, in `warehouse_navigation/README.md`
  ("trails speed changes; no slip for the IMU to correct"), was wrong. The review measured the
  real cause (concept 9), and the README now gives it. Measure before you explain.

#### 15. RViz drew the whole body in one colour

- *Symptom (design review):* rendered in RViz, the first body (an orange lower body, a black band
  in the scanner groove and a grey deck, all visuals of `base_link`) was orange all over. With the
  grey deck moved to be the first visual, the whole body turned grey. Gazebo showed all three
  colours.
- *Cause:* RViz 2's robot model (`rviz_default_plugins`, Jazzy) gives each primitive visual an
  empty material name, so every visual of a link falls back to the material of the link's first
  visual. Gazebo uses each visual's own material.
- *Fix:* one colour per link. The coloured parts are fixed, massless links (`shell_link`,
  `front_bumper_link`, `tote_tray_link`, ..., and a hub link below each wheel), which Gazebo
  merges, so the physics do not change. `test_each_link_has_one_colour` keeps it that way, and
  RViz's TF display starts with these frames hidden.
- *Lesson:* a valid URDF can still be drawn differently by each tool. Look at it in every tool
  that shows it.

#### 16. Gazebo's colours are not RViz's

- *Symptom:* the first safety yellow (0.80, 0.62, 0.02) looked olive in Gazebo, and the charcoal
  deck looked mid-grey in Gazebo but black in RViz.
- *Cause:* Gazebo's URDF converter (sdformat) multiplies a URDF colour by 1.25 and clips at 1
  (`gz sdf -p` shows 0.5 turned into 0.625), and its ogre2 renderer lights in linear space and
  outputs sRGB, which lifts dark colours and pulls a yellow's red and green together. Measured
  from rendered pixels, Gazebo shows sRGB(L x 1.25 x colour), with L about 0.75 on tops lit by
  the default sun, 0.5 on the front and 0.19 on faces in its shade. RViz shows the values as
  they are, lit from the camera.
- *Fix:* a palette checked in both renderers (`materials.xacro`): every channel at or below
  0.8, a yellow with less green and no blue (0.80, 0.58, 0.0: hue about 52° in Gazebo, 43° in
  RViz), charcoal at 0.09.
- *Lesson:* a colour value is an input to a renderer, not a colour. Look at the output.

### Things that look wrong but are not

- `diff_drive_controller` logs `Velocity command timed out. Braking.` about once per second
  whenever no command arrives, including at rest.
- `ros2 topic info /tf` shows one publisher, `diff_drive_controller`, that never sends anything
  (concept 10).
- `Could not enable FIFO RT scheduling policy`: a desktop user may not use real-time
  scheduling; simulation does not need it.
- On mock hardware, `ros2_control_node` logs `pal_statistics: Exception in publisher thread:
  context cannot be slept with because it's invalid!` at shutdown.
- sdformat warns that `gz_frame_id` is not part of the SDF spec; it copies the element through
  and the sensors use it.
- After Ctrl-C in a terminal, `ekf_node` may report `process has died ... exit code -2`
  (problem 12: it received two SIGINTs).
- `ros2 topic echo --once` may print `A message was lost!!!` before the message; harmless.

## Measured results

Measured on the development laptop (Intel Iris Xe, headless EGL rendering) in a clean
environment, in sim time. Ranges cover the build run, 15 test runs in the fix round, both final
verifiers, the run for this report and the polish round's runs. After the design change of
2026-11-05 (lidar puck and new look; physics unchanged) the controllers, frames, scan and drive
were measured again by hand in headless Gazebo, with the launch test's set-up for the scan
check; the launch tests themselves wait until checks resume.

| Acceptance criterion (plan, Phase 1) | Target | Measured | Status |
|---|---|---|---|
| Controllers active, `hardware:=gz` and `hardware:=mock` | all active | 3 of 3 active in both | met |
| `/robot1/cmd_vel` type and subscribers | `TwistStamped`, only `diff_drive_controller` | as required, in both | met |
| TF tree vs plan §2.4 (Phase 1 part) | match | `view_frames` (remapped) shows exactly the tree in concept 3 | met |
| `/robot1/tf` publishers; global `/tf` | RSP and EKF; nothing on `/tf` | `robot_state_publisher`, `ekf_filter_node`; 0 messages on `/tf` and `/tf_static` in 10 s | met |
| `odom/wheel` frames | `odom` → `base_footprint` | as required | met |
| Odometry, 10 m straight | < 1 % | wheels 0.006-0.01 %, EKF 0.09-0.42 % (over about 10.4 m) | met |
| Odometry, 360° turn in place | < 2 % | wheels 0.00 %, EKF at most 0.05 % (over about 371°) | met |
| Rates (±10 %) | scan 10, imu 100, odom/wheel 50 Hz | 10.0, 100.0, 49.9-50.0 Hz; the puck's scan again 10.00 Hz | met |
| Scanner self-hits | 0 | Puck: 0 in 100 scans (72,000 rays) with a box 2 m away, all 2,600 returns on the box; 0 with boxes beside, behind and in front; the launch test's 20 scans see 520 returns on the box. Corner scanner before: 0 in the empty world (100 scans, 52,900 rays), 580 returns on the box per run | met |
| Teleop stops after `cmd_vel_timeout` | stops | measured with `cmd_vel_timeout` 0.5 s (0.15 s since 2026-11-06): braking starts when the 0.5 s timeout expires; standstill about 0.85 s after the last command (0.5 s plus 0.33 s braking from 0.5 m/s at 1.5 m/s²). Real `teleop_twist_keyboard`, keys fed through a pseudo-terminal: still 0.5 m/s at +0.4 s, braking at +0.5 s, standing by +1.0 s | met |
| xacro `gz` and `mock`, `real` refused, `check_urdf` | as stated | as stated; `real` exits with code 2 and the message | met |
| Demo GIF: teleop with the RViz sensor view | recorded | not yet | **open** ([exercise 7](#exercise-7-run-the-demo-with-windows-and-record-the-gif)) |

Additional numbers:

- Braking at the hard limit: 1.50 m/s² from +0.5 m/s and from -0.3 m/s (mock test, every PR).
- Test suite: 104 tests, 0 failures, 0 skipped on the laptop (`colcon test` takes 70-75 s,
  most of it the Gazebo test). In the CI configuration: 104 tests, 0 failures, 11 skipped. Both
  before the design change, which replaces one description test with four: 107 in total, not
  run yet.
- The design change, measured by hand in headless Gazebo: the scan has 720 rays from -180° to
  +179.5° (360°) at 10.00 Hz in sim time; a 2 s drive at 0.3 m/s, stamped in sim time, moved
  the robot 0.71 m and it stood still after `cmd_vel_timeout` (then 0.5 s); Gazebo's
  converted model still has 33.000000000000007 kg in `base_footprint` and the same friction;
  21 of 23 launches stopped cleanly after one SIGINT (the other two hit problem 13).
- CI job reproduced in `ros:jazzy-ros-base`: green in about 8.5 minutes, of which about 7.8
  minutes is `rosdep install` (Gazebo, ros2_control and RViz are now run dependencies). The
  first real GitHub run happens with the pull request.
- `warehouse_gazebo`'s tests in a fresh `ros:jazzy-ros-base` container, with only the declared
  dependencies of `warehouse_description` and `warehouse_gazebo` installed: 19 tests, 0
  failures, without a GPU or a display.
- The IMU noise check (σ within ±20 %) belongs to Phase 2 by plan.

### What was not verified

- **GUI mode** (`sim.launch.py` with the Gazebo window and RViz on the desktop) and the GIF.
  The headless server and the RViz configuration (under a virtual X server) were verified. The
  new look was rendered headless only: Gazebo cameras with the default sun, and RViz in a
  container under a virtual X server.
- **The test suite after the design change** (checks paused): the 19 description tests passed
  when run with `pytest` directly, and each new one failed on a mistake made on purpose; the
  launch tests were not run, so their updated expectations (the TF tree, the scan's full circle)
  were checked by hand only.
- The GIF steps of exercise 7 on a real screen recording: the conversion and size-check
  commands ran in a container, on a generated test clip. The laptop has neither ffmpeg nor
  gifsicle; the exercise installs them with `sudo apt install`.
- `rqt_tf_tree`: not installed on the laptop yet (`rosdep install` adds it).
- A real GitHub Actions run (only the local reproduction).
- More than one robot (Phase 9; see the IMU naming risk in ADR-005).

## Try it yourself

Each exercise lists the commands and what you should see. `~/warehouse_ws` is both the
workspace and the repository (ADR-006); the packages are in `~/warehouse_ws/src/`.

### Before you start

**1. Install the dependencies once** (needs `sudo`; Phase 1 adds Gazebo, ros2_control, RViz,
teleop and others). In any terminal:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/warehouse_ws
rosdep install --from-paths src --ignore-src -y
```

**2. Use a clean shell.** Your normal `~/.bashrc` sources other workspaces and uses ROS's
default discovery, which reaches other machines on the LAN. Packages from another workspace can
shadow ours, and nodes from another session can join our graph. Add this function to the end of
`~/.bashrc` once:

```bash
# Start a shell with only ROS 2 Jazzy and ~/warehouse_ws: no other overlays, discovery on
# this machine only. GUI programs still reach your desktop through the display variables.
wh_shell() {
  env -i HOME="$HOME" USER="$USER" TERM="$TERM" LANG="${LANG:-C.UTF-8}" \
    DISPLAY="$DISPLAY" WAYLAND_DISPLAY="$WAYLAND_DISPLAY" XAUTHORITY="$XAUTHORITY" \
    XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --rcfile <(printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      'if [ -f ~/warehouse_ws/install/setup.bash ]; then source ~/warehouse_ws/install/setup.bash; fi' \
      'export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' \
      'export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1' \
      'cd ~/warehouse_ws' \
      'PS1="(warehouse) \w\$ "')
}
```

Open a new terminal and type `wh_shell`. The prompt changes to `(warehouse) ~/warehouse_ws$`,
and `echo $AMENT_PREFIX_PATH` lists only `/opt/ros/jazzy` and (after the first build) our five
packages. **Every command below runs in such a shell**; exercises that need several terminals
need `wh_shell` in each. `ROS_DOMAIN_ID=42` matches the dev container; pick another number if
something else on your machine uses 42.

**3. Build cleanly once.** The workspace may hold output of earlier builds made with other
environments. Delete it, leave the shell (it still points at the deleted `install/`), and build
in a fresh one:

```bash
rm -rf build install log
exit
wh_shell
colcon build --symlink-install
source install/setup.bash
ros2 daemon stop
```

`colcon build` ends with `Summary: 5 packages finished`. `ros2 daemon stop` makes sure the CLI
tools do not reuse a daemon started from your normal shell. From now on, `wh_shell` sources the
workspace automatically.

### Exercise 1: build, test, and read the measured numbers

Checks are paused (ADR-004), so a plain `colcon build` builds no tests (`colcon.meta`): build
with them first.

```bash
colcon build --symlink-install --metas /dev/null --cmake-args -DBUILD_TESTING=ON
colcon test && colcon test-result --verbose
grep -hE "Measured rates|Scan check|odometry:|cmd_vel_timeout [0-9]|Braking from" \
  build/warehouse_system_tests/launch_test/*.txt
```

The test run takes a little over a minute (the Gazebo test is the long one; no window
opens). Do not run a simulation at the same time: `wh_shell` sets `ROS_DOMAIN_ID`, so the
tests use that domain too, and a second `/robot1/controller_manager` would confuse them.
Expect:

```text
Summary: 107 tests, 0 errors, 0 failures, 0 skipped
```

(107: the design change of 2026-11-05 replaced one description test with four; the last full
run, just before it, reported 104. The scan check line below was measured with the test's own
set-up after the change; the full run is repeated when checks resume.)

and numbers like these (yours will differ in the last digits). Some of the lines begin with
the name of another test case, such as `test_05_tf_stays_in_the_namespace (...) ... Measured
rates ...`: the log mixes unittest's progress lines with the tests' own printed output, so a
printed line can land after a later test's name. Ignore that prefix.

```text
Measured rates (sim time): {'scan': 10.0, 'imu': 100.0, 'odom/wheel': 50.0}
Scan check: 20 scans, 520 returns on the target, 0 inside the outline, 0 elsewhere
wheel odometry: 10.388 m forward, 0.000 m left, truth 10.389 / 0.000 m, error 0.01 %
fused odometry: 10.402 m forward, 0.015 m left, truth 10.389 / 0.000 m, error 0.19 %
wheel odometry: turned 371.23 deg, truth 371.22 deg, error 0.00 %
fused odometry: turned 371.22 deg, truth 371.22 deg, error 0.00 %
cmd_vel_timeout 0.15 s: 5 odom/wheel samples from +0.05 to +0.15 s after the last command, lowest 0.501 m/s; standing still from +0.79 s
cmd_vel_timeout 0.15 s: 5 odom/wheel samples from +0.05 to +0.15 s after the last command, lowest 0.500 m/s; standing still from +0.79 s
Braking from +0.50 m/s: 1.50 m/s^2
Braking from -0.30 m/s: 1.50 m/s^2
```

The two `cmd_vel_timeout` lines (Gazebo, then mock hardware) were added on 2026-11-06 with
the 0.15 s timeout: after the last command the robot keeps its speed until the timeout, then
stands still.

Now run the system tests the way CI does:

```bash
WAREHOUSE_SKIP_GZ_TESTS=1 colcon test --packages-select warehouse_system_tests
colcon test-result --all | grep test_test_
colcon test-result
```

The Gazebo file reports `11 tests, 0 errors, 0 failures, 11 skipped`, the mock file `7 tests,
0 errors, 0 failures, 0 skipped`, and the total becomes `107 tests, 0 errors, 0 failures, 11
skipped`: skipped, not passed and not missing. Run `colcon test` once more without the variable
to get back to 0 skipped.

### Exercise 2: the robot description by hand

```bash
xacro $(ros2 pkg prefix warehouse_description)/share/warehouse_description/urdf/robot.urdf.xacro \
  namespace:=robot1 hardware:=gz > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

`check_urdf` prints `Successfully Parsed XML`, `root Link: base_footprint has 1 child(ren)`,
then `base_link` with 13 children: the two casters, `imu_link`, the two wheels (each with its
hub link below it), `lidar_front_link` and `lidar_front_mount_link`, and the visual-only parts
`emergency_stop_link`, `front_bumper_link`, `rear_bumper_link`, `shell_link`,
`status_light_link` and `tote_tray_link` (concept 2). Now ask for the reserved hardware:

```bash
xacro $(ros2 pkg prefix warehouse_description)/share/warehouse_description/urdf/robot.urdf.xacro \
  hardware:=real
echo $?
```

It prints `error: hardware:=real is reserved for a future warehouse_hardware SystemInterface
(plan A1). Use hardware:=gz or hardware:=mock.` and the exit code is `2`. Finally, see what
Gazebo makes of the URDF:

```bash
gz sdf -p /tmp/robot.urdf 2>/dev/null | grep -E "<link name|<mass>|<mu>"
```

Three links remain: `base_footprint` with `<mass>33.000000000000007</mass>` (body and casters
merged through their fixed joints, concept 2) and the two wheel links with 1 kg each. The
casters show `<mu>0</mu>`, the wheels `<mu>1</mu>`. Try adding `name="caster"` to the caster
`<collision>` in `drive.xacro` and repeat: the caster friction disappears (problem 3). Undo the
change with `git restore src/warehouse_description/urdf/drive.xacro`.

### Exercise 3: the robot without a simulator (mock hardware)

Terminal 1 (RViz opens; add `rviz:=false` if you have no display):

```bash
ros2 launch warehouse_bringup sim.launch.py hardware:=mock
```

Terminal 2, once the log shows the three spawners `finished cleanly`:

```bash
ros2 control list_controllers -c /robot1/controller_manager
ros2 topic info -v /robot1/cmd_vel
ros2 topic echo /robot1/odom/wheel --once --field child_frame_id
ros2 topic info -v /robot1/tf | grep -E "Node name|count"
```

You should see the three controllers `active`; `Type: geometry_msgs/msg/TwistStamped` with
`Publisher count: 0`, `Subscription count: 1` and `Node name: diff_drive_controller`, with
`Reliability: BEST_EFFORT` (concept 7); `base_footprint`; and on `/robot1/tf` the publishers
`robot_state_publisher` and `ekf_filter_node`. Now drive 0.3 m/s for 2 s (40 messages at
20 Hz), then read the position:

```bash
ros2 topic pub -r 20 -t 40 /robot1/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {stamp: now}, twist: {linear: {x: 0.3}}}'
ros2 topic echo /robot1/odom/wheel --once --field pose.pose.position
ros2 topic echo /robot1/odom --once --field pose.pose.position
```

`stamp: now` fills in the publisher's current time; mock hardware runs on the wall clock, so
that is right here. The wheel odometry reports `x: 0.603...`, and the EKF nearly the same
(0.592-0.604 in three runs on 2026-11-06: on mock hardware it trails a sudden start and does
not always catch up completely, problem 7). The wheels' arithmetic: 0.3 m/s for the 1.95 s of
commands plus the 0.15 s until `cmd_vel_timeout` fires is 0.63 m; speeding up at 0.8 m/s²
costs 5.6 cm, and braking at 1.5 m/s² rolls 3 cm further: 0.604 m. (Until 2026-11-06 the
timeout was 0.5 s and the wheels reported 0.708 m.) Stop with Ctrl-C in terminal 1.

### Exercise 4: Gazebo from the outside

Terminal 1 (headless: no windows, the scanner still renders):

```bash
ros2 launch warehouse_bringup sim.launch.py headless:=true
```

Terminal 2, after about 20 s:

```bash
ros2 topic hz -s /robot1/scan          # Ctrl-C after a few seconds; -s measures in sim time
ros2 topic hz -s /robot1/imu
ros2 topic hz -s /robot1/odom/wheel
ros2 topic info -v /robot1/scan | grep -E "Node name|Reliability"
ros2 topic info -v /tf | grep -E "Node name|count"
gz topic -l
```

If `ros2 topic info` prints `Unknown topic '/robot1/scan'` (or `'/tf'`), or shows a publisher as
`Node name: _NODE_NAME_UNKNOWN_`, wait a few seconds and run it again: a freshly started
`ros2 daemon` needs a moment to discover the graph (problem 12).

Expect average rates of about 10, 100 and 50 (the last one between 48 and 50 over a short
window), the scan published by `gz_bridge` with `BEST_EFFORT`, one idle publisher on `/tf`
(`diff_drive_controller`, concept 10) and no subscriber. `gz topic -l` lists the Gazebo side:
`/robot1/scan` (and `/robot1/scan/points`, the same scan as a point cloud), `/robot1/imu`, the
three `/robot1/ground_truth/...` topics, `/clock` and the world's own topics. None of these appear in `ros2 topic list` except through the bridge.

Draw the TF tree (in a folder of your choice; it writes a PDF):

```bash
mkdir -p /tmp/frames && cd /tmp/frames
ros2 run tf2_tools view_frames --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
cd ~/warehouse_ws
```

Open the `frames_<date>.pdf` it names: `odom → base_footprint → base_link → 13 children`, with
a hub below each wheel, exactly concept 3. The static transforms show a rate of `10000` (sent
once on `tf_static`), `base_footprint` about 50 Hz (the EKF), the wheels about 20 Hz
(`robot_state_publisher`'s publish rate). Without the remap, `view_frames` would find an empty tree. Leave the simulation
running for exercise 5.

### Exercise 5: the wall-clock trap

With the headless simulation of exercise 4 still running, drive 2 s with a *sim-time* stamp
(`-s`), wait, and read the speed:

```bash
ros2 topic pub -s -r 20 -t 40 /robot1/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {stamp: now}, twist: {linear: {x: 0.5}}}'
sleep 3; ros2 topic echo /robot1/odom/wheel --once --field twist.twist.linear.x
```

The speed is about `0` (a number like `-2e-16`): the timeout stopped the robot. Now the same
without `-s`, so the stamp is wall-clock time:

```bash
ros2 topic pub -r 20 -t 40 /robot1/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {stamp: now}, twist: {linear: {x: 0.5}}}'
sleep 3; ros2 topic echo /robot1/odom/wheel --once --field twist.twist.linear.x
sleep 3; ros2 topic echo /robot1/odom/wheel --once --field twist.twist.linear.x
```

The robot is still driving at about `0.5` m/s seconds after the last message, and it will keep
going: the last command's stamp is decades "in the future" for a clock that started at zero
(concept 11, plan R1). A single reading can be off, the first one especially: first readings
of 0.62 and even 4.6 m/s have been seen, each followed by 0.50. The controller divides the
wheels' position change by the interval between its update stamps, and when those stamps
jitter or lag (concept 9), a reading jumps. Stop the robot with a sim-time zero command, then
Ctrl-C in terminal 1:

```bash
ros2 topic pub -s -r 10 -t 5 /robot1/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: now}}'
```

### Exercise 6: tuning is validated

Change the wheel radius in the single source, but not in `controllers.yaml`, and run the
description tests. They must be built first, because a plain `colcon build` builds no tests
while checks are paused (skip that line if exercise 1 built them):

```bash
colcon build --symlink-install --metas /dev/null --cmake-args -DBUILD_TESTING=ON
sed -i 's/^  wheel_radius: 0.075$/  wheel_radius: 0.080/' \
  src/warehouse_description/config/robot.yaml
colcon test --packages-select warehouse_description
colcon test-result --verbose | grep -E '^ +E +|FAILED'
```

The output ends with:

```text
    E       assert 0.075 == 0.08 ± 8.0e-08
    E         comparison failed
    E         Obtained: 0.075
    E         Expected: 0.08 ± 8.0e-08
    FAILED ../../src/warehouse_description/test/test_description.py::test_controllers_match_robot_yaml
```

The URDF followed `robot.yaml` automatically, the controller's copy in `controllers.yaml` did
not, and the test caught it.
(The `--symlink-install` build links `robot.yaml` into `install/`, so no rebuild is needed.)
Undo the change and re-run:

```bash
git restore src/warehouse_description/config/robot.yaml
colcon test --packages-select warehouse_description && colcon test-result
```

### Exercise 7: run the demo with windows and record the GIF

This is the one Phase 1 acceptance item left: **a GIF of teleop with the RViz sensor view**,
at most 3 MB, in `docs/media/` (CONTRIBUTING.md). Plan about 15 minutes. You need three
`wh_shell` terminals.

**Terminal 1: the simulation with windows.**

```bash
ros2 launch warehouse_bringup sim.launch.py
```

The Gazebo window and RViz open (the first start can take half a minute). RViz shows the robot
model, its TF frames (the visual-only parts' frames start hidden) and the two odometries (EKF and
wheels); the Fixed Frame is `odom`. If a window does not open,
check `echo $DISPLAY` in that terminal: it must not be empty.

**Terminal 2: give the scanner something to see.** The world is empty, so the scan has no
points yet. Spawn a box 2 m ahead (repeat with another `-name` and position for more):

```bash
ros2 run ros_gz_sim create -name box -x 2.0 -y 0.0 -z 0.25 -string \
  '<sdf version="1.9"><model name="box"><static>true</static><link name="link"><collision name="collision"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>'
```

It prints `Entity creation successful.`, the box appears in Gazebo, and RViz draws its face as a
line of red points. (Gazebo's toolbar can also insert a box; then drag it into the scan plane.)

**Terminal 3: teleop.**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r __ns:=/robot1 -p stamped:=true -p use_sim_time:=true
```

Keep this terminal focused while driving: `i` forward, `,` backward, `j`/`l` turn, `k` stop,
`q`/`z` faster/slower (it starts at 0.5 m/s and 1.0 rad/s). Hold a key to drive; release it and
the robot stands still about 0.5 s later (`cmd_vel_timeout`, 0.15 s, then braking). A held key
first moves the robot a little and stops it for 0.2-0.3 s until the keyboard's auto-repeat
starts; that is expected ([warehouse_bringup README](../../src/warehouse_bringup/README.md#things-that-look-wrong-but-are-not)).
Tap `j` or `l` to turn in small steps of about 4°. Practise once: drive towards the box, turn
so the scan sweeps over it, back off.

**Arrange the windows** so RViz is large and the teleop terminal and (optionally) the Gazebo
window sit beside it, all inside one rectangle of the screen.

**Record** with GNOME's built-in screen recorder (Ubuntu 24.04):

1. Press **Print Screen**. In the bar that appears, switch from the camera icon to the **video
   camera** icon (screencast), choose **Selection**, and drag a rectangle around RViz and the
   teleop terminal.
2. Click the round record button. Click into the teleop terminal and drive for 10-15 s: towards
   the box, a turn, release (the stop is part of the demo).
3. Stop with the red stop button in the top bar. The video is saved in
   `~/Videos/Screencasts/` as a `.webm` file.

**Convert to a GIF** (two passes: build a palette from the video, then use it):

```bash
sudo apt install ffmpeg gifsicle
cd ~/warehouse_ws
mkdir -p docs/media
ffmpeg -i ~/Videos/Screencasts/<your-recording>.webm \
  -vf "fps=10,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=5" \
  -loop 0 docs/media/phase-01-teleop.gif
ls -lh docs/media/phase-01-teleop.gif
```

`fps=10` and `scale=960` keep the file small; `palettegen` picks the best 128 colours for this
clip. If the file is over 3 MB, shrink it losslessly first, then trade quality:

```bash
gifsicle -O3 --lossy=40 docs/media/phase-01-teleop.gif -o /tmp/teleop-small.gif
ls -lh /tmp/teleop-small.gif      # if under 3 MB: mv /tmp/teleop-small.gif docs/media/phase-01-teleop.gif
```

or re-run `ffmpeg` with `fps=8` or `scale=800`. Check the limit the way CI will:

```bash
pre-commit run check-added-large-files --files docs/media/phase-01-teleop.gif
```

`check media size` (the 3 MB limit) must print `Passed`; the 1 MB hook prints `(no files to
check) Skipped`, because it excludes `docs/media/`.
Open the GIF in a browser to check it. Then stop everything (Ctrl-C in terminals 1 and 3); the
GIF goes into the Phase 1 pull request, and the README's demo section will link it.

## Pitfalls and lessons

The problem stories above each end with their lesson. These are the everyday pitfalls:

- **Run in a clean shell** (`wh_shell`). Another workspace earlier in `AMENT_PREFIX_PATH` can
  provide a different `warehouse_*` or controller package, and default discovery lets other
  sessions on the LAN join your graph.
- **Two simulations, one domain, chaos.** Two `/robot1/controller_manager` nodes in one
  `ROS_DOMAIN_ID` confuse the spawners. Stop one simulation before starting another, or use
  different domains and `GZ_PARTITION`s.
- **Stamp commands with the right clock.** In Gazebo, every `TwistStamped` publisher needs
  `use_sim_time:=true` (`-s` for `ros2 topic pub`), or the robot never times out.
- **TF tools need the remaps.** `view_frames`, `rqt_tf_tree`, `tf2_echo` and RViz listen on
  `/tf` by default and see nothing of `/robot1/tf` without `-r /tf:=/robot1/tf -r
  /tf_static:=/robot1/tf_static`.
- **Best effort in, best effort out.** A reliable subscriber to `/robot1/scan` gets nothing;
  set RViz or your node to best effort (`SENSOR_DATA`).
- **Parameter files in a namespace need `/**/`.** A plain node-name key silently matches
  nothing.
- **Unnamed collisions** if you want `<gazebo reference>` surface settings to apply.
- **`ros2 daemon stop`** when CLI tools show topics or nodes that should not exist.

## Glossary

| Term | Meaning |
|---|---|
| AMCL | Adaptive Monte Carlo Localization: a particle filter that localizes the robot on a known map (Phase 2) |
| `base_footprint` / `base_link` | The robot's frame on the floor (the URDF root) and its body frame (here on the drive axle), REP-105 |
| best effort / reliable | QoS reliability: lost messages are not resent / are resent until delivered |
| broadcaster | A ros2_control controller that only reads state interfaces and publishes them |
| command / state interface | A value a hardware plugin accepts (wheel velocity) / reports (wheel position, IMU rate) |
| controller manager | The ros2_control node that loads hardware and controllers and runs the control loop |
| covariance | A measure of uncertainty; its diagonal holds the variances (σ²) of each value |
| dead reckoning | Estimating position by adding up measured motion; errors accumulate |
| DDS | Data Distribution Service: the middleware under ROS 2 that discovers nodes and delivers messages; `ROS_DOMAIN_ID` and the discovery range limit whom it reaches |
| EKF | Extended Kalman filter: fuses predictions and measurements weighted by their uncertainty |
| `gpu_lidar` | Gazebo's rendered lidar sensor; it sees visuals, not collisions |
| ground truth | The true value, here the robot's world pose from Gazebo; for tests only |
| gz-transport | Gazebo's own publish/subscribe system, separate from ROS 2 |
| `GZ_PARTITION` | Keeps gz-transport sessions apart, like `ROS_DOMAIN_ID` for ROS |
| hardware interface | The ros2_control plugin that talks to motors and sensors (or a simulator, or a fake) |
| inertia tensor | How a body's mass is distributed: how hard it is to rotate about each axis |
| KDL | Kinematics and Dynamics Library (Orocos KDL), which `robot_state_publisher` uses to compute the links' transforms from the URDF |
| launch_testing | Framework for tests that run while a launch description is running, plus after-shutdown checks |
| lumping | Gazebo's merging of links on fixed joints into one simulated link |
| mock hardware | `mock_components/GenericSystem`: a fake ros2_control hardware for tests without a simulator |
| namespace | A name prefix (`/robot1`) that relative node and topic names are resolved in |
| Nav2 | The ROS 2 navigation stack: path planning, path following and recovery behaviours (Phase 3) |
| nonholonomic | Unable to move in some direction directly; a diff drive cannot slide sideways |
| odometry | An estimate of the robot's motion from wheels and sensors |
| QoS | Quality of service: per-topic delivery rules (reliability, durability, history) |
| remap | Renaming a topic or node for one node (`-r from:=to`) |
| REP | ROS Enhancement Proposal; REP-103 (units, axes) and REP-105 (frame names) apply here |
| `robot_state_publisher` (RSP) | Publishes the URDF and the transforms of the robot's links |
| ros_gz bridge | `ros_gz_bridge`: copies messages between gz-transport and ROS 2 |
| scan plane | The horizontal plane a 2D lidar measures in, here 0.34 m above the floor; the lidar sees only what crosses it |
| SDF | Simulation Description Format, Gazebo's model and world format |
| SIGINT | The interrupt signal: what Ctrl-C sends, and what launch sends to each process it started when it shuts down |
| sim time | The simulator's clock on `/clock`, used when `use_sim_time` is true |
| spawner | A short-lived process that loads and activates one controller |
| system (Gazebo) | A Gazebo plugin that adds a capability to the world (physics, sensors, IMU) |
| TF / tf2 | ROS's library of coordinate frames and the transforms between them |
| transient local | QoS durability: the last messages are kept for subscribers that join later |
| `TwistStamped` | A velocity command (linear and angular) with a header: stamp and frame |
| URDF | Unified Robot Description Format: links, joints, visuals, collisions, inertia |
| xacro | XML macro language that generates URDF |

## Further reading

- ROS 2 Jazzy tutorials:
  - URDF: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html>
  - xacro: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html>
  - tf2: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html>
  - Launch: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html>
  - Integration tests with launch_testing: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html>
  - Gazebo with ROS 2: <https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html>
  - QoS settings: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
  - Domain IDs: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html>
  - `robot_state_publisher`: <https://docs.ros.org/en/jazzy/p/robot_state_publisher/>
- REP 103, standard units and coordinate conventions: <https://www.ros.org/reps/rep-0103.html>
- REP 105, coordinate frames for mobile platforms: <https://www.ros.org/reps/rep-0105.html>
- ros2_control (Jazzy): <https://control.ros.org/jazzy/index.html>
  - Getting started (the architecture): <https://control.ros.org/jazzy/doc/getting_started/getting_started.html>
  - `diff_drive_controller`: <https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>
  - Mock components: <https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html>
  - `gz_ros2_control`: <https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html>
- Gazebo Harmonic: ROS 2 integration <https://gazebosim.org/docs/harmonic/ros2_integration>,
  sensors <https://gazebosim.org/docs/harmonic/sensors>
- `ros_gz_bridge` README (configuration file format, QoS): <https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge>
- `robot_localization` configuration guide (the 15-value lists, fusing zeros):
  <https://docs.ros.org/en/noetic/api/robot_localization/html/configuring_robot_localization.html>
- ROS 2 clock and time design (sim time): <https://design.ros2.org/articles/clock_and_time.html>
- Nav2 documentation, including its setup guides for odometry and transforms:
  <https://docs.nav2.org/>
- The Gazebo shutdown bug of problem 13: <https://github.com/gazebosim/gz-sim/issues/4173>

## What comes next

Phase 2 builds the warehouse: a world generated from one `layout.yaml` (shelves, aisles,
docks), a map made once with slam_toolbox, and AMCL localization on that map, which adds the
`map -> odom` transform above the EKF. Its acceptance includes the localization accuracy that
concept 9 left open, and the IMU noise check of revision B. Its shelves follow the fit rules of
plan §2.6, among them a plinth of at least 0.40 m, so the lidar sees continuous shelf faces
(concept 14). Its report will be
`docs/phase-reports/phase-02-<name>.md`, and its features will appear in
[PROJECT_STATUS.md](../PROJECT_STATUS.md).
