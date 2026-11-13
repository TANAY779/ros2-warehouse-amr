# Project status

The living record of this project: what exists, what was decided and why, and what comes
next. Every commit that adds or changes a feature updates this file (see
[How this document is maintained](#how-this-document-is-maintained)). For the story of each
finished phase, written to teach, see [phase-reports/](phase-reports/).

**Where to read more:** the learning reports in [phase-reports/](phase-reports/) (start with
[Phase 0](phase-reports/phase-00-foundation.md), then
[Phase 1](phase-reports/phase-01-robot-description.md) and
[Phase 2](phase-reports/phase-02-warehouse-world.md)); the phased plan with architecture and
acceptance criteria in [PROJECT_PLAN.md](PROJECT_PLAN.md); the decision records in
[decisions/](decisions/). New to terms such as rosdep, colcon, ruleset or ADR? See
the [glossary in the Phase 0 report](phase-reports/phase-00-foundation.md#glossary); for URDF,
TF, ros2_control, QoS or sim time, the
[glossary in the Phase 1 report](phase-reports/phase-01-robot-description.md#glossary); for SLAM,
AMCL, occupancy grids, lifecycle managers or rosbag2, the
[glossary in the Phase 2 report](phase-reports/phase-02-warehouse-world.md#glossary).

## At a glance

- **Current phase (as of 2026-11-13):** Phase 3a (Nav2 navigation) is **in progress**. Its
  first two stages are done: S0, the leftovers of the Phase 2 review, and S1, measuring the
  laptop (`measure_load`, Gazebo without its window, the physics step as a launch argument, and
  odometry checks from a bag). Next is S2a, Nav2 in the robot scope. Phase 3 (navigation with
  keep-out and speed zones, safety) is planned in two parts, each with its own learning report,
  ADR, GIF and tag: Phase 3a, Nav2 navigation (stages S0-S5), then Phase 3b, warehouse rules,
  safety and robot revision C (stages S6a-S11). Phase 2 (warehouse world, mapping, localization) is
  **done**: the warehouse generated from one layout file (merged 2026-11-06), the SLAM map made
  on a scripted survey route (2026-11-07) and AMCL localization on it (2026-11-08), with
  [ADR-007](decisions/ADR-007-generate-the-warehouse-from-one-layout.md), the
  [learning report](phase-reports/phase-02-warehouse-world.md) and the mapping GIF. Phase 1
  (robot description and first drive in Gazebo) is **done and merged** into `main` (pull
  request #2, 2026-11-05), including the owner's design review: a 360° lidar puck on the deck
  and a new look. Its [learning report](phase-reports/phase-01-robot-description.md) is
  written. Phases 0, 1 and 2 carry the tags `phase-0`, `phase-1` and `phase-2`. Still open: the
  Phase 1 demo GIF (recorded by the maintainer, Phase 1 report exercise 7) and the owner's own
  teleop map (Phase 2 report exercise 6).
- **Last updated:** 2026-11-13
- **What runs today** (on `main`): our own robot (revision B, safety yellow and
  charcoal) drives by keyboard (the teleop window) through ros2_control, in an empty Gazebo world or in the
  generated 30 x 20 m warehouse (`world:=warehouse`), inside the `/robot1` namespace, with a 360° lidar puck on its deck, an IMU and EKF odometry in RViz. The same launch file runs it on mock hardware
  without a simulator. In the warehouse it maps with slam_toolbox (`localization:=slam`) and
  localizes on the saved map with AMCL (`localization:=amcl`); command-line tools check a map
  against the layout, drive scripted routes, measure localization and odometry against Gazebo's
  ground truth from a bag, and measure what a run costs the laptop and how fast Gazebo runs.
  Gazebo runs with or without its window, and with a 1, 2 or 5 ms physics step for one launch.
  Measured in Gazebo: sensor rates
  10 / 100 / 50 Hz, 0 scanner self-hits, wheel odometry within 0.01 % of ground truth over
  10 m, EKF within 0.09-0.42 %; the saved map lies within 0.043 m and 0.015° of the layout;
  AMCL stays within 0.092 m of ground truth over a 7-minute, 175 m drive; the IMU noise is
  within 3 % of `robot.yaml`.
- **How changes are checked (since 2026-11-07):** there are no automated tests, lint checks or
  CI, and every commit goes straight to `main`, without branches or pull requests
  ([ADR-008](decisions/ADR-008-develop-on-main-without-automated-tests.md)). A change is checked by
  building it and running the system ([CONTRIBUTING](../CONTRIBUTING.md#build-and-run)).

### Progress

| Milestone | Phase | Scope | Status |
|---|---|---|---|
| M0 Foundation | 0 | Repository, lint gates, CI, dev containers, ADRs | **done** ([report](phase-reports/phase-00-foundation.md)); tag `phase-0` (annotated, on 73820c9, 2026-11-05, also on `origin`); its CI and lint gates removed on 2026-11-07 (ADR-008) |
| M1 Core AMR (`v1.0.0`) | 1 | Robot description, ros2_control drive, lidar + IMU, EKF, first drive in Gazebo | **done**, merged 2026-11-05 ([report](phase-reports/phase-01-robot-description.md)); tag `phase-1` on 14cb789, the merge of pull request #2; demo GIF pending |
| | 2 | Warehouse world generated from a layout file, mapping, localization | **done** 2026-11-08 ([report](phase-reports/phase-02-warehouse-world.md)); tag `phase-2` on 7da18a4 |
| | 3a | Nav2 navigation: Nav2 per robot, a command line for named locations, a navigation benchmark, the physics step, teleop and Nav2 through `twist_mux` with a manual mode | **in progress** (since 2026-11-13; stages S0 and S1 of S0-S5 done, S2a next) |
| | 3b | Warehouse rules, safety and robot revision C: keep-out and speed zones, safety fields, two low corner lidars | planned |
| | 4a | Interfaces + behaviour-tree task executor | planned |
| | 4b | Order manager, order generator, KPIs | planned |
| | 4c | Operator dashboard v1 (Flask): overview, orders, live map, live updates | planned |
| | 5 | Battery, autonomous docking, diagnostics | planned |
| | 6 | Core hardening and the `v1.0.0` release | planned |
| M2 Perception (`v2.0.0`) | 7 | ArUco fiducial perception (tote and dock tags) | planned |
| | 8 | Inventory patrol and discrepancy reports | planned |
| | 8b (optional) | Learned ML/DL perception, trained on Google Colab | planned (optional; research done, [design](PERCEPTION.md#5-method-2-learned-tote-detection)) |
| M3 Fleet (`v3.0.0`) | 9 | Multi-robot bring-up | planned |
| | 10 | Fleet manager v1, greedy task allocation | planned |
| | 11 | Auction allocation and aisle traffic control | planned |
| | 12 | Dashboard polish and the showcase scenario | planned |
| Stretch | 13 | Mobile manipulation | stretch |

This table follows [PROJECT_PLAN.md](PROJECT_PLAN.md) (version 2), which holds each phase's
scope and acceptance criteria; its §5 describes Phase 3 as one phase until Phase 3a's documents
divide it.

## Features

Only features that exist on `main` are listed. Commands assume the
repository is cloned as `~/warehouse_ws` (its root is the colcon workspace, ADR-006) and a shell with only ROS 2 Jazzy and this workspace
sourced (the [Phase 1 report](phase-reports/phase-01-robot-description.md#before-you-start)
has a `wh_shell` function for that).

### Repository skeleton

- **What it is:** the files every professional repository needs before any code: licence,
  README, editor and git settings, and the list of third-party sources.
- **What it does:** `.editorconfig` gives every editor the same indentation and line endings;
  `.gitattributes` forces LF line endings and marks images, bags and models as binary;
  `.gitignore` keeps colcon output, bags, datasets and model weights out of git;
  `dependencies.repos` lists source dependencies
  that have no Jazzy binary (none yet) for `vcs import`.
- **How to use it:** nothing to run; editors and git pick these files up automatically. To
  import third-party sources: `vcs import --input dependencies.repos src`
  from the workspace root.
- **Where it lives:** `LICENSE` (Apache-2.0), `README.md`, `.editorconfig`, `.gitattributes`,
  `.gitignore`, `dependencies.repos`.

### Robot description (`warehouse_description`, revision B)

- **What it is:** our own tote-carrying AMR ([plan §2.6](PROJECT_PLAN.md#26-robot-design-d1),
  [ADR-005](decisions/ADR-005-custom-tote-amr.md)), described once in `config/robot.yaml` and
  turned into URDF by xacro. Revision B: the drivable base (rev A) plus the first sensors.
- **What it does:** builds the robot for Gazebo (`hardware:=gz`) or for mock hardware without
  a simulator (`hardware:=mock`; `real` is reserved and refused): a 0.64 x 0.48 m, 35 kg
  differential-drive base with sphere wheel collisions and frictionless casters, a 360° lidar
  puck on the deck (`gpu_lidar`, scan plane 0.34 m, 10 Hz) and an IMU (100 Hz) read through
  ros2_control. The body is safety yellow and charcoal, built from parts that are easy to tell
  apart: charcoal skirt and deck top, yellow panels, black bumpers, a grey tote tray, a red
  emergency-stop button and a green status light (its colour is one property, which Phase 9
  makes per robot).
  Each coloured part is its own link, so RViz shows its colour too.
  `config/controllers.yaml` sets up `joint_state_broadcaster`, `diff_drive_controller` (hard
  limits in both directions of travel, unprefixed frames, no odometry transform) and
  `imu_sensor_broadcaster`. Every gz topic carries the robot namespace. No test checks the URDF
  or `controllers.yaml` against `robot.yaml` any more: check a change with the commands below
  and by running the robot
  ([warehouse_description README](../src/warehouse_description/README.md#checking-a-change)).
- **How to use it:** the launch files load it. To look at the URDF yourself:

  ```bash
  xacro $(ros2 pkg prefix warehouse_description)/share/warehouse_description/urdf/robot.urdf.xacro \
    namespace:=robot1 hardware:=gz > /tmp/robot.urdf
  check_urdf /tmp/robot.urdf          # prints the link tree
  ```

- **Where it lives:** `warehouse_description/` (`config/`, `urdf/`); the README has
  the mass budget and the design notes.

### Gazebo world and robot spawning (`warehouse_gazebo`)

- **What it is:** the simulator side: worlds, spawning, and the bridge between Gazebo and ROS
  topics.
- **What it does:** `gazebo.launch.py` starts Gazebo Harmonic with a world (headless or with a
  window) and bridges `/clock` once per world. It runs `gz sim` directly rather than through a
  shell, so one Ctrl-C (SIGINT) stops it. `gz_gui:=false` runs the Gazebo server without its
  window (since 2026-11-13; in `sim.launch.py` RViz stays on, while `headless:=true` turns both
  off). `physics_step:=0.001`, `0.002` or `0.005` runs one launch with that physics step: the
  world is copied into the launch's log directory with the new `<max_step_size>`, and any other
  value (4 ms, say) stops the launch before Gazebo starts, because the step must divide the
  10 ms period of the controller manager and the IMU. The world files keep 1 ms until stage S4
  decides. `spawn_robot.launch.py` spawns a robot from its
  `robot_description` and bridges its `scan` (sensor-data QoS) and `ground_truth/odom` (for
  the test tools and evaluation only). Two worlds: `empty` and `warehouse`, which
  `warehouse_layout` generates (next entry).
- **How to use it:** normally through `warehouse_bringup`'s `sim.launch.py`. On its own:
  `ros2 launch warehouse_gazebo gazebo.launch.py headless:=true`, or with
  `world:=warehouse physics_step:=0.002`. Measured on 2026-11-13, the robot standing headless:
  real-time factor 1.00 at 1, 2 and 5 ms, and Gazebo reported steps of exactly 1, 2 and 5 ms.
- **Where it lives:** `warehouse_gazebo/` (`launch/`, `worlds/empty.sdf`,
  `worlds/warehouse.sdf` (generated), `config/robot_bridge.yaml`).

### Warehouse layout and the generated world (`warehouse_layout`)

- **What it is:** the warehouse described in one file, `config/layout.yaml` (plan D8), and the
  pure-Python package that checks it and generates everything else from it. The robot's size
  comes from `robot.yaml` through `warehouse_description.robot_config`.
- **What it does:** a 30 x 20 m hall with 4 pick aisles in 2 blocks (8 shelf rows A-H, 414
  slots), a packing station, 3 charging docks, spawn poses for 3 robots and an open handling
  floor in the east; plain uniform racks with no totes until Phase 7. It checks the layout
  against a schema and 9 fit rules (aisle width for the robot, everything the lidar must see
  solid up to 0.40 m, pick poses clear of shelves, ...), draws a to-scale preview, and
  generates three committed files: the Gazebo world (`warehouse_gazebo/worlds/warehouse.sdf`,
  31 static models), the location registry (`generated/locations.yaml`: slots, 36 pick poses,
  docks, the drop point, spawns) and the expected inventory (`generated/expected_inventory.yaml`:
  207 totes, seed 2, data for Phase 8). The output is byte-identical run to run, and
  `generate --check` fails when a committed file is stale. `sim.launch.py world:=warehouse`
  spawns each robot at its spawn pose from the registry; values given on the command line win.
  Measured in Gazebo: spawn pose exact, controllers active, scan at 10 Hz with ranges within
  2 cm of the layout, real-time factor 1.01.
- **How to use it:**

  ```bash
  ros2 run warehouse_layout preview --out /tmp/layout.png    # picture + one PASS/FAIL line per rule
  ros2 run warehouse_layout generate                         # after editing layout.yaml (from ~/warehouse_ws)
  ros2 run warehouse_layout generate --check                 # exit 1 if a generated file is stale
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse
  ```

- **Where it lives:** `warehouse_layout/` (`config/layout.yaml`, `warehouse_layout/`,
  `generated/`); its README has the schema, the fit rules and the edit workflow, and
  `warehouse_bringup`'s README explains where each robot spawns.

### Odometry EKF (`warehouse_navigation`)

- **What it is:** a `robot_localization` extended Kalman filter, the robot's odometry
  ([ADR-005](decisions/ADR-005-custom-tote-amr.md), plan D3).
- **What it does:** fuses the wheel odometry (forward speed, the diff drive's zero sideways
  speed, yaw rate) with the IMU yaw rate, and publishes `odom` and the transform
  `odom -> base_footprint`. Over a 10 m drive in Gazebo its end point is within 0.09-0.42 % of
  ground truth (limit 1 %), over a 360° turn within 0.05 % (limit 2 %). The wheel odometry
  alone is closer (0.01 %): in Gazebo the controller's reported speed reads about 0.2 % high,
  and the EKF integrates that speed (Phase 1 report, concept 9). On mock hardware the IMU
  reports no turning, so there the EKF follows straight driving only.
- **How to use it:** it starts with every robot. Watch it with
  `ros2 topic echo /robot1/odom --field pose.pose`.
- **Where it lives:** `warehouse_navigation/config/ekf.yaml`,
  `warehouse_navigation/launch/ekf.launch.py`. SLAM and AMCL (Phase 2, the entries below) live in
  the same package; Nav2 joins them in Phase 3.

### Map tools (`warehouse_layout`)

- **What it is:** two commands that connect the layout with maps: `rasterize` draws the map an
  ideal SLAM run would make, and `check_map` measures how well a saved map lines up with the
  layout (plan F5, P2-D13).
- **What it does:** `rasterize` cuts every solid of `layout.yaml` at the lidar's scan plane
  (0.34 m) and writes a map in map_server's format (YAML and PGM, 0.05 m cells, map frame = world
  frame, yaw 0): the faces the lidar can see occupied, the hall floor free, the inside of solids
  and the space behind the walls unknown (636 x 428 cells, 6473 occupied). It is the fallback
  map of plan risk R6 and the grid of the Phase 3 masks. `check_map` fits a map's occupied cells
  to those faces (point-to-plane ICP) and prints one `[PASS]` or `[FAIL]` line per mapping limit:
  best-fit offset ≤ 0.05 m, rotation ≤ 0.2°, 95 % of occupied cells within 0.10 m of a face, at
  most 1 % farther than 0.25 m (ghost walls), origin yaw 0. `--overlay` draws the map over the
  layout. Measured: the rasterized map fits within 0.001 m, a copy shifted by 0.15 m fails with
  0.150 m, and the committed SLAM map passes with 0.043 m and 0.015°.
- **How to use it:**

  ```bash
  ros2 run warehouse_layout rasterize --out /tmp/layout_map     # writes layout_map.yaml and .pgm
  ros2 run warehouse_layout check_map src/warehouse_navigation/maps/warehouse.yaml \
    --overlay /tmp/overlay.png                                   # 5 [PASS] lines, exit 0
  ```

- **Where it lives:** `warehouse_layout/warehouse_layout/` (`rasterize.py`, `check_map.py`,
  `occupancy.py`); the package README's "Map tools" section explains the map files and the fit.

### SLAM mapping (`warehouse_navigation`, slam_toolbox)

- **What it is:** online, asynchronous SLAM with slam_toolbox 2.8.5: the robot builds the map of
  the warehouse while it drives. It made the map AMCL uses.
- **What it does:** `localization:=slam` adds slam_toolbox to the robot's namespace, configured
  and activated by launch events (no lifecycle manager). It reads `/robot1/scan`, publishes
  `map -> odom` on `/robot1/tf` and the map on `/map` every 2 s, which RViz (`map.rviz`) shows
  growing. Its configuration is slam_toolbox's own sample with every key written out and each
  change explained in the package README. The committed map `maps/warehouse.{yaml,pgm}` (611 x 403 cells of 0.05 m)
  was made on the scripted survey route: 175.6 m in 9 min 40 s, real-time factor 0.999;
  `check_map` 0.043 m, 0.015°, P95 0.055 m, no ghost walls. It is the best of three runs made
  the same way; one missed the offset limit by 4 mm.
- **How to use it:** each command in its own terminal; `maps/README.md` has the full procedure
  with the recorder:

  ```bash
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam
  ros2 lifecycle get /robot1/slam_toolbox                       # active [3]
  ros2 run warehouse_system_tests drive_route --route survey --speed 0.4 --turn-rate 0.2
  ros2 run nav2_map_server map_saver_cli -f /tmp/my_map -t /map --fmt pgm \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
  ros2 run warehouse_layout check_map /tmp/my_map.yaml --overlay /tmp/my_overlay.png
  ```

- **Where it lives:** `warehouse_navigation/` (`config/slam_toolbox.yaml`,
  `launch/slam.launch.py`, `maps/` with its README: how the map was made, every run's numbers,
  how to make it again).

### AMCL localization (`warehouse_navigation`)

- **What it is:** Nav2's AMCL (adaptive Monte Carlo localization), a particle filter that keeps
  the robot's pose on the saved map, with `map_server` serving the map.
- **What it does:** `localization:=amcl` starts `map_server` and its lifecycle manager in `/` (the
  map on `/map`, reliable and transient local, so a late subscriber still gets it), and AMCL and
  its lifecycle manager in `/robot1`, starting at the spawn pose. AMCL publishes `map -> odom` on
  `/robot1/tf`, `amcl_pose` and `particle_cloud`. `map_server` is active 1.5-1.7 s and AMCL
  2.4-4.9 s after the launch. Tuned on the evaluation: 240 beams per scan and odometry noise 0.02 (Nav2's sample: 60
  and 0.2). Measured on the survey route at 0.5 m/s, started (0.3, -0.3) m and 8° off: converged
  after 0.78 m, then within 0.092 m of ground truth (mean 0.024 m) for 7 min 07 s; a repeat run
  converged after 1.22 m and stayed within 0.090 m.
- **How to use it:**

  ```bash
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl
  ros2 lifecycle get /map_server && ros2 lifecycle get /robot1/amcl   # active [3] both
  ros2 run tf2_ros tf2_echo map base_footprint \
    --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
  ```

  In RViz, the "2D Pose Estimate" tool puts AMCL somewhere else; drive and watch the particle
  cloud find the robot again. `map:=` serves another map, for example the rasterized one.
- **Where it lives:** `warehouse_navigation/` (`config/amcl.yaml`, `config/map_server.yaml`,
  `launch/localization.launch.py`, `launch/map_server.launch.py`); the README explains the
  frames, scopes, QoS, lifecycle and every changed parameter.

### Simulation bring-up (`warehouse_bringup`)

- **What it is:** the top-level launch files, the entry point of every demo.
- **What it does:** `sim.launch.py` starts Gazebo, one robot in its namespace and RViz, and
  sets `use_sim_time` for all of them; with `hardware:=mock` it runs the same robot stack on
  mock hardware, without a simulator. `robot.launch.py` is the robot itself:
  `robot_state_publisher`, the controller manager, the three controllers and the EKF inside
  `/<namespace>`, with the robot's transforms on `/<namespace>/tf`. It sets `use_sim_time` and
  an absolute namespace itself, so it also runs on its own next to `gazebo.launch.py` or
  inside a fleet launch (Phase 9). `localization:=slam` adds slam_toolbox to the robot;
  `localization:=amcl` adds AMCL to the robot and the map server (the `map:=` file, by default
  the committed warehouse map) in the system scope. RViz then opens `map.rviz` (fixed frame
  `map`) instead of `robot.rviz` (fixed frame `odom`). Mock hardware has no lidar, so it refuses
  both before anything starts.
- **How to use it:** from `~/warehouse_ws`:

  ```bash
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install && source install/setup.bash
  ros2 launch warehouse_bringup sim.launch.py                   # Gazebo + RViz, robot in /robot1
  ros2 launch warehouse_bringup sim.launch.py headless:=true    # no windows
  ros2 launch warehouse_bringup sim.launch.py gz_gui:=false     # RViz without the Gazebo window
  ros2 launch warehouse_bringup sim.launch.py physics_step:=0.002   # 2 ms physics steps
  ros2 launch warehouse_bringup sim.launch.py hardware:=mock    # no simulator
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam   # map
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl   # localize
  ros2 control list_controllers -c /robot1/controller_manager   # three controllers, all active
  ```

  Check a change with these commands and the others in
  [Checking the robot by hand](../src/warehouse_bringup/README.md#checking-the-robot-by-hand).
- **Where it lives:** `warehouse_bringup/` (`launch/`, `rviz/robot.rviz`, `rviz/map.rviz`,
  README with every argument and the checks to run by hand).

### Keyboard teleoperation

- **What it is:** driving the robot by keyboard. The teleop window of `warehouse_bringup`
  (since 2026-11-06) drives while a key is held and stops the moment it is released;
  `teleop_twist_keyboard` in a terminal, the Phase 1 demo, is the alternative without a desktop.
- **What it does:** both publish `TwistStamped` commands on `/robot1/cmd_vel`, stamped with the
  controller's clock (sim time in Gazebo), so `diff_drive_controller` brakes once the newest
  command is 0.15 s old (`cmd_vel_timeout`; 0.5 s until 2026-11-06). The window needs no
  arguments: its node starts in `/robot1`, and it chooses the clock itself (sim time as soon as
  `/clock` has a publisher, wall time once the robot has listened 2 s without one). It publishes
  only while the line at its top is green. The window sees key presses and releases: it publishes
  at once, repeats the command at 20 Hz while a key is held, sends three zero commands over 0.1 s
  after the release and then nothing, and sends one zero command when it loses the keyboard
  (another window clicked, window closed, Ctrl-C). Its speeds start at 0.3 m/s and 0.6 rad/s, so
  braking takes 0.2 s; Q/Z and E/C change them. Measured in headless Gazebo on 2026-11-06 at
  0.5 m/s and 1.0 rad/s: after a release the robot stands still in 0.34 s, 8.3 cm or 9.7° further
  on (teleop_twist_keyboard: 0.49 s after the last key event, 16.0 cm or 18.2°), and a held key
  drives without teleop_twist_keyboard's 0.2-0.3 s stop; at the window's defaults, 0.21 s and
  3.1 cm or 3.5°. A tap of `i` with teleop_twist_keyboard moves 1.4 cm and a tap of `j` turns
  3.9° (15.4 cm and 28.6° with the old 0.5 s timeout); a 0.1 s tap in the window moves 0.7 cm or
  turns 1.7°. The controller's hard limits apply whatever teleop asks for (1.2 m/s forward,
  0.3 m/s reverse, 2 rad/s).
- **How to use it:** with the simulation running (`ros2 launch warehouse_bringup
  sim.launch.py`), in a second terminal set up like the first (the same workspace sourced, the
  same `ROS_DOMAIN_ID`):

  ```bash
  ros2 run warehouse_bringup teleop_window                               # drives robot1
  ros2 run warehouse_bringup teleop_window --ros-args -r __ns:=/robot2   # another robot
  # or, in a terminal without a desktop:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r __ns:=/robot1 -p stamped:=true -p use_sim_time:=true
  ```

  In the window: click it, hold the arrows or W A S D to drive, Space stops. The line at its
  top is green ("Ready") when the keys drive; amber while it looks for the robot or chooses the
  clock (2 s at most once the robot is there); red when nothing listens on the topic it names
  (the simulation is not running, it runs with another `ROS_DOMAIN_ID`, or the robot is in
  another namespace, which it names when it finds it), or when it is on sim time and nothing
  publishes `/clock`. If the keys do nothing: click the window; read the line at the top; start
  the window from a terminal set up like the simulation's; hold ↑ and watch the Command line,
  which follows the keys whenever they reach the window, even with no robot. In
  teleop_twist_keyboard: `i` forward, `,` backward, `j`/`l` turn, `k` stop. Leave out
  `use_sim_time:=true` for `hardware:=mock`. Without it in Gazebo, the wall-clock stamps never
  look old and the robot keeps driving after you release the key (plan R1).
- **Where it lives:** `warehouse_bringup/warehouse_bringup/` (`hold_to_drive.py`, the key logic,
  and `connection.py`, the clock choice and the line at the top, both with no Qt and no ROS;
  `teleop_window.py`, the window and its node), and
  `warehouse_bringup/README.md` ("Driving with teleop", with the keys, the status line, a
  checklist for keys that do nothing, parameters and measurements); `teleop_twist_keyboard` is
  an exec dependency of `warehouse_bringup`.

### Simulation test tools (`warehouse_system_tests`)

- **What it is:** command-line tools that check the robot in simulation against Gazebo's ground
  truth; never part of the robot stack. Until 2026-11-07 the package also held the launch tests
  of the bring-up, removed with every other test ([ADR-008](decisions/ADR-008-develop-on-main-without-automated-tests.md)); it keeps
  its name.
- **What it does:** `drive_route` drives a route of the location registry (`survey`, 176 m, the
  mapping run; `smoke`, 24 m) with commands at 20 Hz stamped in sim time, steering on ground
  truth; `evaluate_localization` reads a bag and reports AMCL's error against ground truth, the
  EKF's drift and the IMU noise, with a `[PASS]` or `[FAIL]` line per Phase 2 criterion, and with
  `--odometry` (since 2026-11-13) wheel odometry and the EKF against ground truth on every
  straight and every turn in place, judged with Phase 1's limits (1 % on straights of 10 m or
  more, 2 % on turns of 80° or more); `map_timelapse` turns a mapping bag into a GIF;
  `measure_load` (since 2026-11-13) samples the CPU per process group, the machine total,
  memory, clocks and heat, and Gazebo's real-time factor and physics step, without being a ROS
  node itself. Measured: the survey route at 0.4 m/s took
  9 min 40 s of sim time, commands at 20.00 Hz; `evaluate_localization` on the acceptance bag
  passes 4 of 4 checks, and with `--odometry` 8 of 8 (wheels at most 0.005 %, EKF at most
  0.26 %); `map_timelapse` made the Phase 2 GIF (60 frames, 0.18 MB) in about 3 s;
  `measure_load` itself uses 0.03-0.04 cores.
- **How to use it:** with `ros2 launch warehouse_bringup sim.launch.py world:=warehouse` running:

  ```bash
  ros2 run warehouse_system_tests drive_route --route smoke --dry-run   # print, touch nothing
  ros2 run warehouse_system_tests drive_route --route survey --speed 0.4
  ros2 run warehouse_system_tests evaluate_localization bags/loc_run --plot /tmp/loc.png
  ros2 run warehouse_system_tests evaluate_localization bags/loc_run --odometry
  ros2 run warehouse_system_tests map_timelapse bags/mapping docs/media/phase-02-mapping.gif
  ros2 run warehouse_system_tests measure_load --world warehouse --duration 60
  ```

  The package README says how to record the bags.
- **Where it lives:** `warehouse_system_tests/` (`scripts/`, `warehouse_system_tests/`, README
  with every option).

### Branch protection (ruleset "Protect main")

- **What it is:** a GitHub repository ruleset on the default branch.
- **What it does:** blocks force-pushes and the deletion of `main`. Every change is committed
  directly to `main`: no branches or pull requests since 2026-11-07
  ([ADR-008](decisions/ADR-008-develop-on-main-without-automated-tests.md)). No checks or reviews are required.
- **Status:** the ruleset keeps its "Require a pull request before merging" rule until the owner
  removes it on GitHub (Settings → Rules → Rulesets → Protect main); until then GitHub refuses a
  direct push to `main`.
- **How to use it:** commit on `main` and push. Inspect the rules with
  `gh api repos/TANAY779/ros2-warehouse-amr/rules/branches/main --jq '.[].type'` (prints
  `deletion` and `non_fast_forward`; `pull_request` too until that rule is removed).
- **Where it lives:** GitHub repository settings (Settings → Rules → Rulesets), not in the
  repository. To recreate it (for example on a fork): new branch ruleset named "Protect
  main", enforcement **Active**, no bypass list, target **Default branch**; enable **Restrict
  deletions** and **Block force pushes**, nothing else.

### Dev containers (GUI and headless)

- **What it is:** a Docker development environment for VS Code (Dev Containers extension),
  based on `osrf/ros:jazzy-desktop-full` (ROS 2 Jazzy plus Gazebo Harmonic) with clang-format
  and gdb added.
- **What it does:** gives a clean, reproducible ROS 2 environment. The
  **ros2-warehouse-amr (ROS 2 Jazzy)** configuration is for a Linux desktop: GPU
  access through `/dev/dri` and GUI windows (Gazebo, RViz) through X11, granted only to your
  user. The **ros2-warehouse-amr (headless)** configuration has the same tools without GPU or
  GUI, for macOS, Windows, servers or Codespaces (Codespaces not tested yet). Both set
  `ROS_DOMAIN_ID=42` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, install the dependencies on
  creation, and point VS Code's C++ formatter at the system clang-format and the repository's
  `.clang-format`.
- **How to use it:** open the repository in VS Code and run **Dev Containers: Reopen in
  Container**, then pick a configuration. The workspace is `/home/ros/ws`. Details and
  security notes are in the README.
- **Where it lives:** `.devcontainer/Dockerfile`, `.devcontainer/devcontainer.json`,
  `.devcontainer/headless/devcontainer.json`, `.devcontainer/post-create.sh`.

### Architecture decision records (ADRs)

- **What it is:** one short document per significant decision: context, decision, rejected
  alternatives, consequences.
- **What it does:** keeps the reasons behind the design findable. ADRs are never deleted; a
  changed decision gets a new ADR that supersedes the old one.
- **How to use it:** copy `docs/decisions/ADR-000-template.md` to the next number and add a
  row to the index.
- **Where it lives:** [docs/decisions/](decisions/) (ADR-001 to ADR-008 so far).

### Contribution workflow and conventions

- **What it is:** the written rules for commits, tags, code style, licence headers, media sizes
  and new packages. Every commit goes straight to `main`.
- **What it does:** keeps every change consistent: Conventional Commits with
  the package as scope, a `phase-N` tag per finished phase, release tags at milestones,
  double-quoted ROS includes, the Apache-2.0 header in every source file.
- **How to use it:** read it before the first change.
- **Where it lives:** [CONTRIBUTING.md](../CONTRIBUTING.md).

## Ideas and decisions log

Oldest first; each row is dated by the day the idea or decision was first made. Status
values: **adopted** (in effect), **done** (a one-off action that is complete), **planned**
(agreed, not built yet), **paused**, **replaced** (superseded by a later entry), **in review**.

| Date | Idea or decision | Why | Status | Reference |
|---|---|---|---|---|
| 2026-11-02 | Build a simulated warehouse AMR system: order fulfilment first, then perception, then a fleet | A realistic, demo-able system that covers the core of professional ROS 2 work; doubles as a portfolio | adopted | [Plan](PROJECT_PLAN.md) |
| 2026-11-02 | Stack: ROS 2 Jazzy, Gazebo Harmonic, Nav2; phased plan with a recorded demo per phase | Jazzy is a long-term-support distribution (supported until 2029) and Harmonic is its paired Gazebo release; small phases keep progress visible | adopted | [Plan](PROJECT_PLAN.md) |
| 2026-11-02 | Record decisions as ADRs | Reasons are easy to forget over months; reviewers want the "why" | adopted | [ADR-001](decisions/ADR-001-record-architecture-decisions.md) |
| 2026-11-02 | One multi-package repository inside a colcon workspace; packages prefixed `warehouse_`, created only when needed | Packages change together; keeps build output out of the repository | adopted | [ADR-002](decisions/ADR-002-repository-layout.md) |
| 2026-11-02 | clang-format (not uncrustify) for C++, flake8 for Python, both pinned to the ament versions; cpplint kept | One style across editor, pre-commit and `colcon test`; cpplint proved compatible | replaced (2026-11-07, ADR-008) | [ADR-003](decisions/ADR-003-formatting-and-linting.md) |
| 2026-11-02 | CI runs plain rosdep/colcon commands in `ros:jazzy-ros-base` instead of `action-ros-ci` | A CI failure must be reproducible locally with the same commands | replaced (2026-11-07, ADR-008) | [ADR-004](decisions/ADR-004-ci-pipeline.md) |
| 2026-11-02 | Operator UI is our own Flask web app on the LAN; Foxglove and rosbridge dropped | Own UI is a stronger learning and portfolio piece; fewer moving parts | adopted (replaces Foxglove) | Plan v2, Phase 4c |
| 2026-11-02 | Deployment (running the system as a service on the LAN) after all phases | Keep phases focused on robotics | planned | Plan v2 |
| 2026-11-02 | Perception method 1: ArUco fiducial markers on shelves and docks | Deterministic, cheap on CPU, ideal for a sim-only project | planned (Phases 7-8) | Plan v2 |
| 2026-11-02 | Perception method 2: learned ML/DL detector, trained on a Google Colab Pro GPU | Learn a modern perception pipeline without a local NVIDIA GPU | planned (optional Phase 8b) | Plan v2 |
| 2026-11-02 | Simulation only, no hardware | No robot available; ros2_control keeps the stack hardware-ready | adopted | Plan, assumption A1 |
| 2026-11-02 | Our own custom robot (a tote-carrying AMR), refined in revisions across phases | Biggest learning payoff (URDF, ros2_control, sensors); a TurtleBot would read as a tutorial | adopted | Plan, decision D1 |
| 2026-11-03 | Split Phase 4 into 4a (interfaces + BT task executor), 4b (order manager, generator, KPIs), 4c (dashboard v1, minimal) | Phase 4 was too large for one PR; `v1.0.0` ships with the dashboard | adopted | Plan v2 |
| 2026-11-03 | ML/DL perception research paused by the maintainer (3 of 4 topics done) | Focus on the core stack first | replaced (resumed 2026-11-06) | Plan v2, Phase 8b |
| 2026-11-03 | Repository named `ros2-warehouse-amr` (was `warehouse_amr`); ROS package names keep the `warehouse_` prefix | Descriptive, searchable name for a public repo | adopted | README |
| 2026-11-03 | Tags: `phase-N` marker per finished phase; releases `v1.0.0`, `v2.0.0`, `v3.0.0` at milestones; packages stay at `0.1.0` until `v1.0.0` | A per-phase version tag (`v0.N.0`) would make versions go backwards after `v1.0.0` (Phase 7 would be `v0.7.0`) | adopted (replaces per-phase version tags) | [CONTRIBUTING.md](../CONTRIBUTING.md) |
| 2026-11-03 | Environment notes: the local workspace was cleaned of files from earlier experiments; local shells may source other workspaces, so CI and the dev container are the clean reference | A clean, reproducible reference without disturbing other local work | adopted | [Dev containers](#dev-containers-gui-and-headless) |
| 2026-11-03 | Phase 0 verified in Docker by parallel AI reviewers (Claude Code agents); every bug found was fixed and re-verified | Tooling that silently does nothing is worse than none | done | [Phase 0 report](phase-reports/phase-00-foundation.md#what-happened) |
| 2026-11-03 | cppcheck enabled with `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` in CI, the dev container and native setups | `ament_cppcheck` silently skips cppcheck 2.x otherwise | replaced (2026-11-07, ADR-008) | [ADR-003](decisions/ADR-003-formatting-and-linting.md) |
| 2026-11-03 | File size limits: 1 MB per file, 3 MB for GIFs in `docs/media/`, enforced in CI; long demos as MP4 on GitHub Releases | Keep clones small; git never forgets a large file | adopted; not enforced since 2026-11-07 (ADR-008) | [CONTRIBUTING.md](../CONTRIBUTING.md) |
| 2026-11-03 | Dev container: per-user X11 access, headless variant, render-group warning, CAP_SYS_PTRACE for gdb | Found during verification: the first version leaked X11 access and could not start on non-Linux hosts | adopted | [Dev containers](#dev-containers-gui-and-headless) |
| 2026-11-03 | Repository published on GitHub; `main` protected by a ruleset (PR plus both CI checks required, no force-push or deletion); Dependabot enabled | Nothing reaches `main` without passing CI | done; ruleset and Dependabot replaced (2026-11-07, ADR-008) | [Features](#branch-protection-ruleset-protect-main) |
| 2026-11-04 | An accidental git repository at the workspace root (created from VS Code; it recorded this repository as an embedded repository) was moved aside; the workspace is not version-controlled | ADR-002: only `src/ros2-warehouse-amr` is a repository | done | [ADR-002](decisions/ADR-002-repository-layout.md) |
| 2026-11-04 | Keep this living status document plus one learning report per finished phase | The maintainer learns ROS 2 alongside the build | adopted | [Maintenance rules](#how-this-document-is-maintained) |
| 2026-11-04 | Plan v2: robot design in revisions (Rev A-G), Phases 4a/4b/4c, corrected estimates | Merge the maintainer's answers and the research into one plan | adopted | [Plan](PROJECT_PLAN.md) |
| 2026-11-05 | Our own tote AMR from one `robot.yaml`, driven through ros2_control (`hardware:=gz\|mock`), odometry from a `robot_localization` EKF | Teaches URDF, ros2_control and sensor fusion; mock hardware lets CI test the robot without Gazebo | adopted | [ADR-005](decisions/ADR-005-custom-tote-amr.md) |
| 2026-11-05 | CI's build-and-test job: LOCALHOST discovery for launch tests, Gazebo tests reported as skipped (`WAREHOUSE_SKIP_GZ_TESTS=1`) | The runners cannot render the scanner; the mock-hardware tests still run on every PR | replaced (2026-11-07, ADR-008) | [ADR-004](decisions/ADR-004-ci-pipeline.md) |
| 2026-11-05 | `controllers.yaml` sets the reverse acceleration limits explicitly | Found in review: left unset, braking from reverse fell back to the forward acceleration limit (0.8 instead of 1.5 m/s²); the Phase 3 reverse safety field relies on 1.5 | adopted | [warehouse_description](../src/warehouse_description/README.md#design-notes) |
| 2026-11-05 | The EKF fuses the diff drive's zero sideways speed (vy = 0) | Found in review: unfused, the y variance of `odom` grew without bound (about 4e4 m² after 30 s standing still, 7e5 m² after a minute); robot_localization recommends it | adopted | [ADR-005](decisions/ADR-005-custom-tote-amr.md), plan D3 |
| 2026-11-05 | `robot.launch.py` sets `use_sim_time` and an absolute namespace itself | Found in review: run on its own, its nodes ran on wall time and the EKF published nothing; inside another namespace, the ROS side moved away from the absolute gz names | adopted | [warehouse_bringup](../src/warehouse_bringup/README.md) |
| 2026-11-05 | The scanner self-hit test looks at a box placed 2 m from the robot | Found in review: in the empty world every ray missed, so a blind or misplaced scanner passed | replaced (2026-11-07, ADR-008) | [warehouse_system_tests](../src/warehouse_system_tests/README.md) |
| 2026-11-05 | Namespaced from Phase 1: every robot node and topic under `/robot1`, the robot's TF on `/robot1/tf` through `SetRemap`, unprefixed frames, gz topics prefixed with the namespace, `/**/` keys in parameter files | Phase 9 then only adds namespaces; gz topics are global, so unprefixed ones would merge robots; plain parameter keys silently match nothing in a namespace | adopted | Plan D7, R2, R12; [Phase 1 report](phase-reports/phase-01-robot-description.md#10-namespaces-and-the-tf-remaps) |
| 2026-11-05 | Sphere wheel collisions, frictionless caster spheres, unnamed `<collision>` elements | Cylinder wheels skewed in-place yaw odometry by 8-9 % in the prototype; Gazebo's URDF converter drops `<gazebo reference>` friction from named collisions (found while building) | adopted | Plan R13; [warehouse_description](../src/warehouse_description/README.md#design-notes) |
| 2026-11-05 | One static bridge file (`robot_bridge.yaml`, relative names, `expand_gz_topic_names: true`) instead of a bridge config generated per robot | `ros_gz_bridge` 1.0.24 expands relative gz names in the bridge's namespace, so one file serves every robot with no generator code | adopted (replaces the plan's per-robot generation) | [warehouse_gazebo](../src/warehouse_gazebo/README.md#topics) |
| 2026-11-05 | `gazebo.launch.py` starts `gz sim` directly instead of `ros_gz_sim`'s `gz_sim.launch.py` | That file runs Gazebo through a shell, so launch's SIGINT stopped only the shell and orphaned Gazebo servers broke later test runs | adopted | [warehouse_gazebo](../src/warehouse_gazebo/README.md#why-gz-sim-is-started-directly) |
| 2026-11-05 | Launch tests: each file in its own ROS domain (`run_test_isolated.py`), a private `GZ_PARTITION` for Gazebo, exit code 0 required from every process after one SIGINT, no leftover process; robot numbers read from `robot.yaml` and `controllers.yaml` | Tests must not see each other or other sessions, must catch shutdown leaks, and must not check stale values after a robot revision | replaced (2026-11-07, ADR-008) | [warehouse_system_tests](../src/warehouse_system_tests/README.md) |
| 2026-11-05 | Values the plan left open: wheel width 0.04 m, wheel motor effort limit 20 N m, mass split body 32.6 kg / wheels 2 x 1.0 kg / casters 2 x 0.2 kg, sensors massless | URDF needs them; totals (35 kg, centre of mass 0.14 m) match plan §2.6 and are tested | adopted | [warehouse_description](../src/warehouse_description/README.md#mass-budget) |
| 2026-11-05 | Upstream Gazebo shutdown hang ([gz-sim#4173](https://github.com/gazebosim/gz-sim/issues/4173)) accepted as a known issue: re-run when only the exit-code check fails | About 1 Gazebo test run in 15; not caused by this repository, CI unaffected (it skips the Gazebo test) | replaced (2026-11-07, ADR-008) | [warehouse_gazebo](../src/warehouse_gazebo/README.md#checking-a-change) |
| 2026-11-05 | Phase 1 built, reviewed, fixed and verified by AI agents (Claude Code): one builder, one review and fix round, three parallel final verifiers (CI reproduction, Gazebo acceptance, code review against the Jazzy sources) | Same practice as Phase 0; every acceptance criterion is measured, and every exercise in the report was run again in a clean shell (except the windowed demo) | done | [Phase 1 report](phase-reports/phase-01-robot-description.md#what-happened) |
| 2026-11-05 | Learning-report exercises and local runs use a clean shell (`wh_shell`: only `/opt/ros/jazzy` and this workspace, `ROS_DOMAIN_ID=42`, LOCALHOST discovery) | The maintainer's normal shell sources other workspaces and discovers across the LAN, which can shadow packages and mix graphs | adopted | [Phase 1 report](phase-reports/phase-01-robot-description.md#before-you-start) |
| 2026-11-05 | A render-free test of the Gazebo-side files (`warehouse_gazebo/test/test_gazebo_config.py`) runs in every pull request: the systems every world loads, the bridge file against the URDF (gz names expanded the way the bridge expands them, the same relative name and the matching ROS type on the ROS side, the ground-truth system's filename), and the caster friction after `gz sdf -p` | CI skips the Gazebo launch test, so a typo in the world or the bridge file passed CI: `gz sdf -k` accepts a misspelt system, a bridge entry for a topic nothing publishes fails silently, and one with a misspelt ROS type only logs a warning. Each of 19 single mistakes made on purpose fails the test, among them a misspelt system, an absolute scan topic, a misspelt or wrong ROS type, a misspelt plugin filename and a named caster collision | replaced (2026-11-07, ADR-008) | [warehouse_gazebo](../src/warehouse_gazebo/README.md#checking-a-change) |
| 2026-11-05 | Launch files keep rcutils' default log format: `sim.launch.py` no longer sets `RCUTILS_CONSOLE_OUTPUT_FORMAT`, and plan §6 says to override it only to add fields | The value it set was rcutils' built-in default (`[{severity}] [{time}] [{name}]: {message}`), so it changed nothing, and a reader would copy a no-op | adopted | Plan §6 |
| 2026-11-05 | Phase 1 acceptance reworded: after the key is released, the controller starts braking when `cmd_vel_timeout` (0.5 s) expires and stands still about 0.33 s later, at the -1.5 m/s² limit | "Stops within `cmd_vel_timeout`" cannot hold with a deceleration limit; the new wording is what the tests measure | replaced (0.15 s since 2026-11-06) | Plan, Phase 1 |
| 2026-11-05 | Phase 1 polish round before the pull request: the final review's six low findings and a fact-check of the learning report fixed (EKF explanation, RViz's absolute namespace, a discovery wait in the Gazebo test, the plan's references to ADR-005 and the static bridge file); the IMU sensor-name risk is now recorded in plan R2 and stays open until Phase 9. Two verifiers then checked the round, and a second, smaller round fixed what they found (the ROS side of the bridge test, the README status, the EKF speed explanation, two wordings) | Every document agrees with the code and with re-measured numbers before the merge | done | [Phase 1 report](phase-reports/phase-01-robot-description.md#what-happened) |
| 2026-11-05 | Automatic checks paused: CI runs only by hand, `main` needs a pull request but no passing checks, the pre-commit hook is optional, and a plain `colcon build` builds without tests (`colcon.meta` at the workspace root passes `-DBUILD_TESTING=OFF` to every package) | The owner prefers to verify changes by building and running them while the project is under heavy development; the tests stay in the repository for later | replaced (2026-11-07, ADR-008) | [ADR-004](decisions/ADR-004-ci-pipeline.md), [CONTRIBUTING](../CONTRIBUTING.md#build-and-run) |
| 2026-11-05 | The owner's design review of Phase 1: the 2D lidar becomes a 360° puck standing on the deck near the front edge (scan plane 0.34 m, 720 rays 0.5° apart) instead of a front-left corner scanner in a groove round the body (scan plane 0.20 m, 264°), accepting that a loaded tote blocks much of its rear view. The body is restyled in safety yellow and charcoal from simple, clearly separate parts (skirt, panels, bumpers, deck top, tote tray, emergency stop, status light); the status light's colour is one property, which Phase 9 turns into a per-robot argument | A simpler, familiar sensor layout and a robot that reads as a warehouse AMR. Found while building it: RViz draws all visuals of a link in one colour, so each coloured part is its own link. Open for Phase 3: what the puck cannot see (below 0.34 m, behind a tote), and when a tote is modelled on the robot (no phase does it yet) | adopted | [ADR-005](decisions/ADR-005-custom-tote-amr.md), plan §2.6 (Rev B, Rev C) |
| 2026-11-05 | The repository root is now the colcon workspace: the repository is cloned as `~/warehouse_ws`, packages moved to `src/`, `.gitignore` keeps `build/`, `install/` and `log/` out; CI, the dev container, README, CONTRIBUTING and the reports follow. History is kept (the move is one rename commit) | The owner wants one repository at the workspace root that VS Code finds directly; GitHub still finds the README, licence and `.github/` at the root | adopted | [ADR-006](decisions/ADR-006-workspace-root-repository.md) |
| 2026-11-05 | Phase 2 warehouse: a 30 x 20 m hall, 4 pick aisles in 2 blocks (8 shelf rows, 414 slots), an open handling floor in the east; plain uniform racks with no totes and no per-aisle colours until Phase 7 (colours live in the layout's `style:` block) | The owner's choices in the Phase 2 plan review: big enough for 3 robots and aisle traffic in Phase 11, small enough to map quickly; totes arrive with their tags in Phase 7 | adopted | [warehouse_layout](../src/warehouse_layout/README.md) |
| 2026-11-05 | One `layout.yaml` generates the Gazebo world, the location registry and the expected inventory; the generated files are committed, byte-identical run to run, and `generate --check` catches a stale one; fit rules are tests that read the robot's size from `robot.yaml` (`warehouse_description.robot_config`) | Plan D8: one source of truth, so the world, the waypoints and the inventory can never disagree; a reviewer sees every change to the world in the diff | adopted | [warehouse_layout](../src/warehouse_layout/README.md#generated-files) |
| 2026-11-05 | The committed SLAM map will come from a scripted survey route; a teleop map is a learning-report exercise | A scripted route is repeatable, so the map can be rebuilt after a layout change | adopted (the map of 2026-11-07) | [maps/README.md](../src/warehouse_navigation/maps/README.md) |
| 2026-11-05 | One frame for everything: `layout.yaml` is written in the map frame, which is the Gazebo world frame, with its origin at robot1's spawn pose facing east; a fit rule keeps robot1 at (0, 0, 0) | The EKF starts `odom` and slam_toolbox starts `map` where robot1 stands, so the saved map lines up with the world and the layout with no correction step, and the Phase 3 masks can use the layout's coordinates directly. A hall-corner origin would need every saved map shifted by hand | adopted | [ADR-007](decisions/ADR-007-generate-the-warehouse-from-one-layout.md), plan §2.4 |
| 2026-11-05 | Fit rules for the warehouse: the plinth rule of plan §2.6 becomes "everything the lidar must see is solid from the floor to at least 0.40 m, and nothing overhangs below 1.20 m"; pick aisles exactly 1.6 m between the faces; totes flush with the shelf face | The lidar sees only its 0.34 m plane, so the rule must hold for walls, counters, docks and fences too, not only shelves, and the robot's tote and mast must never meet what the lidar missed. The Phase 7 tag rule caps the aisle at about 1.62 m and leaves no room for a setback: 2 cm back, a tag is 39.6 px, under the 40 px limit | adopted | plan §2.6, [warehouse_layout](../src/warehouse_layout/README.md#fit-rules) |
| 2026-11-05 | The generated world: one static model per object, one collision box per solid, visuals for the detail, primitives only | Cheap physics and a simple generator. Measured on a prototype of the world with a lidar probe in headless Gazebo: real-time factor 1.000 with 262 visuals and 85 collisions, 0.999 with 207 totes added (468 visuals); the built world with the robot ran at 1.01 | adopted | [ADR-007](decisions/ADR-007-generate-the-warehouse-from-one-layout.md) |
| 2026-11-06 | The generated warehouse is merged into `main` before mapping starts | The owner reviews the world first; mapping and localization follow in a later pull request | done | [warehouse_layout](../src/warehouse_layout/README.md) |
| 2026-11-06 | ML/DL perception research resumed and finished: use cases, synthetic data from Gazebo, training on Colab, CPU inference and ROS 2 integration, each reviewed against the current robot and layout | The owner asked to resume it; the report recommends one class `tote`, D-FINE-N run on the CPU from C++, and a new `UNLABELED` discrepancy | done | [ML report](research/ML_PERCEPTION.md) |
| 2026-11-06 | Perception design document: the ArUco method (Phases 7-8) and the learned detector (8b) in one design, from the ArUco research, its skeptical review and the ML report, then two more reviews. It proposes 29 changes to the plan (among them: choose the IPPE pose that fits best, never the one nearest the stored dock yaw; a dock tag tilted 20°; M2 re-estimated at about 75-119 h) and asks five questions (O3-O7); the plan is not edited until they are answered | The review found that the plan's rule (pick the IPPE solution nearest the database dock yaw) makes a dock rotated by 5° worse; the owner wants an in-depth reference before Phase 7 | in review | [Perception design](PERCEPTION.md) |
| 2026-11-06 | Measured what this laptop can run (i5-1335U, 15 GiB, Iris Xe): today's stack in real time on about 1 of 12 threads; one robot with Nav2, two side cameras and ArUco at RTF 0.99; three robots with three Nav2 stacks at RTF 0.59-0.68 at the world's 1 ms physics step, about 1.0 at 2 ms or 5 ms. Phases 2-6 comfortable, 7-8b work with care, 9-12 need a larger physics step (to be tested in Phase 3) and Gazebo without its window | The owner asked whether the laptop can carry the project to the final showcase | done | [PC assessment](research/PC_ASSESSMENT.md), plan A4, R3 |
| 2026-11-06 | `cmd_vel_timeout` 0.5 → 0.15 s in `controllers.yaml`. The rule this creates: every `cmd_vel` publisher publishes continuously at 10 Hz or more while the robot should move (Nav2's velocity smoother 20 Hz, its docking server 50 Hz, the tests 20 Hz), or the robot brakes between commands; a command whose stamp is already 0.15 s old on arrival is ignored. The system tests now derive their time windows from the value | The owner's request after driving the warehouse by teleop: each key event drove the robot for 0.5 s, too long for small, accurate moves. Measured in headless Gazebo (keyboard emulated like teleop on this desktop, 3 runs each, real-time factor 1.00): a tap of `j` turns 3.9° instead of 28.6°, a tap of `i` moves 1.4 instead of 15.4 cm; after a held key is released the robot stands still 0.49 s after the last key event instead of 0.84 s (0.15 s timeout plus 0.33 s braking). The cost: teleop sends one command per key event and GNOME repeats a held key only after 500 ms, so a held key now stops for 0.20-0.30 s before it drives on (0.5 s bridged that gap). Removing that stop needs a teleop that sees key releases: the teleop window (next row) | adopted | [controllers.yaml](../src/warehouse_description/config/controllers.yaml), plan R1, [warehouse_bringup](../src/warehouse_bringup/README.md#driving-with-teleop) |
| 2026-11-06 | The teleop window, `ros2 run warehouse_bringup teleop_window`: a Qt window that publishes when a key is pressed, repeats the command at 20 Hz while it is held, sends three zero commands over 0.1 s when it is released and then nothing, and sends one zero command when the window loses the keyboard (a dead-man switch). Speeds start at 0.3 m/s and 0.6 rad/s, so braking takes 0.2 s. The key logic is plain Python, tested with pytest; `teleop_twist_keyboard` stays as the terminal alternative, and `cmd_vel_timeout` stays 0.15 s | The owner, after driving with 0.15 s: "remove the delay completely, I am not feeling it that in control". A terminal never sees a key release, so teleop_twist_keyboard leaves the stop to the timeout and stops a held key until auto-repeat starts. Measured in headless Gazebo (the real window offscreen, synthetic key events, ground truth, 3 runs each, real-time factor 1.00), at 0.5 m/s and 1.0 rad/s: the first zero command arrives under 2 ms after the release; the robot stands still 0.34 s after it instead of 0.49 s, 8.3 cm or 9.7° further on instead of 16.0 cm or 18.2°; a held key never stops (0.2-0.3 s before). At 0.3 m/s and 0.6 rad/s: 0.21 s and 3.1 cm or 3.5°. What remains is braking at the controller's deceleration limits. Cost: a Python node on sim time, 0.4-0.6 cores while open, the same as teleop_twist_keyboard | adopted | [warehouse_bringup](../src/warehouse_bringup/README.md#the-teleop-window) |
| 2026-11-06 | The teleop window needs no arguments: `ros2 run warehouse_bringup teleop_window` starts its node in `/robot1` (`--ros-args -r __ns:=/robotN` still picks another robot) and chooses its clock itself: sim time as soon as `/clock` has a publisher, wall time once the robot has listened 2 s without one (`use_sim_time:=true` still forces sim time). It publishes nothing until the robot listens and the clock is chosen, and a coloured line at its top says so: green when the keys drive, amber while it looks for the robot or chooses the clock, red when nothing listens on the topic it names, with the likely causes and any robot it finds in another namespace | The owner started the simulation, then the window with no arguments: its node came up in the root namespace and published `/cmd_vel` with wall-clock stamps while the robot listens on `/robot1/cmd_vel`, so real key presses did nothing, and the small status text that said so went unnoticed. Verified in headless Gazebo, with the real window offscreen and keys sent through Qt's window path: with no arguments it drove robot1 (54.8 cm for a 2 s hold of the up arrow, standing still 0.21 s after the release, all stamps in sim time); with `hardware:=mock` it chose wall time after 2 s and the robot moved; with `-r __ns:=/robot2` it published only on `/robot2/cmd_vel` and its red line named `/robot1/cmd_vel`; with nothing running it published nothing | adopted | [warehouse_bringup](../src/warehouse_bringup/README.md#the-teleop-window) |
| 2026-11-06 | Map tools in `warehouse_layout`: `rasterize` draws the map an ideal SLAM run would make (the faces the lidar sees occupied, the inside of solids unknown, written like `map_saver_cli`'s trinary maps with `free_thresh` 0.196); `check_map` fits a map to the layout's faces with point-to-plane ICP and checks the mapping limits, with the distances measured after the fit | A number with a limit instead of judging an overlay by eye (plan P2-D13); the same grid is the fallback map of R6 and the grid of the Phase 3 masks. With `free_thresh` 0.25 the unknown pixels (205) would load back as free floor. Measuring after the fit makes a misplaced map and a blurred map fail different lines | adopted | [warehouse_layout](../src/warehouse_layout/README.md#map-tools) |
| 2026-11-06 | `localization:=none\|slam\|amcl` (default `none`) and `map:=`: the map server in the system scope (one `/map` for every robot), slam_toolbox and AMCL in the robot scope. slam_toolbox is configured and activated by launch events, without a lifecycle manager or bond; `map_server` and AMCL each have a Nav2 lifecycle manager with a bond. Every parameter file uses `/**/` keys, scan topics are relative and every map topic is the absolute `/map`; slam_toolbox's whole 2.8.5 sample is written out, without `min_laser_range` | Plan P2-D8, P2-D9 and D20: Phase 9 adds robots without rewiring. A long loop-closure optimization could miss a bond heartbeat, and a manager would then shut the mapping down. A plain `slam_toolbox:` key matches nothing in `/robot1`; AMCL's default `map` topic resolves to `/robot1/map`, where nothing publishes; a key left out of slam_toolbox's file takes the node's built-in default, which differs from the sample for several keys; slam_toolbox 2.8.5 never reads `min_laser_range` | adopted | [warehouse_navigation](../src/warehouse_navigation/README.md#mapping-and-localization-phase-2) |
| 2026-11-06 | Test tools in `warehouse_system_tests`: `drive_route` drives a registry route steering on Gazebo's ground truth (20 Hz commands stamped in sim time), `evaluate_localization` measures AMCL at each `amcl_pose` stamp against interpolated ground truth and the IMU noise from still periods, `map_timelapse` writes the GIF with Pillow | Repeatable mapping and evaluation drives (plan P2-D11); ground truth stays in test tools, never in the robot stack. `amcl_pose` carries the scan's stamp, while AMCL dates its `map -> odom` transform 1 s into the future (P2-D12) | adopted | [warehouse_system_tests](../src/warehouse_system_tests/README.md) |
| 2026-11-07 | No automated tests, lint checks or CI, and every commit goes straight to `main`, without branches or pull requests. Removed: the CI workflow, Dependabot, the pull request template, the pre-commit and lint configuration (`.pre-commit-config.yaml`, `.flake8`, `.yamllint`), `colcon.meta`, and every test with its build settings (the `test/` folders, the `if(BUILD_TESTING)` blocks, every `test_depend`, the test settings of `warehouse_layout/setup.py`, the helpers only the launch tests used); the dev container drops pre-commit and the lint tools. Kept: the robot, every tool (`warehouse_system_tests` keeps its name and its three tools), `.editorconfig` and `.clang-format`. A change is checked by building it and running the system; the owner removes the pull-request rule of the "Protect main" ruleset on GitHub | The owner's decision: stop linting and testing, remove the testing files, and keep every commit on `main`. Paused tests still had to follow every change, and branches and pull requests added steps with nothing to gate. The cost: a regression shows up only when the system is run, and numbers shared between files are kept in step by hand | adopted | [ADR-008](decisions/ADR-008-develop-on-main-without-automated-tests.md), [CONTRIBUTING](../CONTRIBUTING.md#build-and-run) |
| 2026-11-07 | Mapping settings: slam_toolbox's `fine_search_angle_offset` 0.00087 rad (0.05°) instead of the sample's 0.2°; the survey driven at 0.4 m/s with turns at 0.2 rad/s; nothing else running on the laptop and no ROS command-line tool started during the drive. The committed map is run 1 of 3: `check_map` 0.043 m, 0.015°, P95 0.055 m, no ghost walls; run 2 missed the offset limit by 4 mm (0.054 m), run 3 passed (0.047 m) | The setting is the fine scan match's angle step, so 0.2° steps left the map turned by 0.25°. The EKF lags ground truth by about 19 ms, which misplaces scans taken while turning (0.54° at 0.5 rad/s). A second Gazebo stalled the wheel odometry for 0.08-0.35 s, and starting a command-line tool paused it for 0.07-0.28 s. The cause of a 4-5 cm south-west offset in every map is not found yet | adopted | [maps/README.md](../src/warehouse_navigation/maps/README.md) |
| 2026-11-07 | `drive_route` sends its first command only once `diff_drive_controller`'s subscription is listed and matched and its publisher is 3 s old; the initial pose for AMCL waits the same way and is confirmed by AMCL's answer on `amcl_pose` | DDS matches the two ends of a connection separately: a command that reaches the controller before its end knows the publisher is resent later, by then too old for `cmd_vel_timeout`, and ignored (the first four commands of one run, 0.33-0.89 s late). With the wait, 6 of 6 starts were clean | adopted | [warehouse_system_tests](../src/warehouse_system_tests/README.md#drive_route) |
| 2026-11-08 | AMCL tuned on the evaluation: `max_beams` 60 → 240 and `alpha1`-`alpha4` 0.2 → 0.02, chosen one parameter at a time by replaying recorded drives through AMCL (the replay reproduced the live run's 464 poses exactly) | With Nav2's sample values the particle cloud stayed 0.15-0.45 m wide and the error wandered up to 0.26 m: the simulated odometry drifts only about 0.03 % of the distance driven. Two live survey drives with the tuned values passed every check (converged after 0.78 and 1.22 m, then within 0.092 and 0.090 m for 7 minutes). A real robot needs larger values, from its measured odometry error | adopted | [warehouse_navigation](../src/warehouse_navigation/README.md#amcl-what-changed-and-why) |
| 2026-11-08 | Long simulation runs on mains power with the lid open, under `systemd-inhibit --what=idle:sleep`; a run with a suspend in it does not count | The laptop suspends after 15 minutes idle on battery and when the lid closes; a suspend froze the first AMCL acceptance drive after 288 s | adopted | [Phase 2 report](phase-reports/phase-02-warehouse-world.md) |
| 2026-11-08 | Phase 2 done. Its acceptance is reworded for local runs (plan §9 and ADR-008: `generate --check` and `preview` replace the tests; the map, AMCL and IMU limits are unchanged) and met. [ADR-007](decisions/ADR-007-generate-the-warehouse-from-one-layout.md) records the layout decision; the learning report and the mapping GIF (0.18 MB, made by `map_timelapse` from the mapping bag) are written | Every criterion is measured with the tools; the report teaches the phase with exercises run against the real system | done | [Phase 2 report](phase-reports/phase-02-warehouse-world.md) |
| 2026-11-12 | Phase 3 in two parts, each closed by its own acceptance runs, learning report, ADR, GIF and tag: **Phase 3a, Nav2 navigation** (stages S0-S5, about 42 h; ADR-009, tag `phase-3a`), then **Phase 3b, warehouse rules, safety and robot revision C** (stages S6a-S11, about 31.5 h; ADR-010, tag `phase-3b`). The written acceptance is divided between them: the goal runs without zones, the command age and the physics step in 3a; zones, speed caps, safety fields and the goal runs with zones in 3b | The owner's answer to question Q6 of the Phase 3 plan (option b). Two finished, tagged parts of about 40 h each instead of one of about 75 h, and each report is written while its work is fresh; the split costs about 4.5 h | adopted | [Plan](PROJECT_PLAN.md), Phase 3 |
| 2026-11-12 | Robot revision C: two low corner lidars at 0.17 m in a groove round the body, so the robot sees obstacles from 0.17 m up on every side. The tote on the tray, its self-filter (`laser_filters`), the loaded acceptance checks and the loaded AMCL margin move to Phase 4; Phase 3 installs no new package | Q1 (C). The puck's scan plane at 0.34 m passes over low obstacles; the corner pair covers every side, as diagonal safety scanners do on industrial AMRs. The tote waits for Phase 4, which carries totes, and nothing needs sudo now | planned (Phase 3b) | [Plan](PROJECT_PLAN.md) §2.6 (Rev C) |
| 2026-11-12 | Teleop and Nav2 share the robot through `twist_mux`, in front of the collision monitor: teleop (priority 100) overrides Nav2 (priority 10), and both pass the same safety fields. While navigation runs, the teleop window drives in a manual mode capped at 0.3 m/s, and it never reverses faster than 0.3 m/s, the base's limit | Q2 (b). Commercial AMRs are driven by hand in a reduced-speed mode with their safety fields active; 0.3 m/s is the lowest zone cap, so manual driving keeps every zone's limit. The price: while Nav2's collision monitor is inactive, teleop does nothing (an unprotected bypass will be documented) | planned (Phase 3a) | [Plan](PROJECT_PLAN.md) §2.6 |
| 2026-11-12 | Keep-out zones as two masks that nest: for the planner, a lethal core 0.65 m round each zone with a 0.30 m cost ramp beyond it; for the controller, each zone grown by 0.10 m; the dock-bar boxes in both. The six filter servers run once, in the system scope, under their own lifecycle manager, apart from the map's; this amends plan D20 and D22 | Q3 (a). Paths keep off the core, so tracking error does not cost goals, while the controller's footprint check still stops the robot at a zone; a failing filter server must not reset the map that AMCL uses | planned (Phase 3b) | [Plan](PROJECT_PLAN.md) D20, D22 |
| 2026-11-12 | No checker tool for Phase 3's configuration: each stage reads the running system back by hand (`ros2 param get`, `ros2 topic info -v`, `ros2 lifecycle get`, `ros2 topic echo`), from a list the `warehouse_navigation` README will keep. `masks --report` stays, and the safety-field derivation is a printed table. On 2026-11-13 the owner accepted the sha256 of the mask cells that `masks --report` prints as the by-hand check of the masks | Q4 (c), and ADR-008: a change is checked by running the system. The price: a costmap left on silent defaults is caught only if the stage runs its checks | adopted | [ADR-008](decisions/ADR-008-develop-on-main-without-automated-tests.md) |
| 2026-11-12 | Phase 3 settles the physics step for Phase 9: Phase 3a measures 1, 2 and 5 ms steps and adopts the largest that passes; every comparison in Phase 3 uses three drives per condition | Q5 (a). Three robots with Nav2 run at a real-time factor of 0.59-0.68 at today's 1 ms step; one drive's largest AMCL error varies by about 37 mm, so one run per condition decides nothing | planned (Phase 3a) | [PC assessment](research/PC_ASSESSMENT.md), plan R3 |
| 2026-11-12 | The safety fields are sized for a reaction time of 0.25 s instead of the 0.2 s of plan §2.6 | The collision monitor waits 10-40 ms per command for the odometry transform, which 0.2 s left out; the fields grow by 1-5 cm, so they become more conservative | planned (Phase 3b) | [Plan](PROJECT_PLAN.md) §2.6 |
| 2026-11-13 | Tags `phase-1` on 14cb789 (the merge of pull request #2, 2026-11-05) and `phase-2` on 7da18a4 (Phase 2's documents, 2026-11-08), annotated like `phase-0` (on 73820c9) | CONTRIBUTING.md: a `phase-N` marker tag per finished phase. The owner chose the merge commit for Phase 1; the alternative was 2e487b6, the status commit that marked it merged | done | [CONTRIBUTING.md](../CONTRIBUTING.md) |
| 2026-11-13 | Phase 3a starts with stage S0, the Phase 2 leftovers: six wording fixes from the Phase 2 review (why a bag missed `tf_static`, the cell-centre share of the map offset, how often launch's shutdown error appears, the clock of the start-up times, the EKF's missed update when a tool joins, `PERCEPTION.md`'s status); `drive_route` keeps only its newest command (depth 1), names a subscription whose QoS can never match its publisher, acts on Ctrl-C only between its callbacks (a Ctrl-C inside rclpy could end it with a traceback and no stop command) and ignores a second Ctrl-C while it stops the robot; the plan's header and ADR lists brought up to date | The Phase 2 review's nine low findings, closed before the Nav2 work begins | done | [warehouse_system_tests](../src/warehouse_system_tests/README.md#drive_route) |
| 2026-11-13 | Stage S1, measuring the laptop. `measure_load`: the PC assessment's sampler as a tool. It is not a ROS node, because a node on sim time pays for every `/clock` message, and it reads Gazebo's `/world/<world>/stats` through the gz-transport Python bindings. It groups processes by command line and counts only those with its own `ROS_DOMAIN_ID` and `GZ_PARTITION`; it takes the machine total from `/proc/schedstat`; and it flags a suspend or MemAvailable below 1.5 GiB. `gz_gui:=false` runs Gazebo without its window while RViz stays on. `physics_step:=0.001\|0.002\|0.005` writes a copy of the world into the launch's log directory, and any other value stops the launch before Gazebo starts; the world files keep 1 ms. `evaluate_localization --odometry` brings back Phase 1's odometry checks as a measurement, judging turns in place from 80° rather than the planned 90° | The physics-step comparison of stage S4 needs RTF, CPU and odometry from the same runs (Q5 (a)), and the Gazebo window costs real time (PC assessment recommendations 1, 2 and 9). The step must divide the 10 ms period of the controller manager and the IMU. `drive_route`'s 90° corners turn 85.8° in place, because it stops a turn a few degrees early and turns the rest while driving, so a 90° threshold would judge none of them. Measured headless, robot standing: RTF 1.00 and steps of exactly 1, 2 and 5 ms. Gazebo used 1.57, 1.18 and 0.78 cores, more than the PC assessment's 0.70 at 1 ms. The power profile explains it: balanced here, performance then. A repeat at 1 ms under the performance profile cost Gazebo 0.90 cores (1.30 for the whole run), at 3.6 GHz instead of 2.5. Runs are compared only under one profile. The owner accepted the 80° turn threshold | done | [warehouse_system_tests](../src/warehouse_system_tests/README.md#measure_load), [warehouse_gazebo](../src/warehouse_gazebo/README.md#the-physics-step-physics_step) |

## Planned features

Short list per milestone; details and acceptance criteria are in the plan. All **planned**.

- **M1 Core AMR (`v1.0.0`)**
  - Phase 1's demo GIF: teleop with the RViz sensor view, recorded by the maintainer (steps in
    [exercise 7 of the Phase 1 report](phase-reports/phase-01-robot-description.md#exercise-7-run-the-demo-with-windows-and-record-the-gif)).
  - Nav2 navigation (Phase 3a); keep-out and speed zones, a collision monitor and robot
    revision C (Phase 3b).
  - Custom interfaces and a behaviour-tree task executor (Phase 4a).
  - Order manager, order generator and KPI tracking (Phase 4b).
  - Operator dashboard v1, our own Flask web app: overview, orders, live map, live updates
    (Phase 4c).
  - Simulated battery, autonomous docking and recharging, diagnostics (Phase 5).
  - Hardening and the `v1.0.0` release with a recorded demo (Phase 6).
- **M2 Perception (`v2.0.0`)**
  - ArUco tags on totes and docks, detected by robot cameras (Phase 7).
  - Inventory patrol with discrepancy reports (Phase 8).
  - Optional: learned detector trained on Google Colab, deployed as an ONNX model (Phase 8b).
- **M3 Fleet (`v3.0.0`)**
  - Three namespaced robots in one simulation (Phase 9).
  - Fleet manager with greedy, then auction-based task allocation (Phases 10-11).
  - Aisle traffic control (Phase 11).
  - Dashboard polish and the final showcase scenario (Phase 12).
- **Stretch**
  - Mobile manipulation: navigate to a shelf, then pick with an arm (Phase 13).
- **After all phases**
  - Deployment of the system and dashboard on the LAN.

## How this document is maintained

- **Every commit that adds or changes a feature updates this file, in the same commit:** add or
  update the feature's entry under
  [Features](#features) (what, does, how to use, where), add a dated row to the
  [log](#ideas-and-decisions-log) for any new idea or decision, and move items out of
  [Planned features](#planned-features) once they exist.
- **Every finished phase adds a learning report** at
  `docs/phase-reports/phase-NN-<name>.md` (for example `phase-01-robot-description.md`), updates the
  [Progress](#progress) table, and gets its `phase-N` tag.
- A decision that is hard to reverse also gets an ADR in [decisions/](decisions/); the log
  row links to it.
- Keep entries short and factual. Only list features that exist on `main`.
