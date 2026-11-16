# warehouse_navigation

Localization and navigation of the warehouse AMR. Phase 1 adds the odometry EKF; Phase 2 makes
a map of the warehouse with slam_toolbox and localizes on it with AMCL; Phase 3 adds Nav2, which
drives the robot to a goal pose ([below](#navigation-with-nav2-phase-3)); docking follows in
Phase 5.

## Files

| File | What it does |
|---|---|
| `config/ekf.yaml` | `robot_localization` EKF parameters (key `/**/ekf_filter_node`, so it works in any namespace) |
| `launch/ekf.launch.py` | Starts `ekf_node` as `ekf_filter_node` and remaps its output `odometry/filtered` to `odom`. Include it inside the robot's namespace together with the `/tf` remaps, as `warehouse_bringup/launch/robot.launch.py` does |
| `config/slam_toolbox.yaml` | slam_toolbox's parameters: its own sample with our changes marked ([below](#slam_toolbox-what-changed-and-why)) |
| `config/amcl.yaml` | AMCL's parameters, and those of its lifecycle manager `lifecycle_manager_localization` |
| `config/map_server.yaml` | `map_server`'s parameters, and those of its lifecycle manager `lifecycle_manager_map` |
| `launch/slam.launch.py` | Robot scope: slam_toolbox (`async_slam_toolbox_node`), configured and activated by launch events. Include it in the robot's namespace after the `/tf` remaps, like the EKF (`robot.launch.py localization:=slam` does) |
| `launch/localization.launch.py` | Robot scope: `amcl` and `lifecycle_manager_localization`. `x`, `y`, `yaw`: AMCL's initial pose in the map frame (`robot.launch.py localization:=amcl` passes the spawn pose). `container`: empty (the default) starts both as nodes of their own; the full name of a running component container loads both into it (`navigation.launch.py` does) |
| `launch/map_server.launch.py` | System scope: `map_server` and `lifecycle_manager_map` in `/`, serving the `map:=` file on `/map` (default `maps/warehouse.yaml`). Refuses a map file that does not exist before it starts anything |
| `maps/` | The warehouse map AMCL uses, with a README that says how it was made ([maps/README.md](maps/README.md)) |
| `config/nav2_params.yaml` | Nav2's parameters: the ten servers that `nav2_bringup`'s `navigation_launch.py` starts, and both costmaps. In Nav2's own style (plain node keys such as `controller_server:`, and `<robot_namespace>`), with the reason for every changed value next to it |
| `behavior_trees/` | `navigate_to_pose.xml` and `navigate_through_poses.xml`: Nav2's two default behaviour trees without the recovery that drives the robot backwards |
| `launch/navigation.launch.py` | Robot scope: Nav2 and AMCL in one process, `nav2_container`. Processes `params_file` (default `config/nav2_params.yaml`), starts the container with it, loads AMCL through `localization.launch.py` and the Nav2 servers through `navigation_launch.py`. `namespace`, `use_sim_time`, `x`, `y`, `yaw` as for AMCL. `robot.launch.py navigation:=true` includes it |

## The EKF (plan D3)

| | |
|---|---|
| Inputs | `odom/wheel` (from `diff_drive_controller`): forward speed vx, sideways speed vy and yaw rate. `imu` (from `imu_sensor_broadcaster`): yaw rate |
| Why vy | A diff drive cannot slide sideways, so the controller always reports vy = 0, with a small variance. That zero is a real measurement. Without it nothing observes vy: standing still in Gazebo, the y variance of the output reached about 4e4 m² after 30 s and 7e5 m² after a minute (with it: 2-3 m², like x). Such a covariance is useless to RViz and to later fusion (AMCL, Phase 2). robot_localization's configuration guide recommends fusing it |
| Not fused | The IMU orientation: Gazebo reports the true orientation without noise, which would make localization look better than a real robot could manage. Accelerations: not needed for a planar robot at warehouse speeds |
| Mode | `two_d_mode: true`: z, roll and pitch stay zero |
| Outputs | `odom` (`nav_msgs/Odometry`, 50 Hz) and the transform `odom -> base_footprint` on the robot's `tf`. Nothing else publishes this transform: the controller runs with `enable_odom_tf: false` (plan R5) |

The filter weighs its inputs by their covariances: wheel yaw rate variance 0.01 (from
`controllers.yaml`) against IMU yaw rate variance 4e-6 (σ² of the simulated gyro), so the IMU
dominates the heading while the wheels give the speed.

Measured in Gazebo (Phase 1 acceptance test, 15 runs): over 10.4 m straight the EKF's end point
is within 0.1-0.4 % of ground truth, and over a 370° turn in place its heading is within 0.05 %.
The wheel odometry alone does better in simulation (0.01 % and 0.00 %). The EKF integrates the
controller's reported twist, and in Gazebo that twist reads about 0.2 % high (1.002 m/s at a
true 1.000 m/s): the controller manager's update stamps jitter between 9 and 11 ms, and the
controller computes speed as position change / stamp interval. Each update covers 10 ms of
physics, so a 9 ms interval reads 11 % high and an 11 ms one only 9 % low: the jitter pushes
the average up. The controller's own pose integrates the wheel positions and is unaffected.
Removing the IMU does not change the error. On mock hardware the EKF ends close to the wheel
odometry once the robot stops: 0.01-15 mm apart after eight short drives of 0.3-0.7 m.

**Mock hardware caveat.** With `hardware:=mock` the IMU reports its initial values (zero yaw
rate), and the EKF trusts the IMU's yaw rate far more than the wheels'. In mock mode the EKF
follows straight motion but not turns: mock hardware tests wiring, not localization.

The EKF starts with every robot (`ros2 launch warehouse_bringup sim.launch.py`). To watch it:

```bash
ros2 topic echo /robot1/odom --field pose.pose
ros2 topic echo /diagnostics     # the EKF's status; diagnostic topics are global (plan §2.3)
```

## Mapping and localization (Phase 2)

The EKF knows how far the robot has driven since it started, and that estimate drifts. Mapping
and localization tie the robot to the warehouse itself:

- **SLAM** (simultaneous localization and mapping, slam_toolbox) builds the map while it works
  out where the robot is. It keeps a graph: each node is a robot pose with the scan taken there,
  each edge says how two poses relate (from odometry, or from matching their scans). When the
  robot comes back to a place it has seen, a *loop closure* adds an edge between the old and the
  new pose, and the Ceres optimizer moves every pose so that all edges fit as well as possible.
  The map is drawn from the corrected poses and their scans.
- **Localization** (AMCL, adaptive Monte Carlo localization) uses a finished map and does not
  change it. It keeps 500 to 2000 guesses of the pose (particles), moves them with the odometry
  plus some noise, and weighs each by how well the scan, seen from it, lines up with the map's
  walls. Unlikely guesses die out, so the cloud tightens around the true pose.

Pick one with `localization:=none|slam|amcl` in `warehouse_bringup`'s `sim.launch.py` or
`robot.launch.py`. Never both: each publishes `map -> odom`.

### Frames

```text
map --(slam_toolbox or AMCL)--> odom --(EKF)--> base_footprint --(robot_state_publisher)--> ...
```

`odom -> base_footprint` is smooth but drifts. `map -> odom` is the correction: slam_toolbox or
AMCL computes where the robot is on the map, and publishes the transform that puts the drifting
`odom` frame in the right place. All of it goes to the robot's own `/robot1/tf`.

The map frame is the Gazebo world frame, with its origin at robot1's spawn pose (plan P2-D2).
slam_toolbox starts its map where the robot starts, so a map made with robot1 lines up with
the world and with the layout, with no correction step. Make maps with robot1.

### Scopes and topics

| Scope | Nodes | When |
|---|---|---|
| System (`/`) | `map_server`, `lifecycle_manager_map` | `localization:=amcl` or `navigation:=true`: one map for every robot (plan D20) |
| Robot (`/robot1`) | `slam_toolbox` | `localization:=slam` |
| Robot (`/robot1`) | `amcl`, `lifecycle_manager_localization` | `localization:=amcl`: one AMCL per robot. With `navigation:=true` the same two run inside Nav2's container ([below](#navigation-with-nav2-phase-3)) |

| Topic | From -> to | QoS |
|---|---|---|
| `/robot1/scan` | Gazebo bridge -> slam_toolbox, AMCL, RViz | best effort (sensor data) |
| `/map` (and `/map_metadata` from slam_toolbox) | slam_toolbox or map_server -> AMCL, RViz, `map_saver_cli` | reliable, transient local, depth 1: a subscriber that starts later still gets the map, once |
| `/robot1/tf` | `map -> odom` from slam_toolbox or AMCL | as the EKF's |
| `/robot1/amcl_pose` | AMCL -> RViz, bags | reliable, transient local; published after each filter update |
| `/robot1/particle_cloud` | AMCL -> RViz | best effort |
| `/robot1/initialpose` | RViz's "2D Pose Estimate" -> AMCL | AMCL listens best effort and volatile. Send a pose once AMCL is active: before that it is lost, or replaced by the launch's initial pose at activation |

The scan and `initialpose` are relative names in the parameter files, so they resolve in the
robot's namespace. The map is absolute (`/map`) everywhere: AMCL's default `map` would resolve to
`/robot1/map`, where nothing publishes, and AMCL would wait for a map forever.

### Lifecycle: who starts what

All three are lifecycle nodes: they start *unconfigured* and do nothing until someone asks them
to *configure* (read parameters, create publishers) and then to *activate* (start working).

- **slam_toolbox** is started by launch events in `slam.launch.py`, the way slam_toolbox's own
  `online_async_launch.py` does it: a configure request at start-up, and an event handler that
  sends the activate request once the node reports `inactive`. No lifecycle manager and no bond
  (plan P2-D8): a long loop-closure optimization could delay a heartbeat, and a manager would
  then shut the mapping down.
- **map_server** and **AMCL** each have a Nav2 lifecycle manager (`autostart: true`) that
  configures and activates them, then watches them through a *bond*: both sides send
  heartbeats, and if one stops for `bond_timeout` (4 s), the manager brings the nodes down and
  waits up to 10 s for them to come back. The manager reaches its nodes by relative names, and
  the bond topic is relative too (`/robot1/bond`), so each manager stays within its scope.

Measured without Gazebo (wall time, 2026-11-06): slam_toolbox active 0.7 s after the launch
started, `map_server` 0.6 s, AMCL 0.8 s. In Gazebo (2026-11-07 and 2026-11-08; wall clock,
polled with `ros2 lifecycle get`, so the tool's own start-up is included): slam_toolbox
1.5-1.7 s, `map_server` 1.5-1.7 s, AMCL 2.4-4.9 s.

### slam_toolbox: what changed and why

`config/slam_toolbox.yaml` is slam_toolbox 2.8.5's `mapper_params_online_async.yaml` with these
changes. The whole sample is written out, because a key left out does not fall back to the
sample's value but to the value built into the node, and several of those differ (for example
`minimum_distance_penalty`: 0.05 built in, 0.5 in the sample).

| Key | Sample | Ours | Why |
|---|---|---|---|
| `scan_topic` | `/scan` | `scan` | relative: `/robot1/scan` |
| `map_name` | (not listed) | `/map` | the built-in value, written out: the one shared map |
| `use_lifecycle_manager` | (not listed) | `false` | the built-in value, written out: launch events, no bond |
| `use_map_saver` | `true` | `false` | maps are saved with `map_saver_cli` |
| `map_update_interval` | 5.0 s | 2.0 s | the map grows smoothly in RViz and in the time-lapse |
| `enable_interactive_mode` | `true` | `false` | no interactive markers for editing the graph |
| `minimum_travel_distance`, `minimum_travel_heading` | 0.5 m, 0.5 rad | 0.3 m, 0.3 rad | more graph nodes in the 4-5 m aisles |
| `loop_search_maximum_distance` | 3.0 m | 2.0 m | 3.0 m reaches into the next aisle, 2.7 m away: a false loop closure would merge two aisles (plan risk R2-1) |
| `loop_match_minimum_response_fine` | 0.45 | 0.5 | fewer false loop closures between look-alike aisles |
| `fine_search_angle_offset` | 0.00349 rad (0.2°) | 0.00087 rad (0.05°) | despite the name, the angle step of the fine scan match, which searches ±1° (half of `coarse_angle_resolution`) around the coarse match. With 0.2° steps it cannot correct a heading error under about 0.1° (half a step), and the map took on the EKF's heading drift ([maps/README.md](maps/README.md)) |
| `min_laser_range` | 0.0 | (removed) | slam_toolbox 2.8.5 never reads it; the shortest range comes from each scan's `range_min` (0.10 m from `robot.yaml`) |

`max_laser_range` stays 20.0 m, the lidar's range in `robot.yaml`. `mode: mapping` stays too:
the Ceres plugin reads it.

### AMCL: what changed and why

`config/amcl.yaml` lists only some keys; every other one keeps AMCL's built-in default, which
for AMCL equals Nav2's sample (`nav2_bringup/params/nav2_params.yaml`). Changed from the sample:

| Key | Sample | Ours | Why |
|---|---|---|---|
| `map_topic` | (not listed; built in: `map`) | `/map` | the one shared map (above) |
| `laser_max_range` | 100.0 m | -1 | use the scan's own 20 m from `robot.yaml`, as `laser_min_range: -1` already does; 100 m belongs to another robot's lidar |
| `set_initial_pose` | (not listed; built in: `false`) | `true` | start at the spawn pose from the launch, where the robot stands |
| `max_beams` | 60 | 240 | weigh every third of the 720 rays. With 60, too few beams hit rack ends and end walls to fix the robot's place along an aisle |
| `alpha1`-`alpha4` | 0.2 | 0.02 | the odometry noise. 0.2 suits wheels that slip: each 0.25 m update scattered the particles by about 0.11 m and 6°, while the EKF here drifts about 0.03 % of the distance. The cloud stayed 0.15-0.45 m wide and the estimate wandered up to 0.26 m off. Real robots need larger values |

Both tuned values come from the evaluation (plan F7), one parameter at a time: `max_beams` first,
then the odometry noise. To compare settings on identical input, the recorded scans and
odometry of a drive were played back through AMCL (`ros2 bag play --clock`) with each setting.
AMCL's random numbers repeat from start to start: replayed with the live run's settings, it
published exactly the live run's poses (464 of 464 identical).

| Setting (240 beams unless noted) | Largest error after convergence |
|---|---|
| Nav2's 60 beams, alpha 0.2 | 0.26 m; 141 poses over 0.10 m, a third of them |
| alpha 0.2 | 0.19 m |
| alpha 0.05 | 0.09-0.10 m; over 0.10 m in one of the four drives |
| alpha 0.02 | 0.06-0.09 m |

These replays used the first acceptance drive (to 106 m) and the three survey drives of the
mapping runs ([maps/README.md](maps/README.md)), each started 0.3 m and 8° off. Then, in Gazebo
with the tuned values, two survey drives at 0.5 m/s, started 0.3 m and 8° off, converged after
0.78 m and 1.22 m and stayed within 0.092 m and 0.090 m of ground truth for the next 7 minutes
(P95 0.052 m and 0.057 m). The largest errors lie along the direction of travel, in long aisles
and after the top-right corner, and recur at the same places from drive to drive.

### The map, and how to make it again

`maps/warehouse.{yaml,pgm}` is the map AMCL uses by default: 611 x 403 cells of 0.05 m, made by
slam_toolbox on the scripted survey route on 2026-11-07 (run 1 of 3). `check_map` puts it 0.043 m
and 0.015° from the layout (limits 0.05 m and 0.2°), with every occupied cell within 0.10 m of a
layout face and no ghost walls. [maps/README.md](maps/README.md) has the exact commands to make it
again (after any change to `layout.yaml` that the lidar can see), the numbers of every run, and
what the runs taught: turn slowly, map alone on the laptop, start no ROS tools while driving.

The fallback (plan risk R6) is the map `rasterize` draws from the layout:
`ros2 run warehouse_layout rasterize --out /tmp/layout_map`, then
`localization:=amcl map:=/tmp/layout_map.yaml`. Measured on the smoke route (2026-11-08, started
0.42 m and 8° off): on the rasterized map AMCL converged after 1.62 m, then stayed within 0.096 m
(mean 0.041 m); on the SLAM map, after 1.21 m and within 0.076 m (mean 0.019 m). It works, but
the SLAM map is the better one.

### Mistakes that do not fail at start-up

ROS 2 ignores a parameter that matches no node or no declared name, without a warning, and a
relative topic name resolves wherever the node runs. Such mistakes do not fail at start-up; the
robot just never localizes. Nothing checks the files automatically
([ADR-008](../../docs/decisions/ADR-008-develop-on-main-without-automated-tests.md)), so after
editing one, check by hand: every key starts with `/**/` (a plain `amcl:` matches only a node in
`/`); the scan topics are relative and every map topic is `/map`; the frames equal the EKF's;
the laser ranges agree with `robot.yaml`; slam_toolbox's keys are its sample's keys (no
misspelling) with the sample's types (`20` where the node expects `20.0` stops the node); the
lifecycle managers manage the right node; AMCL takes its initial pose from the launch; nothing
names ground truth. Then run it and read the values back from the running node with
`ros2 param get`.

## Navigation with Nav2 (Phase 3)

`navigation:=true` in `warehouse_bringup`'s `sim.launch.py` or `robot.launch.py` adds Nav2 to
the robot: give it a goal pose on the map, and it plans a path, follows it and stops there. Stage
S2a of Phase 3 built this, and S2b added RViz's navigation view and the checks by hand
([below](#checking-nav2-by-hand)). Later stages add a command line for named locations (S3a),
twist_mux for teleop (S5), keep-out and speed zones (S6b) and the low corner lidars (S7).

### What runs

One process per robot, `/robot1/nav2_container` (a `component_container_isolated`), holds AMCL
and the ten servers that `nav2_bringup`'s `navigation_launch.py` always starts. Each is a
*component*: a node built as a library and loaded into a running process. In this container each
component gets its own executor thread, so a slow planner cannot hold up another server's bond
heartbeat (ROS 2 skill, `lifecycle-components.md` §6). One process for the thirteen nodes (AMCL,
the ten servers, two lifecycle managers) also means one DDS participant instead of thirteen.

| Server | What it does here |
|---|---|
| `bt_navigator` | Runs a behaviour tree per goal: plan, follow, recover ([below](#the-behaviour-trees)). Serves the actions `navigate_to_pose` and `navigate_through_poses` |
| `planner_server` | Plans the path with Smac 2D on the global costmap: the map, the scan and inflation |
| `controller_server` | Follows the path on the local costmap at 20 Hz: the Rotation Shim turns the robot in place until it faces the path, then Regulated Pure Pursuit drives |
| `behavior_server` | The recoveries the trees call: a turn in place (`spin`) and a wait |
| `velocity_smoother` | Holds the commands to the operating limits: 1.0 m/s, 1.0 rad/s, +0.5 and -1.0 m/s² |
| `collision_monitor` | The last check before the base: stops the robot when the scan has points inside the field for the commanded speed ([below](#the-safety-fields)) |
| `smoother_server`, `waypoint_follower`, `route_server`, `docking_server` | Started by `navigation_launch.py` whether used or not, and configured so that they start. Docking is Phase 5's |

`lifecycle_manager_navigation` configures and activates the ten servers, one after the other;
`lifecycle_manager_localization` does the same for AMCL. Both then watch their nodes through
bonds. If one stops answering for 4 s of wall time (a stalled process, a laptop suspend), its
manager resets every node it manages and starts them again once they answer; a goal in progress
fails. The map server stays in the system scope, once for all robots.

### From a goal to the wheels

```text
goal ─> bt_navigator ─> planner_server: the path
controller_server (Shim + RPP, 20 Hz) ──┐
behavior_server (spin, wait, 20 Hz) ────┴─ cmd_vel_nav ─> velocity_smoother ─ cmd_vel_smoothed ─> collision_monitor ─ cmd_vel ─> diff_drive_controller
docking_server (Phase 5): cmd_vel directly, past the monitor (plan D21)
```

- Each of these topics carries `geometry_msgs/msg/TwistStamped` and nothing else.
  `diff_drive_controller` accepts only stamped commands, and Nav2 1.3.13 publishes unstamped
  `Twist` unless `enable_stamped_cmd_vel` is true, so the five servers that publish or subscribe
  commands set it (controller, velocity smoother, collision monitor, docking and behavior server).
- The base brakes when its newest command is 0.15 s old (`cmd_vel_timeout`). So every publisher
  in the chain sends at 20 Hz, and the velocity smoother stamps each output with the time it was
  smoothed for (`stamp_smoothed_velocity_with_smoothing_time`). By default it repeats its last
  input's stamp, and the command the base holds then ages towards the timeout: the robot brakes
  and drives on (Nav2 issue 5857). On the goal [below](#see-it) the command the base held was at
  most 0.071 s old while the robot drove.
- Nav2 never drives backwards: the smoother's lowest speed is 0, Regulated Pure Pursuit may not
  reverse, and the trees have no recovery that backs up.

### How the parameters reach the nodes

`config/nav2_params.yaml` uses plain node keys (`controller_server:`), Nav2's own convention, and
`<robot_namespace>` where a topic must be absolute. `navigation.launch.py` processes it before any
node starts:

1. `<robot_namespace>` becomes `/robot1`. A costmap layer resolves topic names under the costmap
   node (`/robot1/local_costmap/...`), so its scan has to be the absolute `/robot1/scan`.
2. `footprint` (the outline in `warehouse_description`'s `robot.yaml`) and the paths of the two
   trees are written in by their full parameter paths, such as
   `local_costmap.local_costmap.ros__parameters.footprint`. Nobody copies the outline by hand.
3. The result goes to `navigation_launch.py`, which wraps it under `robot1:` and hands each
   server its section. A second copy, wrapped under `robot1:` already, goes on the container's
   own command line, together with the `/tf` remaps.

The second copy is the step that is easy to miss. The two costmaps (made by the controller and
planner servers) and the servers' TF listeners are not loaded by launch: the servers create them
inside the process, and such nodes see only the process's own command line. Without it the costmaps would run on their
built-in defaults instead of this file, and nothing at start-up would say so. So read the values
back from the running nodes, not from the file:

```bash
ros2 param get /robot1/local_costmap/local_costmap footprint          # [[0.32, 0.24], [-0.32, 0.24], [-0.32, -0.24], [0.32, -0.24]]
ros2 param get /robot1/local_costmap/local_costmap footprint_padding  # 0.04; 0.009999999776482582 is the default, 0.01
ros2 topic info /robot1/cmd_vel                                       # Type: geometry_msgs/msg/TwistStamped, no other type
ros2 topic info -v /robot1/tf          # publishers: robot_state_publisher, ekf_filter_node, amcl; nothing on /tf
```

### The behaviour trees

`behavior_trees/` holds copies of Nav2's two default trees (`navigate_to_pose_w_replanning_and_recovery.xml`
and `navigate_through_poses_w_replanning_and_recovery.xml`) with one line removed: the `BackUp`
recovery, which drives the robot 0.30 m backwards. Nav2 never reverses here (reverse only to
undock, plan P3-D12), and once a tote rides on the tray the robot cannot see behind it. They are
copies from the first run on: a tree whose behaviour has no server fails as soon as the
navigator loads it, so the stock trees would need the back-up behaviour loaded, and that would
let Nav2 reverse. When planning or following fails, the recoveries take turns: clear both
costmaps, turn 90° in place, wait 5 s. After 6 failed rounds the goal fails.

### The safety fields

The collision monitor has a stop field for each band of commanded speed (`velocity_fields` in
`config/nav2_params.yaml`): `rotate` (standing or turning in place), `reverse`, `slow` (up to
0.3 m/s), `aisle` (up to 0.5 m/s), `fast` (up to 1.0 m/s) and `catch_all`. Each is the footprint
swept from the moment an obstacle appears until the robot stands, for a reaction time of 0.25 s
and the base's braking at 1.5 m/s², plus 5 cm (plan §3.4; the file's comments give each band's
numbers). The monitor uses the first band that contains the command, so `rotate` comes first and
`catch_all` last. Until stage S7 adds the corner lidars, the puck's `scan` is its only source, and
a scan older than 0.5 s stops the robot. The active field is published on `/robot1/velocity_fields`.

### See it

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse navigation:=true gz_gui:=false
```

In a second terminal set up like the first:

```bash
ros2 lifecycle get   # 14 lines ending in active [3]: map_server, amcl, the ten servers, both costmaps
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 8.2, y: 1.5}, orientation: {w: 1.0}}}}"
ros2 topic echo --once /robot1/ground_truth/odom --field pose.pose.position   # about x 8.2, y 1.5
```

`send_goal` prints `Goal finished with status: SUCCEEDED`; Ctrl-C on it cancels the goal. RViz
opens `nav.rviz`, the navigation view: the map, both costmaps, the path, the footprint and the
active safety field. Its "2D Goal Pose" tool sends a goal too: click on the map and drag to
set the heading
([warehouse_bringup README](../warehouse_bringup/README.md#the-navigation-view-navrviz)).

Measured in headless Gazebo on 2026-11-16, two runs: the 14 lifecycle nodes were active 19.3 s
and 12.0 s after the launch started (wall clock, polled with `ros2 lifecycle get`, so its own
start-up is included). The goal, 8.3 m from the spawn pose, was reached in 9.6 s and 9.7 s; ground
truth then lay 0.070 m and 0.065 m, 0.041 rad and 0.038 rad (2.3° and 2.2°) from it. Nav2 stops
within 0.10 m and 0.10 rad by AMCL's estimate, which was 0.058 m from the truth in the first run.
The first run was recorded: an 8.5 m path at up to 1.00 m/s, never in reverse, commands at
20.0 Hz; the real-time factor stayed at 1.00 and the container used 0.89 cores (`measure_load`).

Until stage S5 puts twist_mux in front of the collision monitor, teleop and Nav2 both publish on
`/robot1/cmd_vel`: cancel a goal before driving by hand.

### Checking Nav2 by hand

Nav2 does not warn about a parameter it does not know, and a section that misses its node
leaves the node on its defaults. Nothing checks the configuration automatically (plan Q4 (c),
[ADR-008](../../docs/decisions/ADR-008-develop-on-main-without-automated-tests.md)), so read the
running system back instead of the file. The checks H1-H7 below do that with standard `ros2`
commands. Run them after changing `config/nav2_params.yaml`, `launch/navigation.launch.py` or
the outline in `warehouse_description`'s `robot.yaml`; each stage of Phase 3 names the checks it
runs. Later stages extend the list: H8, the velocity chain through twist_mux (S5); H9, the zone
filters and their masks (S6b); H10, the safety fields the collision monitor runs (S8a). S7 adds
the corner lidars to H4 and H7.

How to run them:

1. Start Nav2 without RViz:
   `ros2 launch warehouse_bringup sim.launch.py world:=warehouse navigation:=true gz_gui:=false rviz:=false`.
2. Open a second terminal set up like the first (the workspace sourced, the same
   `ROS_DOMAIN_ID`) and wait until H1 shows every node `active [3]`.
3. Leave the robot standing while the checks run: no goal, no teleop. Each `ros2` command joins
   DDS as a new participant, and a joining participant can pause the wheel odometry for a moment
   (Phase 2 report, problem 6). H1-H7 are about 70 commands.
4. Run no recorder and no RViz during the checks: both add subscribers, which H4 counts.
5. Define these two first, in the same terminal:

   ```bash
   P() { n=$1; shift; for p in "$@"; do echo -n "$p: "; ros2 param get "$n" "$p"; done; }
   Q='--qos-durability transient_local --qos-reliability reliable'
   ```

   `P <node> <name>...` prints each parameter's name and its value. `Q` is for the latched
   topics of H9 (stage S6b), used unquoted: `$Q`.

A check passes when its output shows the stated values. `ros2 param get` names the type, for
example `Double values are: [1.0, 0.0, 1.0]`.

**H1. Lifecycle states.** With no node name, `ros2 lifecycle get` lists every lifecycle node,
from one participant:

```bash
ros2 lifecycle get
```

It prints these 14 lines (S6b adds the six filter servers):

```text
/map_server: active [3]
/robot1/amcl: active [3]
/robot1/behavior_server: active [3]
/robot1/bt_navigator: active [3]
/robot1/collision_monitor: active [3]
/robot1/controller_server: active [3]
/robot1/docking_server: active [3]
/robot1/global_costmap/global_costmap: active [3]
/robot1/local_costmap/local_costmap: active [3]
/robot1/planner_server: active [3]
/robot1/route_server: active [3]
/robot1/smoother_server: active [3]
/robot1/velocity_smoother: active [3]
/robot1/waypoint_follower: active [3]
```

Within about 20 s of the launch; until then some say `unconfigured [1]` or `inactive [2]`. A
node that stays there failed to configure or activate: its error is in the launch's output.

**H2. Footprint, padding and frames.**

```bash
for c in local global; do P /robot1/${c}_costmap/${c}_costmap footprint footprint_padding robot_base_frame; done
for n in bt_navigator behavior_server smoother_server; do P /robot1/$n robot_base_frame; done
P /robot1/collision_monitor base_frame_id
for n in docking_server route_server; do P /robot1/$n base_frame; done
```

For each costmap it prints

```text
footprint: String value is: [[0.32, 0.24], [-0.32, 0.24], [-0.32, -0.24], [0.32, -0.24]]
footprint_padding: Double value is: 0.04
robot_base_frame: String value is: base_footprint
```

and then `base_footprint` on each of the other six lines. The footprint is `robot.yaml`'s
outline, which the launch writes in. Wrong values, and what they mean:

- `footprint: String value is: []`: Nav2's default. The launch did not write the outline in.
- `footprint_padding: Double value is: 0.009999999776482582`: Nav2's default, 0.01, kept as a
  float (`nav2_costmap_2d`'s `costmap_2d_ros.cpp`). That costmap's section did not reach it: a
  misspelt key, or a container started without its parameter file
  ([How the parameters reach the nodes](#how-the-parameters-reach-the-nodes)). A misspelt
  `footprint` would not show up here: the launch writes `footprint` in by its full path, and
  adds it where the file lacks it.
- `base_link`: the frame those servers use by default. Their section did not reach them.

**H3. Stamped commands, one type per topic.**

```bash
for n in controller_server velocity_smoother collision_monitor docking_server behavior_server; do ros2 param get /robot1/$n enable_stamped_cmd_vel; done
for t in cmd_vel_nav cmd_vel_smoothed cmd_vel; do ros2 topic info /robot1/$t; done
```

It prints `Boolean value is: True` five times, then:

```text
Type: geometry_msgs/msg/TwistStamped
Publisher count: 3
Subscription count: 1
Type: geometry_msgs/msg/TwistStamped
Publisher count: 1
Subscription count: 1
Type: geometry_msgs/msg/TwistStamped
Publisher count: 2
Subscription count: 1
```

`cmd_vel_nav` has three publishers, the controller and the spin and wait behaviours; `cmd_vel`
has two, the collision monitor and the docking server. A `False`, or a topic that lists two
types (`Type: ['geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped']`), means a server
sends or expects unstamped commands; diff_drive_controller takes only stamped ones. (S5 adds
`cmd_vel_teleop` and `cmd_vel_mux`.)

**H4. Topic map.**

```bash
ros2 topic info -v /robot1/scan
ros2 topic info -v /map
ros2 topic list | grep -xE '/(scan|odom|speed_limit|robot1/map|local_costmap/published_footprint)'
```

- `/robot1/scan` has one publisher, the bridge `gz_bridge`, and four subscribers: `amcl` and
  `collision_monitor` (`Node namespace: /robot1`), `local_costmap`
  (`/robot1/local_costmap`) and `global_costmap` (`/robot1/global_costmap`), all
  `BEST_EFFORT` like the bridge.
- `/map` has one publisher, `map_server` (namespace `/`), and two subscribers: `amcl` and
  `global_costmap`.
- The `grep` prints nothing: none of `/scan`, `/odom`, `/speed_limit`, `/robot1/map` and
  `/local_costmap/published_footprint` exists. These are the absolute names of Nav2's sample
  parameter file; any of them means a topic left the robot's namespace.

**H5. Limits, rates and the route server.**

```bash
P /robot1/velocity_smoother max_velocity min_velocity max_accel max_decel smoothing_frequency velocity_timeout stamp_smoothed_velocity_with_smoothing_time feedback
P /robot1/controller_server controller_frequency speed_limit_topic FollowPath.desired_linear_vel FollowPath.rotate_to_heading_angular_vel FollowPath.max_angular_accel FollowPath.allow_reversing FollowPath.use_rotate_to_heading FollowPath.inflation_cost_scaling_factor goal_checker.xy_goal_tolerance
P /robot1/behavior_server cycle_frequency behavior_plugins
for c in local global; do P /robot1/${c}_costmap/${c}_costmap inflation_layer.inflation_radius inflation_layer.cost_scaling_factor; done
P /robot1/route_server operations
P /robot1/collision_monitor transform_tolerance source_timeout base_shift_correction stop_pub_timeout cmd_vel_in_topic
```

The values of `config/nav2_params.yaml`, in the order asked:

- velocity smoother: `[1.0, 0.0, 1.0]`, `[0.0, 0.0, -1.0]`, `[0.5, 0.0, 2.5]`,
  `[-1.0, 0.0, -2.5]`, `20.0`, `0.2`, `True`, `OPEN_LOOP`
- controller server: `20.0`, `speed_limit`, `1.0`, `1.0`, `2.5`, `False`, `False`, `3.0`, `0.1`
- behavior server: `20.0`, `['spin', 'wait']`
- each costmap: `0.7`, `3.0`
- route server: `['ReroutingService']`. Its default list also has `AdjustSpeedLimit`, which
  would publish on `speed_limit` next to the speed zones (S6b).
- collision monitor: `0.2`, `0.5`, `True`, `1.0`, `cmd_vel_smoothed` (S5 makes it
  `cmd_vel_mux`)

**H6. Trees without `BackUp`.** It prints each tree's path, then how many lines name `BackUp`:

```bash
for k in default_nav_to_pose_bt_xml default_nav_through_poses_bt_xml; do f=$(ros2 param get /robot1/bt_navigator $k | sed 's/^String value is: //'); echo "$f"; grep -c BackUp "$f"; done
```

The paths end in `share/warehouse_navigation/behavior_trees/navigate_to_pose.xml` and
`share/warehouse_navigation/behavior_trees/navigate_through_poses.xml`, and each is followed by
`0`. Nav2's stock trees would print `1`.

**H7. Costmap and collision monitor sources.**

```bash
for c in local global; do P /robot1/${c}_costmap/${c}_costmap voxel_layer.observation_sources voxel_layer.puck.topic voxel_layer.puck.max_obstacle_height voxel_layer.puck.marking voxel_layer.puck.clearing; done
P /robot1/collision_monitor observation_sources puck.topic puck.type
```

- each costmap: `puck`, `/robot1/scan`, `0.8`, `True`, `True`
- collision monitor: `['puck']`, `scan`, `scan`

`0.0` and a `False` for `clearing` are the obstacle layer's defaults (`obstacle_layer.cpp`):
that costmap's layer section did not reach it, and with a top of 0.0 m it marks nothing. A
costmap topic without the leading `/robot1` is relative and resolves under the costmap node,
for example `/robot1/local_costmap/scan`, where nothing publishes. The monitor's `scan` is
relative on purpose: the monitor is a server node in `/robot1`.

Pasted exactly as written here on 2026-11-16, H1-H7 printed the values above in two runs, with
every node active 11.4 s and 12.4 s after the launch. With a copy of `nav2_params.yaml` whose
local costmap section misspells `footprint_padding` (`params_file:=` the copy), H2 printed
`0.009999999776482582` for the local costmap and `0.04` for the global one.

### Things that look wrong but are not

- At start-up the costmaps log `Timed out waiting for transform from base_footprint to odom` (or
  `to map`) a few times, until the EKF and AMCL publish their transforms.
- `docking_server` warns `Dock database filepath nor dock parameters set`: there are no docks
  until Phase 5.
- At the first goal `controller_server` warns `No goal checker was specified in parameter
  'current_goal_checker'` (and the same for the progress checker): the goal names none, so the
  server uses the only one it has.
- `ros2 topic info /robot1/cmd_vel_nav` counts 3 publishers: the controller and one for each
  behaviour (spin, wait). `/robot1/cmd_vel` counts 2: the collision monitor and the docking
  server, which publishes only while it docks.
- At Ctrl-C, launch reports `process[component_container_isolated-10] failed to terminate '5'
  seconds after receiving 'SIGINT', escalating to 'SIGTERM'`, sometimes also `... escalating to
  'SIGKILL'`, then `process has died` with exit code -11 (a segmentation fault) or -9 (killed).
  Every server deactivates and cleans up within about a second, but the container does not
  finish its own shutdown, so launch ends it. The container shut down this way in every run of
  the [PC assessment](../../docs/research/PC_ASSESSMENT.md) (§5.3, "Nav2") and of stage S2a.
  Every process still stops, 6-11 s after the Ctrl-C.

## Running

Mapping and localization need Gazebo (the lidar). From the workspace root:

```bash
# Make a map (maps/README.md has the whole procedure)
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam
ros2 lifecycle get /robot1/slam_toolbox            # active

# Localize on the saved map
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl
ros2 lifecycle get /map_server                     # active
ros2 lifecycle get /robot1/amcl                    # active
ros2 topic info -v /robot1/tf                      # publishers include amcl
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static

# Navigate with Nav2 (AMCL runs inside Nav2's container); a goal: see "See it" above
ros2 launch warehouse_bringup sim.launch.py world:=warehouse navigation:=true gz_gui:=false
```

In RViz (`map.rviz`, fixed frame `map`) the scan should lie on the map's walls. Drag the
"2D Pose Estimate" tool on the map to put AMCL somewhere else, then drive and watch the
particle cloud find the robot again.

The map server also runs on its own, without a simulator. A late subscriber still receives the
map:

```bash
ros2 run warehouse_layout rasterize --out /tmp/layout_map      # a map drawn from the layout
ros2 launch warehouse_navigation map_server.launch.py map:=/tmp/layout_map.yaml
ros2 topic echo /map --qos-durability transient_local --qos-reliability reliable --once --field info
```
