# warehouse_navigation

Localization and navigation of the warehouse AMR. Phase 1 adds the odometry EKF; Phase 2 makes
a map of the warehouse with slam_toolbox and localizes on it with AMCL; Nav2 follows in Phase 3,
docking in Phase 5.

## Files

| File | What it does |
|---|---|
| `config/ekf.yaml` | `robot_localization` EKF parameters (key `/**/ekf_filter_node`, so it works in any namespace) |
| `launch/ekf.launch.py` | Starts `ekf_node` as `ekf_filter_node` and remaps its output `odometry/filtered` to `odom`. Include it inside the robot's namespace together with the `/tf` remaps, as `warehouse_bringup/launch/robot.launch.py` does |
| `config/slam_toolbox.yaml` | slam_toolbox's parameters: its own sample with our changes marked ([below](#slam_toolbox-what-changed-and-why)) |
| `config/amcl.yaml` | AMCL's parameters, and those of its lifecycle manager `lifecycle_manager_localization` |
| `config/map_server.yaml` | `map_server`'s parameters, and those of its lifecycle manager `lifecycle_manager_map` |
| `launch/slam.launch.py` | Robot scope: slam_toolbox (`async_slam_toolbox_node`), configured and activated by launch events. Include it in the robot's namespace after the `/tf` remaps, like the EKF (`robot.launch.py localization:=slam` does) |
| `launch/localization.launch.py` | Robot scope: `amcl` and `lifecycle_manager_localization`. `x`, `y`, `yaw`: AMCL's initial pose in the map frame (`robot.launch.py localization:=amcl` passes the spawn pose) |
| `launch/map_server.launch.py` | System scope: `map_server` and `lifecycle_manager_map` in `/`, serving the `map:=` file on `/map` (default `maps/warehouse.yaml`). Refuses a map file that does not exist before it starts anything |
| `maps/` | The warehouse map AMCL uses, with a README that says how it was made ([maps/README.md](maps/README.md)) |

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
| System (`/`) | `map_server`, `lifecycle_manager_map` | `localization:=amcl`: one map for every robot (plan D20) |
| Robot (`/robot1`) | `slam_toolbox` | `localization:=slam` |
| Robot (`/robot1`) | `amcl`, `lifecycle_manager_localization` | `localization:=amcl`: one AMCL per robot |

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
started, `map_server` 0.6 s, AMCL 0.8 s.

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
| `min_laser_range` | 0.0 | (removed) | slam_toolbox 2.8.5 never reads it; the shortest range comes from each scan's `range_min` (0.10 m from `robot.yaml`) |

`max_laser_range` stays 20.0 m, the lidar's range in `robot.yaml`. `mode: mapping` stays too:
the Ceres plugin reads it.

### AMCL: what changed and why

`config/amcl.yaml` lists only some keys; every other one keeps AMCL's built-in default, which
for AMCL equals Nav2's sample (`nav2_bringup/params/nav2_params.yaml`). Changed from the sample:
`map_topic: /map` (above); `laser_max_range: -1`, which uses the scan's own 20 m from
`robot.yaml` like `laser_min_range: -1` already does (the sample's 100 m belongs to another
robot); and `set_initial_pose: true` with the spawn pose from the launch, so AMCL starts where
the robot stands. Tune only with
evidence from the evaluation (plan F7), one parameter at a time: `max_beams` first, then the
odometry noise `alpha1`-`alpha4`.

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
