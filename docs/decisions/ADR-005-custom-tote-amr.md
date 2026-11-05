# ADR-005: Build our own tote AMR on ros2_control, with an EKF for odometry

- **Status:** accepted
- **Date:** 2026-11-05
- **Plan reference:** D1, D2, D3 in docs/PROJECT_PLAN.md (§2.6 robot design, Phase 1)

## Context

Every later phase (mapping, navigation, docking, inventory, fleet) runs on one robot model, so
the robot has to fit warehouse aisles, carry a tote, see around itself and report honest
odometry. Three choices shape it:

1. **Which robot.** A borrowed TurtleBot or a robot of our own.
2. **How the base is driven.** Gazebo's own `DiffDrive` system, or ros2_control.
3. **How odometry is produced.** Raw wheel odometry, an EKF, or SLAM.

Constraints: simulation only (no hardware, plan A1), ROS 2 Jazzy with Gazebo Harmonic, up to
three identical robots on a laptop without an NVIDIA GPU, and a learning goal: the project should
teach URDF/xacro, ros2_control, gz sensors and sensor fusion, not hide them.

## Decision

**D1: our own tote-carrying pick-assist AMR** from one `robot.yaml`
(`warehouse_description/config/robot.yaml`), built in revisions A-G. Rule: geometry is
generated (xacro reads `robot.yaml`), tuning is validated (tests compare `controllers.yaml` and
later Nav2 and safety parameters with it). Phase 1 delivers revisions A and B:

- 0.64 x 0.48 m outline, 35 kg, centre of mass 0.14 m; `base_footprint` root on the floor,
  `base_link` on the drive axle, which is the geometric centre (turns in place in its own circle).
- Diff drive, wheel radius 0.075 m, separation 0.40 m. Wheels collide as spheres (a cylinder
  skewed in-place yaw odometry by 8-9 %); two frictionless ball casters.
- A 360° `gpu_lidar` puck (`scan`) standing on the deck near the front edge, scan plane 0.34 m:
  720 rays 0.5° apart, 10 Hz, 0 self-hits. IMU at the base centre, 100 Hz, read through
  ros2_control.
- A body in safety yellow and charcoal, built from simple parts that are easy to tell apart
  (skirt, panels, bumpers, deck top, tote tray, emergency stop, status light). Each coloured part
  is a fixed, massless link, because RViz draws all visuals of a link in one colour; the physics
  still sees one collision box.
- The lidar and the look are the owner's choices from the Phase 1 design review (2026-11-05). The
  first Rev B had a front-left corner scanner in a groove round the body (264°, scan plane
  0.20 m); the owner chose the puck instead, accepting that a loaded tote blocks much of its
  rear view. Rear and low coverage is an open Phase 3 question (plan §2.6, Rev C).
- Namespaced from day one: gz topics prefixed with the robot namespace, frames unprefixed,
  every gz sensor sets `<gz_frame_id>`.

**D2: ros2_control** with `diff_drive_controller`, `joint_state_broadcaster` and
`imu_sensor_broadcaster`, and one description with `hardware:=gz|mock|real`:
`gz_ros2_control/GazeboSimSystem` in Gazebo, `mock_components/GenericSystem` without a simulator
(CI), `real` reserved for a future `SystemInterface` (xacro refuses it). `controllers.yaml` uses
`/**/` keys, the spawners remap the controllers' private topics (`~/cmd_vel` → `cmd_vel`,
`~/odom` → `odom/wheel`, `~/imu` → `imu`), and the controller manager enforces the URDF wheel
limit (`enforce_command_limits: true`).

**D3: a `robot_localization` EKF from Phase 1** fusing the wheel twist (vx, vy = 0, yaw rate)
and the IMU yaw rate in `two_d_mode`. A diff drive cannot slide, so the controller's vy is
always 0; fusing that zero keeps the y covariance bounded (unfused, it reached about 4e4 m²
after 30 s of standing still and kept growing). The EKF alone publishes `odom` and
`odom -> base_footprint` (`enable_odom_tf: false` on the controller). The IMU orientation is not
fused: Gazebo reports it noise-free, which would flatter localization. AMCL on a static map
follows in Phase 2; slam_toolbox only builds the map.

## Alternatives considered

| Option | Why not |
|---|---|
| TurtleBot3 | Reads as a tutorial; too small for totes and warehouse aisles |
| TurtleBot4 | A borrowed robot teaches less (no URDF, ros2_control or sensor design of our own) |
| Kiva-style shelf lifter | Needs a different deck, docking and manipulation story (P13) |
| Per-robot sensor profiles | Would need capability-aware task allocation; all robots stay identical |
| Corner safety scanners low in the body (the first Rev B: a front-left 264° scanner in a groove, a rear-right twin planned for Rev C) | How industrial AMRs do it: low scan planes see small obstacles, and two diagonal scanners see past any load. The owner preferred the simpler puck on the deck (2026-11-05); Phase 3 may bring low scanners back |
| A raised sensor head above the tote | Would see over a loaded tote, but makes the robot taller and busier; the owner chose the puck on the deck |
| Gazebo `DiffDrive` system through the bridge | No hardware abstraction: mock hardware in CI and a future real robot would need a different launch path. Kept as the fallback if multi-robot `gz_ros2_control` blocks Phase 9 (plan R2) |
| A `use_sim` boolean instead of `hardware:=` | Cannot express mock hardware |
| Raw wheel odometry only | Wheel slip and yaw errors go uncorrected; real AMRs fuse an IMU |
| Fusing the gz IMU orientation | It is ground truth; the numbers would look better than any real robot's |
| SLAM for localization in production | Deployed AMRs localize on a known map; SLAM only builds it |

## Consequences

- One launch path for every hardware option: `warehouse_bringup/launch/sim.launch.py
  hardware:=gz|mock`. Mock hardware tests the wiring in every pull request without Gazebo;
  Gazebo tests run locally (and nightly from Phase 6). A render-free test
  (`warehouse_gazebo/test/test_gazebo_config.py`) checks the Gazebo-side files in every pull
  request: the world's systems, the bridged topics against the URDF, and the caster friction
  after Gazebo's URDF conversion.
- Measured in Phase 1 (15 Gazebo test runs, corner scanner): rates 10 / 100 / 50 Hz (scan / imu /
  odom/wheel, sim time); 0 scan self-hits; wheel odometry 0.01 % over 10 m and 0.00 % over a 360°
  turn;
  EKF 0.1-0.4 % and at most 0.05 % (plan limits: 1 % and 2 %). The EKF integrates the
  controller's reported speed, which reads about 0.2 % high in Gazebo because the controller
  manager's update stamps jitter (`warehouse_navigation/README.md`). After the change to the puck
  (2026-11-05; physics unchanged), measured again in headless Gazebo: scan at 10.0 Hz over 360°
  (720 rays); 0 self-hits in 100 scans with the launch test's box beside the robot, and in 20
  more with boxes beside, behind and in front of it.
- The robot description is the single source for later phases: the Nav2 footprint (P3), safety
  fields (P3), docking geometry (P5) and camera placement (P7) are derived from or checked
  against `robot.yaml`.
- Every `TwistStamped` publisher must stamp with sim time in Gazebo, or `cmd_vel_timeout` never
  fires (plan R1). Teleop runs with `stamped:=true use_sim_time:=true`.
- Phase 9 risk found while building Phase 1: `gz_ros2_control` 1.2.20 registers IMU sensors by
  name across the whole world, not per model. Two robots with an IMU named `imu_sensor` would
  both match it, so Phase 9 must make the sensor name unique per robot (or confirm the plugin's
  behaviour first, R2).
- More to maintain than a borrowed robot: inertias, collisions and sensor placement are ours to
  get right. `warehouse_description/test` and `warehouse_system_tests` guard them.
