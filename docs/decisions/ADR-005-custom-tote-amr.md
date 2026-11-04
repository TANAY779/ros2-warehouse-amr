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
- Front-left `gpu_lidar` (`scan`) at the set-back corner of a scanner groove: 264°, 0.5°, 10 Hz,
  0 self-hits. IMU at the base centre, 100 Hz, read through ros2_control.
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
| Gazebo `DiffDrive` system through the bridge | No hardware abstraction: mock hardware in CI and a future real robot would need a different launch path. Kept as the fallback if multi-robot `gz_ros2_control` blocks Phase 9 (plan R2) |
| A `use_sim` boolean instead of `hardware:=` | Cannot express mock hardware |
| Raw wheel odometry only | Wheel slip and yaw errors go uncorrected; real AMRs fuse an IMU |
| Fusing the gz IMU orientation | It is ground truth; the numbers would look better than any real robot's |
| SLAM for localization in production | Deployed AMRs localize on a known map; SLAM only builds it |

## Consequences

- One launch path for every hardware option: `warehouse_bringup/launch/sim.launch.py
  hardware:=gz|mock`. Mock hardware tests the wiring in every pull request without Gazebo;
  Gazebo tests run locally (and nightly from Phase 6).
- Measured in Phase 1 (15 Gazebo test runs): rates 10 / 100 / 50 Hz (scan / imu / odom/wheel,
  sim time); 0 scan self-hits; wheel odometry 0.01 % over 10 m and 0.00 % over a 360° turn;
  EKF 0.1-0.4 % and at most 0.05 % (plan limits: 1 % and 2 %).
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
