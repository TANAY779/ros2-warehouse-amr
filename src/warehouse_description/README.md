# warehouse_description

The robot: a tote-carrying pick-assist AMR of our own design (plan §2.6, D1), described once in
`config/robot.yaml` and turned into URDF by xacro. The same description drives Gazebo
(`hardware:=gz`) and mock hardware without a simulator (`hardware:=mock`), through ros2_control
(D2).

**Revision B** (Phase 1): a diff-drive base (Rev A: body, drive with sphere wheel collisions,
casters, ros2_control) plus the first sensors (Rev B: the front-left 2D scanner and an IMU read
through ros2_control). No cameras until Rev E.

## Files

| File | What it does |
|---|---|
| `config/robot.yaml` | The single source of the geometry: outline, heights, masses, wheels, casters, hard limits, scanner and IMU. The xacro reads it with `xacro.load_yaml()`; nothing in the URDF repeats its numbers |
| `config/controllers.yaml` | ros2_control: controller manager (100 Hz, `enforce_command_limits`), `joint_state_broadcaster`, `diff_drive_controller`, `imu_sensor_broadcaster`. Keys start with `/**/` so they match in any namespace |
| `urdf/robot.urdf.xacro` | Top level: the `namespace` and `hardware` arguments, loads `robot.yaml`, includes the parts below |
| `urdf/inertia.xacro` | `<inertial>` macros for a box, a cylinder and a sphere |
| `urdf/materials.xacro` | Colours for RViz and Gazebo |
| `urdf/base.xacro` | `base_footprint` (root, on the floor), `base_link` (on the drive axle), the body visuals (wheel wells, scanner groove, deck), one collision box and the body inertia |
| `urdf/drive.xacro` | The two wheels (continuous joints, sphere collisions) and the two frictionless casters |
| `urdf/lidar.xacro` | A 2D scanner macro: link, housing, gz `gpu_lidar` sensor. Rev B uses it once (`lidar_front`) |
| `urdf/imu.xacro` | The IMU link and its gz `imu` sensor with Gaussian noise |
| `urdf/ros2_control.xacro` | The `<ros2_control>` block (wheel velocity commands, wheel and IMU states) with the hardware plugin for `gz` or `mock`, and the `gz_ros2_control` system plugin for `gz` |
| `urdf/gazebo.xacro` | Gazebo-only systems: the ground-truth `OdometryPublisher` |
| `test/test_description.py` | pytest: builds the xacro for every `hardware`, runs `check_urdf`, and checks frames, mass, namespaced gz topics and `controllers.yaml` against `robot.yaml` |

## Arguments

| Argument | Values | Effect |
|---|---|---|
| `namespace` | e.g. `robot1` (default empty) | Prefixes every gz transport topic (`/robot1/scan`) and puts the controller manager in `/robot1`. gz topics are global, so without the prefix all robots would share one topic (plan R12). Frames are never prefixed (D7) |
| `hardware` | `gz` (default), `mock`, `real` | `gz`: `gz_ros2_control/GazeboSimSystem` plus the Gazebo plugins. `mock`: `mock_components/GenericSystem` with `calculate_dynamics`, no Gazebo plugins. `real` is reserved for a future `SystemInterface` and stops xacro with a clear message, as does any unknown value |

Build and check the URDF by hand (after `colcon build` and sourcing the workspace):

```bash
xacro $(ros2 pkg prefix warehouse_description)/share/warehouse_description/urdf/robot.urdf.xacro \
  namespace:=robot1 hardware:=gz > /tmp/robot.urdf
check_urdf /tmp/robot.urdf          # prints the link tree
gz sdf -p /tmp/robot.urdf | less    # what Gazebo makes of it (merged links, friction, sensors)
```

## Frames (plan §2.4, Phase 1 part)

```text
odom ──(EKF)──> base_footprint ──> base_link            (drive axle, 0.075 m above the floor)
                                   ├── lidar_front_link (0.29, 0.21, 0.20 m, yaw 45°)
                                   ├── imu_link         (0, 0, 0.10 m)
                                   ├── left_wheel_link, right_wheel_link (joint_states)
                                   └── front_caster_link, rear_caster_link
```

`base_footprint` is the root and has no inertia: KDL, used by `robot_state_publisher`, does not
accept a root link with inertia. `base_link` sits on the drive axle, which is also the geometric
centre, so the robot turns in place inside its own circle. All heights in `robot.yaml` are
above the floor; the xacro subtracts the axle height for poses in `base_link`.

## Mass budget

| Part | Shape (macro) | Size | Mass | Centre of mass above the floor |
|---|---|---|---|---|
| Body (chassis, battery, drives, electronics) | box | 0.64 x 0.48 x 0.27 m | 32.6 kg | 0.145 m |
| Wheels | cylinder | r 0.075 m, width 0.04 m | 2 x 1.0 kg | 0.075 m |
| Casters | sphere | r 0.03 m | 2 x 0.2 kg | 0.03 m |
| Scanner, IMU | none | | 0 | |
| **Total** | | | **35.0 kg** | **0.140 m** |

The battery sits low, so the body's centre of mass is 2 cm below the centre of its box; the
uniform-box inertia about that point is a close approximation. Sensor links have no mass: on a
fixed joint Gazebo merges them into the base anyway. The test checks the total (35 ± 0.5 kg) and
the centre of mass (0.14 ± 0.005 m, centred in x and y).

## Design notes

- **Sphere wheel collisions (plan R13).** The wheels look like cylinders but collide as spheres.
  A cylinder touches the floor along a line whose ends drag when the robot turns in place; in
  the prototype that skewed in-place yaw odometry by -8 to -9 %. A sphere touches in one point
  (0.00 %). The spheres stick out 3.5 cm past the outline, inside the 4 cm Nav2 footprint padding.
- **Frictionless casters, unnamed collisions.** The casters only carry load (`mu1`/`mu2` 0 in a
  `<gazebo reference>` block). Gazebo's URDF converter applies such surface settings only to the
  collision names it generates itself (`<link>_collision`): a `<collision name="...">` silently
  loses them. All collisions are therefore unnamed, and a test guards it; `warehouse_gazebo`'s
  test also checks the caster friction in Gazebo's converted model (`gz sdf -p`).
- **Scanner groove, 0 self-hits.** `gpu_lidar` renders the scene, so it sees visuals, not
  collisions. Between 0.15 and 0.25 m the body is set back by 3 cm, and the scanner sits on that
  set-back corner, facing 45°. Its 264° field of view then keeps the edge rays 3° off the body
  faces. The scan plane (0.20 m) is above the wheels and casters and below the deck.
- **IMU through ros2_control.** The gz IMU is read by `gz_ros2_control` (matched by the sensor
  name `imu_sensor`) and published by `imu_sensor_broadcaster`, the path real hardware would use.
  Gyro σ 0.002 rad/s, accelerometer σ 0.02 m/s²; `controllers.yaml` sets the covariances to σ².
  The orientation is noise-free ground truth, so it gets a large covariance and the EKF ignores
  it (D3). Mock hardware reports the initial values (identity orientation, zero rates).
- **Limits.** `diff_drive_controller` enforces the body limits (v +1.2 / -0.3 m/s, ω ±2 rad/s,
  accel 0.8, decel -1.5 m/s², α ±3 rad/s²), the accelerations in both directions of travel:
  braking from reverse is 1.5 m/s² too. `controllers.yaml` therefore sets the `*_reverse` limits
  explicitly. Left unset, `max_deceleration_reverse` falls back to `max_acceleration`, and the
  robot would brake from reverse at only 0.8 m/s². The controller manager clamps wheel commands
  to the URDF joint limit of 22 rad/s because `enforce_command_limits: true` (off by default in
  Jazzy).
- **Every gz sensor sets `<gz_frame_id>`** to its URDF link, so message headers carry
  `lidar_front_link` instead of a scoped gz name. sdformat warns that the element is not in the
  SDF spec and copies it through; the sensors read it.
- **Ground truth.** `OdometryPublisher` (hardware `gz` only) publishes the robot's world pose
  with frame `map` (the Gazebo world frame is the map frame, plan §2.4). It is for tests and
  evaluation only.

## Revisions

| Rev | Phase | Change |
|---|---|---|
| A | 1 | Body, drive with sphere wheel collisions, casters, ros2_control for `gz` and `mock` |
| B | 1 | Front-left scanner `scan`, IMU through ros2_control (current) |

Later revisions (rear scanner, charging contacts, cameras, ...) are listed in plan §2.6.

## Tests

```bash
colcon test --packages-select warehouse_description && colcon test-result --verbose
```
