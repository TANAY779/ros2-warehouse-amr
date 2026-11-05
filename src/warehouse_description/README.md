# warehouse_description

The robot: a tote-carrying pick-assist AMR of our own design (plan §2.6, D1), described once in
`config/robot.yaml` and turned into URDF by xacro. The same description drives Gazebo
(`hardware:=gz`) and mock hardware without a simulator (`hardware:=mock`), through ros2_control
(D2).

**Revision B** (Phase 1): a diff-drive base (Rev A: body, drive with sphere wheel collisions,
casters, ros2_control) plus the first sensors (Rev B: a 360° lidar puck on the deck and an IMU
read through ros2_control). No cameras until Rev E.

**The look** (the owner's design review, 2026-11-05): safety yellow and charcoal, built from simple
parts that are easy to tell apart.

| Part | Colour | Link | Module |
|---|---|---|---|
| Chassis: skirt, wheel arches, deck top | charcoal | `base_link` | `body.xacro` (`chassis`) |
| Body panels (the shell), with a well for each wheel | safety yellow | `shell_link` | `body.xacro` (`shell`) |
| Front and rear bumpers, wrapping round the corners | black | `front_bumper_link`, `rear_bumper_link` | `body.xacro` (`bumper`) |
| Tote tray: a 1.5 cm rim on the rear part of the deck | grey | `tote_tray_link` | `deck.xacro` (`tote_tray`) |
| Emergency-stop button, front-right corner of the deck | red | `emergency_stop_link` | `deck.xacro` (`emergency_stop`) |
| Status light, front-left corner of the deck | green (set in `materials.xacro`) | `status_light_link` | `deck.xacro` (`status_light`) |
| Lidar puck: fixed base and spinning head | graphite, black | `lidar_front_mount_link`, `lidar_front_link` | `lidar.xacro` |
| Wheels: tyre and hub | black, grey | `<side>_wheel_link`, `<side>_wheel_hub_link` | `drive.xacro` (`wheel`) |
| Casters | grey | `<position>_caster_link` | `drive.xacro` (`caster`) |

## Files

| File | What it does |
|---|---|
| `config/robot.yaml` | The single source of the geometry: outline, heights, masses, wheels, casters, hard limits, the tote tray, the lidar and the IMU. The xacro reads it with `xacro.load_yaml()`; nothing in the URDF repeats its numbers. Purely cosmetic sizes (bumper depth, button size) are named properties in the module that draws them |
| `config/controllers.yaml` | ros2_control: controller manager (100 Hz, `enforce_command_limits`), `joint_state_broadcaster`, `diff_drive_controller`, `imu_sensor_broadcaster`. Keys start with `/**/` so they match in any namespace |
| `urdf/robot.urdf.xacro` | Top level: the `namespace` and `hardware` arguments, loads `robot.yaml`, includes the parts below |
| `urdf/inertia.xacro` | `<inertial>` macros for a box, a cylinder and a sphere |
| `urdf/materials.xacro` | The colour palette for RViz and Gazebo, and the status light's colour (one property, for Phase 9's per-robot colours) |
| `urdf/base.xacro` | `base_footprint` (root, on the floor), `base_link` (on the drive axle) with the chassis visuals, one collision box and the body inertia |
| `urdf/body.xacro` | The body's visible parts, one macro each: `chassis` (charcoal, inside `base_link`), `shell` (yellow panels) and `bumper` (front and rear) |
| `urdf/deck.xacro` | What stands on the deck, one macro each: `tote_tray`, `emergency_stop`, `status_light` |
| `urdf/drive.xacro` | The two wheels (continuous joints, sphere collisions, a tyre and a hub) and the two frictionless casters |
| `urdf/lidar.xacro` | A 2D lidar puck macro: the fixed base (`<name>_mount_link`), the spinning head (`<name>_link`, the scan plane) and the gz `gpu_lidar` sensor. Rev B uses it once (`lidar_front`) |
| `urdf/imu.xacro` | The IMU link and its gz `imu` sensor with Gaussian noise |
| `urdf/ros2_control.xacro` | The `<ros2_control>` block (wheel velocity commands, wheel and IMU states) with the hardware plugin for `gz` or `mock`, and the `gz_ros2_control` system plugin for `gz` |
| `urdf/gazebo.xacro` | Gazebo-only systems: the ground-truth `OdometryPublisher` |
| `test/test_description.py` | pytest: builds the xacro for every `hardware`, runs `check_urdf`, and checks frames, mass, namespaced gz topics, the lidar (pose, full circle, nothing but its head in the scan plane), that every visual stays inside the outline, one colour per link, and `controllers.yaml` against `robot.yaml` |

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
                                   ├── lidar_front_link (0.25, 0, 0.34 m: the scan plane)
                                   ├── imu_link         (0, 0, 0.10 m)
                                   ├── left_wheel_link, right_wheel_link (joint_states)
                                   │   └── left_wheel_hub_link, right_wheel_hub_link
                                   ├── front_caster_link, rear_caster_link
                                   └── shell_link, front_bumper_link, rear_bumper_link,
                                       tote_tray_link, emergency_stop_link,
                                       status_light_link, lidar_front_mount_link
```

The links in the last group and the hubs are visual only: fixed, massless, one colour each
(design notes). Gazebo merges them into `base_footprint` and the wheels. `tote_tray_link` sits in
the middle of the tray on the deck, where a tote model goes (no phase adds one yet; plan §2.6,
Rev C).

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
| Lidar, IMU and the visual-only parts | none | | 0 | |
| **Total** | | | **35.0 kg** | **0.140 m** |

The battery sits low, so the body's centre of mass is 2 cm below the centre of its box; the
uniform-box inertia about that point is a close approximation. Sensor links and the visual-only
parts have no mass: on a fixed joint Gazebo merges them into the base anyway, and the body's mass
already includes them. The test checks the total (35 ± 0.5 kg) and
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
- **A lidar puck on the deck, 0 self-hits.** The lidar stands on the deck 7 cm behind the front
  of the outline, TurtleBot style, and scans one horizontal plane 0.34 m above the floor, all
  round: 720 rays, 0.5° apart. The first ray points straight back (-180°) and the last at
  +179.5°, so the same direction is never measured twice. `gpu_lidar` renders the scene, so it
  sees visuals, not collisions: its own spinning head (radius 4 cm) is hidden by `range_min`
  (10 cm), and every other part stays at least 2 cm below the scan plane (tote tray rim 0.315 m,
  stop button 0.316 m, status light 0.318 m, the puck's base 0.32 m). A test checks every visual.
  What the puck cannot see: anything below 0.34 m (pallets, forks, the 0.30 m bodies of other
  robots like this one; only their pucks show) and, once a tote rides on the tray, much
  of the rear. Phase 3 decides how to cover both (plan §2.6, Rev C).
- **One colour per link.** RViz 2 (Jazzy) draws every `<visual>` of a link in the material of the
  link's first visual: `rviz_default_plugins` passes no per-visual material for primitive shapes.
  A body built as differently coloured visuals of `base_link` showed up in RViz in one colour.
  Each coloured part is therefore a link of its own, on a fixed joint and without mass, as in
  Clearpath's Husky (`front_bumper_link`, `top_plate_link`) and TurtleBot 4 (`shell_link`).
  Gazebo keeps each visual's colour anyway and merges the links, so the physics are unchanged.
  `test_each_link_has_one_colour` guards the rule. RViz's TF display starts with these frames
  hidden (`warehouse_bringup/rviz/robot.rviz`).
- **Colours in Gazebo and RViz.** Gazebo's URDF converter (sdformat) multiplies a URDF colour by
  1.25 and clips at 1, so every channel stays at or below 0.8 to keep its hue. Gazebo's ogre2
  renderer then lights in linear space and outputs sRGB, so dark colours look lighter than in
  RViz, which uses the values as they are, lit from the camera. The palette in `materials.xacro`
  is a compromise checked in both: the yellow (0.80, 0.58, 0.0) shows as safety yellow in
  Gazebo, a little more golden in RViz; the charcoal (0.09) is dark grey in Gazebo and nearly
  black in RViz. Under the default sun the faces turned away from it (rear, left side) get
  ambient light only and look darker.
- **Status light colour.** `status_light_rgba` at the top of `materials.xacro` is the one place
  to change it. Phase 9, which gives each robot its own colour, turns it into a xacro argument.
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
| B | 1 | 360° lidar puck on the deck (`scan`), IMU through ros2_control; body restyled in safety yellow and charcoal with a tote tray, an emergency-stop button and a status light (current). A front-left corner scanner in a groove around the body (264°, scan plane 0.20 m) came first; the owner replaced it on 2026-11-05 |

Later revisions (rear and low coverage, charging contacts, cameras, ...) are listed in plan §2.6.

## Tests

Checks are paused (ADR-004): a plain `colcon build` builds without tests. To run them, build
with tests first:

```bash
colcon build --symlink-install --metas /dev/null --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select warehouse_description && colcon test-result --verbose
```
