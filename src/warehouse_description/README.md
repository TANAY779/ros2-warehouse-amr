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
| `config/robot.yaml` | The single source of the geometry: outline, heights, masses, wheels, casters, hard limits, the tote tray, the lidar and the IMU, and the inputs of the safety fields (`safety`: reaction time, stop margin, rotation margin). The xacro reads it with `xacro.load_yaml()`; nothing in the URDF repeats its numbers. Purely cosmetic sizes (bumper depth, button size) are named properties in the module that draws them |
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
| `warehouse_description/robot_config.py` | Python module: `robot_config.load()` returns `robot.yaml` as a frozen dataclass plus the numbers derived from it (half length and width, inscribed and circumscribed radii with and without padding, the scan plane height, the outline as a polygon and as Nav2's `footprint` string). `load_base()` adds the base's limits from `controllers.yaml`, and `field_need()` is the sweep model of the safety fields ([below](#the-safety-fields-safety_fields)). Other packages' Python code uses it instead of copying numbers (the warehouse layout's fit rules, Nav2's footprint) |
| `warehouse_description/safety_fields.py` | `ros2 run warehouse_description safety_fields`: what each safety field must cover, next to the field in `warehouse_navigation`'s `nav2_params.yaml` ([below](#the-safety-fields-safety_fields)). Reads files only, no ROS node |
| `scripts/safety_fields` | Starts the tool above. CMake installs it as `lib/warehouse_description/safety_fields`, the file `ros2 run` runs |

## Arguments

| Argument | Values | Effect |
|---|---|---|
| `namespace` | e.g. `robot1` (default empty) | Prefixes every gz transport topic (`/robot1/scan`) and puts the controller manager in `/robot1`. gz topics are global, so without the prefix all robots would share one topic (plan R12). Frames are never prefixed (D7) |
| `hardware` | `gz` (default), `mock`, `real` | `gz`: `gz_ros2_control/GazeboSimSystem` plus the Gazebo plugins. `mock`: `mock_components/GenericSystem` with `calculate_dynamics`, no Gazebo plugins. `real` is reserved for a future `SystemInterface` and stops xacro with a clear message, as does any unknown value |

The robot's numbers in Python (after `colcon build` and sourcing the workspace):

```bash
python3 -c "from warehouse_description import robot_config as rc; c = rc.load(); print(c.r_circumscribed_padded, c.scan_plane_height)"
# 0.45607017003965516 0.34
```

Build and check the URDF by hand:

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
already includes them.

## Design notes

- **Sphere wheel collisions (plan R13).** The wheels look like cylinders but collide as spheres.
  A cylinder touches the floor along a line whose ends drag when the robot turns in place; in
  the prototype that skewed in-place yaw odometry by -8 to -9 %. A sphere touches in one point
  (0.00 %). The spheres stick out 3.5 cm past the outline, inside the 4 cm Nav2 footprint padding.
- **Frictionless casters, unnamed collisions.** The casters only carry load (`mu1`/`mu2` 0 in a
  `<gazebo reference>` block). Gazebo's URDF converter applies such surface settings only to the
  collision names it generates itself (`<link>_collision`): a `<collision name="...">` silently
  loses them. All collisions are therefore unnamed; Gazebo's converted model (`gz sdf -p`)
  shows whether both casters kept friction 0.
- **A lidar puck on the deck, 0 self-hits.** The lidar stands on the deck 7 cm behind the front
  of the outline, TurtleBot style, and scans one horizontal plane 0.34 m above the floor, all
  round: 720 rays, 0.5° apart. The first ray points straight back (-180°) and the last at
  +179.5°, so the same direction is never measured twice. `gpu_lidar` renders the scene, so it
  sees visuals, not collisions: its own spinning head (radius 4 cm) is hidden by `range_min`
  (10 cm), and every other part stays at least 2 cm below the scan plane (tote tray rim 0.315 m,
  stop button 0.316 m, status light 0.318 m, the puck's base 0.32 m).
  What the puck cannot see: anything below 0.34 m (pallets, forks, the 0.30 m bodies of other
  robots like this one; only their pucks show) and, once a tote rides on the tray, much
  of the rear. Phase 3 decides how to cover both (plan §2.6, Rev C).
- **One colour per link.** RViz 2 (Jazzy) draws every `<visual>` of a link in the material of the
  link's first visual: `rviz_default_plugins` passes no per-visual material for primitive shapes.
  A body built as differently coloured visuals of `base_link` showed up in RViz in one colour.
  Each coloured part is therefore a link of its own, on a fixed joint and without mass, as in
  Clearpath's Husky (`front_bumper_link`, `top_plate_link`) and TurtleBot 4 (`shell_link`).
  Gazebo keeps each visual's colour anyway and merges the links, so the physics are unchanged.
  RViz's TF display starts with these frames hidden (`warehouse_bringup/rviz/robot.rviz`).
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

## The safety fields: `safety_fields`

Nav2's collision monitor stops the robot when the lidar has points inside a *field*, a polygon
round the robot. The field depends on the command: one polygon per band of commanded speed v and
turn rate ω, six bands in all (`velocity_fields` in `warehouse_navigation`'s
`config/nav2_params.yaml`, plan §3.4). The monitor uses the first band that contains the command.
Each polygon must hold the area the robot sweeps from the moment an obstacle appears until it
stands: for that long, nothing stops it. The polygons are written by hand; this tool prints
what each one must cover, worked out from this package's files, next to the polygon itself:

```bash
ros2 run warehouse_description safety_fields
# --robot, --controllers, --nav2-params: read a copy of a file instead of the installed one
```

**The model** (`robot_config`'s `field_need()`, the plan's `sweep_check.py`). An obstacle
appears. The command (v, ω) holds for the reaction time, then the base brakes v and ω to zero,
each at its own deceleration. The outline's corners are moved in steps of 2 ms, and the extents
of everything they pass are kept, in `base_footprint` as it stood when the obstacle appeared.
That is done for 7 x 7 commands spread over the band, each clamped to what the base carries out,
and the extents are grown by the stop margin. Each input and the file it comes from:

| Input | Value | From |
|---|---|---|
| Outline | 0.64 x 0.48 m | `robot.yaml`: `footprint.length`, `footprint.width` |
| Braking | 1.5 m/s² | `robot.yaml`: `limits.linear.max_deceleration` |
| Reaction time | 0.25 s | `robot.yaml`: `safety.reaction_time` (its comments give the budget) |
| Stop margin | 0.05 m | `robot.yaml`: `safety.stop_margin` |
| Rotation margin | 0.08 m | `robot.yaml`: `safety.rotation_margin` |
| The base's limits | v -0.3 … 1.2 m/s, ω ±2.0 rad/s | `controllers.yaml`: `diff_drive_controller` `linear.x`, `angular.z` |
| Angular braking | 3.0 rad/s² | `controllers.yaml`: `angular.z.max_deceleration` |
| Nav2's limits | v 0.0 … 1.0 m/s, ω ±1.0 rad/s | `nav2_params.yaml`: `velocity_smoother` |
| The bands | ranges and polygons | `nav2_params.yaml`: `collision_monitor` `velocity_fields` |

**The table** (its first lines list the files and these inputs):

```text
                                             need                   polygon                spare
 #  band       v [m/s]        ω [rad/s]       front    rear    |y|   front    rear    |y|   front    rear    |y|
 1  rotate     -0.05 … 0.05   -2.05 … 2.05    radius 0.463           radius 0.480           radius 0.017
 2  reverse    -0.30 … -0.05  -1.00 … 1.00    0.427  -0.542  0.417   0.430  -0.550  0.420   0.003   0.008  0.003
 3  slow       0.05 … 0.30    -1.00 … 1.00    0.542  -0.427  0.417   0.550  -0.430  0.420   0.008   0.003  0.003
 4  aisle      0.30 … 0.50    -1.00 … 1.00    0.642  -0.370  0.441   0.650  -0.370  0.450   0.008   0.000  0.009
 5  fast       0.50 … 1.00    -1.00 … 1.00    0.997  -0.369  0.549   1.000  -0.370  0.550   0.003   0.001  0.001
 6  catch_all  -1.25 … 1.25   -2.05 … 2.05    1.183  -0.551  0.870   1.190  -0.560  0.880   0.007   0.009  0.010
```

- `#`: the order in which the monitor tries the bands; the first that contains the command wins.
- `v`, `ω`: the commands the band takes, bounds included.
- `need`: how far the swept outline plus the stop margin reaches, in metres from
  `base_footprint`: forward (`front`), backward (`rear`, negative) and to either side (`|y|`).
- `polygon`: how far the configured polygon reaches the same ways; for the sides, the nearer
  of the two.
- `spare`: the polygon minus the need on each side, so how much further the polygon reaches.
  A negative spare means the polygon is too small on that side.
- `rotate` is not a rectangle but a 24-sided polygon round the circle a robot standing or
  turning in place keeps clear, the corner radius plus the rotation margin, 0.48 m (plan §2.6).
  For it the table gives radii: the farthest the sweep reaches from `base_footprint`, and the
  nearest side of the polygon (0.4796 m; its corners are 0.484 m out). A line under the table
  repeats that circle.

The last part lists, for three sets of commands, the bands they select and the commands that
fall in no band. For such a command the monitor keeps the previous field and warns. The sets are
Nav2's commands (the velocity smoother's limits), every command the base carries out, and the
teleop window's, whose default top speeds are the base's limits and which reverses as fast as it
drives forward. Today Nav2 uses `rotate`, `slow`, `aisle` and `fast`; the base and the teleop
window use all six; and no command falls in no band. Until stage S5 the teleop window publishes
straight to `cmd_vel`, past the monitor. S5 sends it through twist_mux into the monitor, and caps
its reverse at 0.3 m/s.

**Why a table without a verdict.** The owner chose not to have a checker tool (plan Q4 (c)).
What decides whether the fields are right is the robot: the safety scenarios of stages S8a
(turning in place next to a post) and S8b (stopping for boxes), measured in Gazebo. The table
shows the reasoning behind the polygons and how much room each one has, so a change to
`robot.yaml`, `controllers.yaml` or the fields can be read at a glance. It always exits 0 once it
has printed the table, whatever the spares; it exits 1 only when it cannot read a file. The model
reproduces the plan's `sweep_check.py --reaction 0.25` exactly. Its precision is about 1 mm:
with 1 ms steps, three of the needs in the table grow by 1 mm (`fast`'s rear to -0.370,
`catch_all`'s front to 1.184 and its sides to 0.871). So the spares under 1 mm, at the rear of
`aisle` and `fast` and at the sides of `fast`, are within the model's precision. Those two rear
needs are little more than the outline's own rear plus the margin, 0.32 + 0.05 m.

`safety_fields` reads `nav2_params.yaml` from `warehouse_navigation`'s share directory, but this
package cannot declare a dependency on `warehouse_navigation`, which depends on this one: colcon
refuses the cycle. If `warehouse_navigation` is not built, the tool says so; build it, or pass
`--nav2-params`.

## Revisions

| Rev | Phase | Change |
|---|---|---|
| A | 1 | Body, drive with sphere wheel collisions, casters, ros2_control for `gz` and `mock` |
| B | 1 | 360° lidar puck on the deck (`scan`), IMU through ros2_control; body restyled in safety yellow and charcoal with a tote tray, an emergency-stop button and a status light (current). A front-left corner scanner in a groove around the body (264°, scan plane 0.20 m) came first; the owner replaced it on 2026-11-05 |

Later revisions (rear and low coverage, charging contacts, cameras, ...) are listed in plan §2.6.

## Checking a change

Nothing checks this package automatically
([ADR-008](../../docs/decisions/ADR-008-develop-on-main-without-automated-tests.md)). After
changing `robot.yaml`, a xacro file or `controllers.yaml`, rebuild and check by hand:

```bash
xacro $(ros2 pkg prefix warehouse_description)/share/warehouse_description/urdf/robot.urdf.xacro \
  namespace:=robot1 hardware:=gz > /tmp/robot.urdf
check_urdf /tmp/robot.urdf          # prints the link tree
```

then run the robot and go through
[Checking the robot by hand](../warehouse_bringup/README.md#checking-the-robot-by-hand). Numbers
that are in both `robot.yaml` and `controllers.yaml` (wheel radius and separation, the speed and
acceleration limits, the IMU rate and its noise as covariances σ²) must be changed together.
