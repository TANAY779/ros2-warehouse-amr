# warehouse_gazebo

Gazebo Harmonic side of the simulation: worlds, spawning a robot, and the bridge between gz
transport topics and ROS topics. Two worlds: `empty` (Phase 1) and `warehouse`, which
`warehouse_layout` generates from its `layout.yaml`.

Normally you do not start these files yourself: `warehouse_bringup/launch/sim.launch.py`
includes them.

## Files

| File | What it does |
|---|---|
| `worlds/empty.sdf` | Ground plane, sun and the Gazebo systems the robot needs (below). `sim.launch.py` spawns the robot at the origin |
| `worlds/warehouse.sdf` | The 30 x 20 m warehouse: the same physics and systems, plus 31 static models (walls, shelf rows, packing station, docks, fixtures, fence, floor paint). `sim.launch.py world:=warehouse` spawns each robot at its spawn pose from the location registry generated with it (warehouse_bringup README, "Where the robot spawns"). **Generated, do not edit:** change `warehouse_layout/config/layout.yaml` and run `ros2 run warehouse_layout generate` from the workspace root (warehouse_layout README, "Generated files") |
| `launch/gazebo.launch.py` | World scope: starts `gz sim` with a world (`world`, `headless` arguments) and bridges `/clock` once per world |
| `launch/spawn_robot.launch.py` | Robot scope: spawns a robot from its `robot_description` topic (`ros_gz_sim create`) and starts its bridge. Include it inside the robot's namespace |
| `config/robot_bridge.yaml` | The topics bridged per robot: `scan` (SENSOR_DATA QoS) and `ground_truth/odom` |
| `test/test_gazebo_config.py` | pytest without rendering (it runs in CI): the systems of every world, the bridged topics against the robot description, the caster friction after Gazebo's URDF conversion ([Tests](#tests)) |

## Systems in the world

A world that lists any `<plugin>` loads only those systems, so every one the robot relies on
is listed in `worlds/empty.sdf` (and in the generated `worlds/warehouse.sdf`):

| System | Why |
|---|---|
| `gz-sim-physics-system` | Rigid-body dynamics (DART), 1 ms steps |
| `gz-sim-user-commands-system` | The create/remove services that `ros_gz_sim create` uses to spawn the robot |
| `gz-sim-scene-broadcaster-system` | Publishes the scene for the GUI and the rendering sensors |
| `gz-sim-sensors-system` (`ogre2`) | Renders the `gpu_lidar` scanner |
| `gz-sim-imu-system` | Computes the IMU data |

The robot brings two more systems in its URDF (`hardware:=gz`): `gz_ros2_control-system`
(runs the controller manager inside Gazebo) and `gz-sim-odometry-publisher-system` (ground
truth).

## Topics

gz transport topics are global, so the robot description prefixes all of them with the robot
namespace (`/robot1/scan`, `/robot1/imu`, `/robot1/ground_truth/odom`, plan R12). The bridge
runs inside the robot's namespace with `expand_gz_topic_names: true`: it expands the relative gz
topic names of `config/robot_bridge.yaml` with its own namespace, exactly like ROS names, so
`scan` means `/robot1/scan` on both sides. One file therefore serves every robot.

| ROS topic (in `/robot1`) | gz topic | Type | QoS |
|---|---|---|---|
| `scan` | `/robot1/scan` | `sensor_msgs/LaserScan` | SENSOR_DATA (best effort, depth 5) |
| `ground_truth/odom` | `/robot1/ground_truth/odom` | `nav_msgs/Odometry` | reliable |
| `/clock` (no namespace) | `/clock` | `rosgraph_msgs/Clock` | reliable |

The IMU and the wheels are not bridged: they go through `gz_ros2_control`. Inspect the gz side
with `gz topic -l` and `gz topic -e -t /robot1/scan` (in the same `GZ_PARTITION` as the
simulation, if you set one).

## Why `gz sim` is started directly

`ros_gz_sim`'s `gz_sim.launch.py` starts `gz sim` through a shell (`shell=True`). A SIGINT from
`ros2 launch` then stops only the shell, and the Gazebo server keeps running in the background
with its controller manager, which confused every later run in the same ROS domain. Ctrl-C in a
terminal hides the problem, because the terminal signals the whole process group. So
`gazebo.launch.py` runs `gz sim` itself (no shell), and Gazebo receives the signal directly. The
plugin path Gazebo needs (`GZ_SIM_SYSTEM_PLUGIN_PATH`) comes from the ROS environment, set by
`gz_ros2_control`'s setup hook.

## Running

```bash
ros2 launch warehouse_gazebo gazebo.launch.py world:=empty            # server + GUI
ros2 launch warehouse_gazebo gazebo.launch.py world:=warehouse        # the warehouse
ros2 launch warehouse_gazebo gazebo.launch.py headless:=true          # server only
```

`headless:=true` runs only the server, with `--headless-rendering`: the scanner still renders
through EGL without a display (fine on an Intel iGPU; the CI runners have no GPU, so CI skips the
Gazebo tests). Set `GZ_PARTITION` to keep two simulations on one machine apart.

## Tests

CI cannot render the scanner, so it skips the Gazebo launch test (`warehouse_system_tests`).
Gazebo itself does not catch mistakes in these files early: `gz sdf -k` calls a world with a
misspelt system name `Valid.`, a bridge entry for a gz topic that nothing publishes simply
bridges nothing, and the bridge skips an entry with a misspelt ROS type after only a warning.
`test/test_gazebo_config.py` therefore reads the installed files, without starting Gazebo, and
checks that:

- every world in `worlds/` is valid SDF (`gz sdf -k`, which catches, for example, a visual and
  a collision with the same name in one link), is named after its file (`world:=<name>` picks
  the file, tools address the running world by name), steps physics every 1 ms in real time,
  and loads the five systems above, with their class names, rendering with `ogre2`;
- every gz topic in `config/robot_bridge.yaml`, expanded in `/robot1` the way the bridge expands
  it, is a topic the robot description publishes, with the message type it carries. For ground
  truth that includes loading `gz-sim-odometry-publisher-system` under its exact filename;
- each bridge entry uses the same relative name on the ROS side and the ROS type that matches
  its gz type (`sensor_msgs/msg/LaserScan` for a scan, not the `Range` the bridge would also
  accept), and `scan` uses SENSOR_DATA QoS;
- after Gazebo's own URDF-to-SDF conversion (`gz sdf -p`, the conversion that spawning does),
  both casters still have friction 0.

```bash
colcon test --packages-select warehouse_gazebo && colcon test-result --verbose
```

It runs in a few seconds and needs no GPU or display, so every pull request runs it.
