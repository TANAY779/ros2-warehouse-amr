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
through EGL without a display (fine on an Intel iGPU). Set `GZ_PARTITION` to keep two
simulations on one machine apart.

## Checking a change

Nothing checks these files automatically
([ADR-008](../../docs/decisions/ADR-008-develop-on-main-without-automated-tests.md)), and Gazebo
itself does not catch mistakes in them early: `gz sdf -k` calls a world with a misspelt system
name `Valid.`, a bridge entry for a gz topic that nothing publishes simply bridges nothing, and
the bridge skips an entry with a misspelt ROS type after only a warning. After editing a world
or `config/robot_bridge.yaml`, start the simulation and check that the data arrives:

```bash
ros2 launch warehouse_bringup sim.launch.py headless:=true
# in a second terminal:
ros2 topic hz /robot1/scan                                          # about 10 Hz
ros2 topic echo /robot1/ground_truth/odom --once --field pose.pose.position
```

and read the launch output for warnings from the bridge. In Gazebo's own conversion of the
URDF (`gz sdf -p`, the conversion that spawning does) both casters must keep friction 0.

**Gazebo sometimes hangs on shutdown.** About one run in 15 (measured in Phase 1), the Gazebo
server ignores SIGINT and SIGTERM after Ctrl-C, and launch kills it (exit code -9). This is the
upstream bug [gazebosim/gz-sim#4173](https://github.com/gazebosim/gz-sim/issues/4173) (Harmonic
with a `gpu_lidar`, headless): gz-common's signal handler stops the Sensors system from inside
an interrupted gz-transport publish, and the render thread deadlocks.
