# warehouse_gazebo

Gazebo Harmonic side of the simulation: worlds, spawning a robot, and the bridge between gz
transport topics and ROS topics. Two worlds: `empty` (Phase 1) and `warehouse`, which
`warehouse_layout` generates from its `layout.yaml`. Gazebo runs with or without its window
(`gz_gui`) and, for one launch, with a 2 or 5 ms physics step (`physics_step`).

Normally you do not start these files yourself: `warehouse_bringup/launch/sim.launch.py`
includes them.

## Files

| File | What it does |
|---|---|
| `worlds/empty.sdf` | Ground plane, sun and the Gazebo systems the robot needs (below). `sim.launch.py` spawns the robot at the origin |
| `worlds/warehouse.sdf` | The 30 x 20 m warehouse: the same physics and systems, plus 31 static models (walls, shelf rows, packing station, docks, fixtures, fence, floor paint). `sim.launch.py world:=warehouse` spawns each robot at its spawn pose from the location registry generated with it (warehouse_bringup README, "Where the robot spawns"). **Generated, do not edit:** change `warehouse_layout/config/layout.yaml` and run `ros2 run warehouse_layout generate` from the workspace root (warehouse_layout README, "Generated files") |
| `launch/gazebo.launch.py` | World scope: starts `gz sim` with a world and bridges `/clock` once per world. Arguments: `world`, `headless` and `gz_gui` ([Gazebo without its window](#gazebo-without-its-window-gz_gui)), `physics_step` ([The physics step](#the-physics-step-physics_step)) |
| `launch/spawn_robot.launch.py` | Robot scope: spawns a robot from its `robot_description` topic (`ros_gz_sim create`) and starts its bridge. Include it inside the robot's namespace |
| `config/robot_bridge.yaml` | The topics bridged per robot: `scan` (SENSOR_DATA QoS) and `ground_truth/odom` |

## Systems in the world

A world that lists any `<plugin>` loads only those systems, so every one the robot relies on
is listed in `worlds/empty.sdf` (and in the generated `worlds/warehouse.sdf`):

| System | Why |
|---|---|
| `gz-sim-physics-system` | Rigid-body dynamics (DART), 1 ms steps (`physics_step` runs a copy of the world with 2 or 5 ms) |
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
ros2 launch warehouse_gazebo gazebo.launch.py world:=warehouse physics_step:=0.002   # 2 ms steps
```

`headless:=true` runs only the server, with `--headless-rendering`: the scanner still renders
through EGL without a display (fine on an Intel iGPU). Set `GZ_PARTITION` to keep two
simulations on one machine apart. A `world` that has no file in `worlds/` stops the launch
before Gazebo starts, with the list of worlds.

## Gazebo without its window (`gz_gui`)

`gz sim` with a window is three processes: the one launch starts, which forks `gz sim server`
(physics, sensors, and the controller manager inside it) and `gz sim gui` (the window). With `-s`
it runs only the server, in one process. `gz_gui:=false` and `headless:=true` both start it that
way, `gz sim -s --headless-rendering`. In this file the two mean the same; `sim.launch.py`
forwards both, and there `headless:=true` also turns RViz off while `gz_gui:=false` keeps it.

Why: the window draws the 3D scene all the time on the laptop's integrated GPU. With it, one robot
ran at a real-time factor of 0.92 instead of 1.00, the GPU 85 % busy; RViz costs about 0.7 of a
core and 2-3 % of the GPU (`docs/research/PC_ASSESSMENT.md`, section 5.7). Use the window for a
quick look, RViz for anything longer.

See it:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse gz_gui:=false
# in a second terminal:
pgrep -af "gz sim"    # one line: gz sim -s --headless-rendering -r -v 2 .../worlds/warehouse.sdf
pgrep -c rviz2        # 1
```

Without `gz_gui:=false`, `pgrep -af "gz sim"` lists three lines: `gz sim -r -v 2 ...`,
`gz sim server` and `gz sim gui`.

## The physics step (`physics_step`)

Gazebo moves the world forward in fixed steps of `<max_step_size>`: 1 ms in both worlds. After
every step it publishes `/clock`, so at 1 ms every node on sim time receives 1000 clock messages
a second, whether it needs them or not: an idle C++ node pays 0.045 of a core for that, a Python
node 0.27 (`docs/research/PC_ASSESSMENT.md`, section 5.4). A larger step means fewer steps and
fewer clock messages, but coarser physics. Phase 3 measures whether the robot still drives,
localizes and navigates as well at 2 or 5 ms (plan stage S4); the world files keep 1 ms until then.

`physics_step:=0.001`, `0.002` or `0.005` (seconds) runs one launch with that step: the world
file is copied, with the new `<max_step_size>` and its physics profile renamed (`2ms`), into the
launch's log directory (the one launch names at the start, "All log files can be found below
..."), and Gazebo loads the copy. The launch says so:

```text
[INFO] [launch.user]: Gazebo steps the physics every 2 ms (physics_step): it loads /home/<you>/.ros/log/2026-11-13-11-11-44-460715-<host>-106501/warehouse_2ms.sdf
```

**Only steps that divide 10 ms.** The controller manager (`update_rate: 100` in
`controllers.yaml`) and the IMU (100 Hz in `robot.yaml`) work every 10 ms. With a step that divides
10 ms, every control cycle and every IMU sample falls on a physics step; with 4 ms they would fall
between steps (10 ms is 2.5 steps), so neither could keep its 10 ms period exactly (ROS 2 skill,
`simulation.md` section 9, "Physics step size vs sensor update rate"). The lidar's 100 ms is a
multiple of all three. 2.5 ms and 10 ms divide 10 ms too, but Phase 3 compares only 1, 2 and 5 ms
(plan question Q5). Any other value stops the launch before Gazebo starts:

```text
[ERROR] [launch]: Caught exception in launch (see debug for traceback): physics_step:=0.004 is refused. Use 0.001, 0.002 or 0.005 (seconds): these divide the 10 ms period of the controller manager and the IMU, so every control cycle and every IMU sample falls on a physics step (10 ms would be 2.5 steps of 4 ms). Leave physics_step empty to keep the step in the world file.
```

**Measured** on 2026-11-13 with `warehouse_system_tests`' `measure_load`: the robot standing in
the warehouse, headless, 60 s per step, on mains with the balanced power profile. The step is what
Gazebo reported (the change of sim time over the change of its iteration count):

| `physics_step` | Step measured | RTF mean / worst 10 s | Gazebo server (cores) | Everything the launch runs (cores) |
|---|---|---|---|---|
| empty (the world's 1 ms) | 1.000 ms | 1.000 / 0.999 | 1.57 | 2.31 |
| `0.002` | 2.000 ms | 1.000 / 1.000 | 1.18 | 1.73 |
| `0.005` | 5.000 ms | 1.000 / 1.000 | 0.78 | 1.11 |

The cores depend on the CPU's clocks, which depend on the power profile. The PC assessment
measured Gazebo at 0.70 cores at 1 ms with the performance profile (P-cores at 3.2 GHz); these
runs had the balanced profile (P-cores 2.0-2.6 GHz, most of the work on the slower E-cores), and
every process cost 2-3 times as much, which points at the clocks rather than at Gazebo. The two
profiles have not been compared on one day, so compare steps under one profile.

See it:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse headless:=true physics_step:=0.005
# in a second terminal, after the robot has spawned:
ros2 run warehouse_system_tests measure_load --world warehouse --duration 30   # "physics step 5.000 ms"
```

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
