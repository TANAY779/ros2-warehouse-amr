# warehouse_system_tests

The **test tools** of the simulation: `drive_route` drives a scripted route,
`evaluate_localization` measures AMCL (and, with `--odometry`, the odometry) against ground truth
from a bag, `map_timelapse` turns a mapping bag into a GIF, and `measure_load` measures what a
run costs the laptop and how fast Gazebo runs. The navigation benchmark adds three more:
`goal_set` writes a frozen set of goal poses, `nav_benchmark` drives them with Nav2 and records
the run, and `evaluate_navigation` judges that run from its bag. They work in simulation only and
steer on, or compare with, Gazebo's ground truth, so they are never part of the robot stack
(plan P2-D11).

The package keeps its name, but it holds no automated tests any more: its launch tests were
removed on 2026-11-07 with every other test of the project
([ADR-008](../../docs/decisions/ADR-008-develop-on-main-without-automated-tests.md)). A change
is checked by running the system and measuring it, with these tools and with the commands in
[Checking the robot by hand](../warehouse_bringup/README.md#checking-the-robot-by-hand).

## Files

| File | What it does |
|---|---|
| `scripts/drive_route`, `warehouse_system_tests/route_driver.py` | The `drive_route` command: the controller (pure Python) and the ROS node around it |
| `scripts/evaluate_localization`, `warehouse_system_tests/localization_eval.py` | The `evaluate_localization` command |
| `scripts/map_timelapse`, `warehouse_system_tests/map_timelapse.py` | The `map_timelapse` command |
| `scripts/measure_load`, `warehouse_system_tests/measure_load.py` | The `measure_load` command (no ROS node: it reads `/proc` and Gazebo's statistics through gz-transport) |
| `scripts/goal_set`, `warehouse_system_tests/goal_set.py` | The `goal_set` command, and the reader of goal set files |
| `config/goal_sets/warehouse_50.yaml` | The frozen goal set: 50 goal poses, written once by `goal_set` and committed |
| `scripts/nav_benchmark`, `warehouse_system_tests/nav_benchmark.py` | The `nav_benchmark` command: sends goals to Nav2, starts the recorder and `measure_load` |
| `scripts/evaluate_navigation`, `warehouse_system_tests/navigation_eval.py` | The `evaluate_navigation` command |
| `warehouse_system_tests/zones.py` | Distances between many points, or robot outlines, and the layout's zone polygons (numpy) |
| `warehouse_system_tests/bags.py` | Read and write rosbag2 bags (MCAP) with `rosbag2_py`, without a ROS graph |
| `warehouse_system_tests/ros_helpers.py` | Shared helpers: the controller's parameters from `controllers.yaml`, spinning until a condition holds, message stamps, durations and angles |

## Test tools

Run them in a second terminal while the simulation runs (`world:=warehouse`). Bags go into
`bags/` at the workspace root, which git ignores: they are large.

### drive_route

```bash
ros2 run warehouse_system_tests drive_route --route survey --speed 0.4 \
  --turn-rate 0.2                                                           # the mapping run
ros2 run warehouse_system_tests drive_route --route survey --speed 0.5 \
  --settle 30 --initial-pose-offset 0.3 -0.3 8                              # the AMCL evaluation run
ros2 run warehouse_system_tests drive_route --route none \
  --initial-pose-offset 0.3 -0.3 8                                          # then drive by teleop
ros2 run warehouse_system_tests drive_route --route smoke --dry-run           # print, touch nothing
```

It drives a route of the location registry (`routes` in
`warehouse_layout/generated/locations.yaml`, made from `layout.yaml`): `survey` (176 m, the
mapping run) or `smoke` (24 m, a quick check). At each waypoint the robot turns in place to
face along the next segment, then drives along it and stops on the next waypoint. It steers on
`/robot1/ground_truth/odom`, so every run follows the same path whatever SLAM or AMCL believe.

| Option | Meaning |
|---|---|
| `--route NAME` | `survey`, `smoke`, or `none` (drive nothing). Required |
| `--speed M_PER_S` | Cruise speed, default 0.4 m/s; at most the controller's 1.2 m/s |
| `--turn-rate RAD_PER_S` | Yaw rate of the turns in place, default 0.5 rad/s |
| `--settle SECONDS` | Stand still that long (sim time) before driving: the IMU noise check needs at least 25 s standing still |
| `--initial-pose-offset DX DY DYAW_DEG` | Once AMCL is active (its lifecycle state), publish a pose on `/robot1/initialpose`: the true pose moved by DX, DY metres (map frame) and turned by DYAW_DEG degrees, covariance 0.5² m² and 0.2² rad². AMCL then starts from a wrong pose and must find the right one. The pose goes out once the publisher is 3 s old, and again (up to 3 times) until AMCL answers on `amcl_pose` |
| `--namespace NS` | The robot, default `robot1` |
| `--dry-run` | Print the waypoints, the length and the expected time (the route driven by an ideal robot), and exit without touching the ROS graph |

**Commands at 20 Hz, stamped in sim time.** `diff_drive_controller` brakes when its newest
command is older than `cmd_vel_timeout` (`controllers.yaml`), and it measures that age from the
command's header stamp, in sim time. So the driver publishes `TwistStamped` on
`/robot1/cmd_vel` every 0.05 s while driving, also when the command does not change, stamped
with the node's sim-time clock. A wall-clock stamp would be wrong in simulation (plan R1). The
publisher is reliable and keeps only the newest command (depth 1, the command velocity QoS of
the ROS 2 engineering skill): a queued old command is worse than a lost one. The driver logs this
QoS when it starts to drive, because `ros2 topic info -v` cannot show the depth: discovery does
not carry the history setting, so it prints `History (Depth): UNKNOWN`.

**The first command only once the controller listens.** DDS delivers messages only between a
publisher and a subscription that discovery has matched, a fraction of a second after the
publisher is created, and it matches the two ends separately. A command that reaches the
controller before its end knows the publisher is dropped and sent again later, by then too old
for `cmd_vel_timeout`: in one mapping run the first four commands were ignored that way and the
robot started late. So the driver waits until ROS lists `diff_drive_controller`'s subscription
to `/robot1/cmd_vel` and its publisher is matched with every subscription listed (a count above
zero is not enough: a bag recorder that records `/robot1/cmd_vel` may be matched first). The
controller's end cannot be checked from the driver, so the first command also waits until the
driver has run for 3 s (`DISCOVERY_SETTLE`). A subscription whose QoS does not fit the
publisher's is never matched, for example `ros2 topic echo /robot1/cmd_vel --qos-durability
transient_local` (it asks for old messages, which a volatile publisher does not keep): the
driver then gives up after 30 s, and its error names that subscription and the reason.

**The initial pose, the same way.** AMCL subscribes to `initialpose` best effort (the system
default QoS), so a pose that reaches AMCL before its end of the match knows the publisher is
dropped, and DDS never sends it again; AMCL would then stay at the true pose, and the
evaluation would report a convergence that never happened. So the driver publishes the pose
once its publisher is 3 s old, and checks the answer: AMCL publishes `amcl_pose` after every
initial pose, and a robot standing still gives it no other reason to. Without an answer within
5 s it publishes again, up to 3 times, then exits with 1. AMCL logs one warning for the pose,
`Failed to transform initial pose in time (Lookup would require extrapolation into the
future ...)`: the pose carries the stamp of the ground truth it was made from, about 12-20 ms
newer than the EKF's newest transform, so AMCL cannot add the motion since then and adds none.
The robot stands still, so none is right.

**Within the controller's limits.** Speed and turn rate are checked against `controllers.yaml`.
Commands change by at most half the controller's acceleration limits, so the controller never
has to clip them, and stops are planned with half of that braking again: a robot that lags
behind its commands still stops on the waypoint. On a simulated robot that sees its pose and
acts 0.1 s late, the driver stays within 1 cm of the route at 1.2 m/s; without the braking
reserve it left the route by 0.28 m.

**Exit codes:** 0 the route was driven; 1 it could not be (no ground truth within 30 s, AMCL
not active within 60 s or not answering the initial pose, the robot more than 0.5 m from the
route's start, `diff_drive_controller` not receiving the commands within 30 s, no progress for
10 s, or the simulation clock stopped);
2 a bad argument; 130 stopped by Ctrl-C. Before every exit
the robot is commanded to stop. If the driver dies anyway, the controller's timeout stops the
robot: that is what the timeout is for.

**Measured in Gazebo** (2026-11-07 and 2026-11-08): the survey takes 9 min 40 s of sim time at
0.4 m/s with turns at 0.2 rad/s (175.6 m), and 7 min 12 s at 0.5 m/s with the default turns; the
smoke route 1 min 11-14 s at 0.5 m/s. Commands went out at 20.00 Hz, and in 13 logged drives the
robot stopped 0.02-0.03 m from the last waypoint. AMCL answered the initial pose 0.07-0.13 s
after it was sent. The driver uses about 0.4 of a CPU core: it is a Python node on sim time,
and `/clock` ticks 1000 times a second. The driver acts on Ctrl-C between its callbacks, within
0.05 s, and ignores a second Ctrl-C while it is stopping the robot, so the stop (two zero
commands over 0.2 s) always finishes.

### evaluate_localization

```bash
# 1. With sim.launch.py world:=warehouse localization:=amcl running, start recording:
ros2 bag record -o bags/loc_run --use-sim-time --topics /robot1/amcl_pose /robot1/initialpose \
  /robot1/ground_truth/odom /robot1/odom /robot1/odom/wheel /robot1/imu /robot1/tf \
  /robot1/tf_static /clock
# 2. In another terminal, the evaluation run (about 7 min, plus 30 s standing still):
ros2 run warehouse_system_tests drive_route --route survey --speed 0.5 --settle 30 \
  --initial-pose-offset 0.3 -0.3 8
# 3. Stop the recorder (Ctrl-C), then:
ros2 run warehouse_system_tests evaluate_localization bags/loc_run --plot /tmp/loc.png
ros2 run warehouse_system_tests evaluate_localization bags/loc_run --odometry   # + odometry
```

What it computes, from header stamps (sim time) only:

- **AMCL's error** at the stamp of every `amcl_pose`, against ground truth interpolated to that
  stamp: position and heading. (Not from TF: AMCL dates its `map -> odom` transform into the
  future by `transform_tolerance`, plan P2-D12.)
- **The start** is the newest `initialpose` in the bag (without one, the first `amcl_pose`).
  **Convergence** is the first pose after which the error stays below 0.10 m to the end; the
  **convergence distance** is the ground-truth path length from the start to it.
- **After convergence:** mean, RMS, 95th percentile and maximum position error, the largest
  heading error, how long and how far the robot drove.
- **The EKF's drift** (`/robot1/odom`, put on the ground truth at the start), for comparison:
  dead reckoning drifts with distance, AMCL should not.
- **IMU noise:** sigma per axis (gyro x, y, z; accelerometer x, y, z about their means) from
  periods in which ground truth stands still for at least 10 s, without the first second of
  each. Samples identical to the one before are removed and counted (ros2_control can read one
  Gazebo sample twice). The messages' covariances are compared with the measured sigma².

Then a `[PASS]` or `[FAIL]` line per acceptance criterion of plan F7: convergence within 2 m;
error under 0.10 m for at least 5 min after convergence; gyro and accelerometer sigma, each
axis, within ±20 % of `robot.yaml`, from at least 25 s standing still. `--plot` draws the paths
from above (ground truth, AMCL, the EKF) and, over time, AMCL's position and heading error and
the EKF's drift.

**Exit codes:** 0 every criterion passes; 1 one fails; 2 the bag cannot be read, has no
`amcl_pose` or ground truth, or the plot cannot be written.

Start the recorder after the launch, as above: it then takes each publisher's QoS. A recorder
that subscribes before it has discovered the publisher (started before the launch, or with
`--include-unpublished-topics`, even after it) uses volatile QoS and records none of the one
transient-local `tf_static` message. The evaluation does
not need `tf_static`, but a replay of the bag through AMCL does.

**Measured** on the acceptance run of 2026-11-08 (the survey at 0.5 m/s after 30 s standing
still, started 0.3 m, -0.3 m and 8° off): AMCL converged after 0.78 m and stayed within 0.092 m
of ground truth for 7 min 07 s (mean 0.024 m, P95 0.052 m); the EKF drifted 0.058 m over
175.6 m; the IMU's σ was 0.99-1.03 times `robot.yaml` on every axis, from 40.6 s standing still
with 506 repeated samples removed; 4 of 4 checks passed. The full output is in the
[Phase 2 report](../../docs/phase-reports/phase-02-warehouse-world.md#exercise-8-measure-localization-from-a-bag).

#### Odometry (`--odometry`)

`--odometry` adds Phase 1's odometry checks, run on the same bag: wheel odometry
(`/robot1/odom/wheel`, from `diff_drive_controller`) and the EKF (`/robot1/odom`) against ground
truth, on every motion of the robot. Phase 1 drove 10 m and turned 360° just for this check; now
any recorded drive gives the numbers, which is what plan stage S4 needs to compare physics steps.

- **The motions** come from ground truth's speed and turn rate. Faster than 0.02 m/s is
  driving; slower, but turning faster than 0.02 rad/s, is turning in place; anything else is
  standing still. A motion runs from the last ground-truth pose before it to the first one after
  it. A drive that turns by more than 10° is a curve, measured but not judged.
- **A straight's error**: each source's motion over the straight, taken in its own heading at the
  start (forward and left), minus the true motion, as a share of the distance. Taking each source
  in its own start heading cancels where its `odom` frame lies in the world.
- **A turn's error**: the angle the source turned minus the true angle, as a share of the angle.
- **Judged** with Phase 1's limits: under 1 % on every straight of 10 m or more, under 2 % on
  every turn in place of 80° or more. Shorter straights and smaller turns are listed with "not
  judged". The turns are judged from 80° rather than 90° because `drive_route` stops a turn in
  place a few degrees early and turns the rest while it drives: its 90° corners turn 85.8° in place.
- **A bag without `amcl_pose`** (`localization:=none`) can be evaluated with `--odometry`: the
  AMCL part, and the IMU part if the bag has no `/robot1/imu`, say "not evaluated" and add no
  checks. `--plot` still needs `amcl_pose`.

Four more `[PASS]`/`[FAIL]` lines follow the others: straights and turns, for wheel odometry and
for the EKF. Part of the output for the Phase 2 survey bag (`loc_run`, recorded 2026-11-08):

```text
Odometry against ground truth (/robot1/ground_truth/odom), per motion: the error of wheel odometry (/robot1/odom/wheel) and of the EKF (/robot1/odom), as a share of the distance driven or the angle turned:
    #  motion                sim time [s]   ground truth      wheel        EKF
    1  turn in place     45.58 -    47.80      +43.3 deg    0.012 %    0.012 %   (under 80 deg: not judged)
    2  straight          47.78 -    53.16       2.09 m      0.011 %    0.147 %   (under 10 m: not judged)
    3  turn in place     53.14 -    55.26      -41.4 deg    0.021 %    0.084 %   (under 80 deg: not judged)
    4  straight          55.24 -    93.46      18.49 m      0.002 %    0.168 %
    5  turn in place     93.44 -    97.10      +85.8 deg    0.005 %    0.088 %
...
[PASS] wheel odometry (/robot1/odom/wheel) under 1 % on every straight of 10 m or more: worst 0.004 % (#26, 15.07 m) on 9 straights
[PASS] wheel odometry (/robot1/odom/wheel) under 2 % on every turn in place of 80 deg or more: worst 0.005 % (#5, +85.8 deg) on 15 turns
[PASS] EKF odometry (/robot1/odom) under 1 % on every straight of 10 m or more: worst 0.258 % (#10, 15.07 m) on 9 straights
[PASS] EKF odometry (/robot1/odom) under 2 % on every turn in place of 80 deg or more: worst 0.149 % (#27, +85.8 deg) on 15 turns
8 of 8 localization checks pass
```

The EKF's errors are larger than the wheel odometry's; both stay far inside the limits, as in
Phase 1 (wheels 0.01 %, EKF 0.09-0.42 % over 10 m). Use the survey route: the smoke route's
longest straight is 8.7 m, so on a smoke bag the two straight checks fail with "no straight of
10 m or more in the bag" (measured 2026-11-13: 10 motions listed, the three 90° corners judged,
wheel at most 0.009 % and EKF at most 0.079 %).

### map_timelapse

```bash
# While slam_toolbox maps (localization:=slam) and drive_route drives the survey:
ros2 bag record -o bags/mapping --use-sim-time --topics /map /robot1/ground_truth/odom /clock
# Afterwards:
ros2 run warehouse_system_tests map_timelapse bags/mapping docs/media/phase-02-mapping.gif
```

Each frame shows the newest `/map` at its moment (unknown grey, free white, occupied black, as
in RViz), the ground-truth path so far in blue and the robot in orange; frames are evenly
spaced in sim time, and the last one stays for 2 s. Options: `--frames` (default 60),
`--width` in pixels (640), `--frame-ms` (100), `--max-mb` (3, the repository's limit for
`docs/media`), `--namespace`, `--map-topic`. All frames share a six-colour palette and Pillow
stores only what changed from one frame to the next, so the file stays small: the Phase 2 GIF,
`docs/media/phase-02-mapping.gif`, is 60 frames of 640 x 452 px in 0.18 MB, made in about 3 s
from the 218 MiB bag of the survey mapping run (10 min 08 s of mapping); a smoke-route mapping
gives 0.11 MB. The same bag always gives the same bytes. If a GIF would still be larger than
`--max-mb`, the command keeps every second frame, then narrows the frames, until it fits; exit
code 1 if it cannot.

### measure_load

```bash
# With the simulation running, in a second terminal set up like the first:
ros2 run warehouse_system_tests measure_load --world warehouse --duration 60
# Keep the samples of a run (it refuses a folder that already has them):
ros2 run warehouse_system_tests measure_load --world warehouse --duration 120 --out bags/s4_2ms
```

It measures what a simulation run costs this laptop and how fast Gazebo runs: a sample every
2 s (`--interval`) for `--duration` seconds (default 60), a line per sample (`--quiet` hides them),
then a summary table. `--duration 0` samples until Ctrl-C or SIGTERM, which is then the normal
end and not an early stop: `nav_benchmark` runs it that way for as long as a benchmark lasts. It
is the sampler of the
[PC assessment](../../docs/research/PC_ASSESSMENT.md) made into a project tool (its
recommendation 9).

It is not a ROS node, on purpose. A node on sim time receives every `/clock` message, and Gazebo
publishes one per physics step: 1000 a second at the 1 ms step. An idle Python node pays about
0.27 of a core just for that, a C++ node 0.045 (PC assessment, section 5.4). A measuring tool
that cost that much would measure itself. It reads Gazebo's statistics through gz-transport, the
messaging Gazebo itself uses, with its Python bindings `gz.transport13` and `gz.msgs10` (from ROS
2 Jazzy's vendor packages), and everything else from `/proc` and `/sys`.

What each sample holds:

- **The real-time factor (RTF) and the physics step**, from Gazebo's `/world/<world>/stats`
  (about 5 messages a second): sim time, real time and Gazebo's iteration count. The RTF is the
  change of sim time divided by the change of real time: 1.00 means the simulation keeps up with
  the wall clock, 0.90 that it runs 10 % slow, so every sim-time timer and sensor runs 10 % slow
  in wall time too. The physics step is the change of sim time divided by the change of
  iterations: what Gazebo really stepped, so it checks `physics_step:=`. The summary gives the
  mean RTF, the worst 10 s and the lowest single sample.
- **CPU per process group**, in cores: CPU seconds per second, so 1.0 is one of the laptop's 12
  threads fully busy. From `/proc/<pid>/stat` (utime + stime, the scheduler's count for the whole
  process). A process counts only if its `ROS_DOMAIN_ID` and `GZ_PARTITION` equal the tool's, so
  a simulation in another domain lands in "rest of the machine". A process that exits between two
  samples loses its last interval (the controller spawners, for example).
- **The machine's total**, from `/proc/schedstat` (the nanoseconds each CPU ran tasks), split
  into the P-cores and E-cores of this hybrid CPU. Not `/proc/stat`: it samples busy time on the
  timer tick, and Gazebo's 1 kHz loop beats against it.
- **Memory**: MemAvailable, the swap in use and each group's PSS (a shared library counts once,
  split between the processes that use it).
- **Clocks, heat and power**: the mean clock of the P- and E-cores, the package temperature,
  mains or battery, and the CPU's energy-performance preference, which the power profile sets.

| Group | Processes |
|---|---|
| Gazebo server | `gz sim -s ...`, or `gz sim server` and its parent when the window runs |
| Gazebo window | `gz sim gui` |
| ros_gz bridges | `parameter_bridge` (the `/clock` bridge, the robot's bridge), `image_bridge` |
| nav2_container, filter_container | the component containers of that name (Phase 3) |
| EKF, robot_state_publisher, twist_mux | those nodes |
| AMCL, map server, SLAM | `amcl`, `map_server`, their lifecycle managers, slam_toolbox, as plain processes |
| RViz | `rviz2` |
| Python tools | `drive_route`, the teleop window and the other Python programs of the project, `teleop_twist_keyboard` |
| bag recorder | `ros2 bag record` |
| ros2 launch | the launch process |
| other ROS processes | anything else with `--ros-args` (controller spawners, `ros_gz_sim create`), other `ros2` commands, the ros2 daemon |
| measure_load (this tool) | listed apart from the run total |

The summary for a smoke-route drive, measured 2026-11-13 (robot1 driving the route with AMCL and
a bag recorder, 1 ms step, no windows):

```text
measure_load: 120 s from 2026-11-13 11:16:28, 60 samples every 2 s; ROS_DOMAIN_ID 69, GZ_PARTITION p3s1_agent
Gazebo world warehouse (/world/warehouse/stats): RTF 0.998 mean, 0.995 worst 10 s, 0.986 lowest 2 s; physics step 1.000 ms (change of sim time / change of iterations)

group                        processes  cores mean  cores max  PSS MiB
Gazebo server                        1        1.39       1.56      471
ros_gz bridges                       2        0.36       0.42       46
EKF                                  1        0.18       0.21       19
robot_state_publisher                1        0.10       0.14       14
AMCL, map server, SLAM               4        0.40       0.48       73
Python tools                         2        0.59       0.76       62
bag recorder                         1        0.13       0.15       50
ros2 launch                          1        0.00       0.01       29
run total                                     3.15       3.65      761
measure_load (this tool)             2        0.04       0.06       34
rest of the machine                           0.81       2.60
machine total                                 3.99       6.14   (P-cores 1.25, E-cores 2.74)

memory: MemAvailable lowest 4432 MiB; swap in use at most 2632 MiB
clocks: P-cores 2.5 GHz, E-cores 2.0 GHz (mean); package 70 °C mean, 87 °C max
power: on mains; energy-performance preference balance_performance
```

With `--out DIR` it also writes `DIR/load.jsonl` (one JSON object per sample: every number
above, per group), `DIR/load_summary.json` and `DIR/load_summary.txt` (the table).

**What voids a run** (the run rules), printed as `WARNING` lines and with exit code 1: no
statistics from Gazebo (it is not running, `--world` names another world, or `GZ_PARTITION`
differs); MemAvailable below 1.5 GiB (stop the run); a suspend during the run (the boot clock
runs on while the laptop sleeps, the monotonic clock does not, and the tool compares the two).

**Exit codes:** 0 the duration was sampled with Gazebo's statistics; 1 a warning above; 2 a bad
argument or `--out` cannot be written; 130 stopped early by Ctrl-C or SIGTERM (the summary covers
the samples taken). With `--duration 0` a Ctrl-C or SIGTERM ends the run as planned: 0 or 1.

**Cores depend on the clocks.** A core-second is a second of CPU time at whatever clock the core
ran, so the same simulation costs more cores on slower clocks, and the power profile sets the
clocks. On 2026-11-13, with the balanced profile, the robot standing at 1 ms headless cost 2.31
cores, Gazebo 1.57 of them. The same run under the performance profile cost 1.30 cores, Gazebo
0.90 of them, with the P-cores at 3.6 GHz instead of 2.5 (the PC assessment measured 1.0 and
0.70). So the profile, not the simulation, explains the difference.
Compare runs only under the same profile (the `power` line; `powerprofilesctl get` shows it). The
RTF and the physics step do not depend on it as long as the laptop keeps up. Measured per step:
[warehouse_gazebo README](../warehouse_gazebo/README.md#the-physics-step-physics_step).

See it:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse gz_gui:=false
# in a second terminal, once the robot has spawned:
ros2 run warehouse_system_tests measure_load --world warehouse --duration 60
```

## The navigation benchmark

Three tools measure how well Nav2 drives, on the same goals every time (plan stage S3b):

1. `goal_set` writes the **frozen goal set**: 50 goal poses, drawn once with a fixed seed from
   the layout and the saved map, and committed as `config/goal_sets/warehouse_50.yaml`.
2. `nav_benchmark` sends goals of that set to Nav2 one after the other, records the run into a
   bag and runs `measure_load` alongside.
3. `evaluate_navigation` reads the bag and prints, per goal, the result, the time, the path, the
   end error and how long the collision monitor held the robot; for the whole run, the command
   age the base holds, the events of the log, the zones and the load; and a `[PASS]` or `[FAIL]`
   line for each limit of the Phase 3a acceptance.

See it:

```bash
# Terminal 1: the simulation with Nav2, without windows
ros2 launch warehouse_bringup sim.launch.py world:=warehouse navigation:=true gz_gui:=false rviz:=false
# Terminal 2, once Nav2 is up and the robot stands at its spawn: three goals (about 2 min)
ros2 run warehouse_system_tests nav_benchmark --goals 3
# nav_benchmark ends by printing the command that judges its run, for example:
ros2 run warehouse_system_tests evaluate_navigation bags/nav_benchmark_2026_11_17-10_00_00 --plot /tmp/nav.png
```

### goal_set

```bash
# From the workspace root:
ros2 run warehouse_system_tests goal_set --count 50 --seed 1           # the committed set
ros2 run warehouse_system_tests goal_set --seed 2 --out /tmp/try.yaml  # another seed, elsewhere
```

It reads the source tree, as `warehouse_layout generate` does: the layout
(`src/warehouse_layout/config/layout.yaml`), the robot (`src/warehouse_description/config/robot.yaml`)
and the committed SLAM map (`src/warehouse_navigation/maps/warehouse.yaml` and its image). It
writes `src/warehouse_system_tests/config/goal_sets/<world>_<count>.yaml`, or the file given with
`--out`; `--src` points at another source tree. The file is rewritten only when its text changes,
so a second run prints `unchanged`. Build the package afterwards: `nav_benchmark` reads the
installed copy.

**The rules.** Every goal:

1. Has its **goal circle free**: within 0.556 m of the centre, every cell is free, in the SLAM map
   and in `rasterize`'s ideal map of the layout. 0.556 m is the layout's goal clearance, the padded
   circumscribed radius 0.456 m plus 0.10 m: the robot can turn in place there. The distance is
   exact, from the centre to the nearest point of any occupied or unknown cell.
2. Lies at least **0.95 m from every keepout zone's true polygon** in `layout.yaml`: outside the
   planning core (0.65 m) and its cost ramp (0.30 m) of the keepout mask that Phase 3b adds (plan
   section 3.3). No goal lies next to an invisible dock bar.
3. Lies in the **free space of the robot's centre that is connected to the spawn** of robot1, in
   both maps: cells more than 0.28 m (the padded inscribed radius) from every occupied or unknown
   cell, which is how Nav2's planner sees free space, without the 0.65 m cores round the keepout
   zones. So the masks of Phase 3b cannot cut a goal off.
4. Lies at least **3 m from the goal before it** (goal 1: from the spawn), so each goal is a
   real drive.
5. Has a **yaw drawn uniformly** over a full turn.

The goals come from the layout and the map, never from the masks, so the masks of Phase 3b do not
change the set (plan P3-D22).

**Deterministic.** The candidates come from Python's `random.Random(seed).random()`, the one random
stream Python promises never to change for a given seed. x and y are drawn uniformly over the hall
floor and rounded to 1 mm, the yaw rounded to 0.0001 rad, before the rules check them, so the file
holds exactly the values that were checked. The text is written with fixed decimals and holds no
date. The same inputs and seed give the same bytes. The header states the rules, the seed and the
four input files with their sha256, so a changed map or layout shows there. Do not write the set
again to suit a run, because runs compare only on the same goals: if an input changes, write a copy
with `--out`, compare, and keep the committed set unless the owner decides otherwise.

**Which goals each stage drives** (plan S3b): S3b goals 1-10 (the baseline below), S4 and S6b
1-20, S8b 21-40, and the acceptance runs (3a-runs and S9) all 50. Goals 21-50 are first driven
in 3a-runs, and no Phase 3b stage drives goals 41-50 before S9, so each part's acceptance includes
goals that part was not tuned on.

**The report** lists each goal with its pose, its distance from the goal before, its clearance in
the SLAM map and in the ideal map (the distance from the centre to the nearest occupied or unknown
cell) and the nearest keepout zone, then the tightest values. The same numbers are comments in
the file. For the committed set (2026-11-17): 50 goals from 120 candidates (8 turned down for the
3 m step, 26 near a keepout zone, 36 for the goal circle in the SLAM map); the tightest goal is
0.580 m clear in the SLAM map and 0.589 m in the ideal map, 1.03 m from a keepout zone and 3.41 m
from the goal before; the free space connected to the spawn is one region of 375 m² in the SLAM
map. Two runs wrote the same bytes, sha256 `8df04ceca348…`.

### nav_benchmark

```bash
ros2 run warehouse_system_tests nav_benchmark --goals 10 --out bags/s3b_baseline    # goals 1-10
ros2 run warehouse_system_tests nav_benchmark --from 21 --goals 20 --out bags/s8b   # goals 21-40
```

| Option | Meaning |
|---|---|
| `--goals N` | How many goals to send. Required |
| `--from K` | The index of the first goal in the set (default 1): goals K to K+N-1 |
| `--out DIR` | The bag folder to create; it must not exist yet. Default `bags/nav_benchmark_<date>-<time>` in the current folder |
| `--goal-set FILE` | Another goal set (default: the installed `warehouse_50.yaml`) |
| `--with-scans` | Also record `/robot1/scan` (10 Hz, 720 rays): the bag then grows faster, about 15 MB a minute by the plan's estimate, against the 9 MB measured without |
| `--namespace NS` | The robot, default `robot1` |

Start it with the simulation running with `navigation:=true`, normally right after the launch with
the robot at its spawn. It does this, in order:

1. **Waits for Nav2 the way `warehouse_ops navigate` does**: AMCL active and its `amcl_pose`
   received, `bt_navigator` active, the `navigate_to_pose` action server there; then
   `BasicNavigator.waitUntilNav2Active(localizer='robot_localization')`. With its default localizer
   `BasicNavigator` would publish an initial pose at the origin; `nav_benchmark` never publishes on
   `initialpose`.
2. **Reads the configuration** the run uses from the running nodes (the collision monitor's
   velocity bands, the velocity smoother, the progress and goal checkers) into `results.json`.
3. **Starts the recorder** (below) and waits until it has subscribed to every listed topic that has
   a publisher, then 3 s more, because DDS matches the publisher's side of a connection a moment
   later (`drive_route`'s first command waits the same way).
4. **Starts `measure_load --duration 0`** into the bag folder.
5. **Sends the goals**, one `NavigateToPose` goal at a time, each once the one before has ended,
   whatever its result, with 1 s between them. A goal still running after its **timeout, 60 s plus
   4 s per metre of straight distance** from where the robot stands to the goal (in sim time), is
   cancelled, counted as `TIMEOUT`, and the next goal follows. It prints a line every 5 s and each
   goal's result, with the ground-truth error at its end.
6. **Stops `measure_load` and the recorder** 2 s after the last goal, writes `results.json` and
   prints a table and the command for `evaluate_navigation`.

**Recording rules** (plan section 3.6). The recorder is `ros2 bag record`, started:

- **after Nav2 is active and before the robot moves**: it then takes each publisher's QoS and gets
  the one latched `tf_static` message (a recorder that subscribes before a publisher exists uses
  volatile QoS, see `evaluate_localization` above);
- **with `--use-sim-time`**: the bag stores the sim time at which each message arrived, the clock
  `diff_drive_controller` judges command age by;
- **without `/clock`** (1000 messages a second) **and without scans** unless `--with-scans`.

| Topics (in `/robot1`, plus `/rosout`) | What they are for |
|---|---|
| `ground_truth/odom` | The truth: paths, end errors, where the robot stood during a hold, zones |
| `odom`, `odom/wheel`, `amcl_pose` | `evaluate_localization` on the same bag: AMCL during the goals, odometry |
| `cmd_vel_nav`, `cmd_vel_smoothed`, `cmd_vel` | The velocity chain: command age, the monitor's delay and holds |
| `cmd_vel_mux`, `cmd_vel_teleop` | The same chain once twist_mux arrives (stage S5) |
| `collision_monitor_state` | The monitor's state, published when it changes |
| `speed_limit` | The speed zones' limit (stage S6b) |
| `plan` | Nav2's paths |
| `tf`, `tf_static` | A replay of the run |
| `navigate_to_pose/_action/status` | When each goal ran and how it ended, in the bag itself. It is a hidden topic (a name part starts with `_`), so the recorder gets `--include-hidden-topics`; with an explicit list that adds nothing else |
| `/rosout` | The log: the events of `evaluate_navigation` |

Topics that do not exist yet (`cmd_vel_mux` and `cmd_vel_teleop` before stage S5, `speed_limit`
before S6b) stay in the list: the recorder subscribes to a listed topic once it appears, and
`nav_benchmark` names the ones without a publisher. Without scans the bag grows by about 9 MB a
minute (the ten goals of the baseline below: 45 MB).

**The run folder** is the bag folder itself. rosbag2 reads only the files its `metadata.yaml`
lists, so everything else of the run sits next to the MCAP file:

| File | What it holds |
|---|---|
| `<name>_0.mcap`, `metadata.yaml` | The bag (`ros2 bag info DIR`). Its metadata also names the goal set and the goals sent (`custom_data`) |
| `results.json` | Per goal: its index and pose, the result (`SUCCEEDED`, `FAILED`, `CANCELED`, `TIMEOUT`, `REJECTED`, `INTERRUPTED`), Nav2's error code and name, the time in sim time and in wall time, the recoveries (Nav2's own count, from the feedback), the straight distance and the timeout, the goal ID, and ground truth at the start and the end with the end error. Also the run's configuration from step 2 and the goal set's sha256. Written after every goal, so a run that stops early keeps its results |
| `load.jsonl`, `load_summary.json`, `load_summary.txt` | `measure_load`'s samples and summary |
| `recorder.log`, `measure_load.log` | The console output of the two |

**Clocks.** `nav_benchmark` runs on the wall clock and reads sim time from the stamps of the ground
truth (`/robot1/ground_truth/odom`, 50 Hz): a Python node on sim time receives every `/clock`
message, 1000 a second at the 1 ms step, which costs it about 0.27 of a core (see `measure_load`
above). Goal times and timeouts are in sim time, so they do not depend on how fast Gazebo runs.
Without new ground truth for 10 s of wall time the simulation has stopped, and the run ends with
exit code 1.

**The recorder and `measure_load` run in sessions of their own.** A Ctrl-C in the terminal reaches
only `nav_benchmark`, which stops the two itself, with SIGTERM (which both handle as a clean stop),
after the robot has stopped, so the bag shows the end of the run. **Ctrl-C** (or SIGTERM) cancels
the active goal, waits until Nav2 confirms it and ground truth shows the robot standing (at most
3 s), records the goal as `INTERRUPTED`, stops `measure_load` and the recorder, writes
`results.json` and exits with 130; a second Ctrl-C stops waiting for Nav2 and the robot, but the
two are still stopped, so the bag stays readable. A goal cancelled after its timeout also waits
for the robot to stand before the next goal starts. Measured: a Ctrl-C during a goal at
1.0 m/s ended the tool 1.8 s later with exit code 130; Nav2 confirmed the cancel 24 ms after the
request, the bag ends 1.2 s after it with the robot standing, both children had exited, and the
bag read back.

**Exit codes:** 0 the run completed, whatever the goals did; 1 it could not run (Nav2 not ready
within 60 s, the recorder or `measure_load` did not start, the simulation stopped, or Nav2 did not
end a cancelled goal); 2 a usage error (a bad option, an unreadable goal set, goals outside the
set, or `--out` exists already); 130 Ctrl-C.

### evaluate_navigation

```bash
ros2 run warehouse_system_tests evaluate_navigation bags/s3b_baseline --plot /tmp/nav.png
```

It reads the bag folder that `nav_benchmark` wrote, and `results.json` and `measure_load`'s summary
next to it when they are there. Times are the bag's receive times, which are sim time. Each goal's
**window** runs from the first status `EXECUTING` (or `ACCEPTED`) of its goal ID on the action's
status topic to its first final status (`SUCCEEDED`, `ABORTED`, `CANCELED`); without the status
topic, from `results.json`'s times. Without `results.json`, the goals and results come from the
status topic alone, and the goal indices, poses and error codes are unknown.

| Metric | How it is measured | Limit (Phase 3a) |
|---|---|---|
| Goals | Results per goal; `TIMEOUT` and `INTERRUPTED` count as not succeeded | `[PASS]` if at least 96 % of the goals sent succeed (the acceptance: 48 of 50) |
| Held command age (plan P3-D33) | For consecutive messages i and i+1 on `cmd_vel`: the receive time of i+1 minus the stamp of i, judged while message i is not zero. It is how old the newest command is just before the next one replaces it: the base brakes when that age passes `cmd_vel_timeout`, 0.15 s | `[PASS]` if under 0.12 s for 99.9 % and never over 0.15 s |
| Base timeouts | `diff_drive_controller`'s `Velocity command timed out. Braking.` while the newest command on `cmd_vel` is not zero. The base also logs it once a second while it stands without commands: those are counted apart, not judged | `[PASS]` if none |
| Real-time factor | `measure_load`'s mean | `[PASS]` if at least 0.95 |
| The run rules | `measure_load`'s warnings: no Gazebo statistics, a suspend, MemAvailable below 1.5 GiB | `[PASS]` if none (plan section 3.6) |
| Time, path, recoveries, end error | Per goal: the window's length; the ground-truth path length; Nav2's recovery count; ground truth at the window's end against the goal pose | Information |
| The monitor's delay | The receive time on `cmd_vel` minus that on the monitor's input for the same stamp (the monitor copies the header). The input is `cmd_vel_mux` from stage S5; until then it is `cmd_vel_smoothed`, and the report says so | Information |
| Monitor holds | Below | Information (the stall fix is judged on it) |
| Events from `/rosout` | Per goal: `Ignoring the received message` (a command already older than the timeout when it arrived), `Control loop missed its desired rate`, `KeepoutFilter: Pose is in keepout zone`, `Start occupied`, `Velocity is not covered by any of the velocity polygons`; and, for the stalls, the progress checker's `Failed to make progress`, the controller's `detected collision ahead` and the recoveries the behavior server ran (spin, wait). The monitor's changes to `STOP`, for an obstacle or for an `invalid source`, come from `collision_monitor_state` | Information |
| Keepout entries and clearances | Ground-truth poses during goals at which the robot's unpadded outline overlaps a keepout zone's true polygon; the smallest clearance per zone; per goal, the smallest distance from the ground-truth centre to a planning core (the zone grown 0.65 m) | Information, "no masks loaded" (judged from stage S6b) |
| Speed per zone | While the ground-truth centre is in a speed zone's true polygon: the largest commanded speed on `cmd_vel` (cap + 0.005 m/s allowed) and the largest ground-truth speed over 0.1 s (cap + 0.02 m/s) | Information (from S6b) |
| `speed_limit` | Messages, and the speed zones entered | Information (from S6b) |
| Load | The RTF, its worst 10 s, the cores per process group | The RTF above |

`/rosout` is transient local: when the recorder subscribes, every node sends it the log lines of
its last 10 s, all of which then carry the recorder's start as their receive time. The events are
counted per goal window, so those old lines fall outside every goal; the report lists what fell
outside the goals separately.

**Reading the per-goal table.** One line per goal sent: the goal's index in the set, its result
and Nav2's error code (`105 FAILED_TO_MAKE_PROGRESS` is the progress checker: the robot moved less
than 0.5 m in 10 s), the time in sim time, the ground-truth path length next to the straight
distance (their ratio shows detours), Nav2's recoveries, the end error by ground truth (position
and heading; within the goal checker's 0.10 m and 0.10 rad, plus AMCL's error, for a goal that
succeeded), and the monitor's hold time: in all, the longest hold, and how many holds lasted over
2 s.

**The monitor holds.** The collision monitor sits between the velocity smoother and the base. For
every input message that is not zero it either passes the command on, sends zero (`STOP`: an
obstacle is inside the field of the command's velocity band) or sends nothing at all (once it has
sent zeros for `stop_pub_timeout`, 1.0 s, or while it is inactive). Each input message is matched
with the monitor's output by its stamp, and an input the monitor did not pass counts as held for
the time until the next input (at most 0.15 s). A **hold** is an unbroken run of such inputs while
a goal runs. Each hold over 2 s is listed with:

- when it started and how long it lasted, split into the time the monitor sent zeros and the
  time it sent nothing;
- where the robot stood, from ground truth, and how far it moved during the hold;
- the command that was held (its range and median) and the **velocity band** that command
  selects, from the bands in `results.json`: the monitor picks the field from the command it
  receives, not from how fast the robot moves;
- the monitor's state from `collision_monitor_state`. The monitor publishes it only when it
  changes, so the state shown is that of the last message before the hold, and `unknown` before
  the first message of the bag.

So a hold of 20 s at an aisle mouth with the command held at 0.6 m/s in band `fast` and the state
`STOP 'velocity_fields'` says: the smoother kept asking for 0.6 m/s while the robot stood, so the
monitor kept checking the large `fast` field, which reaches the shelves, and kept the robot
standing. That is the stall S3a found, and what the next step fixes.

**The plot** (`--plot OUT.png`): on the left, the SLAM map from above with the keepout zones, the
ground-truth path while goals ran, the goals (a green dot for one that succeeded, a red cross for
one that did not, with its index) and each hold over 2 s as a circle whose area grows with its
time; on the right, over sim time, the commanded speed into and out of the monitor, with the goals
and the holds shaded, and the held command age with its two limits.

**Exit codes:** 0 every check passes; 1 one fails; 2 the bag cannot be read or has no ground truth
or `cmd_vel`, `results.json` cannot be read, or the plot cannot be written.

### Measured: the baseline of goals 1-10

The S3b run, on 2026-11-17: the launch of stage S2a (`navigation:=true`, 1 ms step, no windows),
the robot at its spawn, the performance power profile, nothing else running; then
`nav_benchmark --goals 10`. The run took 5 min; its bag holds 299 s of sim time in 45 MB.

- **Goals:** all 10 succeeded, 0.032-0.095 m from their poses by ground truth (mean 0.057 m), in
  283.4 s of goal time and 169.6 m of driving.
- **Commands:** held command age p99.9 0.081 s and at most 0.082 s over 4470 moving commands,
  none over 0.15 s, and no base timeout while moving. The monitor's delay (from
  `cmd_vel_smoothed`) was 20 ms median, 31 ms at most.
- **Load:** RTF 1.000 mean, 0.996 over the worst 10 s; the run cost 3.04 cores: Gazebo 1.03,
  `nav2_container` 1.05, `nav_benchmark` 0.26 (the Python tools' group), the recorder 0.10.
- **The monitor's holds**, the stall S3a found: the collision monitor held the robot for 62.0 s of
  the 283.4 s (22 %), in five holds, three of them over 2 s. During goal 3 it held it 9.0 s and then
  10.2 s at the north mouth of aisle 3 (x 8.23, y 11.01); during goal 5, 41.2 s at the west end of
  the cross aisle (x 1.45, y 9.03). Each time the smoother kept asking for 0.5-0.95 m/s, band
  `fast`, while the robot stood, and the monitor reported `STOP 'velocity_fields'`; after the
  first second it sent nothing, so the base logged a timeout every second (counted apart, not
  judged). Goals 3 and 5 took 49.7 s and 76.5 s, with 3 and 5 recoveries, six progress-checker
  failures and one spin between them. The next step fixes these holds, judged against these
  numbers.
- **Zones** (information): no keepout entry, the nearest pass 1.09 m from a dock bar's box; the
  robot drove through the staff crossing (cap 0.3 m/s) and four pick-aisle segments (0.5 m/s) at
  up to 1.0 m/s, since no speed filter exists yet.

`evaluate_navigation` printed `6 of 6 navigation checks pass`. A separate reading of the same bag
with `rosbag2_py`, using no code of this package, gave the same success count, the same held
command age (p99.9 and maximum), goal 5's end error (0.053 m) and goal 5's hold (41.2 s at the
same place).
