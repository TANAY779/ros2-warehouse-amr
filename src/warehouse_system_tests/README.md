# warehouse_system_tests

The **test tools** of the simulation: `drive_route` drives a scripted route,
`evaluate_localization` measures AMCL (and, with `--odometry`, the odometry) against ground truth
from a bag, `map_timelapse` turns a mapping bag into a GIF, and `measure_load` measures what a
run costs the laptop and how fast Gazebo runs. They work in simulation only and steer on, or
compare with, Gazebo's ground truth, so they are never part of the robot stack (plan P2-D11).

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
then a summary table. It is the sampler of the
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
the samples taken).

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
