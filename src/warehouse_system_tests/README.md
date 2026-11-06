# warehouse_system_tests

System-level tests: they launch the real bring-up (`warehouse_bringup/launch/sim.launch.py`)
and check the running system from the outside with `launch_testing`, like a user with `ros2`
commands would. Unit-level checks live in the packages themselves (for example
`warehouse_description/test`).

The package also holds the **test tools** of Phase 2: `drive_route` drives a scripted route,
`evaluate_localization` measures AMCL against ground truth from a bag, and `map_timelapse`
turns a mapping bag into a GIF. They work in simulation only and steer on, or compare with,
Gazebo's ground truth, so they are never part of the robot stack (plan P2-D11).

## Files

| File | What it does |
|---|---|
| `test/test_mock_bringup.py` | `hardware:=mock` in `/robot1`, no simulator. Runs everywhere, including PR CI |
| `test/test_gz_bringup.py` | `hardware:=gz`, headless Gazebo in `/robot1`, plus a static box for the scanner to see: the Phase 1 acceptance checks. Needs GPU or EGL rendering; skipped in CI (below) |
| `warehouse_system_tests/ros_helpers.py` | Shared helpers: read `robot.yaml` and `controllers.yaml`, spin until a condition holds, collect messages (and stop collecting when a test ends), list controllers, read the TF tree, rates, odometry and braking deceleration from message stamps, and the time windows in which `cmd_vel_timeout` must show |
| `scripts/drive_route`, `warehouse_system_tests/route_driver.py` | The `drive_route` command: the controller (pure Python) and the ROS node around it |
| `scripts/evaluate_localization`, `warehouse_system_tests/localization_eval.py` | The `evaluate_localization` command |
| `scripts/map_timelapse`, `warehouse_system_tests/map_timelapse.py` | The `map_timelapse` command |
| `warehouse_system_tests/bags.py` | Read and write rosbag2 bags (MCAP) with `rosbag2_py`, without a ROS graph |
| `test/test_route_driver.py` | The driver's geometry and controller on a simulated robot (also one that lags behind its commands), its limits, the dry run. No ROS graph |
| `test/test_localization_eval.py` | `evaluate_localization` on synthetic MCAP bags with known errors, drift and IMU noise |
| `test/test_map_timelapse.py` | `map_timelapse` on a small synthetic mapping bag |

## Test tools

Run them in a second terminal while the simulation runs (`world:=warehouse`). Bags go into
`bags/` at the workspace root, which git ignores: they are large.

### drive_route

```bash
ros2 run warehouse_system_tests drive_route --route survey --speed 0.4        # the mapping run
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
| `--initial-pose-offset DX DY DYAW_DEG` | Once AMCL is active (its lifecycle state), publish one pose on `/robot1/initialpose`: the true pose moved by DX, DY metres (map frame) and turned by DYAW_DEG degrees, covariance 0.5² m² and 0.2² rad². AMCL then starts from a wrong pose and must find the right one |
| `--namespace NS` | The robot, default `robot1` |
| `--dry-run` | Print the waypoints, the length and the expected time (the route driven by an ideal robot), and exit without touching the ROS graph |

**Commands at 20 Hz, stamped in sim time.** `diff_drive_controller` brakes when its newest
command is older than `cmd_vel_timeout` (`controllers.yaml`), and it measures that age from the
command's header stamp, in sim time. So the driver publishes `TwistStamped` on
`/robot1/cmd_vel` every 0.05 s while driving, also when the command does not change, stamped
with the node's sim-time clock. A wall-clock stamp would be wrong in simulation (plan R1).

**Within the controller's limits.** Speed and turn rate are checked against `controllers.yaml`.
Commands change by at most half the controller's acceleration limits, so the controller never
has to clip them, and stops are planned with half of that braking again: a robot that lags
behind its commands still stops on the waypoint. On a simulated robot that sees its pose and
acts 0.1 s late, the driver stays within 1 cm of the route at 1.2 m/s; without the braking
reserve it left the route by 0.28 m (`test_route_driver.py`).

**Exit codes:** 0 the route was driven; 1 it could not be (no ground truth within 30 s, AMCL
not active within 60 s, the robot more than 0.5 m from the route's start, no progress for 10 s,
or the simulation clock stopped); 2 a bad argument; 130 stopped by Ctrl-C. Before every exit
the robot is commanded to stop. If the driver dies anyway, the controller's timeout stops the
robot: that is what the timeout is for.

### evaluate_localization

```bash
# 1. With sim.launch.py world:=warehouse localization:=amcl running, start recording:
ros2 bag record -o bags/loc_run --use-sim-time --topics /robot1/amcl_pose /robot1/initialpose \
  /robot1/ground_truth/odom /robot1/odom /robot1/imu /robot1/tf /robot1/tf_static /clock
# 2. In another terminal, the evaluation run (about 7 min, plus 30 s standing still):
ros2 run warehouse_system_tests drive_route --route survey --speed 0.5 --settle 30 \
  --initial-pose-offset 0.3 -0.3 8
# 3. Stop the recorder (Ctrl-C), then:
ros2 run warehouse_system_tests evaluate_localization bags/loc_run --plot /tmp/loc.png
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
stores only what changed from one frame to the next, so the file stays small: 0.06 MB for 60
frames of a stand-in for the survey mapping run (the rasterized warehouse revealed along the
route). If a GIF would still be larger than `--max-mb`, the command keeps every second frame,
then narrows the frames, until it fits; exit code 1 if it cannot.

## What is checked

| Check | Mock | Gazebo |
|---|---|---|
| `joint_state_broadcaster`, `diff_drive_controller`, `imu_sensor_broadcaster` active | ✓ | ✓ |
| `/robot1/cmd_vel` is `TwistStamped` with `diff_drive_controller` as the only subscriber | ✓ | ✓ |
| Frames without prefix: `odom/wheel` and `odom` are `odom` → `base_footprint`; `imu` is `imu_link`; `scan` is `lidar_front_link`; ground truth is `map` → `base_footprint` | ✓ (no scan) | ✓ |
| TF tree in `/robot1/tf` equals the Phase 1 tree of plan §2.4 (visual-only parts included); publishers are `robot_state_publisher` and `ekf_filter_node`; nothing arrives on the global `/tf` or `/tf_static` | ✓ | ✓ |
| A command moves the robot; with no further commands it keeps its speed until `cmd_vel_timeout` (0.15 s) expires, then brakes to a standstill. The time windows come from `controllers.yaml` (`cmd_vel_timeout_windows()`), so the check holds for any timeout | ✓ | ✓ (stamped in sim time) |
| Braking keeps to the hard limit (1.5 m/s², ±20 %) from forward (+0.5 m/s) and from full reverse (-0.3 m/s) | ✓ | |
| Rates from header stamps: `scan` 10 Hz, `imu` 100 Hz, `odom/wheel` 50 Hz, ±10 % | | ✓ |
| The scanner covers its full field of view (`fov_deg`, a full circle) and sees a box 2 m to the left: every return lies on the box (world coordinates via ground truth, within 5σ of the range noise), none inside the robot's outline (self-hits) | | ✓ |
| Wheel and EKF odometry against ground truth: < 1 % over 10 m straight, < 2 % over a 360° turn | | ✓ |
| Every process exits with code 0 after one SIGINT | ✓ | ✓ |
| No process of the test outlives it (orphaned Gazebo server) | | ✓ |

Rates and durations are measured with message stamps, so they hold in sim time even when
Gazebo runs slower than real time. The tests print the measured rates, braking decelerations,
scan counts and odometry errors; read them in
`build/warehouse_system_tests/launch_test/test_test_<name>.py.txt`.

**No robot numbers in the tests.** The outline, rates, scanner samples and noise, timeout and
limits come from `warehouse_description/config/robot.yaml` and `controllers.yaml`
(`ros_helpers.description_config()`), so a new robot revision cannot leave a test checking a
stale value (plan §6). The numbers in the table above are today's values.

**Commands at 20 Hz.** The tests publish `cmd_vel` every 0.05 s (`COMMAND_PERIOD` in
`ros_helpers.py`), like Nav2's controller server and velocity smoother. `cmd_vel_timeout`
(0.15 s) must stay well above that period, or the robot would brake between two commands.

**Why a box for the scanner check.** The empty world has nothing in the scan plane, so every
ray misses. "No return inside the outline" would then pass for a blind, turned or misplaced
scanner. With a box at a known place, the test also proves the scanner sees, and that its
returns land where the URDF says the scanner is.

## Running

```bash
colcon test --packages-select warehouse_system_tests
colcon test-result --verbose
# one test file, with its output on the terminal:
colcon test --packages-select warehouse_system_tests --ctest-args -R test_gz \
  --event-handlers console_direct+
```

The mock test takes about 20 s, the Gazebo test about a minute.

The test tools' unit tests need no simulator and no ROS graph, and take a few seconds. While
checks are paused (ADR-004) a plain `colcon build` builds no tests, so run them with pytest from
the workspace root, after building and sourcing the workspace:

```bash
python3 -m pytest src/warehouse_system_tests/test/test_route_driver.py \
  src/warehouse_system_tests/test/test_localization_eval.py \
  src/warehouse_system_tests/test/test_map_timelapse.py
```

**Isolation.** Each test file runs in its own ROS domain: CMake wraps it with
`run_test_isolated.py` from `ament_cmake_ros`, which picks a free `ROS_DOMAIN_ID` from
`domain_coordinator` unless `ROS_DOMAIN_ID` is already set. The Gazebo test also gives Gazebo a
private `GZ_PARTITION`, so a simulation you have open does not interfere. If you set
`ROS_DOMAIN_ID` yourself, do not run a simulation in the same domain while testing: two
`/robot1/controller_manager` nodes confuse the spawners.

**Known issue: Gazebo sometimes hangs on shutdown.** About one Gazebo test run in 15 (measured
in Phase 1) fails only `test_processes_exit_cleanly`, with `Proc gazebo-1 exited with code -9`:
the Gazebo server ignored SIGINT and SIGTERM, and launch killed it. This is the upstream bug
[gazebosim/gz-sim#4173](https://github.com/gazebosim/gz-sim/issues/4173) (Harmonic with a
`gpu_lidar`, headless): gz-common's signal handler stops the Sensors system from inside an
interrupted gz-transport publish, and the render thread deadlocks. When every other case
passed, re-run the test.

## The `WAREHOUSE_SKIP_GZ_TESTS` switch

Gazebo's scanner renders the scene, which needs a GPU or EGL. The GitHub runners have neither,
so CI's build-and-test job sets `WAREHOUSE_SKIP_GZ_TESTS=1` (`.github/workflows/ci.yml`). The
Gazebo test's `generate_test_description` is decorated with `unittest.skipIf` on that variable,
so `launch_testing` reports each of its cases as **skipped** with the reason, and
`colcon test-result` counts them as skipped (not passed, not missing). Locally the variable is
unset and the Gazebo tests run. CI still checks the Gazebo-side files without rendering:
`warehouse_gazebo/test/test_gazebo_config.py` (world systems, bridged topics, caster friction)
does not depend on the switch. To skip the Gazebo launch test on purpose:

```bash
WAREHOUSE_SKIP_GZ_TESTS=1 colcon test --packages-select warehouse_system_tests
```

A nightly CI job with headless Gazebo comes in Phase 6 (ADR-004).
