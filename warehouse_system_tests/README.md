# warehouse_system_tests

System-level tests: they launch the real bring-up (`warehouse_bringup/launch/sim.launch.py`)
and check the running system from the outside with `launch_testing`, like a user with `ros2`
commands would. Unit-level checks live in the packages themselves (for example
`warehouse_description/test`).

## Files

| File | What it does |
|---|---|
| `test/test_mock_bringup.py` | `hardware:=mock` in `/robot1`, no simulator. Runs everywhere, including PR CI |
| `test/test_gz_bringup.py` | `hardware:=gz`, headless Gazebo in `/robot1`, plus a static box for the scanner to see: the Phase 1 acceptance checks. Needs GPU or EGL rendering; skipped in CI (below) |
| `warehouse_system_tests/ros_helpers.py` | Shared helpers: read `robot.yaml` and `controllers.yaml`, spin until a condition holds, collect messages (and stop collecting when a test ends), list controllers, read the TF tree, rates, odometry and braking deceleration from message stamps |

## What is checked

| Check | Mock | Gazebo |
|---|---|---|
| `joint_state_broadcaster`, `diff_drive_controller`, `imu_sensor_broadcaster` active | ✓ | ✓ |
| `/robot1/cmd_vel` is `TwistStamped` with `diff_drive_controller` as the only subscriber | ✓ | ✓ |
| Frames without prefix: `odom/wheel` and `odom` are `odom` → `base_footprint`; `imu` is `imu_link`; `scan` is `lidar_front_link`; ground truth is `map` → `base_footprint` | ✓ (no scan) | ✓ |
| TF tree in `/robot1/tf` equals the Phase 1 tree of plan §2.4; publishers are `robot_state_publisher` and `ekf_filter_node`; nothing arrives on the global `/tf` or `/tf_static` | ✓ | ✓ |
| A command moves the robot; with no further commands it brakes after `cmd_vel_timeout` (0.5 s) | ✓ | ✓ (stamped in sim time) |
| Braking keeps to the hard limit (1.5 m/s², ±20 %) from forward (+0.5 m/s) and from full reverse (-0.3 m/s) | ✓ | |
| Rates from header stamps: `scan` 10 Hz, `imu` 100 Hz, `odom/wheel` 50 Hz, ±10 % | | ✓ |
| The scanner sees a box 2 m to the left: every return lies on the box (world coordinates via ground truth, within 5σ of the range noise), none inside the robot's outline (self-hits) | | ✓ |
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
unset and the Gazebo tests run. To skip them on purpose:

```bash
WAREHOUSE_SKIP_GZ_TESTS=1 colcon test --packages-select warehouse_system_tests
```

A nightly CI job with headless Gazebo comes in Phase 6 (ADR-004).
