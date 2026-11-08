# warehouse_system_tests

The **test tools** of the simulation: `drive_route` drives a scripted route,
`evaluate_localization` measures AMCL against ground truth from a bag, and `map_timelapse`
turns a mapping bag into a GIF. They work in simulation only and steer on, or compare with,
Gazebo's ground truth, so they are never part of the robot stack (plan P2-D11).

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
with the node's sim-time clock. A wall-clock stamp would be wrong in simulation (plan R1).

**The first command only once the controller listens.** DDS delivers messages only between a
publisher and a subscription that discovery has matched, a fraction of a second after the
publisher is created, and it matches the two ends separately. A command that reaches the
controller before its end knows the publisher is dropped and sent again later, by then too old
for `cmd_vel_timeout`: in one mapping run the first four commands were ignored that way and the
robot started late. So the driver waits until ROS lists `diff_drive_controller`'s subscription
to `/robot1/cmd_vel` and its publisher is matched with every subscription listed (a count above
zero is not enough: a bag recorder that records `/robot1/cmd_vel` may be matched first). The
controller's end cannot be checked from the driver, so the first command also waits until the
driver has run for 3 s (`DISCOVERY_SETTLE`).

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
and `/clock` ticks 1000 times a second. A second SIGINT while it is stopping the robot prints a
traceback; the first stop command has already gone out by then.

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

Start the recorder after the launch, as above: it then takes each publisher's QoS. A recorder
that subscribes before a publisher exists (for example with `--include-unpublished-topics`) uses
volatile QoS and records none of the one transient-local `tf_static` message. The evaluation does
not need `tf_static`, but a replay of the bag through AMCL does.

**Measured** on the acceptance run of 2026-11-08 (the survey at 0.5 m/s after 30 s standing
still, started 0.3 m, -0.3 m and 8° off): AMCL converged after 0.78 m and stayed within 0.092 m
of ground truth for 7 min 07 s (mean 0.024 m, P95 0.052 m); the EKF drifted 0.058 m over
175.6 m; the IMU's σ was 0.99-1.03 times `robot.yaml` on every axis, from 40.6 s standing still
with 506 repeated samples removed; 4 of 4 checks passed. The full output is in the
[Phase 2 report](../../docs/phase-reports/phase-02-warehouse-world.md#exercise-8-measure-localization-from-a-bag).

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
