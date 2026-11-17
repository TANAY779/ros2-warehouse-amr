# warehouse_ops

The **operator tools** of the warehouse AMR. There is one so far: `navigate` sends the robot to
named locations of the location registry, or to poses on the map, through Nav2's actions, and
reports how each goal ends. It is an operator tool, not a test tool: it never reads Gazebo's
ground truth. The tools that judge the system live in `warehouse_system_tests` (plan P3-D21).
Stage S3a of Phase 3a built it.

## Files

| File | What it does |
|---|---|
| `warehouse_ops/navigate.py` | The `navigate` command: its options, the checks before the first goal, the goals, the feedback, the results and Ctrl-C |
| `warehouse_ops/locations.py` | Reads the location registry that `warehouse_layout` installs, checks its schema version and frame, and gives each pose with its kind |

## See it

The list of named locations needs no simulation:

```bash
ros2 run warehouse_ops navigate --list
```

It prints the 46 poses of the registry, grouped by kind:

```text
46 named poses in .../share/warehouse_layout/generated/locations.yaml
(world warehouse, frame map; x and y in metres, yaw in radians)

spawn: 3, where each robot starts and parks
  spawn_robot1    x   0.000  y   0.000  yaw  0.000  robot robot1
  ...
pick: 36, on a pick aisle's centreline at a bay, facing along the aisle
  pick_1_01       x   2.800  y   3.825  yaw  1.571  aisle 1, bay 1, bays A01 B01
  ...
dock_docked: 3, on a dock's charging contacts: docking only, refused by --to
  dock_1_docked   x  10.000  y  -1.130  yaw -1.571  dock dock_1, docking only
  ...
```

Then start the simulation with Nav2:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse navigation:=true gz_gui:=false
```

and in a second terminal set up like the first, send the robot to a pick pose:

```bash
ros2 run warehouse_ops navigate --to pick_2_03
```

A run prints what it checks, one feedback line a second and the result. This one, to the drop
pose from the spawn, is from the measured run below:

```text
Checking AMCL and Nav2 in /robot1 (for up to 60 s)
AMCL is active, its last estimate x -0.05 m, y -0.07 m, yaw -0.01 rad; /robot1/navigate_to_pose is ready
[INFO] [1794827595.788639612] [robot1.navigate]: Nav2 is ready for use!
Goal 1 of 1: drop_pack_1 (x 5.50 m, y 0.40 m, yaw 0.00 rad), NavigateToPose
[INFO] [1794827595.789199987] [robot1.navigate]: Navigating to goal: 5.5 0.4...
  drop_pack_1: 5.6 m to go, about 18 s left (1 s so far, recoveries 0)
  drop_pack_1: 5.0 m to go, about 6 s left (2 s so far, recoveries 0)
  ...
  drop_pack_1: 0.0 m to go, about 0 s left (8 s so far, recoveries 0)
drop_pack_1: SUCCEEDED after 8.3 s, the robot 0.03 m and 0.07 rad from drop_pack_1 by Nav2's last feedback
```

The `[INFO]` lines come from `BasicNavigator` (`nav2_simple_commander`), the library the tool
drives Nav2 with; every other line is the tool's own.

Until stage S5 puts twist_mux in front of the collision monitor, teleop and Nav2 both publish on
`/robot1/cmd_vel`: stop `navigate` (Ctrl-C cancels its goal) before you drive by hand.

## Options

| Option | Meaning | Example |
|---|---|---|
| `--to LOCATION` | A named location of the registry (`--list` prints them). Repeat it for several goals: they are driven in the order given | `navigate --to pick_2_03` |
| `--pose X Y YAW` | A pose on the map: X and Y in metres, YAW in **radians**, counter-clockwise from +x (1.5708 faces +y). Repeatable, and it mixes with `--to` in the order given | `navigate --pose 8.2 1.5 0` |
| `--through` | All poses as one `NavigateThroughPoses` goal ([below](#several-goals)) | `navigate --to dock_1_staging --to spawn_robot1 --through` |
| `--waypoints` | All poses as one `FollowWaypoints` goal ([below](#several-goals)) | `navigate --to pick_1_01 --to pick_2_01 --waypoints` |
| `--list` | Print the named locations and exit. Needs no ROS graph, only this workspace sourced | `navigate --list` |
| `--namespace NS` | The robot, default `robot1` | `navigate --namespace robot1 --to spawn_robot1` |

`--through` and `--waypoints` exclude each other, and `--list` takes no goals. ROS arguments
(`--ros-args ...`) go to rclpy. `navigate --help` prints the options and the exit codes.

### Several goals

| How | Nav2 action | What the robot does | When a pose cannot be reached |
|---|---|---|---|
| default | one `NavigateToPose` goal per pose, each sent once the one before succeeded | stops at each pose, turned to its yaw | the run ends there, the poses not sent are listed, exit 1 |
| `--through` | one `NavigateThroughPoses` goal | drives through the poses without stopping: a pose counts as passed once the robot is within 0.7 m of it (`RemovePassedGoals` in the tree), and its yaw is not used. It stops at the last pose | the goal fails, exit 1 |
| `--waypoints` | one `FollowWaypoints` goal: the waypoint follower sends a `NavigateToPose` goal per pose | stops at each pose and waits 200 ms there | the follower goes on to the next pose (`stop_on_failure: false`), and the tool lists every missed waypoint at the end, exit 1 |

The default sends one goal per pose so that each pose gets its own result and error code, and the
first failure stops the robot where it is, for a person to look at. `--waypoints` is the way to
go on past a pose that fails.

## Locations

The location registry is `warehouse_layout/generated/locations.yaml`, generated from
`layout.yaml` and installed into `share/warehouse_layout/generated/`. `locations.py` reads that
installed file, found through `ament_index` the way `warehouse_layout` finds its own
`layout.yaml`, and keeps no copy. It refuses a registry whose `schema_version` is not 1 or whose
`frame_id` is not `map` (the goals go to Nav2 in the map frame as they are), and exits with 2.

| Kind | Poses | Where | `--to` |
|---|---|---|---|
| `spawn` | 3 | where each robot starts and parks | yes |
| `pick` | 36 | on a pick aisle's centreline at a bay, facing along the aisle (north) | yes |
| `drop` | 1 | at the packing counter, side-on, 0.60 m from its face | yes |
| `dock_staging` | 3 | 0.70 m in front of a dock, facing it | yes |
| `dock_docked` | 3 | on the dock, the robot's front on the charging contacts | no: **docking only** |

A docked pose puts the robot's front on the contact bar, inside the keepout box round the bar
that Nav2 keeps out of once Phase 3b loads the zones. Phase 5's docking server drives the last
0.70 m from the staging pose. So `--to dock_1_docked` exits with 2 and names the staging pose
instead.

## Before the first goal

**Its own checks.** For up to 60 s, the tool checks, in this order:

1. AMCL (`/robot1/amcl`) reports the lifecycle state active (`get_state`).
2. AMCL has published a pose on `/robot1/amcl_pose`. AMCL publishes it reliable and transient
   local, but only after an update, so the tool subscribes with the same QoS and still gets the
   last estimate when it joins late.
3. `bt_navigator` is active, and for `--waypoints` also `waypoint_follower`.
4. The goal's action server (`navigate_to_pose`, `navigate_through_poses` or
   `follow_waypoints`) answers.

Each check gives the discovery of a service, an answer or the latched pose up to 5 s before it
calls it missing: discovery alone took up to 3 s with Gazebo and Nav2 running. While a check
fails, the tool prints what is missing when that changes, and again every 15 s; after 60 s it
exits with 1 and names it:

```text
navigate: not ready after 60 s: no answer from /robot1/amcl/get_state: is the simulation running, with navigation:=true and in this shell's ROS domain (69)?
```

Started together with the launch, it prints a line or two like
`waiting (6 s): /robot1/bt_navigator is inactive, not active` and goes on once Nav2 is up.

**Then BasicNavigator's own check.** Once its checks pass, the tool calls
`waitUntilNav2Active(navigator='bt_navigator', localizer='robot_localization')`, which logs
`Nav2 is ready for use!`. It waits without a timeout, which is why the tool's checks come
first, and it sleeps 2 s after asking `bt_navigator` for its state, so each run starts about 2 s
later than it has to.

### The localizer trap, and why the tool checks AMCL itself

`waitUntilNav2Active()` takes `localizer='amcl'` by default. Then it waits until AMCL is active
and, until it hears `amcl_pose`, publishes its own initial pose on `initialpose` once a second
(`nav2_simple_commander/robot_navigator.py` lines 495-503 and 795-801 in the installed Nav2
1.3.13). That pose is (0, 0, 0) in the map frame, unless the script called `setInitialPose()`
first. AMCL accepts it, so the estimate of a robot standing anywhere else jumps to the origin,
and Nav2 plans from a place where the robot is not. With robot1 on its spawn pose, the origin,
the mistake stays hidden; after the first goal it no longer does. The waypoint example of the
ROS 2 engineering skill (`navigation.md` §7) calls it with the default.

`localizer='robot_localization'` skips both: the AMCL check and the initial pose. It waits for
the navigator only. So the tool checks AMCL itself (steps 1 and 2 above), and it **never
publishes on `initialpose`**. BasicNavigator does create a publisher on that topic when it
starts, so `ros2 topic info -v /robot1/initialpose` lists `/robot1/navigate` as a publisher
while the tool runs; nothing goes out on it (checked with `ros2 topic echo`, see
[Measured](#measured)).

## Feedback

Once a second, from the goal's action feedback:

```text
  pick_2_03: 4.4 m to go, about 4 s left (6 s so far, recoveries 0)
  1 of 2 poses to go, next spawn_robot1: 9.9 m to go, about 23 s left (7 s so far, recoveries 0)
  waypoint 1 of 1, pick_4_05 (3 s so far)
```

The distance left along the path, Nav2's estimate of the time left (rough while the robot speeds
up), the time since the goal started (sim time) and the recoveries so far. With `--through` the
line starts with how many poses are left and the next one. `FollowWaypoints` feedback carries
only the waypoint the robot drives to, so `--waypoints` shows that, and the time since the goal
started by the tool's own clock.

## Results and exit codes

Each goal ends with one line:

- `SUCCEEDED`, with where Nav2 last saw the robot: AMCL's estimate, so within the goal tolerance
  of 0.10 m and 0.10 rad;
- `FAILED`, BasicNavigator's name for the action status `ABORTED`, with the error code;
- `CANCELED`: after Ctrl-C, or, without one, cancelled by another client (exit 1).

| Exit code | When |
|---|---|
| 0 | every goal succeeded |
| 1 | a goal failed or was rejected, a waypoint was missed, a goal was cancelled by another client, or AMCL and Nav2 were not ready within 60 s |
| 2 | a usage error: an unknown location (the message suggests close names), a docking-only location, a number that is not one, `--pose` without three numbers, `--through` with `--waypoints`, `--list` with goals, no goal at all; or an unreadable registry. Nothing was sent |
| 130 | Ctrl-C (or SIGTERM): the goal was cancelled first |

**Error codes.** `bt_navigator`'s result carries the code of the planner or the controller that
failed (the trees' `error_code_names` in `nav2_params.yaml`; if both failed, the lower code, the
controller's); a missed waypoint carries the code of its `NavigateToPose` goal. The tool prints the code with its name from the installed action
definitions:

| Codes | From | Examples |
|---|---|---|
| 100-107 | controller (`FollowPath`) | 105 `FAILED_TO_MAKE_PROGRESS`: the progress checker saw the robot move less than 0.5 m in 10 s |
| 200-208 | planner, for `NavigateToPose` (`ComputePathToPose`) | 205 `START_OCCUPIED`, 208 `NO_VALID_PATH` |
| 300-309 | planner, for `NavigateThroughPoses` (`ComputePathThroughPoses`) | 305 `START_OCCUPIED` |
| 600-601 | waypoint follower (`FollowWaypoints`) | 601 `TASK_EXECUTOR_FAILED` |

Nav2 1.3.13's `bt_navigator` fills in the code but leaves `error_msg` empty, so the lines end
with `no error message`. A `FAILED` goal with error code 0 had no planner or controller failure:
most often another client's goal replaced it, because a new goal to the same action ends the one
that runs (RViz's "2D Goal Pose", another `navigate`), and the tool says so.

### START_OCCUPIED

The planner reports `START_OCCUPIED` when the robot's centre stands on a lethal cell of the global
costmap: Smac 2D then gives up after one iteration. Today that means the centre within the
padded inscribed radius (0.28 m) of an obstacle; from stage S6b on, also a robot pushed into the
0.65 m core round a keepout zone (plan §3.3). Nav2 runs no recovery for it (the trees'
`WouldAPlannerRecoveryHelp` only answers unknown errors, no valid path and timeouts), so the goal
fails at once. The tool then prints:

```text
  the robot stands in a keepout margin or an obstacle: drive it out by hand
```

Drive it out with the teleop window (until S5, after Ctrl-C on `navigate`), then send the goal
again.

## Ctrl-C

- **The first Ctrl-C** cancels the goal: the tool sends the cancel request, waits until Nav2
  accepts it (up to 5 s) and the goal ends `CANCELED` (up to 10 s), prints both, and exits with
  130. Before a goal is sent it exits with 130 at once. Several goals: those not sent yet are
  listed.
- **A second Ctrl-C** stops any wait at once and exits with 130. If the first had not been acted
  on yet, it sends the cancel request first and leaves without waiting for the answer.
- If Nav2 does not answer, the tool prints a command that cancels every goal of the action, for
  example `ros2 service call /robot1/navigate_to_pose/_action/cancel_goal
  action_msgs/srv/CancelGoal "{}"` (an empty request, with a zero goal ID and a zero stamp,
  means every goal).
- SIGTERM counts as Ctrl-C.

How it works, and why:

- **rclpy's own signal handlers are off** (`rclpy.init(..., signal_handler_options=
  SignalHandlerOptions.NO)`). At the first Ctrl-C they shut the ROS context down, and a cancel
  is a request to `bt_navigator`, which needs a live context. The tool installs its own handler
  for SIGINT and SIGTERM instead.
- **The first press only sets a flag.** The tool looks at it between its own steps, at least
  every 0.1 s (`isTaskComplete()` spins for 0.1 s at a time), and cancels there. Raising
  `KeyboardInterrupt` at once, at whatever line runs, can stop rclpy halfway through a call: a
  Ctrl-C that landed inside rclpy once ended `drive_route` with a traceback and no stop command
  ([warehouse_system_tests](../warehouse_system_tests/README.md#drive_route)).
- **The second press raises `KeyboardInterrupt`**, which ends a wait even inside
  `BasicNavigator`, whose waits have no timeout. A wait in rclpy's executor returns only when
  something arrives, and with rclpy's handlers off a signal does not count, so a 10 Hz timer on
  the tool's node makes every such wait return at least every 0.1 s.
- **Once the tool exits, it ignores further signals.** When Python exits, it puts the default
  handler back for every signal it handled, and a Ctrl-C then kills the process: `ros2 run`
  reports `Interrupt` and exits with 254 instead of the tool's 130. This happened in a first try
  with two Ctrl-Cs 50 ms apart.
- To test Ctrl-C from a script, start the tool with job control (`set -m`) and send SIGINT to
  its process group (`kill -INT -- -PGID`), as a terminal does. `ros2 run` only waits for the
  tool, so SIGINT to the `ros2 run` process alone never reaches it.

## The wall clock, not sim time

The tool does not set `use_sim_time`. Its goals carry a zero stamp in the map frame, the frame
Nav2 plans in, so Nav2 neither transforms them nor reads a time from them; its timeouts and the
once-a-second feedback are for a person watching. A Python node on sim time reads `/clock` 1000
times a second: `drive_route` spends about 0.4 of a core on it.

## Measured

Two runs on 2026-11-16 in headless Gazebo, `sim.launch.py world:=warehouse navigation:=true
gz_gui:=false rviz:=false`, robot1 starting on its spawn pose. `ros2 topic echo
/robot1/initialpose` (best effort, so it matches any publisher) ran from before the first
`navigate` to the end. Ground truth is Gazebo's `/robot1/ground_truth/odom`, logged by a script
outside the robot stack. The first run's code differed from the second's only in the wording of
some messages and a 2 s instead of 5 s answer wait; after the second run one more waiting message
was reworded.

- `--list`: 46 poses: 3 spawn, 36 pick, 1 drop, 3 dock staging, 3 docked, each marked
  `docking only`; exit 0 with no ROS graph.
- `--to pick_2_03`, started while Nav2 was still coming up: `SUCCEEDED` and exit 0 in both
  runs. Ground truth then lay 0.013 m and 0.033 rad, and 0.020 m and 0.057 rad, from the pose.
  The goal took 12.0 s and 32.1 s: in the second run the collision monitor held the robot for
  21 s at the mouth of aisle 2 (see below).
- Ctrl-C while driving north along aisle 2 at 1.0 m/s: the tool saw it 98 ms and 60 ms after
  the signal, Nav2 accepted the cancel within 1 ms, the goal ended `CANCELED` 47 ms and 17 ms
  after the request, and the tool exited with 130. By ground truth the robot was below 0.05 m/s
  for good 1.18 s and 1.09 s after the signal (limit 1.5 s).
- Two Ctrl-Cs 50 ms apart (both before the tool saw the first): exit 130, no traceback, the
  cancel request sent by the second, the robot below 0.05 m/s 0.79 s and 1.13 s after the first.
  A second Ctrl-C sent the moment the tool printed `Ctrl-C: cancelling` (a `--waypoints` goal):
  `stopped waiting for the goal to end`, exit 130, no traceback, the robot stopped 1.09 s after
  the first.
- `--pose 4.15 5.0 0`, inside shelf rows B-C: `FAILED`, error code 208 `NO_VALID_PATH`
  (planner), no error message, exit 1, 23.2 s and 22.4 s after the goal was sent. No free cell
  lies within the planner's 0.125 m goal tolerance, so every plan failed, and Nav2 ran its
  recoveries (clearing the costmaps, a turn in place, a 5 s wait) before it gave up.
- `--to pick_1_01`, then `--to drop_pack_1`, a start away from the spawn: pick_1_01 succeeded
  in both runs (61.2 s and 63.9 s, ground truth 0.011 m and 0.059 m from it). drop_pack_1 failed
  in the first run (error code 105 `FAILED_TO_MAKE_PROGRESS` after 164.4 s, exit 1) and
  succeeded in the second (122.5 s, ground truth 0.095 m and 0.105 rad from it). From the spawn
  it took 8.3 s. The slow goals are not the tool: see the next point.
- The collision monitor (`Robot to stop due to velocity_fields polygon`) stopped the robot
  again and again on some approaches: at the mouth of a pick aisle as the robot turned in, and on
  the diagonal approach to the drop pose, where the packing counter came inside the `fast` field
  (to 1.0 m ahead of the robot's centre and 0.55 m to each side). The long stops lasted 7-20 s,
  one 41.5 s, and each ended only after the progress checker had failed the controller (less
  than 0.5 m in 10 s) and a recovery had run. The likely reason: the velocity smoother works open
  loop, so while the robot stands its command stays in the band it had, and that band's field
  still holds the obstacle. The stops added up to 21 s on pick_2_03 (second run), 42 s and 44 s
  on pick_1_01, and 136 s and 97 s on drop_pack_1; from the spawn, along the counter, 0.6 s. The
  fields and the smoother are Nav2's configuration (`warehouse_navigation`, plan §3.4), not this
  tool's, and were left as they are.
- `--through --to dock_1_staging --to spawn_robot1`: `SUCCEEDED` in 21.3 s and 20.7 s, exit 0.
  `--waypoints --to pick_1_01 --pose 4.15 5.0 0 --to spawn_robot1`: waypoint 2 missed with 208
  `NO_VALID_PATH`, the other two reached, `SUCCEEDED ... but it missed 1 of 3 waypoints`, exit 1.
- `initialpose`: the echo printed nothing while `navigate` started and drove, 19 times over the
  two runs. Only a pose published by hand at the end of each run, to show that the echo listens,
  came through. During a run, `ros2 topic info -v /robot1/initialpose` listed `/robot1/navigate`
  as the only publisher, and AMCL and the echo as subscribers, both best effort.
- `navigate` itself used 0.19-0.27 of a core on average from its start, its start-up included.
- Usage errors (unknown and docking-only locations, `--pose 4.15 abc 0`, `nan`, two numbers,
  `--through --waypoints`, `--list --to ...`, no goal) exited with 2 before anything started.
  With no simulation running, the tool exited with 1 after 61 s, naming AMCL's `get_state`; a
  Ctrl-C during that wait exited with 130 at once.
