# The warehouse map

`warehouse.yaml` and `warehouse.pgm` in this folder are the map AMCL localizes on:
`sim.launch.py localization:=amcl` serves this map unless `map:=` names another one. slam_toolbox
made it on the scripted survey route, `check_map` checked it against the layout, and it is
committed with the code (plan F5, P2-D15: 0.05 m cells, trinary PGM). CMake installs this folder.

**Status: accepted.** `check_map` passes all five criteria of plan F5 on this map. It is the best
of three mapping runs made the same way; one of the other two missed the offset limit by 4 mm
(the table below), so the placement is close to the limit.

## This map

| | |
|---|---|
| Made | 2026-11-07, mapping run 1 of 3 (below) |
| Layout | `warehouse_layout/config/layout.yaml`, sha256 `0e9ae20f6b606107d7b212a7aad4f113722f604a61eacfdb107f17a27ef1d9eb` |
| Software | ROS 2 Jazzy; Gazebo Harmonic (gz-sim 8.15.0); slam_toolbox 2.8.5 (`async_slam_toolbox_node` with `config/slam_toolbox.yaml`); nav2_map_server 1.3.13 (`map_saver_cli`) |
| Drive | `drive_route --route survey --speed 0.4 --turn-rate 0.2`: 175.6 m in 9 min 40 s of sim time and 9 min 41 s of wall time; real-time factor 0.999 while driving |
| Map | 611 x 403 cells of 0.05 m, origin (-1.612, -2.089, 0), 7285 occupied cells |
| `check_map` | best-fit offset 0.043 m (limit 0.05), rotation 0.015° (limit 0.2), 100 % of occupied cells within 0.10 m of a face (P50 0.012 m, P95 0.055 m, max 0.06 m), no ghost cells, origin yaw 0 |

Check it yourself, from the workspace root:

```bash
ros2 run warehouse_layout check_map src/warehouse_navigation/maps/warehouse.yaml \
  --overlay /tmp/overlay.png
```

The overlay draws the layout's faces in blue over the map: every wall of this map lies on them.

## How it was made

From the workspace root, each command in its own terminal, in this order:

```bash
# 1. The warehouse with slam_toolbox mapping. headless:=true runs Gazebo without a window
#    and without RViz, as for the runs below; leave it out to watch the map grow in RViz.
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam headless:=true

# 2. Wait until slam_toolbox reports active [3] (1.5 s after the launch in these runs; the
#    first /map came after 7 to 9 s).
ros2 lifecycle get /robot1/slam_toolbox

# 3. Record the run. The Phase 2 time-lapse GIF is made from this bag.
#    --include-unpublished-topics: subscribe to /robot1/cmd_vel now, before drive_route
#    publishes on it, so the bag holds the first command.
ros2 bag record -o bags/mapping --use-sim-time --include-unpublished-topics --topics /map \
  /robot1/ground_truth/odom /robot1/tf /robot1/tf_static /clock /robot1/scan /robot1/odom \
  /robot1/odom/wheel /robot1/cmd_vel

# 4. Drive the survey route, about 9 min 40 s.
ros2 run warehouse_system_tests drive_route --route survey --speed 0.4 --turn-rate 0.2

# 5. 10 s after the route is done (slam_toolbox takes the last scans and publishes the map
#    every 2 s), save the map.
ros2 run nav2_map_server map_saver_cli -f src/warehouse_navigation/maps/warehouse -t /map \
  --fmt pgm --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0

# 6. Stop the recorder and the simulation (Ctrl-C in their terminals), then check the map
#    and look at the overlay.
ros2 run warehouse_layout check_map src/warehouse_navigation/maps/warehouse.yaml \
  --overlay /tmp/overlay.png
```

Then build `warehouse_navigation` again (`colcon build --symlink-install --packages-select
warehouse_navigation`): a symlink install links the files that exist when it is built, so a
map saved later reaches the install only after the next build.

map_saver_cli warns "Free threshold unspecified" and saves with its defaults, occupied 0.65 and
free 0.25. That is expected. The YAML it writes says `free_thresh: 0.196`, so that the grey
unknown cells (205) load back as unknown, not as free.

Four things matter, each seen in these runs or in the earlier attempt below:

- **Turn slowly.** The EKF's `odom -> base_footprint` lags ground truth by about 19 ms (the
  medians per run and turn direction: 18.0 to 20.4 ms). A scan taken while turning is placed
  that far behind: 0.22° at 0.2 rad/s, 0.54° at drive_route's default 0.5 rad/s. The survey
  turns 1037° left and only 407° right, so these errors do not cancel.
- **Map alone on the laptop.** In an earlier attempt, a second Gazebo running at the same time
  stalled the wheel odometry for 0.08 to 0.35 s at times.
- **Start no ROS tools while driving.** Every `ros2` command (topic echo or hz, lifecycle get,
  map_saver_cli) joins the DDS graph when it starts. In these runs that paused the
  controller's odometry for 0.07 to 0.28 s each time (seen after the drive, when the commands
  of steps 2 and 5 ran). In a short check run, a recorder that subscribed to
  `/robot1/cmd_vel` in the middle of a drive delayed two commands by 0.18 and 0.29 s, so the
  controller ignored them. Start everything before step 4.
- **`save_map_timeout:=10.0`.** map_saver_cli waits only 2 s by default for the map, and the
  0.25 MB message sometimes takes longer to arrive: once it failed with "Failed to spin map
  subscription".

## The mapping runs

All three on 2026-11-07 with the commands above, alone on the laptop (i5-1335U), headless:

| Run | Started | Sim / wall time | RTF | Offset (max 0.05 m) | Rotation (max 0.2°) | Within 0.10 m (P95) | Ghosts | As saved: P50 / P95 |
|---|---|---|---|---|---|---|---|---|
| 1, this map | 18:29 | 9:40 / 9:41 | 0.999 | **0.043 m** | 0.015° | 100 % (0.055 m) | 0 | 0.014 / 0.063 m |
| 2 | 18:41 | 9:40 / 9:41 | 0.999 | **0.054 m, fails** | 0.031° | 100 % (0.044 m) | 0 | 0.052 / 0.071 m |
| 3 | 18:54 | 9:41 / 9:41 | 0.999 | **0.047 m** | 0.125° | 100 % (0.049 m) | 0 | 0.030 / 0.080 m |

- **RTF** is the real-time factor while driving, from Gazebo's world statistics every 10 s:
  0.995 to 1.000 in every 10 s window.
- **As saved** is the distance of the occupied cells to the layout's faces with no fit: how
  well the map lies on the world as it is, which is what AMCL will see. Run 1 lies best.

All runs used the same `slam_toolbox.yaml` (this commit). drive_route did not yet wait 3 s
before its first command (see warehouse_system_tests); that changes only the first seconds,
while the robot stands at the origin.

An earlier attempt made three maps of the same shape and lost the files in a restart. It used
the sample's 0.2° fine search step (0.05° from its second run), turned at 0.5 rad/s and shared
the laptop with another Gazebo at times. Its offsets were 0.048, 0.051 and 0.045 m, and its
rotations 0.249, 0.125 and 0.289°.

## What the runs show

- **The shape is right.** Every map has all its occupied cells within 0.11 m of a face of the
  layout (P95 0.044 to 0.055 m after the fit) and no ghost walls: the loop closes and no aisle
  is merged with the next.
- **The rotation is fixed.** 0.015 to 0.125° now, against 0.125 to 0.289° before. Two changes
  made that. slam_toolbox's `fine_search_angle_offset` is, despite its name, the angle step of
  the fine scan match: at 0.2° steps it cannot correct a heading error under about 0.1° (half
  a step), and in the earlier attempt `map -> odom` kept yaw 0 for the first 130 s. At 0.05°
  the first heading correction came 10 to 12 s into every run. The second change: turns at
  0.2 rad/s. Part of what is left comes from the start: slam_toolbox gives the map the EKF's
  heading at its first scan, and the EKF, drifting slowly while the robot stood, said +0.012°,
  -0.003° and -0.059° there (runs 1 to 3; ground truth 0). That is about half of run 3's
  0.125°.
- **The offset is close to the limit and points the same way every time.** check_map moves
  every map south-west to fit it: run 1 by (-0.022, -0.036) m, run 2 by (-0.036, -0.040) m,
  run 3 by (-0.020, -0.042) m (the earlier attempt's second map by (-0.023, -0.046) m). Every
  run starts at the world origin (ground truth (0, 0); the EKF (0, 0) at slam_toolbox's first
  scan; the first `map -> odom` the identity), so the shift builds up while mapping. SLAM's
  own pose shows it too, and it depends on where the robot heads: 0.3 to 2 cm off while
  driving east, 7 to 8 cm while driving west. The cause is not found yet. AMCL on this map
  shows no such bias: in both acceptance runs of 2026-11-08 its mean error against ground truth
  was (+0.002, -0.009) m, and its largest error after convergence 0.092 and 0.090 m (plan F7
  limit 0.10 m; the warehouse_navigation README has the numbers).

## A map of your own by teleop

Making a map by teleop is an exercise of the Phase 2 report. Save your map under another name,
so it does not replace this one:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam
ros2 run warehouse_bringup teleop_window
# Drive every aisle and come back to the start. Turn slowly (see above).
ros2 run nav2_map_server map_saver_cli -f /tmp/teleop_map -t /map --fmt pgm \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
ros2 run warehouse_layout check_map /tmp/teleop_map.yaml --overlay /tmp/teleop_overlay.png
```

## A map without SLAM

`rasterize` draws the map an ideal SLAM run would make, straight from the layout (plan risk R6).
AMCL can use it with `map:=`:

```bash
ros2 run warehouse_layout rasterize --out /tmp/layout_map
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl map:=/tmp/layout_map.yaml
```
