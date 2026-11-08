# Phase 2 report: the warehouse world, its map and localization

- **Milestone:** M1 Core AMR (Phases 1-6; this is the second)
- **Dates:** 2026-11-05 (plan) to 2026-11-08
- **Branch / tag:** part 1 (the world) on `phase-2-warehouse-world`, merged with pull request #4
  on 2026-11-06; part 2 (mapping and localization) committed straight to `main`
  ([ADR-008](../decisions/ADR-008-develop-on-main-without-automated-tests.md)); tag `phase-2`
  not yet created (as of 2026-11-08)
- **Result:** the warehouse is generated from one file, `layout.yaml`: a 30 x 20 m hall with 8
  shelf rows, 4 pick aisles, a packing station, 3 docks and spawn poses for 3 robots, checked
  against the robot by 9 fit rules. The robot mapped it with slam_toolbox on a scripted route,
  and the map lies within 0.043 m and 0.015° of the layout. AMCL localizes on that map: started
  0.42 m and 8° off, it found the robot within 0.78 m of driving and then stayed within 0.092 m
  of ground truth for a 7-minute drive. Every acceptance criterion was measured and met; the
  mapping time-lapse is [docs/media/phase-02-mapping.gif](../media/phase-02-mapping.gif).

This report is written for someone who has read the [Phase 1 report](phase-01-robot-description.md)
and knows the robot, TF, ros2_control and sim time from there. It explains what Phase 2 built,
why each piece exists, what went wrong on the way, and how to try every piece yourself. Paths are
relative to the repository root unless they start with `~/warehouse_ws`. "The plan" is
[docs/PROJECT_PLAN.md](../PROJECT_PLAN.md). "The Phase 2 plan" is the detailed working plan the
phase was built from (not in the repository); its decision IDs (P2-D1 to P2-D16) and risk IDs
(R2-1 to R2-16) appear in code comments and READMEs, and this report explains the ones that
matter where they come up.

The commands in [Try it yourself](#try-it-yourself) were run on 2026-11-08 on the development
laptop, in a clean shell like `wh_shell` (its own ROS domain), with Gazebo headless. The two long
drives (exercises 5 and 8, about 10 minutes each) were run on the short smoke route instead; their
expected output comes from the acceptance runs of 2026-11-07 and 2026-11-08, made with the same
commands. Not run: the windowed parts (Gazebo's window, RViz, the teleop window driven by hand)
and `xdg-open`.

> **Words you will meet** (full list in the [Glossary](#glossary))
>
> - **layout file:** `layout.yaml`, the one file that describes the warehouse; everything else
>   about the warehouse is generated from it.
> - **fit rule:** a check that the warehouse suits our robot, computed from the robot's own
>   size, for example "pick aisles are exactly 1.6 m wide".
> - **occupancy grid:** a map as a grid of cells, each free, occupied or unknown.
> - **SLAM:** simultaneous localization and mapping: building a map while working out where the
>   robot is in it.
> - **AMCL:** adaptive Monte Carlo localization: finding the robot on a finished map with a
>   cloud of guesses (particles).
> - **lifecycle node:** a node that starts doing nothing and must be configured and activated
>   from outside.
> - **transient local:** a QoS setting with which a publisher keeps its last message for
>   subscribers that join later.
> - **bag:** a recording of ROS topics (rosbag2), here in the MCAP file format.
> - **ground truth:** the robot's true pose, straight from Gazebo; used only to check estimates.

## Contents

- [TL;DR](#tldr)
- [What was built](#what-was-built)
- [Concepts](#concepts)
  1. [One layout file and its fit rules](#1-one-layout-file-and-its-fit-rules)
  2. [Generated files: committed, byte-identical, checked](#2-generated-files-committed-byte-identical-checked)
  3. [The world in SDF, as the lidar sees it](#3-the-world-in-sdf-as-the-lidar-sees-it)
  4. [One frame for map, world and layout](#4-one-frame-for-map-world-and-layout)
  5. [Occupancy grids and map files](#5-occupancy-grids-and-map-files)
  6. [SLAM with slam_toolbox](#6-slam-with-slam_toolbox)
  7. [Lifecycle nodes: launch events or a lifecycle manager](#7-lifecycle-nodes-launch-events-or-a-lifecycle-manager)
  8. [map_server and transient local](#8-map_server-and-transient-local)
  9. [AMCL, a particle filter](#9-amcl-a-particle-filter)
  10. [The TF chain map -> odom -> base_footprint](#10-the-tf-chain-map---odom---base_footprint)
  11. [System scope and robot scope](#11-system-scope-and-robot-scope)
  12. [Checking a map against the layout](#12-checking-a-map-against-the-layout)
  13. [Scripted drives](#13-scripted-drives)
  14. [rosbag2 and evaluation from a bag](#14-rosbag2-and-evaluation-from-a-bag)
  15. [IMU noise from a robot standing still](#15-imu-noise-from-a-robot-standing-still)
- [What happened](#what-happened)
- [Measured results](#measured-results)
- [Try it yourself](#try-it-yourself)
- [Pitfalls and lessons](#pitfalls-and-lessons)
- [Glossary](#glossary)
- [Further reading](#further-reading)
- [What comes next](#what-comes-next)

## TL;DR

Phase 2 built the place the robot works in, and taught the robot where it is in it:

- **The warehouse from one file.** `warehouse_layout/config/layout.yaml` (203 lines) describes the
  hall, 8 shelf rows (A-H) in two blocks, the packing station, 3 docks, 3 spawn poses, keepout and
  speed zones and two scripted routes. The `warehouse_layout` package derives the rest (69 bays,
  414 slots with IDs such as `C07-2-1`, 36 pick poses), checks 9 fit rules computed from the
  robot's `robot.yaml`, draws a to-scale preview, and generates the Gazebo world, a location
  registry and an expected inventory. The generated files are committed, byte-identical from run
  to run, and `generate --check` finds a stale one
  ([ADR-007](../decisions/ADR-007-generate-the-warehouse-from-one-layout.md)).
- **One frame for everything.** The layout is written in the map frame, which is the Gazebo world
  frame, with its origin where robot1 spawns. A map made from there lines up with the world and
  the layout by construction.
- **A map, made by script and checked by a number.** slam_toolbox maps the warehouse while
  `drive_route` drives a 176 m survey route; `check_map` fits the saved map to the layout and
  reports how far it is off: 0.043 m and 0.015° for the committed map (limits 0.05 m and 0.2°).
- **Localization on the map.** `map_server` serves the map on `/map` for every robot, and AMCL in
  `/robot1` publishes the `map -> odom` correction above the EKF. Tuned on recordings: 240 beams
  per scan and an odometry noise of 0.02 instead of Nav2's 60 and 0.2.
- **Evaluation from bags.** `evaluate_localization` compares AMCL with Gazebo's ground truth from a
  recording and estimates the IMU's noise from the seconds the robot stands still: within 3 % of
  `robot.yaml` on every axis.

The most important lesson: **a default belongs to someone else's robot, and a parameter's name
is not its meaning.** slam_toolbox's `fine_search_angle_offset` sounds like a search range and is
a step size; Nav2's odometry noise suits wheels that slip, not this simulation; the first commands
of a fresh publisher vanish in DDS discovery. Each was found by recording a run, measuring it, and
changing one thing at a time on identical input.

## What was built

| Piece | Files | Purpose |
|---|---|---|
| Robot numbers for Python | `warehouse_description/warehouse_description/robot_config.py` | `robot.yaml` plus derived values (padded radius 0.456 m, scan plane 0.34 m) for Python code |
| The layout | `warehouse_layout/config/layout.yaml` | The single source of the warehouse |
| Model, fit rules, preview | `warehouse_layout/warehouse_layout/{model,rules,geometry,preview}.py` | Load and check the layout, derive slots and poses, check 9 fit rules, draw it (`ros2 run warehouse_layout preview`) |
| Generators | `warehouse_layout/warehouse_layout/{cli,world,registry,inventory,formatting}.py` | `ros2 run warehouse_layout generate [--check]` |
| Generated files (committed) | `warehouse_gazebo/worlds/warehouse.sdf`, `warehouse_layout/generated/{locations,expected_inventory}.yaml` | The world (32 models, 461 visuals, 89 collisions), the location registry, the expected inventory |
| Spawn poses | `warehouse_bringup/launch/sim.launch.py` | `world:=warehouse` spawns each robot at its registry pose |
| Map tools | `warehouse_layout/warehouse_layout/{rasterize,check_map,occupancy}.py` | `rasterize` (the map an ideal SLAM run would make), `check_map` (how well a map fits the layout) |
| SLAM | `warehouse_navigation/config/slam_toolbox.yaml`, `launch/slam.launch.py` | slam_toolbox in the robot scope, started by launch events |
| Localization | `warehouse_navigation/config/{amcl,map_server}.yaml`, `launch/{localization,map_server}.launch.py` | AMCL in the robot scope, `map_server` in the system scope, each with a lifecycle manager |
| The map | `warehouse_navigation/maps/warehouse.{yaml,pgm}`, `maps/README.md` | The map AMCL uses, and how it was made |
| Bring-up | `warehouse_bringup/launch/{sim,robot}.launch.py`, `rviz/map.rviz` | `localization:=none\|slam\|amcl`, `map:=`, an RViz view of the map |
| Test tools | `warehouse_system_tests/warehouse_system_tests/{route_driver,localization_eval,map_timelapse,bags}.py` | `drive_route`, `evaluate_localization`, `map_timelapse` |
| Decision | `docs/decisions/ADR-007-generate-the-warehouse-from-one-layout.md` | Why one layout file, committed generated files and one frame |
| Demo | `docs/media/phase-02-mapping.gif` | The map growing during the survey (0.18 MB) |

Each package README explains its files in detail.

## Concepts

### 1. One layout file and its fit rules

**What it is.** Phase 1 described the robot once, in `robot.yaml`, and generated the URDF from it.
Phase 2 does the same for the building: one file holds every number of the warehouse, and every
other description of the warehouse is generated from it or checked against it. The rule from
Phase 1 carries over: *geometry is generated, tuning is validated*.

**How it works here.** Open `src/warehouse_layout/config/layout.yaml`:

- **A few numbers in, many objects out.** The file gives the hall's inner faces
  (`x: [-1.5, 28.5]`, `y: [-2.0, 18.0]`), the shelving dimensions once (`depth: 0.50`,
  `bay_width: 1.00`, `plinth_height: 0.40`), two blocks (where each starts and how many bays it
  has) and eight rows, each just an `id`, the x of its face and the way the face looks
  (`face: +x`). `model.py` turns that into 69 bays, 414 slots, 8 aisle segments and 36 pick
  poses on the aisle centrelines, plus the docks' docked and staging poses, the zones and the
  floor markings.
- **Slot IDs** are `<row><bay>-<level>-<position>`: `C07-2-1` is row C, bay 07, level 2 (the
  first board), position 1 (the south one). Row letters skip I and O, which read like 1 and 0 on
  a label.
- **A strict schema.** `model.py` reads the file into frozen dataclasses and refuses any key it
  does not know, naming it. A typo such as `plinth_heigth` stops the program instead of being
  ignored: the same idea as the `/**/` parameter keys of Phase 1, where a silently ignored
  setting was the worst kind of mistake.
- **Fit rules from the robot's own numbers.** `rules.py` holds nine rules. The robot's numbers
  come from `robot.yaml` through `warehouse_description/warehouse_description/robot_config.py`,
  never typed in: a padded circumscribed radius R = hypot(0.36, 0.28) = 0.456 m, so every goal
  pose needs a free circle of R + 0.10 = 0.556 m; a scan plane of 0.34 m, so everything the lidar
  must see is solid from the floor to 0.34 + 0.06 = 0.40 m; pick aisles exactly 1.60 m, because
  turning in place needs 2 x (R + 0.10) = 1.11 m and the centreline must stay outside Nav2's
  0.70 m inflation.
- **The tag rule caps the aisle width.** In Phase 7 the side cameras (640 px wide, 1.57 rad field
  of view, so a focal length of 320 / tan(0.785) = 320.3 px) must see each 0.12 m tote tag at
  40 px or more. From 0.15 m off the centreline, the far face of a 1.6 m aisle is 0.95 m away: the
  tag is 320.3 x 0.12 / 0.95 = 40.4 px. A tote 2 cm further back would give 39.6 px. That is why
  aisles are exactly 1.6 m and totes stand flush with the shelf face.

`ros2 run warehouse_layout preview` prints one `[PASS]` or `[FAIL]` line per rule and draws the
layout to scale ([exercise 1](#exercise-1-the-layout-and-its-fit-rules)). A failing rule names the
object at fault: narrow aisle 1 by 0.1 m and the output says
`aisle_1_south: 1.500 m between the faces of rows A and B; must be 1.60 m`.

**Why industry does it.** A real site keeps its layout in one place (a CAD drawing, a warehouse
management system) and derives the fleet's maps, zones and location database from it. Copies kept
by hand drift, and a robot sent to a pick pose that moved stops in front of a shelf. Checking the
design against the machine is the idea behind the design-rule check of a circuit board: the rules
are code, and they run every time.

### 2. Generated files: committed, byte-identical, checked

**What it is.** A generator turns the layout into the files other programs read. Three questions
decide how that works: when the files are generated, whether they are kept in git, and how a stale
file is noticed.

**How it works here.** `ros2 run warehouse_layout generate`, run from the workspace root, reads
`src/warehouse_layout/config/layout.yaml` and `src/warehouse_description/config/robot.yaml` (the
source tree, not the install) and writes three files into the source tree (`cli.py`):

```text
unchanged  src/warehouse_gazebo/worlds/warehouse.sdf: 32 models, 461 visuals, 89 collisions
unchanged  src/warehouse_layout/generated/locations.yaml: 3 spawns, 1 drop, 3 docks, 36 picks, 8 aisle segments, 6 keepout and 10 speed zones, 2 routes
unchanged  src/warehouse_layout/generated/expected_inventory.yaml: 207 totes in 414 slots, 60 SKUs
```

- **Committed, not built** (P2-D1). The files are in git, so a layout change shows up in a review
  as a change to the world and the registry, and Gazebo, `sim.launch.py` and later nodes read
  plain files. Each starts with a "do not edit" notice that names the command.
- **Byte-identical.** The same layout must give the same bytes on every run, or every
  regeneration would show changes that are not there. `formatting.py` makes sure of it:
  - numbers are printed with a fixed number of decimals (6 in YAML, 4 in SDF), never with
    `str()`, whose shortest round-trip text depends on how a value was computed (`0.1 + 0.2`
    prints `0.30000000000000004`); `-0.0` is written as `0.0`;
  - lists follow the layout's order, never a set's: Python orders a set of strings differently in
    every process (hash randomization, `PYTHONHASHSEED`);
  - the inventory's randomness comes from `random.Random(seed)` with `seed: 2` from the layout;
  - no dates, and the YAML is written as text, not with `yaml.safe_dump`, so a PyYAML update
    cannot change the files.
- **Checked.** `generate --check` writes nothing; it compares each file with what the layout gives
  now and exits 1, naming the file, if one differs. `generate` rewrites a file only when its text
  changes, so an unchanged file keeps its date.

**Why industry does it.** Generated artifacts in version control trade a little repository size
for reviewability and plain loading; build systems such as Bazel go further and make every
output a pure function of its inputs. Either way, determinism comes first: a generator that writes
a different file each time hides real changes in noise, and no freshness check can work without
it.

### 3. The world in SDF, as the lidar sees it

**What it is.** SDF (Simulation Description Format) is Gazebo's XML format. A `<world>` holds
`<model>`s; a model holds `<link>`s; a link has `<visual>`s (what cameras and the lidar render)
and `<collision>`s (what the physics engine touches). A `<static>` model never moves and costs the
physics engine almost nothing.

**How it works here.** `world.py` writes `src/warehouse_gazebo/worlds/warehouse.sdf`:

- **The same physics and systems as `empty.sdf`.** A world that lists any `<plugin>` loads only
  the systems it lists ([Phase 1, concept 6](phase-01-robot-description.md#6-gazebo-sensors-and-gz-transport)),
  so the generator writes all five.
- **One static model per object** (the walls, each shelf row segment, the packing station, each
  dock, each fixture, the fence, the floor paint) and **one collision box per solid** of the
  layout, named `<solid>_collision`. What a robot can hit is exactly what the fit rules checked
  (P2-D6). sdformat rejects a visual and a collision with the same name in one link
  ("Non-unique names detected"), so names carry `_visual` and `_collision` suffixes.
- **The lidar sees visuals, not collisions.** Gazebo's `gpu_lidar` is a rendering sensor: it
  measures what the renderer draws. So every solid has a visual that fills its footprint from the
  floor to above the scan plane: the shelf plinth (0.40 m), the fence's kick plate (0.50 m), the
  row-end guards (0.45 m). Above the plinth a shelf is open (two square posts per upright, so the
  Phase 7 cameras can look into the bays), and at 0.34 m the lidar sees only the plinth.
- **Floor paint** (keepout tints, the zebra crossing, parking and dock bays, aisle stripes) is
  visual only, 1 to 6 mm thin, each kind on its own layer: two faces in one plane flicker in the
  renderer (z-fighting).
- **Colours in linear light.** `style:` colours are sRGB values, as a designer picks them. Gazebo's
  ogre2 renderer treats material values as linear light and stores the image in sRGB: a surface
  that emits 0.2 shows as pixel value 123, not 51. `linear_rgb()` in `world.py` converts each
  colour first; written unconverted, the dark grey plinth `#3d4148` would show as a mid grey.
- **Every pose in map coordinates, every model at the origin.** A number in the SDF is the number
  in `layout.yaml` and in the preview, and a layout edit changes only the lines of what it moves.

Measured with the robot in it (2026-11-06): Gazebo ran at a real-time factor of 1.01, the scan at
10.000 Hz, and from the spawn the lidar read 17.995 m north (the wall is 18.0 m away), 2.020 m
south (2.0 m) and 1.734 m west (1.75 m): within 2 cm, single readings of a lidar with 0.01 m of
noise.

**Why industry does it.** Simulation cost grows with what the physics engine must check, so real
sites are simulated the same way: detailed visuals for the cameras, a few simple boxes for
contact. And a sensor model is only as honest as what it can see: if the lidar renders visuals,
the world must put solid visuals wherever the real sensor would see something solid.

### 4. One frame for map, world and layout

**What it is.** REP-105 names the frames of a mobile robot: `map` (fixed to the world, globally
consistent), `odom` (smooth, but drifts) and `base_footprint` (the robot). Somewhere a decision
must say where `map` is in the building.

**How it works here.** The map frame is the Gazebo world frame, and its origin is robot1's spawn
pose, facing east (+x) (P2-D2):

1. `layout.yaml` is written in those coordinates, and the fit rule `spawns` keeps robot1 at
   (0, 0, 0). The hall's inner faces still have round numbers (x -1.5 / 28.5, y -2.0 / 18.0).
2. `sim.launch.py world:=warehouse` reads robot1's pose from the generated registry
   (`warehouse_layout/generated/locations.yaml`, entry `spawn_robot1`) and spawns the robot there.
   It does so in an `OpaqueFunction`, a Python function that launch calls once every argument has
   its value: a default computed from another package's file needs Python, not a substitution.
   Values given on the command line (`x:=`, `y:=`, `yaw:=`) win, and a namespace with no spawn
   pose stops the launch before Gazebo starts.
3. The EKF starts `odom` where the robot stands, so at the start `odom` is the world frame.
4. slam_toolbox gives its first scan that scan's odometry pose, so its `map` frame starts as
   `odom`, which is the world frame. AMCL starts at the spawn pose from the same registry.

So the saved map lines up with the world and the layout with no correction step: its origin's yaw
is 0, and `check_map` has to move it only 0.043 m and 0.015° to fit the layout. Whatever error the
EKF has at slam_toolbox's first scan goes into the map: in the three mapping runs the EKF's
heading there was +0.012°, -0.003° and -0.059°, against ground truth 0.

**Why industry does it.** Everything drawn on a map (keepout zones, pick locations, a fleet
dashboard) needs the map's frame. Fixing the frame by construction removes a manual alignment step
that is easy to forget after the next remapping.

### 5. Occupancy grids and map files

**What it is.** An occupancy grid divides the floor into square cells and stores for each one
whether it is free, occupied or unknown. In ROS it travels as `nav_msgs/OccupancyGrid`: `info`
holds the cell size (`resolution`), `width`, `height` and the `origin` (the pose of the
lower-left corner of cell (0, 0)); `data` holds one value per cell, row by row from the origin:
`-1` unknown, `0` free, `100` occupied (1-99 for probabilities).

**How it works here.** On disk a map is two files in `map_server`'s format. The YAML,
`src/warehouse_navigation/maps/warehouse.yaml`:

```yaml
image: warehouse.pgm
mode: trinary
resolution: 0.050
origin: [-1.612, -2.089, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

and the PGM, a greyscale image with one byte per cell and the northern row first (an image runs
top-down, the grid bottom-up). In `trinary` mode a pixel is 0 (black, occupied), 254 (free) or
205 (grey, unknown). When it loads a map, `map_server` computes an occupancy
p = (255 - value) / 255 for each pixel: above `occupied_thresh` the cell is occupied, below
`free_thresh` free, in between unknown. For an unknown pixel p = 50 / 255 = 0.196, so with the
common `free_thresh: 0.25` the grey would load as free floor, and everything behind the walls
would become drivable. `map_saver_cli` writes 0.196 in trinary mode for exactly that reason, and
so does `rasterize`.

The committed map has 611 x 403 cells of 0.05 m (30.55 x 20.15 m): 7,285 occupied, 200,978 free
and 37,970 unknown.

`rasterize` (`warehouse_layout/warehouse_layout/rasterize.py`) draws the same kind of map straight
from the layout. It cuts every solid at the scan plane and marks the faces the lidar can see as
occupied (a solid cell next to a free cell), the hall floor free, and the inside of solids and
everything behind the walls unknown: the map a perfect SLAM run would make. It has 636 x 428
cells, 6,473 of them occupied, and its origin sits on the cell grid, so cell edges fall on the
layout's round numbers ([exercise 3](#exercise-3-map-files-and-check_map)).

**Why industry does it.** Occupancy grids are the common currency of 2D navigation: Nav2's
costmaps, AMCL's likelihood field and every map viewer read them. Plain image files make maps easy
to inspect, edit and diff, which is also why a wrong threshold can quietly turn walls into floor.

### 6. SLAM with slam_toolbox

**What it is.** SLAM builds the map while it works out where the robot is in it; localization
(concept 9) uses a finished map and does not change it. slam_toolbox, the SLAM package Nav2's
documentation uses, is a *graph* SLAM:

- **Nodes** are robot poses, each with the scan taken there. A new node is added after the robot
  has moved `minimum_travel_distance` or turned `minimum_travel_heading`.
- **Edges** say how two poses relate. Each new scan is matched against recent scans: the scan
  matcher tries shifts and turns of the scan within a search window and scores how well it lands
  on what was already seen (a correlative scan matcher). The best match becomes an edge.
- **Loop closure.** When the robot comes back near a pose it has seen before (within
  `loop_search_maximum_distance`), slam_toolbox matches the new scan against a chain of old ones.
  If the match is good enough (`loop_match_minimum_response_fine`), it adds an edge between old
  and new, and the **Ceres** optimizer moves every pose so that all edges fit as well as possible.
  That removes the drift that built up around the loop.
- **The map** is drawn from the corrected poses and their scans, and published every
  `map_update_interval`.
- **`map -> odom`.** slam_toolbox publishes the transform that puts the drifting `odom` frame
  where its corrected pose says the robot is (concept 10).

`async_slam_toolbox_node` is the online, asynchronous variant: it processes scans as fast as it
can and skips the ones it cannot keep up with, so it never falls behind.

**How it works here.** `src/warehouse_navigation/config/slam_toolbox.yaml` is slam_toolbox 2.8.5's
own sample (`mapper_params_online_async.yaml`) with every key written out, because a key left out
takes the node's built-in default, not the sample's, and several differ
(`minimum_distance_penalty` is 0.05 built in and 0.5 in the sample). The changes, each marked in
the file:

| Key | Sample | Ours | Why |
|---|---|---|---|
| the node's key | `slam_toolbox:` | `/**/slam_toolbox:` | the plain key matches only a node in `/`, not `/robot1/slam_toolbox` |
| `scan_topic` | `/scan` | `scan` | relative, so it resolves to `/robot1/scan` |
| `minimum_travel_distance`, `minimum_travel_heading` | 0.5 m, 0.5 rad | 0.3 m, 0.3 rad | more nodes in 4-5 m aisles |
| `loop_search_maximum_distance` | 3.0 m | 2.0 m | 3.0 m reaches into the next aisle, 2.7 m away: a false loop closure there would merge two aisles |
| `loop_match_minimum_response_fine` | 0.45 | 0.5 | fewer false loop closures between aisles that look alike |
| `fine_search_angle_offset` | 0.00349 rad (0.2°) | 0.00087 rad (0.05°) | the fine match's angle step ([problem 4](#4-the-map-came-out-turned)) |
| `map_update_interval` | 5 s | 2 s | the map grows smoothly in RViz and in the GIF |
| `use_map_saver`, `enable_interactive_mode` | true, true | false, false | maps are saved with `map_saver_cli`; no markers for editing the graph |
| `min_laser_range` | 0.0 | removed | slam_toolbox 2.8.5 never reads it; each scan's own `range_min` (0.10 m) applies |

The warehouse itself was designed for SLAM. Aisles are at most 5.25 m long and open at both ends,
so the lidar always sees structure beyond them; rows A, D and E are shorter in the north block, a
fire point stands back from two faces, and the pilasters are irregularly spaced, so no two places
look alike. The survey route (`routes: survey:` in the layout, 176 m) first drives one big loop
round the outside, which closes in the feature-rich south-west corner, then a serpentine through
the four aisles and the mid cross aisle back to the start.

Measured (2026-11-07, three runs): slam_toolbox active 1.5 s after the launch, the first map 7-9 s
after it; 175.6 m driven in 9 min 40 s of sim time at a real-time factor of 0.999; the loop
closed, no aisle merged, no ghost wall in any map. The time-lapse
([docs/media/phase-02-mapping.gif](../media/phase-02-mapping.gif)) shows the map of run 1 growing.
The first frames already show most of the hall: the lidar reaches 20 m.

**Why industry does it.** Deployed AMRs are usually mapped once, by a person driving them or by a
scripted survey, and then localize on that static map (plan D3). Graph SLAM is the standard
because the graph can be corrected after the fact: a loop closure fixes every pose at once, which
a filter that keeps only the newest pose cannot do.

### 7. Lifecycle nodes: launch events or a lifecycle manager

**What it is.** A lifecycle (managed) node starts *unconfigured* and does nothing until someone
asks it to *configure* (read parameters, create publishers and subscriptions; it is then
*inactive*) and then to *activate* (start working: *active*). `deactivate`, `cleanup` and
`shutdown` go back down. Each request is a service call on the node (`/<node>/change_state`).
slam_toolbox, `map_server` and AMCL are all lifecycle nodes in Jazzy, so something has to drive
them up.

**How it works here.** Two different somethings:

- **slam_toolbox, by launch events** (`src/warehouse_navigation/launch/slam.launch.py`). The node is
  a `LifecycleNode` (launch_ros requires a `namespace` argument; an empty one is relative, so the
  node lands in `/robot1`). The launch file emits a `ChangeState` event with
  `TRANSITION_CONFIGURE` at start-up, and registers an `OnStateTransition` handler that emits
  `TRANSITION_ACTIVATE` when the node reaches `inactive`. `matches_action(slam_toolbox)` picks the
  node by the launch action that started it, so the request finds it in whatever namespace it
  landed. The handler is registered before the configure event is emitted, so it cannot miss the
  transition. Nothing watches the node afterwards (P2-D8).
- **map_server and AMCL, by Nav2 lifecycle managers** (`map_server.launch.py`,
  `localization.launch.py`). `lifecycle_manager_map` and `lifecycle_manager_localization` read
  `node_names` and `autostart: true`, configure and activate their node, and then watch it through
  a **bond**: both sides send heartbeats, and if one stops for `bond_timeout` (4 s), the manager
  brings its nodes down and waits up to 10 s for them to come back. A manager also offers
  services: `~/manage_nodes` (`nav2_msgs/srv/ManageLifecycleNodes`: STARTUP 0, PAUSE 1, RESUME 2,
  RESET 3, SHUTDOWN 4, CONFIGURE 5, CLEANUP 6) and `~/is_active`
  ([exercise 4](#exercise-4-map_server-lifecycle-and-transient-local)).

Why the difference: a loop-closure optimization can keep slam_toolbox busy for a while, and under
a bond a late heartbeat would make the manager shut the mapping down. Upstream does the same:
Nav2's `slam_launch.py` includes slam_toolbox's own launch file, which uses launch events.

Measured in Gazebo: slam_toolbox `active [3]` 1.5-1.7 s after the launch, `map_server` 1.5-1.7 s,
AMCL 2.4-4.9 s.

**Why industry does it.** Lifecycle nodes give a system manager control over the order things
start in, and a clean way to stop or restart one part without killing its process. Bonds add
liveness: a crashed map server is noticed within seconds, not when a robot drives without a map.

### 8. map_server and transient local

**What it is.** QoS *durability* decides what a subscriber gets when it joins late. With
*volatile* (the default) it gets only messages published after it joined. With *transient local*
the publisher keeps its last messages (here one, `depth: 1`) and hands them to every subscriber
that joins later, if the subscriber asks for transient local too. The rules:

| Publisher | Subscriber | Result |
|---|---|---|
| transient local | transient local | connected; the late subscriber gets the kept message |
| transient local | volatile | connected; the late subscriber gets only new messages |
| volatile | transient local | **not connected at all** (incompatible QoS) |

**How it works here.** `map_server` (`src/warehouse_navigation/config/map_server.yaml`) loads the
map file and publishes it on `/map` once, when it activates, reliable and transient local.
slam_toolbox publishes `/map` the same way, every 2 s. AMCL, RViz and `map_saver_cli` often start
later, and they still get the map because they subscribe transient local
([exercise 4](#exercise-4-map_server-lifecycle-and-transient-local)):

- `ros2 topic echo /map --qos-durability transient_local --qos-reliability reliable --once` printed
  the map's `info` 2 s after it started, long after the map had been published;
- the same with `--qos-durability volatile` printed nothing in 8 s (exit code 124 from `timeout`).

Two surprises came out of the measurements. A *paused* map server (`inactive`) still hands the map
to a new transient-local subscriber: deactivating stops new publishing, but the DDS writer keeps
the message it already holds. And the same rules bit the bag recorder: `/robot1/tf_static` is
transient local and sent once, and a `ros2 bag record` that subscribed to it before the publisher
existed (`--include-unpublished-topics`) subscribed volatile and recorded none of it
([problem 11](#11-a-bag-without-tf_static)).

**Why industry does it.** "Latched" data (maps, the robot description, static transforms,
configuration) must reach nodes that start in any order or restart later. Transient local makes
that the middleware's job instead of each node's.

### 9. AMCL, a particle filter

**What it is.** AMCL (adaptive Monte Carlo localization, Nav2's `nav2_amcl`) keeps many guesses of
the robot's pose, the *particles* (500 to 2000 here). One filter update has three steps:

1. **Predict.** Move every particle by the odometry's motion since the last update, plus random
   noise. The motion model (`nav2_amcl::DifferentialMotionModel`) scales that noise with
   `alpha1`-`alpha4`: rotation noise from rotation, rotation from translation, translation from
   translation, translation from rotation.
2. **Correct.** Weigh each particle by how well the scan, seen from that pose, matches the map.
   The *likelihood field* model takes `max_beams` of the scan's rays, puts each ray's end point on
   the map as seen from the particle, and looks up how far it lies from the nearest occupied cell
   (a distance table computed once per map). Close to a wall means likely.
3. **Resample.** Draw a new set of particles, favouring the heavy ones. Unlikely guesses die out,
   and the cloud gathers round the true pose. *Adaptive*: the number of particles shrinks when the
   cloud is tight and grows when it is spread (KLD sampling).

The estimate is the weighted mean of the best cluster of particles, published on `amcl_pose` with
its covariance; the variances of x, y and yaw sit at indices 0, 7 and 35 of the row-major 6 x 6
matrix.

**How it works here.** `src/warehouse_navigation/config/amcl.yaml`:

- **Updates** run after 0.25 m of travel or 0.2 rad of turning (`update_min_d`, `update_min_a`).
  A robot standing still gets no new estimate.
- **The start.** `set_initial_pose: true`, with `initial_pose.x`, `.y` and `.yaw` from the launch,
  which passes the spawn pose: AMCL starts where the robot stands, with no global search over the
  whole map. To start it elsewhere, publish a pose on `/robot1/initialpose`: RViz's
  "2D Pose Estimate" tool does, and so does `drive_route --initial-pose-offset`, with a covariance
  of 0.5² m² and 0.2² rad² so that the particles spread round the wrong pose.
- **Tuned** ([problem 9](#9-amcl-never-settled-with-nav2s-defaults)): `max_beams` 240 (one ray in
  three of the lidar's 720) instead of Nav2's 60, and `alpha1`-`alpha4` 0.02 instead of 0.2.
  Nav2's values suit a real robot whose wheels slip: at 0.2, each 0.25 m update scatters the
  particles by about 0.11 m and 6°, while this simulated robot's odometry drifts about 0.03 % of
  the distance driven.

Measured on the survey route at 0.5 m/s, started (0.3, -0.3) m and 8° off: converged after
0.78 m of driving; then mean error 0.024 m, P95 0.052 m and at most 0.092 m for 7 min 07 s, with
the heading within 3.15°. AMCL used about 12.5 % of one core.

**Why industry does it.** AMCL is the workhorse of 2D localization on a known map: cheap, robust
to sensor noise, and able to say "I am not sure" with a spread cloud instead of a single wrong
answer. Its weak spots are long featureless corridors and repeated geometry, which is why the
warehouse was designed with short aisles and broken symmetry (concept 6).

### 10. The TF chain map -> odom -> base_footprint

**What it is.** The localization does not publish the robot's pose in the map directly. A frame
has one parent, and `base_footprint`'s parent is `odom` (the EKF's transform, 50 Hz, smooth). So
slam_toolbox or AMCL publishes the *correction*: the transform `map -> odom` that puts the
drifting `odom` frame where it belongs. The chain then gives the robot's pose on the map:

```text
map ──(slam_toolbox or AMCL)──> odom ──(EKF)──> base_footprint ──(robot_state_publisher)──> base_link ...
```

Controllers keep using `odom`, which never jumps; anything that needs the map position (Nav2's
planner in Phase 3) looks up `map -> base_footprint`.

**How it works here.**

- Exactly one node publishes `map -> odom`: `localization:=slam` or `localization:=amcl` takes one
  value, so the two never run together.
- Everything goes to the robot's own `/robot1/tf`
  ([Phase 1, concept 10](phase-01-robot-description.md#10-namespaces-and-the-tf-remaps)).
  Measured with `view_frames`: `map -> odom` at 10.2 Hz (AMCL republishes its correction with
  every scan), `odom -> base_footprint` at 50.2 Hz, the wheels at 20.1 Hz. `view_frames` names
  every broadcaster `default_authority`, because TF messages do not carry their publisher's name:
  ask `ros2 topic info -v /robot1/tf` instead.
- **Future-dated transforms.** AMCL stamps `map -> odom` `transform_tolerance` (1.0 s) after the
  scan it comes from, so that lookups slightly newer than the last scan still succeed. That makes
  TF the wrong place to measure AMCL's accuracy: `evaluate_localization` uses `amcl_pose`, which
  carries the scan's stamp (P2-D12).
- At the spawn, before moving, `tf2_echo map base_footprint` printed a translation of
  `[-0.000, 0.000, 0.000]` and a yaw of -0.07°: AMCL at the spawn pose, plus the EKF's heading
  drift over a few seconds of standing.

**Why industry does it.** REP-105 makes localization sources interchangeable: SLAM, AMCL, a GPS
fusion node or a fiducial system all publish the same `map -> odom`, and nothing downstream
changes.

### 11. System scope and robot scope

**What it is.** Plan D20 splits the stack into a system scope (what there is one of: the map, later
the keepout and speed masks) and a robot scope (what each robot has: the EKF, AMCL, later Nav2).
Phase 9 runs three robots by including the robot scope three times.

**How it works here.**

| Scope | Nodes | Started by |
|---|---|---|
| System (`/`) | `map_server`, `lifecycle_manager_map` | `sim.launch.py` with `localization:=amcl` (it includes `map_server.launch.py`) |
| Robot (`/robot1`) | `slam_toolbox` | `robot.launch.py` with `localization:=slam` |
| Robot (`/robot1`) | `amcl`, `lifecycle_manager_localization` | `robot.launch.py` with `localization:=amcl` |

- **Names decide where things meet.** Inside `/robot1` a relative name gets the namespace: `scan`
  becomes `/robot1/scan`. The map is shared, so every map topic is the absolute `/map`: AMCL's
  `map_topic: /map` (its default, `map`, would become `/robot1/map`, where nothing publishes, and
  AMCL would wait for a map forever), slam_toolbox's `map_name: /map` and `map_server`'s
  `topic_name: /map`.
- **RViz's pose tool works per robot**, because RViz runs in `/robot1` and the tool publishes on
  the relative `initialpose`, which becomes `/robot1/initialpose`.
- **Parameter keys start with `/**/`**, so they match the node in any namespace.
- **Mock hardware refuses localization** before anything starts: it simulates the wheels and the
  IMU, but no lidar.
- **Bonds stay in their scope**: the bond topic is relative, so the manager in `/robot1` and the
  one in `/` never see each other's heartbeats.

**Why industry does it.** Fleets run one copy of the shared infrastructure and one stack per
robot. Getting the names right from the start is cheap; renaming topics across a fleet later is
not.

### 12. Checking a map against the layout

**What it is.** To say how far a map is off, each occupied cell must be paired with the wall it
belongs to; but to pair them, you must already know how the map sits on the layout. *ICP*
(iterative closest point) breaks this loop by alternating: pair every cell with the nearest point
of a layout face; solve for the small turn and shift that best moves the cells onto their faces;
move the map; pair again; stop when the map stops moving.

**How it works here.** `ros2 run warehouse_layout check_map <map.yaml>`
(`warehouse_layout/warehouse_layout/check_map.py`):

- The layout's visible faces are sampled every 1 cm (33,433 points). A KD-tree
  (`scipy.spatial.cKDTree`) finds each cell's nearest face point without comparing it with all
  33,433.
- **Point-to-plane:** a cell is pulled across its wall, never along it, because along a wall it
  has no true partner. On this warehouse the fit settles in 4 iterations.
- **A shrinking cut-off** (1.0 m, 0.5 m, then 0.25 m): pairs farther apart take no part, so a ghost
  wall cannot drag the fit.
- **Five limits, one line each:** best-fit offset ≤ 0.05 m (one cell) and rotation ≤ 0.2° say
  where the map sits; 95 % of the occupied cells within 0.10 m of a face, and at most 1 % farther
  than 0.25 m (ghost walls), measured *after* the fit, say what shape it has; and the origin's yaw
  must be 0. A perfect map shifted by 0.15 m fails only the offset line, so the output says which
  problem you have.
- `--overlay` draws the map where the file puts it, the layout's faces in blue, and off-face cells
  as amber squares (0.10-0.25 m away) and red crosses (ghosts).

On the committed map:

```text
best fit after 4 ICP iterations: 7285 of 7285 cells within 0.25 m of a face took part
[PASS] best-fit offset <= 0.05 m: 0.043 m (move the map x -0.022, y -0.036 m)
[PASS] best-fit rotation <= 0.2 deg: 0.015 deg (turn the map +0.015 deg)
[PASS] 95% of occupied cells within 0.10 m of a face: 100.0% (P50 0.012 m, P95 0.055 m, max 0.06 m)
[PASS] at most 1% of occupied cells farther than 0.25 m from every face (ghost walls): 0.00% (0 cells)
[PASS] map origin yaw 0: 0 rad
```

The rasterized map reads P50 0.025 m, half a cell: its occupied cells lie just behind each face.
The SLAM map reads 0.012 m, because SLAM marks the cell in which a beam ends, and that cell can lie
on either side of the face.

**Why industry does it.** "The map looks right" is not a test. Robotics teams compare maps and
point clouds with ICP-style registration all the time (map merging, change detection, sensor
calibration), and a number with a limit can be tracked from run to run.

### 13. Scripted drives

**What it is.** A test tool that drives the robot along a fixed route, the same way every time, so
that mapping and evaluation runs can be repeated and compared.

**How it works here.** `ros2 run warehouse_system_tests drive_route --route survey`
(`warehouse_system_tests/warehouse_system_tests/route_driver.py`):

- The routes come from the generated registry (`routes:` in `layout.yaml`), and the `routes` fit
  rule checks that every waypoint has a 0.556 m free circle and every segment 0.40 m of
  clearance.
- It **steers on Gazebo's ground truth** (`/robot1/ground_truth/odom`), so the path is the same
  whatever SLAM or AMCL believe. That is fine for a test tool, which never becomes part of the
  robot stack (P2-D11).
- At each waypoint it turns in place to face the next segment, then follows the segment line,
  aiming 0.6 m ahead of its own position on the line.
- **Commands at 20 Hz, stamped in sim time**, because `diff_drive_controller` brakes when the
  newest command is older than `cmd_vel_timeout` (0.15 s since 2026-11-06).
- Speeds change at half the controller's acceleration limits, and stops are planned with half of
  that braking again, so a robot that lags behind its commands still stops on the waypoint: in
  13 logged drives it stopped 0.02-0.03 m from the last waypoint.
- **It waits for DDS before the first command** ([problem 7](#7-the-first-commands-were-ignored)).
- `--dry-run` prints the waypoints, the length and the expected time without touching ROS: the
  survey has 19 waypoints and 176.0 m, about 9 min 40 s at 0.4 m/s with turns at 0.2 rad/s.

**Why industry does it.** Repeatable test drives turn "it seemed better" into a measurement. Teams
use recorded routes, test tracks and simulation scenarios the same way.

### 14. rosbag2 and evaluation from a bag

**What it is.** rosbag2 records topics into a file and plays them back; Jazzy's default file format
is MCAP. `ros2 bag record --use-sim-time` stamps the recording with `/clock`; `ros2 bag info`
lists the topics and message counts; `ros2 bag play --clock` replays a bag and publishes `/clock`,
so nodes on sim time run as they did; `rosbag2_py` reads a bag from Python.

**How it works here.**

- **Header stamps, not arrival times.** A message's header stamp says when the measurement was
  taken; the recorder receives it later, by a varying amount. `evaluate_localization`
  (`warehouse_system_tests/warehouse_system_tests/localization_eval.py`) uses header stamps only.
  For each `amcl_pose` it interpolates ground truth (50 Hz) at that stamp and computes the
  position and heading error.
- **The start** is the newest `initialpose` in the bag (the wrong pose drive_route gave AMCL).
  **Convergence** is the first pose after which the error stays below 0.10 m to the end of the
  bag, and the **convergence distance** is the ground-truth path length from the start to it.
  After convergence the tool reports the mean, RMS, P95 and maximum error, the largest heading
  error, and how long and how far the robot drove. For comparison it adds the EKF's drift (the
  EKF's path, put on the ground truth at the start), then one `[PASS]` or `[FAIL]` line per
  criterion.
- **Replays to compare settings.** AMCL's random numbers repeat from start to start, so the same
  input gives the same output: one recorded drive (scans, the EKF's transforms, the initial pose)
  played back through AMCL with `ros2 bag play --clock` reproduced all 464 poses of the live run
  exactly, at twice and at four times real time. That made it possible to try one setting after
  another on identical input, without Gazebo ([problem 9](#9-amcl-never-settled-with-nav2s-defaults)).

**Why industry does it.** Bags are how robotics teams debug and catch regressions: record once,
analyse offline as often as needed, and replay a field failure through a new version of the code.

### 15. IMU noise from a robot standing still

**What it is.** Gazebo adds random (Gaussian) noise to the IMU, with the standard deviations of
`robot.yaml` (gyro σ 0.002 rad/s, accelerometer σ 0.02 m/s²), and `imu_sensor_broadcaster` reports
covariances of σ². A robot that stands still has a true angular rate of 0 and a constant true
acceleration (gravity, on z), so whatever varies is noise, and its standard deviation estimates σ.

**How it works here.** `evaluate_localization` finds the periods in which ground truth stands still
for at least 10 s (`drive_route --settle 30` makes one at the start), drops the first second of
each (the body may still rock), and computes σ per axis about that axis's mean. One detail matters:
the controller manager (100 Hz) and Gazebo's IMU (100 Hz) run on clocks that slip against each
other, so now and then the broadcaster reads the same Gazebo sample twice. A repeated value is not
a new measurement, so it is removed and counted: 506 of 4,018 samples in the acceptance run. The
estimate's own precision is about 1 / sqrt(2n): 1.2 % for the 3,512 samples left.

Measured: gyro σ 0.00204, 0.00200 and 0.00199 rad/s (x, y, z), accelerometer 0.0201, 0.0205 and
0.0198 m/s²; that is 0.99 to 1.03 times `robot.yaml`, and the covariances in the messages agree
with the measured σ² within 5 %.

**Why industry does it.** An EKF weighs each sensor by the covariance it reports, so a wrong
covariance makes the filter trust the wrong sensor. Measuring the noise of a sensor at rest is the
standard first step before tuning any filter (its long form is the Allan variance test).

## What happened

### Timeline

- **2026-11-05, the plan.** The Phase 2 plan drafted the layout, checked it with a prototype of the
  fit rules, and measured a prototype world with a static lidar probe in headless Gazebo:
  real-time factor 1.000 with 262 visuals and 85 collisions, 0.999 with 207 totes added
  (468 visuals). The owner chose a 30 x 20 m hall, bare uniform racks until Phase 7, and a map
  made on the scripted survey route, with a teleop map as a report exercise (exercise 6).
- **2026-11-06, part 1: the world.** `robot_config`, the `warehouse_layout` package (schema, 9 fit
  rules, preview), the three generators, and spawn poses from the registry in `sim.launch.py`.
  Checked in headless Gazebo: the robot at its registry pose to 0 m and 0°, all controllers
  active, the scan at 10.000 Hz with ranges within 2 cm of the layout, real-time factor 1.010,
  and one SIGINT stopped every process. Merged with pull request #4.
- **2026-11-06, part 2 without a simulator.** The map tools, the slam_toolbox, AMCL and map_server
  configurations and launch files with `localization:=`, and the three test tools. The map tools
  were checked on the rasterized map and on maps broken on purpose (shifted, turned, with ghost
  walls), the launch files with `map_server` on its own, and the tools on synthetic bags.
- **2026-11-07, the first mapping attempt.** Three survey runs made maps of the right shape, but
  turned by 0.125-0.289° and offset by 0.045-0.051 m (problems 4 to 6). Its files were lost when
  the laptop restarted (problem 14); its findings were not.
- **2026-11-07, no more tests.** The tests, lint checks and CI were removed and work continued
  straight on `main` ([ADR-008](../decisions/ADR-008-develop-on-main-without-automated-tests.md)).
- **2026-11-07, 18:29-19:05, mapping again.** Three runs with the fine search step at 0.05° and
  turns at 0.2 rad/s; run 1 became the committed map (0.043 m, 0.015°). drive_route learnt to wait
  for DDS before its first command (problem 7).
- **2026-11-07, 19:38-19:49, AMCL with Nav2's values.** The smoke check never converged, and the
  first acceptance run needed 106 m to converge before a suspend cut it short (problems 9, 10).
- **2026-11-08, tuning by replay.** Four recorded drives were played back through AMCL with one
  setting changed at a time; the choice was 240 beams and an odometry noise of 0.02.
- **2026-11-08, 12:25-12:45, AMCL accepted.** The F6 checks and two acceptance runs with the tuned
  values: 4 of 4 criteria passed in both.
- **2026-11-08, documents.** ADR-007, this report, the GIF, the plan and status updates; every
  exercise below was run again.

### The problems, and what they taught

Each story is told as symptom → cause → fix → lesson.

#### 1. Colours came out washed out

- *Symptom:* in camera renders of the prototype world, the dark grey plinths looked mid grey and
  every colour pale.
- *Cause:* the prototype wrote each colour as hex / 255. Gazebo's ogre2 renderer treats material
  values as linear light and stores the image in sRGB, so a value of 0.2 came out as pixel value
  123, not 51.
- *Fix:* `world.py` converts every `style:` colour from sRGB to linear light (`linear_rgb()`).
- *Lesson:* a colour value is an input to a renderer, not a colour (Phase 1's problem 16 said the
  same about RViz). Push a known value through the pipeline and measure what comes out.

#### 2. A background launch ignored Ctrl-C, and SIGTERM orphaned Gazebo

- *Symptom:* in the first scripted run of the warehouse world, a `ros2 launch` started in the
  background of a script did not stop on SIGINT. Stopped with SIGTERM, it exited but left `gz sim`
  and the clock bridge running.
- *Cause:* a background job of a non-interactive shell starts with SIGINT ignored. And
  `ros2 launch` treats SIGTERM as "exit now": it does not shut its processes down first (it warns
  about orphaned processes).
- *Fix:* every scripted run starts a launch in its own process group (`setsid`) with SIGINT
  restored (`env --default-signal=INT`), stops it with one SIGINT to the group, as a terminal's
  Ctrl-C does, and checks by name that nothing is left.
- *Lesson:* stop a simulation the way a terminal does and look for leftovers; Phase 1's problem 5
  taught the same from the other side.

#### 3. The world could not be restored with git

- *Symptom:* a check edited a generated file by hand to see `generate --check` fail, then could not
  undo it with `git checkout`.
- *Cause:* the generated files were new and not yet committed, so git had nothing to restore.
- *Fix:* `ros2 run warehouse_layout generate` restores them: the same layout gives the same bytes.
- *Lesson:* byte-identical generation is also an undo button ([exercise 2](#exercise-2-generated-files-stay-current)).

#### 4. The map came out turned

- *Symptom:* the first attempt's maps had the right shape (all occupied cells within 0.10 m of a
  wall after the fit, no ghost walls), but were turned by 0.249°, 0.125° and 0.289° (limit 0.2°).
  In the first run `map -> odom` kept a yaw of exactly 0 for the first 130 s, and the map took on
  the EKF's early heading drift.
- *Cause:* despite its name, `fine_search_angle_offset` is not a search range but the angle step of
  the fine scan match, which searches ±1° (half of `coarse_angle_resolution`) round the coarse
  match: Karto's `ScanMatcher::MatchScan` in slam_toolbox's source. With the sample's 0.2° steps
  the matcher cannot see a heading error under about 0.1°, half a step, so it never corrects one.
- *Fix:* 0.00087 rad (0.05°), 41 angles instead of 11. The first heading correction then came
  10-12 s into every run. Together with slower turns (next problem) the rotation dropped to
  0.015-0.125° in three runs.
- *Lesson:* when a parameter's name and its effect disagree, read the source. The name was
  misleading; the code was not.

#### 5. Scans taken while turning landed in the wrong place

- *Symptom:* in the first attempt, slam_toolbox's own heading wandered by up to 1.4°, while the
  EKF's stayed mostly within 0.3°.
- *Cause:* the EKF's `odom -> base_footprint` lags ground truth by about 19 ms (medians of 18.0 to
  20.4 ms per run and turn direction). A scan taken while turning gets the pose of 19 ms earlier:
  0.54° off at drive_route's default 0.5 rad/s, 0.22° at 0.2 rad/s. The survey turns 1037° left
  and only 407° right, so the errors do not cancel.
- *Fix:* the mapping runs turn at 0.2 rad/s (`--turn-rate 0.2`).
- *Lesson:* a timing error becomes a position error in proportion to speed. Where an estimator
  lags, move slowly.

#### 6. Another simulation, and every command-line tool, paused the wheel odometry

- *Symptom:* in the first attempt, the wheel odometry stopped for 0.08-0.35 s ten times, seven of
  them while another Gazebo ran on the laptop. Once it then reported 2 m/s for 0.15 s and the EKF
  jumped 0.3 m (slam_toolbox corrected it within 2 s). In the second attempt, stalls of
  0.07-0.28 s came exactly when a `ros2` command started (`ros2 lifecycle get`, `map_saver_cli`),
  and a bag recorder that subscribed to `/robot1/cmd_vel` in the middle of a short drive delayed
  two commands by 0.18 and 0.29 s, so the controller ignored them.
- *Cause:* a second simulation competes for the CPU, and every `ros2` command is a new DDS
  participant that sets off discovery traffic in every node. Why the controller's odometry pauses
  then is not known yet.
- *Fix:* map alone on the laptop, start every tool before the drive, and run nothing during it
  (the rules are in [maps/README.md](../../src/warehouse_navigation/maps/README.md)).
- *Lesson:* in simulation the measuring tools are part of the system under test. Start them first.

#### 7. The first commands were ignored

- *Symptom:* in mapping run 3, the first four commands of drive_route reached the controller
  0.33-0.89 s late; the controller ignored them as too old, and the robot started 0.97 s after the
  first command. drive_route already waited until its publisher counted a subscriber.
- *Cause:* DDS matches the two ends of a connection separately. The publisher had matched the
  controller's subscription, but the controller's end had not yet matched the new publisher; it
  dropped the first samples, and reliable delivery sent them again later, by then older than
  `cmd_vel_timeout` (0.15 s). The count was misleading too: a bag recorder subscribed to
  `/robot1/cmd_vel` counts as a subscriber.
- *Fix:* drive_route now waits until ROS lists `diff_drive_controller`'s subscription and its
  publisher is matched with every subscription listed, and then until the publisher is 3 s old
  (`DISCOVERY_SETTLE`). In 6 of 6 fresh starts the first command then arrived at most 0.001 s
  late, and the wheels moved 0.031-0.043 s after it.
- *Lesson:* a publisher cannot see whether the subscriber's end is ready. Never send something
  important right after creating a publisher.

#### 8. The initial pose could be lost the same way

- *Symptom:* none in a run; found while reading AMCL's subscriptions. AMCL subscribes to
  `initialpose` best effort and volatile, so a pose that arrives before AMCL's end of the match is
  dropped and never sent again. AMCL would then stay at the true spawn pose, and the evaluation
  would report a convergence that never happened.
- *Fix:* drive_route publishes the pose once AMCL is active and the publisher is 3 s old, then
  checks AMCL's answer: AMCL publishes `amcl_pose` after every initial pose, and a robot standing
  still gives it no other reason to. Without an answer within 5 s it publishes again, up to 3
  times. Measured in three runs: the answer came 0.07-0.13 s after the pose.
- *Lesson:* when a message matters, confirm that it had its effect.

#### 9. AMCL never settled with Nav2's defaults

- *Symptom:* with Nav2's sample values (60 beams, odometry noise 0.2), the smoke check never
  converged (0.15 m off at the end), and the first acceptance run needed 106.32 m of driving before
  its error stayed below 0.10 m (the limit is 2 m). AMCL's own spread stayed 0.15-0.47 m and
  8-21°, and the error wandered between 0.05 and 0.25 m, mostly along the aisles.
- *Cause:* Nav2's odometry noise scatters the particles by about 0.11 m and 6° at every 0.25 m
  update, far more than this odometry's drift of about 0.03 % of the distance; and 60 beams (one
  every 6°) hit too few rack ends and end walls to pin the robot down along an aisle.
- *Fix:* recorded drives were played back through AMCL (`ros2 bag play --clock`), one setting
  changed at a time. The replay of the first acceptance drive matched the live run pose for pose
  (464 of 464), so the settings were compared on identical input. The largest error after
  convergence:

  | Setting | First acceptance drive (its first 288 s) | Survey drives 1, 2, 3 |
  |---|---|---|
  | 60 beams, noise 0.2 (Nav2's) | 0.255 m; 141 of 458 poses over 0.10 m | |
  | 120 beams, noise 0.2 | 0.190 m; 89 poses over | |
  | 240 beams, noise 0.2 | 0.192 m; 60 poses over | |
  | 240 beams, noise 0.1 | 0.102 m | |
  | 240 beams, noise 0.05 | 0.088 m | 0.102, 0.096, 0.099 m |
  | **240 beams, noise 0.02 (chosen)** | **0.057 m** | **0.081, 0.094, 0.070 m** |
  | 60 beams, noise 0.02 | 0.089 m | survey 2: 0.074 m |
  | 480 beams, noise 0.02 | 0.090 m | survey 2: 0.067 m |
  | 240 beams, noise 0.01 | 0.068 m | survey 2: 0.086 m |

  The survey drives were the mapping runs' bags, with an initial pose 0.3 m, -0.3 m and 8° off
  added 2 s before the first motion. Two live acceptance runs with 240 beams and 0.02 then passed
  all four checks. On 2026-11-08 Nav2's values were set again at runtime for one smoke drive: the
  error crossed 0.10 m again and again, and AMCL only "converged" at its very last pose, after
  23.82 m ([exercise 9](#exercise-9-nav2s-defaults-and-why-they-were-changed)).
- *Lesson:* tune on recorded input, one parameter at a time; a sample configuration is a starting
  point for someone else's robot. And 0.02 is a simulation value: on a real robot, measure the
  odometry's error first.

#### 10. A suspended laptop voided a run

- *Symptom:* the first AMCL acceptance run stopped in the middle of a turn at waypoint 9, after
  288 s; the evaluation showed a 23° heading error, and drive_route reported no progress.
- *Cause:* the laptop suspended at 19:49:07 (`systemd-logind`: "The system will suspend now!"). On
  battery, the desktop suspends after 15 minutes without input, and Gazebo and every node froze.
- *Fix:* long runs on mains power with the lid open (closing it suspends even with an inhibitor),
  under `systemd-inhibit --what=idle:sleep`. After each run the journal is searched for a suspend,
  and a run with one does not count.
- *Lesson:* a measurement needs a healthy measuring set-up. Check the machine, not only the robot.

#### 11. A bag without tf_static

- *Symptom:* the bag of the first acceptance run held 0 messages on `/robot1/tf_static`.
- *Cause:* the recorder ran with `--include-unpublished-topics` and subscribed before
  `robot_state_publisher` existed, with the default, volatile QoS. `tf_static` is sent once,
  transient local, and a volatile subscriber gets no history (concept 8).
- *Fix:* start the recorder after the launch, without `--include-unpublished-topics`, so it takes
  the publisher's QoS (the recorder of [exercise 8](#exercise-8-measure-localization-from-a-bag)
  does, and its bag got its 1 `tf_static` message), or pin the topic's QoS with
  `--qos-profile-overrides-path`.
- *Lesson:* transient local works only when both ends ask for it.

#### 12. map_saver_cli gave up after 2 s

- *Symptom:* once, `map_saver_cli` failed with `Failed to spin map subscription`.
- *Cause:* it waits 2 s for the map by default (`save_map_timeout`), and the 0.25 MB message
  sometimes takes longer to arrive.
- *Fix:* `-p save_map_timeout:=10.0` in every save command.
- *Lesson:* give a one-shot tool a timeout that fits the size of what it waits for.

#### 13. Parameter files that would have matched nothing

- *Symptom:* none, because it was caught before the first run: copied as they are, slam_toolbox's
  sample and AMCL's defaults would each have done nothing in `/robot1`, without an error.
- *Cause:* the sample's key is the plain `slam_toolbox:`, which matches only a node in `/`; its
  `scan_topic: /scan` is absolute; AMCL's default `map_topic` resolves to `/robot1/map`, where
  nothing publishes. And a key left out of slam_toolbox's file takes the node's built-in default,
  not the sample's value.
- *Fix:* `/**/` keys, relative scan topics, the absolute `/map`, and slam_toolbox's whole sample
  written out (without `min_laser_range`, which version 2.8.5 never reads). Every name was checked
  against the installed packages, and the values were read back from the running nodes with
  `ros2 param get`.
- *Lesson:* ROS quietly ignores a parameter that matches nothing. Check names against the sources
  and read the values back from the running node.

#### 14. The first attempt's files were lost

- *Symptom:* after the laptop restarted, the three maps, the configuration changes and the run
  scripts of the first mapping attempt were gone.
- *Cause:* they lived in a scratch folder under `/tmp`, which is emptied at every boot.
- *Fix:* the second attempt started from the written-down findings; work in progress and
  recordings now live on persistent disk, and bags in `bags/` at the workspace root.
- *Lesson:* keep anything you may need tomorrow on persistent disk. `/tmp` is for minutes, not
  days.

#### 15. An offset that is not explained yet

- *Symptom:* every map has to be moved south-west to fit the layout: by (-0.022, -0.036),
  (-0.036, -0.040) and (-0.020, -0.042) m in the three runs, and by (-0.023, -0.046) m in the
  first attempt's second run. Two of the three runs pass the 0.05 m limit; run 2 failed with
  0.054 m. The committed map passes with 7 mm to spare.
- *What is known:* every run starts exactly at the origin (ground truth (0, 0), the EKF (0, 0) at
  the first scan, the first `map -> odom` the identity), so the shift builds up while mapping.
  slam_toolbox's own pose is 0.3-2 cm off while driving east and 7-8 cm while driving west.
  `check_map`'s measuring from cell centres explains only an estimated 0.009-0.017 m. AMCL on the
  map shows no such bias: its mean error was (+0.002, -0.009) m in both acceptance runs.
- *Status:* open. Leads for Phase 3: compare the Gazebo lidar's ray origin and angles with TF, and
  the EKF's lag on straight segments.
- *Lesson:* report a thin margin as what it is, with the runs that failed, rather than keep the
  run that passed and stop looking.

### Things that look wrong but are not

- `[WARN] [robot1.amcl]: Failed to transform initial pose in time (Lookup would require
  extrapolation into the future ...)`, once per initial pose. The pose carries a stamp a few
  milliseconds newer than the EKF's latest transform, so AMCL cannot add the motion since then and
  adds none. The robot stands still, so none is right. RViz's "2D Pose Estimate" causes it too.
- `ros2 topic info /tf` shows one publisher, `diff_drive_controller`, which never sends anything
  ([Phase 1](phase-01-robot-description.md#things-that-look-wrong-but-are-not)). With `-v` its
  durability reads `TRANSIENT_LOCAL`: it uses the system default QoS, which Fast DDS makes
  transient local for a writer.
- At every Ctrl-C of a launch: `[ERROR] [launch]: Caught exception in launch (see debug for
  traceback): Cannot shutdown a ROS adapter that is not running`. Every process still stops,
  within 1-2 s.
- `process has died [... exit code -2 ...]` for some nodes after Ctrl-C: they received SIGINT
  twice, from the terminal and from launch (Phase 1, problem 12).
- `process[gazebo-1] failed to terminate '5' seconds after receiving 'SIGINT', escalating to
  'SIGTERM'`, then `'SIGKILL'`: the upstream Gazebo shutdown hang of Phase 1's problem 13
  ([gz-sim#4173](https://github.com/gazebosim/gz-sim/issues/4173)); seen once in the runs of
  2026-11-08. Launch still stops everything.
- `map_saver_cli` warns `Free threshold unspecified. Setting it to default value: 0.250000`, yet
  the YAML it writes says `free_thresh: 0.196`: that is the trinary mode's value (concept 5).
- AMCL publishes nothing on `amcl_pose` while the robot stands still (updates need 0.25 m or
  0.2 rad of motion), but `ros2 topic echo /robot1/amcl_pose --once` still prints the last pose:
  AMCL publishes it transient local.
- `ros2 topic info -v /robot1/tf` lists `slam_toolbox` (or `amcl`) twice: the node has more than
  one endpoint on the topic.
- `view_frames` names every broadcaster `default_authority` (concept 10).
- `diff_drive_controller` logs `Velocity command timed out. Braking.` once per second whenever no
  command arrives, before and after every drive.
- RViz logs a GLSL message for the Map display on this laptop's Intel GPU ("active samplers with
  a different type refer to the same texture image unit"); the map is drawn correctly.
- `ros2 lifecycle get` right after a launch may answer `Node not found`: discovery has not
  finished. Run it again.
- drive_route uses about 0.4 of a CPU core while it drives: it is a Python node on sim time, and
  `/clock` ticks 1000 times a second.

## Measured results

Measured on the development laptop (i5-1335U, Intel Iris Xe, headless EGL rendering), in a clean
environment, in sim time, alone on the laptop unless noted. The acceptance criteria are the plan's
Phase 2 acceptance as reworded on 2026-11-08 for local runs (the limits unchanged).

| Acceptance criterion (plan, Phase 2) | Target | Measured | Status |
|---|---|---|---|
| Generated files current | `generate --check` exits 0 | exit 0, `all 3 generated files are current`; the same with `PYTHONHASHSEED=1` and `=2` | met |
| Generation deterministic | the same bytes every run | after a hand edit, `generate` restored the world with the same sha256 | met |
| Fit rules | every rule passes | 9 of 9 | met |
| Map: best-fit offset | ≤ 0.05 m | 0.043 m (the other two runs: 0.054 m and 0.047 m) | met |
| Map: best-fit rotation | ≤ 0.2° | 0.015° (other runs: 0.031°, 0.125°) | met |
| Map: occupied cells within 0.10 m of a face | ≥ 95 % | 100 % (P50 0.012 m, P95 0.055 m, max 0.06 m) | met |
| Map: ghost walls (cells over 0.25 m from every face) | ≤ 1 % | 0 cells | met |
| Map: origin yaw | 0 | 0 | met |
| AMCL convergence, started (0.3, -0.3) m and 8° off | within 2.0 m of driving | 0.78 m (repeat run: 1.22 m) | met |
| AMCL error after convergence, for at least 5 min | under 0.10 m | max 0.092 m over 7 min 07 s and 174.8 m; mean 0.024, RMS 0.029, P95 0.052 m; heading within 3.15° (repeat: max 0.090 m over 7 min 06 s) | met |
| IMU σ on each axis, from ≥ 25 s standing still | within ±20 % of `robot.yaml` | gyro 0.99-1.02 times, accelerometer 0.99-1.03 times, from 40.6 s (repeat: 1.00-1.01) | met |
| GIF | mapping time-lapse, ≤ 3 MB | `docs/media/phase-02-mapping.gif`: 60 frames, 640 x 452 px, 0.18 MB | met |

The Phase 2 plan's localization checks, measured too:

- Lifecycle nodes active within 10 s of the launch: `map_server` after 1.5-1.7 s, AMCL after
  2.4-4.9 s, slam_toolbox after 1.5-1.7 s.
- `map -> odom` only from AMCL, and only on `/robot1/tf`: the publishers of `/robot1/tf` were
  `robot_state_publisher`, `ekf_filter_node` and `amcl`; `/tf` carried 0 messages in 10 s.
- Before the robot moves, AMCL within 0.05 m of ground truth: 0.0000 m and 0.000°.
- A late subscriber still gets the map: subscribed 16.3 s after the launch, it received exactly 1
  message in 10 s.

Additional numbers:

- **Mapping runs** (2026-11-07): 175.6 m in 9 min 40 s of sim time and 9 min 41 s of wall time;
  real-time factor 0.999 (every 10 s window 0.995-1.000); commands at 20.00 Hz with no gap over
  0.1 s while driving; turns of 1037° to the left and 407° to the right; at least 8.57 GiB of
  memory free throughout.
- **Localization runs** (2026-11-08): real-time factor 0.990 and 0.994 (10 s windows 0.991-1.000);
  no gap in the wheel odometry over 0.05 s while driving; EKF drift 0.058 m and 0.026 m after
  175.6 m (0.03 % and 0.01 %); AMCL's mean error (+0.002, -0.009) m in both runs.
- **CPU, share of one core:** AMCL about 12.5 % (240 beams), `map_server` about 9.5 %, the EKF
  about 18 %, drive_route about 37 %.
- **Timing:** the EKF's `odom -> base_footprint` lags ground truth by about 19 ms.
- **Files:** the map's PGM 240.5 KiB; the mapping bag (9 topics including the scan) 218.4 MiB for
  607.8 s; the localization bag (11 topics including the scan) 130.8 MiB for 476.8 s; the GIF
  180,883 bytes, made in about 3 s.
- **The check runs of this report** (2026-11-08, smoke route, 24.0 m): with the tuned AMCL,
  converged after 1.21 m, then at most 0.076 m (mean 0.019 m) for 65 s; with Nav2's values set at
  runtime, "converged" only at the last pose, after 23.82 m; on the rasterized layout map,
  converged after 1.62 m, then at most 0.096 m (mean 0.041 m); slam_toolbox's map of the smoke
  route alone passes `check_map` (0.048 m, 0.069°, P95 0.039 m). Real-time factor 0.983 and 0.967
  in the AMCL runs, 0.892 in the mapping run, while a web browser used about 1.5 cores.

### What was not verified

- **The windowed views.** Gazebo's window and RViz's `map.rviz` were not watched by a person.
  `map.rviz` was loaded once under a virtual display during a short AMCL run (in the first
  mapping attempt): it loaded without errors apart from the GLSL message of
  [Things that look wrong but are not](#things-that-look-wrong-but-are-not), and the map was
  drawn. RViz's "2D Pose Estimate" tool was not used; that it publishes on `/robot1/initialpose`
  was checked in RViz's library only.
- **The teleop map** (exercise 6). Its commands were run (slam launch, the teleop window finding
  the robot and choosing sim time, `map_saver_cli`, `check_map`), but nobody drove by hand.
- **Two causes:** the south-west offset of every map (problem 15) and the odometry pauses when a
  tool starts (problem 6).
- **Repeatability beyond five runs.** Three mapping runs and two acceptance runs; one mapping run
  failed the offset limit by 4 mm, and AMCL's largest error after convergence was 0.057-0.094 m
  in six drives (four replays, two live) against the 0.10 m limit.
- **Global localization** (no initial pose) and a robot moved without AMCL knowing (the "kidnapped
  robot"): out of scope for Phase 2.
- **More than one robot** (Phase 9), and AMCL with real odometry: 0.02 is a simulation value.

## Try it yourself

Each exercise lists the commands and what you should see. Numbers from a simulation differ a
little from run to run; the ones below were measured on the development laptop.

### Before you start

1. **Install the dependencies** (needs `sudo`; Phase 2 adds slam_toolbox, Nav2's AMCL, map server
   and lifecycle manager, its RViz plugins, the MCAP storage plugin and `python3-scipy`):

   ```bash
   source /opt/ros/jazzy/setup.bash
   cd ~/warehouse_ws
   rosdep install --from-paths src --ignore-src -y
   ```

2. **Use the clean shell** `wh_shell` from the
   [Phase 1 report](phase-01-robot-description.md#before-you-start). Every command below runs in
   such a shell, from `~/warehouse_ws`; exercises that need several terminals need `wh_shell` in
   each.
3. **Build and source:**

   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 daemon stop
   mkdir -p bags
   ```

   `colcon build` ends with `Summary: 6 packages finished`. `bags/` at the workspace root is where
   the recordings go; git ignores it.
4. **Long runs** (exercises 5, 6 and 8 take 10 minutes or more): keep the laptop on mains power
   with the lid open, and run nothing else heavy, above all no second simulation
   ([problem 6](#6-another-simulation-and-every-command-line-tool-paused-the-wheel-odometry)). On
   battery the desktop suspends after 15 minutes without input; putting
   `systemd-inhibit --what=idle:sleep` in front of a command keeps the laptop awake while that
   command runs. Right after a 10-minute run, `journalctl --since "-15min" | grep "PM: suspend"`
   prints nothing if the laptop did not sleep during it.

### Exercise 1: the layout and its fit rules

No simulator needed.

```bash
ros2 run warehouse_layout preview --out /tmp/layout.png
xdg-open /tmp/layout.png
```

It prints one line per fit rule, for example:

```text
[PASS] pick aisles exactly 1.60 m between the faces: 8 aisle segments, widths [1.6] m; turning in place needs 1.11 m
[PASS] goal poses: free circle >= 0.556 m, no keepout within 0.456 m: 43 goals; tightest drop_pack_1, 0.600 m from pack_1
[PASS] solid from the floor to >= 0.40 m; no overhang below 1.20 m: 88 solids; below the scan plane by design: contact_bar, dock_plate
...
wrote /tmp/layout.png; 9 of 9 fit rules pass
```

The picture shows the hall to scale: rows A-H, the pick poses as dots on the aisle centrelines,
the keepout and speed zones, the docks, the three spawns, the survey route dashed, and two small
drawings: the robot in a pick aisle with the 0.70 m inflation, and the heights of shelf, robot,
dock and fence against the 0.34 m scan plane.

Now make a mistake, in a copy: move row B's face 0.1 m west, so aisle 1 becomes 1.5 m wide.

```bash
sed 's/x_face: 3.6/x_face: 3.5/' src/warehouse_layout/config/layout.yaml > /tmp/narrow.yaml
ros2 run warehouse_layout preview --layout /tmp/narrow.yaml --out /tmp/narrow.png; echo exit=$?
```

```text
[FAIL] pick aisles exactly 1.60 m between the faces: 8 aisle segments, widths [1.5, 1.6] m; turning in place needs 1.11 m
    - aisle_1_south: 1.500 m between the faces of rows A and B; must be 1.60 m
    - aisle_1_north: 1.500 m between the faces of rows A and B; must be 1.60 m
...
wrote /tmp/narrow.png; 8 of 9 fit rules pass
exit=1
```

The other rules still pass, and the `routes` line now says `segments >= 0.70 m` (0.80 m before):
the survey route runs 0.1 m closer to row B. Finally, find a pick pose in the registry:

```bash
grep pick_1_02 src/warehouse_layout/generated/locations.yaml
```

```text
  pick_1_02: {kind: pick, aisle: '1', bay: 2, bays: [A02, B02], x: 2.8, y: 4.825, yaw: 1.570796}
```

On aisle 1's centreline at bay 02, facing north (yaw π/2), between bays A02 and B02. The aisle ID
is quoted (`'1'`): unquoted, YAML would read it back as the number 1.

### Exercise 2: generated files stay current

No simulator needed.

```bash
ros2 run warehouse_layout generate --check
PYTHONHASHSEED=1 ros2 run warehouse_layout generate --check
PYTHONHASHSEED=2 ros2 run warehouse_layout generate --check
sha256sum src/warehouse_gazebo/worlds/warehouse.sdf
```

Three times `all 3 generated files are current`: two different hash seeds, the same files
(concept 2). Note the start of the hash (`074cf620f1178e19...` for the world of 2026-11-08). Now
edit the world by hand and check:

```bash
echo '<!-- a hand edit -->' >> src/warehouse_gazebo/worlds/warehouse.sdf
ros2 run warehouse_layout generate --check; echo exit=$?
```

```text
stale: src/warehouse_gazebo/worlds/warehouse.sdf
1 of 3 generated files are out of date. Regenerate them from the workspace root: ros2 run warehouse_layout generate
[ros2run]: Process exited with failure 1
exit=1
```

Regenerate, and the edit is gone:

```bash
ros2 run warehouse_layout generate
sha256sum src/warehouse_gazebo/worlds/warehouse.sdf
git status --short src
gz sdf -k src/warehouse_gazebo/worlds/warehouse.sdf
```

`generate` prints `wrote      src/warehouse_gazebo/worlds/warehouse.sdf: 32 models, 461 visuals,
89 collisions` and `unchanged` for the other two files; the hash is the one you noted;
`git status` prints nothing; `gz sdf -k` prints `Valid.`

### Exercise 3: map files and check_map

No simulator needed. Draw the map an ideal SLAM run would make, and look at both files:

```bash
ros2 run warehouse_layout rasterize --out /tmp/layout_map
cat /tmp/layout_map.yaml
head -c 15 /tmp/layout_map.pgm; echo
```

`rasterize` reports `636 x 428 cells of 0.05 m, origin (-2.2, -2.7, 0)` and `6473 occupied
cells (the faces the lidar sees at 0.34 m), 214869 free, 50866 unknown`. The YAML has the keys of
concept 5 after a comment header; the PGM starts with its text header `P5`, `636 428` and `255`
(binary greyscale, width and height, the largest value), followed by one byte per cell: the file
is 15 + 636 x 428 = 272,223 bytes. Now check the ideal map, the same map shifted 0.15 m east, and
the committed SLAM map:

```bash
ros2 run warehouse_layout check_map /tmp/layout_map.yaml --overlay /tmp/layout_overlay.png
sed 's/^origin: \[-2.2,/origin: [-2.05,/' /tmp/layout_map.yaml > /tmp/shifted.yaml
ros2 run warehouse_layout check_map /tmp/shifted.yaml --overlay /tmp/shifted.png; echo exit=$?
ros2 run warehouse_layout check_map src/warehouse_navigation/maps/warehouse.yaml --overlay /tmp/overlay.png
xdg-open /tmp/overlay.png
```

- The ideal map: 5 `[PASS]` lines, offset 0.001 m, rotation 0.001°, P50 0.025 m (half a cell:
  its cells lie just behind each face).
- The shifted map: `[FAIL] best-fit offset <= 0.05 m: 0.150 m (move the map x -0.150, y +0.000
  m)`, a hint (`the map frame is not the world frame: make the map in a run that starts at
  robot1's spawn, with the EKF started there`), and the four other lines pass: `4 of 5 map checks
  pass`, `exit=1`. Placement and shape fail separately (concept 12).
- The committed map: the five lines of concept 12, offset 0.043 m. In the overlay every black
  wall cell sits on a blue layout face; behind the machinery fence (north-east) the map stays
  partly unknown, because the lidar looked in only through the gate.

### Exercise 4: map_server, lifecycle and transient local

No simulator needed; it uses the map of exercise 3. Terminal 1:

```bash
ros2 launch warehouse_navigation map_server.launch.py map:=/tmp/layout_map.yaml
```

Its log shows `Server map_server connected with bond.` and `Managed nodes are active`.
Terminal 2:

```bash
ros2 lifecycle get /map_server
ros2 topic info -v /map | grep -E 'Node name|Reliability|Durability'
ros2 topic echo /map --qos-durability transient_local --qos-reliability reliable --once --field info
timeout 5 ros2 topic echo /map --qos-durability volatile --qos-reliability reliable --once --field info; echo exit=$?
```

You should see `active [3]`; `Node name: map_server` with `Reliability: RELIABLE` and
`Durability: TRANSIENT_LOCAL`; the map's `info` (`resolution: 0.05...`, `width: 636`,
`height: 428`, origin x -2.2, y -2.7), although the map was published once, before you
subscribed; and from the volatile subscriber nothing at all, then `exit=124` (`timeout` stopped
it). Now pause and resume the map server through its lifecycle manager:

```bash
ros2 service call /lifecycle_manager_map/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 1}'
ros2 lifecycle get /map_server
ros2 topic echo /map --qos-durability transient_local --qos-reliability reliable --once --field info
ros2 service call /lifecycle_manager_map/manage_nodes nav2_msgs/srv/ManageLifecycleNodes '{command: 2}'
ros2 lifecycle get /map_server
```

`command: 1` is PAUSE: the response says `success=True`, terminal 1 logs `Pausing managed
nodes...`, `Deactivating map_server` and `Destroying bond (map_server) to lifecycle manager.`, and
the state is `inactive [2]`. The new transient-local subscriber still gets the map: deactivating
stopped new publishing, but the DDS writer kept the message it holds (concept 8). `command: 2`
(RESUME) brings back `active [3]` and a new bond. Stop with Ctrl-C in terminal 1.

### Exercise 5: map the warehouse by script, and make the GIF

About 13 minutes, Gazebo. Terminal 1 (windows; add `headless:=true` to save CPU):

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam
```

RViz opens with `map.rviz` (fixed frame `map`). Terminal 2, about 10 s later:

```bash
ros2 lifecycle get /robot1/slam_toolbox
ros2 topic info -v /robot1/tf | awk '/^Node name:/ {name = $3} /^Endpoint type: PUBLISHER/ {print name}' | sort -u
ros2 topic echo /robot1/ground_truth/odom --once --field pose.pose.position
ros2 bag record -o bags/my_mapping --use-sim-time --topics /map /robot1/ground_truth/odom /clock
```

`active [3]`; the publishers of the robot's TF are `ekf_filter_node`, `robot_state_publisher` and
`slam_toolbox`; the robot stands at `x: 0.0`, `y: 0.0` (concept 4); the recorder logs
`Subscribed to topic` three times and keeps running. Terminal 3:

```bash
ros2 run warehouse_system_tests drive_route --route survey --speed 0.4 --turn-rate 0.2
```

Do not start other `ros2` commands while it drives (problem 6). Watch the map grow in RViz. After
about 9 min 40 s it ends with `route survey done: 175.6 m in 9 min 40 s of sim time (9 min 41 s
of wall time); the robot stopped 0.03 m from the last waypoint`. Wait 10 s (slam_toolbox takes the
last scans and publishes every 2 s), then, in terminal 3:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/my_map -t /map --fmt pgm \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

It warns about the thresholds (expected, see
[Things that look wrong](#things-that-look-wrong-but-are-not)), reports
`Received a 611 X 403 map @ 0.05 m/pix` or similar, and ends with `Map saved successfully`. Stop
the recorder and the simulation with Ctrl-C in terminals 2 and 1, then:

```bash
ros2 run warehouse_layout check_map /tmp/my_map.yaml --overlay /tmp/my_overlay.png
ros2 run warehouse_system_tests map_timelapse bags/my_mapping /tmp/my_mapping.gif
xdg-open /tmp/my_mapping.gif
```

Expect numbers like the three runs of 2026-11-07: offset 0.043-0.054 m, rotation 0.015-0.125°,
P95 0.044-0.055 m, no ghost cells. The offset may miss its limit by a few millimetres, as one of
the three did (problem 15). `map_timelapse` writes 60 frames of about 640 x 452 px, about
0.18 MB. Save your maps outside `src/`, so they never replace the committed map
([maps/README.md](../../src/warehouse_navigation/maps/README.md) says how that one is made).

A two-minute version: drive `--route smoke` instead of `--route survey`. On 2026-11-08 that map
(482 x 402 cells) passed all five checks on its own (offset 0.048 m, rotation 0.069°), and its GIF
was 0.11 MB.

### Exercise 6: your own map by teleop

About 30 minutes, Gazebo with windows. Here you make the map yourself, driving by hand. Terminal 1:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=slam
```

Terminal 2:

```bash
ros2 run warehouse_bringup teleop_window
```

Click the window. Its top line turns green ("Ready: the robot listens on /robot1/cmd_vel"), and
the terminal prints `Ready: the robot listens on /robot1/cmd_vel. Commands are stamped with sim
time (from /clock).` Press **C** four times to lower the turning speed from 0.6 to 0.2 rad/s:
scans taken while turning fast land in the wrong place (problem 5). Then drive, at the window's
0.3 m/s or slower:

1. Start where the robot stands: the map frame starts there (concept 4).
2. Drive one big loop first, the survey's: east along the main aisle, north along the open floor,
   west along the north cross aisle, south along the west aisle, back to the start. Watch RViz:
   every wall should stay a single line; a doubled wall means drift that no loop closure has
   corrected.
3. Then drive each pick aisle once, and come back near the start.
4. Start no other `ros2` command while you drive, and stop the robot before you save the map.

Then, in terminal 3:

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/teleop_map -t /map --fmt pgm \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
ros2 run warehouse_layout check_map /tmp/teleop_map.yaml --overlay /tmp/teleop_overlay.png
xdg-open /tmp/teleop_overlay.png
```

Write your numbers next to the scripted map's:

| Map | Offset (≤ 0.05 m) | Rotation (≤ 0.2°) | P95 | Ghost cells | Time driven |
|---|---|---|---|---|---|
| Scripted survey, run 1 (committed) | 0.043 m | 0.015° | 0.055 m | 0 | 9 min 40 s |
| Yours | | | | | |

Questions to answer from the overlay: where are the amber or red cells, and what were you doing
there? If the rotation fails, how fast did you turn? Does a second drive with slower turns do
better? To localize on your own map, stop everything and run
`ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl map:=/tmp/teleop_map.yaml`.

### Exercise 7: AMCL and the TF chain

About 5 minutes, Gazebo. Terminal 1:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl
```

Terminal 2, once the log says `Managed nodes are active` twice:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /robot1/amcl
ros2 topic info -v /robot1/tf | awk '/^Node name:/ {name = $3} /^Endpoint type: PUBLISHER/ {print name}' | sort -u
ros2 topic info -v /map | grep -E 'Node name|Reliability|Durability'
ros2 param get /robot1/amcl max_beams
ros2 param get /robot1/amcl alpha1
ros2 topic echo /robot1/amcl_pose --once --field pose.pose.position
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
```

Expected: `active [3]` twice (after 1.6 s and 2.6 s in the check run); the TF publishers `amcl`,
`ekf_filter_node` and `robot_state_publisher`; on `/map` the publisher `map_server` and AMCL's
subscription, both `RELIABLE` and `TRANSIENT_LOCAL`; `Integer value is: 240` and `Double value
is: 0.02`; AMCL's pose `x: 0.0`, `y: 0.0` (the spawn). `tf2_echo` may first print `Waiting for
transform map ->  base_footprint: Invalid frame ID "map" ...`, then once a second
`- Translation: [-0.000, 0.000, 0.000]` and `- Rotation: in RPY (degree) [0.000, 0.000, -0.065]`
or so. Stop it with Ctrl-C. Now draw the tree:

```bash
mkdir -p /tmp/frames && cd /tmp/frames
ros2 run tf2_tools view_frames --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static
cd ~/warehouse_ws
```

The PDF it names (`frames_<date>.pdf`) shows `map -> odom -> base_footprint -> base_link` and the
robot's links; `map -> odom` at about 10.2 Hz, `odom -> base_footprint` at about 50 Hz, every
broadcaster `default_authority` (concept 10).

In RViz: pick **2D Pose Estimate** in the toolbar, click a free spot about 1 m from the robot and
drag to set a heading. AMCL's arrow and particle cloud jump there, away from the magenta
ground-truth arrow. Drive along an aisle with the teleop window and watch the cloud gather and the
arrow return to the magenta one. This part needs the windows, and its numbers were not measured.

### Exercise 8: measure localization from a bag

About 9 minutes, Gazebo. Terminal 1:

```bash
ros2 launch warehouse_bringup sim.launch.py world:=warehouse localization:=amcl headless:=true
```

Terminal 2, once both lifecycle nodes are active (start the recorder after the launch, so it takes
each publisher's QoS; [problem 11](#11-a-bag-without-tf_static)):

```bash
ros2 bag record -o bags/loc_run --use-sim-time --topics /robot1/amcl_pose /robot1/initialpose \
  /robot1/ground_truth/odom /robot1/odom /robot1/imu /robot1/tf /robot1/tf_static /clock
```

Terminal 3:

```bash
ros2 run warehouse_system_tests drive_route --route survey --speed 0.5 --settle 30 \
  --initial-pose-offset 0.3 -0.3 8
```

It first gives AMCL the wrong pose and checks the answer (the acceptance run of 2026-11-08):

```text
[INFO] [...] [robot1.drive_route]: initial pose for AMCL on /robot1/initialpose (try 1 of 3): (0.30, -0.30, 8.0 deg), the true pose (0.00, 0.00, 0.0 deg) moved by (+0.3, -0.3) m and turned by +8 deg
[INFO] [...] [robot1.drive_route]: AMCL took it: it reports (0.28, -0.22, 8.9 deg), 0.36 m and 8.9 deg from the truth
```

AMCL's answer is not exactly the pose it was given: it reports the estimate of the particle cloud
it spread round that pose. Then the robot stands still for 30 s (the IMU check needs it) and drives
the survey: `route survey done: 175.6 m in 7 min 12 s of sim time (7 min 12 s of wall time); the
robot stopped 0.03 m from the last waypoint`. Stop the recorder with Ctrl-C in terminal 2, then:

```bash
ros2 run warehouse_system_tests evaluate_localization bags/loc_run --plot /tmp/loc.png
xdg-open /tmp/loc.png
```

The acceptance run of 2026-11-08 printed:

```text
AMCL (/robot1/amcl_pose) against ground truth (/robot1/ground_truth/odom):
  start: the newest initial pose, at 15.46 s; first error 0.356 m, 8.9 deg
  converged at 49.90 s, after 0.78 m of driving (error below 0.10 m from then on)
  after convergence: 778 poses, 7 min 07 s, 174.8 m
    position error: mean 0.024 m, RMS 0.029 m, P95 0.052 m, max 0.092 m
    heading error: max 3.15 deg
EKF drift (/robot1/odom, put on the ground truth at the start): 0.058 m after 175.6 m (0.03 % of the distance), max 0.083 m
IMU noise (/robot1/imu) from 1 still period, 40.6 s, 3512 samples (506 repeated samples removed):
  axis       sigma     sigma^2   covariance   covariance / sigma^2
  gyro x    0.00204    4.15e-06    4.00e-06     0.96
  gyro y    0.00200    4.01e-06    4.00e-06     1.00
  gyro z    0.00199    3.96e-06    4.00e-06     1.01
  accel x   0.02010    4.04e-04    4.00e-04     0.99
  accel y   0.02051    4.21e-04    4.00e-04     0.95
  accel z   0.01984    3.94e-04    4.00e-04     1.02
[PASS] AMCL converges within 2 m of driving: 0.78 m (error below 0.10 m from 34.4 s after the start to the end)
[PASS] error under 0.10 m for at least 5 min after convergence: 7 min 07 s and 175 m, max 0.092 m
[PASS] gyro sigma within +-20% of robot.yaml (0.002 rad/s) on each axis, from >= 25 s standing still: x 0.00204 (1.02x), y 0.002 (1.00x), z 0.00199 (0.99x) from 40.6 s
[PASS] accelerometer sigma within +-20% of robot.yaml (0.02 m/s^2) on each axis, from >= 25 s standing still: x 0.0201 (1.01x), y 0.0205 (1.03x), z 0.0198 (0.99x) from 40.6 s
4 of 4 localization checks pass
```

The plot shows the three paths from above (ground truth, AMCL and the EKF lie on top of each
other at this scale), and over time the error dropping below 0.10 m within seconds of the first
motion, then staying between about 0.01 and 0.09 m, while the EKF's drift wanders up to 0.08 m.
The repeat run converged after 1.22 m and stayed within 0.090 m.

A two-minute version: `--route smoke` instead of `--route survey`. The 5-minute line then fails by
design (`[FAIL] error under 0.10 m for at least 5 min after convergence: 1 min 05 s and 23 m,
max 0.076 m`) and the other three pass: on 2026-11-08 AMCL converged after 1.21 m.

### Exercise 9: Nav2's defaults, and why they were changed

About 4 minutes, Gazebo. Start a fresh simulation as in exercise 8 (terminal 1), so the robot
stands at the spawn. Terminal 2: put Nav2's sample values back, at runtime:

```bash
ros2 param set /robot1/amcl max_beams 60
ros2 param set /robot1/amcl alpha1 0.2
ros2 param set /robot1/amcl alpha2 0.2
ros2 param set /robot1/amcl alpha3 0.2
ros2 param set /robot1/amcl alpha4 0.2
ros2 bag record -o bags/defaults --use-sim-time --topics /robot1/amcl_pose /robot1/initialpose \
  /robot1/ground_truth/odom /robot1/odom /robot1/imu /robot1/tf /robot1/tf_static /clock
```

Each `param set` prints `Set parameter successful`: AMCL accepts these changes while it runs.
Terminal 3:

```bash
ros2 run warehouse_system_tests drive_route --route smoke --speed 0.5 --initial-pose-offset 0.3 -0.3 8
```

Stop the recorder, then:

```bash
ros2 run warehouse_system_tests evaluate_localization bags/defaults --plot /tmp/defaults.png
xdg-open /tmp/defaults.png
```

On 2026-11-08 it printed `converged at 252.40 s, after 23.82 m of driving` and
`[FAIL] AMCL converges within 2 m of driving: 23.82 m`: in the plot the error drops below 0.10 m
within seconds, but then crosses it again and again (up to about 0.13 m), so the first pose after
which it stays below is the very last one. The tuned values converged after 1.21 m on the same
route (exercise 8's short version). The two IMU lines fail here too, for another reason: without
`--settle` the robot never stands still for 25 s. Restart the simulation to get the tuned values
back; the file was never changed.

### Exercise 10: read a bag yourself

No simulator needed; it uses `bags/loc_run` from exercise 8. `ros2 bag info bags/loc_run` lists the
topics and counts. Then compute the gyro's noise yourself with `rosbag2_py`:

```bash
python3 - bags/loc_run <<'EOF'
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from sensor_msgs.msg import Imu

reader = SequentialReader()
reader.open(StorageOptions(uri=sys.argv[1], storage_id='mcap'), ConverterOptions('cdr', 'cdr'))
reader.set_filter(StorageFilter(topics=['/robot1/imu']))
stamps, rates = [], []
while reader.has_next():
    _, data, _ = reader.read_next()
    msg = deserialize_message(data, Imu)
    stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    rates.append(msg.angular_velocity.z)
stamps, rates = np.array(stamps), np.array(rates)
# The robot stands still for --settle 30 at the start: use 2 s to 25 s after the first sample.
still = (stamps > stamps[0] + 2.0) & (stamps < stamps[0] + 25.0)
new = np.diff(rates, prepend=np.nan) != 0      # a repeated value is the same Gazebo sample
picked = rates[still & new]
print(f'{still.sum()} samples standing still, {(still & ~new).sum()} repeated')
print(f'gyro z: mean {picked.mean():+.5f} rad/s, sigma {picked.std(ddof=1):.5f} rad/s '
      f'(robot.yaml: 0.002)')
EOF
```

On the acceptance bag it printed:

```text
2300 samples standing still, 291 repeated
gyro z: mean +0.00005 rad/s, sigma 0.00197 rad/s (robot.yaml: 0.002)
```

close to `evaluate_localization`'s 0.00199 rad/s, which used the whole 40.6 s still period. The
script reads header stamps (when Gazebo measured), not the times the recorder received the
messages (concept 14). Try `msg.linear_acceleration.x` instead, and compare with the
`accel x` line of exercise 8.

## Pitfalls and lessons

The problem stories above each end with their lesson. These are the everyday pitfalls:

- **Edit `layout.yaml`, never a generated file.** Then run `preview`, `generate`, and
  `generate --check` before you commit; nothing runs them for you (ADR-008).
- **A layout change invalidates the map.** Move anything the lidar sees and the map no longer
  matches the world: make it again with the survey route and check it with `check_map`.
- **Copy a sample configuration, then check every key against your robot**: `/**/` keys in a
  namespace, relative sensor topics, the absolute `/map`, frames that match the EKF's. ROS
  ignores a parameter that matches nothing, without a word. Read the values back from the running
  node with `ros2 param get`.
- **Late subscribers need transient local on both ends.** A recorder started before its
  publishers subscribes volatile and misses `tf_static`; start it after the launch.
- **One simulation at a time, and no new `ros2` tools during a measured drive.** Start the
  recorder and every check before `drive_route`.
- **Turn slowly while mapping**: 0.2 rad/s for the survey, C four times in the teleop window.
- **Save your maps outside `src/`**, and keep `free_thresh: 0.196` if you edit a map file by
  hand, or the unknown grey loads as free floor.
- **Measure from header stamps**, and measure AMCL from `amcl_pose`, not from TF: AMCL
  future-dates its transform by 1 s.
- **A wrong initial pose is only a test if AMCL took it**: check AMCL's answer on `amcl_pose`,
  as drive_route does.
- **Parameters set at runtime are gone at the next launch**; the files are what counts.
- **Keep the laptop awake for long runs** (mains power, lid open, `systemd-inhibit`), and check
  the journal for a suspend before you believe a result.

## Glossary

| Term | Meaning |
|---|---|
| `alpha1`-`alpha4` | AMCL's odometry noise: how far each update scatters the particles, as a share of the motion |
| AMCL | Adaptive Monte Carlo localization: a particle filter that localizes the robot on a known map (`nav2_amcl`) |
| bag (rosbag2) | A recording of ROS topics; Jazzy writes MCAP files by default |
| bond | A two-way heartbeat between a Nav2 lifecycle manager and a node it manages; when it stops, the manager brings the nodes down |
| byte-identical | The same input gives exactly the same file, byte for byte |
| Ceres | The optimization library slam_toolbox uses to adjust its pose graph after a loop closure |
| `check_map` | `warehouse_layout`'s command that fits a map to the layout and checks the mapping limits |
| convergence (distance) | The point after which AMCL's error stays below 0.10 m to the end; the distance driven until then |
| correlative scan matching | Finding a scan's pose by trying shifts and turns in a search window and scoring each against what was already seen |
| durability | The QoS policy that decides whether late subscribers get earlier messages: volatile or transient local |
| fit rule | A check that the layout suits the robot, with limits computed from `robot.yaml` |
| generated file | A file written by `ros2 run warehouse_layout generate` from the layout; committed, never edited by hand |
| ghost wall | Occupied cells where the world has no wall, often from a false loop closure |
| graph SLAM | SLAM that keeps poses as nodes and their relations as edges, and optimizes them all together |
| header stamp | The time in a message's header: when the data was measured, not when it arrived |
| ICP | Iterative closest point: aligns two shapes by pairing nearest points and solving for the best move, over and over |
| initial pose | Where AMCL starts its particles: from the launch (the spawn pose) or a message on `initialpose` |
| KD-tree | A search tree that finds the nearest point among many without comparing them all |
| KLD sampling | How AMCL adapts the number of particles to how spread the cloud is |
| launch event | A request emitted inside a launch file, such as "change this lifecycle node's state" |
| layout file | `warehouse_layout/config/layout.yaml`, the single source of the warehouse |
| lifecycle manager | Nav2's node that configures, activates and watches a list of lifecycle nodes |
| lifecycle node | A managed node with the states unconfigured, inactive, active and finalized |
| likelihood field | AMCL's scan model: how far each beam's end point lies from the nearest occupied cell |
| location registry | `warehouse_layout/generated/locations.yaml`: every named pose and area of the warehouse |
| loop closure | Recognizing a place seen before and adding that constraint to the pose graph |
| `map -> odom` | The correction transform published by SLAM or AMCL, above the EKF's `odom -> base_footprint` |
| `map_saver_cli` | Nav2's command that saves the map on a topic to a YAML and PGM pair |
| `map_server` | Nav2's lifecycle node that loads a map file and publishes it on `/map` |
| MCAP | The bag file format of Jazzy's rosbag2 |
| motion model | How a particle filter moves its particles with the odometry (here `DifferentialMotionModel`) |
| occupancy grid | A map of square cells, each free, occupied or unknown (`nav_msgs/OccupancyGrid`) |
| P50, P95, RMS | The median; the value 95 % stay under; the root mean square |
| particle | One guess of the robot's pose in a particle filter |
| PGM | Portable greymap: the image file of a `map_server` map, one byte per cell |
| point-to-plane | The ICP variant that measures each point's distance across the surface, not to a point on it |
| `rasterize` | `warehouse_layout`'s command that draws the map an ideal SLAM run would make |
| real-time factor | Sim time passed divided by wall time passed; 1.0 is real time |
| replay | Playing a bag back (`ros2 bag play --clock`) through a node to see what it does with the same input |
| resampling | Drawing a new particle set in proportion to the weights, so unlikely guesses die out |
| robot scope / system scope | What each robot runs in its namespace / what runs once for all robots (plan D20) |
| scan plane | The horizontal plane the 2D lidar measures in, 0.34 m above the floor |
| SDF | Simulation Description Format: Gazebo's world and model format |
| SLAM | Simultaneous localization and mapping |
| slam_toolbox | The graph SLAM package used here (`async_slam_toolbox_node`) |
| slot ID | A tote position's name, `<row><bay>-<level>-<position>`, for example `C07-2-1` |
| sRGB / linear light | The encoding of colours in images / values proportional to light intensity, which renderers compute in |
| static model | A Gazebo model that never moves; it costs the physics engine almost nothing |
| transient local | Durability with which the publisher keeps its last messages for subscribers that join later |
| `transform_tolerance` | How far AMCL dates its `map -> odom` into the future (1.0 s) |
| trinary map | A map whose cells are only occupied (0), free (254) or unknown (205) |
| volatile | Durability with which late subscribers get only new messages |
| z-fighting | Flicker where two surfaces lie in the same plane and the renderer cannot decide which is in front |

## Further reading

- slam_toolbox:
  - The repository and its README (parameters, modes, launch files):
    <https://github.com/SteveMacenski/slam_toolbox>
  - S. Macenski and I. Jambrecic, "SLAM Toolbox: SLAM for the dynamic world", Journal of Open
    Source Software, 2021: <https://joss.theoj.org/papers/10.21105/joss.02783>
- Nav2 documentation (Jazzy):
  - Navigating while mapping, with slam_toolbox and `map_saver_cli`:
    <https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/>
  - AMCL's parameters: <https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/>
  - Map server: <https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/map_server/configuring_map_server/>
  - Lifecycle manager: <https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/>
  - Lifecycle nodes and bonds: <https://docs.nav2.org/jazzy/getting_started/navigation_concepts/ros2/>
  - State estimation, REP-105 frames and `map -> odom`: <https://docs.nav2.org/jazzy/getting_started/navigation_concepts/state_estimation/>
- ROS 2:
  - Managed nodes (the lifecycle design): <https://design.ros2.org/articles/node_lifecycle.html>
  - QoS settings, durability included: <https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html>
  - Recording and playing back data: <https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html>
  - Reading a bag from Python with `rosbag2_py` (the Kilted tutorial; the reader API is the one
    used in exercise 10): <https://docs.ros.org/en/kilted/Tutorials/Advanced/Reading-From-A-Bag-File-Python.html>
  - REP 105, coordinate frames for mobile platforms: <https://www.ros.org/reps/rep-0105.html>
- S. Thrun, W. Burgard and D. Fox, *Probabilistic Robotics*, MIT Press, 2005: particle filters,
  the likelihood field model and Monte Carlo localization, the theory behind AMCL.
- S. Rusinkiewicz and M. Levoy, "Efficient Variants of the ICP Algorithm", 3DIM 2001: the
  variants of ICP, point-to-plane among them.
- SDFormat specification: <http://sdformat.org/spec>
- MCAP, the bag file format: <https://mcap.dev/>
- SciPy's KD-tree: <https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.html>
- sRGB and linear light: <https://en.wikipedia.org/wiki/SRGB>

## What comes next

Phase 3 makes the robot navigate on its own, with the warehouse's rules: Nav2 with Smac 2D and
Regulated Pure Pursuit, keepout and speed zones drawn on `rasterize`'s grid from the layout's
zones (so the masks line up with this phase's map by construction), a collision monitor, and
robot revision C for what the lidar puck cannot see. AMCL moves into Nav2's container per robot
(plan D20), and the map server gains its mask servers in the system scope. Its report will be
`docs/phase-reports/phase-03-<name>.md`, and its features will appear in
[PROJECT_STATUS.md](../PROJECT_STATUS.md).

Carried over from Phase 2, open:

- The south-west offset of every map (problem 15), and the EKF's 19 ms lag behind ground truth
  (problem 5), which matters to Nav2 too.
- The pauses of the wheel odometry when a DDS participant starts (problem 6), before Phase 9 runs
  three robots and their tools.
- drive_route prints a traceback when a second SIGINT arrives while it stops the robot (the first
  stop command has already gone out); a one-line fix.
- Python nodes on sim time handle 1000 `/clock` messages a second (drive_route about 0.4 of a
  core); with three robots that adds up.
- The owner's teleop map ([exercise 6](#exercise-6-your-own-map-by-teleop)) and the tags
  `phase-1` and `phase-2`.
