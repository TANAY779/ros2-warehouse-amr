# warehouse_layout

The warehouse from one file (plan D8). `config/layout.yaml` describes the hall, the shelf rows,
the packing station, the docks, the spawn poses, the zones and the look. This package reads
it, checks it against a schema, derives everything else from it (bays, slots, aisles, pick
poses, dock poses, keepout and speed zones, floor markings), checks the fit rules that make the
warehouse work for our robot, and draws a to-scale preview.

Pure Python, no ROS nodes: everything here runs without a ROS graph. The robot's
dimensions come from `robot.yaml` through `warehouse_description.robot_config`.

From the same `layout.yaml` it generates three files that are committed to git: the Gazebo
world, the location registry and the expected inventory ([Generated files](#generated-files)).
Two map tools compare the layout with what the lidar sees ([Map tools](#map-tools)):
`rasterize` draws the map an ideal SLAM run would make, and `check_map` measures how well a
saved map lines up with the layout.

## Try it

After `colcon build --symlink-install` and `source install/setup.bash`:

```bash
ros2 run warehouse_layout preview --out /tmp/layout.png    # draws it, prints the fit rules
ros2 run warehouse_layout generate                         # from the workspace root
ros2 run warehouse_layout generate --check                 # exit 1 if a generated file is stale
ros2 run warehouse_layout rasterize --out /tmp/layout_map  # the map the lidar would make
ros2 run warehouse_layout check_map /tmp/layout_map.yaml --overlay /tmp/overlay.png
```

`preview` prints one `[PASS]` or `[FAIL]` line per fit rule with what it measured, writes the
PNG and exits with 1 if a rule fails (the picture is written anyway, so you can see what is
wrong) or 2 if `layout.yaml` does not match the schema. `--layout FILE` draws another file,
for example an edited copy; `--robot FILE` uses another `robot.yaml`.

**Edit workflow:** change `config/layout.yaml`, run `preview` and look at the picture, run
`generate`, look at the result in Gazebo (`ros2 launch warehouse_bringup sim.launch.py
world:=warehouse`), and commit `layout.yaml` together with the generated files. With
`--symlink-install` the installed files are links to the source files, so no rebuild is needed
between edits (only after a new file appears).

## Files

| File | What it does |
|---|---|
| `config/layout.yaml` | The single source of the warehouse (schema below) |
| `warehouse_layout/model.py` | `load()` reads and checks `layout.yaml` and returns a `Layout`: the hall and every solid object as boxes, row segments with bays and slots, aisle segments, pick poses, stations, docks, spawns, fences, zones, corridors, routes, markings, inventory settings and style. A malformed file raises `LayoutError` naming the bad key |
| `warehouse_layout/rules.py` | The fit rules (plan §3.6): one function per rule, each returning a `RuleResult` whose violations name the object at fault; `check()` runs them all |
| `warehouse_layout/geometry.py` | Plan-view geometry: axis-aligned boxes, polygons, distances, segment and ray tests |
| `warehouse_layout/preview.py` | The `preview` command (matplotlib, no display needed) |
| `warehouse_layout/cli.py` | The `generate` command: writes the generated files into the source tree, or compares them (`--check`) |
| `warehouse_layout/world.py`, `registry.py`, `inventory.py` | The generators of the Gazebo world, the location registry and the expected inventory |
| `warehouse_layout/formatting.py` | What makes the generated files byte-identical run to run: fixed number formats, YAML written as text, the do-not-edit notice |
| `warehouse_layout/occupancy.py` | Map files in map_server's format (a YAML file and a PGM image): `OccupancyMap`, `load()` as map_server reads them, `save()` as map_saver_cli writes them. A malformed file raises `MapError` naming the file and the problem |
| `warehouse_layout/rasterize.py` | What the lidar sees at its scan plane: `surfaces()` (the faces it can see, as points with directions) and `rasterize()` (the map an ideal SLAM run would make); the `rasterize` command |
| `warehouse_layout/check_map.py` | The `check_map` command: the ICP fit of a map to the faces, the distances, the acceptance criteria and the overlay picture |
| `generated/locations.yaml`, `generated/expected_inventory.yaml` | Generated and committed: do not edit (the world goes to `warehouse_gazebo/worlds/`) |

## Frame and units

Every coordinate is in the **map frame, which is the Gazebo world frame**. Its origin is
robot1's spawn pose, facing +x: the mapping run starts there, so the SLAM map lines up with
the world with no correction step (a fit rule keeps robot1 at the origin). x points east, y
north, yaw is counter-clockwise from +x (REP-103). Metres, radians, m/s; colours are sRGB
`'#rrggbb'`. The hall's inner wall faces are at x -1.5 / 28.5 and y -2.0 / 18.0.

Positions on a wall (`span`, `at`, the pilaster lists) are measured along it: x on the south
and north walls, y on the west and east walls.

## Schema (`schema_version: 1`)

Unknown keys are errors, so a typo cannot be silently ignored. Every `id` must be letters,
digits and `_`; in Python an object's `id` is its `name`, so no attribute shadows the builtin
`id`.

| Section | Keys |
|---|---|
| top level | `schema_version`, `name` (the world's name), `seed` (makes the random inventory repeatable) |
| `hall` | `x`, `y` (inner wall faces as `[low, high]`), `wall_thickness`, `wall_height`; optional `recesses` (`id`, `wall`, `span`, `depth`: the wall steps out round a doorway), `doors` (`id`, `wall`, `span`: visual only), `pilasters` (`size: [along, out]` and a list of positions per wall) |
| `shelving` | `depth`, `bay_width` (upright to upright), `upright`, `plinth_height` (its top is level 1), `board_heights` (tops of levels 2, 3, ...), `board_thickness`, `top_height`, `positions_per_level`, `end_guard: {depth, height}` |
| `blocks` | `id`, `y_start`, `bays`: rows run north-south through every block; bay numbers continue from block to block |
| `rows` | `id` (one letter, see slot IDs), `x_face`, `face` (`+x` or `-x`: the way the face looks), optional `bays: {<block>: n}` for a shorter row |
| `aisles` | `id`, `rows: [west, east]`: two rows facing each other. Every row belongs to exactly one aisle |
| `packing_stations` | `id`, `counter: {x, y, height}` (solid to the floor), `drop: {x, y, yaw}` |
| `docks` | `pedestal: {width, depth, height}`, `contact_bar: {width, depth, height}`, `plate_height`, `tag_setback`, `staging_distance`, `keepout_margin: {side, front}`, `units` (`id`, `wall`, `at`) |
| `spawns` | `<robot>: {x, y, yaw}` |
| `fixtures` | `id`, `kind` (picks the colour), `x`, `y`, `height`: solid boxes from the floor |
| `fences` | `id`, `path` (segments parallel to x or y), `gates` (`segment` index from 0, `span`), `thickness`, `kick_plate_height`, `height` |
| `zones` | `pick_aisle_speed`, `keepout` (`id`, `polygon`), `speed` (`id`, `limit`, `polygon`) |
| `corridors` | `id`, `at: [x, y]`, `axis`: where the corridor rule measures a clear width |
| `markings` | `line_width`, `bay_margin`, `zebras` (`id`, `x`, `y`, `stripe`, `walk`) |
| `inventory` | `fill_ratio`, `skus`, `tote: {length, width, height, setback}` |
| `style` | `show_totes`, `wall_band_height`, one colour per element (see [The look](#the-look)), `fixtures: {<kind>: colour}` |
| `routes` | `<name>: [[x, y], ...]`: waypoints for the route driver |

## What the model derives

| Object | Names | How |
|---|---|---|
| Walls | `wall_south`, `wall_east_1`, `loading_door_back`, ... | The south and north walls run past the corners; a recess splits its wall and adds a back and two side walls |
| Row segments | `A_south`, ... (solids `shelf_A_south`, `guard_A_south_s`, `guard_A_south_n`) | One per row and block; bays of `bay_width` with an upright at each end; a solid end guard at both ends. A shelf is solid only to its plinth: the lidar sees through the levels above |
| Slots | `C07-2-1` | Every level of every bay has `positions_per_level` slots sharing the clear opening; a tote's front stands `setback` behind the face |
| Aisle segments | `aisle_1_south`, ... | Between the two faces, from end guard to end guard |
| Pick poses | `pick_1_02`, ... | On the aisle centreline at each bay centre, yaw +π/2: the Phase 7 side cameras then see both faces. 36 in all |
| Docks | `dock_1` (solids `dock_1_pedestal`, `dock_1_bar`, `dock_1_plate`; poses `dock_1_docked`, `dock_1_staging`) | Pedestal against the wall, contact face `tag_setback` in front of it; docked pose half the robot's length out from the contact face (robots dock forwards), staging `staging_distance` further |
| Zones | `dock_1_bar_keepout`; speed zones named like the aisle segments | A keepout box round each dock's plate and bar (below the scan plane); a `pick_aisle_speed` zone over each aisle segment |
| Markings | `robot1_parking_bay`, `dock_1_bay`, `aisle_1_south_stripe_s`, `staff_crossing_1`, ... | Parking bays round the spawns, dock bays from the contact face to beyond the staging pose, a stripe across each aisle mouth, the zebra bands |
| Goals | `spawn_robot1`, `drop_pack_1`, `dock_1_staging`, `pick_*` | `Layout.goals()`: the poses robots are sent to and stop at (docked poses are not goals: the robot touches the dock there) |

**Slot IDs:** `<row><bay:02d>-<level>-<position>`, for example `C07-2-1` is row C, bay 07, level 2
(the first board), position 1 (the south one). Regex `^[A-HJ-NP-Z][0-9]{2}-[1-9]-[1-9]$`: row
letters skip I and O, which read like 1 and 0 on a label. 69 bays x 3 levels x 2 positions =
414 slots.

## Fit rules

`rules.py` derives every robot-dependent limit from `robot.yaml` (Rev B: 0.64 x 0.48 m, padding
0.04 m, so a padded circumscribed radius R = hypot(0.36, 0.28) = 0.456 m; scan plane 0.34 m).

| Rule | Value | Derivation |
|---|---|---|
| `pick_aisle_width` | exactly 1.60 m between the faces | Turning in place on the centreline needs 2 x (R + 0.10) = 1.11 m; the centreline (0.80 m from each face) stays outside Nav2's 0.70 m inflation; the tag rule below caps the width at about 1.62 m (P2-D3) |
| `corridor_width` | >= 2.40 m clear | Plan §2.6. Measured along each `corridors` entry to the first solid or keepout zone: main aisle 3.40, at the docks 4.60, mid cross aisle 2.50, north cross aisle 2.80, west aisle 2.70 (at a pilaster), office corridor 2.50, east corridor 2.50, column corridor 3.45 m |
| `goal_clearance` | free circle >= 0.556 m round every spawn, drop, staging and pick pose; no keepout within 0.456 m | R + 0.10 m, so the 0.48 m stopped/rotating safety field never blocks the turn away from a goal. Tightest: the drop pose, 0.600 m from the counter |
| `visibility` | solid from the floor to >= 0.40 m; nothing overhangs below 1.20 m | The lidar sees only its scan plane: 0.34 + 0.06 m. Only the dock contact bars (0.12 m) and base plates (0.01 m) are lower, on purpose, and each lies inside a keepout box. No overhang up to the Rev E mast top (1.10 m) + 0.10 m: totes stay behind the face the lidar sees |
| `docks` | docked = contact face + 0.32 m; staging 0.70 m further; bar <= 0.15 m; pedestal face 0.25 m behind the contact face | Plan §2.6 (D10); 0.32 m is the robot's half length from `robot_config` |
| `spawns` | robot1 at (0, 0, 0); spawns >= 1.11 m apart | Map = world (above); two goal circles side by side |
| `totes` | <= 230 totes, flush with the shelf face | D23 reserves ArUco IDs 20-249 for totes; 414 slots x 0.5 = 207. Flush, because the tag rule below has no spare margin |
| `containment` | everything inside the hall, no two objects overlap, slot IDs unique and valid | Touching is fine, and so are parts of one object (a row and its guards, a fence's pieces) |
| `routes` | waypoints >= 0.556 m and segments >= 0.40 m from objects; the robot's half width out of every keepout | The driver turns in place at waypoints (a goal circle); along a segment, half the robot's width (0.24 m) + 0.16 m for steering error |

**The Phase 7 tag rule** (why aisles are 1.6 m and totes flush). A side camera (640 px wide,
horizontal field of view 1.57 rad) has a focal length f = 320 / tan(0.785) = 320.3 px. A 0.12 m
tag at distance d spans f x 0.12 / d pixels and must be at least 40 px (D23). The robot keeps
within 0.15 m of the centreline, so the far face is at most half the aisle + 0.15 m away:

- 1.6 m aisle: d = 0.80 + 0.15 = 0.95 m, 40.4 px. The widest aisle that still gives 40 px is
  2 x (320.3 x 0.12 / 40 - 0.15) = 1.62 m.
- Totes 2 cm back from the face: d = 0.97 m, 39.6 px, too small: totes stand flush (setback 0).
- A 1.8 m aisle (rejected in P2-D3): d = 1.05 m, 36.6 px.

## The look

Every colour of the world and the preview is in `style:`, so changing the look is one edit
(the owner's design decision of 2026-11-05):

- **Racks:** plain industrial, the same in every aisle: dark plinth `rack_plinth`, steel
  uprights `rack_upright`, orange boards `rack_board`, steel end guards `rack_guard`.
- **Bare shelves until Phase 7:** `show_totes: false`. The expected inventory (which slot holds
  which SKU) is still generated, because it is data for Phase 8; setting `show_totes: true`
  puts its totes on the shelves in colour `tote`.
- **Infrastructure:** violet packing counter, green dock pedestals with copper contact bars, a
  red fire point, grey fixtures (`style.fixtures` by kind), off-white walls with a slate band
  below `wall_band_height`, concrete floor.
- **Floor paint:** keepouts red-tinted with a red border, the 0.3 m/s zones amber, a white zebra
  crossing, white parking bays and aisle-mouth stripes, green dashed dock bays. Safety yellow is
  left to the robot.

## Generated files

`ros2 run warehouse_layout generate`, run from the workspace root, reads
`src/warehouse_layout/config/layout.yaml` and the robot's size from
`src/warehouse_description/config/robot.yaml` (the source tree, not the install; `--src`
points elsewhere) and writes:

| File | What it holds | Read by |
|---|---|---|
| `src/warehouse_gazebo/worlds/warehouse.sdf` | The Gazebo world: 32 static models (with the ground plane), 461 visuals, 89 collisions (a box per solid of the layout, and the ground plane) | `gazebo.launch.py world:=warehouse` |
| `generated/locations.yaml` | Poses by location ID (3 spawns, 1 drop, 3 docked and 3 staging poses, 36 picks with their aisle and bays), the docks' contact and tag points, the 8 pick-aisle segments, the keepout and speed zones, the routes | spawn poses in `sim.launch.py`, Phase 4 location lookups, Phase 5 docking, the route driver |
| `generated/expected_inventory.yaml` | 207 totes in 414 slots: per slot the tote ID (also its ArUco tag ID, 20 and up) and its SKU; 60 SKUs, each in 3 or 4 totes | Phase 8 inventory checks; the world shows these totes when `style.show_totes` is true |

It prints what it wrote, rewrites only files whose text changed, and warns if the layout breaks
a fit rule. `generate --check` writes nothing and exits 1, naming the file, if one is stale or
missing.

**Committed, not built (plan P2-D1).** A layout change then shows up in review as a change to
the world and the registry, and Gazebo and later nodes load plain files. The cost is that a
generated file can go stale: every one starts with a do-not-edit notice, and `generate --check`
compares them with what the layout gives now.

**Same layout, same bytes.** Numbers are printed with a fixed number of decimals (6 in YAML, 4
in SDF), never with `str()`, whose shortest round-trip text can change with the order of a
calculation (`0.1 + 0.2` prints `0.30000000000000004`). Lists are written in the layout's
order, never in a set's (Python orders a set of strings differently in every process). The
inventory's randomness comes from `random.Random(seed).random()`, the one part of `random`
whose sequence Python keeps the same across versions. No dates.

**The world** (`world.py`):

- The physics (1 ms steps) and the five systems of `empty.sdf`: a world that lists any
  `<plugin>` loads only those (warehouse_gazebo README).
- One static model per object (the walls with pilasters and doors, each shelf row segment, the
  packing station, each dock, each fixture, the fence, the floor paint) and one collision box per
  solid of the model, named `<solid>_collision`: what a robot can hit is what the fit rules
  checked. One box per shelf segment keeps the physics cheap (P2-D6).
- The lidar (`gpu_lidar`) sees visuals, not collisions, so every solid has a visual that fills
  its footprint from the floor to above the scan plane: the shelf plinth, the fence's kick plate,
  the guards. Below 1.20 m no visual reaches out of the solids.
- Racks: a dark plinth up to level 1, open uprights (a square post at the face and one at the
  back, so cameras can look into the bays), orange level boards and a top board, steel end
  guards.
- Floor paint: boxes 1 to 6 mm thin without collision. Each kind has its own height (speed and
  keepout tints, their borders, white lane paint, dock bays), and parts that pass through others
  stay 2 mm inside them: two faces in one plane flicker in the renderer (z-fighting).
- Colours: `style:` colours are sRGB. Gazebo's ogre2 renderer treats material colours as linear
  light and stores the picture as sRGB, so `world.py` converts them. Measured: a surface that
  emits 0.2 shows as pixel value 123 and 0.5 as 187, the sRGB codes of those values; written
  unconverted, every colour would look washed out.
- Every pose is in the map frame and every model stays at the origin, so a number in the SDF is
  the number in `layout.yaml` and the preview, and a layout edit changes only the lines of the
  parts it moves.

## Map tools

The lidar scans one plane, 0.34 m above the floor (`scan_plane_height`, from `robot.yaml`). It
sees a solid if that plane cuts it: walls, pilasters, shelf plinths, guards, the counter, dock
pedestals, the fence's kick plates and the fixtures. It never sees the dock contact bars
(0.12 m) or base plates (0.01 m). The Gazebo world is generated to agree (`world.py`, above), so
the layout's solids are exactly what the lidar can see. Both tools start from there.

### rasterize: the map an ideal SLAM run would make

```bash
ros2 run warehouse_layout rasterize --out /tmp/layout_map    # layout_map.yaml and .pgm
```

```text
wrote /tmp/layout_map.yaml and /tmp/layout_map.pgm: 636 x 428 cells of 0.05 m, origin (-2.2, -2.7, 0)
6473 occupied cells (the faces the lidar sees at 0.34 m), 214869 free, 50866 unknown
below the scan plane, so not drawn: contact_bar, dock_plate
```

- **Occupied:** the cells just behind every face the lidar can see (a solid cell next to a free
  one). That is where its beams end, so a SLAM map has its occupied cells there too.
- **Free:** the rest of the hall floor, including the floor under the dock contact bars.
- **Unknown:** what the lidar cannot see: the inside of the solids (shelf plinths, the office)
  and everything behind the walls.
- **Map frame = world frame:** the origin's yaw is 0 and its lower-left corner sits on a
  multiple of 0.05 m, so cell edges fall on the layout's round numbers (the hall's inner faces
  at x -1.5 / 28.5, y -2.0 / 18.0). Nothing has to be shifted or turned.

It is the fallback map of plan risk R6 (AMCL can run on it if the SLAM map fails), and in
Phase 3 the grid on which the keepout and speed masks are drawn. Measured in Gazebo on the smoke
route (2026-11-08): AMCL on this map converged after 1.62 m and then stayed within 0.096 m of
ground truth (mean 0.041 m); on the SLAM map, within 0.076 m (mean 0.019 m). `--resolution` changes the
cell size; `--layout` and `--robot` work as in `preview`. The same layout always gives the same
bytes.

The files are what `map_saver_cli` writes in trinary mode, so `map_server` loads them as they
are. The PGM holds one byte per cell, north row first: 0 (black) occupied, 254 free, 205
unknown. The YAML:

```yaml
image: 'layout_map.pgm'
mode: trinary
resolution: 0.05
origin: [-2.2, -2.7, 0.0]     # x, y, yaw of the image's lower-left corner
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**Why `free_thresh` is 0.196, not 0.25.** map_server turns a pixel into an occupancy of
1 - value / 255. An unknown pixel (205) gives 50 / 255 = 0.19608. With `free_thresh: 0.25` it
would load as free, and everything behind the walls would become floor. map_saver_cli writes
0.196 in trinary mode for this reason, and so does `rasterize`. Keep it if you edit a map by
hand.

### check_map: does a saved map line up with the layout?

For the SLAM map of plan F5, saved with `map_saver_cli`:

```bash
ros2 run warehouse_layout check_map src/warehouse_navigation/maps/warehouse.yaml --overlay /tmp/overlay.png
```

On the rasterized layout it prints:

```text
map /tmp/layout_map.yaml: 636 x 428 cells of 0.05 m, origin (-2.2, -2.7, 0), 6473 occupied
layout "warehouse": 33433 points on the faces the lidar sees at 0.34 m
best fit after 4 ICP iterations: 6473 of 6473 cells within 0.25 m of a face took part
[PASS] best-fit offset <= 0.05 m: 0.001 m (move the map x +0.000, y +0.001 m)
[PASS] best-fit rotation <= 0.2 deg: 0.001 deg (turn the map -0.001 deg)
[PASS] 95% of occupied cells within 0.10 m of a face: 100.0% (P50 0.025 m, P95 0.026 m, max 0.03 m)
[PASS] at most 1% of occupied cells farther than 0.25 m from every face (ghost walls): 0.00% (0 cells)
[PASS] map origin yaw 0: 0 rad
wrote /tmp/overlay.png
5 of 5 map checks pass
```

On the committed SLAM map (`src/warehouse_navigation/maps/warehouse.yaml`) all five lines pass:
offset 0.043 m (move the map x -0.022, y -0.036 m), rotation 0.015 deg, 100 % of the 7285 occupied
cells within 0.10 m (P50 0.012 m, P95 0.055 m, max 0.06 m), no ghost cells, yaw 0.

It takes the centre of every occupied cell, fits the cells to the faces the lidar can see (ICP,
below), and then measures each cell's distance to the nearest face after that fit. The lines
are plan F5's acceptance criteria:

| Line | What it measures | Passes when | Why that limit |
|---|---|---|---|
| best-fit offset | How far the fit moves the map's origin (robot1's spawn) to put the map on the layout, and that move in x and y | <= 0.05 m | One cell. The map frame must be the world frame (P2-D2), or every AMCL pose is off by the same amount |
| best-fit rotation | How far the fit turns the map, about the origin | <= 0.2 deg | Turning 0.2 deg moves a wall 30 m away by 0.10 m (two cells) |
| within 0.10 m | The share of occupied cells within 0.10 m of a face; also the median distance (P50), the distance 95 % of the cells stay within (P95) and the largest | >= 95 % | SLAM draws a wall one or two cells thick; the lidar's noise is 0.01 m |
| ghost walls | The share of occupied cells farther than 0.25 m from every face | <= 1 % | A cell that far from every face is a wall that is not there: a false loop closure that doubled an aisle, or something in the world that is not in `layout.yaml` |
| origin yaw | The yaw in the map's `origin` | 0 | Nav2's AMCL and costmaps use only the origin's x and y, and the Phase 4 dashboard draws the map unturned |

A cell's centre lies half a cell (0.025 m) from the face it holds, so a perfect map reads P50
0.025 m. A SLAM map can read less (0.012 m above): slam_toolbox marks the cell in which a beam
ends, and that cell can lie on either side of the face. Exit codes: 0 every line passes; 1 a line fails, or no fit is possible (an empty map,
or a map of somewhere else); 2 the map file, `layout.yaml` or `robot.yaml` cannot be read,
or the overlay cannot be written. A malformed map file gives an error that names the file and
the problem, for example
`missing key resolution`. check_map reads maps as map_server does, in trinary or scale mode.

**The overlay** draws the map where the file places it, with the layout's faces in blue on top.
Cells within 0.10 m of a face are black, cells 0.10-0.25 m away amber squares, ghost cells red
crosses; those distances are measured after the fit. So the colours show the map's shape
(blurred or ghost walls), and a gap between the black cells and the blue lines shows an offset
or a turn: a map turned by 1 degree drifts away from the faces the farther it is from the
origin. The picture is written even when a line fails (or no fit is possible).

### Why ICP

To measure how far the map is from the layout, each occupied cell must be paired with the face
it belongs to; but to pair them, you must already know how the map sits on the layout. ICP
(iterative closest point) breaks this loop by alternating:

1. Pair every cell with the nearest face point. A KD-tree (`scipy.spatial.cKDTree`) finds it
   among the 33,000 face points by narrowing down the space instead of comparing with every
   point, so all the cells are paired in a few milliseconds.
2. Solve for the small turn and shift that best move the cells onto their faces (least
   squares).
3. Move the map and pair again. Stop when the map stops moving.

- **Point-to-plane:** a cell is pulled across its face, never along it. A cell on a long wall
  has no true partner along the wall (its nearest face point is just the foot of a
  perpendicular), so only the distance across the wall is an error. On the maps of Phase 2 it
  settles in 4 iterations; on maps shifted or turned on purpose it needed 4 to 16 and came
  within a millimetre, where the textbook point-to-point form (pull each cell towards the face
  point itself) needed 9 to 28 and stopped 2 to 3 mm short.
- **A shrinking cut-off:** pairs farther apart than a cut-off take no part, so ghost walls
  cannot drag the fit. The cut-off starts at 1.0 m, so a map that is off by more than a few
  cells is still caught (a 0.5 m shift or a 2 degree turn is found), and shrinks to 0.25 m,
  the ghost-wall distance.
- **Distances after the fit:** the offset and the rotation say where the map sits; the
  distances say what shape it has. A perfect map shifted by 0.15 m fails the offset line and
  passes the others, so the output tells you which problem you have.
- **About the origin:** the fit turns the map about the world origin, robot1's spawn, where
  the mapping run starts. Its result is the transform from the map frame to the world frame,
  which should be the identity (P2-D2).

## Design notes

- **Corridors are named in the layout.** The corridor rule needs to know where to measure. Fixed
  probe points in the code would silently measure the wrong place after a layout change; in
  `layout.yaml` they move with the design, and the preview draws each one with its width.
- **Schema errors before fit rules.** `LayoutError` means the file cannot be read as a layout (a
  missing key, a typo, rows that do not face each other). A fit rule failure means the layout
  is readable but does not fit the robot; the rules report every violation at once.
- **One source of truth.** The robot's dimensions are never typed into this package; they come
  from `robot_config`. The constants in `rules.py` are design margins and limits from the plan
  (goal margin, aisle width, tag IDs, ...).
