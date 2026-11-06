# warehouse_layout

The warehouse from one file (plan D8). `config/layout.yaml` describes the hall, the shelf rows,
the packing station, the docks, the spawn poses, the zones and the look. This package reads
it, checks it against a schema, derives everything else from it (bays, slots, aisles, pick
poses, dock poses, keepout and speed zones, floor markings), checks the fit rules that make the
warehouse work for our robot, and draws a to-scale preview.

Pure Python, no ROS nodes: everything here runs and is tested without a ROS graph. The robot's
dimensions come from `robot.yaml` through `warehouse_description.robot_config`.

From the same `layout.yaml` it generates three files that are committed to git: the Gazebo
world, the location registry and the expected inventory ([Generated files](#generated-files)).

## Try it

After `colcon build --symlink-install` and `source install/setup.bash`:

```bash
ros2 run warehouse_layout preview --out /tmp/layout.png    # draws it, prints the fit rules
ros2 run warehouse_layout generate                         # from the workspace root
ros2 run warehouse_layout generate --check                 # exit 1 if a generated file is stale
python3 -m pytest src/warehouse_layout/test -q             # about 5 s
```

`preview` prints one `[PASS]` or `[FAIL]` line per fit rule with what it measured, writes the
PNG and exits with 1 if a rule fails (the picture is written anyway, so you can see what is
wrong) or 2 if `layout.yaml` does not match the schema. `--layout FILE` draws another file,
for example an edited copy; `--robot FILE` uses another `robot.yaml`.

**Edit workflow:** change `config/layout.yaml`, run `preview` and look at the picture, run
`generate`, then the tests, and commit `layout.yaml` together with the generated files. With
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
| `generated/locations.yaml`, `generated/expected_inventory.yaml` | Generated and committed: do not edit (the world goes to `warehouse_gazebo/worlds/`) |
| `test/test_rules.py` | Every rule passes on `layout.yaml`, and every rule fails on a copy with one deliberate mistake (a 1.5 m aisle, a 0.30 m plinth, a pick pose 0.4 m from a shelf, robot1 off the origin, 240 totes, a goal in a keepout, two objects overlapping, ...) with a message that names the object |
| `test/test_model.py` | What the model derives (slot positions, bay numbers, dock poses, walls round the recess, ...) and the schema errors |
| `test/test_geometry.py`, `test/test_preview.py` | The geometry helpers; the `preview` command and its exit codes |
| `test/test_world.py` | The world read back from its SDF: one collision box per solid, every solid seen by the lidar across its footprint up to 0.40 m, nothing sticking out of the solids below 1.20 m (also with totes), bare shelves, thin paint without collision, colours in linear light |
| `test/test_registry.py`, `test/test_inventory.py` | The registry and the inventory read back with PyYAML: the counts the plan quotes, unique IDs, poses equal to the layout's, tote IDs 20 and up in slot order, every SKU in stock |
| `test/test_generate.py` | The `generate` command on a temporary copy of the source tree: it writes, `--check` passes, a hand edit or a missing file makes `--check` exit 1 |
| `test/test_generated_files_are_current.py` | The installed generated files are what the installed layout gives now, byte for byte; generating twice, and in two processes, gives the same bytes |
| `test/test_copyright.py`, `test_flake8.py`, `test_pep257.py` | The lint tests `ros2 pkg create` generates. `test/conftest.py` runs them from the package directory, as `colcon test` does, so they also work when pytest starts from the workspace root |

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
digits and `_`; in Python an object's `id` is its `name` (flake8 forbids attributes that
shadow the builtin `id`).

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
generated file can go stale: every one starts with a do-not-edit notice, `generate --check`
compares them, and `test_generated_files_are_current.py` regenerates from the installed layout
and compares with the installed files byte for byte.

**Same layout, same bytes.** Numbers are printed with a fixed number of decimals (6 in YAML, 4
in SDF), never with `str()`, whose shortest round-trip text can change with the order of a
calculation (`0.1 + 0.2` prints `0.30000000000000004`). Lists are written in the layout's
order, never in a set's (Python orders a set of strings differently in every process). The
inventory's randomness comes from `random.Random(seed).random()`, the one part of `random`
whose sequence Python keeps the same across versions. No dates. The freshness test generates
twice in one process and once in each of two processes with different `PYTHONHASHSEED` values.

**The world** (`world.py`):

- The physics (1 ms steps) and the five systems of `empty.sdf`: a world that lists any
  `<plugin>` loads only those (warehouse_gazebo README).
- One static model per object (the walls with pilasters and doors, each shelf row segment, the
  packing station, each dock, each fixture, the fence, the floor paint) and one collision box per
  solid of the model, named `<solid>_collision`: what a robot can hit is what the fit rules
  checked. One box per shelf segment keeps the physics cheap (P2-D6).
- The lidar (`gpu_lidar`) sees visuals, not collisions, so every solid has a visual that fills
  its footprint from the floor to above the scan plane: the shelf plinth, the fence's kick plate,
  the guards. Below 1.20 m no visual reaches out of the solids. `test_world.py` checks both.
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

## Design notes

- **Corridors are named in the layout.** The corridor rule needs to know where to measure. Fixed
  probe points in the code would silently measure the wrong place after a layout change; in
  `layout.yaml` they move with the design, and the preview draws each one with its width.
- **Schema errors before fit rules.** `LayoutError` means the file cannot be read as a layout (a
  missing key, a typo, rows that do not face each other). A fit rule failure means the layout
  is readable but does not fit the robot; the rules report every violation at once.
- **One source of truth.** The robot's dimensions are never typed into this package; they come
  from `robot_config`. The constants in `rules.py` are design margins and limits from the plan
  (goal margin, aisle width, tag IDs, ...). Tests build the layout for a longer and for a wider
  robot and see the docked poses move out and the aisle rule object.
