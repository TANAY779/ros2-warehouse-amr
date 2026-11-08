# ADR-007: Generate the warehouse from one layout.yaml

- **Status:** accepted
- **Date:** 2026-11-05 (decided in the Phase 2 plan; built 2026-11-06)
- **Plan reference:** D8 in docs/PROJECT_PLAN.md; the Phase 2 plan's P2-D1 (generated files
  committed), P2-D2 (one frame), P2-D6 (the world's structure) and P2-D16 (pure Python)

## Context

The warehouse appears in many places. Phase 2 needs it as a Gazebo world, as named poses (where
each robot spawns, where it picks, drops and docks) and as an expected inventory. Later phases
need it again: keepout and speed masks (Phase 3), a dock database (Phase 5), tag placement
(Phase 7), scan routes (Phase 8) and a route graph (Phase 11). Written by hand, each copy drifts:
move a shelf in the world and forget the registry, and robots are sent into the shelf; move it
in the registry and forget the masks, and Nav2 lets them through a keepout.

The warehouse must also fit the robot, and the robot is described in `robot.yaml`
([ADR-005](ADR-005-custom-tote-amr.md)). The aisle width depends on the robot's turning circle,
what the lidar can see depends on its scan plane (0.34 m), and the free space round a goal
depends on its footprint. A layout that looks right can still be wrong for the robot.

Constraints: Python for tooling (plan A2), simulation only, a learner who should be able to read
every generated file, and a map made with slam_toolbox that must line up with the world.

## Decision

**One file, `warehouse_layout/config/layout.yaml`, describes the warehouse, and everything else
about the warehouse is generated from it.** The `warehouse_layout` package reads the file,
checks it, derives what it implies, checks that it fits the robot, and writes the generated
files.

- **A strict schema.** `model.py` loads the file into dataclasses (schema version 1). An unknown
  key is an error that names the key, so a typo cannot be ignored silently. From the few numbers
  in the file it derives the rest: 69 bays and 414 slots with their IDs (`C07-2-1`), 8 aisle
  segments, 36 pick poses, the docked and staging poses of 3 docks, keepout and speed zones,
  floor markings and the routes.
- **Fit rules from the robot's own numbers.** `rules.py` holds nine rules: pick aisles exactly
  1.60 m, corridors at least 2.40 m, a free circle of at least 0.556 m round every goal pose,
  everything the lidar must see solid from the floor to at least 0.40 m with no overhang below
  1.20 m, the dock geometry, robot1 at the origin, at most 230 totes and flush with the shelf
  face, no overlaps, and routes clear of obstacles. Every robot-dependent limit is computed from
  `robot.yaml` through `warehouse_description.robot_config` (for example 0.556 m = the padded
  circumscribed radius 0.456 m + 0.10 m), never typed in. `ros2 run warehouse_layout preview`
  prints one `[PASS]` or `[FAIL]` line per rule and draws the layout to scale; `generate` warns
  when a rule fails.
- **Generated files are committed, not built** (P2-D1). `ros2 run warehouse_layout generate`,
  run from the workspace root, writes three files into the source tree:
  `warehouse_gazebo/worlds/warehouse.sdf` (the world), `warehouse_layout/generated/locations.yaml`
  (the location registry) and `warehouse_layout/generated/expected_inventory.yaml`. Each starts
  with a do-not-edit notice.
- **Byte-identical output.** The same layout always gives the same bytes: numbers printed with a
  fixed number of decimals (6 in YAML, 4 in SDF), lists in the layout's order (never a set's),
  the inventory's randomness from `random.Random(seed)`, no dates. A file is rewritten only when
  its text changes. `generate --check` writes nothing and exits 1, naming the file, when a
  committed file differs from what the layout gives now.
- **One frame for everything** (P2-D2). The layout is written in the map frame, which is the
  Gazebo world frame, with its origin at robot1's spawn pose. The mapping run starts there, so
  the SLAM map lines up with the world and the layout with no correction step, and the fit rule
  `spawns` keeps robot1 at (0, 0, 0).
- **A world that is cheap to simulate and honest to the lidar** (P2-D6). One static model per
  object, one collision box per solid (what a robot can hit is what the fit rules checked), and
  visuals for the detail. Every solid has a visual from the floor to above the scan plane,
  because Gazebo's `gpu_lidar` sees visuals, not collisions.
- **Pure Python** (P2-D16). The generators and the checks need no ROS graph and read the source
  tree, so they run in seconds without a simulator.

## Alternatives considered

| Option | Why not |
|---|---|
| Write each file by hand | The copies drift, and nothing tells you |
| Generate at build time (a CMake or `setup.py` step) | The output is hidden in `build/` and `install/`, a world change does not show in a diff, and a step that writes into another package is awkward in ament_python |
| Generate at launch time | A layout edit would change the world under a saved map without a word, and a generator error would surface as a launch error |
| One Gazebo model file per shelf, included with `model://` | Row lengths and sizes differ, so it needs many models and resource paths, and it still needs a generator for the registry |
| Origin at a hall corner, and shift the saved map afterwards | A manual step that is easy to forget; with robot1's spawn as the origin the map lines up by construction |
| Check the fit rules with pytest | The original plan. The tests were removed with every other test ([ADR-008](ADR-008-develop-on-main-without-automated-tests.md)); the same rules now run in `preview` |

## Consequences

- **One edit moves everything.** Change `layout.yaml`, run `preview` (look at the picture and the
  `[PASS]` lines), run `generate`, look at the world in Gazebo, and commit the layout together
  with the generated files. A reviewer sees the change to the world, the registry and the
  inventory in the diff.
- **The robot and the warehouse are checked against each other.** A robot revision that changes
  the footprint or the scan plane changes the fit rules' limits, and `preview` shows what no
  longer fits.
- **Measured:** the generated world has 32 models (with the ground plane), 461 visuals and 89
  collisions; with the robot in it, Gazebo ran at a real-time factor of 1.01 on the development
  laptop, and slam_toolbox's map of it lies on the layout with no correction step: `check_map`
  moves it 0.043 m to fit (limit 0.05 m)
  ([maps/README.md](../../src/warehouse_navigation/maps/README.md)).
- **Generated files can go stale.** Nothing runs `generate --check` automatically any more
  (ADR-008), so run it before committing a change that touches the layout or the generators.
- **A layout change invalidates the map.** After moving anything the lidar sees, make the map
  again on the scripted survey route and check it with `check_map`
  ([maps/README.md](../../src/warehouse_navigation/maps/README.md)).
- **Every new generator must keep the determinism rules:** fixed number formats, a stable order,
  seeded randomness, no timestamps (`formatting.py` holds the helpers).
- **The strict schema costs a code change per new key:** `model.py` must learn every key a later
  phase adds.
- **Follow-up:** Phase 3 draws the keepout and speed masks on the grid of `rasterize` (the map
  tool of this package), Phase 5 generates the dock database, Phase 7 the tag placement, Phase 8
  the scan routes and Phase 11 the route graph, all from the same model.
