# Project status

The living record of this project: what exists, what was decided and why, and what comes
next. Every pull request that adds or changes a feature updates this file (see
[How this document is maintained](#how-this-document-is-maintained)). For the story of each
finished phase, written to teach, see [phase-reports/](phase-reports/).

**Where to read more:** the learning reports in [phase-reports/](phase-reports/) (start with
[Phase 0](phase-reports/phase-00-foundation.md), then
[Phase 1](phase-reports/phase-01-robot-description.md)); the phased plan with architecture and
acceptance criteria in [PROJECT_PLAN.md](PROJECT_PLAN.md); the decision records in
[decisions/](decisions/). New to terms such as ament_lint, rosdep, colcon, ruleset or ADR? See
the [glossary in the Phase 0 report](phase-reports/phase-00-foundation.md#glossary); for URDF,
TF, ros2_control, QoS or sim time, the
[glossary in the Phase 1 report](phase-reports/phase-01-robot-description.md#glossary).

## At a glance

- **Current phase (as of 2026-11-06):** Phase 2 (warehouse world, mapping, localization) is in
  progress. Its first half, the warehouse generated from one layout file, is merged into `main`
  (2026-11-06); SLAM mapping and AMCL localization come next, after the owner's review of the
  world. Phase 1 (robot description and first drive
  in Gazebo) is **done and merged** into `main` (pull request #2, 2026-11-05), including the
  owner's design review: a 360° lidar puck on the deck and a new look. Its
  [learning report](phase-reports/phase-01-robot-description.md) is written. Still open for
  Phase 1: the demo GIF (recorded by the maintainer, report exercise 7) and the `phase-1` tag.
- **Last updated:** 2026-11-06
- **What runs today** (on `main`): our own robot (revision B, safety yellow and
  charcoal) drives by keyboard (the teleop window) through ros2_control, in an empty Gazebo world or in the
  generated 30 x 20 m warehouse (`world:=warehouse`), inside the `/robot1` namespace, with a 360° lidar puck on its deck, an IMU and EKF odometry in RViz. The same launch file runs it on mock hardware
  without a simulator (the CI check for it is paused with the other checks, ADR-004). Measured in Gazebo: sensor rates
  10 / 100 / 50 Hz, 0 scanner self-hits, wheel odometry within 0.01 % of ground truth over
  10 m, EKF within 0.09-0.42 %.

### Progress

| Milestone | Phase | Scope | Status |
|---|---|---|---|
| M0 Foundation | 0 | Repository, lint gates, CI, dev containers, ADRs | **done** ([report](phase-reports/phase-00-foundation.md)); tag `phase-0` not yet created (as of 2026-11-04) |
| M1 Core AMR (`v1.0.0`) | 1 | Robot description, ros2_control drive, lidar + IMU, EKF, first drive in Gazebo | **done**, merged 2026-11-05 ([report](phase-reports/phase-01-robot-description.md)); demo GIF and tag `phase-1` pending |
| | 2 | Warehouse world generated from a layout file, mapping, localization | **in progress**: the generated world is merged (2026-11-06); mapping and localization next |
| | 3 | Navigation with keep-out and speed zones, safety | planned |
| | 4a | Interfaces + behaviour-tree task executor | planned |
| | 4b | Order manager, order generator, KPIs | planned |
| | 4c | Operator dashboard v1 (Flask): overview, orders, live map, live updates | planned |
| | 5 | Battery, autonomous docking, diagnostics | planned |
| | 6 | Core hardening and the `v1.0.0` release | planned |
| M2 Perception (`v2.0.0`) | 7 | ArUco fiducial perception (tote and dock tags) | planned |
| | 8 | Inventory patrol and discrepancy reports | planned |
| | 8b (optional) | Learned ML/DL perception, trained on Google Colab | planned (optional; research done, [design](PERCEPTION.md#5-method-2-learned-tote-detection)) |
| M3 Fleet (`v3.0.0`) | 9 | Multi-robot bring-up | planned |
| | 10 | Fleet manager v1, greedy task allocation | planned |
| | 11 | Auction allocation and aisle traffic control | planned |
| | 12 | Dashboard polish and the showcase scenario | planned |
| Stretch | 13 | Mobile manipulation | stretch |

This table follows [PROJECT_PLAN.md](PROJECT_PLAN.md) (version 2), which holds each phase's
scope and acceptance criteria.

## Features

Only features that exist on `main` are listed; Phase 2 entries are added as they merge.
Commands assume the
repository is cloned as `~/warehouse_ws` (its root is the colcon workspace, ADR-006) and a shell with only ROS 2 Jazzy and this workspace
sourced (the [Phase 1 report](phase-reports/phase-01-robot-description.md#before-you-start)
has a `wh_shell` function for that).

### Repository skeleton

- **What it is:** the files every professional repository needs before any code: licence,
  README, editor and git settings, and the list of third-party sources.
- **What it does:** `.editorconfig` gives every editor the same indentation and line endings;
  `.gitattributes` forces LF line endings and marks images, bags and models as binary;
  `.gitignore` keeps colcon output, bags, datasets and model weights out of git (small test
  fixtures under `test/data/` are allowed); `dependencies.repos` lists source dependencies
  that have no Jazzy binary (none yet) for `vcs import`.
- **How to use it:** nothing to run; editors and git pick these files up automatically. To
  import third-party sources: `vcs import --input dependencies.repos src`
  from the workspace root.
- **Where it lives:** `LICENSE` (Apache-2.0), `README.md`, `.editorconfig`, `.gitattributes`,
  `.gitignore`, `dependencies.repos`.

### Robot description (`warehouse_description`, revision B)

- **What it is:** our own tote-carrying AMR ([plan §2.6](PROJECT_PLAN.md#26-robot-design-d1),
  [ADR-005](decisions/ADR-005-custom-tote-amr.md)), described once in `config/robot.yaml` and
  turned into URDF by xacro. Revision B: the drivable base (rev A) plus the first sensors.
- **What it does:** builds the robot for Gazebo (`hardware:=gz`) or for mock hardware without
  a simulator (`hardware:=mock`; `real` is reserved and refused): a 0.64 x 0.48 m, 35 kg
  differential-drive base with sphere wheel collisions and frictionless casters, a 360° lidar
  puck on the deck (`gpu_lidar`, scan plane 0.34 m, 10 Hz) and an IMU (100 Hz) read through
  ros2_control. The body is safety yellow and charcoal, built from parts that are easy to tell
  apart: charcoal skirt and deck top, yellow panels, black bumpers, a grey tote tray, a red
  emergency-stop button and a green status light (its colour is one property, which Phase 9
  makes per robot).
  Each coloured part is its own link, so RViz shows its colour too.
  `config/controllers.yaml` sets up `joint_state_broadcaster`, `diff_drive_controller` (hard
  limits in both directions of travel, unprefixed frames, no odometry transform) and
  `imu_sensor_broadcaster`. Every gz topic carries the robot namespace. A pytest checks the
  URDF (frames, mass, namespaced topics, the lidar's full circle, that nothing but the lidar's
  head reaches its scan plane, that every part stays inside the outline, one colour per link)
  and `controllers.yaml` against `robot.yaml`.
- **How to use it:** the launch files load it. To look at the URDF yourself:

  ```bash
  xacro $(ros2 pkg prefix warehouse_description)/share/warehouse_description/urdf/robot.urdf.xacro \
    namespace:=robot1 hardware:=gz > /tmp/robot.urdf
  check_urdf /tmp/robot.urdf          # prints the link tree
  ```

- **Where it lives:** `warehouse_description/` (`config/`, `urdf/`, `test/`); the README has
  the mass budget and the design notes.

### Gazebo world and robot spawning (`warehouse_gazebo`)

- **What it is:** the simulator side: worlds, spawning, and the bridge between Gazebo and ROS
  topics.
- **What it does:** `gazebo.launch.py` starts Gazebo Harmonic with a world (headless or with a
  window) and bridges `/clock` once per world. It runs `gz sim` directly rather than through a
  shell, so one Ctrl-C (SIGINT) stops it. `spawn_robot.launch.py` spawns a robot from its
  `robot_description` and bridges its `scan` (sensor-data QoS) and `ground_truth/odom` (for
  tests only). Two worlds: `empty` and `warehouse`, which `warehouse_layout` generates (next
  entry). A pytest checks these files without rendering,
  so CI runs it: the systems every world loads, the bridge file against the robot
  description, and the caster friction after Gazebo's own URDF conversion.
- **How to use it:** normally through `warehouse_bringup`'s `sim.launch.py`. On its own:
  `ros2 launch warehouse_gazebo gazebo.launch.py headless:=true`.
- **Where it lives:** `warehouse_gazebo/` (`launch/`, `worlds/empty.sdf`,
  `worlds/warehouse.sdf` (generated), `config/robot_bridge.yaml`, `test/test_gazebo_config.py`).

### Warehouse layout and the generated world (`warehouse_layout`)

- **What it is:** the warehouse described in one file, `config/layout.yaml` (plan D8), and the
  pure-Python package that checks it and generates everything else from it. The robot's size
  comes from `robot.yaml` through `warehouse_description.robot_config`.
- **What it does:** a 30 x 20 m hall with 4 pick aisles in 2 blocks (8 shelf rows A-H, 414
  slots), a packing station, 3 charging docks, spawn poses for 3 robots and an open handling
  floor in the east; plain uniform racks with no totes until Phase 7. It checks the layout
  against a schema and 9 fit rules (aisle width for the robot, everything the lidar must see
  solid up to 0.40 m, pick poses clear of shelves, ...), draws a to-scale preview, and
  generates three committed files: the Gazebo world (`warehouse_gazebo/worlds/warehouse.sdf`,
  31 static models), the location registry (`generated/locations.yaml`: slots, 36 pick poses,
  docks, the drop point, spawns) and the expected inventory (`generated/expected_inventory.yaml`:
  207 totes, seed 2, data for Phase 8). The output is byte-identical run to run, and
  `generate --check` fails when a committed file is stale. `sim.launch.py world:=warehouse`
  spawns each robot at its spawn pose from the registry; values given on the command line win.
  Measured in Gazebo: spawn pose exact, controllers active, scan at 10 Hz with ranges within
  2 cm of the layout, real-time factor 1.01.
- **How to use it:**

  ```bash
  ros2 run warehouse_layout preview --out /tmp/layout.png    # picture + one PASS/FAIL line per rule
  ros2 run warehouse_layout generate                         # after editing layout.yaml (from ~/warehouse_ws)
  ros2 run warehouse_layout generate --check                 # exit 1 if a generated file is stale
  ros2 launch warehouse_bringup sim.launch.py world:=warehouse
  ```

- **Where it lives:** `warehouse_layout/` (`config/layout.yaml`, `warehouse_layout/`,
  `generated/`, `test/`); its README has the schema, the fit rules and the edit workflow, and
  `warehouse_bringup`'s README explains where each robot spawns.

### Odometry EKF (`warehouse_navigation`)

- **What it is:** a `robot_localization` extended Kalman filter, the robot's odometry
  ([ADR-005](decisions/ADR-005-custom-tote-amr.md), plan D3).
- **What it does:** fuses the wheel odometry (forward speed, the diff drive's zero sideways
  speed, yaw rate) with the IMU yaw rate, and publishes `odom` and the transform
  `odom -> base_footprint`. Over a 10 m drive in Gazebo its end point is within 0.09-0.42 % of
  ground truth (limit 1 %), over a 360° turn within 0.05 % (limit 2 %). The wheel odometry
  alone is closer (0.01 %): in Gazebo the controller's reported speed reads about 0.2 % high,
  and the EKF integrates that speed (Phase 1 report, concept 9). On mock hardware the IMU
  reports no turning, so there the EKF follows straight driving only.
- **How to use it:** it starts with every robot. Watch it with
  `ros2 topic echo /robot1/odom --field pose.pose`.
- **Where it lives:** `warehouse_navigation/config/ekf.yaml`,
  `warehouse_navigation/launch/ekf.launch.py`. AMCL (Phase 2) and Nav2 (Phase 3) join it here.

### Simulation bring-up and the lint template (`warehouse_bringup`)

- **What it is:** the top-level launch files, the entry point of every demo, and the first ROS
  2 package, whose build and lint setup every later C++ package copies.
- **What it does:** `sim.launch.py` starts Gazebo, one robot in its namespace and RViz, and
  sets `use_sim_time` for all of them; with `hardware:=mock` it runs the same robot stack on
  mock hardware, without a simulator. `robot.launch.py` is the robot itself:
  `robot_state_publisher`, the controller manager, the three controllers and the EKF inside
  `/<namespace>`, with the robot's transforms on `/<namespace>/tf`. It sets `use_sim_time` and
  an absolute namespace itself, so it also runs on its own next to `gazebo.launch.py` or
  inside a fleet launch (Phase 9). `colcon test` runs the `ament_lint` linters on every
  package, with clang-format instead of uncrustify; a missing clang-format linter fails the
  build instead of being skipped.
- **How to use it:** from `~/warehouse_ws`:

  ```bash
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install && source install/setup.bash
  ros2 launch warehouse_bringup sim.launch.py                   # Gazebo + RViz, robot in /robot1
  ros2 launch warehouse_bringup sim.launch.py headless:=true    # no windows
  ros2 launch warehouse_bringup sim.launch.py hardware:=mock    # no simulator
  ros2 control list_controllers -c /robot1/controller_manager   # three controllers, all active
  ```

  Built with tests (`colcon build --symlink-install --metas /dev/null --cmake-args
  -DBUILD_TESTING=ON`), `colcon test && colcon test-result --verbose` reported `104 tests, 0
  errors, 0 failures, 0 skipped` on a machine that can render (70-75 s) before the 2026-11-05
  design change, which replaces one description test with four (107 in total; not run yet,
  checks are paused); CI reports the 11 Gazebo launch test cases as skipped.
- **Where it lives:** `warehouse_bringup/` (`launch/`, `rviz/robot.rviz`, README with every
  argument and the checks to run by hand); lint rules in
  [ADR-003](decisions/ADR-003-formatting-and-linting.md) and the "New packages" section of
  [CONTRIBUTING.md](../CONTRIBUTING.md).

### Keyboard teleoperation

- **What it is:** driving the robot by keyboard. The teleop window of `warehouse_bringup`
  (since 2026-11-06) drives while a key is held and stops the moment it is released;
  `teleop_twist_keyboard` in a terminal, the Phase 1 demo, is the alternative without a desktop.
- **What it does:** both publish `TwistStamped` commands on `/robot1/cmd_vel`, stamped with sim
  time, so `diff_drive_controller` brakes once the newest command is 0.15 s old
  (`cmd_vel_timeout`; 0.5 s until 2026-11-06). The window sees key presses and releases: it
  publishes at once, repeats the command at 20 Hz while a key is held, sends three zero commands
  over 0.1 s after the release and then nothing, and sends one zero command when it loses the
  keyboard (another window clicked, window closed, Ctrl-C). Its speeds start at 0.3 m/s and
  0.6 rad/s, so braking takes 0.2 s; Q/Z and E/C change them. Measured in headless Gazebo on
  2026-11-06 at 0.5 m/s and 1.0 rad/s: after a release the robot stands still in 0.34 s, 8.3 cm
  or 9.7° further on (teleop_twist_keyboard: 0.49 s after the last key event, 16.0 cm or
  18.2°), and a held key drives without teleop_twist_keyboard's 0.2-0.3 s stop; at the window's
  defaults, 0.21 s and 3.1 cm or 3.5°. A tap of `i` with teleop_twist_keyboard moves 1.4 cm and
  a tap of `j` turns 3.9° (15.4 cm and 28.6° with the old 0.5 s timeout); a 0.1 s tap in the
  window moves 0.7 cm or turns 1.7°. The controller's hard limits apply whatever teleop asks
  for (1.2 m/s forward, 0.3 m/s reverse, 2 rad/s).
- **How to use it:** with the simulation running (`ros2 launch warehouse_bringup
  sim.launch.py`), in a second terminal:

  ```bash
  ros2 run warehouse_bringup teleop_window --ros-args -r __ns:=/robot1 -p use_sim_time:=true
  # or, in a terminal without a desktop:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r __ns:=/robot1 -p stamped:=true -p use_sim_time:=true
  ```

  In the window: click it, hold the arrows or W A S D to drive, Space stops. In
  teleop_twist_keyboard: `i` forward, `,` backward, `j`/`l` turn, `k` stop. Leave out
  `use_sim_time:=true` for `hardware:=mock`. Without it in Gazebo, the wall-clock stamps never
  look old and the robot keeps driving after you release the key (plan R1).
- **Where it lives:** `warehouse_bringup/warehouse_bringup/` (`hold_to_drive.py`, the logic with
  no Qt and no ROS; `teleop_window.py`, the window and its node), the tests in
  `warehouse_bringup/test/`, and `warehouse_bringup/README.md` ("Driving with teleop", with the
  keys, parameters and measurements); `teleop_twist_keyboard` is an exec dependency of
  `warehouse_bringup`.

### System tests (`warehouse_system_tests`)

- **What it is:** `launch_testing` tests that start the real bring-up and check the running
  system from the outside, like a user with `ros2` commands.
- **What it does:** the mock-hardware test (every pull request) checks the controllers, the
  stamped `cmd_vel`, unprefixed frames, the namespaced TF tree, that the robot keeps a
  command until `cmd_vel_timeout` expires and then stops (time windows derived from
  `controllers.yaml`), and braking at the hard limit in both directions. The Gazebo test (local,
  needs GPU or EGL rendering) adds the Phase 1 acceptance checks: sensor rates, that the
  scanner sees a target box and never the robot itself, wheel and EKF odometry against ground
  truth over 10 m and a 360° turn, and a clean shutdown. The robot's numbers come from
  `robot.yaml` and `controllers.yaml`, never from the tests.
- **How to use it:** `colcon test --packages-select warehouse_system_tests && colcon
  test-result --verbose`; skip the Gazebo test with `WAREHOUSE_SKIP_GZ_TESTS=1`, as CI does.
- **Where it lives:** `warehouse_system_tests/` (`test/`, the shared helpers in
  `warehouse_system_tests/ros_helpers.py`, README with the list of checks).

### pre-commit hooks

- **What it is:** git hooks that check every commit before it is created.
- **What it does:** formats C++ with clang-format 18 (ROS 2 style), lints Python with flake8
  and the same plugins as `ament_flake8`, lints YAML with yamllint (rejects `yes`/`no`
  booleans), checks JSON/XML/YAML syntax, blocks files over 1 MB (3 MB under `docs/media/`),
  private keys, merge-conflict markers, trailing whitespace and missing final newlines. The
  versions of clang-format and flake8 are pinned to what Ubuntu 24.04 ships, so the hooks
  agree with `colcon test`.
- **How to use it:** once per clone, `pre-commit install` in the repository; after that it
  runs on every `git commit`. Run it on demand with `pre-commit run --all-files` or
  `pre-commit run --files <file>...`.
- **Where it lives:** `.pre-commit-config.yaml`, `.clang-format`, `.flake8`, `.yamllint`.

### Continuous integration (GitHub Actions)

- **Status:** paused since 2026-11-05 at the owner's request: the workflow runs only when
  started by hand (Actions → CI → Run workflow). The description below is how it works when
  it runs; [ADR-004](decisions/ADR-004-ci-pipeline.md) says how to resume it.
- **What it is:** a workflow that runs on every pull request, on every push to `main`, and on
  demand.
- **What it does:** job **Lint (pre-commit)** runs every pre-commit hook on every file; job
  **Build & test (ROS 2 Jazzy)** runs, inside the official `ros:jazzy-ros-base` container,
  the same `vcs import`, `rosdep install`, `colcon build`, `colcon test` and
  `colcon test-result` commands a developer runs, then uploads the logs and test results as
  the `colcon-logs` artifact. Superseded pull request runs are cancelled; every commit on
  `main` gets its own result. Since Phase 1 the build-and-test job keeps DDS discovery on the
  runner (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) and sets `WAREHOUSE_SKIP_GZ_TESTS=1`: the
  runners cannot render Gazebo's scanner, so the Gazebo launch tests are reported as skipped
  while the mock-hardware launch tests and `warehouse_gazebo`'s render-free configuration test
  run. In Phase 0 each job took about 35-45 s. Phase 1
  adds Gazebo, ros2_control and RViz to the rosdep install: a local reproduction of the
  build-and-test job took about 8.5 minutes, 7.8 of them in `rosdep install` (the first run on
  GitHub comes with the Phase 1 pull request).
- **How to use it:** open a pull request; both checks appear on it. From a terminal:
  `gh run list --workflow ci.yml`, then `gh run view <run-id>`. To reproduce a failure
  locally, see [ADR-004](decisions/ADR-004-ci-pipeline.md) (Consequences).
- **Where it lives:** `.github/workflows/ci.yml`; rationale in
  [ADR-004](decisions/ADR-004-ci-pipeline.md).

### Branch protection (ruleset "Protect main")

- **What it is:** a GitHub repository ruleset on the default branch.
- **What it does:** changes reach `main` only through a pull request. Force-pushes and
  deleting `main` are blocked. While CI is paused (since 2026-11-05) the two CI checks
  (`Lint (pre-commit)` and `Build & test (ROS 2 Jazzy)`) are not required; the recreation
  steps below include them for when CI resumes. No approving review is required (single maintainer).
- **How to use it:** work on a branch, push it, open a PR, merge when green. Inspect the
  rules with `gh api repos/TANAY779/ros2-warehouse-amr/rules/branches/main --jq '.[].type'`
  (prints `deletion`, `non_fast_forward`, `pull_request`; `required_status_checks` too when CI
  is active).
- **Where it lives:** GitHub repository settings (Settings → Rules → Rulesets), not in the
  repository. To recreate it (for example on a fork): new branch ruleset named "Protect
  main", enforcement **Active**, no bypass list, target **Default branch**; enable **Restrict
  deletions**, **Block force pushes**, **Require a pull request before merging** (0 required
  approvals) and **Require status checks to pass** with `Lint (pre-commit)` and
  `Build & test (ROS 2 Jazzy)`. The job names in `ci.yml` are the required check names, so
  renaming a job requires updating the ruleset.

### Dependabot

- **What it is:** GitHub's dependency update bot.
- **What it does:** once a month, opens a pull request with a `ci:` commit if a GitHub Action
  used in the workflows has a new version. pre-commit hooks are deliberately not included:
  clang-format and flake8 move only with the ROS distribution.
- **How to use it:** review and merge its pull requests like any other.
- **Where it lives:** `.github/dependabot.yml`.

### Pull request template

- **What it is:** the default body of every new pull request.
- **What it does:** asks for what changed, why (with a link to the plan phase), how it was
  tested (build, test, pre-commit, demo evidence), and a checklist for docs, ADRs and large
  binaries.
- **How to use it:** fill it in when opening a PR on GitHub.
- **Where it lives:** `.github/pull_request_template.md`.

### Dev containers (GUI and headless)

- **What it is:** a Docker development environment for VS Code (Dev Containers extension),
  based on `osrf/ros:jazzy-desktop-full` (ROS 2 Jazzy plus Gazebo Harmonic) with the lint
  tools added.
- **What it does:** gives a clean, reproducible ROS 2 environment with the same tool versions
  as CI. The **ros2-warehouse-amr (ROS 2 Jazzy)** configuration is for a Linux desktop: GPU
  access through `/dev/dri` and GUI windows (Gazebo, RViz) through X11, granted only to your
  user. The **ros2-warehouse-amr (headless)** configuration has the same tools without GPU or
  GUI, for macOS, Windows, servers or Codespaces (Codespaces not tested yet). Both set
  `ROS_DOMAIN_ID=42`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and
  `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1`, install dependencies and the git hooks on creation,
  and configure VS Code to use the same clang-format and flake8 as CI.
- **How to use it:** open the repository in VS Code and run **Dev Containers: Reopen in
  Container**, then pick a configuration. The workspace is `/home/ros/ws`. Details and
  security notes are in the README.
- **Where it lives:** `.devcontainer/Dockerfile`, `.devcontainer/devcontainer.json`,
  `.devcontainer/headless/devcontainer.json`, `.devcontainer/post-create.sh`.

### Architecture decision records (ADRs)

- **What it is:** one short document per significant decision: context, decision, rejected
  alternatives, consequences.
- **What it does:** keeps the reasons behind the design findable. ADRs are never deleted; a
  changed decision gets a new ADR that supersedes the old one.
- **How to use it:** copy `docs/decisions/ADR-000-template.md` to the next number and add a
  row to the index.
- **Where it lives:** [docs/decisions/](decisions/) (ADR-001 to ADR-005 so far).

### Contribution workflow and conventions

- **What it is:** the written rules for branches, commits, tags, code style, licence headers,
  media sizes and new packages.
- **What it does:** keeps every change consistent and reviewable: Conventional Commits with
  the package as scope, a `phase-N` tag per finished phase, release tags at milestones,
  double-quoted ROS includes, the Apache-2.0 header in every source file.
- **How to use it:** read it before the first change; the checklists in it are the review
  criteria.
- **Where it lives:** [CONTRIBUTING.md](../CONTRIBUTING.md).

## Ideas and decisions log

Oldest first; each row is dated by the day the idea or decision was first made. Status
values: **adopted** (in effect), **done** (a one-off action that is complete), **planned**
(agreed, not built yet), **paused**, **replaced** (superseded by a later entry), **in review**.

| Date | Idea or decision | Why | Status | Reference |
|---|---|---|---|---|
| 2026-11-02 | Build a simulated warehouse AMR system: order fulfilment first, then perception, then a fleet | A realistic, demo-able system that covers the core of professional ROS 2 work; doubles as a portfolio | adopted | [Plan](PROJECT_PLAN.md) |
| 2026-11-02 | Stack: ROS 2 Jazzy, Gazebo Harmonic, Nav2; phased plan with a recorded demo per phase | Jazzy is a long-term-support distribution (supported until 2029) and Harmonic is its paired Gazebo release; small phases keep progress visible | adopted | [Plan](PROJECT_PLAN.md) |
| 2026-11-02 | Record decisions as ADRs | Reasons are easy to forget over months; reviewers want the "why" | adopted | [ADR-001](decisions/ADR-001-record-architecture-decisions.md) |
| 2026-11-02 | One multi-package repository inside a colcon workspace; packages prefixed `warehouse_`, created only when needed | Packages change together; keeps build output out of the repository | adopted | [ADR-002](decisions/ADR-002-repository-layout.md) |
| 2026-11-02 | clang-format (not uncrustify) for C++, flake8 for Python, both pinned to the ament versions; cpplint kept | One style across editor, pre-commit and `colcon test`; cpplint proved compatible | adopted | [ADR-003](decisions/ADR-003-formatting-and-linting.md) |
| 2026-11-02 | CI runs plain rosdep/colcon commands in `ros:jazzy-ros-base` instead of `action-ros-ci` | A CI failure must be reproducible locally with the same commands | adopted (replaces the plan's `action-ros-ci`) | [ADR-004](decisions/ADR-004-ci-pipeline.md) |
| 2026-11-02 | Operator UI is our own Flask web app on the LAN; Foxglove and rosbridge dropped | Own UI is a stronger learning and portfolio piece; fewer moving parts | adopted (replaces Foxglove) | Plan v2, Phase 4c |
| 2026-11-02 | Deployment (running the system as a service on the LAN) after all phases | Keep phases focused on robotics | planned | Plan v2 |
| 2026-11-02 | Perception method 1: ArUco fiducial markers on shelves and docks | Deterministic, cheap on CPU, ideal for a sim-only project | planned (Phases 7-8) | Plan v2 |
| 2026-11-02 | Perception method 2: learned ML/DL detector, trained on a Google Colab Pro GPU | Learn a modern perception pipeline without a local NVIDIA GPU | planned (optional Phase 8b) | Plan v2 |
| 2026-11-02 | Simulation only, no hardware | No robot available; ros2_control keeps the stack hardware-ready | adopted | Plan, assumption A1 |
| 2026-11-02 | Our own custom robot (a tote-carrying AMR), refined in revisions across phases | Biggest learning payoff (URDF, ros2_control, sensors); a TurtleBot would read as a tutorial | adopted | Plan, decision D1 |
| 2026-11-03 | Split Phase 4 into 4a (interfaces + BT task executor), 4b (order manager, generator, KPIs), 4c (dashboard v1, minimal) | Phase 4 was too large for one PR; `v1.0.0` ships with the dashboard | adopted | Plan v2 |
| 2026-11-03 | ML/DL perception research paused by the maintainer (3 of 4 topics done) | Focus on the core stack first | replaced (resumed 2026-11-06) | Plan v2, Phase 8b |
| 2026-11-03 | Repository named `ros2-warehouse-amr` (was `warehouse_amr`); ROS package names keep the `warehouse_` prefix | Descriptive, searchable name for a public repo | adopted | README |
| 2026-11-03 | Tags: `phase-N` marker per finished phase; releases `v1.0.0`, `v2.0.0`, `v3.0.0` at milestones; packages stay at `0.1.0` until `v1.0.0` | A per-phase version tag (`v0.N.0`) would make versions go backwards after `v1.0.0` (Phase 7 would be `v0.7.0`) | adopted (replaces per-phase version tags) | [CONTRIBUTING.md](../CONTRIBUTING.md) |
| 2026-11-03 | Environment notes: the local workspace was cleaned of files from earlier experiments; local shells may source other workspaces, so CI and the dev container are the clean reference | A clean, reproducible reference without disturbing other local work | adopted | [Dev containers](#dev-containers-gui-and-headless) |
| 2026-11-03 | Phase 0 verified in Docker by parallel AI reviewers (Claude Code agents); every bug found was fixed and re-verified | Tooling that silently does nothing is worse than none | done | [Phase 0 report](phase-reports/phase-00-foundation.md#what-happened) |
| 2026-11-03 | cppcheck enabled with `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` in CI, the dev container and native setups | `ament_cppcheck` silently skips cppcheck 2.x otherwise | adopted | [ADR-003](decisions/ADR-003-formatting-and-linting.md) |
| 2026-11-03 | File size limits: 1 MB per file, 3 MB for GIFs in `docs/media/`, enforced in CI; long demos as MP4 on GitHub Releases | Keep clones small; git never forgets a large file | adopted | [CONTRIBUTING.md](../CONTRIBUTING.md) |
| 2026-11-03 | Dev container: per-user X11 access, headless variant, render-group warning, CAP_SYS_PTRACE for gdb | Found during verification: the first version leaked X11 access and could not start on non-Linux hosts | adopted | [Dev containers](#dev-containers-gui-and-headless) |
| 2026-11-03 | Repository published on GitHub; `main` protected by a ruleset (PR plus both CI checks required, no force-push or deletion); Dependabot enabled | Nothing reaches `main` without passing CI | done | [Features](#branch-protection-ruleset-protect-main) |
| 2026-11-04 | An accidental git repository at the workspace root (created from VS Code; it recorded this repository as an embedded repository) was moved aside; the workspace is not version-controlled | ADR-002: only `src/ros2-warehouse-amr` is a repository | done | [ADR-002](decisions/ADR-002-repository-layout.md) |
| 2026-11-04 | Keep this living status document plus one learning report per finished phase | The maintainer learns ROS 2 alongside the build | adopted | [Maintenance rules](#how-this-document-is-maintained) |
| 2026-11-04 | Plan v2: robot design in revisions (Rev A-G), Phases 4a/4b/4c, corrected estimates | Merge the maintainer's answers and the research into one plan | adopted | [Plan](PROJECT_PLAN.md) |
| 2026-11-05 | Our own tote AMR from one `robot.yaml`, driven through ros2_control (`hardware:=gz\|mock`), odometry from a `robot_localization` EKF | Teaches URDF, ros2_control and sensor fusion; mock hardware lets CI test the robot without Gazebo | adopted | [ADR-005](decisions/ADR-005-custom-tote-amr.md) |
| 2026-11-05 | CI's build-and-test job: LOCALHOST discovery for launch tests, Gazebo tests reported as skipped (`WAREHOUSE_SKIP_GZ_TESTS=1`) | The runners cannot render the scanner; the mock-hardware tests still run on every PR | adopted | [ADR-004](decisions/ADR-004-ci-pipeline.md) |
| 2026-11-05 | `controllers.yaml` sets the reverse acceleration limits explicitly | Found in review: left unset, braking from reverse fell back to the forward acceleration limit (0.8 instead of 1.5 m/s²); the Phase 3 reverse safety field relies on 1.5 | adopted | [warehouse_description](../src/warehouse_description/README.md#design-notes) |
| 2026-11-05 | The EKF fuses the diff drive's zero sideways speed (vy = 0) | Found in review: unfused, the y variance of `odom` grew without bound (about 4e4 m² after 30 s standing still, 7e5 m² after a minute); robot_localization recommends it | adopted | [ADR-005](decisions/ADR-005-custom-tote-amr.md), plan D3 |
| 2026-11-05 | `robot.launch.py` sets `use_sim_time` and an absolute namespace itself | Found in review: run on its own, its nodes ran on wall time and the EKF published nothing; inside another namespace, the ROS side moved away from the absolute gz names | adopted | [warehouse_bringup](../src/warehouse_bringup/README.md) |
| 2026-11-05 | The scanner self-hit test looks at a box placed 2 m from the robot | Found in review: in the empty world every ray missed, so a blind or misplaced scanner passed | adopted | [warehouse_system_tests](../src/warehouse_system_tests/README.md) |
| 2026-11-05 | Namespaced from Phase 1: every robot node and topic under `/robot1`, the robot's TF on `/robot1/tf` through `SetRemap`, unprefixed frames, gz topics prefixed with the namespace, `/**/` keys in parameter files | Phase 9 then only adds namespaces; gz topics are global, so unprefixed ones would merge robots; plain parameter keys silently match nothing in a namespace | adopted | Plan D7, R2, R12; [Phase 1 report](phase-reports/phase-01-robot-description.md#10-namespaces-and-the-tf-remaps) |
| 2026-11-05 | Sphere wheel collisions, frictionless caster spheres, unnamed `<collision>` elements | Cylinder wheels skewed in-place yaw odometry by 8-9 % in the prototype; Gazebo's URDF converter drops `<gazebo reference>` friction from named collisions (found while building) | adopted | Plan R13; [warehouse_description](../src/warehouse_description/README.md#design-notes) |
| 2026-11-05 | One static bridge file (`robot_bridge.yaml`, relative names, `expand_gz_topic_names: true`) instead of a bridge config generated per robot | `ros_gz_bridge` 1.0.24 expands relative gz names in the bridge's namespace, so one file serves every robot with no generator code | adopted (replaces the plan's per-robot generation) | [warehouse_gazebo](../src/warehouse_gazebo/README.md#topics) |
| 2026-11-05 | `gazebo.launch.py` starts `gz sim` directly instead of `ros_gz_sim`'s `gz_sim.launch.py` | That file runs Gazebo through a shell, so launch's SIGINT stopped only the shell and orphaned Gazebo servers broke later test runs | adopted | [warehouse_gazebo](../src/warehouse_gazebo/README.md#why-gz-sim-is-started-directly) |
| 2026-11-05 | Launch tests: each file in its own ROS domain (`run_test_isolated.py`), a private `GZ_PARTITION` for Gazebo, exit code 0 required from every process after one SIGINT, no leftover process; robot numbers read from `robot.yaml` and `controllers.yaml` | Tests must not see each other or other sessions, must catch shutdown leaks, and must not check stale values after a robot revision | adopted | [warehouse_system_tests](../src/warehouse_system_tests/README.md) |
| 2026-11-05 | Values the plan left open: wheel width 0.04 m, wheel motor effort limit 20 N m, mass split body 32.6 kg / wheels 2 x 1.0 kg / casters 2 x 0.2 kg, sensors massless | URDF needs them; totals (35 kg, centre of mass 0.14 m) match plan §2.6 and are tested | adopted | [warehouse_description](../src/warehouse_description/README.md#mass-budget) |
| 2026-11-05 | Upstream Gazebo shutdown hang ([gz-sim#4173](https://github.com/gazebosim/gz-sim/issues/4173)) accepted as a known issue: re-run when only the exit-code check fails | About 1 Gazebo test run in 15; not caused by this repository, CI unaffected (it skips the Gazebo test) | adopted (until fixed upstream) | [warehouse_system_tests](../src/warehouse_system_tests/README.md) |
| 2026-11-05 | Phase 1 built, reviewed, fixed and verified by AI agents (Claude Code): one builder, one review and fix round, three parallel final verifiers (CI reproduction, Gazebo acceptance, code review against the Jazzy sources) | Same practice as Phase 0; every acceptance criterion is measured, and every exercise in the report was run again in a clean shell (except the windowed demo) | done | [Phase 1 report](phase-reports/phase-01-robot-description.md#what-happened) |
| 2026-11-05 | Learning-report exercises and local runs use a clean shell (`wh_shell`: only `/opt/ros/jazzy` and this workspace, `ROS_DOMAIN_ID=42`, LOCALHOST discovery) | The maintainer's normal shell sources other workspaces and discovers across the LAN, which can shadow packages and mix graphs | adopted | [Phase 1 report](phase-reports/phase-01-robot-description.md#before-you-start) |
| 2026-11-05 | A render-free test of the Gazebo-side files (`warehouse_gazebo/test/test_gazebo_config.py`) runs in every pull request: the systems every world loads, the bridge file against the URDF (gz names expanded the way the bridge expands them, the same relative name and the matching ROS type on the ROS side, the ground-truth system's filename), and the caster friction after `gz sdf -p` | CI skips the Gazebo launch test, so a typo in the world or the bridge file passed CI: `gz sdf -k` accepts a misspelt system, a bridge entry for a topic nothing publishes fails silently, and one with a misspelt ROS type only logs a warning. Each of 19 single mistakes made on purpose fails the test, among them a misspelt system, an absolute scan topic, a misspelt or wrong ROS type, a misspelt plugin filename and a named caster collision | adopted | [warehouse_gazebo](../src/warehouse_gazebo/README.md#tests) |
| 2026-11-05 | Launch files keep rcutils' default log format: `sim.launch.py` no longer sets `RCUTILS_CONSOLE_OUTPUT_FORMAT`, and plan §6 says to override it only to add fields | The value it set was rcutils' built-in default (`[{severity}] [{time}] [{name}]: {message}`), so it changed nothing, and a reader would copy a no-op | adopted | Plan §6 |
| 2026-11-05 | Phase 1 acceptance reworded: after the key is released, the controller starts braking when `cmd_vel_timeout` (0.5 s) expires and stands still about 0.33 s later, at the -1.5 m/s² limit | "Stops within `cmd_vel_timeout`" cannot hold with a deceleration limit; the new wording is what the tests measure | replaced (0.15 s since 2026-11-06) | Plan, Phase 1 |
| 2026-11-05 | Phase 1 polish round before the pull request: the final review's six low findings and a fact-check of the learning report fixed (EKF explanation, RViz's absolute namespace, a discovery wait in the Gazebo test, the plan's references to ADR-005 and the static bridge file); the IMU sensor-name risk is now recorded in plan R2 and stays open until Phase 9. Two verifiers then checked the round, and a second, smaller round fixed what they found (the ROS side of the bridge test, the README status, the EKF speed explanation, two wordings) | Every document agrees with the code and with re-measured numbers before the merge | done | [Phase 1 report](phase-reports/phase-01-robot-description.md#what-happened) |
| 2026-11-05 | Automatic checks paused: CI runs only by hand, `main` needs a pull request but no passing checks, the pre-commit hook is optional, and a plain `colcon build` builds without tests (`colcon.meta` at the workspace root passes `-DBUILD_TESTING=OFF` to every package) | The owner prefers to verify changes by building and running them while the project is under heavy development; the tests stay in the repository for later | adopted (until resumed) | [ADR-004](decisions/ADR-004-ci-pipeline.md), [CONTRIBUTING](../CONTRIBUTING.md#build-and-test) |
| 2026-11-05 | The owner's design review of Phase 1: the 2D lidar becomes a 360° puck standing on the deck near the front edge (scan plane 0.34 m, 720 rays 0.5° apart) instead of a front-left corner scanner in a groove round the body (scan plane 0.20 m, 264°), accepting that a loaded tote blocks much of its rear view. The body is restyled in safety yellow and charcoal from simple, clearly separate parts (skirt, panels, bumpers, deck top, tote tray, emergency stop, status light); the status light's colour is one property, which Phase 9 turns into a per-robot argument | A simpler, familiar sensor layout and a robot that reads as a warehouse AMR. Found while building it: RViz draws all visuals of a link in one colour, so each coloured part is its own link. Open for Phase 3: what the puck cannot see (below 0.34 m, behind a tote), and when a tote is modelled on the robot (no phase does it yet) | adopted | [ADR-005](decisions/ADR-005-custom-tote-amr.md), plan §2.6 (Rev B, Rev C) |
| 2026-11-05 | The repository root is now the colcon workspace: the repository is cloned as `~/warehouse_ws`, packages moved to `src/`, `.gitignore` keeps `build/`, `install/` and `log/` out; CI, the dev container, README, CONTRIBUTING and the reports follow. History is kept (the move is one rename commit) | The owner wants one repository at the workspace root that VS Code finds directly; GitHub still finds the README, licence and `.github/` at the root | adopted | [ADR-006](decisions/ADR-006-workspace-root-repository.md) |
| 2026-11-05 | Phase 2 warehouse: a 30 x 20 m hall, 4 pick aisles in 2 blocks (8 shelf rows, 414 slots), an open handling floor in the east; plain uniform racks with no totes and no per-aisle colours until Phase 7 (colours live in the layout's `style:` block) | The owner's choices in the Phase 2 plan review: big enough for 3 robots and aisle traffic in Phase 11, small enough to map quickly; totes arrive with their tags in Phase 7 | adopted | [warehouse_layout](../src/warehouse_layout/README.md) |
| 2026-11-05 | One `layout.yaml` generates the Gazebo world, the location registry and the expected inventory; the generated files are committed, byte-identical run to run, and `generate --check` catches a stale one; fit rules are tests that read the robot's size from `robot.yaml` (`warehouse_description.robot_config`) | Plan D8: one source of truth, so the world, the waypoints and the inventory can never disagree; a reviewer sees every change to the world in the diff | adopted | [warehouse_layout](../src/warehouse_layout/README.md#generated-files) |
| 2026-11-05 | The committed SLAM map will come from a scripted survey route; a teleop map is a learning-report exercise | A scripted route is repeatable, so the map can be rebuilt after a layout change | planned (Phase 2) | Plan, Phase 2 |
| 2026-11-06 | The generated warehouse is merged into `main` before mapping starts | The owner reviews the world first; mapping and localization follow in a later pull request | done | [warehouse_layout](../src/warehouse_layout/README.md) |
| 2026-11-06 | ML/DL perception research resumed and finished: use cases, synthetic data from Gazebo, training on Colab, CPU inference and ROS 2 integration, each reviewed against the current robot and layout | The owner asked to resume it; the report recommends one class `tote`, D-FINE-N run on the CPU from C++, and a new `UNLABELED` discrepancy | done | [ML report](research/ML_PERCEPTION.md) |
| 2026-11-06 | Perception design document: the ArUco method (Phases 7-8) and the learned detector (8b) in one design, from the ArUco research, its skeptical review and the ML report, then two more reviews. It proposes 29 changes to the plan (among them: choose the IPPE pose that fits best, never the one nearest the stored dock yaw; a dock tag tilted 20°; M2 re-estimated at about 75-119 h) and asks five questions (O3-O7); the plan is not edited until they are answered | The review found that the plan's rule (pick the IPPE solution nearest the database dock yaw) makes a dock rotated by 5° worse; the owner wants an in-depth reference before Phase 7 | in review | [Perception design](PERCEPTION.md) |
| 2026-11-06 | Measured what this laptop can run (i5-1335U, 15 GiB, Iris Xe): today's stack in real time on about 1 of 12 threads; one robot with Nav2, two side cameras and ArUco at RTF 0.99; three robots with three Nav2 stacks at RTF 0.59-0.68 at the world's 1 ms physics step, about 1.0 at 2 ms or 5 ms. Phases 2-6 comfortable, 7-8b work with care, 9-12 need a larger physics step (to be tested in Phase 3) and Gazebo without its window | The owner asked whether the laptop can carry the project to the final showcase | done | [PC assessment](research/PC_ASSESSMENT.md), plan A4, R3 |
| 2026-11-06 | `cmd_vel_timeout` 0.5 → 0.15 s in `controllers.yaml`. The rule this creates: every `cmd_vel` publisher publishes continuously at 10 Hz or more while the robot should move (Nav2's velocity smoother 20 Hz, its docking server 50 Hz, the tests 20 Hz), or the robot brakes between commands; a command whose stamp is already 0.15 s old on arrival is ignored. The system tests now derive their time windows from the value | The owner's request after driving the warehouse by teleop: each key event drove the robot for 0.5 s, too long for small, accurate moves. Measured in headless Gazebo (keyboard emulated like teleop on this desktop, 3 runs each, real-time factor 1.00): a tap of `j` turns 3.9° instead of 28.6°, a tap of `i` moves 1.4 instead of 15.4 cm; after a held key is released the robot stands still 0.49 s after the last key event instead of 0.84 s (0.15 s timeout plus 0.33 s braking). The cost: teleop sends one command per key event and GNOME repeats a held key only after 500 ms, so a held key now stops for 0.20-0.30 s before it drives on (0.5 s bridged that gap). Removing that stop needs a teleop that sees key releases: the teleop window (next row) | adopted | [controllers.yaml](../src/warehouse_description/config/controllers.yaml), plan R1, [warehouse_bringup](../src/warehouse_bringup/README.md#driving-with-teleop) |
| 2026-11-06 | The teleop window, `ros2 run warehouse_bringup teleop_window`: a Qt window that publishes when a key is pressed, repeats the command at 20 Hz while it is held, sends three zero commands over 0.1 s when it is released and then nothing, and sends one zero command when the window loses the keyboard (a dead-man switch). Speeds start at 0.3 m/s and 0.6 rad/s, so braking takes 0.2 s. The key logic is plain Python, tested with pytest; `teleop_twist_keyboard` stays as the terminal alternative, and `cmd_vel_timeout` stays 0.15 s | The owner, after driving with 0.15 s: "remove the delay completely, I am not feeling it that in control". A terminal never sees a key release, so teleop_twist_keyboard leaves the stop to the timeout and stops a held key until auto-repeat starts. Measured in headless Gazebo (the real window offscreen, synthetic key events, ground truth, 3 runs each, real-time factor 1.00), at 0.5 m/s and 1.0 rad/s: the first zero command arrives under 2 ms after the release; the robot stands still 0.34 s after it instead of 0.49 s, 8.3 cm or 9.7° further on instead of 16.0 cm or 18.2°; a held key never stops (0.2-0.3 s before). At 0.3 m/s and 0.6 rad/s: 0.21 s and 3.1 cm or 3.5°. What remains is braking at the controller's deceleration limits. Cost: a Python node on sim time, 0.4-0.6 cores while open, the same as teleop_twist_keyboard | adopted | [warehouse_bringup](../src/warehouse_bringup/README.md#the-teleop-window) |

## Planned features

Short list per milestone; details and acceptance criteria are in the plan. All **planned**.

- **M1 Core AMR (`v1.0.0`)**
  - Phase 1's demo GIF: teleop with the RViz sensor view, recorded by the maintainer (steps in
    [exercise 7 of the Phase 1 report](phase-reports/phase-01-robot-description.md#exercise-7-run-the-demo-with-windows-and-record-the-gif)).
  - SLAM mapping of the warehouse from a scripted survey route, and AMCL localization
    (Phase 2).
  - Nav2 navigation with keep-out and speed zones and a collision monitor (Phase 3).
  - Custom interfaces and a behaviour-tree task executor (Phase 4a).
  - Order manager, order generator and KPI tracking (Phase 4b).
  - Operator dashboard v1, our own Flask web app: overview, orders, live map, live updates
    (Phase 4c).
  - Simulated battery, autonomous docking and recharging, diagnostics (Phase 5).
  - Hardening and the `v1.0.0` release with a recorded demo (Phase 6).
- **M2 Perception (`v2.0.0`)**
  - ArUco tags on totes and docks, detected by robot cameras (Phase 7).
  - Inventory patrol with discrepancy reports (Phase 8).
  - Optional: learned detector trained on Google Colab, deployed as an ONNX model (Phase 8b).
- **M3 Fleet (`v3.0.0`)**
  - Three namespaced robots in one simulation (Phase 9).
  - Fleet manager with greedy, then auction-based task allocation (Phases 10-11).
  - Aisle traffic control (Phase 11).
  - Dashboard polish and the final showcase scenario (Phase 12).
- **Stretch**
  - Mobile manipulation: navigate to a shelf, then pick with an arm (Phase 13).
- **After all phases**
  - Deployment of the system and dashboard on the LAN.

## How this document is maintained

- **Every feature PR updates this file:** add or update the feature's entry under
  [Features](#features) (what, does, how to use, where), add a dated row to the
  [log](#ideas-and-decisions-log) for any new idea or decision, and move items out of
  [Planned features](#planned-features) once they exist.
- **Every finished phase adds a learning report** at
  `docs/phase-reports/phase-NN-<name>.md` (for example `phase-01-robot-description.md`), updates the
  [Progress](#progress) table, and gets its `phase-N` tag.
- A decision that is hard to reverse also gets an ADR in [decisions/](decisions/); the log
  row links to it.
- Keep entries short and factual. Only list features that exist on `main`.
