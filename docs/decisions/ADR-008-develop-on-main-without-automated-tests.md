# ADR-008: Develop on main without automated tests

- **Status:** accepted (supersedes [ADR-003](ADR-003-formatting-and-linting.md) and
  [ADR-004](ADR-004-ci-pipeline.md))
- **Date:** 2026-11-07
- **Plan reference:** D14 and D18 (both replaced); plan §6, rows "Testing pyramid" and "Git"

## Context

Since 2026-11-05 the automatic checks were paused ([ADR-004](ADR-004-ci-pipeline.md), update at
the end): CI ran only when started by hand, `main` needed a pull request but no passing checks,
the pre-commit hook was optional, and `colcon.meta` built every package without tests. The
tests, the lint configuration and the CI workflow stayed in the repository for later.

Kept but not run, they still cost work: every change to the robot, the layout or a launch file
also meant updating tests that nobody ran, every new package needed the lint set-up, and every
change needed a branch and a pull request. The owner builds this project alone and decided on
2026-11-07: stop linting and testing, remove the testing files, and keep every commit on
`main`, with no side branches.

## Decision

Develop on `main` only, with no automated tests, lint checks or CI. A change is checked by
building it and running the system.

**Removed on 2026-11-07:**

- The CI workflow (`.github/workflows/ci.yml`), Dependabot (`.github/dependabot.yml`, which only
  updated the workflow's action versions through pull requests) and the pull request template.
- The pre-commit configuration (`.pre-commit-config.yaml`) and the lint configuration (`.flake8`,
  `.yamllint`).
- `colcon.meta`, which only switched the tests off.
- Every test: the `test/` folder of every package (unit tests, launch tests, the ament lint
  tests, `conftest.py`), the `if(BUILD_TESTING)` block of every `CMakeLists.txt`, every
  `test_depend` in every `package.xml`, and the test settings in `warehouse_layout`'s
  `setup.py`. In `warehouse_system_tests`, the launch tests and the helpers that only they used.
- In the dev container: pre-commit and its `pre-commit install`, the `ament_clang_format` and
  `ament_cmake_clang_format` packages (only `colcon test` used them), the
  `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` variable, and VS Code's flake8 extension (without
  `.flake8` it would lint with flake8's own defaults).

**Kept:**

- The robot and every tool: `warehouse_layout`'s `preview` (one `[PASS]` or `[FAIL]` line per
  fit rule), `generate` and `generate --check`, `rasterize` and `check_map`; the teleop window;
  and `warehouse_system_tests` under the same name, with its simulation test tools
  `drive_route`, `evaluate_localization` and `map_timelapse`. The tools measure the running
  system, which is how acceptance is checked now.
- The editor formatting files `.editorconfig` and `.clang-format`, and clang-format in the dev
  container. Editors apply them; nothing enforces them.
- The conventions in [CONTRIBUTING.md](../../CONTRIBUTING.md): Conventional Commits, the licence
  header, the include style, the media size limits.

**Workflow from now on:**

- Commit directly to `main` with a Conventional Commits message and push. No feature branches,
  no pull requests.
- The GitHub ruleset "Protect main" keeps blocking force-pushes and the deletion of `main`. The
  owner removes its "Require a pull request before merging" rule on GitHub (Settings → Rules →
  Rulesets → Protect main); until then GitHub refuses a direct push to `main`.
- **How a change is checked:** build the workspace (`colcon build --symlink-install`), run what
  changed (`ros2 launch warehouse_bringup sim.launch.py`, with `headless:=true`, `hardware:=mock`
  or `world:=warehouse` as needed) and measure it: the commands in
  [Checking the robot by hand](../../src/warehouse_bringup/README.md#checking-the-robot-by-hand)
  (controllers active, `/robot1/scan` at about 10 Hz, the TF tree), and for the layout, the map
  and localization the tools above. A phase's acceptance criteria are checked the same way.

## Alternatives considered

| Option | Why not |
|---|---|
| Keep the paused state: tests and CI in the repository, switched off | The files must still follow every change or they rot, and the owner wants them gone |
| Keep only the fast pure-Python unit tests, drop the launch tests and lint | Still a test suite to maintain and run; the owner chose to remove everything test-related |
| Keep branches and pull requests, without checks | Extra steps with nothing to gate: the only maintainer would review their own pull requests |

## Consequences

- Changes are quicker: no test to update, no lint finding to fix, no branch or pull request.
  A plain `colcon build` needs no test or lint tools.
- **What this gives up:**
  - A regression is found only when someone runs the system. Before, a test failed when
    `controllers.yaml` drifted from `robot.yaml`, a generated world went stale, a topic left the
    robot's namespace or the scanner saw the robot itself; now such a mistake shows up later, in
    a run, or not at all.
  - Numbers that several files share (`robot.yaml` with `controllers.yaml` and the SLAM and AMCL
    parameters) are kept in step by hand; the files say so in their comments.
  - Style can drift unless the editor applies `.editorconfig` and `.clang-format`.
  - `main` is the only line of work, so a half-finished change is on `main` as soon as it is
    pushed: commit working steps, and undo one with `git revert`.
  - The repository shows visitors no CI badge or passing checks.
- The deleted files stay in git history: `git show <commit>^:<path>`, with the commit that
  removed them, brings one back.
- [ADR-003](ADR-003-formatting-and-linting.md) and [ADR-004](ADR-004-ci-pipeline.md) are
  superseded. What [ADR-001](ADR-001-record-architecture-decisions.md),
  [ADR-005](ADR-005-custom-tote-amr.md) and [ADR-006](ADR-006-workspace-root-repository.md) say
  about pull requests, tests or CI, the Phase 0 and Phase 1 reports, and the log rows of
  [PROJECT_STATUS.md](../PROJECT_STATUS.md) before 2026-11-07 record how the project worked then.
