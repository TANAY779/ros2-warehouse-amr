# Project status

The living record of this project: what exists, what was decided and why, and what comes
next. Every pull request that adds or changes a feature updates this file (see
[How this document is maintained](#how-this-document-is-maintained)). For the story of each
finished phase, written to teach, see [phase-reports/](phase-reports/).

**Where to read more:** the learning reports in [phase-reports/](phase-reports/) (start with
[Phase 0](phase-reports/phase-00-foundation.md)); the phased plan with architecture and
acceptance criteria in [PROJECT_PLAN.md](PROJECT_PLAN.md); the decision records in
[decisions/](decisions/). New to terms such as ament_lint, rosdep, colcon, ruleset or ADR? See
the [glossary in the Phase 0 report](phase-reports/phase-00-foundation.md#glossary).

## At a glance

- **Current phase (as of 2026-11-04):** Phase 0 (repository foundation) is finished. The
  revised project plan (v2) is adopted, and Phase 1 (robot description and first drive in
  Gazebo) is next.
- **Last updated:** 2026-11-04
- **What runs today:** tooling only. The repository builds and tests in CI, the automatic
  checks (lint gates) work, and the dev container works. No robot, world or node exists yet.

### Progress

| Milestone | Phase | Scope | Status |
|---|---|---|---|
| M0 Foundation | 0 | Repository, lint gates, CI, dev containers, ADRs | **done** ([report](phase-reports/phase-00-foundation.md)); tag `phase-0` not yet created (as of 2026-11-04) |
| M1 Core AMR (`v1.0.0`) | 1 | Robot description, ros2_control drive, lidar + IMU, EKF, first drive in Gazebo | next |
| | 2 | Warehouse world generated from a layout file, mapping, localization | planned |
| | 3 | Navigation with keep-out and speed zones, safety | planned |
| | 4a | Interfaces + behaviour-tree task executor | planned |
| | 4b | Order manager, order generator, KPIs | planned |
| | 4c | Operator dashboard v1 (Flask): overview, orders, live map, live updates | planned |
| | 5 | Battery, autonomous docking, diagnostics | planned |
| | 6 | Core hardening and the `v1.0.0` release | planned |
| M2 Perception (`v2.0.0`) | 7 | ArUco fiducial perception (shelf and dock tags) | planned |
| | 8 | Inventory patrol and discrepancy reports | planned |
| | 8b (optional) | Learned ML/DL perception, trained on Google Colab | planned (optional; research paused) |
| M3 Fleet (`v3.0.0`) | 9 | Multi-robot bring-up | planned |
| | 10 | Fleet manager v1, greedy task allocation | planned |
| | 11 | Auction allocation and aisle traffic control | planned |
| | 12 | Dashboard polish and the showcase scenario | planned |
| Stretch | 13 | Mobile manipulation | stretch |

This table follows [PROJECT_PLAN.md](PROJECT_PLAN.md) (version 2), which holds each phase's
scope and acceptance criteria.

## Features

Only features that exist on `main` are listed. Commands assume the standard layout: the
colcon workspace at `~/warehouse_ws` and this repository at `~/warehouse_ws/src/ros2-warehouse-amr`.

### Repository skeleton

- **What it is:** the files every professional repository needs before any code: licence,
  README, editor and git settings, and the list of third-party sources.
- **What it does:** `.editorconfig` gives every editor the same indentation and line endings;
  `.gitattributes` forces LF line endings and marks images, bags and models as binary;
  `.gitignore` keeps colcon output, bags, datasets and model weights out of git (small test
  fixtures under `test/data/` are allowed); `dependencies.repos` lists source dependencies
  that have no Jazzy binary (none yet) for `vcs import`.
- **How to use it:** nothing to run; editors and git pick these files up automatically. To
  import third-party sources: `vcs import --input src/ros2-warehouse-amr/dependencies.repos src`
  from the workspace root.
- **Where it lives:** `LICENSE` (Apache-2.0), `README.md`, `.editorconfig`, `.gitattributes`,
  `.gitignore`, `dependencies.repos`.

### `warehouse_bringup` package and the ament lint gates

- **What it is:** the first ROS 2 package. It will hold the top-level launch files and
  per-environment configuration (Phase 1 adds `sim.launch.py`). Today it contains only the
  build and lint setup that every later C++ package copies.
- **What it does:** `colcon test` runs the `ament_lint` linters on the package: copyright
  headers, CMake style, `package.xml` schema validation, and (as soon as C++ or Python files
  exist) clang-format, cpplint, cppcheck, flake8 and pep257. uncrustify is switched off in
  favour of clang-format, and the build fails loudly if the clang-format linter is missing
  instead of skipping it.
- **How to use it:** from `~/warehouse_ws`:

  ```bash
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src --ignore-src -y
  colcon build --symlink-install
  colcon test && colcon test-result --verbose
  ```

  Expect `Summary: 5 tests, 0 errors, 0 failures, 0 skipped`.
- **Where it lives:** `warehouse_bringup/` (`package.xml`, `CMakeLists.txt`, `README.md`);
  rules in [ADR-003](decisions/ADR-003-formatting-and-linting.md) and the "New packages"
  section of [CONTRIBUTING.md](../CONTRIBUTING.md).

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

- **What it is:** a workflow that runs on every pull request, on every push to `main`, and on
  demand.
- **What it does:** job **Lint (pre-commit)** runs every pre-commit hook on every file; job
  **Build & test (ROS 2 Jazzy)** runs, inside the official `ros:jazzy-ros-base` container,
  the same `vcs import`, `rosdep install`, `colcon build`, `colcon test` and
  `colcon test-result` commands a developer runs, then uploads the logs and test results as
  the `colcon-logs` artifact. Superseded pull request runs are cancelled; every commit on
  `main` gets its own result. Each job takes about 35-45 s.
- **How to use it:** open a pull request; both checks appear on it. From a terminal:
  `gh run list --workflow ci.yml`, then `gh run view <run-id>`. To reproduce a failure
  locally, see [ADR-004](decisions/ADR-004-ci-pipeline.md) (Consequences).
- **Where it lives:** `.github/workflows/ci.yml`; rationale in
  [ADR-004](decisions/ADR-004-ci-pipeline.md).

### Branch protection (ruleset "Protect main")

- **What it is:** a GitHub repository ruleset on the default branch.
- **What it does:** changes reach `main` only through a pull request, and only when both CI
  checks (`Lint (pre-commit)` and `Build & test (ROS 2 Jazzy)`) pass. Force-pushes and
  deleting `main` are blocked. No approving review is required (single maintainer).
- **How to use it:** work on a branch, push it, open a PR, merge when green. Inspect the
  rules with `gh api repos/TANAY779/ros2-warehouse-amr/rules/branches/main --jq '.[].type'`
  (prints `deletion`, `non_fast_forward`, `pull_request`, `required_status_checks`).
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
- **Where it lives:** [docs/decisions/](decisions/) (ADR-001 to ADR-004 so far).

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
| 2026-11-03 | ML/DL perception research paused by the maintainer (3 of 4 topics done) | Focus on the core stack first | paused | Plan v2, Phase 8b |
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

## Planned features

Short list per milestone; details and acceptance criteria are in the plan. All **planned**.

- **M1 Core AMR (`v1.0.0`)**
  - Custom tote-carrying robot: URDF/xacro description, `ros2_control` differential drive,
    2D lidar and IMU, EKF odometry fusion, teleoperation in Gazebo (Phase 1).
  - Warehouse world generated from a layout file; SLAM mapping and AMCL localization
    (Phase 2).
  - Nav2 navigation with keep-out and speed zones and a collision monitor (Phase 3).
  - Custom interfaces and a behaviour-tree task executor (Phase 4a).
  - Order manager, order generator and KPI tracking (Phase 4b).
  - Operator dashboard v1, our own Flask web app: overview, orders, live map, live updates
    (Phase 4c).
  - Simulated battery, autonomous docking and recharging, diagnostics (Phase 5).
  - Hardening and the `v1.0.0` release with a recorded demo (Phase 6).
- **M2 Perception (`v2.0.0`)**
  - ArUco tags on shelves and docks, detected by robot cameras (Phase 7).
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
