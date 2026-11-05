# Phase 0 report: repository foundation

- **Milestone:** M0 Foundation
- **Dates:** 2026-11-02 to 2026-11-04
- **Tag:** `phase-0` (not yet created as of 2026-11-04)
- **Result:** the repository builds and tests in CI, the automatic checks block bad code, the
  dev container gives a reproducible environment, and `main` is protected. No robot code yet.

This report is written for someone who knows basic Linux, Python and C++ but is new to
professional ROS 2 tooling. It explains what was built, why each piece exists, what went
wrong on the way, and how to try every piece yourself. Paths are relative to the repository
root unless they start with `~/warehouse_ws`.

The files were written and verified with AI assistance (Claude Code agents). Verification ran
in Docker on scratch copies of the repository, and every fix was checked again in a second
round.

> **Words you will meet** (full list in the [Glossary](#glossary))
>
> - **colcon workspace:** the folder colcon (the ROS 2 build tool) builds; source code lives in
>   its `src/`.
> - **lint / linter:** an automatic check of code style or likely bugs, without running the
>   code. A **lint gate** is a linter that blocks a change when it fails.
> - **ament_lint:** ROS 2's set of linters, run as tests by `colcon test`.
> - **cppcheck:** one of those linters; it finds bugs such as reading past the end of an array.
> - **pre-commit:** a tool that runs checks automatically before every `git commit`.
> - **CI (continuous integration):** GitHub builds and tests every change on a clean machine.
> - **ruleset:** GitHub's rules for a branch, such as "merge only through a pull request with
>   green CI".
> - **ADR (architecture decision record):** a short document recording one decision and why.

## Contents

- [TL;DR](#tldr)
- [What was built](#what-was-built)
- [Concepts](#concepts)
  1. [colcon workspace vs repository](#1-colcon-workspace-vs-repository)
  2. [Anatomy of a ROS 2 package](#2-anatomy-of-a-ros-2-package)
  3. [The ament_lint linters](#3-the-ament_lint-linters)
  4. [pre-commit](#4-pre-commit)
  5. [CI with GitHub Actions](#5-ci-with-github-actions)
  6. [Dev containers](#6-dev-containers)
  7. [Architecture decision records](#7-architecture-decision-records)
  8. [Git workflow](#8-git-workflow)
- [What happened](#what-happened)
- [Try it yourself](#try-it-yourself)
- [Pitfalls and lessons](#pitfalls-and-lessons)
- [Glossary](#glossary)
- [Further reading](#further-reading)
- [What comes next](#what-comes-next)

## TL;DR

Before writing any robot code, we built the machinery that keeps code correct:

- A **repository** that lives inside a build folder (the colcon workspace), with one ROS 2
  package (`warehouse_bringup`) whose only job so far is to carry the automatic-check setup
  (lint setup) every later package copies.
- **Two layers of automatic checks** that agree with each other: checks on your machine before
  every commit (pre-commit; fast) and ROS 2's own linters in `colcon test` (ament_lint;
  complete, also in CI). Tool versions are pinned so both layers give the same answer.
- **Automatic build and test on GitHub** for every change (CI), running exactly the commands a
  developer runs inside the official ROS image, and **branch rules** (a ruleset) that make
  green CI a condition for merging into `main`.
- A **ready-made development environment** in Docker (a dev container, plus a headless
  variant) that reproduces the CI toolchain on any machine.
- **Short decision records** (ADRs) that say why each choice was made.

The most important lesson came from verification: four of the checks *looked* green but were
checking nothing (a skipped cppcheck, a crashing flake8 hidden by the absence of Python files,
a file-size check that never ran in CI, and a licence check that the package template skips).
A passing check is only evidence if you have seen it fail.

## What was built

| Piece | Files | Purpose |
|---|---|---|
| Repository skeleton | `README.md`, `LICENSE`, `.editorconfig`, `.gitattributes`, `.gitignore`, `dependencies.repos` | Licence, entry point, consistent editors and line endings, keep build output and large data out of git, list of source dependencies |
| `warehouse_bringup` package | `warehouse_bringup/package.xml`, `CMakeLists.txt`, `README.md` | First ROS 2 package; holds the lint setup; launch files arrive in Phase 1 |
| Formatter and linter config | `.clang-format`, `.flake8`, `.yamllint` | Same rules as ROS 2's `ament_lint` |
| pre-commit hooks | `.pre-commit-config.yaml` | Check every commit locally |
| CI workflow | `.github/workflows/ci.yml` | Lint, build and test on every PR and every push to `main` |
| Dependabot | `.github/dependabot.yml` | Monthly update PRs for GitHub Actions |
| PR template | `.github/pull_request_template.md` | What, why, how tested, checklist |
| Dev containers | `.devcontainer/` (`Dockerfile`, `devcontainer.json`, `headless/devcontainer.json`, `post-create.sh`) | Reproducible environment with GUI/GPU, or headless |
| ADRs | `docs/decisions/ADR-000` to `ADR-004` | Why the repository looks the way it does |
| Contribution rules | `CONTRIBUTING.md` | Branches, commits, tags, style, new-package checklist |
| Project plan | `docs/PROJECT_PLAN.md` | Phases, architecture, acceptance criteria |
| Branch ruleset | GitHub settings ("Protect main") | PR plus both CI checks required; no force-push or deletion |

## Concepts

### 1. colcon workspace vs repository

**What it is.** A *colcon workspace* is a directory that colcon, the ROS 2 build tool, builds
as a unit. By convention it has a `src/` folder with the source code, and colcon creates
`build/` (intermediate files), `install/` (what you source and run) and `log/` next to it. A
*repository* is what git tracks. They are different things: the workspace is a place on your
disk, and the repository is one folder inside it.

**How it works here.** [ADR-002](../decisions/ADR-002-repository-layout.md) puts this
repository at `~/warehouse_ws/src/ros2-warehouse-amr`, with one subfolder per ROS 2 package:

```text
warehouse_ws/                 colcon workspace (not version-controlled)
├── build/ install/ log/      colcon output
└── src/
    └── ros2-warehouse-amr/   this repository: one folder per ROS 2 package
```

> **Update 2026-11-05:** the repository root is now the colcon workspace itself
> ([ADR-006](../decisions/ADR-006-workspace-root-repository.md)): clone it as `~/warehouse_ws`;
> packages live in `~/warehouse_ws/src/`. The commands in this report are updated; the layout
> described here is how Phase 0 started.

colcon, run from the workspace root, searches the folders below it recursively for
`package.xml` files (skipping its own `build/`, `install/` and `log/`), so it does not matter
that the packages sit one level deeper. Third-party sources that have no binary
package go in `dependencies.repos` and are cloned next to the repository with
`vcs import --input dependencies.repos src` (the list is empty today,
but CI and the dev container already run the step, so adding an entry cannot break them).

**Why industry does it.** Build output never enters the repository, so `git status` stays
clean and nobody commits a 200 MB `build/` folder by accident. One repository with many
packages (a *monorepo*) lets one pull request change a message definition and every package
that uses it together. CI can clone the repository into a fresh `src/` and build it the same
way you do.

### 2. Anatomy of a ROS 2 package

**What it is.** A ROS 2 package is a folder with a `package.xml` (the manifest: name,
version, licence, dependencies) and a build description: `CMakeLists.txt` for C++ packages
(build type `ament_cmake`) or `setup.py` for Python packages (`ament_python`).

**How it works here.** Open `warehouse_bringup/package.xml`:

- `<package format="3">` is the current manifest format. The second line points at its XML
  schema, which the `xmllint` test downloads and validates against (that is why
  `colcon test` needs network access).
- `<name>`, `<version>` (`0.1.0` until the `v1.0.0` release), `<description>`,
  `<maintainer>`, `<license>Apache-2.0</license>` and a repository `<url>`.
- `<buildtool_depend>ament_cmake</buildtool_depend>`: the build system.
- Three `<test_depend>` entries: `ament_lint_auto`, `ament_lint_common` and
  `ament_cmake_clang_format`. Test dependencies are only needed to run the tests.
- `<export><build_type>ament_cmake</build_type></export>` tells colcon how to build it.

Dependency tags have different meanings: `build_depend` (needed to compile),
`exec_depend` (needed to run), `depend` (shorthand for build, build-export and run),
`test_depend` (needed for tests),
`buildtool_depend` (the build tool itself). Getting these right matters because
**rosdep** reads them: `rosdep install --from-paths src --ignore-src -y` looks at every
`package.xml` under `src/`, maps each dependency key to a system package (for example
`ament_cmake_clang_format` → `ros-jazzy-ament-cmake-clang-format`), and installs it with apt.
`--ignore-src` skips dependencies that are themselves packages in your workspace.

Now `warehouse_bringup/CMakeLists.txt` (excerpt, comments removed):

```cmake
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

- `BUILD_TESTING` is on by default; the block only adds tests.
- `ament_lint_auto_find_test_dependencies()` calls `find_package(<dep> QUIET)` for every
  `test_depend` in `package.xml`. Each linter package found registers a hook, and those hooks
  add their tests when `ament_package()` runs.
- `QUIET` means a missing linter is skipped **silently**. That is why
  `ament_cmake_clang_format` is also found with `REQUIRED`: if it is not installed, the
  configure step fails with a clear CMake error instead of quietly dropping the C++ format
  check.
- `AMENT_LINT_AUTO_EXCLUDE` removes uncrustify, the other C++ formatter that
  `ament_lint_common` brings in, because it formats differently from clang-format.
- `ament_package()` must be the last call. It generates the files that let other packages
  `find_package(warehouse_bringup)` and registers the package in the ament index.

**Why industry does it.** The manifest makes dependencies machine-readable, so a new machine
or CI can install everything with one rosdep command instead of a README full of apt lines.
Declaring test dependencies separately keeps runtime images small.

### 3. The ament_lint linters

**What it is.** `ament_lint` is ROS 2's family of linters, run as ordinary tests by
`colcon test`. `ament_lint_common` is the standard bundle.

**How it works here.**

| Linter | Checks | Registered when | Here |
|---|---|---|---|
| `copyright` | Every `.py`, C/C++ and `.cmake` file starts with a known licence header | always | on (checks 0 files today) |
| `lint_cmake` | CMake style in `CMakeLists.txt` | always | on |
| `xmllint` | `package.xml` is valid against the format 3 schema | always | on (needs network) |
| `clang_format` | C++ formatting matches the ROS 2 clang-format style | C/C++ files exist | on (added by us; not in `ament_lint_common`) |
| `cpplint` | Google C++ style rules (include order, header guards, ...) | C/C++ files exist | on |
| `cppcheck` | Static analysis: out-of-bounds access, null dereference, leaks | C/C++ files exist | on, **only with `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` set** |
| `uncrustify` | Alternative C++ formatter | C/C++ files exist | **excluded** |
| `flake8` | Python style and errors | Python files exist | on |
| `pep257` | Python docstring style | Python files exist | on |

Three details matter in practice:

- **Tests are registered at configure time.** CMake decides which lint tests exist when it
  configures the package. If you add the first `.cpp` file to a package that is already
  configured, a plain `colcon build` may not reconfigure, and the new tests do not appear.
  Use `colcon build --cmake-force-configure` (or delete `build/<pkg>`) after adding the
  first file of a new language.
- **The cppcheck environment variable.** On Ubuntu 24.04, cppcheck only runs if the
  environment variable `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` is set (to any value); otherwise
  it is skipped while CTest still prints `Passed`. The full story is
  [bug 2](#the-bugs-verification-found), and exercise 5 lets you see it yourself.
- **Include style.** ROS, third-party and project headers use double quotes
  (`#include "rclcpp/rclcpp.hpp"`); angle brackets are only for the C/C++ standard library.
  With angle brackets, clang-format sorts a ROS header into the standard-library block and
  cpplint's `build/include_order` rule rejects the result.

`colcon test` itself exits with 0 even when tests fail; it only runs them.
`colcon test-result` reads the result files and exits non-zero if any test failed, so every
command in this repository is `colcon test && colcon test-result --verbose`.

On today's package `colcon test-result` reports 5 tests although CTest knows only three;
exercise 2 shows why.

**Why industry does it.** Style arguments are settled by a machine, reviewers spend their time
on logic, and simple bugs (an off-by-one array access) are caught before review. Running
linters as tests means they run wherever tests run, including CI, with no extra setup.

### 4. pre-commit

**What it is.** [pre-commit](https://pre-commit.com/) is a framework for git hooks. A hook is
a script git runs at a certain moment; the *pre-commit* hook runs before a commit is created
and can block it. The framework installs each hook tool in its own isolated environment, at
the exact version written in the configuration.

**How it works here.** `.pre-commit-config.yaml` lists the hooks:

- **File hygiene** from `pre-commit-hooks` v6.0.0: size limits (1 MB per file, 3 MB for
  `docs/media/`, both with `--enforce-all` so they also check files already committed), case
  conflicts, shebang/executable consistency, JSON/XML/YAML syntax, merge-conflict markers,
  private keys, final newline, LF line endings, trailing whitespace.
- **clang-format** 18.1.3, restricted to C and C++ (`types_or: [c++, c]`); without that line
  the hook also tries to format JSON and fails on `devcontainer.json`.
- **flake8** 7.0.0 with the plugins `ament_flake8` uses (builtins, comprehensions,
  docstrings, import-order, quotes), configured by `.flake8`, which is a copy of
  `ament_flake8`'s configuration.
- **yamllint** v1.38.0 with `.yamllint`, which allows `True`/`False` (common in ROS
  parameter files) and the `on:` key (GitHub Actions) but rejects `yes`/`no`.

`pre-commit install` writes `.git/hooks/pre-commit`; from then on every `git commit` runs the
hooks on the staged files. A hook that modifies files (clang-format, end-of-file fixer) fails
the commit and leaves the fixed file in your working tree; you review it, `git add` it again,
and commit.

**Pinning for parity with ament.** The point is that pre-commit and `colcon test` must give the
same answer. `colcon test` uses the clang-format and flake8 that Ubuntu 24.04 ships with ROS 2
Jazzy, so the hooks pin exactly those versions: clang-format 18.1.3, flake8 7.0.0, each
flake8 plugin, pydocstyle 6.3.0, and even setuptools 68.1.2 (the reason is
[bug 1](#the-bugs-verification-found)). `.clang-format` is a copy of the file bundled with
`ament_clang_format`, and verification showed both tools produce byte-identical output on a
realistic ROS 2 node. The other hooks (pre-commit-hooks, yamllint) are not tied to ROS and are
updated by hand with `pre-commit autoupdate --repo <url>`; a plain `pre-commit autoupdate`
would also bump the pinned ones.

pre-commit is the **fast subset**: it does not run `copyright`, `cpplint`, `cppcheck`,
`lint_cmake` or the `package.xml` schema check (`check-xml` only checks that XML is
well-formed). Those run in `colcon test`. See
[ADR-003](../decisions/ADR-003-formatting-and-linting.md).

**Why industry does it.** Feedback in seconds, before a commit even exists, is cheaper than a
failed CI run ten minutes later. Pinned versions mean a check that passes today passes
tomorrow on another machine; unpinned linters change their rules under you.

### 5. CI with GitHub Actions

**What it is.** Continuous integration (CI) runs the build and tests on a clean machine for
every change. GitHub Actions runs *workflows* (YAML files in `.github/workflows/`) made of
*jobs*, each made of *steps*, on GitHub-hosted *runners*.

**How it works here.** Read `.github/workflows/ci.yml` alongside this list:

- **Triggers (`on:`):** every pull request, every push to `main`, and manual runs
  (`workflow_dispatch`).
- **`permissions: contents: read`:** the workflow's token can only read the repository. Both
  checkouts also set `persist-credentials: false`, so the token is not left in `.git/config`
  where a later step could use it.
- **`concurrency:`** groups runs. A pull request's runs share one group, and a new push
  cancels the superseded run (`cancel-in-progress` is true only for pull requests). Each
  push to `main` gets its own group (its commit SHA). That detail was a bug fix: with one
  shared group, GitHub replaces a *waiting* run with a newer one even when
  `cancel-in-progress` is false, which would leave some commits on `main` without a result.
- **Job `Lint (pre-commit)`:** on `ubuntu-24.04`, sets up Python 3.12, restores the
  pre-commit cache (key: OS, Python build and a hash of the config), installs
  `pre-commit==3.6.2` (the version Ubuntu ships, so CI matches the dev container), and runs
  `pre-commit run --all-files --show-diff-on-failure`. Timeout: 10 minutes.
- **Job `Build & test (ROS 2 Jazzy)`:** runs inside the container image
  `ros:jazzy-ros-base`. The job environment sets `DEBIAN_FRONTEND=noninteractive` (apt never
  waits for input) and `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1`. `shell: bash` makes every
  step run with `bash --noprofile --norc -eo pipefail`, which guarantees `source` and stops
  at the first failing command, even inside a pipe. The steps check the repository out into
  `src/ros2-warehouse-amr` (making the job's working directory a colcon workspace) and then
  run (since ADR-006 the checkout itself is the workspace, so it goes to the job's root):

  ```bash
  vcs import --input dependencies.repos src
  apt-get update
  rosdep update --rosdistro jazzy
  rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
  source /opt/ros/jazzy/setup.bash
  colcon build --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  colcon test --event-handlers console_cohesion+
  colcon test-result --verbose
  ```

  `rosdep update` is not optional here: GitHub sets `HOME=/github/home` in container jobs,
  where no rosdep cache exists, so `rosdep install` would fail with "your rosdep installation
  has not been initialized yet". `console_cohesion+` prints each package's output in one
  block instead of interleaved. Timeout: 30 minutes (one verification run saw an apt mirror
  stall for over five minutes; GitHub's default timeout is six hours).
- **Artifacts:** the last step uploads `log/` and `build/*/test_results/` as the artifact
  `colcon-logs` whenever the run was not cancelled (`if: ${{ !cancelled() }}`), so you can
  download the test reports of failed *and* green runs.
- **Required checks:** the job names are the required status checks of the `main` ruleset.
  Rename a job and the ruleset would wait forever for a check that no longer exists, so the
  two must change together (a comment in `ci.yml` says so).

[ADR-004](../decisions/ADR-004-ci-pipeline.md) explains why these are plain commands instead
of the popular `ros-tooling/action-ros-ci`: when CI fails, you can paste the same commands
into a local `ros:jazzy-ros-base` container and see the same failure (exercise 6).

**Why industry does it.** "Works on my machine" is not evidence. A clean, scripted build on
every change catches missing dependencies, forgotten files and broken tests before they reach
`main`, and the required checks make that rule impossible to skip.

### 6. Dev containers

**What it is.** A [dev container](https://containers.dev/) is a Docker container configured
for development; VS Code's Dev Containers extension builds it, mounts your repository into it
and runs the editor's tools inside it.

**How it works here.**

- **Image.** `.devcontainer/Dockerfile` starts from `osrf/ros:jazzy-desktop-full` (ROS 2
  Jazzy with Gazebo Harmonic, about 6.5 GB) and adds clang-format, pre-commit, gdb and the
  `ament_clang_format` packages. `DEBIAN_FRONTEND` is a build-time `ARG`, not an `ENV`, so
  apt can still ask questions in your terminal later.
- **Users and UIDs.** Files in a bind mount keep their numeric owner. If the container user
  has a different UID from your host user, files created in the container belong to someone
  else on the host, and git refuses to work with "detected dubious ownership". The image
  therefore deletes Ubuntu's default `ubuntu` user (UID 1000), creates `ros` with UID/GID
  1000, and `updateRemoteUserUID: true` lets VS Code change that UID to yours if it differs.
  The workspace directory `~/ws/src` is created by `ros` in the image; otherwise Docker would
  create it as root for the bind mount and `ros2 pkg create` would fail there.
- **Workspace.** The repository was mounted at `/home/ros/ws/src/ros2-warehouse-amr` (since
  ADR-006 the whole repository, which is the workspace, is mounted at `/home/ros/ws`), and VS
  Code opens `/home/ros/ws`, the colcon workspace. `post-create.sh` runs once: `vcs import`,
  `rosdep install` (the same commands as CI) and `pre-commit install`.
- **GPU.** `--device=/dev/dri` passes the Intel/AMD GPU render node through, and
  `--group-add=${localEnv:RENDER_GID:992}` adds the user to the host's `render` group so it
  may open it. If the group ID is wrong, OpenGL silently falls back to software rendering
  (llvmpipe); the container prints a warning at start in that case.
- **X11 GUI.** Gazebo and RViz draw windows through X11 (XWayland on a Wayland desktop). The
  X11 socket directory `/tmp/.X11-unix` is mounted, `DISPLAY` is passed through, and before the
  container starts VS Code runs `xhost +SI:localuser:$(id -un)` on the host
  (`initializeCommand`). That grants display access to processes running with your UID only,
  which includes the container user. The common advice `xhost +local:` grants it to every
  local user and every container.
- **ROS networking.** `--network=host` and `--ipc=host` share the host's network and IPC
  namespaces, so ROS 2 discovery and Fast DDS's shared-memory transport work between host
  and container. `ROS_DOMAIN_ID=42` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` keep the
  traffic on this machine and away from other ROS systems on the LAN.
- **Debugging.** `--cap-add=SYS_PTRACE` lets `sudo gdb -p <pid>` attach to a running node.
- **Editor parity.** The VS Code settings select the system clang-format 18 and the system
  flake8 with the ament plugins, run from the repository folder. Without them, the C++
  extension uses its own newer bundled clang-format and the flake8 extension its own flake8
  without the ament plugins, and the editor disagrees with CI.
- **Headless variant.** `.devcontainer/headless/devcontainer.json` uses the same image without
  `/dev/dri`, X11 or the `xhost` step. The default configuration cannot even start on a host
  without `/dev/dri` or `/tmp/.X11-unix` (Docker refuses to create the container), so macOS,
  Windows, servers and Codespaces use the headless one.

**Security trade-offs** (listed in the README, because a public repository should not hide
them):

- Host networking: software in the container can listen on your network interfaces; bind web
  servers to `127.0.0.1`.
- Host IPC: it can read shared memory in `/dev/shm`.
- X11: it can see other windows and keystrokes.
- `SYS_PTRACE` and passwordless `sudo`: install only trusted dependencies.

**Why industry does it.** A new developer gets a working environment in minutes instead of a
day, and "it works on my machine" becomes "it works in the container", which is the same
toolchain CI uses.

### 7. Architecture decision records

**What it is.** An ADR is a short document for one decision: the context, the decision, the
alternatives that were rejected and why, and the consequences.

**How it works here.** [docs/decisions/](../decisions/) holds a template (ADR-000) and four
decisions: record decisions as ADRs (001), repository layout (002), formatting and linting
(003), CI pipeline (004). ADRs are numbered, never deleted, and a changed decision gets a new
ADR that supersedes the old one. ADR-003 was corrected during verification when a stated fact
turned out to be false (see [What happened](#what-happened)): an ADR is only useful if its
reasons are true.

**Why industry does it.** Six months later, "why don't we use action-ros-ci?" has a
one-minute answer instead of an archaeology session through old chat logs, and a new team
member can disagree with the *reasoning* instead of guessing it.

### 8. Git workflow

**What it is.** The rules for how changes reach `main`.

**How it works here** (see [CONTRIBUTING.md](../../CONTRIBUTING.md)):

- **Conventional Commits.** Messages look like `type(scope): summary`, for example
  `build(warehouse_bringup): add bringup package with ament lint setup`. Types: `feat`,
  `fix`, `docs`, `test`, `refactor`, `perf`, `build`, `ci`, `chore`; the scope is the package.
  The five commits on `main` show the pattern, one per area: `chore` (licence, readme,
  config), `build(warehouse_bringup)`, `ci`, `build` (dev container), `docs`.
- **Branches.** One branch per phase or feature (`phase-1-robot-description`,
  `feat/<name>`), merged into `main` with a pull request that uses the template.
- **Ruleset.** The "Protect main" ruleset requires a pull request and both CI checks, and
  blocks force-pushes and deletion of `main`. It requires zero approvals because there is one
  maintainer. The five bootstrap commits were pushed straight to `main` before the ruleset
  was switched on; from now on every change goes through a pull request.
- **Tags.** Each finished phase gets a marker tag `phase-N` (`phase-0`, `phase-4a`, ...).
  Releases are tagged only at milestones (`v1.0.0`, `v2.0.0`, `v3.0.0`) with a GitHub Release.
  Package versions stay `0.1.0` until `v1.0.0`.

**Why industry does it.** Structured commit messages make history searchable and allow
generated changelogs. Protected branches and required checks turn "please run the tests"
from a request into a rule. Separate phase markers and release versions avoid versions that go
backwards (see the tag story below).

## What happened

### Timeline

- **2026-11-02.** Project chosen and plan v1 written. Tooling decided and recorded as
  ADR-001 to ADR-004.
- **2026-11-02 to 2026-11-03.** All Phase 0 files written. The maintainer answered the plan's
  open questions: our own Flask operator UI instead of Foxglove, ArUco first and learned
  perception later (optional), simulation only, a custom robot refined over phases.
- **2026-11-03, verification round 1.** Four AI reviewers worked in parallel, each in Docker on a
  scratch copy so the real repository stayed untouched: one reproduced the CI build job
  exactly as GitHub runs it, one tested every pre-commit gate with deliberately bad files,
  one built and ran the dev container, one reviewed every file statically. Together they
  reported about forty findings. Several were found independently by more than one reviewer (the
  cppcheck skip by three), which is a strong signal that a finding is real.
- **2026-11-03, fix round.** All findings were triaged and fixed; the repository was renamed
  from `warehouse_amr` to `ros2-warehouse-amr`.
- **2026-11-03, verification round 2.** The same kinds of checks ran again on the fixed
  repository; CI reproduction, pre-commit gates and dev container found no problems. A final
  cross-file review found one medium and a few low-severity issues, which were fixed in turn.
  A last pass found three more low issues; two were fixed, and the stale names in the v1 plan
  were left for plan v2.
- **2026-11-03.** Repository published on GitHub. The first CI run was green (lint 35 s,
  build & test 42 s). The "Protect main" ruleset and Dependabot were switched on.
- **2026-11-04.** A git repository that had been created by accident at the workspace root
  (from VS Code; it recorded the project repository as an embedded repository) was moved
  aside. The workspace itself is not version-controlled (ADR-002).

### The bugs verification found

Each story is told as symptom → root cause → fix → lesson.

**1. flake8 crashed on every Python file, but CI was green.**
- *Symptom:* in a scratch copy, committing any `.py` file, even a perfectly clean one, failed
  with `FailedToLoadPlugin: Flake8 failed to load plugin "flake8-import-order" due to No
  module named 'pkg_resources'`. On the real repository the hook said `(no files to check)
  Skipped`, because there were no Python files yet.
- *Root cause:* the flake8 hook listed `setuptools` without a version. pip installed the
  newest one (84.2.1 at the time), but setuptools removed the `pkg_resources` module in
  version 82.0.0 (released 2026-02-08), and flake8-import-order 0.18.2 imports it.
- *Fix:* pin `setuptools==68.1.2`, the version Ubuntu 24.04 ships, with a comment explaining
  why (`.pre-commit-config.yaml`), and record it in ADR-003 so nobody "tidies up" the pin.
- *Lesson:* **unpinned dependencies drift.** The configuration had worked when written; the
  world changed underneath it. Pin what your tools depend on, and test a gate with a real
  input before trusting it. This would have broken every commit and every CI run on the day
  the first launch file arrived.

**2. cppcheck reported "Passed" without running.**
- *Symptom:* a C++ file with `int a[3]; return a[3];` (reading past the end of an array)
  passed CI. CTest printed `cppcheck ... Passed`.
- *Root cause:* `ament_cppcheck` skips cppcheck 2.x unless `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS`
  is set, and Ubuntu 24.04 ships 2.13. The skip is reported in the xUnit file, which only
  `colcon test-result` reads; CTest only sees the exit code 0.
- *Fix:* set the variable in `ci.yml`, in both dev container configurations and in the native
  setup in `CONTRIBUTING.md`; document the trap in ADR-003. Re-verified: the planted bug now
  fails with `(error: arrayIndexOutOfBounds)`.
- *Lesson:* **a passing test can be a skipped test.** Always read the `skipped` count, and
  prove a gate works by feeding it something it must reject.

**3. The file-size limit never ran in CI.**
- *Symptom:* a 1.5 MB file committed with `git commit --no-verify` passed the CI lint job.
- *Root cause:* `check-added-large-files` only checks files that are *staged as added*. In
  CI's fresh checkout nothing is staged, so the hook checked nothing.
- *Fix:* `--enforce-all`, which checks every file. A second entry allows up to 3 MB for demo
  GIFs in `docs/media/`.
- *Lesson:* a hook designed for the commit moment can mean something different in CI.
  Understand what input each check actually sees.

**4. The licence check for Python packages is skipped by the package template.**
- *Symptom (found in round 2):* a package generated with
  `ros2 pkg create --build-type ament_python` reported `1 skipped` in `colcon test-result`.
- *Root cause:* the generated `test/test_copyright.py` carries
  `@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')`,
  even when you pass `--license Apache-2.0`.
- *Fix:* `CONTRIBUTING.md` now says to delete that line in every new Python package and to
  check for `0 skipped` after the first `colcon test`.
- *Lesson:* the same class of bug as the cppcheck skip. Templates and defaults are written for
  the general case, not for your rules.

**5. A missing formatter would have been skipped silently.**
- *Symptom:* on a machine without `ros-jazzy-ament-cmake-clang-format`, the package
  configured fine and simply had no `clang_format` test.
- *Root cause:* `ament_lint_auto_find_test_dependencies()` uses `find_package(... QUIET)`.
- *Fix:* `find_package(ament_cmake_clang_format REQUIRED)` in `CMakeLists.txt`. Now the
  configure step fails with `Could not find a package configuration file provided by
  "ament_cmake_clang_format"`.
- *Lesson:* prefer loud failure over silent degradation.

**6. YAML booleans were "checked" but never enforced.**
- *Symptom:* `use_sim_time: yes` was accepted.
- *Root cause:* yamllint's `truthy` rule defaults to `level: warning`, and pre-commit hides
  the output of hooks that pass.
- *Fix:* `level: error` in `.yamllint`. ROS parameter files should use `true`/`false` (or
  `True`/`False`); `yes`/`no` are booleans in YAML 1.1 but strings in YAML 1.2, so different
  parsers can disagree about them.
- *Lesson:* a warning nobody sees is the same as no rule.

**7. The README quick start did not work in a clean shell.**
- *Symptom:* in a clean environment, `rosdep install` failed with "ROS distro is not set" and
  colcon could not find `ament_cmake`.
- *Root cause:* the instructions assumed ROS was already sourced, which is true on the
  author's machine (it is in `~/.bashrc`) and false for everyone else.
- *Fix:* the quick start now begins with `source /opt/ros/jazzy/setup.bash` and runs
  `rosdep update`, with a note about the one-time `sudo rosdep init`.
- *Lesson:* test instructions in a clean environment (`env -i`, a container), not in your own
  shell.

**8. Dev container: `src/` owned by root.**
- *Symptom:* inside the container, `vcs import` and `ros2 pkg create` failed with "Permission
  denied" in `~/ws/src`.
- *Root cause:* Docker creates missing parent directories of a bind mount as root.
- *Fix:* the Dockerfile creates `~/ws/src` as the `ros` user before the mount happens.
- *Lesson:* in containers, always ask "who owns this file?".

**9. Dev container: GUI blocked, and the documented fix was insecure.**
- *Symptom:* Gazebo/RViz could not open a window ("Authorization required"). The README told
  users to run `xhost +local:docker`.
- *Root cause:* the X server only accepts authorized clients. `+local:docker` does not mean
  "docker"; the `docker` part is ignored, and it admits every local process of every user.
- *Fix:* `initializeCommand` runs `xhost +SI:localuser:$(id -un)`, which admits only your UID
  (the container user has the same UID). Verified on a private X server: the user with your
  UID (1000) connected, and another local user (UID 1001) was refused.
- *Lesson:* copy-pasted setup commands can carry security costs; check what they grant.

**10. Dev container: the editor disagreed with CI.**
- *Symptom:* the flake8 extension flagged lines of 80-99 characters as too long (`E501`),
  which `ament_flake8` allows; the C++ extension would format with its own, much newer
  bundled clang-format (23.1.1 in the extension version tested), not 18.1.3.
- *Root cause:* both extensions use their own bundled tools by default, and the flake8
  extension ran from the workspace root, where it does not find the repository's `.flake8`.
- *Fix:* VS Code settings in `devcontainer.json` select `/usr/bin/clang-format`, flake8
  "from environment", and the repository as flake8's working directory.
- *Lesson:* there are three places code is checked (editor, pre-commit, CI); all three need
  the same tool versions and configuration.

**11. Dev container: could not start outside a Linux desktop; GPU failures were silent.**
- *Symptom:* on a host without `/dev/dri` or `/tmp/.X11-unix`, Docker refused to create the
  container (exit codes 127 and 125). With a wrong render group ID, rendering silently fell
  back to software, and every new terminal printed `groups: cannot find name for group ID
  992`.
- *Fix:* a headless configuration; a start-up warning when the render node is not writable;
  a named `render` group (GID 992) in the image; documentation for setting `RENDER_GID` in
  `~/.profile` (VS Code started from the desktop does not read `~/.bashrc`).
- *Lesson:* a public repository is opened on machines you have never seen; provide a path that
  works everywhere.

**12. CI worked on the happy path only.**
- *Symptom:* the first `ci.yml` passed, but a review asked "what if?": two quick pushes to
  `main` could leave one commit without a result, a stalled apt mirror could hang a job for
  six hours, and green runs left no test reports to inspect.
- *Root cause:* defaults: one shared concurrency group, no job timeouts, the checkout token
  kept in `.git/config`, a cache key without the Python build, reports uploaded only on
  failure, and `dependencies.repos` never imported.
- *Fix:* per-commit concurrency groups for `main`, 10/30-minute timeouts,
  `persist-credentials: false`, `env.pythonLocation` in the cache key, upload unless
  cancelled, an "Import third-party sources" step (all explained in
  [concept 5](#5-ci-with-github-actions)).
- *Lesson:* CI is code; review it with the same care, especially for security (token scope)
  and for what happens under load (concurrency).

**13. An ADR stated a false fact.**
- *Symptom:* ADR-003 said Jazzy's clang-format lint hook "has no config-file option".
- *Root cause:* reading `ament_clang_format.cmake` showed that it honours
  `ament_cmake_clang_format_CONFIG_FILE`. The decision (use the bundled ROS 2 style) still
  held, but one of its stated reasons was wrong.
- *Fix:* ADR-003 corrected.
- *Lesson:* documentation is checked against the source, like code.

**14. The tag scheme would have made versions go backwards.**
- *Symptom:* a first scheme tagged phases as `v0.N.0` and milestones as `v1.0.0` to
  `v3.0.0`. Phase 7 comes after `v1.0.0` (end of Phase 6) but would have been tagged `v0.7.0`.
- *Fix:* phase markers are plain `phase-N` tags; only milestones get version numbers.
- *Lesson:* version numbers must only increase; markers and versions are different things.

### What was not verified

- `devcontainer up` (Dev Containers CLI) was not run; the same `docker run` arguments,
  `devcontainer build` and `read-configuration` were.
- A real X11 window was shown only in round 1 (with an X cookie); the `xhost` step was tested
  on a private X server, not a live desktop.
- GitHub Codespaces is untested (it may ignore `workspaceMount`, so `post-create.sh` would
  not be found).
- The base image `osrf/ros:jazzy-desktop-full` is a floating tag; pinning it by digest was
  left for later.

## Try it yourself

Each exercise lists the commands and what you should see. `~/warehouse_ws` is both the
workspace and the repository (ADR-006); the packages are in `~/warehouse_ws/src/`.

**Before you start:**

- Ubuntu 24.04 with ROS 2 Jazzy installed, and `sudo rosdep init` run once (see the README
  quick start). Source ROS in every new terminal: `source /opt/ros/jazzy/setup.bash`.
- pre-commit for exercises 3-4: `sudo apt install pre-commit` (3.6.2, the version CI uses).
- Docker for exercise 6.
- The GitHub CLI for exercise 7: `gh` installed and logged in with `gh auth login`.
- VS Code with the Dev Containers extension for exercise 9.
- A clean working tree (`git status` shows nothing to commit) for exercises 3-5. They create
  throwaway files; the clean-up steps are included. Never commit them.

### Exercise 1: build and test the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/warehouse_ws
vcs import --input dependencies.repos src
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

`vcs` may print a `pkg_resources` `DeprecationWarning`; it is harmless, so ignore it (it is not
bug 1). The build and the test run each end with a summary, and `colcon test-result` prints
the totals (times vary):

```text
Summary: 1 package finished [1.4s]
Summary: 1 package finished [6.6s]
Summary: 5 tests, 0 errors, 0 failures, 0 skipped
```

If `colcon build` fails with `Could not find a package configuration file provided by
"ament_cmake_clang_format"`, the `rosdep install` step was skipped: that is the loud failure
from bug 5, working as intended.

### Exercise 2: see which tests exist and read the results

```bash
cd ~/warehouse_ws/build/warehouse_bringup && ctest -N
cd ~/warehouse_ws && colcon test-result --all
```

`ctest -N` lists `copyright`, `lint_cmake` and `xmllint` (`Total Tests: 3`). `--all` lists every
result file: `Test.xml` with 3 tests, `copyright.xunit.xml` with 0 tests (no source files to
check yet), `lint_cmake` and `xmllint` with 1 each. colcon counts every result file, so
3 + 0 + 1 + 1 gives the "5 tests" of exercise 1. The number is cosmetic; the `failures` and
`skipped` columns are what matter.

### Exercise 3: watch pre-commit block a badly formatted C++ file

```bash
cd ~/warehouse_ws
pre-commit install
git switch -c try/pre-commit
mkdir -p warehouse_bringup/src
printf '#include <string>\nnamespace demo{\nint add(int a,int b){return a+b;}\n}\n' \
  > warehouse_bringup/src/bad_format.cpp
git add warehouse_bringup/src/bad_format.cpp
git commit -m "test: try pre-commit"
git diff -- warehouse_bringup/src/bad_format.cpp
```

The exercise works on a throwaway branch, so `main` is never at risk. You should see
`clang-format....Failed`, `- files were modified by this hook`, and no new commit (`git log -1`
still shows the previous one). `git diff` shows the fix: the brace moved to its own line,
spaces added (`int add(int a, int b) { return a + b; }`) and a `}  // namespace demo` comment.
Clean up:

```bash
git restore --staged warehouse_bringup/src/bad_format.cpp
rm -r warehouse_bringup/src
git switch -
git branch -D try/pre-commit
```

### Exercise 4: flake8 and yamllint, the same rules as ament

```bash
cd ~/warehouse_ws
mkdir -p warehouse_bringup/launch warehouse_bringup/config
printf 'import sys\nimport os\n\nprint("hello", os.sep, sys.argv)\n' \
  > warehouse_bringup/launch/try.launch.py
printf 'demo_node:\n  ros__parameters:\n    use_sim_time: yes\n' \
  > warehouse_bringup/config/try.yaml
pre-commit run --files warehouse_bringup/launch/try.launch.py warehouse_bringup/config/try.yaml
```

You should see flake8 fail with `I100 Import statements are in the wrong order` and
`Q000 Double quotes found but single quotes preferred`, and yamllint fail with
`error truthy value should be one of [False, True, false, true] (truthy)`. These are the
codes `ament_flake8` reports in `colcon test`. Fix the files (swap the imports, use single
quotes, write `true`) and run the same command again to see both pass. Clean up:

```bash
rm -r warehouse_bringup/launch warehouse_bringup/config
```

### Exercise 5: a passing test can be a skipped test

Plant a real bug (reading past the end of an array) and run cppcheck with and without the
environment variable. `ament_copyright --add-missing` adds the Apache-2.0 header, so the
`copyright` test passes and only the planted bug fails.

```bash
cd ~/warehouse_ws
mkdir -p src/warehouse_bringup/src
cat > src/warehouse_bringup/src/oob.cpp <<'CPP'
int sum_readings()
{
  int readings[3] = {1, 2, 3};
  int total = 0;
  for (int i = 0; i <= 3; ++i) {
    total += readings[i];
  }
  return total;
}
CPP
ament_copyright --add-missing "Your Name" apache2 src/warehouse_bringup/src/oob.cpp
colcon build --symlink-install --packages-select warehouse_bringup --cmake-force-configure
(unset AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS; colcon test --packages-select warehouse_bringup)
colcon test-result --all | grep cppcheck
```

The file is never compiled (no target uses it), but the linters scan every source file in
the package. `--cmake-force-configure` makes CMake register the C++ linters (see "Tests are
registered at configure time" in concept 3). Without the variable, the test run reports no
failures, and the last command prints:

```text
build/warehouse_bringup/test_results/warehouse_bringup/cppcheck.xunit.xml: 1 test, 0 errors, 0 failures, 1 skipped
```

Now with the variable, and look at the exit codes:

```bash
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select warehouse_bringup
echo $?
colcon test-result --verbose
echo $?
```

`colcon test` prints `1 package had test failures: warehouse_bringup` and yet `echo $?` prints
`0`: it only runs the tests. `colcon test-result` shows the bug and exits with `1`, which is
why every command in this repository chains `colcon test && colcon test-result --verbose`:

```text
- warehouse_bringup.cppcheck error: arrayIndexOutOfBounds (src/oob.cpp:20)
```

The other new tests (`clang_format`, `cpplint`, `copyright`) pass. Clean up, including the
package's build folder: old result files such as `cppcheck.xunit.xml` would otherwise stay
there and keep showing up in `colcon test-result`.

```bash
rm -r src/warehouse_bringup/src
rm -r build/warehouse_bringup
colcon build --symlink-install --packages-select warehouse_bringup
```

### Exercise 6: reproduce the CI job in Docker

From the repository root, start the same image CI uses, with the repository mounted where CI
checks it out:

```bash
cd ~/warehouse_ws
docker run --rm -it -v "$PWD":/ws -w /ws ros:jazzy-ros-base bash
```

Inside the container, run the job's environment and `run:` commands from `ci.yml`:

```bash
export DEBIAN_FRONTEND=noninteractive AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1
vcs import --input dependencies.repos src
apt-get update
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
source /opt/ros/jazzy/setup.bash
colcon build --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

`rosdep install` installs `ros-jazzy-ament-cmake-clang-format` and clang-format 18, and the
run ends with `Summary: 5 tests, 0 errors, 0 failures, 0 skipped`, the same as CI. To see why
CI needs `rosdep update`, simulate GitHub's empty home directory:

```bash
mkdir -p /tmp/gh-home
HOME=/tmp/gh-home rosdep install --from-paths src --ignore-src --rosdistro jazzy -y --simulate
```

It prints `ERROR: your rosdep installation has not been initialized yet.` Leave with `exit`;
`--rm` deletes the container. The container runs as root, and `build/`, `install/` and
`log/` stay inside it (only the repository is mounted). Do not create files in the mounted
repository from inside it: they would be owned by root on your host.

### Exercise 7: CI and branch protection on GitHub

```bash
cd ~/warehouse_ws
gh run list --workflow ci.yml --limit 5
```

Each row is one CI run. The columns are `STATUS` (a tick for success), `TITLE` (the commit
message), `WORKFLOW`, `BRANCH`, `EVENT` (`push` or `pull_request`), `ID`, `ELAPSED` and `AGE`.
Copy a run's `ID` and use it in place of `<run-id>`:

```bash
gh run view <run-id>
gh run download <run-id> -n colcon-logs -D /tmp/colcon-logs-ci
gh api repos/TANAY779/ros2-warehouse-amr/rules/branches/main --jq '.[].type'
```

`gh run view` shows the two jobs, `Build & test (ROS 2 Jazzy)` and `Lint (pre-commit)`, with
their durations, and the artifact `colcon-logs`. The download contains `log/` and
`build/warehouse_bringup/test_results/`, the same files you read in exercise 2. The last
command prints `deletion`, `non_fast_forward`, `pull_request` and `required_status_checks`:
the four rules of the "Protect main" ruleset. (Artifacts expire after GitHub's retention
period; pick a recent run.)

### Exercise 8: one rule, enforced in many places

```bash
cd ~/warehouse_ws
git grep -l AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS -- ':!docs/phase-reports' ':!docs/PROJECT_STATUS.md'
```

The `':!...'` patterns leave out the documents that only explain the variable. You should see
six files: both `devcontainer.json` files, `ci.yml`, `CONTRIBUTING.md`, ADR-003 and ADR-004.
One fact, six places: that is why it is written down in an ADR, and why the reviewers checked
every place when it was fixed. Try the same with `setuptools==68.1.2` and with a CI job name
such as `Lint (pre-commit)`.

### Exercise 9 (optional): inside the dev container

Open the repository in VS Code, run **Dev Containers: Reopen in Container**, pick a
configuration (the first build downloads about 6.5 GB), and in its terminal run:

```bash
id
echo "$ROS_DOMAIN_ID $ROS_AUTOMATIC_DISCOVERY_RANGE $AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS"
stat -c '%U' ~/ws/src
clang-format --version
```

You should see the user `ros` (with `992(render)` among the groups in the default
configuration), `42 LOCALHOST 1`, `ros` as the owner of `src`, and clang-format 18.1.3, the
same version CI uses.

## Pitfalls and lessons

The bug stories above each end with their lesson. These are the everyday pitfalls they do not
cover:

- **`colcon test` does not fail on test failures; `colcon test-result` does.** Always chain
  them (exercise 5).
- **Source ROS in every new shell** (`source /opt/ros/jazzy/setup.bash`, then the workspace's
  `install/setup.bash` after a build).
- **Lint tests are registered when CMake configures.** After adding the first file of a new
  language to a package, run `colcon build --cmake-force-configure`.
- **`colcon test` needs network access** for the `package.xml` schema check.
- **Mind file ownership around Docker.** A root container writing into a bind mount leaves
  root-owned files on your host.

## Glossary

| Term | Meaning |
|---|---|
| ADR | Architecture decision record: a short document recording one decision and its reasons |
| ament | The ROS 2 build-system layer on top of CMake (`ament_cmake`) and Python setuptools (`ament_python`) |
| ament_lint | ROS 2's linters, run as tests by `colcon test` |
| artifact | A file or folder a CI run uploads for later download |
| bind mount | A host folder made visible inside a container; changes are shared both ways |
| CI | Continuous integration: building and testing every change automatically on a clean machine |
| colcon | The ROS 2 command-line tool that builds and tests all packages in a workspace |
| Conventional Commits | A commit-message format: `type(scope): summary` |
| CTest | CMake's test runner; colcon uses it for `ament_cmake` packages |
| dev container | A Docker container configured as a development environment for an editor |
| Fast DDS | The default ROS 2 middleware implementation (RMW) in Jazzy |
| hook (git) | A script git runs at a certain moment, such as before a commit |
| linter | A tool that checks code for style problems or likely bugs without running it |
| package.xml | A ROS package's manifest: name, version, licence, dependencies |
| pre-commit | A framework that installs and runs pinned git hooks |
| rosdep | Installs the system dependencies declared in `package.xml` files |
| ruleset | GitHub branch rules, such as "pull request and green checks required" |
| runner | The machine that executes a GitHub Actions job |
| UID / GID | Numeric user and group IDs; file ownership is stored as numbers |
| vcs (vcstool) | Clones a list of repositories from a `.repos` file |
| workspace (colcon) | A folder with `src/` that colcon builds into `build/`, `install/`, `log/` |
| X11 / XWayland | The display protocol GUI apps use on Linux; XWayland provides it on Wayland desktops |
| xUnit file | An XML test-result file; `colcon test-result` reads them |

## Further reading

- ROS 2 Jazzy documentation: <https://docs.ros.org/en/jazzy/>
  - Creating a workspace: <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>
  - Creating a package: <https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>
  - Managing dependencies with rosdep: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html>
  - Code style and language versions: <https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html>
  - Testing: <https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html>
- colcon documentation: <https://colcon.readthedocs.io/en/released/>
  - `colcon test-result`: <https://colcon.readthedocs.io/en/released/reference/verb/test-result.html>
- ament_lint source and README: <https://github.com/ament/ament_lint>
- REP 149, package manifest format 3: <https://ros.org/reps/rep-0149.html>
- pre-commit: <https://pre-commit.com/>; list of available hooks: <https://pre-commit.com/hooks.html>
- GitHub Actions documentation: <https://docs.github.com/en/actions> (the deep links below may
  move; search from this page if one breaks)
  - Workflow syntax, including `defaults.run.shell`: <https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax>
  - Running jobs in a container: <https://docs.github.com/en/actions/how-tos/write-workflows/choose-where-workflows-run/run-jobs-in-a-container>
  - Concurrency: <https://docs.github.com/en/actions/how-tos/write-workflows/choose-when-workflows-run/control-workflow-concurrency>
- Rulesets: <https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/about-rulesets>
- Development containers specification: <https://containers.dev/>; VS Code guide:
  <https://code.visualstudio.com/docs/devcontainers/containers>
- Conventional Commits: <https://www.conventionalcommits.org/>
- ADRs, the original article by Michael Nygard:
  <https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions>

## What comes next

Phase 1 builds the robot: a URDF/xacro description of the custom tote-carrying AMR,
differential drive through `ros2_control`, a 2D lidar and an IMU, an EKF that fuses wheel
odometry and the IMU, and the first teleoperated drive in Gazebo Harmonic, launched by
`ros2 launch warehouse_bringup sim.launch.py`. Its report will be
`docs/phase-reports/phase-01-<name>.md`, and the features it adds will appear in
[PROJECT_STATUS.md](../PROJECT_STATUS.md).
