# ADR-004: CI runs plain rosdep/colcon steps in the official ROS image

- **Status:** accepted
- **Date:** 2026-11-02
- **Plan reference:** Phase 0 (replaces the planned `ros-tooling/action-ros-ci`)

## Context

Every push and pull request must prove that the workspace builds and its tests pass on a
clean machine. The plan proposed `ros-tooling/setup-ros` + `ros-tooling/action-ros-ci`.

## Decision

GitHub Actions workflow `.github/workflows/ci.yml` with two jobs:

1. **Lint:** `pre-commit run --all-files`, with the same hooks developers run locally.
2. **Build & test:** inside the official `ros:jazzy-ros-base` container, run exactly the
   commands a developer runs: `vcs import` of `dependencies.repos`, `rosdep install`,
   `colcon build`, `colcon test`, `colcon test-result --verbose`. Logs and test results are
   always uploaded, unless the run was cancelled.

The workflow has read-only repository permissions and does not keep the checkout token. It
cancels superseded pull request runs, while every push to `main` runs to completion: each push
gets its own concurrency group (its commit SHA), because GitHub replaces a waiting run with a
newer one from the same group even when `cancel-in-progress` is false.
Dependabot keeps the action versions current.

A separate nightly simulation job (headless Gazebo) is added in Phase 6; it does not block
pull requests.

**Phase 1 addition (2026-11-05).** With the first launch tests, the build-and-test job sets two
environment variables:

- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, like the dev containers: DDS discovery stays on the
  runner. Each launch test file also runs in its own `ROS_DOMAIN_ID`, picked by
  `domain_coordinator` through `ament_cmake_ros`'s `run_test_isolated.py`.
- `WAREHOUSE_SKIP_GZ_TESTS=1`: the runners have no GPU or EGL for Gazebo's rendering sensors,
  so the Gazebo launch tests report every case as skipped. The mock-hardware launch tests
  (`hardware:=mock`, no simulator) run in every pull request.

## Alternatives considered

| Option | Why not |
|---|---|
| `ros-tooling/action-ros-ci` | Works well, but hides the build behind a third-party action; a CI failure is harder to reproduce locally |
| `ros-industrial/industrial_ci` | Powerful but large; more than this project needs |
| Self-built Docker image for CI | Faster later, but another artifact to maintain; revisit if rosdep install time grows |

## Consequences

- A CI failure can be reproduced from the repository root with
  `docker run --rm -it -v "$PWD":/ws/src/ros2-warehouse-amr -w /ws ros:jazzy-ros-base bash`,
  then `export DEBIAN_FRONTEND=noninteractive AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` and the
  `run:` commands from `ci.yml`.
- Steps that `action-ros-ci` provided are added by hand: the `vcs import` of
  `dependencies.repos` exists now; coverage (colcon-lcov-result for C++, coverage.py for
  Python) for the coverage badge, and ccache if builds slow down, come later.
- Logs and test results are uploaded on success too, so the test reports of a green run can
  be inspected.
- Job timeouts (10 minutes for lint, 30 for build & test) stop a stalled apt mirror or a hung
  test from holding a runner for GitHub's 6-hour default.
- rosdep installs dependencies on every run; add caching or a prebuilt image if build times
  grow past a few minutes. From Phase 1 that includes Gazebo, ros2_control and RViz (the
  packages' run dependencies), even though CI skips the Gazebo tests.
- `colcon test-result` reports the Gazebo test cases as skipped in CI. A skipped count above
  zero is expected there; locally it should be zero.
