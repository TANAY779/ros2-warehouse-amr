# Warehouse AMR

[![CI](https://github.com/TANAY779/ros2-warehouse-amr/actions/workflows/ci.yml/badge.svg)](https://github.com/TANAY779/ros2-warehouse-amr/actions/workflows/ci.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)

A simulated warehouse in which autonomous mobile robots (AMRs) will fulfil orders, recharge
themselves, scan inventory and work as a fleet. It is being built with ROS 2 Jazzy, Nav2 and
Gazebo Harmonic, one phase at a time, and each phase will end with a recorded demo.

> **Status:** Phase 1 (the robot) is done except its demo GIF: one robot drives by keyboard
> teleop, in Gazebo or on mock hardware without a simulator. Phase 2 (the warehouse world,
> mapping and localization) is next.

## Roadmap

| Milestone | Planned deliverables | Status |
|---|---|---|
| M0 Foundation (Phase 0) | CI green on a one-package repo, lint gates, GUI and headless dev containers, protected `main` | done |
| M1 Core AMR (Phases 1-6, v1.0.0) | A custom tote-carrying AMR that fulfils a stream of orders in a generated warehouse, respects keepout and speed zones, docks to recharge on its own and logs KPIs. Operators submit orders and watch the live map in our Flask dashboard | in progress |
| M2 Perception (Phases 7-8, v2.0.0) | Docking on a detected ArUco tag. An inventory patrol that reads tote tags with the side cameras, keeps inventory state and flags discrepancies in RViz and the dashboard | planned |
| M3 Fleet (Phases 9-12, v3.0.0) | 3 identical robots, greedy then auction allocation, aisle traffic control, fleet and traffic views in the dashboard, and the final showcase | planned |
| Optional (Phases 8b, 13) | Learned tote detection trained on Colab (8b, research paused) and mobile manipulation (13, stretch) | planned |

The full phased plan is in [docs/PROJECT_PLAN.md](docs/PROJECT_PLAN.md); design decisions are
recorded in [docs/decisions/](docs/decisions/).

## Documentation

- [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md): current status, every feature that exists
  (what it does, how to use it, where it lives) and a dated log of ideas and decisions.
- [docs/phase-reports/](docs/phase-reports/): one learning report per finished phase,
  explaining what was built, the concepts behind it, what went wrong and how it was fixed, with
  hands-on exercises. Start with
  [Phase 0: repository foundation](docs/phase-reports/phase-00-foundation.md), then
  [Phase 1: the robot](docs/phase-reports/phase-01-robot-description.md).
- [docs/decisions/](docs/decisions/): architecture decision records (ADRs).
- [docs/PROJECT_PLAN.md](docs/PROJECT_PLAN.md): the phased project plan.

## Quick start

Requires Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
(including `ros-dev-tools`), or the dev container below.

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/TANAY779/ros2-warehouse-amr.git ~/warehouse_ws
cd ~/warehouse_ws
vcs import --input dependencies.repos src
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

Tests and CI are paused while the project is under heavy development: `colcon.meta` builds
every package without tests. See [CONTRIBUTING.md](CONTRIBUTING.md#build-and-test) to run them.

First time using rosdep on this machine? Run `sudo rosdep init` once before `rosdep update`.

### Dev container

Open the repository in VS Code and run **Dev Containers: Reopen in Container**. The image
has ROS 2 Jazzy, Gazebo Harmonic and all lint tools; the workspace is at `/home/ros/ws`.

- The default configuration, **ros2-warehouse-amr (ROS 2 Jazzy)**, is for a Linux desktop only:
  it needs `/dev/dri` and an X11 display. On any other host (macOS, Windows, a headless server),
  choose **ros2-warehouse-amr (headless)** when VS Code asks for a configuration. It has the
  same tools but no GUI or GPU access. GitHub Codespaces does not ask (and is not tested yet):
  create the codespace with **Code → Codespaces → ⋯ → New with options** and pick the headless
  configuration under *Dev container configuration*.
- GPU: `/dev/dri` is passed through. If your `render` group ID is not 992, put
  `export RENDER_GID=$(getent group render | cut -d: -f3)` in `~/.profile` and log in again
  (or start VS Code with `code .` from a shell that has it). VS Code started from the dock
  does not see variables set only in `~/.bashrc`. With the wrong ID, rendering falls back to
  software (llvmpipe) without an error; the container prints a warning when it starts.
- GUI apps (Gazebo, RViz) use X11, through XWayland on a Wayland desktop. Before the container
  starts, VS Code runs `xhost +SI:localuser:$(id -un)` on the host (`initializeCommand`), which
  lets only processes running with your UID, including the container user, open windows. Never
  use `xhost +local:`: it lets every local user and every container use your display.
- The container mounts the whole workspace at `/home/ros/ws`, so it shares `build/`, `install/`
  and `log/` with the host. When you switch between building on the host and in the container,
  run `rm -rf build install log` first: CMake refuses a build folder made at another path.
- The container sets `ROS_DOMAIN_ID=42` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Export
  the same values on the host to see the container's topics there.
- Security: the container shares the host network (`--network=host`) and IPC namespace
  (`--ipc=host`), has `CAP_SYS_PTRACE` (so `sudo gdb -p <pid>` can attach to a running node),
  and `ros` has passwordless sudo. Anything you install in it can listen on your network
  interfaces, read shared memory in `/dev/shm` and, through X11, see other windows and
  keystrokes. Install only trusted dependencies, and bind web servers (such as the Flask
  dashboard) to `127.0.0.1`.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full setup and the branch, commit and review
workflow. The pre-commit hook is optional while checks are paused; one-time setup on a
native install:

```bash
sudo apt install pre-commit clang-format
cd ~/warehouse_ws && pre-commit install
```

## Repository layout

The repository root is the colcon workspace: packages live in `src/`, and colcon's `build/`,
`install/` and `log/` (git-ignored) appear next to it.

| Path | Contents |
|---|---|
| `src/warehouse_bringup/` | Top-level launch files (`sim.launch.py`), RViz configuration |
| `src/warehouse_description/` | The robot: `robot.yaml`, xacro model, ros2_control and controller configuration |
| `src/warehouse_gazebo/` | Gazebo worlds, robot spawning, ros_gz bridge configuration |
| `src/warehouse_navigation/` | Localization (the odometry EKF); AMCL and Nav2 in later phases |
| `src/warehouse_system_tests/` | Launch tests of the running system (mock hardware in CI, Gazebo locally) |
| `docs/` | Project status, phase reports, project plan and architecture decision records |
| `.devcontainer/` | Development container (default and headless configurations) |
| `.github/` | CI workflow, Dependabot, PR template |

More packages are added as the phases need them (see the plan).

## License

[Apache-2.0](LICENSE)
