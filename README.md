# Warehouse AMR

[![CI](https://github.com/TANAY779/ros2-warehouse-amr/actions/workflows/ci.yml/badge.svg)](https://github.com/TANAY779/ros2-warehouse-amr/actions/workflows/ci.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)

A simulated warehouse in which autonomous mobile robots (AMRs) will fulfil orders, recharge
themselves, scan inventory and work as a fleet. It is being built with ROS 2 Jazzy, Nav2 and
Gazebo Harmonic, one phase at a time, and each phase will end with a recorded demo.

> **Status:** Phase 0, repository foundation. Nothing drives yet.

## Roadmap

| Milestone | Planned deliverables | Status |
|---|---|---|
| M0 Foundation | Repository, CI, lint gates, dev container | in progress |
| M1 Core AMR | A custom robot that will fulfil orders in a generated warehouse, respect keep-out and speed zones and dock to recharge, plus an operator dashboard | planned |
| M2 Perception | ArUco shelf and dock tags, an inventory patrol and discrepancy reports | planned |
| M3 Fleet | Three robots, a fleet manager with task allocation, aisle traffic control and a showcase scenario | planned |
| Later | Learned perception (trained on Colab) and mobile manipulation | planned |

The full phased plan is in [docs/PROJECT_PLAN.md](docs/PROJECT_PLAN.md); design decisions are
recorded in [docs/decisions/](docs/decisions/).

## Quick start

Requires Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
(including `ros-dev-tools`), or the dev container below.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/warehouse_ws/src && cd ~/warehouse_ws/src
git clone https://github.com/TANAY779/ros2-warehouse-amr.git
cd ~/warehouse_ws
vcs import --input src/ros2-warehouse-amr/dependencies.repos src
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
colcon test && colcon test-result --verbose
```

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
- The container sets `ROS_DOMAIN_ID=42` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Export
  the same values on the host to see the container's topics there.
- Security: the container shares the host network (`--network=host`) and IPC namespace
  (`--ipc=host`), has `CAP_SYS_PTRACE` (so `sudo gdb -p <pid>` can attach to a running node),
  and `ros` has passwordless sudo. Anything you install in it can listen on your network
  interfaces, read shared memory in `/dev/shm` and, through X11, see other windows and
  keystrokes. Install only trusted dependencies, and bind web bridges (rosbridge,
  foxglove_bridge) to `127.0.0.1`.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full setup and the branch, commit and review
workflow. One-time setup on a native install:

```bash
sudo apt install pre-commit clang-format
cd ~/warehouse_ws/src/ros2-warehouse-amr && pre-commit install
```

## Repository layout

| Path | Contents |
|---|---|
| `warehouse_bringup/` | Top-level launch files and configuration |
| `docs/` | Project plan and architecture decision records |
| `.devcontainer/` | Development container (default and headless configurations) |
| `.github/` | CI workflow, Dependabot, PR template |

More packages are added as the phases need them (see the plan).

## License

[Apache-2.0](LICENSE)
