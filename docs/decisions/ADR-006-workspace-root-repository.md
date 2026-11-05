# ADR-006: The repository root is the colcon workspace

- **Status:** accepted (supersedes [ADR-002](ADR-002-repository-layout.md))
- **Date:** 2026-11-05
- **Plan reference:** D9, §3

## Context

ADR-002 put this repository in the `src/` folder of a colcon workspace
(`~/warehouse_ws/src/ros2-warehouse-amr`). Opened at the workspace root, VS Code did not find
the repository, which sits two folders down, and the owner prefers a single repository at the
workspace root that holds everything needed to build the project.

## Decision

The repository root is the colcon workspace. Clone it straight into the workspace folder:

```text
warehouse_ws/                 this repository and the colcon workspace
├── .github/ .devcontainer/   CI and the dev container (must stay at the root)
├── README.md LICENSE ...     GitHub shows them only from the root
├── docs/                     plan, status, phase reports, ADRs
├── build/ install/ log/      colcon output, kept out by .gitignore
└── src/
    └── warehouse_*/          one folder per ROS 2 package
```

- `git clone https://github.com/TANAY779/ros2-warehouse-amr.git ~/warehouse_ws`, then build
  from `~/warehouse_ws` as before.
- Third-party sources still go to `src/` with `vcs import --input dependencies.repos src`.
- The move was one rename commit, so history is kept (`git log --follow` traces a file back).

## Alternatives considered

| Option | Why not |
|---|---|
| Keep ADR-002 (repository in `src/`) | Standard for ROS 2, but VS Code needs `git.repositoryScanMaxDepth: 2` to find it, and the owner wants the repository at the workspace root |
| Repository at the root with every file under `src/` | GitHub shows the README and licence and runs `.github/` only from the root, and VS Code finds `.devcontainer/` only there |

## Consequences

- `.gitignore` must keep `build/`, `install/` and `log/` out; it already did.
- CI checks the repository out at the job's root, which is now the workspace.
- The dev container mounts the whole workspace at `/home/ros/ws`, so it shares `build/`,
  `install/` and `log/` with the host: clean them when switching between host and container
  builds.
- Several workspaces from this repository on one machine are separate clones, as before.
