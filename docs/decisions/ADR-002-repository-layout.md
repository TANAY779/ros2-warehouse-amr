# ADR-002: One multi-package repository inside a colcon workspace

- **Status:** superseded by [ADR-006](ADR-006-workspace-root-repository.md) (2026-11-05)
- **Date:** 2026-11-02
- **Plan reference:** D9

## Context

The system will grow to roughly 15 ROS 2 packages (description, simulation, navigation,
task execution, fleet management, perception, dashboard, ...). They change together: a new
message in `warehouse_interfaces` usually comes with changes in two or three other packages.

## Decision

Keep all packages in one Git repository, cloned into the `src/` folder of a standard colcon
workspace:

```text
warehouse_ws/                 colcon workspace (not version-controlled)
├── build/ install/ log/      colcon output
└── src/
    └── ros2-warehouse-amr/   this repository: one folder per ROS 2 package
```

- Package names share the `warehouse_` prefix.
- Packages are created in the phase that first needs them; no empty scaffolding.
- Third-party sources that have no Jazzy binary go in `dependencies.repos` and are imported
  next to this repository with `vcs import`.

## Alternatives considered

| Option | Why not |
|---|---|
| Repository root = workspace root | Mixes build output with sources; non-standard for ROS 2 tooling and CI |
| One repository per package | Cross-package changes need coordinated PRs and version pinning; too much overhead for one developer |

## Consequences

- One PR can change an interface and all its users atomically.
- CI builds and tests every package on every change; build time will grow and may need
  caching later.
