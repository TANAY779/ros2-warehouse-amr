# Architecture decision records

Why the system is built the way it is. New decisions start from
[ADR-000-template.md](ADR-000-template.md); see [ADR-001](ADR-001-record-architecture-decisions.md)
for when to write one.

| ADR | Decision | Status |
|---|---|---|
| [001](ADR-001-record-architecture-decisions.md) | Record architecture decisions | accepted |
| [002](ADR-002-repository-layout.md) | One multi-package repository inside a colcon workspace | accepted |
| [003](ADR-003-formatting-and-linting.md) | Formatting and linting aligned with ROS 2 Jazzy's ament_lint | accepted |
| [004](ADR-004-ci-pipeline.md) | CI runs plain rosdep/colcon steps in the official ROS image | accepted |
| [005](ADR-005-custom-tote-amr.md) | Build our own tote AMR on ros2_control, with an EKF for odometry | accepted |
