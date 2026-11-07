# ADR-003: Formatting and linting aligned with ROS 2 Jazzy's ament_lint

- **Status:** superseded by [ADR-008](ADR-008-develop-on-main-without-automated-tests.md) (2026-11-07)
- **Date:** 2026-11-02
- **Plan reference:** D14 (refined: cpplint is kept)

## Context

Code is checked in two places: pre-commit hooks on the developer's machine, and the
`ament_lint` tests that run during `colcon test` (locally and in CI). If the two use
different rules or tool versions, code that passes one fails the other.

Facts checked on Jazzy (2026-11-02 and 2026-11-03):

- `ament_cmake_clang_format`'s lint hook calls `ament_clang_format()` without arguments, so
  unless a package sets `ament_cmake_clang_format_CONFIG_FILE`, `colcon test` uses the style
  file bundled with `ament_clang_format`.
- `ament_lint_common` does not include clang-format (it is commented out); it does include
  uncrustify, which formats C++ differently.
- `ament_copyright` checks `.py`, C/C++ and `.cmake` files, not `CMakeLists.txt`.
- `ament_cppcheck` skips cppcheck 2.x (Ubuntu 24.04 ships 2.13) unless the
  `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` environment variable is set. CTest still reports the
  test as passed; only `colcon test-result` shows it as skipped.

## Decision

- **C++:** clang-format. The repository's `.clang-format` is a copy of the file bundled with
  `ament_clang_format`. Every C++ package adds `ament_cmake_clang_format` as a test
  dependency and excludes `ament_cmake_uncrustify` via `AMENT_LINT_AUTO_EXCLUDE`. The other
  `ament_lint_common` linters (cppcheck, cpplint, copyright, xmllint, lint_cmake, flake8,
  pep257) stay on.
- **cppcheck:** enabled by setting `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` in CI
  (`ci.yml`), in the dev container (`containerEnv`) and in the native setup in
  CONTRIBUTING.md.
- **Python:** flake8 with the same configuration and plugins as `ament_flake8` (`.flake8`).
- **Versions:** pre-commit pins clang-format 18.1.3 and flake8 7.0.0 plus its plugins at the
  versions Ubuntu 24.04 ships, because those are what `ament_lint` runs. That includes
  setuptools 68.1.2: flake8-import-order 0.18.2 imports `pkg_resources`, which setuptools
  removed in 82.0, so an unpinned setuptools makes flake8 fail to load. Upgrade them only
  together with the ROS distribution.
- **YAML:** yamllint with a config tolerant of ROS parameter files (`.yamllint`).
- **XML:** pre-commit's `check-xml` only checks that XML is well-formed; `package.xml` schema
  validation (format 3 XSD) runs in `colcon test` through `ament_xmllint`.
- **Licence headers:** every `.py`, C/C++ and `.cmake` file starts with the Apache-2.0
  header (see CONTRIBUTING.md), which `ament_copyright` checks.
- **clang-tidy:** deferred to Phase 4a, when the first C++ package exists.

## Alternatives considered

| Option | Why not |
|---|---|
| uncrustify (ROS 2 core default) | Weaker editor integration; most C++ developers and IDEs use clang-format |
| A custom `.clang-format` passed to ament | Possible via `ament_cmake_clang_format_CONFIG_FILE`, but every package's CMakeLists.txt must set it and the code would no longer follow the ROS 2 style |
| Latest linter versions in pre-commit | Formatting and warnings would drift from what `colcon test` reports |
| ruff instead of flake8 | Faster, but ament runs flake8; two Python linters would disagree |

## Consequences

- pre-commit is the fast subset of the ament checks: formatting, flake8 and file hygiene run
  on every commit. `ament_copyright`, `ament_cpplint`, `ament_cppcheck`, `ament_lint_cmake`
  and the `package.xml` schema check (`ament_xmllint`) run only in `colcon test`.
- ROS, third-party and project headers are included with double quotes
  (`#include "rclcpp/rclcpp.hpp"`); angle brackets are only for the C/C++ standard library.
  With angle brackets, clang-format sorts a ROS header into the standard-library block and
  cpplint's `build/include_order` check rejects the result.
- cpplint and clang-format agree on a realistic rclcpp node (checked 2026-11-03) as long as
  non-standard headers use double quotes. If they conflict on real code, revisit and exclude
  cpplint with a new ADR.
- For the checks they share (clang-format, flake8), pre-commit and `colcon test` give the
  same result: same tool versions, same configuration. The editor matches them only
  through the dev container's VS Code settings, which select the system clang-format 18 and
  the system flake8 with the ament plugins, run from the repository root. Without those
  settings, the extensions use their own bundled, newer tools.
- Dependabot updates GitHub Actions only. clang-format and flake8 move with the ROS
  distribution; the other pre-commit hooks (pre-commit-hooks, yamllint) are updated by hand
  with `pre-commit autoupdate --repo <url>`.
