# Contributing

## Workflow

- **One branch per phase or feature:** `phase-<N>-<short-name>` (e.g. `phase-1-robot-description`)
  or `feat/<short-name>`.
- **Pull request into `main`** using the PR template. CI must be green before merging.
- **Commit messages** follow [Conventional Commits](https://www.conventionalcommits.org/), with
  the package as the scope:

  ```text
  feat(warehouse_navigation): add keep-out filter mask
  fix(warehouse_battery): clamp charge at 100 %
  docs: add ADR-005 for localization
  ```

  Types: `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `build`, `ci`, `chore`.
- **Tags:** each finished phase gets a marker tag `phase-N` (e.g. `phase-3`, `phase-4a`).
  Releases are tagged only at milestones: `v1.0.0` (M1), `v2.0.0` (M2), `v3.0.0` (M3), each
  with a GitHub Release. Before a release tag, set `<version>` in every `package.xml` to the
  release number; between releases packages stay at their last release (0.1.0 before v1.0.0).
- **Design decisions** that are hard to reverse get an ADR in `docs/decisions/`.
- **Media:** demo GIFs and other files in `docs/media/` stay under 3 MB; every other file
  stays under 1 MB (pre-commit enforces both). Longer demos go on a GitHub Release as MP4
  files, linked from the README.

## Setup

Native Ubuntu 24.04 + ROS 2 Jazzy:

```bash
sudo apt install pre-commit clang-format
echo 'export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1' >> ~/.bashrc  # let colcon test run cppcheck
cd ~/warehouse_ws/src/ros2-warehouse-amr
pre-commit install          # run hooks on every commit
```

`ament_cppcheck` skips cppcheck 2.x (Ubuntu 24.04 ships 2.13) unless
`AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` is set. CTest still reports the skipped test as passed;
only `colcon test-result` shows it as skipped. CI and the dev container set the variable, so
set it on a native install too.

The dev container already has these tools and settings and runs `pre-commit install` for you.

## Build and test

From the workspace root (`~/warehouse_ws`):

```bash
source /opt/ros/jazzy/setup.bash
vcs import --input src/ros2-warehouse-amr/dependencies.repos src
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
colcon test && colcon test-result --verbose
(cd src/ros2-warehouse-amr && pre-commit run --all-files)
```

`colcon test` needs network access: the xmllint test downloads the `package.xml` schema from
download.ros.org and fails offline.

To check a single package: `colcon build --packages-select <pkg>` and
`colcon test --packages-select <pkg>`.

## Code style

Enforced by pre-commit and by the `ament_lint` tests in `colcon test`; see
[ADR-003](docs/decisions/ADR-003-formatting-and-linting.md).

- **C++:** ROS 2 style via clang-format 18 and the repository `.clang-format`.
- **Includes:** angle brackets only for the C/C++ standard library; ROS, third-party and
  project headers use double quotes (`#include "rclcpp/rclcpp.hpp"`). clang-format sorts every
  `<...>` include into the standard-library block, which cpplint rejects
  (`build/include_order`).
- **Python:** flake8 with the ament configuration (99-character lines, Google import order,
  single quotes, pep257 style for the docstrings you write; missing docstrings are not
  flagged).
- **Licence header:** every `.py`, C/C++ and `.cmake` file begins with this header, placed
  after the `#!` line if the file has one:

  ```python
  # Copyright 2026 Tanay Baisware
  #
  # Licensed under the Apache License, Version 2.0 (the "License");
  # you may not use this file except in compliance with the License.
  # You may obtain a copy of the License at
  #
  #     http://www.apache.org/licenses/LICENSE-2.0
  #
  # Unless required by applicable law or agreed to in writing, software
  # distributed under the License is distributed on an "AS IS" BASIS,
  # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  # See the License for the specific language governing permissions and
  # limitations under the License.
  ```

  C++ files use the same text with `//` comments.

## New packages

- Name it `warehouse_<purpose>` and create it only in the phase that needs it.
- **ament_cmake packages:** copy the lint setup from `warehouse_bringup`: the
  `ament_cmake_clang_format`, `ament_lint_auto` and `ament_lint_common` test dependencies in
  `package.xml`, and the `if(BUILD_TESTING)` block of `CMakeLists.txt`, which requires
  `ament_cmake_clang_format` and excludes `ament_cmake_uncrustify`.
- **ament_python packages:** keep the `test/test_copyright.py`, `test/test_flake8.py` and
  `test/test_pep257.py` files that `ros2 pkg create --build-type ament_python` generates, with
  the test dependencies `ament_copyright`, `ament_flake8`, `ament_pep257` and `python3-pytest`.
  Delete the `@pytest.mark.skip(...)` line in `test_copyright.py`: the generated file skips the
  licence check.
- After the first `colcon test`, check that `colcon test-result --all` lists the expected
  linters (for C++: `clang_format`, not `uncrustify`) and reports `0 skipped`.
- Add a short `README.md` describing what the package does and how to run it.

## Licensing of contributions

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
