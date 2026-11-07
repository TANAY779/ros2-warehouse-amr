# Contributing

## Workflow

- **Everything goes to `main`:** commit directly on `main` and push. No feature branches, no
  pull requests, no automated tests, lint checks or CI
  ([ADR-008](docs/decisions/ADR-008-develop-on-main-without-automated-tests.md)). Commit working
  steps: whatever is pushed is on `main`. To undo a commit that is already pushed, use
  `git revert <commit>`; never rewrite `main`'s history (force-pushes are blocked).
- **Check a change by building and running it** ([below](#build-and-run)) before you commit.
- **Commit messages** follow [Conventional Commits](https://www.conventionalcommits.org/), with
  the package as the scope:

  ```text
  feat(warehouse_navigation): add keep-out filter mask
  fix(warehouse_battery): clamp charge at 100 %
  docs: add ADR-005 for localization
  ```

  Types: `feat`, `fix`, `docs`, `refactor`, `perf`, `build`, `chore`.
- **Tags:** each finished phase gets a marker tag `phase-N` (e.g. `phase-3`, `phase-4a`).
  Releases are tagged only at milestones: `v1.0.0` (M1), `v2.0.0` (M2), `v3.0.0` (M3), each
  with a GitHub Release. Before a release tag, set `<version>` in every `package.xml` to the
  release number; between releases packages stay at their last release (0.1.0 before v1.0.0).
- **Design decisions** that are hard to reverse get an ADR in `docs/decisions/`, in the commit
  that makes the decision.
- **Documentation:** a commit that adds or changes a feature updates
  [docs/PROJECT_STATUS.md](docs/PROJECT_STATUS.md) in the same commit (the feature entry, the
  decisions log and the planned list). Every finished phase adds a learning report,
  `docs/phase-reports/phase-NN-<name>.md` (for example `phase-01-robot-description.md`), before
  its `phase-N` tag.
- **Media:** demo GIFs and other files in `docs/media/` stay under 3 MB; every other file
  stays under 1 MB. Nothing checks this for you: look at the size (`ls -lh`) before committing
  a picture or a data file. Longer demos go on a GitHub Release as MP4 files, linked from the
  README.

## Build and run

Native Ubuntu 24.04 with ROS 2 Jazzy (including `ros-dev-tools`), or the dev container (see the
README). From the workspace root (`~/warehouse_ws`):

```bash
source /opt/ros/jazzy/setup.bash
vcs import --input dependencies.repos src
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

To rebuild a single package: `colcon build --symlink-install --packages-select <pkg>`.

Then run what you changed and measure it:

```bash
ros2 launch warehouse_bringup sim.launch.py headless:=true    # Gazebo without windows, or
ros2 launch warehouse_bringup sim.launch.py hardware:=mock    # no simulator
# in a second terminal with the same workspace sourced:
ros2 control list_controllers -c /robot1/controller_manager   # three controllers, all active
ros2 topic hz /robot1/scan                                    # about 10 Hz (Gazebo)
```

The full list of checks is in
[Checking the robot by hand](src/warehouse_bringup/README.md#checking-the-robot-by-hand). For the
warehouse layout, the map and localization, use the tools: `ros2 run warehouse_layout preview`
(the fit rules), `generate --check`, `check_map`, and `warehouse_system_tests`' `drive_route`
and `evaluate_localization` (their READMEs say how). A phase's acceptance criteria in
[docs/PROJECT_PLAN.md](docs/PROJECT_PLAN.md) are checked the same way.

## Code style

Nothing checks the style automatically (ADR-008). Editors apply `.editorconfig` (indentation,
line endings, final newline) and, for C++, `.clang-format`; follow the rest by hand.

- **C++:** ROS 2 style via clang-format and the repository `.clang-format` (`clang-format -i
  <file>`, or the editor's format command).
- **Includes:** angle brackets only for the C/C++ standard library; ROS, third-party and
  project headers use double quotes (`#include "rclcpp/rclcpp.hpp"`), as in ROS 2's own code.
  clang-format sorts every `<...>` include into the standard-library block.
- **Python:** the ROS 2 style: 99-character lines, Google import order, single quotes, pep257
  style for the docstrings you write.
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

- Name it `warehouse_<purpose>`, put it in `src/` and create it only in the phase that needs it.
- `ros2 pkg create` adds test scaffolding; delete it, since the project has no tests: for
  `ament_cmake`, the `if(BUILD_TESTING)` block in `CMakeLists.txt` and the `test_depend` entries
  in `package.xml`; for `ament_python`, the `test/` folder, the `test_depend` entries and the
  `tests_require` or `extras_require` test entry in `setup.py`.
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
