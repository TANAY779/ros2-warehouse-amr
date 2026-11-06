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

"""
The generate command: write the files made from layout.yaml, or check that they are current.

Usage, from the workspace root::

    ros2 run warehouse_layout generate            # write the files that changed
    ros2 run warehouse_layout generate --check    # write nothing; exit 1 if one is stale

It reads the source tree, not the install: src/warehouse_layout/config/layout.yaml and the
robot's size from src/warehouse_description/config/robot.yaml (``--src`` points elsewhere).
It writes, in the source tree:

- warehouse_gazebo/worlds/<name>.sdf: the Gazebo world (world.py);
- warehouse_layout/generated/locations.yaml: the location registry (registry.py);
- warehouse_layout/generated/expected_inventory.yaml: the expected inventory (inventory.py).

The files are committed to git rather than made at build time (plan P2-D1): a layout change
then shows up in review as a change to the world and the registry, and Gazebo and later nodes
load plain files. The price is that they can go stale, so ``--check`` and the test
test_generated_files_are_current.py compare them with what the layout gives now. A file is
rewritten only when its text changes, so an unchanged file keeps its date.

Exit codes: 0 done (or, with --check, every file current); 1 with --check, a file is stale or
missing; 2 the layout or robot.yaml cannot be read.
"""

import argparse
from dataclasses import dataclass
import os
import sys

from warehouse_description import robot_config
from warehouse_layout import inventory
from warehouse_layout import model
from warehouse_layout import registry
from warehouse_layout import rules
from warehouse_layout import world


@dataclass(frozen=True)
class Output:
    """
    One generated file: where it goes, its text and a one-line summary of what it holds.

    ``path`` is relative to the package directory, in the source tree (src/<package>/<path>)
    and in the install (the package's share directory) alike.
    """

    package: str
    path: str
    text: str
    summary: str


def outputs(layout):
    """Return every file generated from the layout, as Output, in a fixed order."""
    totes = inventory.expected_inventory(layout)
    world_text = world.render(layout)
    return (
        Output('warehouse_gazebo', f'worlds/{layout.name}.sdf', world_text,
               world.summary(world_text)),
        Output('warehouse_layout', 'generated/locations.yaml', registry.render(layout),
               registry.summary(layout)),
        Output('warehouse_layout', 'generated/expected_inventory.yaml',
               inventory.render(layout, totes), inventory.summary(layout, totes)),
    )


def main(argv=None):
    """Run the generate command; return its exit code."""
    parser = argparse.ArgumentParser(
        description='Generate the Gazebo world, the location registry and the expected '
                    'inventory from layout.yaml, in the source tree.')
    parser.add_argument('--src', default='src',
                        help='the workspace source directory (default: ./src)')
    parser.add_argument('--check', action='store_true',
                        help='write nothing; exit 1 if a generated file is stale or missing')
    args = parser.parse_args(argv)

    layout_path = os.path.join(args.src, 'warehouse_layout', 'config', 'layout.yaml')
    robot_path = os.path.join(args.src, 'warehouse_description', 'config', 'robot.yaml')
    try:
        robot = robot_config.load(robot_path)
        layout = model.load(layout_path, robot=robot)
        files = outputs(layout)
    except OSError as error:
        print(f'error: {error}\nRun this from the workspace root (the folder with src/ in it), '
              'or pass --src.', file=sys.stderr)
        return 2
    except ValueError as error:   # a LayoutError, a bad robot.yaml, or too many totes
        print(f'error: {error}', file=sys.stderr)
        return 2

    # The fit rules decide whether the robot can work in this layout; generating does not
    # depend on them, but a world that breaks them is worth a warning.
    failed = [result.rule for result in rules.check(layout) if not result.passed]
    if failed:
        print(f'warning: the layout breaks {len(failed)} fit rule(s): {", ".join(failed)}. '
              'Run `ros2 run warehouse_layout preview --out /tmp/layout.png` for details.',
              file=sys.stderr)
    return _check(args.src, files) if args.check else _write(args.src, files)


def _read(path):
    """Return a file's bytes, or None if there is no such file."""
    try:
        with open(path, 'rb') as stream:
            return stream.read()
    except FileNotFoundError:
        return None


def _write(src, files):
    for output in files:
        path = os.path.join(src, output.package, output.path)
        # Bytes, not text: no newline translation, so the file is the same on every system.
        data = output.text.encode('utf-8')
        if _read(path) == data:
            print(f'unchanged  {path}: {output.summary}')
            continue
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as stream:
            stream.write(data)
        print(f'wrote      {path}: {output.summary}')
    return 0


def _check(src, files):
    stale = []
    for output in files:
        path = os.path.join(src, output.package, output.path)
        found = _read(path)
        if found != output.text.encode('utf-8'):
            stale.append(path)
            print(f'{"missing" if found is None else "stale"}: {path}', file=sys.stderr)
    if stale:
        print(f'{len(stale)} of {len(files)} generated files are out of date. Regenerate them '
              'from the workspace root: ros2 run warehouse_layout generate', file=sys.stderr)
        return 1
    print(f'all {len(files)} generated files are current')
    return 0


if __name__ == '__main__':
    sys.exit(main())
