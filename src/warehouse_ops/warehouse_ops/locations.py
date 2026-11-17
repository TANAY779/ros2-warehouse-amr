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
The named locations of the warehouse, read from the location registry.

warehouse_layout generates the registry from layout.yaml and installs it as
``share/warehouse_layout/generated/locations.yaml``. This module reads the installed file (it
keeps no copy), checks that it has the schema version this code was written for and that its
poses are in the map frame, and returns every entry of ``poses`` with its kind:

- ``spawn``: where a robot starts, and where it parks;
- ``pick``: on a pick aisle's centreline at a bay, facing along the aisle;
- ``drop``: at a packing station's counter;
- ``dock_staging``: in front of a charging dock, where docking starts;
- ``dock_docked``: on the dock, the robot's front on the charging contacts, inside the keepout
  box round the contact bar that Nav2 will keep out of once Phase 3b loads the zones. Only the
  docking server drives there (Phase 5), straight from the staging pose onto the contacts. So
  these poses are *docking only*: navigate lists them and refuses to send the robot to them.
"""

from dataclasses import dataclass
import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml

# The registry version this code reads: SCHEMA_VERSION in warehouse_layout's registry.py.
SCHEMA_VERSION = 1
# The registry's frame. Nav2's goals are in the map frame too, so a pose goes to Nav2 as it is.
FRAME_ID = 'map'

# Every kind of pose, in the order navigate --list prints them.
KINDS = ('spawn', 'pick', 'drop', 'dock_staging', 'dock_docked')
# The kinds only the docking server drives to.
DOCKING_ONLY = ('dock_docked',)


@dataclass(frozen=True)
class Location:
    """One named pose of the registry, in the map frame: metres and radians."""

    name: str
    kind: str
    x: float
    y: float
    yaw: float
    # The entry's other fields, as ((key, value), ...): robot, station, dock, aisle, bay, bays.
    details: tuple = ()

    @property
    def docking_only(self):
        """Return True for a pose that only the docking server drives to."""
        return self.kind in DOCKING_ONLY

    def detail(self, key, default=None):
        """Return one of the entry's other fields, such as 'dock'."""
        return dict(self.details).get(key, default)


@dataclass(frozen=True)
class Registry:
    """The location registry: where it was read from, its world and its poses by name."""

    path: str
    world: str
    locations: dict          # name -> Location, in the file's order

    def of_kind(self, kind):
        """Return the locations of one kind, in the file's order."""
        return [location for location in self.locations.values() if location.kind == kind]

    def staging_pose_of(self, location):
        """Return the name of the staging pose of a docked pose's dock, or None."""
        for staging in self.of_kind('dock_staging'):
            if staging.detail('dock') == location.detail('dock'):
                return staging.name
        return None


class RegistryError(ValueError):
    """The registry cannot be read, or is not what this code expects; the message says why."""


def registry_path():
    """Return the path of the installed location registry, in warehouse_layout's share folder."""
    share = get_package_share_directory('warehouse_layout')
    return os.path.join(share, 'generated', 'locations.yaml')


def load(path=None):
    """Return the Registry in ``path`` (default: the installed one); raise RegistryError."""
    path = path or registry_path()
    try:
        with open(path) as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RegistryError(f'cannot read the location registry {path}: {error}') from None
    if not isinstance(data, dict):
        raise RegistryError(f'{path} is not a location registry')
    version = data.get('schema_version')
    if version != SCHEMA_VERSION:
        # The tool and the registry are installed from the same workspace, so a different
        # version means one of them is stale: building again brings them together.
        raise RegistryError(
            f'{path} has schema_version {version}, but this tool reads version '
            f'{SCHEMA_VERSION}: build the workspace again')
    frame = data.get('frame_id')
    if frame != FRAME_ID:
        raise RegistryError(
            f'{path} has its poses in the frame {frame!r}, but navigate sends goals in the '
            f'{FRAME_ID!r} frame')
    poses = data.get('poses')
    if not isinstance(poses, dict) or not poses:
        raise RegistryError(f'{path} has no poses')
    locations = {str(name): _location(path, str(name), entry) for name, entry in poses.items()}
    return Registry(path=path, world=str(data.get('world', '')), locations=locations)


def _location(path, name, entry):
    """Return one ``poses`` entry as a Location; raise RegistryError if it is malformed."""
    try:
        kind = entry['kind']
        x, y, yaw = (float(entry[key]) for key in ('x', 'y', 'yaw'))
    except (TypeError, KeyError, ValueError) as error:
        raise RegistryError(
            f'{path}: the pose {name} needs a kind and numbers x, y and yaw ({error})') from None
    if kind not in KINDS:
        raise RegistryError(
            f'{path}: the pose {name} has the kind {kind!r}; this tool knows {", ".join(KINDS)}')
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise RegistryError(f'{path}: the pose {name} has a coordinate that is not a number')
    details = tuple((key, value) for key, value in entry.items()
                    if key not in ('kind', 'x', 'y', 'yaw'))
    return Location(name=name, kind=kind, x=x, y=y, yaw=yaw, details=details)
