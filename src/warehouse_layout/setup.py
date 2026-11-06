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

from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'warehouse_layout'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # The empty marker file that lets ament_index find the package.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Made from layout.yaml by `generate` and committed (the world goes to warehouse_gazebo).
        (os.path.join('share', package_name, 'generated'), glob('generated/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanay Baisware',
    maintainer_email='tanay.baisware@gmail.com',
    description='The warehouse from one file: layout.yaml, its loader and schema check, '
                'the layout fit rules, a to-scale preview picture, and the generators of '
                'the Gazebo world, the location registry and the expected inventory.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'generate = warehouse_layout.cli:main',
            'preview = warehouse_layout.preview:main',
        ],
    },
)
