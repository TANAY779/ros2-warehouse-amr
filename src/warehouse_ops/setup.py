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

from setuptools import find_packages
from setuptools import setup

package_name = 'warehouse_ops'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        # The empty marker file that lets ament_index find the package.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanay Baisware',
    maintainer_email='tanay.baisware@gmail.com',
    description='Operator tools of the warehouse AMR: navigate sends the robot to named '
                'locations of the location registry, or to poses on the map, through '
                "Nav2's actions.",
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navigate = warehouse_ops.navigate:main',
        ],
    },
)
