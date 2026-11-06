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
Read and write rosbag2 bags with rosbag2_py, without a running ROS graph.

``ros2 bag record`` writes MCAP files in Jazzy. These helpers turn a bag into
(topic, message, time) tuples and back: the offline tools (evaluate_localization,
map_timelapse) read bags this way, and tests write small synthetic bags with known content.

A recorded message carries two times. The bag stores the time the recorder received it (sim
time when recorded with ``--use-sim-time``, else wall-clock time). The message's header holds
the stamp its publisher gave it: for a scan or an AMCL pose, the moment the measurement was
taken. The tools compare messages by header stamp, so recording delays and the recorder's
clock never enter a result.
"""

from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def _open(path):
    """Open the bag at ``path`` (a bag folder or one .mcap file) for reading."""
    reader = rosbag2_py.SequentialReader()
    # storage_id '' lets rosbag2 tell MCAP from SQLite by itself.
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def topic_types(path):
    """Return {topic name: message type name} of the bag at ``path``."""
    reader = _open(path)
    try:
        return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    finally:
        reader.close()


def read(path, topics):
    """
    Yield (topic, message, bag time in ns) for every message on ``topics``, oldest first.

    Topics the bag does not have are skipped. Messages are deserialized one at a time, so a
    long bag never has to fit in memory.
    """
    reader = _open(path)
    try:
        types = {topic.name: get_message(topic.type)
                 for topic in reader.get_all_topics_and_types() if topic.name in topics}
        if not types:
            return
        reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(types)))
        while reader.has_next():
            topic, data, time_ns = reader.read_next()
            yield topic, deserialize_message(data, types[topic]), time_ns
    finally:
        reader.close()


def type_name(message):
    """Return the ROS type name of a message, for example 'nav_msgs/msg/Odometry'."""
    package = type(message).__module__.split('.')[0]
    return f'{package}/msg/{type(message).__name__}'


def write(path, messages, storage_id='mcap'):
    """
    Write a new bag at ``path`` from an iterable of (topic, message, bag time in ns).

    Each topic is created the first time it appears, with its message's type.
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    try:
        created = set()
        for topic, message, time_ns in messages:
            if topic not in created:
                # Jazzy's TopicMetadata starts with a numeric id (new in rosbag2 0.26).
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=len(created), name=topic, type=type_name(message),
                    serialization_format='cdr'))
                created.add(topic)
            writer.write(topic, serialize_message(message), int(time_ns))
    finally:
        writer.close()
