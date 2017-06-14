# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch.exit_handler import restart_exit_handler


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    ld.add_process(
        cmd=['kobuki_node'],
        name='kobuki_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['astra_camera_node', '-dw', '320', '-dh', '240', '-C', '-I'],
        name='astra_camera_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['depth_to_pointcloud_node'],
        name='depth_to_pointcloud_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        # The XYZ/Quat numbers for base_link -> camera_rgb_frame are taken from the
        # turtlebot URDF in
        # https://github.com/turtlebot/turtlebot/blob/931d045/turtlebot_description/urdf/sensors/astra.urdf.xacro
        cmd=[
            'static_transform_publisher',
            '-0.087', '-0.0125', '0.287',
            '0', '0', '0', '1',
            'base_link', 'camera_rgb_frame'
        ],
        name='static_tf_pub_base_rgb',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        # The XYZ/Quat numbers for camera_rgb_frame -> camera_depth_frame are taken from the
        # turtlebot URDF in
        # https://github.com/turtlebot/turtlebot/blob/931d045/turtlebot_description/urdf/sensors/astra.urdf.xacro
        cmd=[
            'static_transform_publisher',
            '0', '0.0250', '0',
            '0', '0', '0', '1',
            'camera_rgb_frame',
            'openni_depth_optical_frame'
        ],
        name='static_tf_pub_rgb_depth',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['joy_node'],
        name='joy_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['teleop_node'],
        name='teleop_node',
        exit_handler=restart_exit_handler,
    )
    cartographer_ros_prefix = get_package_share_directory('cartographer_ros')
    cartographer_config_dir = os.path.join(cartographer_ros_prefix, 'configuration_files')
    ld.add_process(
        cmd=[
            'cartographer_node',
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'turtlebot_3d.lua'
        ],
        name='cartographer_node',
        exit_handler=restart_exit_handler,
    )
