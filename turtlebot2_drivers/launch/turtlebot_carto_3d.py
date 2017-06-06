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

from launch.exit_handler import default_exit_handler, restart_exit_handler

def launch(launch_descriptor, argv):
    ld = launch_descriptor
    ld.add_process(
        cmd=['kobuki_node'],
        name='kobuki_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['astra_camera_node', '-dw', '320', '-dh', '240'],
        name='astra_camera_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['depth_to_pointcloud_node'],
        name='depth_to_pointcloud_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['static_transform_publisher', '0', '0', '0', '0', '0', '0', '1', 'base_link', 'openni_depth_optical_frame'],
        name='static_transform_publisher',
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
    ld.add_process(
        cmd=['cartographer_node', '-configuration_directory', '/home/ubuntu/ros2_ws-new-cart/install_isolated/cartographer_ros/share/cartographer_ros/configuration_files', '-configuration_basename', 'turtlebot_3d.lua'],
        name='cartographer_node',
        exit_handler=restart_exit_handler,
    )
