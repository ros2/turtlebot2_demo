# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from launch.exit_handler import ignore_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'turtlebot2_drivers'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='kobuki_node')],
        name='kobuki_node',
        exit_handler=restart_exit_handler,
    )
    package = 'astra_camera'
    ld.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='astra_camera_node'),
            '-dw', '320', '-dh', '240', '-C', '-I'],
        name='astra_camera_node',
        exit_handler=restart_exit_handler,
    )
    package = 'turtlebot2_follower'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='follower')],
        name='follower',
        exit_handler=restart_exit_handler,
    )
    package = 'teleop_twist_joy'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='teleop_node')],
        name='teleop_node',
        # The teleop node is optional, we don't care if it actually launches
        exit_handler=ignore_exit_handler,
    )
    package = 'joy'
    ld.add_process(
        cmd=[get_executable_path(package_name=package, executable_name='joy_node')],
        name='joy',
        # The joy node is optional, we don't care if it actually launches
        exit_handler=ignore_exit_handler,
    )

    return ld
