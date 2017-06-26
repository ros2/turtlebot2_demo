#!/usr/bin/python3
#
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

import os
import sys

from ament_index_python import get_package_prefix
from launch import LaunchDescriptor
from launch.exit_handler import default_exit_handler, restart_exit_handler
from launch.launcher import DefaultLauncher


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    package = 'turtlebot2_drivers'
    ld.add_process(
        cmd=[os.path.join(get_package_prefix(package), 'lib', package, 'kobuki_node')],
        name='kobuki_node',
        exit_handler=restart_exit_handler,
    )
    package = 'teleop_twist_joy'
    ld.add_process(
        cmd=[os.path.join(get_package_prefix(package), 'lib', package, 'teleop_node')],
        name='teleop_node',
        exit_handler=restart_exit_handler,
    )
    package = 'joy'
    ld.add_process(
        cmd=[os.path.join(get_package_prefix(package), 'lib', package, 'joy_node')],
        name='joy_node',
        # The joy node is required, die if it dies
        exit_handler=default_exit_handler,
    )

    return ld


def main(argv=sys.argv[1:]):
    launcher = DefaultLauncher()
    launch_descriptor = launch(LaunchDescriptor(), argv)
    launcher.add_launch_descriptor(launch_descriptor)
    rc = launcher.launch()
    return rc


if __name__ == '__main__':
    sys.exit(main())
