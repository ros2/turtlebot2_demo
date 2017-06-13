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


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    ld.add_process(
        cmd=['kobuki_node'],
        name='kobuki_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['astra_camera_node', '-C', '-I', '-dw', '320', '-dh', '240'],
        name='astra_camera_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['follower'],
        name='follower',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['joy_node'],
        name='joy',
        # The joy node is optional, we don't care if it actually launches
        exit_handler=ignore_exit_handler,
    )
