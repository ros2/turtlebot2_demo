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

from launch.exit_handler import default_exit_handler, restart_exit_handler


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    ld.add_process(
        cmd=['kobuki_node'],
        name='kobuki_node',
        exit_handler=restart_exit_handler,
    )
    ld.add_process(
        cmd=['joy_node'],
        name='joy_node',
        # The joy node is required, die if it dies
        exit_handler=default_exit_handler,
    )
