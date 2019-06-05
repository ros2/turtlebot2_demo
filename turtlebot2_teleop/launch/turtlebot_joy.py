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

import launch
import launch_ros.actions

def generate_launch_description():
    kobuki_node = launch_ros.actions.Node(package='turtlebot2_drivers',
                                          node_executable='kobuki_node',
                                          output='screen')

    teleop_twist_node = launch_ros.actions.Node(package='teleop_twist_joy',
                                                node_executable='teleop_node',
                                                output='screen')

    joy_node = launch_ros.actions.Node(package='joy',
                                       node_executable='joy_node',
                                       output='screen')

    return launch.LaunchDescription([joy_node,
                                     kobuki_node,
                                     teleop_twist_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=kobuki_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
