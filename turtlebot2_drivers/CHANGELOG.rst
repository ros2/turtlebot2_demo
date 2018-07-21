^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package turtlebot2_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2018-07-20)
------------------

0.4.0 (2017-12-08)
------------------
* add missing maintainer names (`#73 <https://github.com/ros2/turtlebot2_demo/issues/73>`_)
* Remove parameter_service namespace re-introduced (`#72 <https://github.com/ros2/turtlebot2_demo/issues/72>`_)
* Update for rclcpp namespace removals (`#71 <https://github.com/ros2/turtlebot2_demo/issues/71>`_)
* [kobuki_driver] use rclcpp logging (`#69 <https://github.com/ros2/turtlebot2_demo/issues/69>`_)
* Switch to using the RCUTILS\_* logging macros. (`#66 <https://github.com/ros2/turtlebot2_demo/issues/66>`_)
* 0.0.3
* Disable strict-aliasing warnings in the kobuki driver. (`#55 <https://github.com/ros2/turtlebot2_demo/issues/55>`_)
* 0.0.2
* Tb2 repos refactor (`#46 <https://github.com/ros2/turtlebot2_demo/issues/46>`_)
* Make sure to fully initialize the QoS structures. (`#43 <https://github.com/ros2/turtlebot2_demo/issues/43>`_)
* Fixes to make TB2 follower demo build/launch. (`#41 <https://github.com/ros2/turtlebot2_demo/issues/41>`_)
* use CMAKE_X_STANDARD and check compiler rather than platform
* Disable the RGB and IR cameras when launching the tb2 carto demo. (`#39 <https://github.com/ros2/turtlebot2_demo/issues/39>`_)
* Update exec dependencies (`#37 <https://github.com/ros2/turtlebot2_demo/issues/37>`_)
* Make the static transform for the IMU agree with reality.
* Add in launch files for cartographer 2D and 3D.
* More fixes for the kobuki node.
* Switch to rcutils_system_time_now. (`#25 <https://github.com/ros2/turtlebot2_demo/issues/25>`_)
* Add depth_to_pointcloud node and add some necessary information to kobuki_node (`#11 <https://github.com/ros2/turtlebot2_demo/issues/11>`_)
* fix isolated build by adding missing dep (`#17 <https://github.com/ros2/turtlebot2_demo/issues/17>`_)
* add tf2 dep (`#16 <https://github.com/ros2/turtlebot2_demo/issues/16>`_)
* add missing sensor_msgs dependency (`#10 <https://github.com/ros2/turtlebot2_demo/issues/10>`_)
* Flesh out kobuki driver (`#7 <https://github.com/ros2/turtlebot2_demo/issues/7>`_)
* tweak
* all manner of improvements
* add some launch files
* comments
* add some param parsing
* comment out debug
* start adding parameters
* turtlebot follows
* Merge branch 'master' of github.com:ros2/turtlebot2_demo
* make dumb teleop repeat
* add watchdog and improve cleanup
* first working driver
* first version of ROS 2 node that links against kobuki_driver
* Contributors: Brian Gerkey, Chris Lalancette, Daniel Stonier, Dirk Thomas, Mikael Arguedas, Morgan Quigley, Rohan Agrawal, dhood, gerkey, vdiluoffo
