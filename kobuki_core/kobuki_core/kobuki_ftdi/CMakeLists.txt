cmake_minimum_required(VERSION 3.5)
project(kobuki_ftdi)

find_package(ament_cmake REQUIRED)
find_package(ecl_command_line REQUIRED)

# pkg-config packages
find_package(PkgConfig)
pkg_search_module(libusb REQUIRED libusb)
pkg_search_module(libftdi REQUIRED libftdi)



include_directories(include)

add_subdirectory(src)

install(DIRECTORY bluetooth
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY eeproms
        DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/turtlebot_config
                 scripts/create_udev_rules
        DESTINATION bin/${PROJECT_NAME}
)

install(FILES 57-kobuki.rules
        DESTINATION share/${PROJECT_NAME}
)

# This has issues....hydro doing th same thing will result in
# a file conflict. We could call it 57-groovy-kobuki.rules and
# 57-hydro-kobuki.rules. But that seems kind of lame and could
# break things if they are actually different (which should
# be called - udev will have no idea).

#install(FILES 57-kobuki.rules
#        DESTINATION /etc/udev/rules.d/
#)

ament_export_include_directories(include)
ament_export_dependencies(libusb libftdi ecl_command_line)

ament_package()
