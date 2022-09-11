cmake_minimum_required(VERSION 3.0.2)
project(robomaster_driver)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_install()

cs_install_scripts(src/robomaster_driver_node.py)

cs_export()