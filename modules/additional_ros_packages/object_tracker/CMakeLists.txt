cmake_minimum_required(VERSION 2.8)
project(test_tensorflow)

find_package(catkin_simple REQUIRED tensorflow_catkin)
catkin_simple()

catkin_package(CATKIN_DEPENDS tensorflow_catkin)

cs_add_executable(example src/example.cpp)

cs_install()
cs_export()