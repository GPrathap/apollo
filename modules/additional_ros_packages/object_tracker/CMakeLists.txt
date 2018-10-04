cmake_minimum_required(VERSION 2.8)
project(test_tensorflow)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_executable(example src/example.cpp)

cs_install()
cs_export()