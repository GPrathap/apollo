cmake_minimum_required(VERSION 2.8)
project(test_tensorflow)

add_definitions("-O3 -w -Wall -c -fmessage-length=0 -MMD -MP -std=c++11")

find_package(catkin REQUIRED COMPONENTS)

catkin_package(CATKIN_DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})

message("----------------------------" ${catkin_LIBRARIES})

INCLUDE_DIRECTORIES(/home/tmp/ros/include/tensorflow_catkin/include/)
LINK_DIRECTORIES(/home/tmp/ros/lib/tensorflow_catkin/lib/)
# export LD_LIBRARY_PATH=/apollo/modules/devel_isolated/tensorflow_catkin/lib/${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
# add the publisher example
add_executable(test_tensorflow src/example.cpp)
add_dependencies(test_tensorflow ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_tensorflow tensorflow)

# Mark executables and/or libraries for installation
install(TARGETS test_tensorflow
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

