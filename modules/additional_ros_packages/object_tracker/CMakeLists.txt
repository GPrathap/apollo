cmake_minimum_required(VERSION 2.8)
project(test_tensorflow)

find_package(catkin REQUIRED COMPONENTS tensorflow_catkin)

catkin_package(CATKIN_DEPENDS tensorflow_catkin)

include_directories(include ${catkin_INCLUDE_DIRS})

message("----------------------------" ${catkin_LIBRARIES})
# add the publisher example
add_executable(test_tensorflow src/example.cpp)
add_dependencies(test_tensorflow ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_tensorflow ${catkin_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS test_tensorflow
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

