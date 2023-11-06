cmake_minimum_required(VERSION 0.0.1)
project(unitree_legged_real)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    unitree_legged_msgs
    unitree_legged_sdk
    tf
)
find_package (Eigen3 REQUIRED NO_MODULE)

catkin_package()

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(LEGGED_SDK_NAME -pthread libunitree_legged_sdk.a lcm)

set(EXTRA_LIBS ${LEGGED_SDK_NAME})


set(CMAKE_CXX_FLAGS "-O3 -fPIC")


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${UNITREE_LEGGED_SDK_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}/unitree_legged_sdk/include
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})
add_library(unitree_ros SHARED)
target_link_libraries(unitree_ros ${EXTRA_LIBS} ${catkin_LIBRARIES})
target_sources(
  unitree_ros PRIVATE
  src/ros_udp_node.cpp
)




add_executable(ros_udp_node src/ros_udp_node.cpp)
target_link_libraries(ros_udp_node ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(ros_udp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(real_robot src/real_robot.cpp)
target_link_libraries(real_robot ${EXTRA_LIBS} ${catkin_LIBRARIES} unitree_ros)
add_dependencies(real_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(legged_twist src/legged_twist.cpp)
target_link_libraries(legged_twist ${catkin_LIBRARIES})
add_dependencies(legged_twist ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} Eigen3::Eigen)

# add_executable(example_walk src/exe/example_walk.cpp)
# target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(example_position src/exe/example_position.cpp)
# target_link_libraries(example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(state_sub src/exe/state_sub.cpp)
# target_link_libraries(state_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(state_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(ros_udp src/exe/ros_udp.cpp)
# target_link_libraries(ros_udp ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
# target_link_libraries(control_via_keyboard ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(control_via_keyboard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(twist_sub src/exe/twist_sub.cpp)
# target_link_libraries(twist_sub ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(twist_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
