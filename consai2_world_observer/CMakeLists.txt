cmake_minimum_required(VERSION 2.8.3)
project(consai2_world_observer)

find_package(PkgConfig)

if("$ENV{ROS_DISTRO}" STRGREATER "lunar")
    pkg_check_modules(BFL REQUIRED orocos-bfl)
else()
    pkg_check_modules(BFL REQUIRED bfl)
endif()


message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  consai2_msgs
)

catkin_package()

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

add_executable(consai2_world_observer
  src/main.cpp
  src/geometry/geometry.cpp
  src/world_observer_ros.cpp
  src/estimator/estimator.cpp
)

target_link_libraries(consai2_world_observer
 ${catkin_LIBRARIES}
 ${BFL_LIBRARIES}
)

add_dependencies(consai2_world_observer consai2_msgs_generate_messages_cpp)
