cmake_minimum_required(VERSION 3.0.2)
project(door_angle)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  cv_bridge
  sensor_msgs
  visualization_msgs
  pcl_ros
  message_generation
  tf2
)
find_package (OpenCV 3.2.0 REQUIRED)
#find_package(Eigen3 REQUIRED)

add_message_files(FILES bounding.msg BoundingBox.msg BoundingBoxes.msg DoorPose.msg DoorPoses.msg)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES door_angle
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
  INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(door src/door.cpp)
target_link_libraries(door ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(plane_seg src/plane_seg.cpp)
target_link_libraries(plane_seg ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(fake_bounding src/fake_bounding.cpp)
target_link_libraries(fake_bounding ${catkin_LIBRARIES})

add_executable(storage_ src/storage_.cpp)
target_link_libraries(storage_ ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(door_camera_lidar src/door_camera_lidar.cpp)
target_link_libraries(door_camera_lidar ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(door_register src/door_register.cpp)
target_link_libraries(door_register ${catkin_LIBRARIES} ${OpenCV_LIBS})
