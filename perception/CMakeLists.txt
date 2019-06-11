cmake_minimum_required(VERSION 2.8.3)
project(perception)
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


# Catkin
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  tf
  image_geometry
  darknet_ros_msgs
)

add_service_files(
  FILES
  perc.srv
)

 generate_messages(
   DEPENDENCIES 
   geometry_msgs
   std_msgs
 )

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs darknet_ros_msgs)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(perception src/perception.cpp)
target_link_libraries(perception
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS perception
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

