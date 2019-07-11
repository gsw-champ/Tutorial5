cmake_minimum_required(VERSION 2.8.3)
project(perception_person)
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

)

add_service_files(
  FILES
  PercepPerson.srv
)

generate_messages(
  DEPENDENCIES 
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(perception_person src/perception_person.cpp)
target_link_libraries(perception_person
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

#add_executable(text_per src/text_per.cpp)
#target_link_libraries(text_per
#  ${catkin_LIBRARIES}
#)

install(TARGETS perception_person
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

