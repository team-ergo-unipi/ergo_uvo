cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_ros
)

find_package(OpenCV 4 REQUIRED) 

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package()

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


####################################################################################
####################################### NODES ######################################
####################################################################################

# MONO VO
add_executable(mono_VO src/mono_VO.cpp)
add_dependencies(mono_VO visual_odometry_generate_messages_cpp)
target_link_libraries(mono_VO  
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})


# STEREO VO
add_executable(stereo_VO src/stereo_VO.cpp)
add_dependencies(stereo_VO visual_odometry_generate_messages_cpp)
target_link_libraries(stereo_VO  
${catkin_LIBRARIES}
${OpenCV_LIBRARIES})