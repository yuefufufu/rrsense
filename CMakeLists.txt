cmake_minimum_required(VERSION 3.0.2)
project(rrsense_test)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL 1.1 REQUIRED)

catkin_package(
  
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(tutorial src/tutorial.cpp)
target_link_libraries(tutorial ${catkin_LIBRARIES})
add_executable(tutorial1 src/tutorial1.cpp)
target_link_libraries(tutorial1 ${catkin_LIBRARIES})
add_executable(tutorial2 src/tutorial2.cpp)
target_link_libraries(tutorial2 ${catkin_LIBRARIES})
add_executable(tutorial3 src/tutorial3.cpp)
target_link_libraries(tutorial3 ${catkin_LIBRARIES})
add_executable(tutorial4 src/tutorial4.cpp)
target_link_libraries(tutorial4 ${catkin_LIBRARIES})