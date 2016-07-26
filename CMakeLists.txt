cmake_minimum_required(VERSION 2.8.3)
project(normal_extraction)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions pcl_ros custom_msgs
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(normal_extraction src/normal_extraction.cpp src/normalextractor.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(normal_extraction
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 )
