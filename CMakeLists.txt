# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.8)
project(dynamic_detector)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mavx -mbmi2")
set(CMAKE_BUILD_TYPE Release)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg pcl_ros pcl_conversions)
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} external_libs ${EIGEN3_INCLUDE_DIR})

## Build the process_clouds node
add_executable(dynamic_detector src/dynamic_detector.cpp)
set_property(TARGET dynamic_detector PROPERTY CXX_STANDARD 17)
target_link_libraries(dynamic_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} phtree)
#if(OpenMP_CXX_FOUND)
#    target_link_libraries(dynamic_detector PUBLIC OpenMP::OpenMP_CXX)
#endif()
#add_dependencies(dynamic_detector beginner_tutorials_generate_messages_cpp)

## Build the apply_tf node
add_executable(apply_tf src/apply_tf.cpp)
target_link_libraries(apply_tf ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(apply_tf beginner_tutorials_generate_messages_cpp)

## Build the upsampling node
add_executable(upsampling src/upsampling.cpp)
set_property(TARGET dynamic_detector PROPERTY CXX_STANDARD 17)
target_link_libraries(upsampling ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# %EndTag(FULLTEXT)%

# Add OpenMP support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
