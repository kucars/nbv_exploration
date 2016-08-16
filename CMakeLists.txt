cmake_minimum_required(VERSION 2.8.3)
project(nbv_exploration)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
add_definitions(-D__PX4_ROS)
add_definitions(-D__EXPORT=)
add_definitions(-DMAVLINK_DIALECT=common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  cmake_modules
  gazebo_msgs
  sensor_msgs
  mav_msgs
  libmavconn
  tf
  tf_conversions
  eigen_conversions
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS #src/include
  DEPENDS Eigen
  #LIBRARIES px4
  #CATKIN_DEPENDS message_runtime
  #roscpp rospy std_msgs libmavconn
  #DEPENDS system_lib
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})


#add_executable(wall_follower src/wall_follower.cpp)
#add_dependencies(wall_follower ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wall_follower ${catkin_LIBRARIES} -lgmp -lmpfr ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(nbv_loop src/nbv_loop.cpp)
add_dependencies(nbv_loop ${catkin_EXPORTED_TARGETS})
target_link_libraries(nbv_loop ${catkin_LIBRARIES} -lgmp -lmpfr ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${Eigen_LIBRARIES})
