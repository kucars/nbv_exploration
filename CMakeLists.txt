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
  keyboard
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen REQUIRED)

add_service_files(
  FILES
  MappingSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS #src/include
  DEPENDS Eigen
  #LIBRARIES px4
  CATKIN_DEPENDS message_runtime
  #roscpp rospy std_msgs libmavconn
  #DEPENDS system_lib
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    include #Adds my own custom class headers
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${OCTOMAP_LIBRARIES})

#add_executable(wall_follower src/wall_follower.cpp)
#add_dependencies(wall_follower ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wall_follower ${catkin_LIBRARIES} -lgmp -lmpfr ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_library(view_generator src/view_generator.cpp)
add_library(view_selecter src/view_selecter.cpp)
add_library(frustum_culling   src/culling/frustum_culling.cpp)
add_library(occlusion_culling src/culling/occlusion_culling.cpp)
add_library(vox_grid_occ_est  src/culling/voxel_grid_occlusion_estimation)

add_executable(nbv_loop src/nbv_loop.cpp)
add_dependencies(nbv_loop ${catkin_EXPORTED_TARGETS})
target_link_libraries(nbv_loop
    view_generator
    view_selecter
    frustum_culling
    occlusion_culling
    vox_grid_occ_est
    ${catkin_LIBRARIES} -lgmp -lmpfr ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${Eigen_LIBRARIES})


add_executable(sensing_and_mapping src/sensing_and_mapping.cpp)
add_dependencies(sensing_and_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensing_and_mapping ${catkin_LIBRARIES} -lgmp -lmpfr ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} ${Eigen_LIBRARIES})

add_executable(teleop_uav src/keyboard_teleop.cpp)
add_dependencies(teleop_uav ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_uav ${catkin_LIBRARIES} )

add_executable(correct_laser_scan_and_depth src/correct_laser_scan_and_depth.cpp)
add_dependencies(correct_laser_scan_and_depth ${catkin_EXPORTED_TARGETS})
target_link_libraries(correct_laser_scan_and_depth ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_octomap src/test_octomap.cpp)
add_dependencies(test_octomap ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_octomap ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES})

