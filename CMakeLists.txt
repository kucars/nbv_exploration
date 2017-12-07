cmake_minimum_required(VERSION 2.8.3)
project(nbv_exploration)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
#PCL fails in debug mode with c++11. Set to release mode
set(CMAKE_BUILD_TYPE Release)
include(CheckCCompilerFlag)

function(create_custom_compiler_flags name)
   foreach (flag ${ARGN})
      check_c_compiler_flag(${flag} ok)
      if (ok)
         if (cflags)
            set(cflags "${cflags} ${flag}")
         else ()
            set(cflags "${flag}")
         endif ()
      endif ()
   endforeach ()

   string(TOUPPER ${name} upper)
   set(CMAKE_C_FLAGS_${upper} "${cflags}" CACHE STRING "${name} C flags" FORCE)
   set(CMAKE_CXX_FLAGS_${upper} "${cflags}" CACHE STRING "${name} CXX flags" FORCE)
   mark_as_advanced(CMAKE_CXX_FLAGS_${upper} CMAKE_C_FLAGS_${upper})
endfunction ()

add_definitions(-D__PX4_ROS)
add_definitions(-D__EXPORT=)
add_definitions(-DMAVLINK_DIALECT=common)

create_custom_compiler_flags(-g -O2 -fstack-protector-strong --param ssp-buffer-size=4 -fstack-check -flto)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  gazebo_msgs
  gazebo_ros
  geometry_msgs
#  keyboard
  libmavconn
  mav_msgs
  message_generation
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL REQUIRED)

set( ROS_DISTRO $ENV{ROS_DISTRO} )
if(${ROS_DISTRO} STREQUAL "indigo" )
    find_package(fcl REQUIRED)
elseif(${ROS_DISTRO} STREQUAL "kinetic")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBFCL REQUIRED fcl)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message("Unsupported ROS Distro")
endif()

# OCTOMAP_OMP = enable OpenMP parallelization (experimental, defaults to OFF)
SET(OCTOMAP_OMP TRUE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{OCTOMAP_OMP})
  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
ENDIF(DEFINED ENV{OCTOMAP_OMP})
IF(OCTOMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OCTOMAP_OMP)

add_message_files(
  FILES
  IterationInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
    ${Eigen_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include #Adds my own custom class headers
    include/nbv_exploration
    include/control
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${OCTOMAP_LIBRARIES})

####
# Gazebo plugins
####

add_library(floating_sensor_position_plugin src/vehicle/floating_sensor_position_plugin.cpp)
target_link_libraries(floating_sensor_position_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  floating_sensor_position_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

####
# Executables
####

#add_executable(wall_follower src/wall_follower.cpp)
#add_dependencies(wall_follower ${catkin_EXPORTED_TARGETS})
#target_link_libraries(wall_follower ${catkin_LIBRARIES} -lgmp -lmpfr ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(nbv_main
  src/nbv_main.cpp
  src/nbv_history.cpp
  src/nbv_loop.cpp

  src/mapping_module.cpp

  src/model_profiler_base.cpp
  src/model_profiler_bounded_box.cpp
  src/model_profiler_circular_adaptive.cpp

  src/symmetry_detector.cpp

  src/termination_check_base.cpp
  src/termination_check_global_entropy_percent_diff.cpp
  src/termination_check_local_entropy_per_voxel.cpp
  src/termination_check_max_iterations.cpp
  src/termination_check_utility_threshold.cpp

  src/view_generator_base.cpp
  src/view_generator_frontier.cpp
  src/view_generator_nn.cpp
  src/view_generator_nn_adaptive.cpp
  src/view_generator_nn_frontier.cpp

  src/view_selecter_base.cpp
  src/view_selecter_ig.cpp
  src/view_selecter_ig_exp_distance.cpp
  src/view_selecter_point_density.cpp
  src/view_selecter_proposed.cpp
  src/view_selecter_proposed_ray_length.cpp

  src/control/vehicle_control_base.cpp
  src/control/vehicle_control_floating_sensor.cpp
  src/control/vehicle_control_iris.cpp

  src/culling/frustum_culling.cpp
  src/culling/occlusion_culling.cpp
  src/culling/voxel_grid_occlusion_estimation.cpp

  src/utilities/time_profiler.cpp

  src/lib/MeanShift/MeanShift.cpp
  )
add_dependencies(nbv_main
  ${catkin_EXPORTED_TARGETS}
  nbv_exploration_generate_messages_cpp
  )
target_link_libraries(nbv_main
    ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} 
    ${catkin_LIBRARIES} -lgmp -lmpfr
    ${Eigen_LIBRARIES}
    ${fcl_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${PCL_LIBRARIES}
    ${LIBFCL_LIBRARIES}
    )

add_executable(correct_laser_scan_and_depth src/correct_laser_scan_and_depth.cpp)
add_dependencies(correct_laser_scan_and_depth ${catkin_EXPORTED_TARGETS})
target_link_libraries(correct_laser_scan_and_depth ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_cloud_sync src/component_test/test_cloud_sync.cpp)
add_dependencies(test_cloud_sync ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cloud_sync ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(evaluate_coverage
  src/component_test/evaluate_coverage.cpp
  src/component_test/voxel_grid_occlusion_estimation.cpp
  )
add_dependencies(evaluate_coverage ${catkin_EXPORTED_TARGETS})
target_link_libraries(evaluate_coverage ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_executable(test_object_segmentation
  src/component_test/test_object_segmentation.cpp
  )
add_dependencies(test_object_segmentation ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_object_segmentation ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(test_octomap src/component_test/test_octomap.cpp)
add_dependencies(test_octomap ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_octomap ${catkin_LIBRARIES}  ${OCTOMAP_LIBRARIES})

add_executable(test_sensor_sync
  src/component_test/test_sensor_sync.cpp

  src/control/vehicle_control_base.cpp
  src/control/vehicle_control_floating_sensor.cpp

  src/mapping_module.cpp
  src/symmetry_detector.cpp
  src/lib/MeanShift/MeanShift.cpp
  src/utilities/time_profiler.cpp
  )
add_dependencies(test_sensor_sync ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_sensor_sync
	${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
	${catkin_LIBRARIES} -lgmp -lmpfr
	${Eigen_LIBRARIES}#
	${fcl_LIBRARIES}#
	${OCTOMAP_LIBRARIES}
	${PCL_LIBRARIES}
	)

add_executable(test_symmetry_detection
  src/component_test/test_symmetry_detection.cpp
  src/symmetry_detector.cpp
  src/lib/MeanShift/MeanShift.cpp
  src/utilities/time_profiler.cpp)
add_dependencies(test_symmetry_detection ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_symmetry_detection ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(pcd2ply src/utilities/pcd2ply.cpp)
add_dependencies(pcd2ply ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd2ply ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ply2pcd src/utilities/ply2pcd.cpp)
add_dependencies(ply2pcd ${catkin_EXPORTED_TARGETS})
target_link_libraries(ply2pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcd_cut_cuboid src/utilities/pcd_cut_cuboid.cpp)
add_dependencies(pcd_cut_cuboid ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_cut_cuboid ${catkin_LIBRARIES} ${PCL_LIBRARIES})
