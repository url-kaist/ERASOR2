cmake_minimum_required(VERSION 2.8.3)
project(erasor2)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    nav_msgs
    roslaunch
    cv_bridge
    pcl_conversions
    pcl_ros
    geometry_msgs
    laser_geometry
    sensor_msgs
    visualization_msgs
    grid_map_msgs
    grid_map_ros
    grid_map_core
    grid_map_cv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  include
)

#add_executable(marker src/pub_marker.cpp ${unavlib_source})
#target_link_libraries(marker ${catkin_LIBRARIES})
#add_dependencies(marker erasor_generate_messages_cpp)

###############
# Online Map refinement
set(erasor2_src
    src/dataloader/dataloader.cpp
    src/erasor2/erasor_utils.cpp
    
)

add_executable(inheritance_test src/example/inheritance_test.cpp ${erasor2_src})
target_link_libraries(inheritance_test ${catkin_LIBRARIES} stdc++fs)
add_dependencies(inheritance_test erasor_generate_messages_cpp)

#add_executable(compare_complement src/utils/compare_complement.cpp ${unavlib_source})
#target_link_libraries(compare_complement ${catkin_LIBRARIES})
#add_dependencies(compare_complement erasor_generate_messages_cpp)
#
#add_executable(merge_map src/utils/merge_map.cpp ${unavlib_source})
#target_link_libraries(merge_map ${catkin_LIBRARIES})
#add_dependencies(merge_map erasor_generate_messages_cpp)
################


#add_executable(viz_kitti_map src/utils/viz_kitti_map.cpp src/offline_map_updater/src/erasor_utils.cpp)
#target_link_libraries(viz_kitti_map ${catkin_LIBRARIES})
#add_dependencies(viz_kitti_map erasor_generate_messages_cpp)
#
#add_executable(offline_map_updater src/offline_map_updater/main_kitti.cpp ${erasor_src})
#target_link_libraries(offline_map_updater ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(offline_map_updater erasor_generate_messages_cpp)

###################################### UTILS ##########################################
add_executable(accum_4dmos src/accum_4dmos.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(accum_4dmos ${catkin_LIBRARIES} stdc++fs)
add_dependencies(accum_4dmos erasor2_generate_messages_cpp)

add_executable(compare_map src/compare_map.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(compare_map ${catkin_LIBRARIES})
add_dependencies(compare_map erasor2_generate_messages_cpp)

add_executable(fill_removert_labels src/fill_removert_labels.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(fill_removert_labels ${catkin_LIBRARIES} stdc++fs)
add_dependencies(fill_removert_labels erasor2_generate_messages_cpp)
######################################################################################

set(mapgen_src
    src/erasor2/erasor_utils.cpp
    src/dataloader/dataloader.cpp
)

add_executable(mapgen src/mapgen/main.cpp ${mapgen_src})
target_link_libraries(mapgen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)
add_dependencies(mapgen erasor2_generate_messages_cpp)

set(erasor2_src
    src/erasor2/erasor_utils.cpp
    src/erasor2/erasor2.cpp
    src/dataloader/dataloader.cpp
)

add_executable(run_erasor2 src/erasor2/main.cpp ${erasor2_src})
target_link_libraries(run_erasor2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} stdc++fs)
target_compile_options(run_erasor2 PRIVATE ${OpenMP_CXX_FLAGS})
add_dependencies(run_erasor2 erasor2_generate_messages_cpp)


set(dataprocessor_src
  src/dataprocessor/visualizer.cpp
  src/dataprocessor/PointCloudProcessor.cpp
  src/dataprocessor/utility.cpp
  src/dataprocessor/BsplineSE3.cpp
)

# Add executable
add_executable(helipr_to_kitti src/dataprocessor/pointSaver.cpp ${dataprocessor_src})

# Link libraries
target_link_libraries(helipr_to_kitti 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
)

catkin_install_python(PROGRAMS
  src/dataprocessor/python/transformINStoLiDAR.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)