cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(erasor2 VERSION 0.1.0)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++17)

# NOTE: roscpp/rospy/message_generation/{std,nav,sensor,geometry,visualization}_msgs/
# cv_bridge/pcl_conversions/pcl_ros/laser_geometry/grid_map_msgs/grid_map_ros are
# all gone — runtime no longer touches ROS at all. We still pull in grid_map_core
# (the in-memory 2D grid data structure used by the ERASOR2 algorithm) and
# grid_map_cv (CvConverter::toImage, used by dilateAndErode). Both are catkin
# packages but neither requires a roscore at runtime.
find_package(catkin REQUIRED COMPONENTS
    grid_map_core
    grid_map_cv
)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)

# --- de-ROS migration deps -----------------------------------------------
# yaml-cpp replaces ros::param. Available as libyaml-cpp-dev on Ubuntu.
find_package(yaml-cpp REQUIRED)

# rerun_sdk replaces every visualization publisher. Pulled at configure
# time as a prebuilt zip; no system install needed.
include(FetchContent)
FetchContent_Declare(
    rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/download/0.21.0/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)
# -------------------------------------------------------------------------

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  src
  include
)

#add_executable(marker src/pub_marker.cpp ${unavlib_source})
#target_link_libraries(marker ${catkin_LIBRARIES})
#add_dependencies(marker erasor_generate_messages_cpp)

###############
# de-ROS shim sources (unused by legacy targets; built as a static lib so a
# linker error here surfaces immediately rather than later).
add_library(erasor2_deros STATIC
    src/erasor2/Config.cpp
    src/erasor2/RerunLogger.cpp
)
target_include_directories(erasor2_deros PUBLIC include)
target_link_libraries(erasor2_deros PUBLIC
    yaml-cpp
    rerun_sdk
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Online Map refinement
set(erasor2_src
    src/dataloader/dataloader.cpp
    src/erasor2/erasor_utils.cpp

)

set(mapgen_src
    src/erasor2/erasor_utils.cpp
    src/dataloader/dataloader.cpp
)

add_executable(mapgen src/mapgen/main.cpp ${mapgen_src})
target_link_libraries(mapgen erasor2_deros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)

set(erasor2_src
    src/erasor2/erasor_utils.cpp
    src/erasor2/erasor2.cpp
    src/dataloader/dataloader.cpp
)

add_executable(run_erasor2 src/erasor2/main.cpp ${erasor2_src})
target_link_libraries(run_erasor2 erasor2_deros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} stdc++fs)
target_compile_options(run_erasor2 PRIVATE ${OpenMP_CXX_FLAGS})

# --- Re-enabled secondary binaries (rerun-migrated) ----------------------
add_executable(accum_4dmos src/accum_4dmos.cpp src/erasor2/erasor_utils.cpp src/dataloader/dataloader.cpp)
target_link_libraries(accum_4dmos erasor2_deros ${catkin_LIBRARIES} ${PCL_LIBRARIES} stdc++fs)

add_executable(compare_map src/compare_map.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(compare_map erasor2_deros ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(fill_removert_labels src/fill_removert_labels.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(fill_removert_labels ${catkin_LIBRARIES} ${PCL_LIBRARIES} stdc++fs)

set(dataprocessor_src
  src/dataprocessor/visualizer.cpp
  src/dataprocessor/PointCloudProcessor.cpp
  src/dataprocessor/utility.cpp
  src/dataprocessor/BsplineSE3.cpp
)

add_executable(helipr_to_kitti src/dataprocessor/pointSaver.cpp ${dataprocessor_src})
target_link_libraries(helipr_to_kitti
  yaml-cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(merge_heliclouds src/dataprocessor/pointMerger.cpp ${dataprocessor_src})
target_link_libraries(merge_heliclouds
  yaml-cpp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
