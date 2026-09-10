cmake_minimum_required(VERSION 3.10)
project(pointcloud_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters features registration surface visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Threads REQUIRED)

add_executable(fuse_scans
    src/main.cpp
    src/fusion_pipeline.cpp
    src/fusion_session.cpp
    src/fusion_reconcile.cpp
    src/scan_state.cpp
    src/rig_geometry.cpp
    src/legend_panel.cpp
    src/scan_discovery.cpp
    src/visualizer.cpp
)
target_link_libraries(fuse_scans ${PCL_LIBRARIES} Threads::Threads)
