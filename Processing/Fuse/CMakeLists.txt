cmake_minimum_required(VERSION 3.10)
project(pointcloud_fusion)

# Refuse in-source builds: they scatter CMakeCache.txt and generated Makefiles
# (which hold absolute paths) through the source tree, and those break as soon
# as the folder is copied or cloned somewhere else.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Build in a separate directory: mkdir build && cd build && cmake ..\n"
                        "Delete the CMakeCache.txt and CMakeFiles/ just created in ${CMAKE_SOURCE_DIR}.")
endif()

# Default to an optimised build. A plain `cmake ..` otherwise compiles with no
# optimisation at all, and PCL's registration code (ICP, correspondence search,
# the point-to-plane solver) is header-only, so it is compiled here and inherits
# that. Measured on a 28-scan session: 83 s unoptimised, 15 s Release, identical
# output. Pass -DCMAKE_BUILD_TYPE=Debug when you need a debugger.
#
# Don't add -march=native: the distro PCL libraries are built without it, and
# mixing the two gives Eigen types different alignment on either side of the
# library boundary, which can crash.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.10 REQUIRED COMPONENTS common io filters features registration surface visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Threads REQUIRED)

# jsoncpp reads and writes the poses / QC files (--save-poses, --load-poses,
# --qc). Ubuntu: sudo apt install libjsoncpp-dev
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})

# Stamped into every poses / QC file, so a result can be traced to the build
# that produced it. Taken at configure time - re-run cmake after committing.
execute_process(COMMAND git describe --always --dirty
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                OUTPUT_VARIABLE FUSE_GIT_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT FUSE_GIT_VERSION)
    set(FUSE_GIT_VERSION "unknown")
endif()
add_definitions(-DFUSE_VERSION="${FUSE_GIT_VERSION}")

add_executable(fuse_scans
    src/main.cpp
    src/fusion_pipeline.cpp
    src/fusion_session.cpp
    src/fusion_reconcile.cpp
    src/scan_state.cpp
    src/rig_geometry.cpp
    src/legend_panel.cpp
    src/scan_discovery.cpp
    src/tag_alignment.cpp
    src/visualizer.cpp
    src/pose_io.cpp
)
target_link_libraries(fuse_scans ${PCL_LIBRARIES} ${JSONCPP_LIBRARIES} Threads::Threads)
