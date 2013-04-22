cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(VTK 5.10 REQUIRED)
include(${VTK_USE_FILE})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

FILE(GLOB CV_FILES "src/cv/*")
FILE(GLOB CV_HEADERS "include/tracking_mapping/cv/*")

FILE(GLOB TM_FILES "src/tm/*")
FILE(GLOB TM_HEADERS "include/tracking_mapping/tm/*")

set(T_M_SOURCE_FILES
    src/main.cpp
    ${TM_FILES}
	${CV_FILES}

)

set(T_M_HEADER_FILES         
    ${TM_HEADERS}
	${CV_HEADERS}

)

# testing
rosbuild_add_executable(tracking src/tracking.cpp)
rosbuild_add_executable(tracking2 src/tracking2.cpp)
# video
rosbuild_add_executable(record_video src/record_video.cpp)
rosbuild_add_executable(broadcast_video src/broadcast_video.cpp)
# t_m
rosbuild_add_executable(tracking_mapping ${T_M_SOURCE_FILES} )
rosbuild_add_executable(tracking_mapping ${T_M_SOURCE_FILES} ${T_M_HEADER_FILES} )
# t_m init
rosbuild_add_executable(init_featrues src/init_featrues.cpp)
rosbuild_add_executable(init src/init.cpp)
# ROS/VTK

rosbuild_add_executable(show_vtk src/show_vtk.cpp)
target_link_libraries(show_vtk ${VTK_LIBRARIES})
