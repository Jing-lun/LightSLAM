cmake_minimum_required(VERSION 2.8.3)
project(light_slam)

SET(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# 显示 编译模式信息
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# 检查c++11或者 c++0x 编译支持  Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED
cv_bridge
roscpp
std_msgs
)
find_package(OpenCV 3.0 QUIET)

if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES light_slam
	CATKIN_DEPENDS 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_executable(light_slam src/ros_mono.cc)

add_library(lightslam SHARED
src/VisualOdometry.cc
src/RunSLAM.cc
src/VO.cc
)

target_link_libraries(light_slam
	lightslam
	${catkin_LIBRARIES}
  	${OpenCV_LIBRARIES}
)

add_executable(dataset src/run_dataset.cc)

target_link_libraries(dataset
	lightslam
	${catkin_LIBRARIES}
  	${OpenCV_LIBRARIES}
)

add_executable(vo src/test_dataset.cc)

target_link_libraries(vo
	lightslam
	${catkin_LIBRARIES}
  	${OpenCV_LIBRARIES}
)