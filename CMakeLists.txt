cmake_minimum_required(VERSION 3.0.2)
project(bspline_rrt_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Debug
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ClampedCubicBSpline.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/CubicBezierSpline.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/PlannerCore.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/visualization.cpp     
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
)

## Flann
set(CMAKE_MODULE_PATH "/home/chelizi/ThirdPartyLib/flann/cmake")
find_package(Flann MODULE REQUIRED)

if(FLANN_FOUND)
  message(STATUS "Found Flann")
else()
  message(STATUS "Not found flann")
endif()

message(WARNING "${FLANN_INCLUDE_DIRS}")
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS 
    include/bspline_rrt_planner
  # LIBRARIES bspline_rrt_planner
  CATKIN_DEPENDS 
    roscpp
    nav_core
  # DEPENDS system_lib
)

include_directories(
  include/bspline_rrt_planner
  ${catkin_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_executable(test_neihbors src/TestNeighbors.cpp)
target_link_libraries(test_neihbors ${catkin_LIBRARIES} ${FLANN_LIBRARIES})

add_executable(test_bezier src/CubicBezierSpline.cpp src/TestCubicBezierSpline.cpp src/ClampedCubicBSpline.cpp)
target_link_libraries(test_bezier ${catkin_LIBRARIES})

add_library(bspline_rrt_planner ${SOURCES})
target_link_libraries(bspline_rrt_planner ${catkin_LIBRARIES} ${FLANN_LIBRARIES} ${OMPL_LIBRARIES}) 