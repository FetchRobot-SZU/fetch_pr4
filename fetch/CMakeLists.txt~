cmake_minimum_required(VERSION 2.8.3)
project(fetch)
##############################################
## -------- End of Zeon ------------------- ##
##############################################
set(CMAKE_CXX_FLAGS "-march=native -mno-avx -ggdb")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/src/cmake")

macro(CANONIFY_BOOL var)
  if (${var})
    set(${var} TRUE)
  else()
    set(${var} FALSE)
  endif()
endmacro()
#find_package(Eigen REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(CUDA QUIET)
CANONIFY_BOOL(CUDA_FOUND)
message(STATUS "CUDA found: " ${CUDA_FOUND})

find_package(OpenNI QUIET)
CANONIFY_BOOL(OPENNI_FOUND)
message(STATUS "OpenNI found: " ${OPENNI_FOUND})

find_package(libuvc QUIET)
CANONIFY_BOOL(libuvc_FOUND)
message(STATUS "libuvc found: ${libuvc_FOUND}")

find_package(RealSense QUIET)
CANONIFY_BOOL(REALSENSE_FOUND)
message(STATUS "librealsense found: ${REALSENSE_FOUND}")

find_package(PNG QUIET)
CANONIFY_BOOL(PNG_FOUND)
message(STATUS "libpng found: ${PNG_FOUND}")

OPTION(WITH_CUDA "Build with CUDA support?" ${CUDA_FOUND})
INCLUDE(${PROJECT_SOURCE_DIR}/src/cmake/UseOpenMP.cmake)
OPTION(WITH_REALSENSE "Build with Intel RealSense support?" ${REALSENSE_FOUND})

IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=corei7-avx -stdlib=libstdc++ -Wno-deprecated-declarations -Wno-unused-function")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -march=corei7-avx -stdlib=libstdc++")
ENDIF()

add_subdirectory(src/ORUtils)
add_subdirectory(src/ITMLib)
add_subdirectory(src/Utils)
add_subdirectory(src/Engine)
IF(WITH_CUDA)
  include_directories(${CUDA_INCLUDE_DIRS})
ELSE()
  add_definitions(-DCOMPILE_WITHOUT_CUDA)
ENDIF()
##############################################
## -------- End of Zeon ------------------- ##
##############################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  actionlib 
  actionlib_msgs
  cv_bridge
  message_filters
  moveit_msgs
  moveit_ros_planning_interface
  roslib
  sensor_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
DIRECTORY action
FILES 
  fetch.action
  ros_goal.action
)

#add_action_files(FILES
#  ros_goal.action
#)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}) # ${Eigen_INCLUDE_DIRS}

generate_messages(
DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime actionlib_msgs 
 
)


add_executable(fetch_server
  src/fetch_server.cpp
)
add_dependencies(fetch_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fetch_server
  ${catkin_LIBRARIES}
  Engine
  Utils
)

## Declare a C++ executable
add_executable(fetch_client
  src/fetch_client.cpp
)
add_dependencies(fetch_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fetch_client
  ${catkin_LIBRARIES}
)


## Declare a C++ executable
add_executable(arm
  src/arm.cpp
)
add_dependencies(arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm
  ${catkin_LIBRARIES}
)
