cmake_minimum_required(VERSION 2.8.3)
project(wisson_kn800)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
rospy
roscpp
std_msgs
std_srvs
geometry_msgs
message_generation
rviz
urdf
)



## Generate messages in the 'msg' folder
add_message_files(FILES
  kn800Command.msg
)

## Generate services in the 'srv' folder
#add_service_files(FILES
# 
#)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs 
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_msgs roscpp std_msgs
)

catkin_package(
    CATKIN_DEPENDS
  message_runtime
  std_msgs
#  INCLUDE_DIRS include
#  LIBRARIES origarm_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)


# if(CMAKE_SIZEOF_VOID_P EQUAL 8)
#         link_directories(${catkin_LIB_DIRS}  lib/lib64 lib/lib64/log4cplus lib/lib64/config lib/lib64/wissoncontroller)
# else()
#         link_directories(${catkin_LIB_DIRS}  lib/lib32)
# endif()


## Declare a C++ library
add_library(kn800Interface
  src/kn800Interface.cpp
)
add_dependencies(kn800Interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kn800Interface
  ${catkin_LIBRARIES}
)

install(TARGETS kn800Interface
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


add_executable(kn800Driver 
    src/kn800Driver.cpp
)
target_link_libraries(kn800Driver
  ${catkin_LIBRARIES} kn800Interface pthread
)
 
 
add_dependencies(kn800Driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

 

