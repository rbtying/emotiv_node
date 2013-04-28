cmake_minimum_required(VERSION 2.8.3)
project(emotiv_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Data.msg
  Expressiv.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES emotiv_node
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(emotiv_node
#   src/${PROJECT_NAME}/emotiv_node.cpp
# )

set(CMAKE_OLD_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32 -std=c++0x -pthread")

FIND_LIBRARY(BOOST_FS boost_filesystem "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(BOOST_RE boost_regex "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(BOOST_SER boost_serialization "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(BOOST_SYS boost_system "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(EDK edk "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(EDK_UTIL edk_utils "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(HAL hal "${CMAKE_CURRENT_SOURCE_DIR}/lib/")
FIND_LIBRARY(USB usb-1.0 "${CMAKE_CURRENT_SOURCE_DIR}/lib/")

## Declare a cpp executable
# add_executable(emotiv_node_node src/emotiv_node_node.cpp)

add_library(JSON src/JSONValue.cpp src/JSON.cpp)
add_executable(emotiv_server src/main.cpp src/Emotiv.cpp src/die.cpp src/request.cpp src/serv.cpp)
target_link_libraries(emotiv_server ${BOOST_FS} ${BOOST_RE} ${BOOST_SER} ${BOOST_SYS} ${EDK} ${EDK_UTIL} ${HAL} ${USB} JSON)#) ${catkin_LIBRARIES})

#set(CMAKE_CXX_FLAGS "${CMAKE_OLD_FLAGS}")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(emotiv_node_node emotiv_node_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(emotiv_node_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS emotiv_node emotiv_node_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_emotiv_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
