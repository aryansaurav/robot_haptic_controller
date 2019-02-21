cmake_minimum_required(VERSION 2.8.3)
project(robot_haptic_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mathlib
  robotlib
  std_tools
  #allegro_lib
  sensor_msgs
  motion_generators
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  message_generation
  cmake_modules
  kuka_robot_model
  kdl_parser
  pcl_ros
  tek_patch
  ope_space_controller
  impedance_controller
  my_msgs
  visualization_msgs
  joint_limits_interface

  tf2
  tf2_ros
  tf2_eigen
  tf2_kdl
  tf2
  kdl_conversions
  eigen_conversions
moveit_msgs
moveit_core

)



find_package(Eigen REQUIRED)
find_package(YARP REQUIRED)
find_package(PCL REQUIRED)

MESSAGE("FOUND PCL_INCLUDE_DIR?: " ${PCL_INCLUDE_DIRS})
MESSAGE("FOUND YARP: " ${YARP_FOUND})
MESSAGE("FOUND Motiongenerators: " ${MotionGenerators_FOUND})

# Specifying release adds the -03 optimization tag. It's indeed faster
SET(CMAKE_BUILD_TYPE RELEASE)
#SET(CMAKE_BUILD_TYPE DEBUG)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS thread)
#MESSAGE("FOUND Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

# Enable C++11, and fpermissive code
#set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x ${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "-fpermissive -std=c++0x ${CMAKE_CXX_FLAGS}")
# Be verbose when running cmake
#set(CMAKE_VERBOSE_MAKEFILE ON)
# Debug about compile time
#add_definitions(-ftime-report)
#add_definitions(-v)

# Add Warnings
#add_definitions(-Wall)
# Find unreachable code ..
#add_definitions(-Wunreachable-code)
# Maximum optimization (takes more time: O3, O0 is minimum (normal))
#add_definitions(-O3)

# Add Memory report
#add_definitions(-fmem-report )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
##Starting from here, everything serves as a template for completing each new RTK module's CMakelists

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   customMsg1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 # generate_messages(
 #   DEPENDENCIES
 #   std_msgs  # Or other packages containing msgs
 # )

#  generate_dynamic_reconfigure_options(
#   cfg/params.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime tf
#  CATKIN_DEPENDS MathLib RobotLib StdTools
  DEPENDS system_lib Eigen
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}  ${YARP_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)
#  ${catkin_INCLUDE_DIRS}  ${YARP_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${PCL_INCLUDE_DIRS}

#${ARMADILLO_INCLUDE_DIRS}

## Declare a cpp library
#FILE(GLOB theSourceFiles src/*.cpp)
# add_library(${PROJECT_NAME}
#   ${theSourceFiles}
# )

FILE(GLOB theSourceFiles src/*.cpp)
FILE(GLOB theIncludeFiles include/*.h)
add_library(robot_haptic_controller ${theSourceFiles} ${theIncludeFiles})

# Not sure if it should stay here
add_dependencies(robot_haptic_controller my_msgs_gencfg)

## Declare a cpp executable
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

### This should probably be un commented...
## Specify libraries to link a library or executable target against
 target_link_libraries(robot_haptic_controller
   ${catkin_LIBRARIES}   ${YARP_LIBRARIES}  ${Eigen_LIBRARIES} ${PCL_LIBRARIES}
 )
#   ${catkin_LIBRARIES}   ${YARP_LIBRARIES}  ${Eigen_LIBRARIES} ${BOOST_LIBRARIES} ${PCL_LIBRARIES}
#${ARMADILLO_LIBRARIES}
# ${LAPACK_LIBRARIES}

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)




###################################
## RobotToolKit specific stuff  ###
###################################
# to comply with the file structure of RobotToolKit, you need to create symlinks 
# to any compiled modules in the RobotToolKit/module folder. This is automated below.
# just add the modules of this package as set(rtk_modules myAwesomeModule1 myAwesomeModule2)
# modules can be world or robot modules. Note that you need to specify your targets using 
# standard cmake/catkin procedures above. This is step is only for linking the build modules.
set(rtk_modules robot_haptic_controller)
find_package(catkin REQUIRED COMPONENTS rtk_pkg_tools)
include(${rtk_pkg_tools_SOURCE_DIR}/rtk_create_symlinks.cmake)
