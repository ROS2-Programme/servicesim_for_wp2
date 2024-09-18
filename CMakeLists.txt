cmake_minimum_required(VERSION 3.3)
project(servicesim_for_wp2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(ros_environment)

message(STATUS "${PROJECT_NAME}: ENV{ROS_VERSION} = \"$ENV{ROS_VERSION}\"")

find_package(ament_cmake
  COMPONENTS
#    rclcpp
    pcl_ros
#    visualization_msgs
)
if(ament_cmake_FOUND)
  message(STATUS "${PROJECT_NAME}: Configuring as ROS2 package")
  if(DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
    if(NOT ${ROS_VERSION} EQUAL 2)
      message(WARNING
        "${PROJECT_NAME}: ROS_VERSION not as expected (2): ${ROS_VERSION}")
    endif()
  else()
    set(ROS_VERSION 2)
    message(STATUS
      "${PROJECT_NAME}: Manually set ROS_VERSION to ${ROS_VERSION}")
  endif()
else()
  message(STATUS "${PROJECT_NAME}: Configuring as ROS (ROS1) package")
  if(NOT DEFINED ROS_VERSION)
    set(ROS_VERSION 1)
    message(STATUS
      "${PROJECT_NAME}: Manually set ROS_VERSION to ${ROS_VERSION}")
  else()
    if(NOT ${ROS_VERSION} EQUAL 1)
      message(WARNING
        "${PROJECT_NAME}: ROS_VERSION not as expected (1): ${ROS_VERSION}")
    endif()
  endif()
  find_package(catkin REQUIRED COMPONENTS
    gazebo_plugins
    xacro
    roscpp
    std_msgs
    geometry_msgs
    message_generation
  )
endif()

set(CMAKE_CXX_FLAGS "-DROS_VERSION=${ROS_VERSION}")

find_package( gazebo REQUIRED)
# find_package( ignition-math5 REQUIRED)

find_package( visualization_msgs REQUIRED)
if(${ROS_VERSION} EQUAL 2)
  find_package( rclcpp REQUIRED)
  find_package( rosidl_default_generators REQUIRED)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
set(trajectory_actor_plugin_name TrajectoryActorExtendedPlugin)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if(${ROS_VERSION} EQUAL 2)
  ament_export_libraries(
    ${trajectory_actor_plugin_name}
  )
  ament_export_include_directories(
    pcl_ros
    visualization_msgs
  )
  ament_export_dependencies(
    gazebo_plugins
    xacro
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
    visualization_msgs
  )
#  ament_package()
else()
  catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES servicesim_for_wp2
    LIBRARIES
      ${trajectory_actor_plugin_name}
    CATKIN_DEPENDS
      gazebo_plugins
      xacro
      geometry_msgs
      message_runtime
      roscpp
      std_msgs
#    DEPENDS system_lib
  )
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
#  ${IGNITION-MATH_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)



## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/servicesim_for_wp2.cpp
# )

add_library(velodyne_plugin SHARED
  src/velodyne_plugin.cc
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/servicesim_for_wp2_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(velodyne_plugin
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
)


#############################
## Trajectory Actor plugin ##
#############################

# Create the lib${trajectory_actor_plugin_name}.so library.
add_library(${trajectory_actor_plugin_name} SHARED
  src/TrajectoryActorPlugin.cc
)
if(${ROS_VERSION} EQUAL 2)
ament_target_dependencies(${trajectory_actor_plugin_name}
  gazebo
  rclcpp
  visualization_msgs
)
# rosidl_target_interfaces(${trajectory_actor_plugin_name} ${PROJECT_NAME}_msg_srv
#   "rosidl_typesupport_cpp"
# )
ament_package()
else()
target_link_libraries(${trajectory_actor_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
)
endif()

install(TARGETS ${trajectory_actor_plugin_name}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_servicesim_for_wp2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
