cmake_minimum_required(VERSION 2.8.3)
project(iri_object_transportation_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ********************************************************************
#                 Add catkin additional components here
# ********************************************************************
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ********************************************************************
#           Add system and labrobotica dependencies here
# ********************************************************************
# find_package(<dependency> REQUIRED)

# ********************************************************************
#           Add topic, service and action definition here
# ********************************************************************
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  wrenchStampedArray.msg
  localForcesSFM.msg
  localForcesCoefficients.msg
  narrowPathMarkersArray.msg
  wrenchStampedWithCoeff.msg
  wrenchWithCoeffArray.msg
  twistStamped.msg
  twistStampedArray.msg
  explicit_information.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# ********************************************************************
#                 Add the dynamic reconfigure file
# ********************************************************************
# generate_dynamic_reconfigure_options(cfg/PeopleDetectionMsgs.cfg)

# ********************************************************************
#                 Add run time dependencies here
# ********************************************************************
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
# ********************************************************************
#            Add ROS and IRI ROS run time dependencies
# ********************************************************************
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
# ********************************************************************
#      Add system and labrobotica run time dependencies here
# ********************************************************************
#  DEPENDS
)

###########
## Build ##
###########

# ********************************************************************
#                   Add the include directories
# ********************************************************************
# include_directories(include)
# include_directories(${catkin_INCLUDE_DIRS})
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
# add_executable(${PROJECT_NAME} src/people_detection_msgs_alg.cpp src/people_detection_msgs_alg_node.cpp)

# ********************************************************************
#                   Add the libraries
# ********************************************************************
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})

# ********************************************************************
#               Add message headers dependencies
# ********************************************************************
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
# ********************************************************************
#               Add dynamic reconfigure dependencies
# ********************************************************************
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
