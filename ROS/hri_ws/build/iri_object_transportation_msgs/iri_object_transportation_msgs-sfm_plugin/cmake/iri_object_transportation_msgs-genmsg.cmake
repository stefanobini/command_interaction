# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "iri_object_transportation_msgs: 9 messages, 0 services")

set(MSG_I_FLAGS "-Iiri_object_transportation_msgs:/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(iri_object_transportation_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" "geometry_msgs/Wrench:geometry_msgs/Vector3:geometry_msgs/WrenchStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" "geometry_msgs/Wrench:iri_object_transportation_msgs/wrenchStampedArray:geometry_msgs/Vector3:geometry_msgs/WrenchStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" "geometry_msgs/Vector3:geometry_msgs/Twist:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" "geometry_msgs/Wrench:geometry_msgs/Vector3:geometry_msgs/WrenchStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" "geometry_msgs/Wrench:iri_object_transportation_msgs/wrenchStampedWithCoeff:geometry_msgs/Vector3:geometry_msgs/WrenchStamped:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" "iri_object_transportation_msgs/twistStamped:geometry_msgs/Vector3:geometry_msgs/Twist:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" NAME_WE)
add_custom_target(_iri_object_transportation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iri_object_transportation_msgs" "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" "std_msgs/Bool:geometry_msgs/Wrench:geometry_msgs/Vector3:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/WrenchStamped:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_cpp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(iri_object_transportation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(iri_object_transportation_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(iri_object_transportation_msgs_generate_messages iri_object_transportation_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_cpp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iri_object_transportation_msgs_gencpp)
add_dependencies(iri_object_transportation_msgs_gencpp iri_object_transportation_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iri_object_transportation_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_eus(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(iri_object_transportation_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(iri_object_transportation_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(iri_object_transportation_msgs_generate_messages iri_object_transportation_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_eus _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iri_object_transportation_msgs_geneus)
add_dependencies(iri_object_transportation_msgs_geneus iri_object_transportation_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iri_object_transportation_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_lisp(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(iri_object_transportation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(iri_object_transportation_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(iri_object_transportation_msgs_generate_messages iri_object_transportation_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_lisp _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iri_object_transportation_msgs_genlisp)
add_dependencies(iri_object_transportation_msgs_genlisp iri_object_transportation_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iri_object_transportation_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_nodejs(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(iri_object_transportation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(iri_object_transportation_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(iri_object_transportation_msgs_generate_messages iri_object_transportation_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iri_object_transportation_msgs_gennodejs)
add_dependencies(iri_object_transportation_msgs_gennodejs iri_object_transportation_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iri_object_transportation_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)
_generate_msg_py(iri_object_transportation_msgs
  "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/WrenchStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(iri_object_transportation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(iri_object_transportation_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(iri_object_transportation_msgs_generate_messages iri_object_transportation_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg" NAME_WE)
add_dependencies(iri_object_transportation_msgs_generate_messages_py _iri_object_transportation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iri_object_transportation_msgs_genpy)
add_dependencies(iri_object_transportation_msgs_genpy iri_object_transportation_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iri_object_transportation_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iri_object_transportation_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(iri_object_transportation_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(iri_object_transportation_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iri_object_transportation_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(iri_object_transportation_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(iri_object_transportation_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iri_object_transportation_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(iri_object_transportation_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(iri_object_transportation_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iri_object_transportation_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(iri_object_transportation_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iri_object_transportation_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(iri_object_transportation_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(iri_object_transportation_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
