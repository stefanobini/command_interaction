# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "speech_pkg: 12 messages, 3 services")

set(MSG_I_FLAGS "-Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg;-Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Imove_base_msgs:/home/felice/command_interaction/ROS/hri_ws/src/move_base_msgs/msg;-Imove_base_msgs:/home/felice/command_interaction/ROS/hri_ws/devel/.private/move_base_msgs/share/move_base_msgs/msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(speech_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" ""
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" "speech_pkg/Command"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" "speech_pkg/SpeechData"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:speech_pkg/IntentFeedback:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" ""
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" ""
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" "speech_pkg/SpeechData"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" ""
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" "actionlib_msgs/GoalID:speech_pkg/IntentActionResult:actionlib_msgs/GoalStatus:speech_pkg/IntentGoal:speech_pkg/IntentActionFeedback:speech_pkg/IntentFeedback:std_msgs/Header:speech_pkg/IntentActionGoal:speech_pkg/IntentResult"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" "actionlib_msgs/GoalID:speech_pkg/IntentGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" ""
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:speech_pkg/IntentResult:std_msgs/Header"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" "speech_pkg/Command"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" "speech_pkg/SpeechData"
)

get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" NAME_WE)
add_custom_target(_speech_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "speech_pkg" "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_msg_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)

### Generating Services
_generate_srv_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_srv_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)
_generate_srv_cpp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
)

### Generating Module File
_generate_module_cpp(speech_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(speech_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(speech_pkg_generate_messages speech_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_cpp _speech_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(speech_pkg_gencpp)
add_dependencies(speech_pkg_gencpp speech_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_msg_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)

### Generating Services
_generate_srv_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_srv_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)
_generate_srv_eus(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
)

### Generating Module File
_generate_module_eus(speech_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(speech_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(speech_pkg_generate_messages speech_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_eus _speech_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(speech_pkg_geneus)
add_dependencies(speech_pkg_geneus speech_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_msg_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)

### Generating Services
_generate_srv_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_srv_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)
_generate_srv_lisp(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
)

### Generating Module File
_generate_module_lisp(speech_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(speech_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(speech_pkg_generate_messages speech_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_lisp _speech_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(speech_pkg_genlisp)
add_dependencies(speech_pkg_genlisp speech_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_msg_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)

### Generating Services
_generate_srv_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_srv_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)
_generate_srv_nodejs(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
)

### Generating Module File
_generate_module_nodejs(speech_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(speech_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(speech_pkg_generate_messages speech_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_nodejs _speech_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(speech_pkg_gennodejs)
add_dependencies(speech_pkg_gennodejs speech_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_msg_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)

### Generating Services
_generate_srv_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_srv_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)
_generate_srv_py(speech_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv"
  "${MSG_I_FLAGS}"
  "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
)

### Generating Module File
_generate_module_py(speech_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(speech_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(speech_pkg_generate_messages speech_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentFeedback.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentAction.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionGoal.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentActionResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/devel/.private/speech_pkg/share/speech_pkg/msg/IntentResult.msg" NAME_WE)
add_dependencies(speech_pkg_generate_messages_py _speech_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(speech_pkg_genpy)
add_dependencies(speech_pkg_genpy speech_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS speech_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/speech_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(speech_pkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET move_base_msgs_generate_messages_cpp)
  add_dependencies(speech_pkg_generate_messages_cpp move_base_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(speech_pkg_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/speech_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(speech_pkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET move_base_msgs_generate_messages_eus)
  add_dependencies(speech_pkg_generate_messages_eus move_base_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(speech_pkg_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/speech_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(speech_pkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET move_base_msgs_generate_messages_lisp)
  add_dependencies(speech_pkg_generate_messages_lisp move_base_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(speech_pkg_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/speech_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(speech_pkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET move_base_msgs_generate_messages_nodejs)
  add_dependencies(speech_pkg_generate_messages_nodejs move_base_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(speech_pkg_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/speech_pkg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(speech_pkg_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET move_base_msgs_generate_messages_py)
  add_dependencies(speech_pkg_generate_messages_py move_base_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(speech_pkg_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
