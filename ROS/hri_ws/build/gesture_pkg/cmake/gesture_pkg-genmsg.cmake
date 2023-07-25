# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gesture_pkg: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Ivision_msgs:/home/felice/command_interaction/ROS/hri_ws/src/vision_msgs/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gesture_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" NAME_WE)
add_custom_target(_gesture_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gesture_pkg" "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(gesture_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gesture_pkg
)

### Generating Module File
_generate_module_cpp(gesture_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gesture_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gesture_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gesture_pkg_generate_messages gesture_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" NAME_WE)
add_dependencies(gesture_pkg_generate_messages_cpp _gesture_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gesture_pkg_gencpp)
add_dependencies(gesture_pkg_gencpp gesture_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gesture_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(gesture_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gesture_pkg
)

### Generating Module File
_generate_module_eus(gesture_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gesture_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gesture_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gesture_pkg_generate_messages gesture_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" NAME_WE)
add_dependencies(gesture_pkg_generate_messages_eus _gesture_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gesture_pkg_geneus)
add_dependencies(gesture_pkg_geneus gesture_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gesture_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(gesture_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gesture_pkg
)

### Generating Module File
_generate_module_lisp(gesture_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gesture_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gesture_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gesture_pkg_generate_messages gesture_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" NAME_WE)
add_dependencies(gesture_pkg_generate_messages_lisp _gesture_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gesture_pkg_genlisp)
add_dependencies(gesture_pkg_genlisp gesture_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gesture_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(gesture_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gesture_pkg
)

### Generating Module File
_generate_module_nodejs(gesture_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gesture_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gesture_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gesture_pkg_generate_messages gesture_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" NAME_WE)
add_dependencies(gesture_pkg_generate_messages_nodejs _gesture_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gesture_pkg_gennodejs)
add_dependencies(gesture_pkg_gennodejs gesture_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gesture_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(gesture_pkg
  "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gesture_pkg
)

### Generating Module File
_generate_module_py(gesture_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gesture_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gesture_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gesture_pkg_generate_messages gesture_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/felice/command_interaction/ROS/hri_ws/src/gesture_pkg/srv/CameraInfo.srv" NAME_WE)
add_dependencies(gesture_pkg_generate_messages_py _gesture_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gesture_pkg_genpy)
add_dependencies(gesture_pkg_genpy gesture_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gesture_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gesture_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gesture_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gesture_pkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(gesture_pkg_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET vision_msgs_generate_messages_cpp)
  add_dependencies(gesture_pkg_generate_messages_cpp vision_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gesture_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gesture_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gesture_pkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(gesture_pkg_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET vision_msgs_generate_messages_eus)
  add_dependencies(gesture_pkg_generate_messages_eus vision_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gesture_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gesture_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gesture_pkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(gesture_pkg_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET vision_msgs_generate_messages_lisp)
  add_dependencies(gesture_pkg_generate_messages_lisp vision_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gesture_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gesture_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gesture_pkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(gesture_pkg_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET vision_msgs_generate_messages_nodejs)
  add_dependencies(gesture_pkg_generate_messages_nodejs vision_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gesture_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gesture_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gesture_pkg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gesture_pkg_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(gesture_pkg_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET vision_msgs_generate_messages_py)
  add_dependencies(gesture_pkg_generate_messages_py vision_msgs_generate_messages_py)
endif()
