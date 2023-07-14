# Install script for directory: /home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/felice/command_interaction/ROS/hri_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iri_object_transportation_msgs/msg" TYPE FILE FILES
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedArray.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesSFM.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/localForcesCoefficients.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/narrowPathMarkersArray.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchStampedWithCoeff.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/wrenchWithCoeffArray.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStamped.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/twistStampedArray.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/msg/explicit_information.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iri_object_transportation_msgs/cmake" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/build/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/catkin_generated/installspace/iri_object_transportation_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/include/iri_object_transportation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/iri_object_transportation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/share/common-lisp/ros/iri_object_transportation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/share/gennodejs/ros/iri_object_transportation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/felice/command_interaction/ROS/hri_ws/devel/lib/python2.7/dist-packages/iri_object_transportation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/lib/python2.7/dist-packages/iri_object_transportation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/build/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/catkin_generated/installspace/iri_object_transportation_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iri_object_transportation_msgs/cmake" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/build/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/catkin_generated/installspace/iri_object_transportation_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iri_object_transportation_msgs/cmake" TYPE FILE FILES
    "/home/felice/command_interaction/ROS/hri_ws/build/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/catkin_generated/installspace/iri_object_transportation_msgsConfig.cmake"
    "/home/felice/command_interaction/ROS/hri_ws/build/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/catkin_generated/installspace/iri_object_transportation_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iri_object_transportation_msgs" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/src/iri_object_transportation_msgs/iri_object_transportation_msgs-sfm_plugin/package.xml")
endif()

