# Install script for directory: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/speech_pkg/msg" TYPE FILE FILES
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg"
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/speech_pkg/srv" TYPE FILE FILES
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv"
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv"
    "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Talker.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/speech_pkg/cmake" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/build/speech_pkg/catkin_generated/installspace/speech_pkg-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/include/speech_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/share/common-lisp/ros/speech_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/share/gennodejs/ros/speech_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/felice/command_interaction/ROS/hri_ws/devel/lib/python2.7/dist-packages/speech_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/felice/command_interaction/ROS/hri_ws/devel/lib/python2.7/dist-packages/speech_pkg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/build/speech_pkg/catkin_generated/installspace/speech_pkg.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/speech_pkg/cmake" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/build/speech_pkg/catkin_generated/installspace/speech_pkg-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/speech_pkg/cmake" TYPE FILE FILES
    "/home/felice/command_interaction/ROS/hri_ws/build/speech_pkg/catkin_generated/installspace/speech_pkgConfig.cmake"
    "/home/felice/command_interaction/ROS/hri_ws/build/speech_pkg/catkin_generated/installspace/speech_pkgConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/speech_pkg" TYPE FILE FILES "/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/package.xml")
endif()

