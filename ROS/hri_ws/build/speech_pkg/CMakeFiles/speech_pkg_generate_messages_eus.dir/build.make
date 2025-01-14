# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/felice/command_interaction/ROS/hri_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/felice/command_interaction/ROS/hri_ws/build

# Utility rule file for speech_pkg_generate_messages_eus.

# Include the progress variables for this target.
include speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/progress.make

speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Gesture.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Command.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Speech.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SpeechData.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SystemHealth.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/ClassificationMSI.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Manager.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Classification.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Talker.l
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/manifest.l


/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Gesture.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Gesture.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Gesture.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating EusLisp code from speech_pkg/Gesture.msg"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Gesture.msg -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Command.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Command.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating EusLisp code from speech_pkg/Command.msg"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Speech.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Speech.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Speech.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Command.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating EusLisp code from speech_pkg/Speech.msg"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/Speech.msg -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SpeechData.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SpeechData.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating EusLisp code from speech_pkg/SpeechData.msg"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SystemHealth.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SystemHealth.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating EusLisp code from speech_pkg/SystemHealth.msg"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SystemHealth.msg -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/ClassificationMSI.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/ClassificationMSI.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/ClassificationMSI.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating EusLisp code from speech_pkg/ClassificationMSI.srv"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/ClassificationMSI.srv -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Manager.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Manager.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Manager.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating EusLisp code from speech_pkg/Manager.srv"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Manager.srv -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Classification.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Classification.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Classification.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg/SpeechData.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating EusLisp code from speech_pkg/Classification.srv"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Classification.srv -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Talker.l: /opt/ros/melodic/lib/geneus/gen_eus.py
/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Talker.l: /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Talker.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating EusLisp code from speech_pkg/Talker.srv"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/srv/Talker.srv -Ispeech_pkg:/home/felice/command_interaction/ROS/hri_ws/src/speech_pkg/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p speech_pkg -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv

/home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/manifest.l: /opt/ros/melodic/lib/geneus/gen_eus.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/felice/command_interaction/ROS/hri_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Generating EusLisp manifest code for speech_pkg"
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/geneus/cmake/../../../lib/geneus/gen_eus.py -m -o /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg speech_pkg std_msgs

speech_pkg_generate_messages_eus: speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Gesture.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Command.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/Speech.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SpeechData.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/msg/SystemHealth.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/ClassificationMSI.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Manager.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Classification.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/srv/Talker.l
speech_pkg_generate_messages_eus: /home/felice/command_interaction/ROS/hri_ws/devel/share/roseus/ros/speech_pkg/manifest.l
speech_pkg_generate_messages_eus: speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/build.make

.PHONY : speech_pkg_generate_messages_eus

# Rule to build all files generated by this target.
speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/build: speech_pkg_generate_messages_eus

.PHONY : speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/build

speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/clean:
	cd /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg && $(CMAKE_COMMAND) -P CMakeFiles/speech_pkg_generate_messages_eus.dir/cmake_clean.cmake
.PHONY : speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/clean

speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/depend:
	cd /home/felice/command_interaction/ROS/hri_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/felice/command_interaction/ROS/hri_ws/src /home/felice/command_interaction/ROS/hri_ws/src/speech_pkg /home/felice/command_interaction/ROS/hri_ws/build /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg /home/felice/command_interaction/ROS/hri_ws/build/speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : speech_pkg/CMakeFiles/speech_pkg_generate_messages_eus.dir/depend

