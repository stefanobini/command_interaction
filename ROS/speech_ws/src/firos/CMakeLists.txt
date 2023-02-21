cmake_minimum_required(VERSION 2.8.3)
project(firos)

## Requiered dependencies for firos
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # message_generation
)

# Generate message in the 'msg' folder
# add_message_files(
#   FILES
  # hri_msgs/Command.msg
  # hri_msgs/Gesture.msg
  # hri_msgs/Speech.msg
# )

## Generate messages which firos depends on
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

## Included directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})