# Install script for directory: /home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/src/puzzlebot_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/puzzlebot_msgs/action" TYPE FILE FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/src/puzzlebot_msgs/action/GoToPose.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/puzzlebot_msgs/msg" TYPE FILE FILES
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseAction.msg"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseActionGoal.msg"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseActionResult.msg"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseActionFeedback.msg"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseGoal.msg"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseResult.msg"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/puzzlebot_msgs/msg/GoToPoseFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/puzzlebot_msgs/cmake" TYPE FILE FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/build/puzzlebot_msgs/catkin_generated/installspace/puzzlebot_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/include/puzzlebot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/roseus/ros/puzzlebot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/common-lisp/ros/puzzlebot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/share/gennodejs/ros/puzzlebot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/lib/python2.7/dist-packages/puzzlebot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/devel/lib/python2.7/dist-packages/puzzlebot_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/build/puzzlebot_msgs/catkin_generated/installspace/puzzlebot_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/puzzlebot_msgs/cmake" TYPE FILE FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/build/puzzlebot_msgs/catkin_generated/installspace/puzzlebot_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/puzzlebot_msgs/cmake" TYPE FILE FILES
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/build/puzzlebot_msgs/catkin_generated/installspace/puzzlebot_msgsConfig.cmake"
    "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/build/puzzlebot_msgs/catkin_generated/installspace/puzzlebot_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/puzzlebot_msgs" TYPE FILE FILES "/home/lftronic/Documents/code/ROS/puzzlebot_workspace_backup/puzzlebot_ws/src/puzzlebot_msgs/package.xml")
endif()

