# Install script for directory: /mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/src/ar_track_alvar/ar_track_alvar_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/msg" TYPE FILE FILES
    "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/src/ar_track_alvar/ar_track_alvar_msgs/msg/AlvarMarker.msg"
    "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/src/ar_track_alvar/ar_track_alvar_msgs/msg/AlvarMarkers.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/cmake" TYPE FILE FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/devel/include/ar_track_alvar_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/devel/share/roseus/ros/ar_track_alvar_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/devel/share/common-lisp/ros/ar_track_alvar_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/devel/share/gennodejs/ros/ar_track_alvar_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/devel/lib/python2.7/dist-packages/ar_track_alvar_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/devel/lib/python2.7/dist-packages/ar_track_alvar_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/cmake" TYPE FILE FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs/cmake" TYPE FILE FILES
    "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgsConfig.cmake"
    "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/build/ar_track_alvar/ar_track_alvar_msgs/catkin_generated/installspace/ar_track_alvar_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ar_track_alvar_msgs" TYPE FILE FILES "/mnt/c/bioe106a/bioe106a_final_project/106a-project/a1_soccer_vision/src/ar_track_alvar/ar_track_alvar_msgs/package.xml")
endif()

