# Install script for directory: /home/linzp/catkin_op/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/linzp/catkin_op/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/linzp/catkin_op/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/linzp/catkin_op/install" TYPE PROGRAM FILES "/home/linzp/catkin_op/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/linzp/catkin_op/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/linzp/catkin_op/install" TYPE PROGRAM FILES "/home/linzp/catkin_op/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/linzp/catkin_op/install/setup.bash;/home/linzp/catkin_op/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/linzp/catkin_op/install" TYPE FILE FILES
    "/home/linzp/catkin_op/build/catkin_generated/installspace/setup.bash"
    "/home/linzp/catkin_op/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/linzp/catkin_op/install/setup.sh;/home/linzp/catkin_op/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/linzp/catkin_op/install" TYPE FILE FILES
    "/home/linzp/catkin_op/build/catkin_generated/installspace/setup.sh"
    "/home/linzp/catkin_op/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/linzp/catkin_op/install/setup.zsh;/home/linzp/catkin_op/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/linzp/catkin_op/install" TYPE FILE FILES
    "/home/linzp/catkin_op/build/catkin_generated/installspace/setup.zsh"
    "/home/linzp/catkin_op/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/linzp/catkin_op/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/linzp/catkin_op/install" TYPE FILE FILES "/home/linzp/catkin_op/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/linzp/catkin_op/build/gtest/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench/dynamixel_workbench/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/navigation/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/realsense-ros/realsense2_description/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench-msgs/dynamixel_workbench_msgs/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/aruco_ros/aruco_msgs/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/cartographer_ros_msgs/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench/dynamixel_workbench_toolbox/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench/dynamixel_workbench_single_manager/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench/dynamixel_workbench_single_manager_gui/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/feetech/feetechlib/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/map_server/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/ddynamic_reconfigure/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/aruco_ros/aruco/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/darknet_ros/darknet_ros_msgs/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench/dynamixel_workbench_controllers/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/dynamixel-workbench/dynamixel_workbench_operators/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/darknet_ros/darknet_ros/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/ros_hand_gesture_recognition/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/rplidar_ros/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/sort_track/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/rfid/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/aruco_ros/aruco_ros/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/odom/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/realsense-ros/realsense2_camera/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/amcl/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/fake_localization/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/laser_filters-noetic-devel/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/pointcloud_to_laserscan-lunar-devel/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/feetech_controls/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/main_run/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/urdf_tutorial/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/voxel_grid/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/costmap_2d/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/nav_core/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/base_local_planner/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/carrot_planner/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/clear_costmap_recovery/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/dwa_local_planner/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/move_slow_and_clear/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/navfn/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/global_planner/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/rotate_recovery/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/navigation/move_base/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/zed-ros-wrapper/zed-ros-interfaces/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/zed-ros-wrapper/zed_nodelets/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/zed-ros-wrapper/zed_ros/cmake_install.cmake")
  include("/home/linzp/catkin_op/build/zed-ros-wrapper/zed_wrapper/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/linzp/catkin_op/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
