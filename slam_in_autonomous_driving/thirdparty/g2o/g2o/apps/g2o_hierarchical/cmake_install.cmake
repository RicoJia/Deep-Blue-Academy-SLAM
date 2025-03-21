# Install script for directory: /home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libg2o_hierarchical.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/lib/libg2o_hierarchical.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so"
         OLD_RPATH "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libg2o_hierarchical.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/g2o/apps/g2o_hierarchical/backbone_tree_action.h;/usr/local/include/g2o/apps/g2o_hierarchical/edge_creator.h;/usr/local/include/g2o/apps/g2o_hierarchical/edge_labeler.h;/usr/local/include/g2o/apps/g2o_hierarchical/edge_types_cost_function.h;/usr/local/include/g2o/apps/g2o_hierarchical/g2o_hierarchical_api.h;/usr/local/include/g2o/apps/g2o_hierarchical/simple_star_ops.h;/usr/local/include/g2o/apps/g2o_hierarchical/star.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/g2o/apps/g2o_hierarchical" TYPE FILE FILES
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/backbone_tree_action.h"
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/edge_creator.h"
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/edge_labeler.h"
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/edge_types_cost_function.h"
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/g2o_hierarchical_api.h"
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/simple_star_ops.h"
    "/home/rico/slam_in_autonomous_driving/thirdparty/g2o/g2o/apps/g2o_hierarchical/star.h"
    )
endif()

