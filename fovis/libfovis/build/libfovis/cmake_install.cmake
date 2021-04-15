# Install script for directory: /home/mxr/catkin_ws3/src/fovis/libfovis/libfovis

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mxr/catkin_ws3/src/fovis/libfovis/build/lib/pkgconfig/libfovis.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/usr/local/lib")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/mxr/catkin_ws3/src/fovis/libfovis/build/lib/libfovis.so.1"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/build/lib/libfovis.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libfovis.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "::::::::::::::"
           NEW_RPATH "/usr/local/lib")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/fovis" TYPE FILE FILES
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/fovis.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/visual_odometry.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/motion_estimation.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/frame.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/keypoint.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/depth_source.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/camera_intrinsics.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/primesense_depth.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/grid_filter.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/sad.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/internal_utils.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/intensity_descriptor.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/pyramid_level.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/feature_match.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/feature_matcher.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/refine_feature_match.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/stereo_calibration.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/stereo_depth.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/stereo_frame.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/depth_image.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/stereo_disparity.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/rectification.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/stereo_rectify.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/options.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/tictoc.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/refine_motion_estimate.hpp"
    "/home/mxr/catkin_ws3/src/fovis/libfovis/libfovis/initial_homography_estimation.hpp"
    )
endif()

