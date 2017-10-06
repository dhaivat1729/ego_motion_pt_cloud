# Install script for directory: /media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/build/subscriber_pkg/catkin_generated/installspace/subscriber_pkg.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/subscriber_pkg/cmake" TYPE FILE FILES
    "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/build/subscriber_pkg/catkin_generated/installspace/subscriber_pkgConfig.cmake"
    "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/build/subscriber_pkg/catkin_generated/installspace/subscriber_pkgConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/subscriber_pkg" TYPE FILE FILES "/media/dhaivat1729/e4187b00-2f8d-4c09-b475-5966c6563009/Rockwell_collins_fast_stereo/computation_optimization_workspace/src/subscriber_pkg/package.xml")
endif()

