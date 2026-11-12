cmake_minimum_required( VERSION 3.10 )
PROJECT( task4 )

IF ( "$ENV{ROBOCOMP}" STREQUAL "")
  MESSAGE(STATUS "ROBOCOMP variable not set. Using default: /home/osika/robocomp")
  SET (ENV{ROBOCOMP} "/home/osika/robocomp")
ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

SET(RC_COMPONENT_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "component base path" FORCE)
SET(RC_COMPONENT_DEVEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "component binary path" FORCE)
SET(RC_COMPONENT_INSTALL_PATH $ENV{ROBOCOMP} CACHE STRING "component install path")

INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

add_subdirectory(generated)

INSTALL(FILES etc/config DESTINATION ${RC_COMPONENT_INSTALL_PATH}/etc-default/ RENAME task4.conf )
