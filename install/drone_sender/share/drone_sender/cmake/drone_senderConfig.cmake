# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_drone_sender_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED drone_sender_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(drone_sender_FOUND FALSE)
  elseif(NOT drone_sender_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(drone_sender_FOUND FALSE)
  endif()
  return()
endif()
set(_drone_sender_CONFIG_INCLUDED TRUE)

# output package information
if(NOT drone_sender_FIND_QUIETLY)
  message(STATUS "Found drone_sender: 0.0.0 (${drone_sender_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'drone_sender' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${drone_sender_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(drone_sender_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${drone_sender_DIR}/${_extra}")
endforeach()
