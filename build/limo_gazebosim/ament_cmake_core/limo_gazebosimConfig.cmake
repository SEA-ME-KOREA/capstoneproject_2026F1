# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_limo_gazebosim_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED limo_gazebosim_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(limo_gazebosim_FOUND FALSE)
  elseif(NOT limo_gazebosim_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(limo_gazebosim_FOUND FALSE)
  endif()
  return()
endif()
set(_limo_gazebosim_CONFIG_INCLUDED TRUE)

# output package information
if(NOT limo_gazebosim_FIND_QUIETLY)
  message(STATUS "Found limo_gazebosim: 0.1.0 (${limo_gazebosim_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'limo_gazebosim' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${limo_gazebosim_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(limo_gazebosim_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${limo_gazebosim_DIR}/${_extra}")
endforeach()
