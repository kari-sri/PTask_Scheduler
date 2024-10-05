# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ptask_scheduler_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ptask_scheduler_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ptask_scheduler_FOUND FALSE)
  elseif(NOT ptask_scheduler_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ptask_scheduler_FOUND FALSE)
  endif()
  return()
endif()
set(_ptask_scheduler_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ptask_scheduler_FIND_QUIETLY)
  message(STATUS "Found ptask_scheduler: 0.0.0 (${ptask_scheduler_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ptask_scheduler' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ptask_scheduler_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ptask_scheduler_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ptask_scheduler_DIR}/${_extra}")
endforeach()
