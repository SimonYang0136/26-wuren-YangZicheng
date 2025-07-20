# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 3.0.2)
project(Project)

set(CATKIN_TOPLEVEL TRUE)

# search for catkin within the workspace
set(_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")
execute_process(COMMAND ${_cmd}
  RESULT_VARIABLE _retcode
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
  )
if(NOT _retcode EQUAL 0)
  message(FATAL_ERROR "Catkin was not found in the specified workspace.")
endif()

# include catkin from workspace or via find_package()
if(_out)
  set(catkin_EXTRAS_DIR ${CMAKE_SOURCE_DIR}/${_out}/cmake)
  # include all.cmake without add_subdirectory to let it operate in same scope
  include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  add_subdirectory("${CMAKE_SOURCE_DIR}/${_out}")

else()
  # use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
  # or CMAKE_PREFIX_PATH from the environment
  set(catkin_DIR $ENV{CATKIN_ENV_HOOK_WORKSPACE}/share/catkin/cmake)
  find_package(catkin REQUIRED)
  list(APPEND CMAKE_PREFIX_PATH ${catkin_PREFIX_PATH})
endif()

# list of catkin package directories
if(";" STREQUAL " ")
  set(catkin_DIRS ${CMAKE_SOURCE_DIR})
else()
  set(catkin_DIRS "")
  set(_start_root ${CMAKE_SOURCE_DIR})
  if(";" MATCHES "^/")
    set(_start_root "")
  endif()

  foreach(idir ${catkin_DIRS})
    set(idir "${_start_root}/${idir}")
    list(APPEND catkin_DIRS ${idir})
  endforeach()
endif()

# verify that package.xml and CMakeLists.txt exist
foreach(idir ${catkin_DIRS})
  if(NOT EXISTS ${idir}/package.xml)
    message(FATAL_ERROR "Could not find package.xml in ${idir}")
  endif()
  if(NOT EXISTS ${idir}/CMakeLists.txt)
    message(FATAL_ERROR "Could not find CMakeLists.txt in ${idir}")
  endif()
endforeach()

# call catkin_workspace()
if(catkin_DIRS)
  catkin_workspace()
endif()
