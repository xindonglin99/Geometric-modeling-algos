#
# Allow using Find*.cmake files we provide
#
# CURRENT_LIST_DIR is the directory where the current file is located
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_LIST_DIR}")


set(CMAKE_CXX_STANDARD 14)

#
# Default build type is debug
#
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
    "Choose the type of build, options are: Debug Release RelWithDebInfo.")
endif(NOT CMAKE_BUILD_TYPE)

#
# What is the COMPILER?
#
#if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
if(APPLE)
    set(COMPILER_IS_CLANG 1)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(COMPILER_IS_GNU 1)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    set(COMPILER_IS_INTEL 1)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(COMPILER_IS_MSVC 1)
endif()


##
## Compiler flags -- Build type independant
##
if(COMPILER_IS_MSVC)
  
  ## Don't define min and max with Windows.h
  add_definitions("-DNOMINMAX")
  add_definitions("-DWIN32_LEAN_AND_MEAN")
  
  # Allow using printf, ...
  add_definitions("-D_SCL_SECURE_NO_WARNINGS")
  add_definitions("-D_CRT_SECURE_NO_WARNINGS")

  # Warnings
  add_definitions(/W3) 

  # stupid things
  add_definitions(/bigobj)

  # Parallel building of object files, 8 cores
  # When using this it is better to disable the solution level
  # paralelism of visual studio. That cannot be done using CMAKE.
  # One has to find "Maximum number of parallel project builds" under
  # Tools -> Options -> Projects and Solutions -> Build and Run
  # and set that to 1
  add_definitions(/MP8)
  
elseif(COMPILER_IS_GNU)
  
  add_definitions("-Wall -pedantic")
  
elseif(COMPILER_IS_CLANG)
  
  add_definitions("-DGL_SILENCE_DEPRECATION=1")
  
endif()

##
## Build type dependent compilation flags
## Also, Turn off cmake multi config generator
##
string( TOLOWER  "${CMAKE_BUILD_TYPE}" MINIMESH_BUILD_TYPE)

## RELEASE
if ("${MINIMESH_BUILD_TYPE}" STREQUAL "release")

  if(COMPILER_IS_MSVC)
    set(CMAKE_CONFIGURATION_TYPES Release)
  endif()
  
  add_definitions("-DMINIMESH_NO_DEBUG")
  add_definitions("-DNDEBUG")
  add_definitions("-DEIGEN_NO_DEBUG")

  if(COMPILER_IS_GNU)
    add_definitions("-O3 -funroll-loops -march=native -ffast-math")
  endif()
  
## RELWITHDEBINFO
elseif ("${MINIMESH_BUILD_TYPE}" STREQUAL "relwithdebinfo")
  
  if(COMPILER_IS_MSVC)
    set(CMAKE_CONFIGURATION_TYPES RelWithDebInfo)
  endif()

  add_definitions("-DMINIMESH_NO_DEBUG")
  add_definitions("-DNDEBUG")
  add_definitions("-DEIGEN_NO_DEBUG")
  
  if(COMPILER_IS_GNU)
    add_definitions("-g")
  endif()
  
## DEBUG
elseif("${MINIMESH_BUILD_TYPE}" STREQUAL "debug")

  if(COMPILER_IS_MSVC)
    set(CMAKE_CONFIGURATION_TYPES Debug)
  endif()
  
  add_definitions("-DDEBUG")

  if(COMPILER_IS_GNU)
    add_definitions("-O0 -g")
  endif()

# UNSUPPORTED BUILD TYPE
else()
  
  message(FATAL_ERROR "Build type ${MINIMESH_BUILD_TYPE} is not supported. \n"
    "Supported types are: debug, release, relwithdebinfo. \n" )
  
endif()

unset(MINIMESH_BUILD_TYPE)
## ===========================================================
##               NICE FILE HIERARCHY IN MSVC
## ===========================================================

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#macro(minimesh_folders_lib _target _rootdir )
macro(minimesh_folders_lib _target )

  set_property(TARGET ${_target} PROPERTY FOLDER "minimesh")
  source_group(src FILES ${ARGN})
  #source_group(TREE ${_rootdir} FILES ${ARGN})

endmacro()

## ===========================================================
##            MACHINERY FOR SETTINGS EXE DIRECTORIES
##            AND COPYING DLL's
## ===========================================================

set(MINIMESH_BINARY_DIR ${CMAKE_BINARY_DIR}/bin)

macro(minimesh_setup_exe_location _target)
  set_target_properties( ${_target}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY   ${MINIMESH_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${MINIMESH_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${MINIMESH_BINARY_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${MINIMESH_BINARY_DIR}
    )
endmacro()


macro(minimesh_set_required_dlls)
  if(WIN32)
    add_custom_target(COPY_DLLS)
    add_custom_command(TARGET COPY_DLLS
      PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E
      copy ${ARGN} ${MINIMESH_BINARY_DIR})
    set_property(TARGET COPY_DLLS
      PROPERTY FOLDER "CMakePredefinedTargets")
  endif()
endmacro()


## ===========================================================
##   Create a fake executable, i.e., copy an executable with a different name
## ===========================================================

# Input: target -- target name to be copied
#        newname -- the new name for the copied version
# on windows copies the excutable, on linux creates a symlink.
function(minimesh_fake_executable  __target __newname)

  # On windows cannot use cmake to create symlinks.
  # MKLINKS: with /H we get hardlinks which are weird
  # without /H we need admin permission which is tricky
  # Let's just brutally copy.
  if(WIN32)

    set(SYMLINK_LINK_NAME "\"$<TARGET_FILE_DIR:${__target}>/${__newname}.exe\"")
    set(SYMLINK_SOURCE_NAME "\"$<TARGET_FILE:${__target}>\"")
    add_custom_command(
      TARGET ${__target} POST_BUILD
      COMMAND 
      cp ${SYMLINK_SOURCE_NAME} ${SYMLINK_LINK_NAME}
    )

  # ON Non-windows, cmake can create a symbolic link.
  else() 

    set(SYMLINK_LINK_NAME "$<TARGET_FILE_DIR:${__target}>/${__newname}")
    set(SYMLINK_SOURCE_NAME "$<TARGET_FILE:${__target}>")
    add_custom_command(
      TARGET ${__target} POST_BUILD
      COMMAND 
      ${CMAKE_COMMAND} -E create_symlink ${SYMLINK_SOURCE_NAME} ${SYMLINK_LINK_NAME}
      )

  endif()

endfunction()

##  ===========================================================
##    Convert the new CMAKE imported policy dependency management
##    style to the old one
##  ===========================================================
# For returning a value
# https://stackoverflow.com/questions/22487215/cmake-function-parameter-and-return
# public vs private vs interface
# https://stackoverflow.com/questions/26243169/cmake-target-include-directories-meaning-of-scope
function(minimesh_get_imported_target_include_dirs  IMPORTED_TARGET OUTPUT_VAR)
  # For include dirs
  get_target_property(include_dirs ${IMPORTED_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
  # export to parent directory
  set(${OUTPUT_VAR} ${include_dirs} PARENT_SCOPE)
endfunction()

##  ===========================================================
##    Set policy is exsists
##  ===========================================================
# policy id should be CMPXXX (must include CMP)
macro(minimesh_set_policy_to_new policy_id )
  if(POLICY ${policy_id})
    cmake_policy(SET ${policy_id} NEW)
  endif()
endmacro()

