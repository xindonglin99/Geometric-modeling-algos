cmake_minimum_required(VERSION 3.11)
project(minimesh)


##
## Set C++ standard and import some macros
##
set(MINIMESH_SOURCE_DIR  ${PROJECT_SOURCE_DIR})
include("${PROJECT_SOURCE_DIR}/cmake/minimesh.cmake")

##
## Options
##
option(MINIMESH_WITH_EXTRAS "The contents of the private REPO are present" 0)

##
## CMAKE POLICIES
##
minimesh_set_policy_to_new(CMP0072) #related to how OpenGL is found
minimesh_set_policy_to_new(CMP0074) #FindPackage uses ..._ROOT



##
##  Find libraries that are shared among all projects
##  The results are XXX_INCLUDE_DIR, and XXX_LIBRARIES for 
##  library XXX.
##
# Eigen -- turn of eigen vectorization for starters
find_package(Eigen3 REQUIRED)
add_definitions("-DEIGEN_MAX_ALIGN_BYTES=0") 
add_definitions("-DEIGEN_DONT_VECTORIZE")
#
find_package(OpenGL)
# convert OpenGL::GL;OpenGL::GLU to OPENGL_INCLUDE_DIRS and OPENGL_LIBRARIES
minimesh_get_imported_target_include_dirs(OpenGL::GL OPENGL_INCLUDE_DIRS)
set(OPENGL_LIBRARIES OpenGL::GL OpenGL::GLU)
#
find_package(FREEGLUT)
#
find_package(GLUI)

if (FREEGLUT_FOUND AND GLUI_FOUND AND OPENGL_FOUND)
  set(MINIMESH_WITH_GUI  TRUE)
  message("** MINIMESH ** OpenGL libraries found, will compile GUI.")
else()
  set(MINIMESH_WITH_GUI  FALSE)
  message("** MINIMESH ** OpenGL libraries not found, will not compile GUI.")
endif()

# ================= Include directories

# Minimesh include directory (allows including as <minimesh/...>
include_directories("${PROJECT_SOURCE_DIR}/src")

# Include eigen
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

if(MINIMESH_WITH_GUI)
  include_directories(SYSTEM ${OPENGL_INCLUDE_DIRS})
  include_directories(SYSTEM ${FREEGLUT_INCLUDE_DIRS})
  include_directories(SYSTEM ${GLUI_INCLUDE_DIRS})
endif()

# ================= Setup a target to copy all the dll files we need
minimesh_set_required_dlls(
  ${FREEGLUT_DLLS}
  ${GLUI_DLLS}
  )

# ================= Add subprojects

# Core library
add_subdirectory(src/minimesh/core)

# Command line inteface
add_subdirectory(src/minimesh/cli)

# Graphical user interface
if(MINIMESH_WITH_GUI)
  add_subdirectory(src/minimesh/viz) # library
  add_subdirectory(src/minimesh/gui) # executable
endif()

# The extras code -- not part of the public repo.
if(MINIMESH_WITH_EXTRAS)
  add_subdirectory(extras)
endif()

# ================= Print some info for debugging

# Say if you found stuff
message("** MINIMESH ** EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
if(MINIMESH_WITH_GUI)
  message("** MINIMESH ** FREEGLUT_INCLUDE_DIRS: ${FREEGLUT_INCLUDE_DIRS}")
  message("** MINIMESH ** FREEGLUT_LIBRARIES: ${FREEGLUT_LIBRARIES}")
  message("** MINIMESH ** FREEGLUT_DLLS: ${FREEGLUT_DLLS}")
  message("** MINIMESH ** OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")
  message("** MINIMESH ** OPENGL_INCLUDE_DIRS: ${OPENGL_INCLUDE_DIRS}")
endif()
