# Locate GLUI
#
# This module defines:
#
# TO USE IT MUST SET
#
# GLUI_ROOT:   Must be the folder having GL/glut.h
# or  GLUI_DIR
#
# GLUI_FOUND:       TRUE if the library is available
# GLUI_INCLUDE_DIRS:  Directory containing header files of GLUI
# GLUI_LIBRARIES:     Libraries to link against when using GLUI
# GLUI_DLLS:          DLLs required to load the library (only on Windows)
#
# Shayan Hoshyari 03/10/2018
#

if(WIN32)

  find_library(GLUI_LIBRARIES
    NAMES glui_static glui_staticd
    PATH_SUFFIXES lib
    PATHS
    ${GLUI_ROOT}
    ${GLUI_DIR}
    )
  
  set(GLUI_DLLS "")

  find_path(GLUI_INCLUDE_DIRS GL/glui.h
    PATH_SUFFIXES include
    PATHS
    ${GLUI_ROOT}
    ${GLUI_DIR}
    )

elseif(APPLE)


# COPID FROM https://github.com/rpavlik/cmake-modules/blob/master/FindGLUI.cmake

find_path(GLUI_INCLUDE_DIRS
    GL/glui.h
    HINTS
	${GLUI_DIR}/include
	${GLUI_ROOT}/include
    DOC
    "GLUI include directory")
    
find_library(GLUI_LIBRARIES
			NAMES
			glui
			HINTS
			${GLUI_DIR}/lib
			${GLUI_ROOT}/lib
			DOC
			"GLUI library")
else()
  
    find_library(GLUI_LIBRARIES
    NAMES glui gluid
    PATH_SUFFIXES lib
    PATHS
    ${GLUI_ROOT}
    ${GLUI_DIR}
    )
  
  set(GLUI_DLLS "")

  find_path(GLUI_INCLUDE_DIRS GL/glui.h
    PATH_SUFFIXES include
    PATHS
    ${GLUI_ROOT}
    ${GLUI_DIR}
    )

endif()


find_package(PackageHandleStandardArgs REQUIRED)
find_package_handle_standard_args(GLUI
  REQUIRED_VARS GLUI_INCLUDE_DIRS GLUI_LIBRARIES)

