set(TARGET_NAME minimeshcore)

##
## SOURCE FILES
##
set(SOURCE_FILES
  ##
  ## UTIL
  ##
  util/assert.hpp
  util/console_colors.hpp
  util/numbers.hpp
  util/foldertools.hpp
  util/foldertools.cpp
  util/macros.hpp
  util/disable_irrelevant_warnings.hpp
  util/enable_irrelevant_warnings.hpp
  util/bit_vector.hpp
  util/get_basefilename.hpp
  util/get_basefilename.cpp
  ##
  ## Manifold oriented half edge
  ##
  mohe/mesh_connectivity.hpp
  mohe/mesh_connectivity.cpp
  mohe/mesh_io.hpp
  mohe/mesh_io.cpp
  mohe/mesh_modifier.hpp
  mohe/mesh_modifier.cpp
 )

 
# create a library that the viewer can use to link to.
add_library(${TARGET_NAME} ${SOURCE_FILES})

##
## Create a folder structure inside the visual studio project
##
minimesh_folders_lib(${TARGET_NAME}  ${SOURCE_FILES})
