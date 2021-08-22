set(TARGET_NAME minimeshviz)

##
## SOURCE FILES
##
set(SOURCE_FILES
  opengl_headers.hpp
  mesh_buffer.hpp
  mesh_buffer.cpp
  mesh_viewer.hpp
  mesh_viewer.cpp
  util/glui_slider.hpp
  util/glui_slider.cpp
)

 
# create a library that the viewer can use to link to.
add_library(${TARGET_NAME} ${SOURCE_FILES})

##
## Create a folder structure inside the visual studio project
##
minimesh_folders_lib(${TARGET_NAME}  ${SOURCE_FILES})
