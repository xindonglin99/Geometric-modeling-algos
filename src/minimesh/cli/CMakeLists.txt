set(TARGET_NAME minimeshcli)


##
## SOURCE FILES
##
set(SOURCE_FILES
  main.cpp
)

 
add_executable(${TARGET_NAME}  ${SOURCE_FILES})
minimesh_setup_exe_location(${TARGET_NAME})
target_link_libraries(${TARGET_NAME}
  minimeshcore
)

##
## Create a folder structure inside the visual studio project
##
minimesh_folders_lib(${TARGET_NAME}  ${SOURCE_FILES})
