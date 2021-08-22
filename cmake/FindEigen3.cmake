find_path(EIGEN3_INCLUDE_DIRS Names Eigen/Dense
  HINTS
  ${EIGEN3_ROOT}
  ${EIGEN3_DIR}
  "C:/Eigen"
  /usr/include/eigen3
  /usr/local/include/eigen3
  )

find_package(PackageHandleStandardArgs REQUIRED)
find_package_handle_standard_args(EIGEN3 REQUIRED_VARS EIGEN3_INCLUDE_DIRS)
set(EIGEN3_LIBRARIES "")
set(EIGEN3_DLLS "")

