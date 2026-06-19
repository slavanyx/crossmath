# Shared numeric core. CMake scans Fortran `use` statements and orders the
# module compilation automatically, so source order here is irrelevant.
add_library(bladecam SHARED
  src/vec3.f90
  src/ruled_surface.f90
  src/flank_geom.f90
  src/flank_opt.f90
  src/kinematics.f90
  src/topp.f90
  src/bladecam_capi.f90)

set_target_properties(bladecam PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  Fortran_MODULE_DIRECTORY ${CMAKE_BINARY_DIR}/modules)

target_include_directories(bladecam PUBLIC ${CMAKE_BINARY_DIR}/modules)
