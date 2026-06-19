cmake_minimum_required(VERSION 3.18)
project(bladecam LANGUAGES Fortran)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_Fortran_FLAGS_RELEASE "-O2")
set(CMAKE_Fortran_FLAGS_DEBUG "-O0 -g -fcheck=all -fbacktrace")

add_subdirectory(core)

enable_testing()
add_subdirectory(tests)
