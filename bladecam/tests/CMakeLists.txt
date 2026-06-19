add_executable(test_core test_core.f90)
target_link_libraries(test_core bladecam)
add_test(NAME core COMMAND test_core)

# Python pipeline smoke test (skips gracefully if numpy is unavailable)
find_program(PYTHON3 python3)
if(PYTHON3)
  add_test(NAME python
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_python.py)
  set_tests_properties(python PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")
endif()
