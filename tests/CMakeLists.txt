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

  # Regression tests for the bugs found in the adversarial audit
  add_test(NAME audit_regressions
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_audit_regressions.py)
  set_tests_properties(audit_regressions PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Physics-validation tests (developable exact case, optimization monotonicity)
  add_test(NAME validation
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_validation.py)
  set_tests_properties(validation PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # STEP/IGES import round-trip (skips gracefully without OpenCASCADE)
  add_test(NAME cad_step
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_cad_step.py)
  set_tests_properties(cad_step PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Automatic STEP rail extraction (skips gracefully without OpenCASCADE)
  add_test(NAME rail_extraction
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_rail_extraction.py)
  set_tests_properties(rail_extraction PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Measured-FRF stability lobes vs the modal model
  add_test(NAME frf_chatter
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_frf_chatter.py)
  set_tests_properties(frf_chatter PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")
endif()

# Compiler-warnings gate: catches the dead-code / out-of-bounds-subscript class
# of bugs automatically (audit bugs 3 and 6). Treats those specific warnings as
# errors without being flaky on cross-version -Wmaybe-uninitialized noise.
if(CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/wcheck)
  add_test(NAME warnings COMMAND ${CMAKE_Fortran_COMPILER}
    -fsyntax-only -Wall -Wextra
    -Werror=do-subscript -Werror=unused-variable -Werror=unused-dummy-argument
    -J ${CMAKE_BINARY_DIR}/wcheck
    ${CMAKE_SOURCE_DIR}/core/src/vec3.f90
    ${CMAKE_SOURCE_DIR}/core/src/ruled_surface.f90
    ${CMAKE_SOURCE_DIR}/core/src/flank_geom.f90
    ${CMAKE_SOURCE_DIR}/core/src/flank_opt.f90
    ${CMAKE_SOURCE_DIR}/core/src/kinematics.f90
    ${CMAKE_SOURCE_DIR}/core/src/topp.f90
    ${CMAKE_SOURCE_DIR}/core/src/chatter.f90
    ${CMAKE_SOURCE_DIR}/core/src/bladecam_capi.f90)
endif()
