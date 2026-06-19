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

  # Fuzz the CAD importer with pathological geometry (graceful degradation)
  add_test(NAME cad_fuzz
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_cad_fuzz.py)
  set_tests_properties(cad_fuzz PROPERTIES ENVIRONMENT
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

  # Double-flank channel milling (exact planar channel + curved channel)
  add_test(NAME double_flank
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_double_flank.py)
  set_tests_properties(double_flank PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Multi-face blisk extraction (one rail pair per blade)
  add_test(NAME blisk_extraction
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_blisk_extraction.py)
  set_tests_properties(blisk_extraction PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Swept-envelope deviation (cross-station interference)
  add_test(NAME swept_envelope
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_swept_envelope.py)
  set_tests_properties(swept_envelope PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Tool+holder collision / gouge (signed distance)
  add_test(NAME collision
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_collision.py)
  set_tests_properties(collision PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Trimmed-face (edge-based) rail extraction
  add_test(NAME trimmed_face
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_trimmed_face.py)
  set_tests_properties(trimmed_face PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Stacked flank passes + roughing estimate
  add_test(NAME stacked_passes
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_stacked_passes.py)
  set_tests_properties(stacked_passes PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Point-milling (edge/fillet) and layered roughing toolpaths
  add_test(NAME toolpaths
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_toolpaths.py)
  set_tests_properties(toolpaths PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Physical consistency of feed / machine-limit / cycle-time coupling
  add_test(NAME physical_consistency
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_physical_consistency.py)
  set_tests_properties(physical_consistency PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # G-code post-processor (inverse-time feed realises the TOPP cycle)
  add_test(NAME postproc
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_postproc.py)
  set_tests_properties(postproc PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # GUI integration: every parameter / strategy / operation / chart path
  add_test(NAME gui_integration
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_gui_integration.py)
  set_tests_properties(gui_integration PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Alternate machine kinematics (table-table vs head-head) through the pipeline
  add_test(NAME kinematics
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_kinematics.py)
  set_tests_properties(kinematics PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # End-to-end acceptance: CAD blisk STEP -> extract -> optimise -> G-code
  add_test(NAME end_to_end
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_end_to_end.py)
  set_tests_properties(end_to_end PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # Workflow stage model: every step of the visual workflow renders finitely
  add_test(NAME workflow
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_workflow.py)
  set_tests_properties(workflow PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_SOURCE_DIR}/python;BLADECAM_LIB=$<TARGET_FILE:bladecam>")

  # C-ABI / boundary differential tests (each binding vs independent NumPy)
  add_test(NAME abi_differential
           COMMAND ${PYTHON3} ${CMAKE_SOURCE_DIR}/tests/test_abi_differential.py)
  set_tests_properties(abi_differential PROPERTIES ENVIRONMENT
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
    ${CMAKE_SOURCE_DIR}/core/src/collision.f90
    ${CMAKE_SOURCE_DIR}/core/src/bladecam_capi.f90)
endif()
