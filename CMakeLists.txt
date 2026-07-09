cmake_minimum_required(VERSION 3.16)
project(robstride_sdk VERSION 1.0.0 LANGUAGES CXX)

# -----------------------------------------------------------------
# C++ Standard
# -----------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------
# Build Options
# -----------------------------------------------------------------
option(ROBSTRIDE_BUILD_PYTHON "Build Python bindings via pybind11" ON)
option(ROBSTRIDE_BUILD_TESTS  "Build test executables"             ON)
option(ROBSTRIDE_BUILD_EXAMPLES "Build example executables"        ON)
option(ROBSTRIDE_BUILD_DOCS   "Build Doxygen documentation"        ON)
option(ROBSTRIDE_INSTALL      "Generate install targets"           ON)

# -----------------------------------------------------------------
# System Dependencies (non-ROS)
# -----------------------------------------------------------------
# The serial transport uses POSIX termios (kernel built-in); no libserial needed.
find_package(Threads REQUIRED)

# -----------------------------------------------------------------
# Core Library: robstride
# -----------------------------------------------------------------
add_library(robstride SHARED
  src/transport/robstride_transport.cpp
  src/motor/robstride_motor.cpp
  src/protocols/protocol_private.cpp
  src/protocols/protocol_mit.cpp
)
add_library(robstride::robstride ALIAS robstride)

target_include_directories(robstride
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robstride PRIVATE Threads::Threads)
set_target_properties(robstride PROPERTIES
  OUTPUT_NAME "robstride"
  VERSION     ${PROJECT_VERSION}
  SOVERSION   ${PROJECT_VERSION_MAJOR}
)

# -----------------------------------------------------------------
# Python Bindings (optional — requires pybind11)
# -----------------------------------------------------------------
if(ROBSTRIDE_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development QUIET)
  if(Python3_FOUND)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
      OUTPUT_VARIABLE _PB11_DIR ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    find_package(pybind11 QUIET HINTS "${_PB11_DIR}" PATHS "${_PB11_DIR}")
  endif()

  if(Python3_FOUND AND pybind11_FOUND)
    message(STATUS "[robstride_sdk] Building pyrobstride Python module")
    pybind11_add_module(pyrobstride python_bindings/pyrobstride.cpp)
    target_link_libraries(pyrobstride PRIVATE robstride::robstride)
    target_include_directories(pyrobstride PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  else()
    message(STATUS "[robstride_sdk] pybind11 not found — skipping Python bindings")
    message(STATUS "               Install: pip3 install pybind11")
  endif()
endif()

# -----------------------------------------------------------------
# Documentation (Doxygen)
# -----------------------------------------------------------------
if(ROBSTRIDE_BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
        add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
    else()
        message(WARNING "Doxygen need to be installed to generate the doxygen documentation")
    endif()
endif()

# -----------------------------------------------------------------
# Tests (optional)
# -----------------------------------------------------------------
# if(BUILD_TESTING)
if(ROBSTRIDE_BUILD_TESTS)
  # GTest unit tests
  find_package(GTest QUIET)
  if(GTest_FOUND)
    include(GoogleTest)
    enable_testing()
    add_executable(test_motor tests/test_motor.cpp)
    target_link_libraries(test_motor PRIVATE robstride::robstride GTest::gtest_main)
    target_include_directories(test_motor PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    gtest_discover_tests(test_motor)
    message(STATUS "[robstride_sdk] Building test_motor (GTest)")
    
    add_executable(test_protocol tests/test_protocol.cpp)
    target_link_libraries(test_protocol PRIVATE robstride::robstride GTest::gtest_main)
    target_include_directories(test_protocol PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    gtest_discover_tests(test_protocol)
    message(STATUS "[robstride_sdk] Building test_protocol (GTest)")
    
    add_executable(test_transport tests/test_transport.cpp)
    target_link_libraries(test_transport PRIVATE robstride::robstride GTest::gtest_main)
    target_include_directories(test_transport PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    gtest_discover_tests(test_transport)
    message(STATUS "[robstride_sdk] Building test_transport (GTest)")
  else()
    message(STATUS "[robstride_sdk] GTest not found — skipping test_motor")
  endif()

  # Smoke test (no GTest required)
  add_executable(test_vcan tests/test_vcan.cpp)
  target_link_libraries(test_vcan PRIVATE robstride::robstride)
  target_include_directories(test_vcan PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()


# -----------------------------------------------------------------
# Examples (optional)
# -----------------------------------------------------------------
if(ROBSTRIDE_BUILD_EXAMPLES)
  add_executable(example_mit_operation examples/cpp/mit/example_operation.cpp)
  target_link_libraries(example_mit_operation PRIVATE robstride::robstride)
  target_include_directories(example_mit_operation PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_mit_position examples/cpp/mit/example_position.cpp)
  target_link_libraries(example_mit_position PRIVATE robstride::robstride)
  target_include_directories(example_mit_position PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_mit_velocity examples/cpp/mit/example_velocity.cpp)
  target_link_libraries(example_mit_velocity PRIVATE robstride::robstride)
  target_include_directories(example_mit_velocity PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_mit_current examples/cpp/mit/example_current.cpp)
  target_link_libraries(example_mit_current PRIVATE robstride::robstride)
  target_include_directories(example_mit_current PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_mit_csp examples/cpp/mit/example_csp.cpp)
  target_link_libraries(example_mit_csp PRIVATE robstride::robstride)
  target_include_directories(example_mit_csp PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_private_operation examples/cpp/private/example_operation.cpp)
  target_link_libraries(example_private_operation PRIVATE robstride::robstride)
  target_include_directories(example_private_operation PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_private_operation_serial examples/cpp/private/example_operation_serial.cpp)
  target_link_libraries(example_private_operation_serial PRIVATE robstride::robstride)
  target_include_directories(example_private_operation_serial PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_private_csp examples/cpp/private/example_csp.cpp)
  target_link_libraries(example_private_csp PRIVATE robstride::robstride)
  target_include_directories(example_private_csp PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_private_position examples/cpp/private/example_position.cpp)
  target_link_libraries(example_private_position PRIVATE robstride::robstride)
  target_include_directories(example_private_position PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_private_velocity examples/cpp/private/example_velocity.cpp)
  target_link_libraries(example_private_velocity PRIVATE robstride::robstride)
  target_include_directories(example_private_velocity PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  add_executable(example_private_current examples/cpp/private/example_current.cpp)
  target_link_libraries(example_private_current PRIVATE robstride::robstride)
  target_include_directories(example_private_current PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  
  add_executable(example_benchmark examples/cpp/example_benchmark.cpp)
  target_link_libraries(example_benchmark PRIVATE robstride::robstride)
  target_include_directories(example_benchmark PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# -----------------------------------------------------------------
# CMake Package Config (enables find_package(robstride))
# -----------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/robstride-config-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/robstride-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/robstride-config.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/robstride"
)

# -----------------------------------------------------------------
# Install
# -----------------------------------------------------------------
if(ROBSTRIDE_INSTALL)
  install(TARGETS robstride
    EXPORT  robstride-targets
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
  )
  install(DIRECTORY include/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  )
  install(EXPORT robstride-targets
    FILE      robstride-targets.cmake
    NAMESPACE robstride::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/robstride"
  )
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/robstride-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/robstride-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/robstride"
  )

  if(ROBSTRIDE_BUILD_PYTHON AND TARGET pyrobstride)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getusersitepackages())"
      OUTPUT_VARIABLE _PY_SITE OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    install(TARGETS pyrobstride LIBRARY DESTINATION "${_PY_SITE}")
  endif()

  if(ROBSTRIDE_BUILD_TESTS)
    install(TARGETS test_vcan RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    if(TARGET test_motor)
      install(TARGETS test_motor RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    endif()
  endif()
endif()
