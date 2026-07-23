cmake_minimum_required(VERSION 3.16)

project(
  motion-core
  VERSION 0.1.0
  DESCRIPTION "Deterministic CPU-only autonomy control primitives"
  LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

option(MOTION_CORE_BUILD_EXAMPLES "Build the library usage example" ON)

add_library(motion_core
  src/models.cpp
  src/pid.cpp
  src/lqr.cpp
  src/path_tracking.cpp
  src/predictive_controller.cpp)
add_library(motion_core::motion_core ALIAS motion_core)

target_compile_features(motion_core PUBLIC cxx_std_17)
set_target_properties(motion_core PROPERTIES
  CXX_EXTENSIONS OFF
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME motion_core)
target_include_directories(motion_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(MSVC)
  target_compile_options(motion_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(motion_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(motion-core-cli src/cli.cpp)
target_link_libraries(motion-core-cli PRIVATE motion_core::motion_core)
target_compile_features(motion-core-cli PRIVATE cxx_std_17)
set_target_properties(motion-core-cli PROPERTIES OUTPUT_NAME motion-core)

if(MOTION_CORE_BUILD_EXAMPLES)
  add_executable(motion-core-example examples/library_example.cpp)
  target_link_libraries(motion-core-example PRIVATE motion_core::motion_core)
  target_compile_features(motion-core-example PRIVATE cxx_std_17)
endif()

install(TARGETS motion_core motion-core-cli
  EXPORT motion-core-targets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT motion-core-targets
  FILE motion-core-targets.cmake
  NAMESPACE motion_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motion-core)

configure_package_config_file(
  cmake/motion-core-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/motion-core-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motion-core)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/motion-core-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/motion-core-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/motion-core-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/motion-core)

if(BUILD_TESTING)
  add_executable(motion-core-tests tests/test_motion_core.cpp)
  target_link_libraries(motion-core-tests PRIVATE motion_core::motion_core)
  target_compile_features(motion-core-tests PRIVATE cxx_std_17)

  foreach(test_group nominal saturation invalid numerical determinism finite)
    add_test(NAME motion-core.${test_group}
      COMMAND motion-core-tests ${test_group})
  endforeach()
endif()
