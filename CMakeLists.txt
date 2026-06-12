# Works with 3.14 and tested through 3.29
cmake_minimum_required(VERSION 3.14...3.29)

# Use vcpkg toolchain if available (Windows)
if(WIN32 AND DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "vcpkg toolchain")
endif()

# Project name and a few useful settings. Other commands can pick up the results
project(orienteering-cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("----------------------------------------")
message("[INFO] Start configuring project ${PROJECT_NAME}: ${CMAKE_SOURCE_DIR}")

# FetchContent added in CMake 3.11, downloads during the configure step
# FetchContent_MakeAvailable was added in CMake 3.14; simpler usage
include(FetchContent)

# The compiled library code is here
add_subdirectory(src)

# The executable code is here
add_subdirectory(examples)

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
# OTHERWISE, DEACTIVATE THIS PART
option(PACKAGE_TESTS "Build the tests" ON)
message("[INFO] Build the tests: ${PACKAGE_TESTS}")
if(PACKAGE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Optional experimental module: external-solver-backed knowledge base
# (kb_ls_cpp heritage -- SAT/BDD/CP backends). OFF by default so the core library
# stays dependency-free. See experimental/external_kb/README.md.
option(ENABLE_EXTERNAL_KB "Build the experimental external-solver KB backends (needs CryptoMiniSat/CUDD/CPLEX)" OFF)
message("[INFO] External KB backends (ENABLE_EXTERNAL_KB): ${ENABLE_EXTERNAL_KB}")
if(ENABLE_EXTERNAL_KB)
    add_subdirectory(experimental/external_kb)
endif()

message("[INFO] Configuring project ${PROJECT_NAME} successful!")
message("----------------------------------------")