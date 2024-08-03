# Works with 3.14 and tested through 3.29
cmake_minimum_required(VERSION 3.14...3.29)

# Project name and a few useful settings. Other commands can pick up the results
project("genopLib")

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

message("[INFO] Configuring project ${PROJECT_NAME} successful!")
message("----------------------------------------")