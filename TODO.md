# TODO lists

- [x] Add data & implement `parser` 
- [x] Implement package `model`
- [x] Implement package `solver`
- [ ] Config debugger


# CMake notes

```python
# # Only do these if this is the main project, and not if it is included through add_subdirectory
# # if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

# # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here

# # Let's ensure -std=c++xx instead of -std=g++xx
# set(CMAKE_CXX_EXTENSIONS OFF)

# # Let's nicely support folders in IDEs
# set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Testing only available if this is the main app
# Note this needs to be done in the main CMakeLists
# since it calls enable_testing, which must be in the
# main CMakeLists.
# OTHERWISE, DEACTIVATE THIS PART
# include(CTest)

# FetchContent added in CMake 3.11, downloads during the configure step
# FetchContent_MakeAvailable was added in CMake 3.14; simpler usage
# include(FetchContent)

# Accumulator library
# This is header only, so could be replaced with git submodules or FetchContent
# find_package(Boost REQUIRED)
# Adds Boost::boost

# # Formatting library
# FetchContent_Declare(
#   fmtlib
#   GIT_REPOSITORY https://github.com/fmtlib/fmt.git
#   GIT_TAG 5.3.0)
# FetchContent_MakeAvailable(fmtlib)
# # Adds fmt::fmt

# include(FetchContent)
# FetchContent_Declare(
#   ortools
#   GIT_REPOSITORY https://github.com/google/or-tools.git
#   GIT_TAG        v9.9)
# set(BUILD_DEPS ON)
# set(BUILD_DOC OFF)
# set(BUILD_SAMPLES OFF)
# set(BUILD_EXAMPLES OFF)
# FetchContent_MakeAvailable(ortools)
```

```python
# # Testing library
# FetchContent_Declare(
#   catch
#   GIT_REPOSITORY https://github.com/catchorg/Catch2.git
#   GIT_TAG v2.13.6)
# FetchContent_MakeAvailable(catch)
# # Adds Catch2::Catch2

# Tests need to be added as executables first
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS "*.cpp")
add_executable(run_tests ${TEST_SOURCES})

# I'm using C++17 in the test
target_compile_features(run_tests PRIVATE cxx_std_17)

# Should be linked to the main library, as well as the Catch2 testing library
target_link_libraries(run_tests PRIVATE orienteering_library Catch2::Catch2)

# If you register a test, then ctest and make test will run it.
# You can also run examples and check the output, as well.
add_test(NAME test_testlib COMMAND run_tests) # Command can be a target
add_test(NAME test_orienteering_model COMMAND run_tests)
# enable_testing()

```