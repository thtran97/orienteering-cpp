message("[INFO] Start configuring tests directory: ${CMAKE_CURRENT_SOURCE_DIR}")

# Prefer a pre-installed GTest; fall back to FetchContent when not found.
message("[INFO] Looking for GoogleTest ...")
find_package(GTest CONFIG QUIET)
if(GTest_FOUND)
    message("[INFO] Using system GoogleTest: ${GTest_VERSION}")
    # Create unversioned alias so the link command below works unchanged.
    if(NOT TARGET gtest)
        add_library(gtest ALIAS GTest::gtest)
    endif()
else()
    message("[INFO] System GoogleTest not found – fetching from GitHub ...")
    FetchContent_Declare(
      gtest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG        v1.15.2
    )
    FetchContent_MakeAvailable(gtest)
endif()

message("[INFO] Add test executable ...")
file(GLOB_RECURSE test_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
message("[INFO] Test source files: ${test_SOURCE}")
add_executable(test_orienteeringLib ${test_SOURCE})
target_link_libraries(test_orienteeringLib PUBLIC
  gtest 
  orienteeringLib
  )
add_test(NAME test_orienteeringLib COMMAND test_orienteeringLib)

# Link CP-SAT solver and enable its tests when OR-Tools is available.
if(CPSAT_AVAILABLE)
    target_link_libraries(test_orienteeringLib PUBLIC cpsat_solver_lib)
    target_compile_definitions(test_orienteeringLib PRIVATE ENABLE_CPSAT_SOLVER=1)
    message("[INFO] CP-SAT solver tests enabled.")
endif()
