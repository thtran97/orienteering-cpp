message("[INFO] Start configuring tests directory: ${CMAKE_CURRENT_SOURCE_DIR}")

message("[INFO] Fetching GoogleTest ...")
FetchContent_Declare(
  gtest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.15.2
)
FetchContent_MakeAvailable(gtest)

message("[INFO] Add test executable ...")
file(GLOB_RECURSE test_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
message("[INFO] Test source files: ${test_SOURCE}")
add_executable(test_orienteeringLib ${test_SOURCE})
target_link_libraries(test_orienteeringLib PUBLIC
  gtest 
  orienteeringLib
  )
add_test(NAME test_orienteeringLib COMMAND test_orienteeringLib)
