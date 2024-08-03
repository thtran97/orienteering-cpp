message("[INFO] Start configuring examples directory: ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(check_dummy check_dummy.cpp)
target_link_libraries(check_dummy genopLib)
