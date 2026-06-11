message("[INFO] Start configuring examples directory: ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(check_dummy check_dummy.cpp)
target_link_libraries(check_dummy orienteeringLib)

add_executable(solve_op solve_op.cpp)
target_link_libraries(solve_op orienteeringLib)

add_executable(benchmark_greedy benchmark_greedy.cpp)
target_link_libraries(benchmark_greedy orienteeringLib)

add_executable(benchmark_randomized_greedy benchmark_randomized_greedy.cpp)
target_link_libraries(benchmark_randomized_greedy orienteeringLib)

add_executable(benchmark_lns benchmark_lns.cpp)
target_link_libraries(benchmark_lns orienteeringLib)

add_executable(benchmark_grasp benchmark_grasp.cpp)
target_link_libraries(benchmark_grasp orienteeringLib)

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark orienteeringLib)

add_executable(benchmark_all benchmark_all.cpp)
target_link_libraries(benchmark_all orienteeringLib)

add_executable(benchmark_exact benchmark_exact.cpp)
target_link_libraries(benchmark_exact orienteeringLib)
