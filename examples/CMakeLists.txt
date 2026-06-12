message("[INFO] Start configuring examples directory: ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(check_dummy check_dummy.cpp)
target_link_libraries(check_dummy orienteeringLib)

add_executable(solve_op solve_op.cpp)
target_link_libraries(solve_op orienteeringLib)

add_executable(benchmark_greedy benchmark_greedy.cpp)
target_link_libraries(benchmark_greedy orienteeringLib)

# NOTE: benchmark_lns / benchmark_grasp / benchmark_randomized_greedy were retired
# in Phase 0 of MIGRATION.md. They exercised the duplicate
# local_search::{LNSSolver,GraspSolver} (removed) or relied on the pre-refactor
# RandomizedGreedyConfig name. All are superseded by benchmark_all / benchmark_exact.

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark orienteeringLib)

add_executable(benchmark_all benchmark_all.cpp)
target_link_libraries(benchmark_all orienteeringLib)

add_executable(benchmark_exact benchmark_exact.cpp)
target_link_libraries(benchmark_exact orienteeringLib)

add_executable(benchmark_toptw_vehicles benchmark_toptw_vehicles.cpp)
target_link_libraries(benchmark_toptw_vehicles orienteeringLib)
