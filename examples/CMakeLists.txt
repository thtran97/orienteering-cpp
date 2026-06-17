message("[INFO] Start configuring examples directory: ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(check_dummy check_dummy.cpp)
target_link_libraries(check_dummy orienteeringLib)

# NOTE: benchmark_lns / benchmark_grasp / benchmark_randomized_greedy were retired
# in Phase 0 of MIGRATION.md. They exercised the duplicate
# local_search::{LNSSolver,GraspSolver} (removed) or relied on the pre-refactor
# RandomizedGreedyConfig name. All are superseded by benchmark_all / benchmark_exact.



# Per-solver benchmarks (bench_utils + thin wrappers)
add_library(bench_utils STATIC bench_utils.cpp)
target_link_libraries(bench_utils orienteeringLib)
target_include_directories(bench_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

function(add_bench name)
    add_executable(bench_${name} bench_${name}.cpp)
    target_link_libraries(bench_${name} bench_utils)
endfunction()

add_bench(lns)
add_bench(greedy)
add_bench(rand_greedy)
add_bench(grasp_vns)
add_bench(ils09)
add_bench(ils_rr)
add_bench(mcts)
add_bench(forward_dp)
add_bench(dssr)
add_bench(pulse)
