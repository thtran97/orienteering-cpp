message("[INFO] Configuring CP-SAT solver library ...")

add_library(cpsat_solver_lib STATIC cpsat_solver.cpp cpsat_optw_solver.cpp)

target_include_directories(cpsat_solver_lib PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(cpsat_solver_lib PUBLIC orienteeringLib ortools::ortools)

if(MSVC)
    target_compile_options(cpsat_solver_lib PRIVATE /W4 /FS)
else()
    target_compile_options(cpsat_solver_lib PRIVATE -Wall -Wextra)
endif()

message("[INFO] CP-SAT solver library configured.")
