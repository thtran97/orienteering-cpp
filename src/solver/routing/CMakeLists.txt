message("[INFO] Configuring RoutingModel-based OPTW solver library ...")

add_library(routing_solver_lib STATIC routing_optw_solver.cpp)

target_include_directories(routing_solver_lib PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(routing_solver_lib PUBLIC orienteeringLib ortools::ortools)

if(MSVC)
    target_compile_options(routing_solver_lib PRIVATE /W4 /FS)
else()
    target_compile_options(routing_solver_lib PRIVATE -Wall -Wextra)
endif()

message("[INFO] RoutingModel-based OPTW solver library configured.")
