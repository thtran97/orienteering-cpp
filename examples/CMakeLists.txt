add_executable(check_dummy check_dummy.cpp)
target_compile_features(check_dummy PRIVATE cxx_std_17)
target_link_libraries(check_dummy PRIVATE orienteering_library fmt::fmt)
