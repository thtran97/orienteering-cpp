# Define the library
# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.
project(orienteeringLib)

message("[INFO] Start configuring src directory: ${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB_RECURSE orienteeringLib_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

# Remove the old orienteering_model.cpp if it conflicts or we want to retire it
# For now, we keep it but it might need refactoring.
# Actually, let's keep everything but ensure Boost is handled correctly.

add_library(orienteeringLib SHARED ${orienteeringLib_SOURCE})

# Define include directories
target_include_directories(orienteeringLib PUBLIC ${CMAKE_SOURCE_DIR}/include) 

# Find Boost
find_package(Boost REQUIRED COMPONENTS serialization)
if (Boost_FOUND)
    message("[INFO] Boost version: ${Boost_VERSION_STRING}")
    target_link_libraries(orienteeringLib PUBLIC Boost::serialization)
else()
    message(FATAL_ERROR "Boost not found! 
    Please install Boost using these commands: \n
    sudo apt-get update \n
    sudo apt-get install libboost-all-dev
    ")
endif()

# Compiler options
if(MSVC)
    # MSVC warnings: /W4 is a good level for development, but you might want to adjust it for production.
    target_compile_options(orienteeringLib PRIVATE /W4)
else()
    # GCC/Clang warnings: -Wall -Wextra -Wpedantic are good for development.
    target_compile_options(orienteeringLib PRIVATE -Wall -Wextra -Wpedantic)
endif()
