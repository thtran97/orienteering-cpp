# Define the library
# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.
project(genopLib)

message("[INFO] Start configuring src directory: ${CMAKE_CURRENT_SOURCE_DIR}")
file(GLOB_RECURSE genopLib_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(genopLib SHARED ${genopLib_SOURCE})

# Define include directories
target_include_directories(genopLib PUBLIC ${CMAKE_SOURCE_DIR}/include) 

# For all projects
message("[INFO] Find Boost ...")
find_package(Boost REQUIRED) 
if (Boost_FOUND)
    message("[INFO] Boost version: ${Boost_VERSION_STRING}")
else()
    message(WARNING "Boost not found! 
    Please install Boost using these commands: \n
    sudo apt-get update \n
    sudo apt-get install libboost-all-dev
    ")
endif()
