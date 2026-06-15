# Experimental external-solver KB backends.
# Only processed when the top-level option ENABLE_EXTERNAL_KB is ON.
#
# The backend sources are a planned port from kb_ls_cpp (see README.md). Until
# they are migrated, this module's job is to locate the heavy third-party solvers
# and fail fast with a helpful message if they are missing, so the toolchain can
# be validated before the port.

message("[INFO] Configuring experimental external_kb module ...")

find_library(CRYPTOMINISAT_LIB cryptominisat5)
find_path(CUDD_INCLUDE_DIR cudd.h PATH_SUFFIXES cudd)
find_library(CUDD_LIB cudd)
# CPLEX / CP Optimizer are usually located via a dedicated FindCPLEX.cmake; it is
# intentionally not required here yet (the CP backend is the last to be ported).

set(_external_kb_missing "")
if(NOT CRYPTOMINISAT_LIB)
    list(APPEND _external_kb_missing "CryptoMiniSat (libcryptominisat5)")
endif()
if(NOT CUDD_LIB OR NOT CUDD_INCLUDE_DIR)
    list(APPEND _external_kb_missing "CUDD (libcudd + cudd.h)")
endif()

if(_external_kb_missing)
    message(FATAL_ERROR
        "ENABLE_EXTERNAL_KB=ON but missing dependencies: ${_external_kb_missing}. "
        "Install them, or reconfigure with -DENABLE_EXTERNAL_KB=OFF (the default). "
        "See experimental/external_kb/README.md.")
endif()

message("[INFO] external_kb dependencies located; backend port from kb_ls_cpp pending (see README.md).")

# Once ported, the backend library will be defined here, e.g.:
#   file(GLOB_RECURSE external_kb_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
#   add_library(external_kb STATIC ${external_kb_SOURCE})
#   target_link_libraries(external_kb PUBLIC orienteeringLib
#                         ${CRYPTOMINISAT_LIB} ${CUDD_LIB})
#   target_include_directories(external_kb PUBLIC ${CUDD_INCLUDE_DIR})
