# Orienteering Problem Solver Library

Lightweight C++ library and examples for solving multiple variants of Orienteering Problem (OP) instances.

## Requirements

- CMake 3.11+ (3.14+ recommended)
- A C++17-compatible compiler (g++, clang++)
- Boost (header-only parts are sufficient)
- Git

## Configure & build

```bash
cmake -S . -B build                 					# configure
cmake --build build                 					# build
cmake --build build --target test_orienteeringLib -j4   # build tests
./build/tests/test_orienteeringLib                      # run tests
```

Use `-GNinja` with `cmake` if you prefer Ninja builds.

## Repository layout

- `build/` — out-of-source CMake build directory (generated after configuration)
- `cmake/` — CMake modules and configuration files
- `data/` — problem instances and datasets (not included in repo, to be added by users)
- `docs/` — documentation and design notes
- `examples/` — example programs and small runners
- `include/` — public headers
- `src/` — library implementation and internal components
- `tests/` — unit tests and test runners

## Usage

See `examples/` for small runnable examples demonstrating setup and
execution.

## Contributing

Contributions are welcome. Please follow the existing CMake layout and
add small, focused changes with tests where appropriate.
