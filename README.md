# Orienteering Problem Solver Library

Lightweight C++ library and examples for solving multiple variants of Orienteering Problem (OP) instances.

## Requirements

- CMake 3.14+ (3.30+ tested)
- A C++17-compatible compiler
  - **Linux**: g++ or clang++
  - **macOS**: clang++ or g++ (via Homebrew)
  - **Windows**: GCC 13+ via MSYS2 (recommended) or MSVC 2022
- Git

## Configure & Build

### Linux

```bash
cmake -S . -B build                                      # configure
cmake --build build                                      # build
cmake --build build --target test_orienteeringLib -j4   # build tests
./build/tests/test_orienteeringLib                      # run tests
```

### Windows (Recommended: GCC via MSYS2)

#### Prerequisites

1. **Install CMake 3.14+** - Download from [cmake.org](https://cmake.org/download/)

2. **Install MSYS2 with GCC 13+**:
   - Download from [msys2.org](https://www.msys2.org/)
   - Run the installer and follow setup instructions
   - In MSYS2 terminal, run:
     ```bash
     pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake
     ```

#### Build Steps

```powershell
# Navigate to project directory
cd C:\path\to\orienteering-cpp

# Setup MSYS2 environment (one-time per session)
$env:PATH = "C:\msys64\mingw64\bin;C:\msys64\usr\bin;$env:PATH"

# Configure (no external dependencies required)
cmake -S . -B build -G"Unix Makefiles"

# Build the project
cmake --build build -j4

# Run tests
.\build\tests\test_orienteeringLib.exe
```

#### Alternative: Windows (MSVC 2022)

If you prefer MSVC, Boost serialization must be installed via vcpkg:

```powershell
# Install vcpkg and Boost (one-time setup)
git clone https://github.com/Microsoft/vcpkg.git C:\vcpkg
cd C:\vcpkg
.\bootstrap-vcpkg.bat
.\vcpkg install boost-serialization:x64-windows

# Configure and build
cd C:\path\to\orienteering-cpp
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build build --config Release -j4
.\build\tests\Release\test_orienteeringLib.exe
```

**Note**: MSVC build requires Boost library installation, making setup more complex.

### macOS with Homebrew

```bash
cmake -S . -B build
cmake --build build
cmake --build build --target test_orienteeringLib -j4
./build/tests/test_orienteeringLib
```

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
