# Orienteering Problem Solver Library

This is an Orienteering project using CMake, which aims to develop solvers for solving variants of Orienteering problems.

--------------------------------------------

The requirements are:

- CMake 3.11 or better; 3.14+ highly recommended.
- A C++17 compatible compiler
- The Boost libararies (header only part is fine)
- Git

To configure:

```bash
cmake -S . -B build
```

Add `-GNinja` if you have Ninja.

To build:

```bash
cmake --build build
```

To test (`--target` can be written as `-t` in CMake 3.15+):

```bash
cmake --build build --target test
```

To use an IDE, such as Xcode:

```bash
cmake -S . -B xbuild -GXcode
cmake --open xbuild
```

The CMakeLists show off several useful design patterns for CMake.
