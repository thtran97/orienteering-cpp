# Orienteering Problem Solver Library

This is an Orienteering project using CMake.

## Configure & build

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

## Code structure

- `model` package defines key attributes of an OP model
- `solver` package defines various solvers for solving the OP model
- `utils` package defines several necessary tools

## How is an OP problem solved using this library ? 

Scripts will be written in `examples` folder. 
Users need to define a custom parser to read the input instance file, load important data and then create an OP model from that. Then, users can choose a solver, define parameters and launch the resolution procedure.