# CLAUDE.md — orienteering-cpp

This file is a guide for AI assistants working on this codebase. Read it before making any changes.

---

## 1. Project Purpose and Scope

`orienteering-cpp` is a C++17 research library for solving the **Orienteering Problem (OP)** and 8 of its variants using heuristic and metaheuristic algorithms. The OP asks: given a graph of nodes with rewards and a time budget, find a route starting and ending at depots that maximizes collected reward without exceeding the budget.

Supported variants: OP, OPTW, TOP, TOPTW, TDOP, TDOPTW, MCTOPMTW, SingleSat, TTDP.

This is **not** an application or service. It is a static library (`orienteeringLib`) with example binaries for benchmarking and a test suite. There is no UI, no server, no database.

---

## 2. Repository Structure

```
orienteering-cpp/
├── CMakeLists.txt           # Root: C++17, FetchContent for GoogleTest, subdirs
├── include/                 # All public headers (library API lives here)
│   ├── core/                # Primitive types, constants, RNG, routing utils
│   │   ├── types.h          # NodeId, Reward, Time, TimeWindow, enums
│   │   ├── constants.h      # DEFAULT_SEED=42, DEFAULT_TIMELIMIT_SECONDS, etc.
│   │   ├── random.h         # oplib::utils::Random (mt19937 wrapper)
│   │   └── routing_utils.h  # oplib::utils::RoutingUtils (time-dependent helpers)
│   ├── io/                  # Parser headers (one per variant)
│   │   ├── instance_parser.h  # Abstract base: read(filepath) -> unique_ptr<Problem>
│   │   └── *_parser.h       # OPParser, TOPParser, TOPTWParser, TDOPParser, ...
│   ├── model/               # Problem domain types
│   │   ├── node.h           # Node struct: id, x, y, reward, service_time, tw, neighbors
│   │   ├── problem.h        # Abstract Problem base class
│   │   ├── solution.h       # Solution: vector<vector<NodeId>> + metadata
│   │   ├── graph.h          # OPGraph: arc/detour preprocessing (legacy, not used by current solvers)
│   │   └── variants/        # Concrete problem classes (9 total)
│   │       ├── op.h         # OPProblem (root of variant hierarchy)
│   │       ├── optw.h       # OPTWProblem extends OPProblem
│   │       ├── top.h        # TOPProblem extends OPProblem
│   │       ├── toptw.h      # TOPTWProblem extends OPTWProblem
│   │       ├── tdop.h       # TDOPProblem extends OPProblem
│   │       ├── tdoptw.h     # TDOPTWProblem extends OPTWProblem
│   │       ├── mctopmtw.h   # MCTOPMTWProblem extends TOPTWProblem
│   │       ├── singlesat.h  # SingleSatProblem extends OPTWProblem
│   │       └── ttdp.h       # TTDPProblem extends TOPTWProblem
│   └── solver/
│       ├── solver.h         # Abstract Solver + SolverConfig
│       ├── constructive/
│       │   ├── move_evaluator.h       # Abstract MoveEvaluator + concrete impls
│       │   ├── greedy.h               # GreedySolver + RouteContext + InfeasibilityCache
│       │   └── randomized_greedy.h    # RandomizedGreedySolver (RCL-based)
│       └── local_search/
│           ├── LNS.h                  # LNSSolver (destroy + repair)
│           └── grasp.h                # GraspSolver (multi-iteration metaheuristic)
├── src/                     # Implementation files mirroring include/
│   ├── CMakeLists.txt       # Builds orienteeringLib (STATIC), optional Boost
│   ├── io/                  # 8 parser .cpp files
│   ├── model/graph.cpp
│   └── solver/
│       ├── constructive/    # greedy.cpp, randomized_greedy.cpp, move_evaluator.cpp
│       └── local_search/    # LNS.cpp, grasp.cpp
├── examples/                # 6 standalone executables (not part of the library)
│   ├── benchmark_utils.h    # Shared parsing/CSV helpers for all benchmarks
│   ├── solve_op.cpp         # CLI: -file, -seed, -timeout; uses LNSSolver
│   ├── benchmark_greedy.cpp
│   ├── benchmark_randomized_greedy.cpp
│   ├── benchmark_lns.cpp
│   ├── benchmark_grasp.cpp
│   └── check_dummy.cpp
└── tests/                   # GoogleTest suite
    ├── main.cpp
    ├── test_greedy_solver.cpp   # Unit + integration tests for GreedySolver
    ├── test_io_parsers.cpp      # Parser tests (require data/ folder with instances)
    ├── test_model_extensions.cpp
    └── test_ds.cpp              # Minimal data-structure sanity checks
```

---

## 3. Build and Test Commands

### Configure and Build

```bash
# Standard Release build (Linux/macOS)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Debug build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Disable test compilation (faster CI or library-only builds)
cmake -S . -B build -DPACKAGE_TESTS=OFF
cmake --build build

# Parallel build
cmake --build build -j$(nproc)
```

### Run Tests

```bash
# Via CTest (preferred)
ctest --test-dir build

# Directly (shows individual test names and failure messages)
./build/tests/test_orienteeringLib

# Run tests matching a name pattern
./build/tests/test_orienteeringLib --gtest_filter="GreedySolverTest.*"
```

### Run Example Executables

```bash
# Solve a single OP instance with LNS
./build/examples/solve_op -file data/op/ChaoSet_64/set_64_1_15.txt -seed 42 -timeout 60.0

# Benchmark GRASP over a dataset
./build/examples/benchmark_grasp -dataset data/ -mode quick -alpha 0.3 -lns_iters 10 -timelimit 30

# Other benchmarks: benchmark_greedy, benchmark_randomized_greedy, benchmark_lns
```

### Windows (MSYS2/GCC)

```powershell
$env:PATH = "C:\msys64\mingw64\bin;C:\msys64\usr\bin;$env:PATH"
cmake -S . -B build -G"Unix Makefiles"
cmake --build build -j4
.\build\tests\test_orienteeringLib.exe
```

### Windows (MSVC + vcpkg for Boost)

```powershell
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
cmake --build build --config Release -j4
```

### Important Notes

- GoogleTest v1.15.2 is auto-downloaded via `FetchContent` on first configure — requires internet access.
- Boost serialization is **optional**. If not found, a `[WARNING]` is printed but the build succeeds.
- The `data/` folder is **not** in the repository (listed in `.gitignore`). Integration tests that load real instance files will fail or be skipped without it. Unit tests using synthetic problems always pass.
- All build artifacts go in `build/` (out-of-source). Never commit `build/`.

---

## 4. Key Architecture Concepts

### 4.1 Namespace Layout

```
oplib::                         — root (types, constants)
oplib::constants::              — compile-time constants
oplib::model::                  — Node, Problem, Solution, OPGraph
oplib::model::variants::        — 9 concrete problem classes
oplib::solver::                 — Solver (abstract), SolverConfig
oplib::solver::constructive::   — GreedySolver, RandomizedGreedySolver, MoveEvaluator
oplib::solver::local_search::   — LNSSolver, GraspSolver
oplib::io::                     — InstanceParser (abstract), all parsers
oplib::utils::                  — Random, RoutingUtils
```

### 4.2 Core Types (`include/core/types.h`, `include/core/constants.h`)

```cpp
using NodeId = int32_t;
using Reward = double;
using Time   = double;   // used for both travel time and time budget

struct TimeWindow {
    Time opening = 0.0;
    Time closing = 1e18;  // default = "no constraint"
    bool is_within(Time t) const;
};

enum class ScalingMode { RAW, SCALED_INTEGER };
enum class InsertionStrategyMode { CUSTOMER_WISE, VEHICLE_WISE };
enum class MoveEvaluatorType { REWARD, REWARD_PER_TIME, REWARD_PER_DISTANCE, SQUARED_REWARD_PER_DISTANCE };
```

Constants (from `constants.h`): `DEFAULT_SEED = 42`, `DEFAULT_TIMELIMIT_SECONDS`, `DEFAULT_MAX_ITERATIONS`.

### 4.3 Node Layout Convention

Every problem instance uses **index 0 as source depot** and **index (n-1) as sink depot**. Customers occupy indices 1 through n-2. Always call `get_source_depot()` and `get_sink_depot()` — never hardcode 0 or n-1.

Routes in a `Solution` always begin with `source_depot` and end with `sink_depot`.

### 4.4 Problem Variant Hierarchy

```
Problem (abstract)
└── OPProblem            — single vehicle, Euclidean distances, budget constraint
    ├── OPTWProblem      — adds per-node time windows + service times
    │   ├── TOPTWProblem     — adds multiple vehicles
    │   │   ├── MCTOPMTWProblem  — adds multi-commodity knapsack constraints
    │   │   └── TTDPProblem      — multi-day variant (num_days = num_vehicles)
    │   ├── TDOPTWProblem    — time-dependent travel + time windows
    │   └── SingleSatProblem — satellite scheduling (domain extension)
    ├── TOPProblem       — multiple vehicles, no time windows
    └── TDOPProblem      — time-dependent travel, no time windows
```

Capability flags (virtual methods on `Problem`):
- `has_time_windows()` — nodes have `[opening, closing]` constraints
- `is_time_dependent()` — `get_travel_time(i, j, departure_time)` varies with time
- `is_multi_vehicle()` — `get_num_vehicles() > 1`

### 4.5 Problem Lifecycle

Every problem goes through three stages before solving:

```cpp
auto problem = parser.read("instance.txt");  // 1. Parse: allocate nodes, set budget
problem->finalize();        // 2. Compute Euclidean distance matrix O(n²)
problem->preprocessing();   // 3. Prune infeasible arcs, sort neighbors by heuristic
```

`preprocessing()` populates `nodes[i].neighbors` — a sorted list of reachable successors. The greedy solver iterates over `neighbors` to find insertions. **Without calling `preprocessing()`, `neighbors` is empty and no customers are ever inserted.**

`OPParser` calls `finalize()` internally but NOT `preprocessing()`. Always check which stages a specific parser performs, and call the missing ones explicitly.

### 4.6 Solution Structure

```cpp
// routes[v] = [source_depot, c1, c2, ..., sink_depot]
vector<vector<NodeId>> routes;
Reward total_reward       = 0.0;
Time   total_travel_time  = 0.0;
```

- Each route starts with `source_depot` and ends with `sink_depot`.
- No customer node appears in more than one route.
- `total_reward` and `total_travel_time` are maintained incrementally by the solver — do not recompute unless validating.

### 4.7 Greedy Solver Internals

`GreedySolver` is the central building block used by all other solvers.

**`RouteContext`** — per-vehicle state updated incrementally on each insertion:
```cpp
struct RouteContext {
    vector<Time> arrival_times;    // at each route position
    vector<Time> departure_times;  // from each route position
    vector<Time> max_shift;        // how much delay each position can absorb
    Time cumulative_time;          // total travel time on route
};
```

**`InfeasibilityCache`** — a `set<tuple<NodeId, vehicle, position>>` of known-infeasible insertions. Avoids re-checking. Entries for a customer are invalidated via `invalidate_customer(c)` when that customer is inserted (since route structure changes). The cache is cleared at the start of each `solve()` call.

**`solve_from_partial()`** — rebuilds `RouteContext` from an existing partial solution, then continues greedy insertion. Used by `LNSSolver` after its destroy step. Calls `rebuild_route_context()` per vehicle internally.

**`InsertionStrategyMode`**:
- `CUSTOMER_WISE` (default): evaluates all customers × all vehicles × all positions per step; picks the global best
- `VEHICLE_WISE`: fills one vehicle at a time

### 4.8 Algorithm Stack

| Solver | Class | Config | Key behavior |
|--------|-------|--------|--------------|
| Greedy | `GreedySolver` | `GreedySolverConfig` | Deterministic best-insertion |
| Randomized Greedy | `RandomizedGreedySolver` | `RandomizedGreedyConfig` (alpha) | RCL-based: picks from top-alpha fraction |
| LNS | `LNSSolver` | `SolverConfig` | Greedy seed → destroy K nodes → greedy repair; accept only improvements |
| GRASP | `GraspSolver` | `GraspConfig` (alpha, lns_iterations) | Multi-iteration: RandomizedGreedy + LNS; tracks global best |

GRASP terminates at `max_cpu_time` or `max_iterations`, whichever comes first.

LNS has no simulated annealing — only improving moves are accepted.

### 4.9 Time-Dependent Travel

For TDOP/TDOPTW, `get_travel_time(i, j, departure_time)` overrides the static `get_distance(i, j)`. The solver calls `get_travel_time` everywhere during construction. `get_distance` returns free-flow Euclidean distance used only in preprocessing.

`estimate_departure_time(i, j, arrival_at_j)` is the inverse operation: "latest departure from i to arrive at j by time T". TDOP variants override this using piecewise-linear interpolation via `RoutingUtils`.

---

## 5. Coding Conventions

### Naming

| Element | Convention | Example |
|---------|-----------|---------|
| Classes | CamelCase | `OPTWProblem`, `GreedySolver` |
| Methods | snake_case verbs | `get_reward()`, `check_insertion_feasible()` |
| Variables | snake_case | `travel_time`, `best_move` |
| Constants | SCREAMING_SNAKE_CASE | `DEFAULT_SEED`, `INF_TIME` |
| Type aliases | CamelCase | `NodeId`, `Reward`, `Time` |
| Namespaces | snake_case | `oplib::model::variants` |

### Formatting

- Indentation: **4 spaces**, no tabs.
- Braces: **K&R style** — opening brace on same line for functions, `if`, loops, and class bodies.
- Always use `override` on overriding virtual methods.
- No enforced line length; target ~120 chars.

### Memory and Ownership

- `unique_ptr` for ownership transfer (e.g., parsers return `unique_ptr<Problem>`).
- `const&` for non-owning input parameters.
- No raw owning pointers. Prefer stack allocation for local algorithm state.

### Comments and Documentation

- **Doxygen** (`/** ... */`) on all public API in headers.
- **Inline** (`//`) for non-obvious algorithmic steps in `.cpp` files.
- Never comment what the code already says clearly. If a comment restates the code, delete it.

### Headers

- Use `#pragma once` (not include guards).
- Include relative to the library root: `#include "model/problem.h"`, never `../model/problem.h`.
- `include/` is the root include path — use it as such everywhere.

---

## 6. How to Add a New Problem Variant

### Step 1: Header in `include/model/variants/`

Pick the correct base class from the hierarchy (section 4.4):

```cpp
// include/model/variants/my_variant.h
#pragma once
#include "model/variants/toptw.h"

namespace oplib::model::variants {

class MyVariantProblem : public TOPTWProblem {
public:
    MyVariantProblem(std::string name, int num_vehicles, Time tmax)
        : TOPTWProblem(std::move(name), num_vehicles, tmax) {}

    // Override only what changes from the parent:
    // bool has_time_windows() const override { return true; }  // inherited — no need

    // New constraint accessors:
    void set_my_constraint(/* ... */) {}

private:
    // Additional data fields
};

} // namespace oplib::model::variants
```

Rules:
- Override `has_time_windows()` if the variant introduces time windows not in the parent.
- Override `is_time_dependent()` and `get_travel_time()` for time-dependent variants.
- Override `is_multi_vehicle()` and `get_num_vehicles()` for multi-vehicle variants.
- Override `preprocessing()` if arc-pruning must account for new constraints. Call `OPProblem::preprocessing()` (or the parent's) first, then apply additional pruning.
- Do NOT override `get_distance()` for time-dependent variants — override `get_travel_time()` instead. `get_distance()` always returns static Euclidean distance.

### Step 2: Parser header in `include/io/`

```cpp
// include/io/my_variant_parser.h
#pragma once
#include "io/instance_parser.h"
#include "model/variants/my_variant.h"

namespace oplib::io {

class MyVariantParser : public InstanceParser {
public:
    std::unique_ptr<model::Problem> read(const std::string& filepath) override;
};

} // namespace oplib::io
```

### Step 3: Parser implementation in `src/io/my_variant_parser.cpp`

Model on `src/io/op_parser.cpp` or `src/io/toptw_parser.cpp`. The parser must:
1. Open and read the file.
2. Construct with `std::make_unique<MyVariantProblem>(...)`.
3. Call `problem->finalize()`.
4. Return `nullptr` on parse error (do not throw).
5. Do NOT call `preprocessing()` — leave that to the caller (consistent with other parsers).

Note: `TDOPParser` uses **static methods** taking multiple file paths (`TDOPParser::read(instance, speed_matrix, arc_cat)`). Match the pattern of the specific parser you extend.

### Step 4: Tests in `tests/`

Add to an existing test file or create `tests/test_my_variant.cpp`. Always build a complete synthetic problem: source depot (index 0), customers, sink depot (index n-1). Call both `finalize()` and `preprocessing()` before solving.

```cpp
#include <gtest/gtest.h>
#include "model/variants/my_variant.h"
#include "solver/constructive/greedy.h"

TEST(MyVariantTest, SyntheticInstance) {
    MyVariantProblem problem("test", 2, 100.0);
    // source depot: reward=0
    problem.add_node(Node{0, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    // customers ...
    // sink depot: reward=0
    problem.add_node(Node{n-1, 0.0, 0.0, 0.0, 0.0, {0.0, 100.0}});
    problem.finalize();
    problem.preprocessing();

    oplib::solver::constructive::GreedySolver solver;
    oplib::solver::SolverConfig config;
    auto solution = solver.solve(problem, config);
    EXPECT_EQ(static_cast<int>(solution.routes.size()), 2);
}
```

---

## 7. How to Add a New Solver

### Step 1: Header in `include/solver/`

Place constructive solvers in `include/solver/constructive/`, metaheuristics in `include/solver/local_search/`:

```cpp
// include/solver/local_search/my_solver.h
#pragma once
#include "solver/solver.h"

namespace oplib::solver::local_search {

struct MySolverConfig : public SolverConfig {
    double my_param = 0.5;
};

class MySolver : public Solver {
public:
    std::string get_name() const override { return "MySolver"; }
    model::Solution solve(const model::Problem& problem, const SolverConfig& config) override;
    model::Solution solve(const model::Problem& problem, const MySolverConfig& config);
};

} // namespace oplib::solver::local_search
```

Always implement both the base `solve(const Problem&, const SolverConfig&)` and a typed overload for solver-specific parameters.

### Step 2: Implementation in `src/solver/`

```cpp
// src/solver/local_search/my_solver.cpp
#include "solver/local_search/my_solver.h"
#include "solver/constructive/greedy.h"

namespace oplib::solver::local_search {

model::Solution MySolver::solve(const model::Problem& p, const SolverConfig& cfg) {
    MySolverConfig my_cfg;
    my_cfg.seed = cfg.seed;
    my_cfg.max_cpu_time = cfg.max_cpu_time;
    my_cfg.max_iterations = cfg.max_iterations;
    my_cfg.verbose = cfg.verbose;
    return solve(p, my_cfg);
}

model::Solution MySolver::solve(const model::Problem& p, const MySolverConfig& cfg) {
    constructive::GreedySolver greedy;
    model::Solution sol = greedy.solve(p, cfg);
    // ... improve ...
    return sol;
}

} // namespace oplib::solver::local_search
```

See `LNSSolver::solve` for the canonical pattern of a destroy-repair solver using `solve_from_partial`.

### Step 3: Optional benchmark example

Copy `examples/benchmark_lns.cpp` as a template. Add to `examples/CMakeLists.txt`:

```cmake
add_executable(benchmark_my_solver benchmark_my_solver.cpp)
target_link_libraries(benchmark_my_solver orienteeringLib)
```

### Step 4: New MoveEvaluator (if needed)

1. Add an entry to `MoveEvaluatorType` in `include/core/types.h`.
2. Add a class inheriting `MoveEvaluator` in `include/solver/constructive/move_evaluator.h`.
3. Implement in `src/solver/constructive/move_evaluator.cpp`.
4. Register in `create_move_evaluator()` factory in the same file.

---

## 8. Testing Patterns

### Synthetic Problem Setup

```cpp
OPProblem problem("test_name", /*budget=*/100.0);
problem.add_node(Node{0,  0.0,  0.0, 0.0, 0.0});  // source depot
problem.add_node(Node{1, 10.0,  0.0, 20.0, 0.0}); // customer
problem.add_node(Node{2,  0.0,  0.0, 0.0, 0.0});  // sink depot
problem.finalize();
problem.preprocessing();
```

For OPTW/TOPTW, pass `TimeWindow` as the last `Node` field:
```cpp
problem.add_node(Node{1, 10.0, 0.0, 20.0, /*svc=*/5.0, {/*open=*/10.0, /*close=*/50.0}});
```

### Integration Tests with Real Instances

Tests that load from `data/` should guard against missing files:

```cpp
auto problem = OPParser().read("data/op/set_64_1_15.txt");
ASSERT_NE(problem, nullptr) << "Skipping: data/ not available";
```

Do not rely on real instance files for invariant validation — use synthetic problems for that.

### Accessing Protected Members in Tests

Use a thin test subclass:

```cpp
class TestOPProblem : public OPProblem {
public:
    using OPProblem::OPProblem;
    const std::vector<Node>& get_nodes_public() const { return nodes; }
};
```

### Test Config Defaults

```cpp
oplib::solver::SolverConfig config;
config.seed    = 42;
config.verbose = false;
```

---

## 9. Common Pitfalls and Important Notes

1. **Skipping `preprocessing()` before `solve()`**: `nodes[i].neighbors` is empty without it. The greedy solver will find no insertions and return an empty solution. Always call `finalize()` then `preprocessing()`.

2. **Stale `RouteContext` after manual route modification**: `RouteContext` is only initialized in `solve()` and `solve_from_partial()`. If you modify a `Solution` externally (e.g., remove nodes for a destroy step), you must call `rebuild_route_context()` before resuming greedy construction. `LNSSolver` handles this via `solve_from_partial`.

3. **`total_travel_time` after destroy**: `LNSSolver`'s destroy step decrements `total_reward` but does NOT update `total_travel_time`. Do not read `total_travel_time` on a partially destroyed solution — it is reset when `solve_from_partial` rebuilds from scratch.

4. **Depot node indices are runtime values**: `OPParser` may rotate the parsed node list (`std::rotate`) so the sink ends up last. Always use `get_source_depot()` and `get_sink_depot()` instead of assuming positions match raw file order.

5. **All vehicles share a single budget**: `get_budget()` returns the per-route time limit for all vehicles. There is no per-vehicle budget in any current variant.

6. **`ScalingMode::SCALED_INTEGER` affects benchmark comparisons**: OP instances typically use `SCALED_INTEGER` with `time_scale=1.0` (`floor(raw)`). This matches published benchmark results. OPTW/TOPTW variants may use `RAW`. Check which mode the parser sets before comparing to published tables.

7. **`OPGraph` is legacy**: `include/model/graph.h` implements `OPGraph` with arc/detour preprocessing. It is NOT used by any current solver. Do not couple new solvers to it; use the `Problem` interface instead.

8. **Parser static vs. instance methods**: Most parsers are instance methods. `TDOPParser` and `TDOPTWParser` use **static methods** taking multiple file paths. Check the specific parser before calling.

9. **`INF_TIME = 1e18` comparisons**: `INF_TIME` is used as "no constraint" for time windows and budgets. Avoid `== 1e18`; test with `>= 1e17` when checking for unconstrained values.

10. **`InfeasibilityCache` is per-solver-instance**: It is cleared internally at the start of each `solve()` call. Creating a new `GreedySolver` object also clears it. Reusing a solver object across calls is safe.
