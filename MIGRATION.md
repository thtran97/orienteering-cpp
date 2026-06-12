# Migration Plan: Consolidating Prior Work into `orienteering-cpp`

This document tracks the incremental migration of the author's earlier research
codebases into this repository (`orienteering-cpp`), the clean C++17 rewrite of
the Orienteering Problem solver line.

## 1. Source repositories & lineage

The work spans three generations of the same research line — *knowledge-base–boosted
local search for Orienteering Problems*:

| Generation | Repo | Active | Model API | Role |
|---|---|---|---|---|
| Gen 1 (2022–23) | `kb_ls_cpp` | Aug–Dec 2022 | `optw_model` | AAAI'23 / CPAIOR'23 research: KB via **external** SAT/BDD/CP solvers |
| Gen 2 (2024–26) | `toptwLib` | …Feb 2026 | two APIs (below) | Full research library: OP variants + dependency-free KB + RR/pulse/xplainer |
| Gen 3 (2026) | `orienteering-cpp` | current | `oplib::model::Problem` (virtual) | Clean rewrite — **this repo** |

`orienteering-cpp` is the direct continuation of `toptwLib`'s `wsl-quickstart`
branch (same `Solver`/`SolverConfig`/`model::Problem` interfaces; namespace
`toptwlib::` → `oplib::`). Everything else in `toptwLib` lives on the **old API**
(`GenericOrienteeringProblem`) and must be *ported + adapted*, not just copied.

### `toptwLib` — per branch
- **`main`** — full research codebase, old API. Source of truth for the heavy
  artifacts: `gen_op_solvers/` (`base_solver`, `solver_with_rr` 81 KB,
  `smt_based_solver` 33 KB, `solver_with_xp`, `xplainer`), `route_recombinator/`,
  `conflict_extractor/`, full `dynamic_programming/` (`bidirectional_DP`,
  `dd_construction`, `dssr`), full `pulse_algo/` (`pulse_graph` 71 KB),
  `local_search/` (`base_LS`, `GRASP_VNS`, `ILS09`, `ILS09_with_xpdp`,
  `ILS_with_route_recombination`, `LNS`, `LNS_with_pulse`, `SAILS`),
  `policy_learning/` (`mcts`, `mcls`), watched-literal KBs, `utils/`.
- **`wsl-quickstart`** — the clean rewrite on the new API; the actual ancestor of
  this repo. ~90 % already present here.

### `kb_ls_cpp` — per branch
Implements different KB representations to boost LS for (T)OPTW. The *concept*
survives here as `oplib::knowledge_base::sel_manager` (dependency-free
pseudo-Boolean constraint manager). Not migrated: the external SAT/BDD/CP
backends and the conflict-explanation glue.
- `master` / `remake_branch` — baseline (CPLEX + CUDD + CryptoMiniSat).
- `kbls_otpw_aaai23` — AAAI'23 snapshot (CUDD + CryptoMiniSat).
- `kbls_optw_cpaior23` — CPAIOR'23 snapshot.
- `tmp/cp_approach` — experimental CP-Optimizer model + weighted OBDD.

## 2. Current state of `main` (baseline for this plan)

The base-solver suite has already landed on `main`:

- **Model**: `graph`, `node`, `problem`, `solution` + 9 variants (op, optw, top,
  toptw, tdop, tdoptw, mctopmtw, singlesat, ttdp).
- **IO**: 8 parsers.
- **Solvers**: `constructive/{greedy, randomized_greedy, move_evaluator}`,
  `local_search/{base_ls, moves}`, `dynamic_programming/{dp_solvers, label}`
  (reduced), `metaheuristic/{grasp_vns, ils09, ils_route_recombination, lns}`,
  `policy_learning/mcts_*`, `pulse/pulse_solver` (reduced).
- **Knowledge base**: `knowledge_base/sel_manager` (PB constraint manager).

### Known issues on `main` (merged-lineage tech debt)
1. **Build break / duplication** — the squash that landed the suite kept *both*
   development lineages side by side:
   - Two LNS solvers: `local_search::LNSSolver` **and** `metaheuristic::LNSSolver`.
   - Two GRASP solvers: `local_search::GraspSolver` (GRASP-LNS) vs
     `metaheuristic::GraspVnsSolver` (GRASP-VNS).
   - `src/solver/local_search/grasp.cpp` references `RandomizedGreedyConfig`
     (renamed to `RandomizedGreedySolverConfig`), so **`main` does not compile**.
2. **Docs drift** — README documents only the old GRASP-LNS feature; no
   `CLAUDE.md` on `main`.
3. **Test coverage gaps** — TDOP, TDOPTW, MCTOPMTW, SingleSat, TTDP have no
   solver tests; `tests/test_metaheuristic_solvers.cpp` is entirely `#if 0`.

## 3. Incremental migration plan

Each phase is an independent PR off `main`; it must **build + pass tests +
hold the benchmark regression baseline** before the next begins. Two porting
sources: new-API code (near-direct, namespace swap) vs old-API code in
`toptwLib/main` (port **and** adapt to the virtual `Problem`/`Solution` API).
**Dependency policy:** core stays Boost-optional / C++17; anything needing
CPLEX/CUDD/CryptoMiniSat goes behind an OFF-by-default CMake option.

### Phase 0 — Re-baseline & consolidate  *(in progress)*
- [x] Retire the duplicate, broken `local_search::{GraspSolver, LNSSolver}`;
      canonicalise on the `metaheuristic::` versions (built on `BaseLSUtils`).
- [x] Fix the build break; collapse overlapping benchmarks onto
      `benchmark_all` / `benchmark_exact`.
- [x] Add comprehensive solver × variant tests (≥ 1 instance per variant).
- [x] Commit a curated test-data subset (~32 KB, force-added from toptwLib's
      identically-structured `data/`) so the IO/parser and real-instance tests
      pass. The large `singlesat/` set (~103 MB) is excluded and its parser test
      skips when absent. All other `data/` stays git-ignored.
- [ ] Sync docs (README to describe the real solver suite; fold in `CLAUDE.md`).
- [ ] Port dependency-free `utils/` (`custom_queue`, `comparator`, `hash`,
      `argument_parser`) + a solver registry/CLI so new solvers are instantly
      benchmarkable.

### Phase 1 — Harden the exact / bounding layer  *(correctness hardened)*
- [x] Fixed an invalid backward bound — a path-recursion that *under*estimated
      on the cyclic graph — which had made `PulseSolver` non-optimal and risked
      incorrect pruning in `BidirectionalDP`. Replaced with a sound
      sum-of-reachable-customers upper bound (valid via the triangle inequality).
- [x] Added `tests/test_exact_optimality.cpp`: an independent brute-force oracle
      proves `ForwardDP`, `BidirectionalDP` and `Pulse` return the true optimum
      across 50 randomized OP/OPTW instances; backward bounds are verified to
      dominate the optimum.
- [ ] Further fidelity from `toptwLib/main` (decremental state-space relaxation,
      decision-diagram construction, full `pulse_algo` bounding) for scaling to
      larger instances.

### Phase 2 — Route Recombination (flagship)  *(route-level operator added)*
- [x] Added a true route-level recombination operator
      `ILSRouteRecombinationSolver::recombine_routes` — a max-weight set-packing
      over the elite pool's routes (greedily selects the highest-reward
      customer-disjoint routes, assigns them to vehicles, repairs leftovers).
      Replaces the previous customer-level path-relinking, matching the intent of
      toptwLib's `route_recombinator` / `combinator`. Valid for TOP/TOPTW because
      vehicles are independent, so a route stays feasible when reassigned.
- [x] Added `tests/test_route_recombination.cpp` (combines disjoint routes,
      never worse than the best pool member, empty-pool safety).
- [ ] Full `solver_with_rr` fidelity (exact set-packing / column-generation
      route selection over a larger pool) for stronger recombination.

### Phase 3 — Knowledge Base + conflict explanation (research core)  *(KB verified)*
- [x] Verified the dependency-free pseudo-Boolean `SelectionConstraintManager`
      (successor of kb_ls_cpp's KB), which previously had **zero** coverage and no
      solver using it. Added `tests/test_knowledge_base.cpp`: conflict /
      disjunctive / knapsack propagation, objective lower-bound infeasibility
      detection, unit-propagation of complements, backtracking via `unassign`,
      and a KB-guided greedy selection loop.
- [ ] Port `xplainer` / `conflict_extractor` to generate no-goods from infeasible
      local-search moves and feed them (as conflict constraints) into this KB;
      then adapt `solver_with_xp` to drive construction from the KB's decision
      heuristic.

### Phase 4 — Remaining solvers
`mcls`, `SAILS`, `LNS_with_pulse`, `ILS09_with_xpdp`.

### Phase 5 — (Optional, CMake-gated) external-backend KB
Reintroduce `kb_ls_cpp`'s SAT/BDD/CP backends as an experimental module behind
an OFF-by-default option, for research comparison vs. the dependency-free PB KB.
