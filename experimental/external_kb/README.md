# Experimental: external-solver knowledge-base backends

**Status:** opt-in scaffold (disabled by default). Enable with
`cmake -S . -B build -DENABLE_EXTERNAL_KB=ON`.

## Purpose

The core library ships a **dependency-free** pseudo-Boolean knowledge base,
`oplib::knowledge_base::SelectionConstraintManager` (see
`include/knowledge_base/sel_manager.h`). It is the clean successor of the
knowledge bases in the `kb_ls_cpp` research repo, which were built on **external
solvers**:

| Backend | Library (in `kb_ls_cpp`) | Role |
|---|---|---|
| SAT   | CryptoMiniSat | clause store / conflict learning |
| BDD   | CUDD          | exact model counting / compilation |
| CP/MILP | CPLEX (CP Optimizer) | exact selection / bounding |

This module is where those external-backed KBs are **reintroduced for research
comparison** against the dependency-free PB KB — without burdening the core
library, which stays Boost-optional and builds with no third-party solvers.

## Why it is gated OFF by default

CryptoMiniSat, CUDD and CPLEX are heavy, separately-licensed dependencies. Most
users only need the PB KB. The `ENABLE_EXTERNAL_KB` CMake option keeps them
entirely out of the default build; when it is OFF this directory is not even
added to the build.

## Intended architecture

All backends are meant to satisfy one selection-KB contract — the same surface
the PB `SelectionConstraintManager` already exposes:

- `add_conflict_constraint` / `add_disjunctive_constraint` /
  `add_knapsack_constraint` / objective bound,
- `assign` / `unassign` with unit propagation,
- a branching/decision heuristic for KB-guided construction.

A future `oplib::knowledge_base::KnowledgeBaseBackend` abstract interface would
let solvers swap the PB KB for a SAT/BDD/CP backend transparently. The PB KB
would be the default implementation; the modules here would be alternatives.

## Source mapping (planned port from `kb_ls_cpp`)

| `kb_ls_cpp` source | Target here |
|---|---|
| `src/kb/kb_factory.{h,cpp}` | `src/sat_kb.cpp` (CryptoMiniSat backend) |
| `src/kb/wobdd.{h,cpp}` (branch `tmp/cp_approach`) | `src/bdd_kb.cpp` (CUDD backend) |
| `src/solver/cpo_model.{h,cpp}` (branch `tmp/cp_approach`) | `src/cp_kb.cpp` (CPLEX backend) |
| `src/solver/xplainer.{h,cpp}` | feeds learned no-goods into any backend |

Until those are migrated, enabling the module only **verifies the dependencies
are present** (see `CMakeLists.txt`) so the toolchain can be validated ahead of
the port.
