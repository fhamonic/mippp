# Why MIP++

Every OR researcher has lived this trade-off: Python modeling layers (PuLP, gurobipy, Pyomo) are pleasant to write but painfully slow to *build* models with, while the solvers' C APIs are fast but force you to think in row/column indices and rewrite everything when you switch solvers. MIP++ is designed around the claim that you should not have to choose. Its whole architecture follows from a handful of design decisions, described below — the rest of the Getting-started guide then shows them in action.

## One model, every solver

In MIP++, a "model" is not a data structure you fill and then hand to a solver. There is **no intermediate model representation at all**: classes like `highs_lp` or `gurobi_milp` implement a common modeling interface *directly on top of the solver's own C API*. When you call `add_constraint`, the row goes straight into the solver.

What makes the model code portable is that this common interface is specified by **C++ concepts** — `lp_model`, `milp_model`, `qp_model`, plus fine-grained capability concepts such as `has_dual_solution` or `has_mip_start` (see the [concepts reference](../reference/concepts.md)). Any code written against the concepts runs on any backend that satisfies them:

```cpp
using api_type  = highs_api;   // change these two lines to re-run the same
using milp_type = highs_milp;  // experiment on Gurobi, CPLEX, SCIP, ...
```

Benchmarking Gurobi vs. CPLEX vs. HiGHS becomes a two-line change and a recompile — no `#ifdef` soup, no per-solver code paths. Templatize your model building code over the backend type and you can even select the solver at runtime.

For a researcher this matters beyond convenience: reviewers increasingly ask for results across solvers, and licenses differ between your laptop, the cluster, and your coauthors' machines.

## No modeling tax

The convenience of algebraic modeling usually costs one or two orders of magnitude in model-building time. That cost is invisible for a one-shot solve of a small model, but it dominates in exactly the situations research code cares about: large instances, column generation, row generation, and repeated re-solves in a benchmark loop.

MIP++ avoids the tax with a **functional, zero-copy expression system**:

- Expressions like `4 * x1 + 5 * x2` or `xsum(cities, [&](int j) { return X(i, j); })` are *lazy views* built from C++ ranges. No term vector is ever materialized; adding, negating, or scaling an expression just composes `concat` / `transform` / `join` views at compile time.
- When a constraint is registered, its terms are streamed **once** into a small buffer that the model reuses for every row (coefficients of repeated variables are merged on the fly), then passed to the solver's C function.
- Variables are trivially-copyable strongly-typed handles (an integer id), and variable *names* can be assigned lazily, so you never pay for strings you don't use.

The result: building an N-Queens model is within **10–20% of hand-written C API calls**, where Python layers are **40–550× slower** and JuMP 14–24× (see the [benchmark](https://github.com/fhamonic/mippp_nqueens)).

## Index variables by *your* problem's coordinates

Textbook models are written over meaningful index sets — *x(i,j)* for arcs, *y(p)* for patterns — but solver APIs only know flat column numbers. MIP++ bridges the two with **lambda id-maps**: create a batch of variables together with a function mapping your coordinates to an offset, and get back a range callable with those coordinates:

```cpp
// One binary variable per board cell; X(row, col) is O(1) arithmetic —
// no hash map, no lookup table, no string keys.
auto X = model.add_binary_variables(
    n * n, [n](int row, int col) { return row * n + col; });

model.add_constraint(X(0, 3) + X(1, 1) <= 1);
```

Constraint families work symmetrically: `add_constraints(keys, generator)` builds one row per key and returns a range from which each constraint handle can be retrieved *by its key* — which is exactly what you need to look up dual values later:

```cpp
auto demand = model.add_constraints(orders, [&](int o) { /* ... */ });
// after solving:
auto duals = model.get_dual_solution();
double price_of_o = duals[demand(o)];
```

## Built for algorithms, not just models

Most research codes are not "build model, call solve": they are branch-and-cut, column generation, decomposition, or parameter studies. The MIP++ interface exposes the building blocks these methods need, uniformly across the backends that support them:

- **candidate-solution callbacks** with `add_lazy_constraint` for
  branch-and-cut ([guide](../advanced/callbacks.md)),
- **dual solutions** and **`add_column`** for column generation
  ([guide](../advanced/column-generation.md)), plus a `column_manager` utility
  for large-scale pricing,
- **MIP starts**, **SOS1/SOS2** and **indicator constraints**,
- readable and modifiable objectives, bounds, and constraint rows for
  in-place model updates between solves,
- time/node limits and tolerance parameters for reproducible experiments.

Each of these is itself a concept (`has_candidate_solution_callback`, `has_add_column`, …), so a generic algorithm can state its requirements in its template signature and fail at *compile time* — with a clear diagnostic — if you instantiate it with a backend that lacks a capability, rather than at hour three of a run.

## Header-only, solvers loaded at runtime

MIP++ itself is header-only, and solver libraries are **not linked** — each `<solver>_api` object loads the solver's shared library dynamically when constructed. Your binary has no link-time dependency on any solver SDK; only the backends you actually instantiate need to be installed on the machine running it. Details and the library-resolution rules are in [Choosing a solver](solvers.md).

## Next steps

Head to [Installation](installation.md), then walk through [A first model](first-model.md).
