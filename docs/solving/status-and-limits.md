# Status, limits and tolerances

Research code rarely gets to assume the solve finished. Benchmarks run under a time limit, instances turn out infeasible, and a run that stopped early may still carry a usable incumbent. This page covers what MIP++ tells you about how a solve ended, and the knobs that decide when it ends.

## Solving

```cpp
model.solve();
double obj = model.get_solution_value();
```

`solve()` runs the solver synchronously and can be called any number of times on the same model; what the solver reuses between calls is discussed in [Re-solving and model updates](updates.md).

## The coarse status

Model classes satisfying `has_lp_status` answer the three classical questions:

```cpp
model.solve();
if(model.proven_optimal())         { /* solution is optimal */ }
else if(model.proven_infeasible()) { /* the model has no feasible point */ }
else if(model.proven_unbounded())  { /* the objective is unbounded */ }
```

These are *proofs*, not a partition: a solve stopped by a time limit answers `false` to all three. Never treat `!proven_infeasible()` as "feasible".

!!! note "Which classes provide it"
    `has_lp_status` is currently satisfied by the **LP (and QP) model classes** — Clp, COPT, CPLEX, GLPK, Gurobi, HiGHS, MOSEK, Xpress. The **MILP classes** report their outcome through `termination_reason()` below, available on `gurobi_milp`, `cplex_milp` and `highs_milp`; the remaining MILP backends (Cbc, COPT, GLPK, MOSEK, SCIP, Xpress) expose no status query yet — broadening this is on the [roadmap](https://github.com/fhamonic/mippp#roadmap). Write status handling behind `if constexpr` so it degrades cleanly (see below).

## The precise reason: `termination_reason()`

Backends satisfying `has_termination_reason` — currently **Gurobi**, **CPLEX** and **HiGHS** — return a `std::variant` of tag types from `namespace reason`, and the tags form a **hierarchy**, so you can ask questions at whatever granularity you need:

```text
any
├── completed
│   ├── optimal
│   │   ├── optimal_face_unbounded          (infinitely many optima)
│   │   └── optimal_infeasible_unscaled
│   └── infeasible_or_unbounded
│       ├── infeasible
│       └── unbounded
└── stopped
    ├── interrupted
    ├── failed → numerical_failure, out_of_memory
    ├── limit_reached → time_limit, iteration_limit,
    │                   node_limit, solution_limit, memory_limit
    └── unknown
```

`reason::is<R>(r)` tests membership in a whole branch of that tree, which is what experiment drivers usually want:

```cpp
auto r = model.termination_reason();

if(reason::is<reason::optimal>(r))            record_optimal(model);
else if(reason::is<reason::limit_reached>(r)) record_timeout(model);
else if(reason::is<reason::failed>(r))        record_failure(model);
```

Crucially, a stopped solve may or may not leave an incumbent behind, and that is reported separately:

```cpp
if(reason::solution_available(r)) {
    auto sol = model.get_solution();     // safe: values exist
    record_bound(model.get_solution_value());
}
```

Reading a solution when no solution is available is a solver-level error — always gate on `proven_optimal()` or `reason::solution_available()` in code that runs under limits.

Which tags a backend can return is part of its type (`model_termination_reason_t<M>`, a `std::variant`), so exhaustive handling with `std::visit` is checkable at compile time; and a limit setter only exists on a backend whose `termination_reason()` can actually report that limit — the concepts require it.

## Limits

| Concept | Setter / getter | Backends |
| :--- | :--- | :--- |
| `has_time_limit` | `set_time_limit(std::chrono duration)`, `get_time_limit()` | Cbc, COPT, CPLEX, Gurobi, HiGHS, Xpress |
| `has_iteration_limit` | `set_iteration_limit(n)`, `get_iteration_limit()` | Gurobi, HiGHS |
| `has_node_limit` | `set_node_limit(n)`, `get_node_limit()` | CPLEX, Gurobi |
| `has_solution_limit` | `set_solution_limit(n)`, `get_solution_limit()` | CPLEX, Gurobi |
| `has_memory_limit` | `set_memory_limit(size)`, `get_memory_limit()` | CPLEX, Gurobi |

Time limits are `std::chrono` durations, so the unit is in the type and never in a comment:

```cpp
using namespace std::chrono_literals;
model.set_time_limit(10min);
model.set_time_limit(std::chrono::duration<double>(0.5));  // sub-second is fine
```

Memory limits use the `memory_size` units of [`utility/memory_size.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/utility/memory_size.hpp) — `bytes`, `kilobytes`/`megabytes`/`gigabytes` (SI) and `kibibytes`/`mebibytes`/`gibibytes` (binary):

```cpp
model.set_memory_limit(mebibytes{4096u});
```

A limit is a property of the model and survives across `solve()` calls, so setting it once before a benchmark loop is enough.

## Tolerances

| Concept | Provides | Backends |
| :--- | :--- | :--- |
| `has_feasibility_tolerance` | `get`/`set_feasibility_tolerance` | Cbc, Clp, COPT, CPLEX, GLPK, Gurobi, SCIP, Xpress |
| `has_optimality_tolerance` | `get`/`set_optimality_tolerance` (the MIP gap, where applicable) | Cbc, COPT, CPLEX, Gurobi, SCIP, Xpress |
| `has_integrality_tolerance` | `get`/`set_integrality_tolerance` | *declared, not yet provided by any backend* |

Two habits worth adopting in experimental code:

- **Read the tolerance instead of hard-coding `1e-9`.** Post-processing that rounds a binary (`sol[x] > 0.5`) or tests a reduced cost should be expressed against the solver's own tolerance where one is available, so the same code stays correct when you change backend or tighten the setting.
- **Report the tolerances with the results.** An optimality tolerance is part of what "optimal" meant in a table of results; the getters make dumping them into the run log a one-liner.

## Reproducible experiments

A minimal, portable driver that gets the same reporting on every backend:

```cpp
template <typename Model>
run_record run(Model & model, std::chrono::seconds budget) {
    if constexpr(has_time_limit<Model>) model.set_time_limit(budget);

    const auto start = std::chrono::steady_clock::now();
    model.solve();
    const auto elapsed = std::chrono::steady_clock::now() - start;

    run_record rec{.seconds = std::chrono::duration<double>(elapsed).count()};
    if constexpr(has_termination_reason<Model>) {
        auto r = model.termination_reason();
        rec.optimal = reason::is<reason::optimal>(r);
        rec.stopped = reason::is<reason::limit_reached>(r);
        rec.has_solution = reason::solution_available(r);
    } else if constexpr(has_lp_status<Model>) {
        rec.optimal = model.proven_optimal();
        rec.has_solution = rec.optimal;
    } else {
        rec.has_solution = true;  // backend reports no status: assume a solve
    }
    if(rec.has_solution) rec.objective = model.get_solution_value();
    return rec;
}
```

The `if constexpr` guards are the general pattern for optional capabilities; [Writing solver-generic code](../solvers/generic-code.md) develops it.

## Next

[Solutions, duals and reduced costs](solutions.md) — getting the numbers back out.
