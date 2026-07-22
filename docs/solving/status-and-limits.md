# Status, limits and tolerances

Research code rarely gets to assume the solve finished. Benchmarks run under a time limit, instances turn out infeasible, and a run that stopped early may still carry a usable incumbent. This page covers what MIP++ tells you about how a solve ended, and the knobs that decide when it ends.

## Solving

```cpp
model.solve();
double obj = model.get_solution_value();
```

`solve()` runs the solver synchronously and can be called any number of times on the same model; what the solver reuses between calls is discussed in [Re-solving and model updates](updates.md).

## The solve status

`solve_status()` is part of the `lp_model` concept itself, so **every** model class reports how its last solve ended. It returns a `std::variant` of tag types from `namespace status`, and the tags form a **hierarchy**, so you can ask questions at whatever granularity you need:

```text
any
├── unknown            (also the status before the first solve())
├── completed
│   ├── optimal
│   │   ├── optimal_face_unbounded          (infinitely many optima)
│   │   └── optimal_infeasible_unscaled
│   └── infeasible_or_unbounded
│       ├── infeasible → primal_and_dual_infeasible
│       └── unbounded
└── stopped
    ├── interrupted
    ├── failed → numerical_failure, out_of_memory
    └── limit_reached → time_limit, iteration_limit,
                        node_limit, solution_limit, memory_limit
```

Two query functions mirror the two questions you can ask of a hierarchy:

- `is_a<S>(r)` — is the status in the branch rooted at `S`? This is what experiment drivers usually want.
- `is<S>(r)` — is the status *exactly* the tag `S`? Needed when a parent tag carries meaning of its own, as `infeasible_or_unbounded` does below.

```cpp
model.solve();
const auto & r = model.solve_status();

if(is_a<status::optimal>(r))            record_optimal(model);
else if(is_a<status::limit_reached>(r)) record_timeout(model);
else if(is_a<status::failed>(r))        record_failure(model);
```

These are *proofs*, not a partition: a solve stopped by a time limit is in none of the `completed` branches. Never treat `!is_a<status::infeasible>(r)` as "feasible".

Crucially, a stopped solve may or may not leave an incumbent behind, and that is reported separately:

```cpp
if(status::solution_available(r)) {
    auto sol = model.get_solution();     // safe: values exist
    record_bound(model.get_solution_value());
}
```

Reading a solution when no solution is available is a solver-level error — always gate on `is_a<status::optimal>(r)` or `status::solution_available(r)` in code that runs under limits.

Which tags a backend can return is part of its type (`model_solve_status_t<M>`, a `std::variant`), so exhaustive handling with `std::visit` is checkable at compile time; and a limit setter only exists on a backend whose `solve_status()` can actually report that limit — the concepts require it.

## `infeasible_or_unbounded` and `refine_lp_status()`

Solvers whose presolve applies dual reductions can terminate knowing the model is infeasible *or* unbounded without knowing which; backends where this happens carry the exact `status::infeasible_or_unbounded` tag in their variant. The test for this undecided outcome is the exact `is<status::infeasible_or_unbounded>(r)` — `is_a` would also match the decided `infeasible` and `unbounded` tags, which derive from it.

Two concepts describe what a model class can tell you:

- `has_lp_status<Model>` — the status variant can report `infeasible` and `unbounded` as distinct tags. Every model class satisfies it except `glpk_milp`, which cannot report `infeasible`.
- `has_refinable_lp_status<Model>` — the model provides `refine_lp_status()`: if the current status is exactly `infeasible_or_unbounded`, it re-solves with the offending reductions disabled, so that `solve_status()` afterwards reports `infeasible` or `unbounded`; on any other status it is a no-op. Currently satisfied by `gurobi_lp` and `cplex_lp`.

Because refining may mean a full re-solve, it never happens behind your back — the cost is only paid where the call is written:

```cpp
model.solve();
if constexpr(has_refinable_lp_status<Model>) model.refine_lp_status();
```

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
    const auto & r = model.solve_status();
    rec.optimal = is_a<status::optimal>(r);
    rec.stopped = is_a<status::limit_reached>(r);
    rec.has_solution = status::solution_available(r);
    if(rec.has_solution) rec.objective = model.get_solution_value();
    return rec;
}
```

The `if constexpr` guards are the general pattern for optional capabilities; [Writing solver-generic code](../solvers/generic-code.md) develops it.

## Next

[Solutions, duals and reduced costs](solutions.md) — getting the numbers back out.
