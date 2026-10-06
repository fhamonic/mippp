# Status, limits and tolerances

Research code rarely gets to assume the solve finished. Benchmarks run under a time limit, instances turn out infeasible, and a run that stopped early may still carry a usable incumbent. This page covers what MIP++ tells you about how a solve ended, and the knobs that decide when it ends.

## Solving

```cpp
model.solve();
double obj = model.get_solution_value();
```

`solve()` runs the solver synchronously and can be called any number of times on the same model; what the solver reuses between calls is discussed in [Re-solving and model updates](updates.md).

## The solve status

`get_status()` is part of the `lp_model` concept itself, so **every** model class reports how its last solve ended. It returns a `std::variant` of tag types from `namespace status`, and the tags form a **hierarchy**, so you can ask questions at whatever granularity you need:

```text
any
├── unknown            (also the status before the first solve())
├── completed
│   ├── optimal
│   │   ├── optimal_face_unbounded          (infinitely many optima)
│   │   └── optimal_infeasible_unscaled     (violated once unscaled)
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
const auto & r = model.get_status();

if(is_a<status::optimal>(r))            record_optimal(model);
else if(is_a<status::limit_reached>(r)) record_timeout(model);
else if(is_a<status::failed>(r))        record_failure(model);
```

These are *proofs*, not a partition: a solve stopped by a time limit is in none of the `completed` branches. Never treat `!is_a<status::infeasible>(r)` as "feasible".

Nor is every `optimal` a feasible point. `optimal_infeasible_unscaled` means the solver reached an optimum of its internally rescaled problem that violates a bound or constraint of your model beyond the feasibility tolerance. Only the Clp, CPLEX and SoPlex models report it, and a model that is in fact infeasible can end there. `is_a<status::optimal>(r)` accepts it, so code that relies on feasibility also checks for this exact tag. `is<S>(r)` does not compile on a variant without the alternative `S`, so generic code guards the check with the `variant_with_alternative` concept:

```cpp
template <typename Model>
bool feasible_optimum(const Model & model) {
    const auto & r = model.get_status();
    if constexpr(variant_with_alternative<model_status_t<Model>,
                                          status::optimal_infeasible_unscaled>)
        if(is<status::optimal_infeasible_unscaled>(r)) return false;
    return is_a<status::optimal>(r);
}
```

Crucially, a stopped solve may or may not leave an incumbent behind, and that is reported separately:

```cpp
if(status::solution_available(r)) {
    auto sol = model.get_solution();     // safe: values exist
    record_bound(model.get_solution_value());
}
```

Reading a solution when no solution is available is a solver-level error — always gate on `is_a<status::optimal>(r)` or `status::solution_available(r)` in code that runs under limits.

Which tags a backend can return is part of its type (`model_status_t<M>`, a `std::variant`), and a limit setter only exists on a backend whose `get_status()` can actually report that limit — the concepts require it.

!!! note "A backend's tag list may grow in a minor release"
    As a solver outcome that MIP++ used to fold into a coarser tag gets its own, the tag is added to that backend's variant in a minor release. It derives from the coarser tag, so `is_a` and `solution_available` answer as before, but the exact `is` on the coarser tag turns false on those solves. An exhaustive `std::visit` is a compile-time check against the pinned version only: give the visitor a catch-all `auto` overload, or expect to add a case when you upgrade. Removing or renaming a tag stays a major-version change.

## `infeasible_or_unbounded` and `refine_lp_status()`

Solvers whose presolve applies dual reductions can terminate knowing the model is infeasible *or* unbounded without knowing which; backends where this happens carry the exact `status::infeasible_or_unbounded` tag in their variant. `glpk_milp` carries it for another reason: GLPK stops as soon as the LP relaxation has no dual feasible solution, before it knows whether any integer point exists, so an unbounded MIP ends `infeasible_or_unbounded` there. The test for this undecided outcome is the exact `is<status::infeasible_or_unbounded>(r)` — `is_a` would also match the decided `infeasible` and `unbounded` tags, which derive from it.

Two concepts describe what a model class can tell you:

- `has_lp_status<Model>` — the status variant can report `infeasible` and `unbounded` as distinct tags. Every model class satisfies it, though `glpk_milp` leaves an unbounded MIP undecided, as above.
- `has_refinable_lp_status<Model>` — the model provides `refine_lp_status()`: if the current status is exactly `infeasible_or_unbounded`, it re-solves with the offending reductions disabled, so that `get_status()` afterwards reports `infeasible` or `unbounded`; on any other status it is a no-op. Currently satisfied by `gurobi_lp` and `cplex_lp`.

Because refining may mean a full re-solve, it never happens behind your back — the cost is only paid where the call is written:

```cpp
model.solve();
if constexpr(has_refinable_lp_status<Model>) model.refine_lp_status();
```

## Resetting the status

`get_status()` keeps reporting the last solve until the next `solve()`, even after the model has changed. `solve()` starts by resetting it, so a solve that throws leaves `unknown`, not the status of the solve before it. Code that changes a model and solves it on its own behalf, as an algorithm probing variants of the caller's model does, leaves behind the status of its own last solve, which says nothing about the model the caller gets back. `reset_status()` makes `get_status()` report `unknown` again, without a solution, as on a model never solved:

```cpp
model.reset_status();
assert(is<status::unknown>(model.get_status()));
```

It changes nothing else: the model's data stay, and so does what the solver keeps for a warm start. Every model class provides it, under the concept `has_status_reset<Model>`, which a generic algorithm can require.

## Limits

| Concept | Setter / getter | Backends |
| :--- | :--- | :--- |
| `has_time_limit` | `set_time_limit(std::chrono duration)`, `get_time_limit()` | all |
| `has_iteration_limit` | `set_iteration_limit(n)`, `get_iteration_limit()` | CPLEX, Gurobi, HiGHS *(LP and QP models)* |
| `has_node_limit` | `set_node_limit(n)`, `get_node_limit()` | CPLEX, Gurobi |
| `has_solution_limit` | `set_solution_limit(n)`, `get_solution_limit()` | CPLEX, Gurobi |
| `has_memory_limit` | `set_memory_limit(size)`, `get_memory_limit()` | CPLEX, Gurobi |

Time limits are `std::chrono` durations, so the unit is in the type and never in a comment:

```cpp
using namespace std::chrono_literals;
model.set_time_limit(10min);
model.set_time_limit(std::chrono::duration<double>(0.5));  // sub-second is fine
```

`get_time_limit()` never returns a negative duration, so `std::min(remaining, model.get_time_limit())` is a valid limit to forward on every backend. A fresh model reports the solver's own "no limit", which differs by backend: +inf, `DBL_MAX`, 1e100, 1e75 or 1e20. Writing that value back lifts a limit, and so do an infinite duration and `std::chrono::duration<double>::max()` on every backend: where a solver caps the limit, a larger value is stored as the cap. A negative limit throws, except on MOSEK, where it means no limit, and on COPT, which stores 0. On `clp_lp`, `glpk_lp` and `glpk_milp`, a NaN limit throws `solver_error` too, and a fresh model reports +inf.

Memory limits use the `memory_size` units of [`utility/memory_size.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/utility/memory_size.hpp) — `bytes`, `kilobytes`/`megabytes`/`gigabytes` (SI) and `kibibytes`/`mebibytes`/`gibibytes` (binary):

```cpp
model.set_memory_limit(mebibytes{4096u});
```

A limit is a property of the model and survives across `solve()` calls, so setting it once before a benchmark loop is enough.

An iteration limit counts simplex iterations; on `highs_qp` it also caps HiGHS's QP solver, which a quadratic objective runs instead. Barrier iterations are not counted: Gurobi's own `BarIterLimit`, for one, is set through `native_api()`. A limit larger than the solver can store (HiGHS and CPLEX keep an `int`) means no limit.

SoPlex keeps its own default clock, the CPU time of the whole process: time spent by the program's other threads counts, so its limit can run out before the wall-clock duration. Very short limits are unreliable on SoPlex whichever clock it uses: in our measurements a 50 ms limit stopped solves after about 15 ms, while a 1 s limit stopped them at 1.07 s.

Clp also counts the CPU time of the whole process: its user time, from the start of `solve()`. Clp's C API offers no wall-clock limit. Time spent by the program's other threads counts, that of a multithreaded BLAS included. In our measurements, on a Clp `devel` build linked to OpenBLAS, an 800-column dense LP under a 1 s limit stopped after 0.33 s, having used 1.2 s of user CPU time. With `OPENBLAS_NUM_THREADS=1` the two clocks agreed: a 1600-column LP stopped at 1.01 s on both. Clp checks its limit only when it refactorizes, so a stop lands near the limit rather than at it.

`clp_lp` reports a stop by its time limit as `time_limit`, and any other stop of Clp's, such as one on an iteration limit set through the native handles, as `limit_reached`. Both carry a solution when Clp's last point is primal feasible.

Cbc counts wall time. Its master branch does so by default; below Cbc 3.0, whose C API counts the CPU time of the whole process as Clp's does, `cbc_milp` sets Cbc's `timeMode` to `elapsed`. Cbc 2.10 still takes the CPU time its preprocessing used, other threads included, off the limit, although its clock has already counted that time, so a solve could stop before the limit by that much. In our measurements on the test suite's small models, under a 0.5 s limit, the shortfall stayed within about 2 ms with Ubuntu's OpenBLAS, and reached 43 ms on a first solve with conda-forge's, whose idle threads spin in user time for a while after the library loads. `cbc_milp` therefore gives Cbc 2.10 50 ms more than you set, as `glpk_milp` gives GLPK two, and `get_time_limit()` reads back the duration you set; a solve whose preprocessing uses more CPU time than that can still stop before the limit.

GLPK uses the wall clock, in whole milliseconds. MIP++ rounds a limit up to the next millisecond, and `get_time_limit()` reads back the duration you set. GLPK's branch-and-bound stops once the limit less one millisecond has passed. Before GLPK 4.63, its clock also truncates to the millisecond, which can take off almost one more. `glpk_milp` therefore gives GLPK two milliseconds more than you set, so that a stop by the limit never comes before it. `glpk_lp` reports a stop by the limit as `time_limit`, as `glpk_milp` does.

On `glpk_milp` the limit does not bound the whole solve. GLPK's MIP presolver runs without a limit, then the LP relaxation and the branch-and-bound each get the full limit. In our measurements on GLPK 5.0, a dense integer model of 500 rows and 500 columns took about 2 s in all under a 0.2 s limit, about 1.7 s of it in the presolver.

## Tolerances

| Concept | Provides | Backends |
| :--- | :--- | :--- |
| `has_feasibility_tolerance` | `get`/`set_feasibility_tolerance` | Cbc, Clp, COPT, CPLEX, GLPK *(LP only)*, Gurobi, SCIP, Xpress |
| `has_optimality_tolerance` | `get`/`set_optimality_tolerance` (the MIP gap, where applicable) | Cbc, COPT, CPLEX, Gurobi, HiGHS, SCIP, Xpress |
| `has_integrality_tolerance` | `get`/`set_integrality_tolerance` | GLPK, HiGHS, Xpress *(MILP only)* |

On a MILP model the optimality tolerance is the relative gap between the incumbent and the best bound, and a solve that stops there reports `optimal` on every backend. So `optimal` means optimal within that gap, 1e-4 by default on every backend but SCIP, where it is 0: set it to 0 where the exact optimum matters. On `cplex_lp` it is the simplex's reduced-cost tolerance instead.

HiGHS has no integrality tolerance of its own: `set_integrality_tolerance` sets its `mip_feasibility_tolerance`, which also bounds the row and bound violations its MIP solver accepts.

Two habits worth adopting in experimental code:

- **Read the tolerance instead of hard-coding `1e-9`.** Post-processing that rounds a binary (`sol[x] > 0.5`) or tests a reduced cost should be expressed against the solver's own tolerance where one is available, so the same code stays correct when you change backend or tighten the setting.
- **Report the tolerances with the results.** An optimality tolerance is part of what "optimal" meant in a table of results; the getters make dumping them into the run log a one-liner.

## Solver output

Models are quiet: building one, setting its parameters and solving it print nothing, on every backend. The solver's own log is one call away:

| Concept | Provides | Backends |
| :--- | :--- | :--- |
| `has_verbosity` | `set_verbose(bool)`, `is_verbose()` | all |

```cpp
model.set_verbose(true);  // the solver's log, on stdout
model.solve();
```

Both calls drive the solver's own switch: HiGHS `output_flag`, Gurobi `OutputFlag`, CPLEX `ScreenOutput`, COPT `Logging`, SCIP `display/verblevel`, the Cbc and Clp log levels, SoPlex `VERBOSITY`, GLPK `msg_lev`, Xpress `OUTPUTLOG` and MOSEK `MSK_IPAR_LOG`. MIP++ never redirects the standard output. Three backends need more than the switch:

- Xpress and MOSEK hand their log to a callback rather than printing it, so their models install one that prints it on `stdout`.
- GLPK prints the setup of its cover and clique cuts whatever `msg_lev` says, so a quiet `glpk_milp` also switches GLPK's terminal output off (`glp_term_out`) for the duration of `solve()` and restores it afterwards. That switch belongs to the calling thread's GLPK environment, or to the whole process on a GLPK built without thread-local storage.
- Gurobi prints its licence banner when an environment starts, before any setter could run, so the switch is set on the environment before it starts.

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
    const auto & r = model.get_status();
    rec.optimal = is_a<status::optimal>(r);
    rec.stopped = is_a<status::limit_reached>(r);
    rec.has_solution = status::solution_available(r);
    if(rec.has_solution) rec.objective = model.get_solution_value();
    return rec;
}
```

The `if constexpr` guards are the general pattern for optional capabilities; [Writing solver-generic code](../solvers/generic-code.md) develops it.

## Next

[Diagnosing infeasibility](infeasibility.md) — which bounds and constraints make a model infeasible.
