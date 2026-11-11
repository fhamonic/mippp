# Watching a solve

A branch-and-bound can run for hours, and `solve()` says nothing until it returns. On the MILP models of Gurobi, CPLEX, Xpress, COPT, MOSEK, HiGHS, SCIP and GLPK, a progress callback reads the incumbent and the bound while the search runs, and can end it.

## The callback

`has_progress_callback<Model>` promises `set_progress_callback(f)`, where `f` takes the backend's `progress_callback_handle`. Take it by `auto &`, as for a [candidate-solution callback](../algorithms/branch-and-cut.md):

```cpp
--8<-- "test/doc_snippets/progress.cpp:includes"

--8<-- "test/doc_snippets/progress.cpp:report"
```

The handle has:

| Member | What it does |
|---|---|
| `get_solution_value()` | The best incumbent so far, offset included. Until there is one, `+inf` when minimizing and `-inf` when maximizing. |
| `get_dual_bound()` | The best proven bound so far, offset included. Until there is one, `-inf` when minimizing and `+inf` when maximizing. |
| `interrupt()` | Ends the solve. See [Stopping a solve](#stopping-a-solve). |
| `get_node_count()` | The nodes the solver has counted so far, on every model above but `copt_milp`, whose callback data has no such count: test `has_node_count<progress_callback_handle_t<Model>>`. |

Elapsed time is not on the handle. Read `std::chrono::steady_clock` in the callback, as above: it costs nothing and means the same on every solver.

## What to expect

- **How often it runs is the solver's business.** On one multidimensional knapsack we measured about 27 calls a second on Xpress, 200 on CPLEX, 1000 on SCIP and 100 000 on Gurobi, so the snippet above throttles itself. A call does not mean anything changed.
- **It can stay silent.** The solver calls when it gets to it: COPT went 2 s between two calls on that knapsack, and nothing promises a call during presolve or a long root node.
- **The last call is not the final state.** Once `solve()` returns, read the model.
- **It may never run on a model without integer variables.** Solving such a model with the callback set is fine, on every backend.
- **It may run on a worker thread of the solver.** We saw 16 distinct threads on CPLEX and 32 on Xpress. It never runs twice at once, nor at the same time as another callback of the same model, but `thread_local` state in it is not safe.
- **Keep it short.** CPLEX holds a global lock across the call, and CPLEX, Xpress and COPT a mutex of ours, shared with the other callbacks of the model, so a slow callback blocks the search there.
- **Do not call the model from inside it**, and do not let an exception leave it: that is undefined behavior, since the exception would cross the solver's C frames.
- **It leaves the search alone.** At one thread, the node count is the same with and without it on every solver that reports one, which a [candidate-solution callback](../algorithms/branch-and-cut.md#watching-a-search-is-another-callback) does not promise.

A progress callback and a candidate-solution callback can be set on the same model. Neither runs during `compute_iis()`. The [deletion filter](../algorithms/deletion-filter.md) solves through `solve()`, so a progress callback runs in each of its trials, and an `interrupt()` there stops that trial.

## Stopping a solve

`interrupt()` ends the solve as `status::interrupted`, which carries whether a solution is available like the other [stopped statuses](status-and-limits.md). Only that solve: the next `solve()` runs normally, and resumes the search on the solvers that can.

```cpp
--8<-- "test/doc_snippets/progress.cpp:target"
```

## Support by model

| Model | Native hook | Node count |
|---|---|---|
| `gurobi_milp` | `GRB_CB_MIP` | yes |
| `cplex_milp` | `CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS` | yes |
| `xpress_milp` | `XPRSaddcbmiplog`, every 100 nodes unless you change `MIPLOG`, and `XPRSaddcbintsol` | yes |
| `copt_milp` | `COPT_CBCONTEXT_MIPNODE` and `COPT_CBCONTEXT_INCUMBENT` | no |
| `mosek_milp` | `MSK_CALLBACK_IM_MIO` and `MSK_CALLBACK_NEW_INT_MIO` | yes |
| `highs_milp` | `kCallbackMipInterrupt` | yes |
| `scip_milp` | an event handler on solved nodes and new best solutions | yes |
| `glpk_milp` | `mip.cb_func`, on `GLP_ISELECT` and `GLP_IBINGO`. The handle reads a bound and a node count the model itself has no getter for. | yes |

`cbc_milp` has none: the C API of Cbc 2.10 reaches no callback from its search. The LP models have none either.

## Without a callback

- `set_verbose(true)` prints the solver's own log, which is often enough for a person watching.
- A short time limit and a second `solve()` read the model between the two, but only where the second solve carries on. Gurobi, CPLEX and SCIP resume the search. Xpress, COPT, MOSEK, HiGHS, GLPK and Cbc start the search over and get no further.
