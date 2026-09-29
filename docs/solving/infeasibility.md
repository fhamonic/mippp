# Diagnosing infeasibility

A model that should have a solution comes back `infeasible`, and the status says nothing more. An irreducible infeasible subsystem (IIS) says where to look: a set of variable bounds and constraint sides that has no solution on its own, and has one as soon as any single one of them is dropped. On a model built from data, it narrows a failed instance down to the few rows and bounds whose data disagree.

MIP++ computes an IIS along two paths, a solver's native routine and a deletion filter that runs on most backends. Both return a snapshot that you query with the handles of your own variables and constraints, as you would a solution.

## A first IIS

A workshop makes chairs, tables and desks from boards of wood and hours of labour. Its orders must be filled exactly, and it has 30 hours of labour plus at most one hour of overtime, which it minimizes. `lp_type` is the model class, `highs_lp` say, and the code assumes `using namespace mippp` and `using namespace mippp::operators`, as in [A first model](../getting-started/first-model.md):

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-model"
```

The orders need 10 + 18 + 4 = 32 hours of labour, one more than the 31 available, so the solve reports `infeasible`. The arithmetic is easy to spot here and much less so among thousands of rows, which is what an IIS is for. The snapshot's `get_status(v)` and `get_status(c)` tell whether a variable or a constraint takes part in the conflict, and through which side. Two helpers print the members to `out`, any `std::ostream`: one for a single entity, and one for the workshop's own variables and constraints:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:print-member"
```

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-report"
```

The deletion filter runs on every model class of Cbc, Clp, GLPK, HiGHS, MOSEK, SCIP and SoPlex:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-deletion"
```

```text
--8<-- "test/doc_snippets/infeasibility_workshop.txt"
```

The labour row cannot hold the three orders with one hour of overtime. The wood row takes no part, nor do the bounds `make(p) >= 0`, which the orders make redundant: an IIS names only what the conflict needs. Each order is an `==` row, and its lower side is the one in conflict: the trouble is making that much, not making no more. Without any one of the five members the other four have a solution, and as this model has no other conflict, removing any one of them repairs it. Here the remedy is two hours of overtime:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-fix"
```

The model is the one you built. The filter changes bounds and sides while it works and writes every one of them back before it returns, see [The model afterwards](#the-model-afterwards).

## Two paths

| Path | Call | Where | Returns |
| :--- | :--- | :--- | :--- |
| Native routine | `model.compute_iis()` | `has_iis<M>`: `highs_lp` and `highs_qp`, with HiGHS 1.14 or later at runtime | `model_iis_t<M>` |
| Deletion filter | `compute_iis_by_deletion(model, limits)` | `iis_by_deletion_model<M>`: every model class of Cbc, Clp, GLPK, HiGHS, MOSEK, SCIP and SoPlex | a snapshot type of its own, taken through `auto` |

Both analyze the model as it currently is and never rely on an earlier solve, whose status is stale as soon as the model changes: the workshop's `solve()` only showed that there was something to explain. A feasible model is an outcome, not an error. Neither runs behind your back: each is an explicit call, and can cost many solves.

The native routine is the solver's own, HiGHS's `Highs_getIis`. On the workshop it finds the same conflict and prints the same five lines:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-native"
```

A model can have several IISs, and the two paths may then return different ones, each valid. The workshop has only one.

`has_iis` is a property of the model type, while the routine depends on the HiGHS library loaded at runtime. With a HiGHS older than 1.14, `compute_iis()` throws `solver_error`, whose message names the release it found, the library's path and the 1.14 floor.

The deletion filter is an algorithm of the library, independent of the solver. It relaxes each finite variable bound and constraint side in turn, re-solves, and keeps a side only when the rest becomes feasible without it. It runs in place, on your model, which must let it enumerate its variables and constraints and read and change every bound and side; `iis_by_deletion_model` lists the [requirements](../reference/concepts.md#infeasibility-analysis). [The deletion filter](../algorithms/deletion-filter.md) describes the algorithm, what a run changes and costs, and how to run it on constraints of your own.

Each path has its own snapshot type, so generic code picks one at compile time and hands the answer to code written for any IIS, as `print_conflict` is:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:diagnose"
```

`diagnose(model, print_conflict)` prints the same five lines on `highs_lp`, whatever the HiGHS release, and on `clp_lp`, which has only the filter.

## Reading the answer

`get_status(v)` and `get_status(c)` return a `std::variant` over the tag types of namespace `iis_status`, a hierarchy like that of the [solve status](status-and-limits.md#the-solve-status):

```text
absent
member
├── member_lower    the lower bound of a variable, or the lower side of a row
├── member_upper    the upper bound, or the upper side
└── member_both     both of them
```

Test membership with `is_a<iis_status::member>(s)`, which accepts every refinement. `is<iis_status::member>(s)` tests for the plain tag alone, and does not compile on a variant that does not list it.

The side matters on anything with two: a variable's bounds, a ranged row, an `==` row. The workshop's orders conflict through their lower sides. `member_both` means that both sides are needed, for instance when they cross, a lower bound above the upper bound, or when integrality [needs both sides of a row](#lp-or-milp). Plain `member` is for a row whose side a routine cannot name. Neither path reports it today, since both name every side, but code meant for any IIS handles it, as `member_side` does.

Iterating your own variable and constraint families, as `print_conflict` does, gives each member your name for it. Code that did not build the model lists the members through `model.variables()` and `model.constraints()`, which every model class provides:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:member-rows"
```

On the workshop it returns the labour row and the three order rows. `num_variable_members()` and `num_constraint_members()` count the members without a loop: one variable and four rows there. A query is a lookup in the snapshot and never calls the solver.

## Outcomes and reasons

`get_outcome()` says how far the answer goes, and `get_reason()`, a `std::optional<iis_reason>`, why it stopped short:

| `iis_outcome` | Members | Meaning |
| :--- | :--- | :--- |
| `irreducible` | an IIS | The members conflict, and without any one of them the others have a solution. |
| `not_proven_minimal` | a conflict, perhaps larger than needed | Not every member could be proven necessary: a limit stopped the run, a trial was inconclusive, or the native routine could not tell. The deletion filter's members are the last subset it proved infeasible; a native routine's are those it could not rule out. |
| `feasible` | none | The model is feasible as it is. |
| `undetermined` | none | Nothing was proven: the run stopped before its first proof, its first solve was inconclusive, or the native routine returned no answer. |

`irreducible` with no member means that the background alone is infeasible: integrality and special constraints, which neither path counts as candidates, conflict on their own, without any bound or side.

| `iis_reason` | Set when |
| :--- | :--- |
| `solve_limit` | the filter made `max_solves` solves |
| `time_limit` | the filter's deadline passed, or the model's time limit stopped a native call |
| `cancelled` | a stop was requested through the filter's `stop_token` |
| `inconclusive_trial` | a trial of the filter ended without proving either infeasibility or a feasible point, as a numerical failure does |

When several limits are reached at once, a stop request is reported before the deadline, and the deadline before the solve count. An empty reason means that no limit stopped the run. It comes with every `irreducible` and `feasible` answer, and with a native answer that the routine itself could not complete: there is then no limit to raise.

## Limits

The filter's second argument, `iis_limits`, bounds the whole run:

- `max_solves`: the number of `solve()` calls;
- `time_limit`: one duration for the whole call, which becomes a single deadline when the call starts. A negative or NaN duration throws `std::invalid_argument`, and an infinite one, the default, means no deadline;
- `stop_token`: a `std::stop_token`, from a `std::stop_source` or the `std::jthread` running the analysis; a stop requested from another thread ends the run before its next trial.

Here `stop` is such a token:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:limits"
```

A run stopped by a limit keeps what it proved, which is why the snippet prints a `not_proven_minimal` answer too. With a budget of three solves, the workshop's run ends `not_proven_minimal` with `solve_limit`, and its members include those of the IIS.

Limits are checked between trials, and a trial that has started runs to its end. On a model with a time limit (`has_time_limit`) and under a finite `time_limit`, each trial gets the time that remains as its own time limit, never more than the limit you had set, which is restored afterwards. `clp_lp`, `glpk_lp` and `glpk_milp` have no time limit, so there one trial can run past the deadline, as `cbc_milp`'s root LP can: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) in the notable limitations. With the default limits, the filter never reads or writes the model's time limit.

`compute_iis()` takes no argument. The model's own time limit, set by `set_time_limit`, bounds each call as a fresh budget, so `solve()` followed by `compute_iis()` may take twice the limit. HiGHS applies an IIS time limit of its own during its search, and `compute_iis()` copies the model's limit there for the call and restores it afterwards. That limit bounds the search only, not the checks HiGHS runs before and after it, so a call can return after the limit. A stop may also leave no answer: the outcome is then `undetermined`, with `time_limit` as the reason.

## LP or MILP

An IIS explains the problem that the model class solves. On a `*_milp` model it explains the MIP: integrality is part of the background, never a member, so rows and bounds can conflict where the LP relaxation has a solution. On a `*_lp` model it explains the LP. Ten tonnes cannot travel in full three-tonne trucks:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:trucks"
```

On `highs_milp` the answer is `irreducible`, with `load` a member through both sides, since `3 * trucks <= 10` and `3 * trucks >= 10` each have an integer solution, three trucks or four. The bounds of `trucks` are not members. The same row over a continuous variable is feasible, and the filter says so on `highs_lp`.

This is why `highs_milp` has no `compute_iis()`: HiGHS's routine explains the LP relaxation of a MIP, which answers another question. `highs_milp` runs the deletion filter, whose trials are MIP solves.

## The model afterwards

Both paths leave the model's data as they found them. The filter changes bounds and sides during its trials, and sets the objective to zero so that each trial is a feasibility problem. It saves each item first and writes it back on every exit, exceptions included: bounds, sides, the objective from a copy (the Hessian too, on `highs_qp`) and the time limit it forwarded. If writing one item back fails, it still restores the others, then throws the first error. `compute_iis()` restores the solver options it sets.

The status is another matter. After any run that solved, the solver holds the solution of a trial rather than of your model, so `get_status()` reports `unknown`, after an exception too. A run answered without a solve leaves the status as it was: one stopped before its first trial, or one on a model without variables, which the filter decides from the constraint sides alone. `compute_iis()` makes the status `unknown` on every call, whether it returns or throws. Call `solve()` again before reading a solution, as the workshop's remedy does.

## A snapshot of the model as it was

A snapshot is computed once, when the call returns, and later changes to the model do not reach it. It is keyed by handle id: the handles of entities that remain keep their answers after a `remove_variable`, but a variable added later may reuse a removed variable's id, and would then read the removed variable's entry. Compute a new IIS after changing the model.

## Support by model

| Model | `has_iis` | `iis_by_deletion_model` | Notes |
| :--- | :--- | :--- | :--- |
| `highs_lp`, `highs_qp` | yes, with HiGHS 1.14 or later | yes | HiGHS repairs sides crossed by less than its tolerance: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `highs_milp` | no: the routine explains the relaxation | yes | as above |
| `clp_lp`, `glpk_lp` | no | yes | no time limit: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `glpk_milp` | no | yes | no time limit, and endless branching on some integer rows: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) and [Integrality proofs](../solvers/index.md#limitation-integrality-proofs) |
| `cbc_milp` | no | yes | wrong answers of Cbc 2.10 on integer rows, and root LPs past the deadline: see [Integrality proofs](../solvers/index.md#limitation-integrality-proofs) and [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `scip_milp` | no | yes | can throw on an infeasible model with binary columns: see [Deletion filter on SCIP binaries](../solvers/index.md#limitation-scip-binaries) |
| `mosek_lp`, `mosek_milp`, `soplex_lp` | no | yes | |
| Gurobi, CPLEX, Xpress and COPT models | no | no | |

## Native changes

!!! warning "Changes made through the native handles are outside the guarantee"
    The [warning on the native handles](../solvers/index.md#limitation-native-handles) holds for both paths. Bounds, rows or special constraints added through `native_model()` and `native_api()` are background that MIP++ does not know, and an IIS computed over them loses its guarantee. The filter's save and restore and the enumeration of `variables()` and `constraints()` do not see native changes either.

## Next

[Solutions, duals and reduced costs](solutions.md) — getting the numbers back out.
