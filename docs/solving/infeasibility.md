# Diagnosing infeasibility

A model that should have a solution comes back `infeasible`, and the status says nothing more. An irreducible infeasible subsystem (IIS) says where to look: a set of variable bounds and constraint sides that has no solution on its own, and has one as soon as any single one of them is dropped. On a model built from data, it narrows a failed instance down to the few rows and bounds whose data disagree.

MIP++ computes an IIS along two paths, a solver's native routine and a deletion filter that runs on most backends. Both return a snapshot that you query with the handles of your own variables and constraints, as you would a solution.

## A first IIS

A workshop makes chairs, tables and desks from boards of wood and hours of labour. Its orders must be filled exactly, and it has 30 hours of labour plus at most one hour of overtime, which it minimizes. The code includes the backend's header, HiGHS's here, and `mippp/utility/iis_by_deletion.hpp`, which provides the deletion filter used below, `compute_iis_by_deletion`, with its `iis_limits`. The solver headers do not include it:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:includes"
```

`lp_type` is the model class, `highs_lp` say, and the code assumes `using namespace mippp` and `using namespace mippp::operators`, as in [A first model](../getting-started/first-model.md):

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-model"
```

The orders need 10 + 18 + 4 = 32 hours of labour, one more than the 31 available, so the solve reports `infeasible`. The arithmetic is easy to spot here and much less so among thousands of rows, which is what an IIS is for. The snapshot's `get_status(v)` and `get_status(c)` tell whether a variable or a constraint takes part in the conflict, and through which side. Two helpers print the members: one for a single entity, to any `std::ostream`, and one for the workshop's own variables and constraints, to `std::cout` here:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:print-member"
```

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-report"
```

The deletion filter, `compute_iis_by_deletion` from the header included above, runs on every model class of Cbc, Clp, COPT, CPLEX, GLPK, HiGHS, MOSEK, SCIP, SoPlex and Xpress:

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

[`examples/infeasible_transportation/`](https://github.com/fhamonic/mippp/blob/main/examples/infeasible_transportation/main.cpp) is a complete, runnable program along these lines: a transportation plan whose conflict the deletion filter finds on any of its backends, and the native routine too where the model class has one.

## Two paths

| Path | Call | Where | Returns |
| :--- | :--- | :--- | :--- |
| Native routine | `model.compute_iis()` | `has_iis<M>`: `gurobi_lp`, `gurobi_milp`, `cplex_lp`, `cplex_milp`, `xpress_lp`, `xpress_milp`, `copt_lp` and `copt_milp`, and `highs_lp` and `highs_qp` with HiGHS 1.14 or later at runtime | `model_iis_t<M>` |
| Deletion filter | `compute_iis_by_deletion(model, limits)`, from `mippp/utility/iis_by_deletion.hpp` | `iis_by_deletion_model<M>`: every model class of Cbc, Clp, COPT, CPLEX, GLPK, HiGHS, MOSEK, SCIP, SoPlex and Xpress | a snapshot type of its own, taken through `auto` |

Both analyze the model as it currently is and never rely on an earlier solve, whose status is stale as soon as the model changes: the workshop's `solve()` only showed that there was something to explain. A feasible model is an outcome, not an error. Neither runs behind your back: each is an explicit call, and can cost many solves.

The native routine is the solver's own: HiGHS's `Highs_getIis`, Gurobi's `GRBcomputeIIS`, CPLEX's `CPXrefineconflictext`, Xpress's `XPRSiisfirst` or COPT's `COPT_ComputeIIS`. On the workshop each finds the same conflict. `highs_lp`, `xpress_lp` and `copt_lp` print the same five lines, while on `gurobi_lp` and `cplex_lp` the three order rows print `side not named`, see [Reading the answer](#reading-the-answer):

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-native"
```

A model can have several IISs, and the two paths may then return different ones, each valid. The workshop has only one.

`has_iis` is a property of the model type, while the routine depends on the HiGHS library loaded at runtime. With a HiGHS older than 1.14, `compute_iis()` throws `solver_error`, whose message names the release it found, the library's path and the 1.14 floor. To load another HiGHS, set `MIPPP_HIGHS_LIBRARY` to the full path of its library file, see [How solver libraries are found](../solvers/index.md#how-solver-libraries-are-found). The routines of Gurobi, CPLEX, Xpress and COPT exist throughout the releases MIP++ validates, and their api objects bind them like any other function, so a library that loads has them.

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

Test membership with `is_a<iis_status::member>(s)`, which accepts every refinement. `is<iis_status::member>(s)` tests for the plain tag alone, and does not compile on a variant that does not list it. `is_a<iis_status::member_both>(s)` likewise needs a variant with a tag derived from `member_both`, which the row status of the Gurobi and CPLEX routines lacks, so code meant for any IIS reads the side through a visitor, as `member_side` above does.

The side matters on anything with two: a variable's bounds, a ranged row, an `==` row. The workshop's orders conflict through their lower sides. `member_both` means that both sides are needed, for instance when they cross, a lower bound above the upper bound, or when integrality [needs both sides of a row](#lp-or-milp). Plain `member` is for a row whose side a routine cannot name. The routines of Gurobi and CPLEX report it for every `==` row, and CPLEX's for every ranged row too, since they flag a row's membership only and an inequality row takes the side of its sense. COPT's routine flags one side of a two-sided row, or one bound of an integer column, on a MIP where both may be needed, so `copt_milp` reports such a row or column as a plain `member`, while on `copt_lp` every member has its side. Xpress's routine names the side it kept of an `==` row, and both sides where integrality needs them. HiGHS's routine and the deletion filter name every side. Code meant for any IIS handles the plain tag, as `member_side` does.

Iterating your own variable and constraint families, as `print_conflict` does, gives each member your name for it. Code that did not build the model lists the members through `model.variables()` and `model.constraints()`, which every model class provides:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:member-rows"
```

On the workshop it returns the labour row and the three order rows. To print such handles, name the variables and constraints as you add them: `get_variable_name(v)` and `get_constraint_name(c)` read the names back on models with `has_named_variables` and `has_named_constraints`, see [Names](../modeling/variables.md#names) for variables and [Constraint families](../modeling/expressions.md#constraint-families) for constraints. `num_variable_members()` and `num_constraint_members()` count the members without a loop: one variable and four rows there. A query is a lookup in the snapshot and never calls the solver.

## Repairing the model

An IIS says where the data disagree, not how to settle it. One repair always removes the conflict: relax every side the answer names, the lower side of a `member_lower` row or bound to `-infinity()`, the upper side of a `member_upper` one to `infinity()`, and both sides of a `member_both` or a plain `member`:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:relax-members"
```

It is one repair among others, and seldom the one you want. On the workshop it frees the labour row, the overtime and the lower sides of the three orders, and the model then solves with no overtime at all, since no order binds any more. Dropping any single member already removes a conflict, and which one to change, and by how much, is a decision on the data that the IIS leaves to you: the remedy above moved one member by one hour.

A model can hold several conflicts, and an IIS explains one of them. Suppose that the workshop's labour comes from one team per product, whose hours cannot be shared:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:teams-model"
```

The chairs need 10 hours of a team that has 8, and the desks 4 of a team that has 3: two conflicts, with no member in common. A loop that relaxes the members of each answer and runs the filter again explains one conflict per round, until an answer has no member left:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:repair-loop"
```

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:teams-repair"
```

```text
--8<-- "test/doc_snippets/infeasibility_repair.txt"
```

The loop returns `feasible` once no conflict remains. An answer can also have no member because the run proved nothing, `undetermined`, or because integrality and special constraints conflict on their own, `irreducible`, which no relaxed side repairs: the loop then returns that outcome. It changes the model for good, so save the sides you want back before calling it, as `rerun_on_members` does under [Limits](#limits).

## Outcomes and reasons

`get_outcome()` says how far the answer goes, and `get_reason()`, a `std::optional<iis_reason>`, why it stopped short:

| `iis_outcome` | Members | Meaning |
| :--- | :--- | :--- |
| `irreducible` | an IIS | The members conflict, and without any one of them the others have a solution. |
| `not_proven_minimal` | a conflict, perhaps larger than needed | Not every member could be proven necessary: a limit stopped the run, a trial was inconclusive, or the native routine could not tell. The deletion filter's members are the last subset it proved infeasible; a native routine's are those it could not rule out. |
| `feasible` | none | The model is feasible as it is. |
| `undetermined` | none | Nothing was proven: the run stopped before its first proof, its first solve was inconclusive, or the native routine returned no answer. |

`irreducible` with no member means that the background alone is infeasible: integrality and special constraints, which neither path counts as candidates, conflict on their own, without any bound or side. Indicator and SOS constraints are background for every native routine as for the filter: never members, and the rows and bounds an answer names conflict against them.

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

Such an answer of the filter is a start. Run the filter again with a larger budget, or narrow the next run down to the members, which the filter proved to conflict: relax every other side, since a side at infinity is no candidate, run again, and write the relaxed sides back. That run makes at most one solve per member side plus one, and the IIS it finds among the members is an IIS of the whole model:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:narrow-partial"
```

On the workshop, `rerun_on_members(model, iis)`, passing the answer of the three-solve run, has 11 candidates rather than 13, makes 12 solves and prints the five lines of the complete run.

Limits are checked between trials, and a trial that has started runs to its end. On a model with a time limit (`has_time_limit`) and under a finite `time_limit`, each trial gets the time that remains as its own time limit, never more than the limit you had set, which is restored afterwards. `clp_lp`, `glpk_lp` and `glpk_milp` have no time limit, so there one trial can run past the deadline, and on `cbc_milp` the time limit did not bound a trial's root LP on the Cbc build where this was measured: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) in the notable limitations. With the default limits, the filter never reads or writes the model's time limit.

`compute_iis()` takes no argument. The model's own time limit, set by `set_time_limit`, bounds each call as a fresh budget, so `solve()` followed by `compute_iis()` may take twice the limit. HiGHS applies an IIS time limit of its own during its search, and `compute_iis()` copies the model's limit there for the call and restores it afterwards. That limit bounds the search only, not the checks HiGHS runs before and after it, so a call can return after the limit. A stop may also leave no answer: the outcome is then `undetermined`, with `time_limit` as the reason. The iteration limit set by `set_iteration_limit`, the only other limit of `highs_lp` and `highs_qp`, does not stop the routine: a call answers in full under a limit that stops `solve()`.

On `cplex_lp` and `cplex_milp`, CPLEX's routine reads the model's time limit itself and stops under it, returning up to 15 ms late (measured on CPLEX 22.1.1 and 22.1.2). A stop never carries a partial answer: at a stop CPLEX flags every candidate it has not examined as a possible member, on a feasible model too, and a stop on the node limit was measured to exclude candidates that the conflict needs, so a stopped call is `undetermined` with no member, and `time_limit` as its reason when the time limit stopped it. The model's other limits stop the routine the same way, with no reason: the iteration limit of `cplex_lp`, and the node limit of `cplex_milp`, whose stop can come seconds late on a hard model. CPLEX's own memory, objective and deterministic-time limits, set through the [native handles](../solvers/index.md#limitation-native-handles), stop it too.

On `gurobi_lp` and `gurobi_milp`, Gurobi's routine reads the model's time limit itself and returns within a few milliseconds of it (measured on Gurobi 11.0.3, 12.0.1 and 13.0.2). A stop leaves either no subsystem, `undetermined`, or one without the proof of minimality, `not_proven_minimal`; either way the reason is `time_limit` when the time measured around the call reached the limit, and empty otherwise. A stop can still answer in full on a very small conflict, since Gurobi's cheap checks run before its clock, and on a model that an earlier `solve()` proved infeasible. The memory limit of `set_memory_limit` stops the routine too, and on `gurobi_lp` so does the iteration limit of `set_iteration_limit`, with no answer and no reason.

On `xpress_lp` and `xpress_milp`, Xpress's routine reads the model's time limit itself and returns within about 10 ms of it, before or after (measured on Xpress 45.01 and 47.01). A stop with no subsystem is `undetermined`, one with a subsystem `not_proven_minimal`, in both cases with `time_limit` when the time measured around the call reached the limit within 20 ms, and with no reason otherwise, as after an interrupt or an iteration limit set through the native handles. A zero limit stops every call before any answer, trivial conflicts and feasible models included. No other limit of MIP++ reaches the routine on Xpress.

On `copt_lp` and `copt_milp`, COPT's routine reads the model's time limit itself, which COPT does not document. One budget covers the whole call, because `compute_iis()` also solves the model: on `copt_milp` before the search, since COPT answers a whole-model conflict on an unsolved feasible MIP, and on `copt_lp` after a search that found nothing, since COPT returns the same code on a feasible model and on a failure; the second step gets what remains of the budget. A stop may return late, since the routine has phases that a limit does not interrupt and that grow with the model (44 ms late on a 60000 by 20000 LP under a 0.02 s limit, measured on COPT 8.0.5), with no subsystem, `undetermined` with `time_limit`, or with a subsystem it did not prove minimal, `not_proven_minimal`; a stopped answer whose counts and flags disagree is `undetermined`. The node limit of `copt_milp` stops the solve that precedes the search, `undetermined` with no reason, or `feasible` when that solve found a feasible point; it does not stop the search itself.

## LP or MILP

An IIS explains the problem that the model class solves. On a `*_milp` model it explains the MIP: integrality is part of the background, never a member, so rows and bounds can conflict where the LP relaxation has a solution. On a `*_lp` model it explains the LP. Ten tonnes cannot travel in full three-tonne trucks:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:trucks"
```

On `highs_milp` the answer is `irreducible`, with `load` a member through both sides, since `3 * trucks <= 10` and `3 * trucks >= 10` each have an integer solution, three trucks or four. The bounds of `trucks` are not members. The same row over a continuous variable is feasible, and the filter says so on `highs_lp`.

This is why `highs_milp` has no `compute_iis()`: HiGHS's routine explains the LP relaxation of a MIP, which answers another question. `highs_milp` runs the deletion filter, whose trials are MIP solves. The routines of Gurobi, CPLEX, Xpress and COPT explain the MIP, so their `*_milp` classes have `compute_iis()`: on the trucks each answers `irreducible`, with `load` a plain `member` on `gurobi_milp`, `cplex_milp` and `copt_milp` and a member through both sides on `xpress_milp`, and each of their `*_lp` classes answers `feasible`.

## The model afterwards

Both paths leave the model's data as they found them. The filter changes bounds and sides during its trials, and sets the objective to zero so that each trial is a feasibility problem. It saves each item first and writes it back on every exit, exceptions included: bounds, sides, the objective from a copy (the Hessian too, on `highs_qp`) and the time limit it forwarded. If writing one item back fails on a normal return, it still restores the others and then throws the first error; after an exception from a trial, it restores what it can and lets that exception through. `compute_iis()` restores the solver options it sets: HiGHS's IIS options, the Xpress option that keeps integrality and special constraints in the background, the Gurobi attributes that keep its special constraints in the conflict for the call, and the time limit COPT's second step gets; on CPLEX it sets none, the candidates being arguments of the call. CPLEX keeps a completed answer until the model's data change, so a second `compute_iis()` on an unchanged model returns at once.

The status is another matter. After any run that solved, the solver holds the solution of a trial rather than of your model, so `get_status()` reports `unknown`, after an exception too. A run answered without a solve leaves the status as it was: one stopped before its first trial, or one on a model without variables, which the filter decides from the constraint sides alone. `compute_iis()` makes the status `unknown` on every call, whether it returns or throws, and the solver's own state is no longer your solve's either: the routines of Gurobi and Xpress solve an unsolved model or overwrite the solver's status, CPLEX's replaces its status with the conflict's, and on COPT `compute_iis()` resets the solver first, dropping its held solution, then solves the model as [Limits](#limits) describes, on `copt_milp` without your candidate-solution callback, which is detached for the call and reattached afterwards. Call `solve()` again before reading a solution, as the workshop's remedy does.

## A snapshot of the model as it was

A snapshot is computed once, when the call returns, and later changes to the model do not reach it. It is keyed by handle id: the handles of entities that remain keep their answers after a `remove_variable`, but a variable added later may reuse a removed variable's id, and would then read the removed variable's entry. Compute a new IIS after changing the model.

## Support by model

| Model | `has_iis` | `iis_by_deletion_model` | Notes |
| :--- | :--- | :--- | :--- |
| `highs_lp`, `highs_qp` | yes, with HiGHS 1.14 or later | yes | HiGHS repairs sides crossed by less than its tolerance: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `highs_milp` | no: the routine explains the relaxation | yes | as above |
| `clp_lp`, `glpk_lp` | no | yes | no time limit: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `glpk_milp` | no | yes | no time limit, and endless branching on some integer rows: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) and [Integrality proofs](../solvers/index.md#limitation-integrality-proofs) |
| `cbc_milp` | no | yes | an [experimental](../solvers/index.md#the-backends) backend, with wrong answers of Cbc 2.10 on integer rows, and root LPs past the deadline (measured on a Cbc `devel` build): see [Integrality proofs](../solvers/index.md#limitation-integrality-proofs) and [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `scip_milp` | no | yes | can throw on an infeasible model with binary columns: see [Deletion filter on SCIP binaries](../solvers/index.md#limitation-scip-binaries) |
| `mosek_lp`, `mosek_milp`, `soplex_lp` | no | yes | |
| `gurobi_lp`, `gurobi_milp` | yes | no: Gurobi ranges a row through a slack column, so its models have no modifiable row bounds | native: `==` rows are members without a side, see [Reading the answer](#reading-the-answer); indicator constraints are background; the bounds of a binary variable are never members, and the iteration limit of `gurobi_lp` stops the routine with no answer, see [Native IIS on Gurobi](../solvers/index.md#limitation-gurobi-iis) |
| `cplex_lp`, `cplex_milp` | yes | yes | native: rows are members without a side, see [Reading the answer](#reading-the-answer), a stopped call has no answer, see [Limits](#limits), and indicator constraints are background; both: a row whose sides cross cannot be built, the setter throws, see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints) |
| `xpress_lp`, `xpress_milp` | yes | yes | native: a column whose bounds cross or hold no integer makes the routine refuse the whole search, see [Native IIS on Xpress](../solvers/index.md#limitation-xpress-iis); both: a row whose sides cross cannot be built, the setter throws, see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints) |
| `copt_lp`, `copt_milp` | yes | yes | native: `compute_iis()` also solves the model, see [Limits](#limits); on `copt_milp` two-sided rows and integer columns are members without a side, see [Reading the answer](#reading-the-answer), and COPT's answer on a MIP may leave out a bound the conflict needs, see [Native IIS on COPT](../solvers/index.md#limitation-copt-iis) |

Every model class has a path, and all but Gurobi's have both. `gurobi_lp` and `gurobi_milp` have the native routine only: Gurobi implements a ranged row through a slack column, so its models have no modifiable row bounds, which the filter needs to relax a side. What a routine offers beyond an IIS, such as forcing a candidate in or out of the conflict or enumerating several IISs, stays reachable through the solver's C API on the objects `native_model()` returns, with `native_id(v)` and `native_id(c)` for the index each of your handles has there, outside MIP++, see [Native changes](#native-changes). The engine of [The deletion filter](../algorithms/deletion-filter.md#the-engine) runs over an oracle of your own on any backend.

## Native changes

!!! warning "Changes made through the native handles are outside the guarantee"
    The [warning on the native handles](../solvers/index.md#limitation-native-handles) holds for both paths. Bounds, rows or special constraints added through `native_model()` and `native_api()` are background that MIP++ does not know, and an IIS computed over them loses its guarantee. The filter's save and restore and the enumeration of `variables()` and `constraints()` do not see native changes either.

## Next

[Solutions, duals and reduced costs](solutions.md) — getting the numbers back out.
