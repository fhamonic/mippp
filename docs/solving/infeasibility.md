# Diagnosing infeasibility

A model that should have a solution comes back `infeasible`, and the status says nothing more. An irreducible infeasible subsystem (IIS) says where to look: a set of variable bounds and constraint sides that has no solution on its own, and has one as soon as any single one of them is dropped. On a model built from data, it narrows a failed instance down to the few rows and bounds whose data disagree.

MIP++ computes an IIS along two paths, a solver's native routine and a deletion filter that runs on every model class. Both return a snapshot that you query with the handles of your own variables and constraints, as you would a solution.

## A first IIS

A workshop makes chairs, tables and desks. A chair takes 2 boards and 1 hour of labour, a table 5 boards and 3 hours, a desk 4 boards and 2 hours. It has 60 boards and 30 hours, plus at most one hour of overtime, which it minimizes, and orders for 10 chairs, 6 tables and 2 desks to fill exactly. This program builds the model, finds it infeasible, asks the deletion filter for an IIS, prints its members and repairs the model:

```cpp
#include <iostream>
#include <string_view>

--8<-- "test/doc_snippets/infeasibility.cpp:includes"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;

--8<-- "test/doc_snippets/infeasibility.cpp:print-member"

int main() {
    --8<-- "test/doc_snippets/infeasibility.cpp:workshop-model"

    --8<-- "test/doc_snippets/infeasibility.cpp:workshop-report"
    --8<-- "test/doc_snippets/infeasibility.cpp:workshop-deletion"

    --8<-- "test/doc_snippets/infeasibility.cpp:workshop-fix"
    return 0;
}
```

It prints:

```text
--8<-- "test/doc_snippets/infeasibility_workshop.txt"
```

The orders need 10 + 18 + 4 = 32 hours of labour and the workshop has 31, so the first `solve()` reports `infeasible`. That is easy to see here and hard among thousands of rows, which is what an IIS is for. The first five lines are the conflict and nothing else: the upper side of the labour row, the upper bound of `overtime` and the lower side of each order, since the trouble with an `==` row is making that much, not making no more. Without any one of the five the other four have a solution: that is what irreducible means. The wood row is absent, since the orders use 58 of the 60 boards, and so are the bounds `>= 0` of `chairs`, `tables` and `desks`, which the orders make redundant.

`compute_iis_by_deletion`, from `mippp/utility/iis_by_deletion.hpp`, which the solver headers do not include, is the deletion filter. A program usually gets there after `is_a<status::infeasible>(model.get_status())`, but the filter does not need that solve: it analyzes the model as it is. It returns a snapshot, whose `get_outcome()` says how the run ended, as a `std::variant` over the tags of namespace `iis_outcome`: `is<iis_outcome::irreducible>` holds on an IIS, a run cut short can still hold a conflict whose members are worth printing, and a feasible model is an outcome, not an error, see [How a run ends](#how-a-run-ends).

`get_status(v)` and `get_status(c)` say whether a variable or a constraint takes part, and through which side, as a `std::variant` over the tags of namespace `iis_status`. `iis_status::sides_of(s)` reads such a status as an `iis_sides`, three flags: `lower` and `upper` name the sides that take part, and `whole` marks a member that the answer names as one unit, all its finite sides together. `print_member` prints a line per side, or one line, `member`, for a member named whole, and nothing for an entity that takes no part, see [Reading the answer](#reading-the-answer). `print_conflict` queries the snapshot with the handles of the model, and so gives each member your name for it.

An IIS says where the data disagree, not how to settle it. Relaxing any single member removes this conflict, and as the workshop has no other, repairs the model: here, two hours of overtime, which the last line reports. The model is the one you built: the filter changes bounds and sides while it works and writes every one of them back before it returns, see [The model afterwards](#the-model-afterwards), and leaves the status `unknown`, so the program solves again before reading the solution.

The filter runs on every model class, so `clp_lp` with Clp's header runs the same program. Several solvers also have a routine of their own, `model.compute_iis()`, see [Two paths](#two-paths). The two paths may return different types, which is why `print_conflict` is a generic lambda: the later sections hand it the answers of other calls. `print_member` is a template because, within one answer too, a variable's status and a row's may be different variants, as on Gurobi, CPLEX and `xpress_milp`.

[`examples/infeasible_transportation/`](https://github.com/fhamonic/mippp/blob/main/examples/infeasible_transportation/main.cpp) is a complete, runnable program along these lines: a transportation plan whose conflict the deletion filter finds on any of its backends.

## Two paths

| Path | Call | Where | Returns |
| :--- | :--- | :--- | :--- |
| Native routine | `model.compute_iis()` | `has_iis<M>`: `gurobi_lp`, `gurobi_milp`, `cplex_lp`, `cplex_milp`, `xpress_lp`, `xpress_milp`, `copt_lp` and `copt_milp`, and `highs_lp` and `highs_qp` with HiGHS 1.14 or later at runtime | `model_iis_t<M>` |
| Deletion filter | `compute_iis_by_deletion(model)`, `(model, limits)` with an `iis_limits`, see [Limits](#limits), or `(model, within)` and `(model, within, limits)` to narrow an answer, see [Narrowing an answer](#narrowing-an-answer), from `mippp/utility/iis_by_deletion.hpp` | `iis_by_deletion_model<M>`: every model class | `iis_by_deletion_t<M>` |

Both analyze the model as it currently is and never rely on an earlier solve, whose status is stale as soon as the model changes: the workshop's `solve()` only showed that there was something to explain. A feasible model is an outcome, not an error. Neither runs behind your back: each is an explicit call, and can cost many solves.

The native routine is the solver's own: HiGHS's `Highs_getIis`, Gurobi's `GRBcomputeIIS`, CPLEX's `CPXrefineconflictext`, Xpress's `XPRSiisfirst` or COPT's `COPT_ComputeIIS`. On the workshop each finds the same conflict. `highs_lp`, `xpress_lp` and `copt_lp` print the same five lines, while on `gurobi_lp` and `cplex_lp` the three order rows print `member`, see [Reading the answer](#reading-the-answer), and the filter can name their sides, see [Narrowing an answer](#narrowing-an-answer):

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:workshop-native"
```

A model can have several IISs, and the two paths may then return different ones, each valid. The workshop has only one.

`has_iis` is a property of the model type, while the routine depends on the HiGHS library loaded at runtime. With a HiGHS older than 1.14, `compute_iis()` throws `solver_error`, whose message names the release it found, the library's path and the 1.14 floor. To load another HiGHS, set `MIPPP_HIGHS_LIBRARY` to the full path of its library file, see [How solver libraries are found](../solvers/index.md#how-solver-libraries-are-found). The routines of Gurobi, CPLEX, Xpress and COPT exist throughout the releases MIP++ validates, and their api objects bind them like any other function, so a library that loads has them.

The deletion filter is an algorithm of the library, independent of the solver. It relaxes each finite variable bound and constraint side in turn, re-solves, and keeps a side only when the rest becomes feasible without it. It runs in place, on your model, which must let it enumerate its variables and constraints and read and change every bound and side; `iis_by_deletion_model` lists the [requirements](../reference/concepts.md#infeasibility-analysis). [The deletion filter](../algorithms/deletion-filter.md) describes the algorithm, what a run changes and costs, and how to run it on constraints of your own.

The two paths may return different types, so code that chooses between them hands the answer to code written for any IIS, as `print_conflict` is. `diagnose` chooses at compile time, and at run time too when the native call throws:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:diagnose"
```

`diagnose(model, print_conflict)` prints the same five lines on `highs_lp`, whatever the HiGHS release, and on `clp_lp`, which has only the filter. Every model class runs the filter, so `diagnose` asks for `iis_by_deletion_model` alone.

## Reading the answer

`get_status(v)` and `get_status(c)` return a `std::variant` over the tag types of namespace `iis_status`, a hierarchy like that of the [solve status](status-and-limits.md#the-solve-status):

```text
absent
member
├── member_lower    the lower bound of a variable, or the lower side of a row
├── member_upper    the upper bound, or the upper side
└── member_both     both of them
```

Test membership with `is_a<iis_status::member>(s)`, which accepts every refinement. `is<iis_status::member>(s)` tests for the plain tag alone, and does not compile on a variant that does not list it, nor does `is_a<iis_status::member_both>(s)` on the row status of the Gurobi and CPLEX routines, which does not list `member_both`. `iis_status::sides_of(s)` compiles on any answer, and reads each tag as an `iis_sides`:

| Tag | `lower` | `upper` | `whole` |
| :--- | :--- | :--- | :--- |
| `absent` | `false` | `false` | `false` |
| `member_lower` | `true` | `false` | `false` |
| `member_upper` | `false` | `true` | `false` |
| `member_both` | `true` | `true` | `false` |
| `member` | `true` | `true` | `true` |

The side matters on anything with two: a variable's bounds, a ranged row, an `==` row. The workshop's orders conflict through their lower sides. `member_both` means that each side is needed on its own, and relaxing either one alone removes the conflict: the sides cross, a lower bound above the upper bound, or integrality [needs both sides of a row](#lp-or-milp). Plain `member` holds the entity's finite sides as one unit: relaxing all of them removes the conflict, relaxing one alone may not. A routine whose unit is the row answers so, as Gurobi and CPLEX do for an `==` row, and so does MIP++ where a routine named one side that integrality may need with the other, as on a ranged row of an `xpress_milp` with integer columns. Code meant for any IIS tests `whole` before the sides, as `print_member` does.

What each routine names:

| Routine | Sides named |
| :--- | :--- |
| Deletion filter | every side |
| HiGHS | every side |
| Xpress | every side. An `==` row or a column that integrality needs whole is named through both sides. On an `xpress_milp` with integer columns, a ranged row the routine flags on one side is a plain `member`, see [Native IIS on Xpress](../solvers/index.md#limitation-xpress-iis) |
| Gurobi, CPLEX | a bound by its side, a row by its membership only: an inequality row takes the side of its sense, and an `==` row is a plain `member`; so is a ranged row on CPLEX, and Gurobi's models have no ranged row, see [Support by model](#support-by-model) |
| COPT | every side on `copt_lp` and on a `copt_milp` without integer columns; on a `copt_milp` with integer columns, a two-sided row or a column with two finite bounds is a plain `member`, see [Native IIS on COPT](../solvers/index.md#limitation-copt-iis) |

Querying your own handles, as `print_conflict` does, gives each member your name for it. Code that did not build the model lists the members through `model.variables()` and `model.constraints()`, which every model class provides:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:member-rows"
```

On the workshop, `rows` holds the labour row and the three order rows. To print such handles, name the variables and constraints as you add them: `get_variable_name(v)` and `get_constraint_name(c)` read the names back on models with `has_named_variables` and `has_named_constraints`, see [Names](../modeling/variables.md#names) for variables and [Constraint families](../modeling/expressions.md#constraint-families) for constraints. `num_variable_members()` and `num_constraint_members()` count the members without a loop: one variable and four rows there. A query is a lookup in the snapshot and never calls the solver.

## Repairing the model

One repair always removes the conflict: relax every side the answer names, a lower side to `-infinity()` and an upper side to `infinity()`, which takes both sides of a `member_both` and of a plain `member`:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:relax-members"
```

It is one repair among others, and seldom the one you want. On the workshop it frees the labour row, the overtime and the lower sides of the three orders, and the model then solves with no overtime at all, since no order binds any more. Dropping any single member already removes a conflict, and which one to change, and by how much, is a decision on the data that the IIS leaves to you: the remedy above moved one member by one hour.

`relax_members` writes a row through `set_constraint_lower_bound` and `set_constraint_upper_bound`, which `gurobi_lp` and `gurobi_milp` do not have, see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints). There, read the row's sense with `get_constraint_sense`: a `<=` or `>=` row is freed through an infinite rhs, written with `set_constraint_rhs`, an `==` row that loses one side keeps the other through its sense alone, written with `set_constraint_sense`, and one that loses both becomes a `<=` row with an infinite rhs. The deletion filter writes the rows of these models the same way, see [Candidates and trials](../algorithms/deletion-filter.md#candidates-and-trials).

A model can hold several conflicts, and an IIS explains one of them. Suppose that the workshop's labour comes from one team per product, whose hours cannot be shared:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:teams-model"
```

The chairs need 10 hours of a team that has 8, and the desks 4 of a team that has 3: two conflicts, with no member in common. A loop that relaxes the members of each answer and runs the filter again explains one conflict per round, until an answer has no member left:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:teams-repair"
```

```text
--8<-- "test/doc_snippets/infeasibility_repair.txt"
```

The last answer has no member, and its outcome says why: `feasible` here, since no conflict remains. An answer can also have no member because the run proved nothing, a tag of the `incomplete` branch without a conflict, or because the background conflicts on its own, `irreducible`, which no relaxed side repairs, see [How a run ends](#how-a-run-ends): read `iis.get_outcome()` before relying on the solve that follows. The loop changes the model for good, so save the sides you want back before running it.

## How a run ends

`get_outcome()` says how the run ended, as `get_status()` says how a solve ended: it returns a `std::variant` over the tag types of namespace `iis_outcome`, a hierarchy like that of the [solve status](status-and-limits.md#the-solve-status):

```text
any                        carries conflict_available
├── completed
│   ├── irreducible        an IIS
│   └── feasible           the model is feasible as it is
└── incomplete             no decision, for a cause the routine does not name
    ├── inconclusive_trial a trial of the filter proved neither way
    └── stopped            cut short from outside the routine
        ├── interrupted
        └── limit_reached
            ├── time_limit
            ├── solve_limit
            ├── iteration_limit
            ├── node_limit
            └── memory_limit
```

| Tag | Meaning |
| :--- | :--- |
| `irreducible` | The members conflict, and without any one of them the others have a solution. |
| `feasible` | The model is feasible as it is. There is no member. |
| `incomplete` | The run decided nothing, for a cause the routine does not name: a gap in its proof, numerical trouble, or a stop it cannot tell from those; or, on a run that narrows an answer, the named sides have a solution together, see [Narrowing an answer](#narrowing-an-answer). It does not imply that a limit stopped the run: raising one may or may not help, see [Limits](#limits). |
| `inconclusive_trial` | A trial of the filter ended without proving either infeasibility or a feasible point, as a numerical failure does. |
| `stopped` | Something outside the routine cut the run short, and the routine does not say what. |
| `interrupted` | A stop was requested: through the filter's `stop_token`, or through the solver during a native call. |
| `limit_reached` | A limit without a tag of its own stopped the run, see [Limits](#limits). |
| `time_limit` | The filter's deadline passed, or the model's time limit stopped a native call. |
| `solve_limit` | The filter made `max_solves` solves. |
| `iteration_limit` | An iteration limit stopped a native call. |
| `node_limit` | A node limit stopped a native call, or the solve that precedes COPT's search. |
| `memory_limit` | A memory limit stopped a native call. |

Every tag carries `conflict_available`, which `iis_outcome::conflict_available(o)` reads, as `status::solution_available` reads a status. It says whether the members form a subsystem proven infeasible, minimal or not. It holds on `irreducible` and never on `feasible`. On the `incomplete` branch it says whether the run kept a conflict: the deletion filter's members are then the last subset it proved infeasible, untested candidates included, and a native routine's are those it could not rule out. An answer without it has no member. The snippet of [Limits](#limits) prints every answer for which it holds.

`irreducible` with no member means that the background alone is infeasible, without any bound or side. The background is what neither path counts as candidates and both keep in every check: integrality, and the indicator constraints of `gurobi_milp` and `cplex_milp`. Neither is ever a member, and the rows and bounds an answer names conflict against them. On the filter, a registered candidate-solution callback is part of the background as well, while the native routines run without it. A filter answer without a member can thus also mean that the callback rejects every point, as on a feasible `xpress_milp` in [Callbacks](../algorithms/deletion-filter.md#callbacks). The filter's proof also needs a callback whose decision to accept or reject a point depends on that point alone: one with memory can answer two trials of the same sides differently, and `irreducible` is then unproven. MIP++ adds no SOS constraint: one added through the native handles, like any native change, is outside the guarantee, see [Native changes](#native-changes). COPT's routine counts the SOS and indicator constraints added there among its candidates, so an answer that names rows or bounds and only some of those constraints may not be minimal against the background: it is reported on the `incomplete` branch rather than `irreducible`, see [Native IIS on COPT](../solvers/index.md#limitation-copt-iis).

Each path's variant lists only the tags its routine can tell apart, as a backend's solve status does, so test with `is_a`: `is_a<iis_outcome::stopped>(o)` holds for every stop the path can tell apart, whatever its cause, and `is_a<iis_outcome::incomplete>(o)` for every run that decided nothing. A stop the routine cannot tell from a gap in its proof is plain `incomplete`, see [Limits](#limits). `is<T>(o)` does not compile on a variant that does not list `T`, nor `is_a<T>(o)` on one that lists neither `T` nor a tag under it. Every path lists `incomplete`, `irreducible` and `feasible`, which the concept `lp_iis_outcome` requires, so generic code can rely on them and on `conflict_available`, and guards a test for a cause with `variant_containing_a`, as in `if constexpr(variant_containing_a<decltype(iis.get_outcome()), iis_outcome::node_limit>)`.

| Path | Tags its variant lists |
| :--- | :--- |
| Deletion filter | `incomplete`, `irreducible`, `feasible`, `inconclusive_trial`, `interrupted`, `time_limit`, `solve_limit`. A run returns plain `incomplete` only when it narrows an answer whose named sides have a solution together, see [Narrowing an answer](#narrowing-an-answer). |
| HiGHS | `incomplete`, `irreducible`, `feasible`, `time_limit` |
| Gurobi | `incomplete`, `irreducible`, `feasible`, `stopped`, `interrupted`, `limit_reached`, `time_limit`, `iteration_limit`, `memory_limit`. `interrupted`, `limit_reached`, `iteration_limit` and `memory_limit` are reachable on `gurobi_lp` only. |
| CPLEX | `incomplete`, `irreducible`, `feasible`, `interrupted`, `limit_reached`, `time_limit`, `iteration_limit`, `node_limit`, `memory_limit`. `iteration_limit` is reachable on `cplex_lp` only, and `node_limit` on `cplex_milp` only. |
| Xpress | `incomplete`, `irreducible`, `feasible`, `stopped`, `time_limit` |
| COPT | `incomplete`, `irreducible`, `feasible`, `interrupted`, `time_limit`, `node_limit`. `node_limit` is reachable on `copt_milp` only. |

The `*_lp` and `*_milp` classes of one solver share a list. A cause that the routine reports and its path has no tag for is reported as its nearest ancestor that the path lists, as `limit_reached` stands for CPLEX's objective and deterministic-time limits.

The filter checks its limits before each trial. When several are reached at once, a stop request is reported before the deadline, and the deadline before the solve count, see [Limits](../algorithms/deletion-filter.md#limits) on the algorithm's page.

## Limits

`iis_limits`, the filter's last argument, bounds the whole run:

- `max_solves`: the number of `solve()` calls;
- `time_limit`: one duration for the whole call, which becomes a single deadline when the call starts. A negative or NaN duration throws `std::invalid_argument`, and an infinite one, the default, means no deadline;
- `stop_token`: a `std::stop_token`, from a `std::stop_source` or the `std::jthread` running the analysis; a stop requested from another thread ends the run before its next trial.

Here `stop` is such a token:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:limits"
```

A run stopped by a limit keeps what it proved, which is why the snippet prints every answer that holds a conflict, not only an `irreducible` one. With a budget of three solves, the workshop's run ends `solve_limit` with a conflict, and its members include those of the IIS.

Such an answer of the filter is a start: run the filter again with a larger budget, or narrow the answer, see [Narrowing an answer](#narrowing-an-answer).

Limits are checked between trials, and a trial that has started runs to its end. Every model class has a time limit (`has_time_limit`), so under a finite `time_limit` each trial gets the time that remains as its own time limit. It never gets more than the limit you had set, which is restored afterwards. A trial can still run past the deadline on `glpk_milp` and `cbc_milp`, where the time limit does not bound every phase of a solve. On `clp_lp` and `soplex_lp`, whose time limit counts CPU time, a trial can stop before the deadline. See [Time limits](../algorithms/deletion-filter.md#time-limits). With the default limits, the filter never reads or writes the model's time limit.

`compute_iis()` takes no argument. The model's own time limit, set by `set_time_limit`, bounds each call as a fresh budget, so `solve()` followed by `compute_iis()` may take twice the limit. A stop leaves either no subsystem or a subsystem without the proof of minimality, which `conflict_available` tells apart. The tag names the cause where the routine reports it. Where it does not, the time measured around the call decides: `time_limit` once it reached the budget (on Xpress, once it came within 20 ms of it), otherwise `stopped` on Gurobi and Xpress, which tell that a stop occurred, and `incomplete` on HiGHS, on COPT and on a Gurobi answer that holds a subsystem, where a stop cannot be told from an answer the routine did not complete. A call may return late, since the routines have phases that a limit does not interrupt. What each routine does under the model's limits:

| Model | Stop by the time limit | Other limits that stop it, and their tag | Notes |
| :--- | :--- | :--- | :--- |
| `highs_lp`, `highs_qp` | `time_limit`, with or without a conflict; the limit bounds HiGHS's search only, not the checks it runs before and after it, so a call can return after the limit | none: the iteration limit of `set_iteration_limit` stops `solve()`, not a call | the model's limit is copied into HiGHS's own `iis_time_limit` for the call and restored afterwards |
| `gurobi_lp`, `gurobi_milp` | `time_limit`, with or without a conflict | on `gurobi_lp`, a stop that leaves no subsystem takes the cause Gurobi reports: `iteration_limit` for the iteration limit, which stops the routine with no answer, see [Native IIS on Gurobi](../solvers/index.md#limitation-gurobi-iis); by Gurobi's documentation, `memory_limit` for the memory limit of `set_memory_limit`, `interrupted` for an interrupt, and `limit_reached` for the objective limits set through the native handles; `limit_reached` for the work limit set there (measured). On `gurobi_milp`, where Gurobi reports no cause, such a stop is `stopped`. A stop after a subsystem was found is `incomplete` with a conflict | a small conflict, or a small model an earlier `solve()` proved infeasible, may still answer in full under a limit that stops other calls, a zero limit included, since Gurobi's cheap checks run before its clock |
| `cplex_lp`, `cplex_milp` | `time_limit`, without a conflict: CPLEX never returns a partial answer | `iteration_limit` for the iteration limit of `cplex_lp`; `node_limit` for the node limit of `cplex_milp`, whose stop can come seconds late on a hard model; `limit_reached` for the deterministic-time limit set through the [native handles](../solvers/index.md#limitation-native-handles) and, by CPLEX's documentation, the objective limit set there; by CPLEX's documentation, `memory_limit` for the memory limit of `set_memory_limit` on `cplex_milp` and `interrupted` for a user abort, as for a solve. None of them holds a conflict | after an iteration-limit stop, write a bound or a side, even to its current value, before calling again: until then the calls end `incomplete`, without a conflict, see [Native IIS on CPLEX](../solvers/index.md#limitation-cplex-iis); a time-limit stop is forgotten once the limit is raised |
| `xpress_lp`, `xpress_milp` | `time_limit`, with or without a conflict; a zero limit stops every call on a model with rows before any answer, feasible models included; a model without rows is answered from its columns' bounds, see [Native IIS on Xpress](../solvers/index.md#limitation-xpress-iis) | none of MIP++'s; an interrupt or an iteration limit set through the native handles is `stopped`, since Xpress does not report the cause | |
| `copt_lp`, `copt_milp` | `time_limit`, with or without a conflict; one budget covers the routine and the solve described below | COPT's node limit, set through the native handles, stops the solve that precedes the search on `copt_milp`, `node_limit`, or `feasible` when that solve found a point; it does not stop the search. An interrupt of the solve, through the native handles, is `interrupted` | the phases that a limit does not interrupt grow with the model, and a late return with them; see [Native IIS on COPT](../solvers/index.md#limitation-copt-iis) |

On COPT, `compute_iis()` solves the model as well as running the routine, both under the call's budget. On a `copt_milp` with integer columns, which COPT solves as a MIP, the solve comes first, since COPT answers a whole-model conflict on an unsolved feasible MIP. That solve stops at the first incumbent, which settles a feasible model; an infeasible one is solved to its proof. On `copt_lp`, and on a `copt_milp` whose columns are all continuous, the search comes first, and the solve follows when the search found nothing, since COPT returns the same code on a feasible model and on a failure. The solve gets what remains of the budget.

## Narrowing an answer

`compute_iis_by_deletion(model, within)`, or `(model, within, limits)`, runs the filter on the sides that an earlier answer `within` names, a plain `member` naming every finite side of its entity. Every other finite side stays relaxed in every trial, and is written back with the rest when the call returns. Its first trial checks the named sides together, so `within` may come from either path, or from before a change to the model: it is read by handle id on this model. The IIS that the run finds among the named sides is an IIS of the whole model: each trial reads only the sides it keeps and the background, so the sides kept have no solution together and have one without any single one of them. When the named sides have a solution together, the run answers `incomplete` without a conflict, never `feasible`: that solution says nothing of the sides it relaxed.

A run that a limit stopped is the usual start, since its members hold a conflict:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:narrowing"
```

On the workshop, the three-solve answer names 11 of the 13 finite sides. The second run makes 12 solves, one per named side plus one, and prints the five lines of the complete run. A run never makes more solves than that.

A native answer is another start: `compute_iis_by_deletion(model, model.compute_iis())` names the side of each member that the routine reports whole. On the workshop's `gurobi_lp` and `cplex_lp`, it answers the five sides of the complete run, the orders through their lower sides.

On a `*_milp` model, the bounds of integer columns that the answer leaves out are relaxed in every trial, and some solvers branch without end on rows over unbounded integer columns, see [Integrality proofs](../solvers/index.md#limitation-integrality-proofs). On `scip_milp`, a bound of a binary column that the answer leaves out makes the first trial throw, see [Deletion filter on SCIP binaries](../solvers/index.md#limitation-scip-binaries).

## LP or MILP

An IIS explains the problem that the model class solves. On a `*_milp` model it explains the MIP: integrality is part of the background, never a member, so rows and bounds can conflict where the LP relaxation has a solution. On a `*_lp` model it explains the LP. Ten tonnes cannot travel in full three-tonne trucks:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:trucks"
```

On `highs_milp` the answer is `irreducible`, with `load` a member through both sides, since `3 * trucks <= 10` and `3 * trucks >= 10` each have an integer solution, three trucks or four. The bounds of `trucks` are not members. The same row over a continuous variable is feasible, and the filter says so on `highs_lp`.

This is why `highs_milp` has no `compute_iis()`: HiGHS's routine explains the LP relaxation of a MIP, which answers another question. `highs_milp` runs the deletion filter, whose trials are MIP solves. The routines of Gurobi, CPLEX, Xpress and COPT explain the MIP, so their `*_milp` classes have `compute_iis()`. On the trucks each answers `irreducible`, with `load` a plain `member` on `gurobi_milp`, `cplex_milp` and `copt_milp` and a member through both sides on `xpress_milp`, and each of their `*_lp` classes answers `feasible`. On a `copt_milp` with integer columns a column with two finite bounds is a plain `member` in the same way, see [Reading the answer](#reading-the-answer). `xpress_milp` lists such a column once per bound and the answer names both sides.

Whether an IIS names the bounds of a binary variable depends on the solver. Some solvers hold its domain in the type rather than in the bounds: relaxing a bound then widens nothing, and the filter does not name it. With a binary `z` under the row `z >= 2`, measured:

| Model | Deletion filter | `compute_iis()` |
| :--- | :--- | :--- |
| `gurobi_milp`, `cplex_milp` | the row alone | the row alone |
| `copt_milp` | the row alone | the row, and `z` as a plain `member` |
| `xpress_milp` | the row and the upper bound of `z` | the row and the upper bound of `z` |
| `highs_milp`, `mosek_milp`, `cbc_milp`, `glpk_milp` | the row and the upper bound of `z` | no routine |
| `scip_milp` | throws, see [Deletion filter on SCIP binaries](../solvers/index.md#limitation-scip-binaries) | no routine |

An integer `z` in [0, 1] gives the row and the upper bound of `z` everywhere, except that the routine of `copt_milp` names `z` as a plain `member` and that `scip_milp`, which types such a column binary, throws.

## The model afterwards

Both paths leave the model's data as they found them. The filter changes bounds and sides during its trials and sets the objective to zero. It saves each item first and writes every one back on every exit, exceptions included. An item whose write-back fails stays as the trial left it, and the call throws the first such error once every item was attempted; when an exception from a trial is already propagating, the call throws that one instead and drops the failure. The items are bounds, sides, the objective from a copy, the Hessian on `highs_qp`, the time limit it forwarded, and the row senses and right-hand sides of `gurobi_lp` and `gurobi_milp`, see [What a run changes](../algorithms/deletion-filter.md#what-a-run-changes). Its trials use the solver's MIP starts, basis and incumbent as any `solve()` does, and can add a MIP start or replace yours. `compute_iis()` restores the solver options it sets for the call:

- HiGHS: its IIS options.
- Gurobi: the attributes that keep indicator constraints in the background, with the SOS and quadratic constraints that MIP++ does not add.
- Xpress: the option that keeps integrality in the background, with the general, piecewise-linear, SOS and indicator constraints that only the native handles can add on Xpress.
- COPT: the time limit written for the solve, see [Limits](#limits).
- CPLEX sets none: the candidates are arguments of the call.

The status is another matter. After any run of the filter that solved, the solver holds the solution of a trial rather than of your model, so `get_status()` reports `unknown`, after an exception too. A run answered without a solve leaves the status as it was: one stopped before its first trial, or one on a model without variables, which the filter decides from the constraint sides alone. `compute_iis()` makes the status `unknown` on every call, whether it returns or throws, and the solver's own state is no longer your solve's either:

- HiGHS solves an unsolved model.
- Gurobi and Xpress solve an unsolved model, or overwrite the solver's status.
- CPLEX replaces its status with the conflict's. It keeps a completed answer until the model's data change, so a second `compute_iis()` on an unchanged model returns at once. An iteration-limit stop is kept the same way, see [Limits](#limits).
- COPT resets the solver, dropping its held solution, then solves the model as [Limits](#limits) describes.
- CPLEX replaces the [MIP starts](updates.md#giving-the-solver-a-starting-point) of `cplex_milp`: measured with one start of yours, a call on a feasible model leaves a point of the routine as the only start, and a call on an infeasible model leaves none, as `solve()` does there. Gurobi's `Start` attribute survives the call, as it survives the filter.
- `cplex_milp`, `xpress_milp` and `copt_milp` detach a registered candidate-solution callback for the call and reattach it afterwards, see [Native IIS on CPLEX](../solvers/index.md#limitation-cplex-iis), [on Xpress](../solvers/index.md#limitation-xpress-iis) and [on COPT](../solvers/index.md#limitation-copt-iis). Gurobi's routine never runs it. No native routine honours the callback's lazy constraints, while the filter's trials run it, see [Callbacks](../algorithms/deletion-filter.md#callbacks).

Call `solve()` again before reading a solution, as the workshop's remedy does.

## A snapshot of the model as it was

A snapshot is computed once, when the call returns, and later changes to the model do not reach it. It is keyed by handle id: the handles of entities that remain keep their answers after a `remove_variable`, but a variable added later may reuse a removed variable's id, and would then read the removed variable's entry. Compute a new IIS after changing the model.

## Support by model

| Model | `has_iis` | `iis_by_deletion_model` | Notes |
| :--- | :--- | :--- | :--- |
| `highs_lp`, `highs_qp` | yes, with HiGHS 1.14 or later at runtime, see [Two paths](#two-paths) | yes | HiGHS repairs sides crossed by less than its tolerance: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) |
| `highs_milp` | no: the routine explains the relaxation, see [LP or MILP](#lp-or-milp) | yes | as above |
| `clp_lp`, `soplex_lp` | no | yes | the time limit a trial gets counts the CPU time of the whole process, so a trial can stop before the deadline, see [Time limits](../algorithms/deletion-filter.md#time-limits) |
| `glpk_lp` | no | yes | |
| `glpk_milp` | no | yes | a trial can run past the deadline, see [Time limits](../algorithms/deletion-filter.md#time-limits); on some integer rows it branches without end unless a time limit stops it, see [Integrality proofs](../solvers/index.md#limitation-integrality-proofs) |
| `cbc_milp` | no | yes | an [experimental](../solvers/index.md#the-backends) backend, with wrong answers of Cbc 2.10 on integer rows, and root LPs past the deadline (measured on a Cbc `devel` build): see [Integrality proofs](../solvers/index.md#limitation-integrality-proofs) and [Time limits](../algorithms/deletion-filter.md#time-limits) |
| `scip_milp` | no | yes | can throw on an infeasible model with binary columns: see [Deletion filter on SCIP binaries](../solvers/index.md#limitation-scip-binaries) |
| `mosek_lp`, `mosek_milp` | no | yes | |
| `gurobi_lp`, `gurobi_milp` | yes | yes, writing each row through its sense and rhs, see [Candidates and trials](../algorithms/deletion-filter.md#candidates-and-trials) | native: `==` rows are plain members, and an inequality row takes the side of its sense, see [Reading the answer](#reading-the-answer). The iteration limit of `gurobi_lp` stops the routine with no answer, see [Native IIS on Gurobi](../solvers/index.md#limitation-gurobi-iis) |
| `cplex_lp`, `cplex_milp` | yes | yes | native: `==` and ranged rows are plain members, and an inequality row takes the side of its sense, see [Reading the answer](#reading-the-answer). A stopped call has no answer, see [Limits](#limits) and [Native IIS on CPLEX](../solvers/index.md#limitation-cplex-iis). On either path, crossed sides cannot be built, see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints) |
| `xpress_lp`, `xpress_milp` | yes | yes | native: a column whose bounds admit no value is answered from its bounds, since the routine refuses the search there. On an `xpress_milp` with integer columns, a ranged row the routine flags on one side is a plain member, see [Native IIS on Xpress](../solvers/index.md#limitation-xpress-iis). On either path, crossed sides cannot be built, see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints) |
| `copt_lp`, `copt_milp` | yes | yes | `compute_iis()` also solves the model, see [Limits](#limits); on a `copt_milp` with integer columns, two-sided rows and two-bounded columns are plain members, see [Reading the answer](#reading-the-answer); the answer may leave out a bound the conflict needs, see [Native IIS on COPT](../solvers/index.md#limitation-copt-iis) |

Which paths name the bounds of a binary variable depends on the solver, see [LP or MILP](#lp-or-milp). A registered candidate-solution callback runs in the filter's trials and in no native routine, see [The model afterwards](#the-model-afterwards). Every model class has the deletion filter, and those of Gurobi, CPLEX, Xpress and COPT, with `highs_lp` and `highs_qp`, have a native routine too. What a routine offers beyond an IIS, such as forcing a candidate in or out of the conflict or enumerating several IISs, is reached through the solver's C API, outside MIP++: `native_model()` returns the solver's objects, and `native_id(v)` and `native_id(c)` give the index each of your handles has there, see [Native changes](#native-changes). When Xpress's routine found an IIS on `xpress_lp`, the problem keeps its data after `compute_iis()` returns: `native_api().getiisdata` reads it as IIS 1, and an `XPRSiisnext`, which the api object does not bind, searches under your own `IISOPS`, which the call restores, see [Native IIS on Xpress](../solvers/index.md#limitation-xpress-iis). Gurobi discards its IIS attributes on a model with SOS, quadratic or general constraints, indicators included, since the call writes back their forcing attributes: there, call `GRBcomputeIIS` yourself to read them. The engine of [The deletion filter](../algorithms/deletion-filter.md#the-engine) runs over an oracle of your own on any backend.

## Native changes

!!! warning "Changes made through the native handles are outside the guarantee"
    The [warning on the native handles](../solvers/index.md#limitation-native-handles) holds for both paths. Bounds, rows or special constraints added through `native_model()` and `native_api()`, SOS constraints included, since MIP++ adds none, are background that MIP++ does not know, and an IIS computed over them loses its guarantee. The filter's save and restore and the enumeration of `variables()` and `constraints()` do not see native changes either.

## Next

[Solutions, duals and reduced costs](solutions.md) — getting the numbers back out.
