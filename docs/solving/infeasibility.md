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

`compute_iis_by_deletion`, from `mippp/utility/iis_by_deletion.hpp`, which the solver headers do not include, is the deletion filter. A program usually gets there after `is_a<status::infeasible>(model.get_status())`, but the filter does not need that solve: it analyzes the model as it is. It returns a snapshot, whose `get_outcome()` says how far the answer goes: `irreducible` is an IIS, `not_proven_minimal` a conflict the run could not reduce, whose members are still worth printing, and a feasible model is an outcome, not an error, see [Outcomes and reasons](#outcomes-and-reasons).

`get_status(v)` and `get_status(c)` say whether a variable or a constraint takes part, and through which side, as a `std::variant` over the tags of namespace `iis_status`. `is_a<iis_status::member>(s)` holds for every member, whichever side, and `is_a<iis_status::member_lower>(s)` or `is_a<iis_status::member_upper>(s)` for a member through that one side. A member through both sides, or one whose side a solver's routine did not name, prints `member` here, see [Reading the answer](#reading-the-answer). `print_conflict` queries the snapshot with the handles of the model, and so gives each member your name for it.

An IIS says where the data disagree, not how to settle it. Relaxing any single member removes this conflict, and as the workshop has no other, repairs the model: here, two hours of overtime, which the last line reports. The model is the one you built: the filter changes bounds and sides while it works and writes every one of them back before it returns, see [The model afterwards](#the-model-afterwards), and leaves the status `unknown`, so the program solves again before reading the solution.

The filter runs on every model class, so `clp_lp` with Clp's header runs the same program. Several solvers also have a routine of their own, `model.compute_iis()`, see [Two paths](#two-paths). Each path returns a snapshot of a type of its own, which is why `print_member` is a template and `print_conflict` a generic lambda: the later sections hand `print_conflict` the answers of other calls.

[`examples/infeasible_transportation/`](https://github.com/fhamonic/mippp/blob/main/examples/infeasible_transportation/main.cpp) is a complete, runnable program along these lines: a transportation plan whose conflict the deletion filter finds on any of its backends, and the native routine too where the model class has one.

## Two paths

| Path | Call | Where | Returns |
| :--- | :--- | :--- | :--- |
| Native routine | `model.compute_iis()` | `has_iis<M>`: `gurobi_lp`, `gurobi_milp`, `cplex_lp`, `cplex_milp`, `xpress_lp`, `xpress_milp`, `copt_lp` and `copt_milp`, and `highs_lp` and `highs_qp` with HiGHS 1.14 or later at runtime | `model_iis_t<M>` |
| Deletion filter | `compute_iis_by_deletion(model)`, or `(model, limits)` with an `iis_limits`, see [Limits](#limits), from `mippp/utility/iis_by_deletion.hpp` | `iis_by_deletion_model<M>`: every model class | a snapshot type of its own, taken through `auto` |

Both analyze the model as it currently is and never rely on an earlier solve, whose status is stale as soon as the model changes: the workshop's `solve()` only showed that there was something to explain. A feasible model is an outcome, not an error. Neither runs behind your back: each is an explicit call, and can cost many solves.

The native routine is the solver's own: HiGHS's `Highs_getIis`, Gurobi's `GRBcomputeIIS`, CPLEX's `CPXrefineconflictext`, Xpress's `XPRSiisfirst` or COPT's `COPT_ComputeIIS`. On the workshop each finds the same conflict. `highs_lp`, `xpress_lp` and `copt_lp` print the same five lines, while on `gurobi_lp` and `cplex_lp` the three order rows print `member`, see [Reading the answer](#reading-the-answer):

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

Test membership with `is_a<iis_status::member>(s)`, which accepts every refinement. `is<iis_status::member>(s)` tests for the plain tag alone, and does not compile on a variant that does not list it. `is_a<iis_status::member_both>(s)` likewise needs a variant that lists `member_both`, which the row status of the Gurobi and CPLEX routines does not: `print_member` above tests `member_lower` and `member_upper`, which every answer lists, and prints `member` for the two other cases. Code that must tell them apart reads the side through a visitor over the tags, which compiles on any answer, since `std::visit` instantiates only the tags a variant lists:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:side-name"
```

The side matters on anything with two: a variable's bounds, a ranged row, an `==` row. The workshop's orders conflict through their lower sides. `member_both` means that each side is needed on its own, and relaxing either one alone removes the conflict: the sides cross, a lower bound above the upper bound, or integrality [needs both sides of a row](#lp-or-milp). Plain `member` is a member whose side the answer does not name: the routine did not name it, or, on a ranged row of an `xpress_milp` with integer columns, named one side where integrality may need both. Code meant for any IIS handles the plain tag, as `side_of` does.

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

On the workshop it returns the labour row and the three order rows. To print such handles, name the variables and constraints as you add them: `get_variable_name(v)` and `get_constraint_name(c)` read the names back on models with `has_named_variables` and `has_named_constraints`, see [Names](../modeling/variables.md#names) for variables and [Constraint families](../modeling/expressions.md#constraint-families) for constraints. `num_variable_members()` and `num_constraint_members()` count the members without a loop: one variable and four rows there. A query is a lookup in the snapshot and never calls the solver.

## Repairing the model

One repair always removes the conflict: relax every side the answer names, the lower side of a `member_lower` row or bound to `-infinity()`, the upper side of a `member_upper` one to `infinity()`, and both sides of a `member_both` or a plain `member`:

```cpp
--8<-- "test/doc_snippets/infeasibility.cpp:relax-members"
```

It is one repair among others, and seldom the one you want. On the workshop it frees the labour row, the overtime and the lower sides of the three orders, and the model then solves with no overtime at all, since no order binds any more. Dropping any single member already removes a conflict, and which one to change, and by how much, is a decision on the data that the IIS leaves to you: the remedy above moved one member by one hour.

On a model with row-bound setters, `relax_row` relaxes each named side through `set_constraint_lower_bound` or `set_constraint_upper_bound`. `gurobi_lp` and `gurobi_milp` have no such setters, see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints), so there it reads the row's sense instead:

- an `==` row that loses one side keeps the other through its sense alone, written with `set_constraint_sense`, and its rhs is not written;
- an `==` row that loses both sides becomes a `<=` row with an infinite rhs;
- a `<=` or `>=` row whose side is named keeps its sense and is freed through an infinite rhs.

The deletion filter writes the rows of these models through their sense and rhs too, by its own rule, see [Candidates and trials](../algorithms/deletion-filter.md#candidates-and-trials). `rerun_on_members` below saves and restores their rows as a sense and an rhs.

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

The loop returns `feasible` once no conflict remains. An answer can also have no member because the run proved nothing, `undetermined`, or because the background conflicts on its own, `irreducible`, which no relaxed side repairs, see [Outcomes and reasons](#outcomes-and-reasons): the loop then returns that outcome. It changes the model for good, so save the sides you want back before calling it, as `rerun_on_members` does under [Limits](#limits).

## Outcomes and reasons

`get_outcome()` says how far the answer goes, and `get_reason()`, a `std::optional<iis_reason>`, why it stopped short:

| `iis_outcome` | Members | Meaning |
| :--- | :--- | :--- |
| `irreducible` | an IIS | The members conflict, and without any one of them the others have a solution. |
| `not_proven_minimal` | a conflict, perhaps larger than needed | Not every member could be proven necessary: a limit stopped the run, a trial was inconclusive, or the native routine could not tell. The deletion filter's members are the last subset it proved infeasible; a native routine's are those it could not rule out. |
| `feasible` | none | The model is feasible as it is. |
| `undetermined` | none | Nothing was proven: the run stopped before its first proof, its first solve was inconclusive, or the native routine returned no answer. |

`irreducible` with no member means that the background alone is infeasible, without any bound or side. The background is what neither path counts as candidates and both keep in every check: integrality, and the indicator constraints of `gurobi_milp` and `cplex_milp`. Neither is ever a member, and the rows and bounds an answer names conflict against them. On the filter, a registered candidate-solution callback is part of the background as well, while the native routines run without it. A filter answer without a member can thus also mean that the callback rejects every point, as on a feasible `xpress_milp` in [Callbacks](../algorithms/deletion-filter.md#callbacks). MIP++ adds no SOS constraint: one added through the native handles, like any native change, is outside the guarantee, see [Native changes](#native-changes).

| `iis_reason` | Set when |
| :--- | :--- |
| `solve_limit` | the filter made `max_solves` solves |
| `time_limit` | the filter's deadline passed, or the model's time limit stopped a native call |
| `cancelled` | a stop was requested through the filter's `stop_token` |
| `inconclusive_trial` | a trial of the filter ended without proving either infeasibility or a feasible point, as a numerical failure does |

When several limits are reached at once, a stop request is reported before the deadline, and the deadline before the solve count. An empty reason means that no limit of the filter stopped the run, and that no time limit stopped a native call. It comes with every `irreducible` and `feasible` answer, with a native answer that the routine itself could not complete, and with a native stop by another limit or an interrupt, which [Limits](#limits) lists per model.

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

Limits are checked between trials, and a trial that has started runs to its end. Every model class has a time limit (`has_time_limit`), so under a finite `time_limit` each trial gets the time that remains as its own time limit. It never gets more than the limit you had set, which is restored afterwards. A trial can still run past the deadline on `glpk_milp` and `cbc_milp`, where the time limit does not bound every phase of a solve. On `clp_lp` and `soplex_lp`, whose time limit counts CPU time, a trial can stop before the deadline. See [Time limits](../algorithms/deletion-filter.md#time-limits). With the default limits, the filter never reads or writes the model's time limit.

`compute_iis()` takes no argument. The model's own time limit, set by `set_time_limit`, bounds each call as a fresh budget, so `solve()` followed by `compute_iis()` may take twice the limit. A stop leaves either no subsystem, `undetermined`, or a subsystem without the proof of minimality, `not_proven_minimal`. The reason is `time_limit` when the time limit stopped the call, and empty when another limit or an interrupt did. A call may return late, since the routines have phases that a limit does not interrupt. What each routine does under the model's limits:

| Model | Stop by the time limit | Other limits that stop it | Notes |
| :--- | :--- | :--- | :--- |
| `highs_lp`, `highs_qp` | `undetermined` or `not_proven_minimal`; the limit bounds HiGHS's search only, not the checks it runs before and after it, so a call can return after the limit | none: the iteration limit of `set_iteration_limit` stops `solve()`, not a call | the model's limit is copied into HiGHS's own `iis_time_limit` for the call and restored afterwards |
| `gurobi_lp`, `gurobi_milp` | `undetermined` or `not_proven_minimal` | the iteration limit of `gurobi_lp`, with no answer, see [Native IIS on Gurobi](../solvers/index.md#limitation-gurobi-iis); the memory limit of `set_memory_limit`, by Gurobi's documentation | a small conflict, or a small model an earlier `solve()` proved infeasible, may still answer in full under a limit that stops other calls, a zero limit included, since Gurobi's cheap checks run before its clock |
| `cplex_lp`, `cplex_milp` | `undetermined`: CPLEX never returns a partial answer | the iteration limit of `cplex_lp`, the node limit of `cplex_milp`, whose stop can come seconds late on a hard model, and the deterministic-time limit set through the [native handles](../solvers/index.md#limitation-native-handles); by CPLEX's documentation, the memory limit of `set_memory_limit` on `cplex_milp` and the objective limit set through the native handles | after an iteration-limit stop, write a bound or a side, even to its current value, before calling again: until then the calls stay `undetermined`, see [Native IIS on CPLEX](../solvers/index.md#limitation-cplex-iis); a time-limit stop is forgotten once the limit is raised |
| `xpress_lp`, `xpress_milp` | `undetermined` or `not_proven_minimal`; a zero limit stops every call before any answer, feasible models included | none of MIP++'s; an interrupt or an iteration limit set through the native handles stops it with no reason | |
| `copt_lp`, `copt_milp` | `undetermined` or `not_proven_minimal`; one budget covers the routine and the solve described below | COPT's node limit, set through the native handles, stops the solve that precedes the search on `copt_milp`, `undetermined` with no reason, or `feasible` when that solve found a point; it does not stop the search | the phases that a limit does not interrupt grow with the model, and a late return with them; see [Native IIS on COPT](../solvers/index.md#limitation-copt-iis) |

On COPT, `compute_iis()` solves the model as well as running the routine, both under the call's budget. On a `copt_milp` with integer columns, which COPT solves as a MIP, the solve comes first, since COPT answers a whole-model conflict on an unsolved feasible MIP. That solve stops at the first incumbent, which settles a feasible model; an infeasible one is solved to its proof. On `copt_lp`, and on a `copt_milp` whose columns are all continuous, the search comes first, and the solve follows when the search found nothing, since COPT returns the same code on a feasible model and on a failure. The solve gets what remains of the budget.

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

Both paths leave the model's data as they found them. The filter changes bounds and sides during its trials and sets the objective to zero. It saves each item first and writes it back on every exit, exceptions included. The items are bounds, sides, the objective from a copy, the Hessian on `highs_qp`, the time limit it forwarded, and the row senses and right-hand sides of `gurobi_lp` and `gurobi_milp`, see [What a run changes](../algorithms/deletion-filter.md#what-a-run-changes). Its trials use the solver's MIP starts, basis and incumbent as any `solve()` does, and can add a MIP start or replace yours. `compute_iis()` restores the solver options it sets for the call:

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

Which paths name the bounds of a binary variable depends on the solver, see [LP or MILP](#lp-or-milp). A registered candidate-solution callback runs in the filter's trials and in no native routine, see [The model afterwards](#the-model-afterwards). Every model class has the deletion filter, and those of Gurobi, CPLEX, Xpress and COPT, with `highs_lp` and `highs_qp`, have a native routine too. What a routine offers beyond an IIS, such as forcing a candidate in or out of the conflict or enumerating several IISs, stays reachable through the solver's C API, outside MIP++. `native_model()` returns the solver's objects, and `native_id(v)` and `native_id(c)` give the index each of your handles has there, see [Native changes](#native-changes). The engine of [The deletion filter](../algorithms/deletion-filter.md#the-engine) runs over an oracle of your own on any backend.

## Native changes

!!! warning "Changes made through the native handles are outside the guarantee"
    The [warning on the native handles](../solvers/index.md#limitation-native-handles) holds for both paths. Bounds, rows or special constraints added through `native_model()` and `native_api()`, SOS constraints included, since MIP++ adds none, are background that MIP++ does not know, and an IIS computed over them loses its guarantee. The filter's save and restore and the enumeration of `variables()` and `constraints()` do not see native changes either.

## Next

[Solutions, duals and reduced costs](solutions.md) — getting the numbers back out.
