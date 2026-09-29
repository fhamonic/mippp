# The deletion filter

An infeasible set of constraints usually owes its infeasibility to a few of them. An irreducible infeasible subsystem (IIS) is a subset of them that is infeasible on its own and becomes feasible as soon as any one of its members is dropped. The deletion filter computes one from nothing more than a feasibility test on subsets, so it works where no dedicated routine exists: on a solver without one, or on constraints that no solver sees.

MIP++ provides it in two layers:

- `deletion_filter`, in `mippp/utility/deletion_filter.hpp`, is the algorithm alone. It works on candidate indices and asks an oracle of yours about subsets of them. It knows nothing of models or solvers.
- `compute_iis_by_deletion`, in `mippp/utility/iis_by_deletion.hpp`, runs it on a model: the candidates are the model's variable bounds and constraint sides, and the oracle re-solves the model. [Diagnosing infeasibility](../solving/infeasibility.md) shows how to call it and read its answer; this page says what it does.

## The algorithm

The filter first proves the whole set infeasible. It then takes each candidate once: it drops it and asks whether the others are still infeasible. If they are, the candidate stays out. If they are feasible, the candidate is necessary and goes back in. What remains after this single pass is an IIS.

One pass suffices because the oracle is monotone: dropping candidates never makes a feasible set infeasible. A candidate goes back in because the set without it was feasible. The final set without it is a subset of that set, hence feasible too, so the candidate is still necessary at the end. A complete run on an infeasible set thus makes one oracle call per candidate, plus the first one, and a run never makes more.

A set may contain several IISs. The filter returns one of them, and which one depends on the order in which it tries the candidates. The method is the deletion filter of J. W. Chinneck and E. W. Dravnieks, *Locating minimal infeasible constraint sets in linear programs*, ORSA Journal on Computing 3(2), 1991.

## The engine

`deletion_filter(candidate_count, oracle, limits)` runs the filter over the candidates `0` to `candidate_count - 1`, which stand for whatever you choose. `limits` is optional, see [Limits](#limits). The oracle is any callable that satisfies the concept `deletion_oracle`:

- It receives the active candidates as a `std::span<const std::size_t>`, in no particular order. The span is only valid during the call: copy it to keep it.
- It returns a `deletion_verdict`: `feasible`, `infeasible` or `inconclusive`. A callable returning `bool` does not satisfy the concept.
- The engine calls it as an lvalue and never copies it, so what it keeps from call to call, a count or a warm start, is still in your object after the run. A move-only oracle works; one whose call operator needs an rvalue does not satisfy the concept.
- `inconclusive` is always allowed, for a check that failed or gave up. It never drops a candidate: the candidate under test stays in, and the run can then no longer end `irreducible`.
- An exception thrown by the oracle propagates out of `deletion_filter` unchanged.

### Monotonicity

The filter's one precondition is that the oracle is monotone: every subset of a feasible set of candidates is feasible. It holds whenever `feasible` means that one point satisfies every active candidate, whatever the candidates are: rows and bounds, the timing rules of the example below, the clauses of a SAT instance. It fails when a candidate can help rather than restrict, as one that adds a resource or offers an alternative: dropping it can make the others infeasible.

The engine cannot check it. Without it, the members still form a set that the oracle found infeasible, but the outcome can say `irreducible` of a set that is not: a candidate that went back in because the others were feasible when it was tested may no longer be needed once later candidates are dropped.

### The answer

`deletion_filter` returns a `deletion_filter_result`:

| Field | Holds |
| :--- | :--- |
| `members` | The members' indices, in ascending order. Empty unless a call proved infeasibility. |
| `outcome` | An `iis_outcome`: `irreducible`, `not_proven_minimal`, `feasible` or `undetermined`, as in [Outcomes and reasons](../solving/infeasibility.md#outcomes-and-reasons). |
| `reason` | A `std::optional<iis_reason>`, empty on an `irreducible` or `feasible` answer, otherwise why the run fell short. |

`irreducible` with no member means that the oracle answered `infeasible` for the empty set: what no candidate covers, the background, is infeasible on its own. A `feasible` answer comes from a single call, on the whole set.

## Example: a morning that does not fit

A morning holds a talk, then lunch, and a few rules on their timing. Times count minutes after 9:00, and each rule bounds the delay from one event to another from below, from above, or both:

```cpp
--8<-- "test/doc_snippets/deletion_filter.cpp:timing-rules"
```

```cpp
--8<-- "test/doc_snippets/deletion_filter.cpp:timing-data"
```

These are difference constraints. They have a schedule exactly when a graph with one arc per bound, from `from` to `to` of length `at_most` and from `to` to `from` of length `-at_least`, has no cycle of negative length. Bellman-Ford decides it: start every event at 0 and lower times until every active rule holds. If the times settle, they are a schedule. If they still move after one round per event, a negative cycle keeps lowering them, and no schedule exists:

```cpp
--8<-- "test/doc_snippets/deletion_filter.cpp:timing-oracle"
```

A schedule of some rules is a schedule of any subset of them, so the oracle is monotone. The filter needs nothing else, and the loop prints the rules of the answer to `out`, any `std::ostream`:

```cpp
--8<-- "test/doc_snippets/deletion_filter.cpp:timing-filter"
```

```text
--8<-- "test/doc_snippets/deletion_filter_morning.txt"
```

The talk starts at 9:00 at the earliest and lasts at least 50 minutes, and lunch comes at least 20 minutes after it, so lunch cannot start before 10:10, while the last rule wants it by 10:00. The outcome is `irreducible`: without any one of the four rules, a schedule exists. The talk's latest start and lunch's earliest start take no part. The run makes seven calls, one on the whole set and one per rule.

The talk's duration is one rule with two sides, hence one candidate, and the answer names the rule rather than the side in conflict. Making each side a candidate of its own would name the side, as the model version does for bounds and rows.

## Limits

The third argument is the `iis_limits` aggregate of [Diagnosing infeasibility](../solving/infeasibility.md#limits). For the engine:

- `max_solves` counts oracle calls;
- `time_limit` becomes one deadline when `deletion_filter` starts. A negative or NaN duration throws `std::invalid_argument` before any call, and an infinite one, the default, means no deadline;
- `stop_token` ends the run once a stop is requested.

The engine checks them before each call, never during one: an oracle call that has started runs to its end. When several are reached at once, a stop request is reported before the deadline, and the deadline before the call count. A stopped run keeps what it proved: the last set the oracle found infeasible, untested candidates included, as `not_proven_minimal`, or nothing, as `undetermined`, if it stopped before its first proof. A limit reached after the last call is not reported: a run whose final call completes the proof is `irreducible`, without a reason.

An `inconclusive` answer sets the reason `inconclusive_trial`, or `time_limit` when the deadline has passed by the time the call returns. A later stop replaces it with its own reason.

Since the engine never interrupts a call, an oracle that can run long must bound itself. Here `search` stands for your own check, told how much time it has left and answering `inconclusive` when that runs out:

```cpp
--8<-- "test/doc_snippets/deletion_filter.cpp:own-deadline"
```

A call that gives up drops nothing, so the answer is then `not_proven_minimal` or `undetermined`, never `irreducible`.

## On a model

`compute_iis_by_deletion(model, limits)` runs the engine on any model of the concept `iis_by_deletion_model`, which every model class of Cbc, Clp, GLPK, HiGHS, MOSEK, SCIP and SoPlex satisfies. It works in place, on your model, without copying it; the concept lists what it reads and writes, see [Infeasibility analysis](../reference/concepts.md#infeasibility-analysis). Its answer is a snapshot keyed by your handles, whose tags name the side of each member: `member_lower`, `member_upper`, or `member_both` when both sides of one variable or row are members.

### Candidates and trials

Every finite variable bound and every finite row side is a candidate, each side on its own: a variable in [0, 10] gives two candidates, an `==` row or a ranged row two, a `<=` row one. A lower side is finite when it is above `-model.infinity()`, an upper side when it is below `model.infinity()`, so a side at the opposite infinity, as in `x >= infinity()`, is a real bound and a candidate. Integrality, special constraints and anything added through the native handles are not candidates: they stay in every trial, as background.

A trial deactivates the candidates the engine left out by relaxing them to `-infinity()` or `infinity()`. It never removes a row and never changes the matrix, so each trial is a re-solve of the same model, and writes only the sides whose state differs from the previous trial's.

The objective is zero during the trials. No trial can then be unbounded, and every feasible point is optimal, so a MIP trial can stop at its first integer point. The sense stays as you set it: minimizing or maximizing zero is the same problem. On `highs_qp` the zero objective clears the Hessian too, so its trials are LPs.

Each trial's status becomes a verdict:

| Status of the trial | Verdict |
| :--- | :--- |
| `infeasible` and its refinements, or exactly `infeasible_or_unbounded`, since a zero objective cannot be unbounded | `infeasible` |
| `optimal_infeasible_unscaled`, `failed` and its refinements, `unbounded` | `inconclusive` |
| any other status with a solution (`status::solution_available`), such as `optimal`, or a time limit reached with an incumbent | `feasible` |
| any other status without one, such as `unknown`, or a limit reached without an incumbent | `inconclusive` |

A run makes at most one solve per candidate plus one, only two when a crossed pair decides it, and none on a model without variables, see [below](#two-cases-decided-early). `max_solves` counts these `solve()` calls, so a budget of one more than the number of finite sides never stops a run.

### What a run changes

| | During the trials | Afterwards |
| :--- | :--- | :--- |
| Candidate bounds and sides | at their value or relaxed, as each trial needs | the values read when the run started |
| Objective | zero, offset included | the coefficients and offset copied when the run started, and the Hessian on `highs_qp` |
| Time limit | forwarded, see [Time limits](#time-limits) | the value read when the run started |
| Status | that of each trial | `unknown` after a run that solved, unchanged otherwise |
| Sense, matrix, variable types, special constraints, verbosity, tolerances, other limits | unchanged | unchanged |

Nothing is written before the first trial, so a run stopped before it leaves the model untouched. The model is restored on every exit. After an exception from a trial, the restore runs before the exception reaches you. On a normal exit, if writing one item back fails, the others are still restored, and the first error is thrown.

The objective is saved as a copy of its terms, with the offset read apart, since on several backends `get_objective()` is a view over the solver's live coefficients, which would read back the zeros of the trials. Code of your own that saves an objective to restore it needs the same care: `materialize(model.get_objective())` copies the terms, and `get_objective_offset()` gives the offset.

What stays unchanged still applies to every trial. A verbose model prints the log of each trial. The model's other limits, and its time limit when the run has no deadline, bound each trial as they bound any solve: a trial they stop without a point is inconclusive.

### Two cases decided early

A model without variables needs no solve, and gets none. Every row's activity is then 0, so the first row side that 0 violates, a lower side above 0 or an upper side below 0, in the order of `constraints()`, is the IIS, and without one the model is feasible. The comparison with 0 is exact. No limit stops this case, as `max_solves`, the deadline and `stop_token` are never checked, but a NaN or negative `time_limit` still throws `std::invalid_argument`. The model and its status stay as they were.

A variable whose bounds cross, lower above upper, or a row whose sides cross, is infeasible on its own. The first such pair, variables before rows, stands for the proof of the whole model, and the engine continues from it with every other candidate relaxed: a first trial keeps only the upper side of the pair, and a second only the lower side, or neither side if the upper side sufficed. They decide whether both sides are needed or one suffices, as the lower side of a row without terms suffices when it is above 0. The solver never receives the crossed pair. Limits that stop the run before the two trials leave the pair as the answer, `not_proven_minimal`, with the stop's reason. HiGHS repairs sides that cross by less than its primal feasibility tolerance, while this test is exact: see [Deletion filter](../solvers/index.md#limitation-deletion-filter) in the notable limitations.

### Time limits

On a model with a time limit (`has_time_limit`) and under a finite `time_limit`, the run reads the model's own limit once, before its first trial. Each trial then runs under the smaller of the time left before the deadline and that limit, and the model's limit is written back, exactly, on every exit:

```cpp
--8<-- "test/doc_snippets/deletion_filter.cpp:forwarded-limit"
```

Each trial runs under 5 s at most, and under less once less than 5 s remain of the minute. Afterwards `get_time_limit()` reads 5 s again. With an infinite `time_limit`, the default, the run never reads or writes the model's time limit.

A trial that has started runs to its end, so the deadline can be overrun where the forwarded limit does not bound a trial:

- `clp_lp`, `glpk_lp` and `glpk_milp` have no time limit, so their trials run without one;
- on `cbc_milp`, the forwarded limit did not bound the root LP of a trial on the Cbc build where this was measured, a `devel` build; Cbc 2.10 was not measured.

See [Deletion filter](../solvers/index.md#limitation-deletion-filter) in the notable limitations. A stop requested through `stop_token` also waits for the running trial to end.

### The status afterwards

After any run that solved, the solver holds the solution of a trial rather than of your model, so `get_status()` reports `unknown`, after an exception too. Call `solve()` again before reading a solution. A run that solved nothing leaves the status as it was: one decided without variables, or stopped before its first trial. The reset is `reset_status()`, described in [Resetting the status](../solving/status-and-limits.md#resetting-the-status), which algorithms of your own can use the same way.

### The cost of a trial

Whether a trial starts from the work of the previous one depends on the backend. In our measurements:

| Model | Trials | Why |
| :--- | :--- | :--- |
| `glpk_lp` | warm | GLPK's setters keep the basis. |
| `highs_lp`, `highs_qp` | warm | HiGHS starts from the previous basis. |
| `soplex_lp` | warm, mostly | A trial that frees the active side of a nonbasic row reloads the LP and solves from scratch. |
| `highs_milp` | cold | Each trial runs the whole MIP solver. |
| `mosek_lp` | cold | MOSEK's default optimizer for an LP, interior point, does not warm-start. |
| `mosek_milp` | cold | MOSEK tries the previous integer solution as a first incumbent, but solves the root relaxation from scratch. |
| `scip_milp` | cold | Any change returns SCIP to its original problem, so each trial presolves again. |
| `cbc_milp` | cold on a MIP | Each MIP solve runs on a private copy of the model, see [The backends](../solvers/index.md#the-backends). |

`clp_lp`, `glpk_milp`, and `cbc_milp` on a model without integer variables, were not measured.

### Solver gaps

The trials inherit what each solver proves. Three of the notable limitations in [Choosing a solver](../solvers/index.md#feature-support) concern the filter:

- [Deletion filter](../solvers/index.md#limitation-deletion-filter): the time limits above, and HiGHS's repair of nearly crossed sides;
- [Integrality proofs](../solvers/index.md#limitation-integrality-proofs): wrong answers of Cbc 2.10, and endless branching of Cbc and `glpk_milp`, on some integer rows;
- [Deletion filter on SCIP binaries](../solvers/index.md#limitation-scip-binaries): `scip_milp` throws once a trial relaxes a bound of a binary column.

### Native changes

The filter reads, changes and restores the model through MIP++ only, so the [warning on the native handles](../solvers/index.md#limitation-native-handles) holds. Bounds, rows or special constraints added through `native_model()` and `native_api()` are background the filter cannot name, and an IIS computed over them loses its guarantee. Its save and restore and the enumeration of `variables()` and `constraints()` do not see such changes either.

## Next

[Choosing a solver](../solvers/index.md) — which backends provide duals, `add_column`, IIS routines, and the rest.
