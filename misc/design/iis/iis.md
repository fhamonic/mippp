# IIS computation: design decisions

Design note for the infeasibility-diagnosis feature (roadmap item "IIS").
It records the rulings taken on 2026-09-22 while reviewing the external IIS
pull request, those of 2026-09-27 on [iis_pr_plan.md](iis_pr_plan.md), the
plan for adapting it, the amendments of 2026-09-28 from a simplification
review held before wave 2, and the rulings of 2026-09-29 on the questions
wave 3 left, so that the feature is coded once, in the library's own shape.
A *Confirmed reading* spells out a short ruling, as the maintainer confirmed
it on 2026-09-27. It is not user documentation. The implementation order is
in [iis_todo.md](iis_todo.md), and [Open questions](#open-questions) says
that none remains.

Statements about solvers were checked on 2026-09-22 against the C headers and
runtime probes of Gurobi 12.0.1, CPLEX 22.1.2, COPT 8.0, Xpress 47.01, HiGHS
1.10.0 and 1.15.1 and the SCIP 9.2.1 and 10.0 sources. On 2026-09-27 the HiGHS
headers from 1.11.0 to 1.15.1 were read, and `Highs_getIis` was run on 1.15.1.
The time limit was probed on each of the 15 classes with `has_time_limit`,
against a real solver library. Later that day each native IIS routine ran
under a time limit through its C API: Gurobi 11.0.3, 12.0.1 and 13.0.2, CPLEX
22.1.1 and 22.1.2, Xpress 45.01 and 47.01, COPT 8.0.5, and HiGHS 1.14.0 and
1.15.1. `COPT_SolveLp` ran under a time limit on 8.0.5 too. Notes on these
probes mark a run *measured*, a vendor manual *documented*, and a reading of
either *inferred*. None of the analyses of 2026-09-27 was built with CMake:
the time-limit probes, the enumeration prototype, the sanitizer simulation of
the HiGHS and CPLEX removal code, and the filter sketch. Other releases of the
validated ranges were not checked. Line numbers are on main at `f5e833f`
unless marked otherwise.

## Shape of the API

- **Native IIS routines.** `has_iis<T>` is a concept in `model_concepts.hpp`
  that answers one question: does the model have a native IIS routine (Q1 b)?
  `compute_iis()` is then a member of the native backends only. It takes no
  argument (N9) and never runs the deletion filter (Q1 b). The model's time
  limit bounds each call as a fresh budget, on HiGHS through a per-call copy
  into `iis_time_limit` (N29 a). Other model limits may also stop it, as
  [Native routines](#native-routines) lists per backend. `has_iis` stays a
  single-parameter whole-model concept, like `has_lp_basis`.
- **A public deletion filter.** The deletion filter is a separate,
  solver-agnostic algorithm of the library, in two public layers
  (Q1 b): an engine over a user-supplied oracle, which knows nothing about
  models, and a free function over any model that meets its requirements
  concept. The section on
  [the deletion filter](#the-deletion-filter-a-public-algorithm) describes
  both.
- **Callers choose the path.** Generic code tests `has_iis<M>` for the
  native routine and the free function's concept for the filter. A model
  that meets both, such as `highs_lp` once it has row bounds, offers both.
  The two paths may then report different IISs of the same model, each
  valid.
- **Explicit computation of the model as it is.** Both paths are distinct
  calls, like `refine_lp_status()`: the member `compute_iis()` and the free
  function `compute_iis_by_deletion`. Each is expensive and may be partial, so
  it never runs behind the caller's back. Each analyzes the model as it
  currently is and never relies on the status of an earlier solve, which is
  stale as soon as the model is modified. A feasible model is an outcome, not
  an error.
- **An IIS object shaped like a basis.** Both paths return the same snapshot
  template by value, like `get_basis()`, and nothing is stored in the model
  (Q2 a). Its `get_status(v)` and `get_status(c)` return a variant over the
  tags of an `iis_status` namespace, mirroring `basis_status`: `absent`,
  `member`, and the refinements `member_lower`, `member_upper` and
  `member_both`. Each variant lists only the tags its path can report. The
  free function always knows the side, so it uses `absent` and the three
  sided tags, for variables and rows alike, and HiGHS decodes the same set.
  Gurobi and CPLEX rows add `member` for equality rows, and `copt_milp`
  reports its equality rows as `member`. The per-entity storage is
  `detail::handle_status_table<Status>`, a `std::vector` of the variant
  itself, two bytes per handle id for every IIS status variant (N45 e), where
  ids past the bound read as the variant's alternative 0 (N16 b). The snapshot
  template requires `iis_status::absent` as that alternative, and
  `get_basis()` later instantiates the same table with `basis_status`
  variants. A backend whose routine has details that the unified tags drop,
  such as a native "possible member" flag or a side the unified set cannot
  name, may keep them through the variant and tag hierarchy (N1 a): refinement
  tags that derive from the unified ones, listed only in that backend's status
  variant, so that `is_a` on a unified tag still gives the generalization.
  This is a guideline for when it is needed, and no work is planned for it.
  The template's arguments are not a commitment: docs and tests obtain the
  type through `model_iis_t<T>`, `iis_by_deletion_t<M>` or `auto` and never
  spell `iis_snapshot<...>`, and `lp_iis<I, T>` checks members, not the shape,
  so that later candidate kinds, integrality first, can come as further per-kind
  tables with their own accessors, never as new tags (N37 a, b).
- **Interpretation through the caller's own ranges.** Handles carry no
  reverse index to the caller's keys, so the consumer iterates its own
  variable and constraint families and queries each entity, as for a basis.
  Once the entity enumeration lands, it may also iterate `model.variables()`
  and `model.constraints()` (N20). The snapshot is taken once and keyed by
  handle id, so that a later `remove_variable` cannot shift it. Handle results
  translate lazily today (`highs_lp.hpp:104-130`), so the snapshot is the
  first eager one. A native snapshot translates native indices through
  `_var_handle` when it is taken. On the remapping backends its id bound is
  `_remap_ids ? _native_ids_map.size() : N`, where N is the native column
  count. `clp_lp` is not on the remapping layer: it keeps removed columns
  fixed at [0, 0] and recycles their ids. The free function keys its answer
  by the handles it enumerated. Every query is then a constant-time lookup
  and never a solver call. Member counts are the only summary, and member
  ranges are not needed.
- **A snapshot describes the model as it was.** A handle created after the
  snapshot may reuse a removed id, and would then read the entry of the
  removed variable (N8 a). The entity enumeration's snapshot follows the
  same rule.
- **A completion status of its own.** `get_outcome()` returns a `std::variant`
  per path over the tags of namespace `iis_outcome` (N44), as `get_status()`
  does over the `status` tags: `irreducible` and `feasible` under
  `completed`, and under `incomplete` the runs that decided nothing,
  `inconclusive_trial` when a trial of the filter could not decide (N2 b) and
  `stopped` for a run cut short from outside, refined into `interrupted` and
  the `limit_reached` causes `time_limit`, `solve_limit`, `iteration_limit`,
  `node_limit` and `memory_limit`. Every tag carries `conflict_available`,
  read by `iis_outcome::conflict_available(o)`: the members form a subsystem
  proven infeasible, minimal or not. Every native routine can end partially:
  Gurobi `IISMinimal`, COPT `IsMinIIS`, Xpress `IISSOLSTATUS`, HiGHS's "maybe
  in conflict". CPLEX's abort statuses come with "possible member" flags, but
  these prove nothing (see Native routines), so a CPLEX stop holds no
  conflict. The unified tags have no possible-member tag (N1 a).
  Retained, untested and native "possible" members get `member_*` under a tag
  of the `incomplete` branch with `conflict_available`, plain `member`
  included where the routine does not name the side. A native answer not
  proven minimal while no limit stopped it is plain `incomplete`: the routine
  itself did not prove minimality, and there is no limit to raise (N27 a, in
  the spelling of N44). A tag names a stop's cause where the routine reports
  it: CPLEX's conflict status, Gurobi's `Status` on `gurobi_lp` after a stop
  that left no subsystem, COPT's status of the solve around the search.
  Xpress reports that a stop occurred but not its cause. Elsewhere the time
  measured around the call attributes a stop to the time limit: on HiGHS
  (N34 a), and on Gurobi, Xpress and COPT (N29 evidence).
- **Each model class explains its own problem.** The IIS of a `*_milp`
  model explains the MIP and that of an `*_lp` model the LP, following the
  2026-07-22 ruling that `*_milp` models expose no LP-only feature.
  Integrality is fixed background, never a member, and no domain flag is
  needed. A native routine that only analyzes the relaxation is therefore
  used on `*_lp` models only, and `highs_milp` has no `compute_iis()`.
- **Scope: linear rows and variable bounds.** SOS, indicator, quadratic and
  general constraints have no portable handle, so both paths keep them as
  fixed background. No backend implements `add_sos1_constraint` or
  `add_sos2_constraint`, so `has_sos1_constraints` holds nowhere and SOS exist
  only through `native_model()`. Indicators exist only on `gurobi_milp` and
  `cplex_milp`. The free function leaves the background active and claims
  nothing without a solve, except on a column-less model (N6 b): a crossed
  pair is decided by two trials that run with the background in place (N36,
  amending N7 a2), so no type-level list of background capabilities exists.
  Background added through `native_model()` is outside the guarantee (N23 a),
  and the free function's guard and the entity enumeration do not see native
  changes either.
  `docs/solvers/index.md` warns that modifying the model
  through the native handles invalidates MIP++ features, and the IIS page
  restates it for IIS (WP17). Native wrappers keep the background out of the
  candidates: Gurobi's `IISSOSForce`, `IISQConstrForce` and
  `IISGenConstrForce` set to 1, Xpress's `IISOPS` class bits, and CPLEX by
  leaving them out of the conflict groups. These mechanisms exist in the
  headers, but their effect on special constraints was not probed. COPT's
  routine has no such switch and counts native SOS and indicator constraints
  among its candidates, so `copt_milp` reports `irreducible` only when the
  answer names every one of them or no linear member, and otherwise
  `incomplete` with a conflict (N45 g). A wrapper
  that cannot keep them out must not claim that the reported rows and bounds
  conflict on their own. On either path, `irreducible` with zero members means
  that the background alone is infeasible.
- **Names.** No generic identifiers such as `result`, `options` or `member`
  in a namespace users are told to open. The tags live in `iis_status`. The
  other names are the namespace `iis_outcome`, `iis_limits`,
  `lp_iis<I, T>`, `lp_iis_outcome<O>`, `model_iis_t<T>`, `has_iis<T>`,
  `iis_by_deletion_t<M>`, `iis_sides` and `iis_status::sides_of`, from the
  recommendation of Q2, N44 and N45. Under N19, the names of the
  engine, the free function, its concept and the snapshot start with
  `deletion_` or `iis_`, except the verb `compute_iis_by_deletion`. The
  outcome is a tag hierarchy (N44, which overturns N19 e).

## Native routines

Outcome names in this section predate N44; the current mapping is N44 (c) to
(e) under [Rulings of 2026-10-05](#rulings-of-2026-10-05).

| Solver, probed release | Entry point | Explains | Row sides | Bound sides | Partial answer | Stopped by `set_time_limit` |
| --- | --- | --- | --- | --- | --- | --- |
| Gurobi 11.0.3, 12.0.1, 13.0.2 | `GRBcomputeIIS` | the MIP | membership only | `IISLB`, `IISUB`, never a binary's | `IISMinimal`, 10005 after a stop with no subsystem | yes, documented and measured |
| CPLEX 22.1.1, 22.1.2 | `CPXrefineconflictext` | the MIP | membership only | lower, upper | none: the "possible" flags of an abort status prove nothing (p10, p13 below) | yes, documented and measured |
| COPT 8.0.5 | `COPT_ComputeIIS`, after `COPT_Reset` and, on a MIP, a solve stopped at its first incumbent | the MIP | per side, reliable on LPs only | per side, one bound of a two-bounded column on MIPs, continuous too | `IsMinIIS`, `HasIIS` 0 after a stop | yes, on both classes, measured, undocumented; the confirming solve shares the budget |
| Xpress 45.01, 47.01 | `XPRSiisfirst`, `XPRSgetiisdata` | the MIP | `L`, `G`, or `E` where integrality needs both sides; `R` rows whole on a MIP | `L`, `U`, `F` | `IISSOLSTATUS`, `p_status` 3 with `NUMIIS` 1 | yes, measured, implied by the manual |
| HiGHS 1.15.1, routine from 1.12.0, floor 1.14.0 | `Highs_getIis` | the relaxation | per side | per side | "maybe in conflict" | no, `iis_time_limit` replaces it, the option documented, the replacement read in the sources and measured |
| SCIP 10.0 sources | `SCIPgenerateIIS`, `SCIPgetIIS` | not examined | a sub-SCIP | a sub-SCIP | irreducible flag | not examined |

The last column comes from the time-limit probes of 2026-09-27 and the
wave 5 probes of 2026-09-29, on the releases the rows name. The wave 5
probe programs and outputs stayed outside the repository; the bullets below
record what they established.

- **Gurobi and CPLEX.** Neither names the side of a row. An inequality row
  gets its side from its sense; an equality row is reported as `member`.
- **Gurobi.** Probed on 11.0.3, 12.0.1 and 13.0.2 on 2026-09-29, with
  identical results unless said otherwise. A feasible model returns 10015,
  solved or not, the empty model included; the call solves an unsolved
  feasible LP (`Status` 2, optimum held) and keeps the solution of a solved
  one. A solved infeasible LP keeps `Status` 3 with `SolCount` 0. A stop
  overwrites `Status` with the limit code on an LP (9 time, 7 iteration,
  16 work) and leaves 1 on a stopped MIP, and overwrites `Runtime`. On a
  model with SOS, quadratic or general constraints, writing a force
  attribute and updating discards the held solution before the call
  (`Status` 1) and the IIS attributes after the restore. Time limit: a
  3000 by 6001 LP under 2 s returns 0 at 2.000 to 2.001 s with `IISMinimal`
  10005 and no subsystem; a market split MIP under 1 s returns a 4-row
  subsystem, verified infeasible, with `IISMinimal` 1 at 0.95 to 0.97 s on
  11.0.3 and 13.0.2 and `IISMinimal` 0 at 1.012 s on 12.0.1, which the
  elapsed-time rule sorts out. A limit of 0 leaves a row conflict
  unanswered, but a singleton bound-row conflict answers in full, since the
  cheap checks run before the clock, and a small model an earlier solve
  proved infeasible may: on 12.0.1 a six-row conflict answered in full under
  a zero limit and a 200-row chain did not (a measurement the user page
  recorded before wave 5, not one of the wave 5 probes). The force
  attributes, measured at 1: a needed indicator gives
  linear members irreducible relative to it, with `IISGenConstr` 1; an
  unneeded one is forced in and the answer unchanged; a conflict among
  indicators alone gives `IISMinimal` 1 with zero linear members; SOS and
  quadratic rows behave alike, and a column deleted between two calls is
  handled. A binary z under `z >= 2` names the row alone, the [0, 1] bounds
  being implicit in the type (documented in 10.0 and 13.0); an integer z in
  [0, 1] names the row and its upper bound. A column-less `0 >= 1` names
  the row. `IterationLimit` 0 returns 0 with 10005 and `Status` 7, solved
  or not. Zero-length attribute reads return 0. The 10.0 documentation
  lists `GRBcomputeIIS` and the six `IIS*Force` attributes, and the three
  local headers declare them, so every symbol is bound mandatory. The
  wrapper (`70374bf`) forces `IISSOSForce`, `IISQConstrForce` and
  `IISGenConstrForce` to 1 for the call and restores them through a guard
  whose restore runs every write-back before checking the first error;
  `IISMinimal` 0 gives `not_proven_minimal`, 10005 after return code 0
  `undetermined`, 10015 `feasible`, and `time_limit` is attributed when the
  measured time reached the limit. Numerical trouble with no limit can
  also give `IISMinimal` 0, seen on 13.0.2 under `IISMethod` 1 only, which
  the wrapper never sets. Gurobi 10, the range floor, is not installed.
- **CPLEX.** `CPXrefineconflict` is deprecated since 20.1, so the wrapper
  calls `CPXrefineconflictext` with one group per row and per bound; its group
  preferences are also the hook for forcing later. The refiner replaces
  `CPXgetstat` with its own status (31 minimal, 30 feasible), which MIP++ does
  not see because it caches the solve status. Statuses 32 to 39 end a
  refinement early (documented): 32 on a contradiction, then aborts on the
  time limit (33), the iteration limit (34), the node limit (35), the
  objective limit (36), the memory limit (37), a user abort (38) and the
  deterministic time limit (39). 33, 34, 35 and 39 were measured. The call
  returns 0 on each, and each group reads as a member, a possible member or
  excluded. Under a limit of 0 s, 22.1.1 and 22.1.2 returned status 33 with
  every group excluded, as did 0.001 s on 22.1.1, which is no answer at all,
  and a limit of 0.003 s on 22.1.1 marked every group possible, the whole
  model (measured). The probes of 2026-09-29 (p10, p13, on 22.1.1 and
  22.1.2) showed that no stop carries an answer, so the wrapper maps every
  status from 32 to 39 to `undetermined` with no member, `time_limit` as the
  reason for 33 only, and never `not_proven_minimal`: a feasible market split
  stopped by the time limit returns 33 with every group possible, exactly as
  the infeasible one does, and a feasible dense LP under a tiny limit all
  excluded then all possible, so the flags of a stop prove no infeasibility
  (p13); and a node-limit stop (35, two groups possible and 41 excluded, two
  seeds, both releases) flagged rows that re-solve MIP optimal, because the
  refiner excluded groups whose sub-MIP had only hit the limit (p10). No
  mixed answer under 33 or 34 was seen in about 25 stops. Two more facts
  shape the call. A stopped refinement resumes when the same groups are
  passed again on an unchanged problem and then returns 30, feasible, on an
  infeasible model, caching that answer (undocumented, p7, p11): the wrapper
  passes a per-model counter, bumped on every call, as the equal preference
  of every group, which makes each call fresh; the counter is a member of
  the model and moves with it, and the second review of 2026-09-29 pinned
  in `48a59cd` that a moved model refines afresh rather than resuming the
  aborted state. And a completed answer is
  cached by CPLEX until a data edit (`chgrhs`, `chgbds`, `chgcoef`,
  `chgsense`, `chgctype`, `newcols`, `addindconstr`, `addsos`), not by an
  objective or parameter change (p7, p13). A native simplex iteration limit on a
  MIP makes `CPXrefineconflictext` itself fail with 3019 (p9), unreachable
  through MIP++ since `cplex_milp` has no `set_iteration_limit`. The final
  review of wave 5 (2026-09-29) measured two more facts. `CPXrefineconflictext`
  returns 1811 (`CPXERR_UNSUPPORTED_OPERATION`) while a generic callback is
  registered, whatever the callback does, on a feasible and on an
  infeasible MIP, and returns 0 with status 30 once
  `CPXcallbacksetfunc(env, lp, 0, nullptr, nullptr)` detached it, so
  `cplex_milp::compute_iis()` detaches a registered callback for the call
  through a guard like COPT's and re-registers it on every exit, pinned by
  a test (0 calls, `feasible`, the callback back for the next solve). And
  a 34 stop on `cplex_lp` outlives its limit on the unchanged problem: the
  next calls, fresh preference included, end 32 (a contradiction) with no
  member, after an objective change too, until a bound or a side is
  written, even to its current value, after which the call answers 31
  (60-row chain, 22.1.1 and 22.1.2), where a 33 stop is forgotten as soon
  as the limit is raised. The wrapper writes nothing after a 34 stop; the
  user page names the way out and the iteration-limit test pins both. The wave 5
  probes added: a feasible model returns 0 with status 30 and
  `CPXgetconflictext` fails with 1719, solved or not; the held solution survives
  the refiner (objective, point, primal feasibility) while `CPXgetstat` reads 30
  or 31 and `dfeasind` flips from 1 to 0 on a MIP, so the status and the
  solution disagree and the reset stays (N15). Every time-limit stop is 33, 0.7
  to 15 ms late, deterministic over 30 runs under 0 s; a node limit of 0 gives
  35, whose stop can come seconds late on a hard model, an iteration limit
  of 0 gives 34 on an LP and 3019 on a MIP, and a
  deterministic time limit 39. A column-less row is named as the violated
  member. Crossed bounds read both `MEMBER`. Indicators and SOS, left out
  of the groups, are background, and a conflict among indicators alone is
  31 with zero members. Row bounds (`9c4e9b6`, `2d3d909`): sense `R` with
  `rhs` and `rngval` is native, `rngval > 0` giving [rhs, rhs + rngval] and
  `<= 0` the reverse (documented and measured); the setters write `E`, `G`,
  `L` or `R` from the two sides and rewrite the range after every switch to
  `R`, since `chgsense` did not zero it although the manual says it does;
  crossed sides have no encoding and throw `std::invalid_argument`; the
  sense and rhs getters, `get_constraint`, `set_constraint_rhs` and
  `set_constraint_sense` throw `std::runtime_error` on an `R` row, whose
  sense the mapping read as `>=` before. Community Edition 22.1.2 refines
  the m = 3 market split in 5 to 16 s against 0.1 to 1.9 s on 22.1.1.
  Measured on 22.1.1 and 22.1.2; 22.1.0, the range floor, is not installed.
- **COPT.** On MIPs it flags a single side of an equality row even when both
  are needed: integer x with `x = 0.5`, integer x with `2x = 1.5`, and
  integers x, y with `x + y = 1.5` each got one side only. An LP IIS never
  needs both sides of one row, so the flags are consistent on LPs. `copt_milp`
  therefore reports equality rows as `member`. `copt_lp` got its time limit
  in wave 1 (`d205cf6`, N35 a), and under N29 (a) the model's limit bounds a
  native call on both classes. On 8.0.5, a stop before any subsystem
  leaves `HasIIS` at 0, and a feasible model returns the generic code 3
  (measured). Two findings of 2026-09-27 needed care: on a time-limited MIP
  IIS equal to the whole model, 101 rows and 200 columns, the per-entity
  getters flagged two rows and one column, against `IISRows` and `IISCols`,
  and one `IsMinIIS` = 1 answer on a 20-column, 10-row binary model was
  feasible when re-solved. Wave 5 (2026-09-29, on 8.0.5 only: 7.2.5 loads
  and binds every IIS symbol, but the local license refuses
  `COPT_CreateEnv` there) settled the rest. `COPT_ComputeIIS` returns its
  previous answer after bound, row or objective changes and after a solve,
  only `AddCol` and `AddRow` invalidating it, and returns code 3 without
  looking at a model whose `LpStatus` is optimal, so the wrapper calls
  `COPT_Reset(prob, 0)` first, which clears the cache, the statuses and the
  held solution and keeps the parameters (measured). With `IsMIP` 1 the
  routine flags the whole model with `HasIIS` 1 and `IsMinIIS` 1 on an
  unsolved feasible MIP, on a stale infeasible status and on `INF_OR_UNB`,
  and after a solve to optimal returns 3 like an LP (24 of 24 and 29 of 29
  random models), so `copt_milp` always solves first: optimal or unbounded
  gives `feasible`, infeasible runs the routine under the remaining budget,
  a timeout is `undetermined` with `time_limit`, a stop with an incumbent
  `feasible`, and `INF_OR_UNB` or a node-limit stop `undetermined` with no
  reason. That solve fires a registered candidate-solution callback, whose
  lazy rows could make a feasible model solve infeasible and the routine
  then flag the whole model as irreducible, so the callback is detached
  for the call and re-registered afterwards (measured: 0 calls,
  `feasible`). The second review of 2026-09-29 found that solve running
  to optimality, seconds on a hard feasible model whose first incumbent
  already settles the question, so the wrapper registers its own
  `COPT_CBCONTEXT_INCUMBENT` callback calling `COPT_Interrupt` once the
  user's is detached, and detaches it again before the solve's return
  code is checked: the m = 5 market split answers `feasible` in 4 ms
  with `MipStatus` 10 and `HasMipSol` 1 against 2 s for the full solve,
  the same interrupt from a `COPT_CBCONTEXT_MIPSOL` callback stops before
  the candidate is committed and leaves `HasMipSol` 0, and the interrupt
  does not outlive the call, a later solve running to optimal (measured
  on 8.0.5; `f73ac7a`, pinned in `8baba3f`). On the LP path the routine never
  solves, so a code 3 gets one confirming `COPT_SolveLp` under the remaining
  budget: optimal or unbounded is `feasible`, a timeout `undetermined` with
  `time_limit`, and infeasible throws. The routine segfaults on a row-less model
  without SOS or indicator whose bounds are LP-feasible, the empty model
  included, and reports an integer column whose interval holds no integer as an
  empty `HasIIS` 1 answer or as code 3 after an infeasible solve, so the wrapper
  decides such a column from its bounds, clamping a `COPT_BINARY` column
  to [0, 1] first since COPT rejects one whose bounds exclude 0 and 1; a
  binary bound beyond the domain is the member alone (`lb` 2 alone or
  `ub` -2 alone solve infeasible, `ub` 3 or `lb` -3 alone optimal,
  measured in the second review), while crossed bounds or an interval
  without an integer need both, since freeing either readmits a value.
  Since `2e52aa8` that column arithmetic is
  `detail::iis_self_infeasible_column` in
  `include/mippp/detail/iis_arithmetic.hpp`, over (lower, upper, kind),
  beside the column-less precheck and the side that 0 violates, which
  HiGHS's crossed term-less row uses too: the "shared detail arithmetic" of
  N28 (a), which does not depend on the deletion filter. Its kind `other`
  covers column types whose admissible values it does not decide, such as
  semi-continuous ones, and never qualifies. COPT answers exactly as
  before, and Xpress is its second user (`5c2192f`).
  The routine also leaks 48 bytes in 4 allocations on a column-less model
  (LeakSanitizer in the sanitized build on 8.0.5, the two
  `column_less_model_names_the_violated_row` cases exiting 1), so the
  wrapper answers a model without columns from its constant rows, the
  first side that 0 violates, or `feasible`, without calling the routine,
  as N28 (a) allows a wrapper that fails the shared case (final review of
  wave 5, `f73ac7a`, pinned in `8baba3f`). On
  a MIP COPT flags one side of an equality row (206 flagged rows, none with
  both flags) and one bound of a two-bounded column where both are needed,
  continuous columns included (second review: continuous x in [0.5, 1.5],
  integer y and x + 2y = 4 flag the upper bound of x alone, and the
  subsystem with x's lower bound freed re-solves optimal), and of 176
  random binary answers with `IsMinIIS` 1, 6 re-solved feasible with only
  the flagged sides and none with the flagged columns kept whole, so
  `copt_milp` reports rows and two-bounded columns whole;
  `copt_lp` and `IsMIP` 0 models decode per side, whose flags are complete.
  A stop whose `IISRows` and `IISCols` disagree with the flags (5 and 40
  against 1 and 1 on a time-limited market split) is `undetermined`;
  `IISCols` is documented as a count of bounds but 8.0.5 counts columns,
  and the wrapper accepts either. `TimeLimit` bounds the routine,
  undocumented: an 18040-row LP under 0.01 s returns `HasIIS` 0 after
  18.5 ms, a market split under 1 s at 1.0025 s with a non-minimal answer,
  0 s in 0.2 ms, and a 60000 by 20000 LP under 0.02 s 44 ms late, inside
  the routine itself. `NodeLimit` does not stop the routine, which it slows
  from 1.9 to 5.8 s, but stops the confirming solve. Held solution and
  statuses survive the routine itself (`LpObjval`, the point), but the
  reset the wrapper needs drops them, so the status reset stays (N15).
  Indicators and SOS are listed as members natively (`IISIndicators`,
  `IISSOSs`) with the linear members relative to them; the snapshot never
  counts them. Row bounds (`c2af30c`): COPT stores two sides natively,
  `SetRowLower` on `x <= 3` gives [1, 3], a freed side reads ±1e30, and any
  side at or beyond ±1e30 reads ±1e30, so the getters of WP6c, which read
  `COPT_DBLINFO_LB` and `UB`, were already consistent.
- **Xpress.** By default integrality restrictions are removable candidates,
  listed as `I` members. With integers x, y and rows `y = 0` and
  `x + y = 1.5`, the default returned both rows, which is not irreducible once
  integrality is background; with the `IISOPS` integrality bits set it
  returned `x + y = 1.5` alone. `IISOPS` bits mark element classes as fixed,
  and fixed elements are still listed, so the wrapper sets the bits and drops
  the `I` entries. Rows map `L` to `member_upper`, `G` to `member_lower` and
  `E` to `member_both`, confirmed at 45.01, the range floor, and on 47.01
  in wave 5 (2026-09-29): `E` never appears on an LP,
  `one_side_of_an_equality_row` gives `L` or `G` on LP and MIP, and `E`
  comes only where integrality needs both sides; bound types `U`, `L` and
  `F` (a fixed column in a MIP IIS: x in [1, 1], integer y, x + y = 0.5
  gives row `E`, x `F`, y `I`) map to sides, `F` to `member_both`, and `B`
  and `I` entries are dropped. The final review of wave 5 (2026-09-29)
  found `XPRSgetiisdata` listing one column twice, `U` then `L`, where a
  MIP IIS needs both bounds of a continuous column (x in [0.5, 1.5],
  integer y, x + 2y = 4, both releases), which note 8 of its manual entry
  allows; the decoder replaced the first entry by the second and answered
  `member_lower` alone, a subsystem that re-solves feasible, so it now
  merges a second entry of one entity into `member_both`, rows included
  (`cd3b3bd`, pinned in `a12f76d`). The same review measured a registered
  preintsol callback firing inside `XPRSiisfirst`, once on a feasible
  2-column MIP and 28 times on a feasible 3 by 30 market split, none on
  the infeasible 4 by 26 one, a rejecting callback turning both feasible
  models into `irreducible` answers naming the whole model, where
  `GRBcomputeIIS` fired the callback 0 times on the same models, so
  `xpress_milp::compute_iis()` removes the callback with
  `XPRSremovecbpreintsol` for the call and adds it back on every exit,
  pinned by a test (0 calls, `feasible`, the callback back for the next
  solve). A ranged row whose two sides are both needed under integrality
  was reported `L` only: an integer x in [-10, 10] under 1.25 <= x <= 1.75
  came back `irreducible` with the row's upper side alone, while x <= 1.75
  alone admits x = 1 (45.01 and 47.01), where `copt_milp` and `cplex_milp`
  answer the row as a plain member. Since `916db02`, `xpress_milp` reads the
  row types after the call (`XPRSgetrowtype`) and reports an `R` row
  flagged on one side as a plain member, as COPT does on a MIP, so its row
  status lists absent, member and the three sided tags; an `E` row keeps
  the mapping measured for it, and `xpress_lp` stays sided. The flag cannot
  tell a row that needs both sides from one that needs the flagged side
  alone, so every such `R` row on a MIP is reported whole. The final review
  of 2026-10-02 found that rule applied to every `xpress_milp`, while
  integrality is what makes the routine drop the second side: since
  `f89be76` it applies only when the original problem has MIP entities
  (`XPRS_ORIGINALMIPENTS` > 0), as `copt_milp` keys its rules on `IsMIP`,
  and an `xpress_milp` whose columns are all continuous keeps the side, as
  `xpress_lp` does, pinned by
  `xpress_milp_iis_test.ranged_row_keeps_its_side_without_integers`. The decoder
  accumulates per-side flags before mapping them, which keeps merging an
  entity listed once per side into `member_both`. A
  feasible model gives `p_status` 1, `IISSOLSTATUS` 1 and `NUMIIS` 0, after
  which `LPSTATUS` and `MIPSTATUS` keep reading optimal while the objective
  attributes read 0 and `XPRSgetsolution` fails with 422, so status and
  solution disagree and the reset stays (N15). The routine reads
  `TIMELIMIT`: the market split under 1 s returns `p_status` 3,
  `IISSOLSTATUS` 3 and `NUMIIS` 0, under 3 s `NUMIIS` 1 with four `E` rows
  not proven minimal, and `TIMELIMIT` 0 stops every model, trivial and
  feasible ones included, before any answer (`IISSOLSTATUS` 0);
  `STOPSTATUS` is unreliable. `p_status` 3 is any stop: `XPRSinterrupt`, a
  checktime interrupt and `LPITERLIMIT` 1 all give it under `TIMELIMIT`
  1e20, and 42 limit stops on both releases returned from 9.2 ms early to
  10 ms late, Xpress reading its clock in 10 ms steps, so the wrapper
  attributes `time_limit` when the measured time plus a 20 ms slack reached
  the limit, and no reason otherwise. Under load the same 10 ms stop of the
  market split also returns `p_status` 0 with `IISSOLSTATUS` 0 and `NUMIIS`
  0 (1 of 160 calls with four processes, 9 of 18 processes with six, on
  47.01, 2026-10-02), which the wrapper read as an answer without a reason,
  so `xpress_milp_iis_test.market_split_stop_is_a_time_limit_stop` failed
  once in a parallel run: such an unstarted, empty answer now counts as a
  stop, attributed by the same rule. `p_status` 2 ("?727 Warning: Bound
  conflict on column; IIS will not continue") comes on crossed or
  integer-empty column bounds even when the conflict is elsewhere, with
  `XPRSgetlasterror` empty: a crossed continuous column beside an unrelated
  row, an integer column in [0.25, 0.75], a binary with lower bound 2 and a
  binary with upper bound -2 all threw `solver_error` on 45.01 and 47.01.
  Such a column is an IIS by itself, so since `5c2192f` the wrapper
  answers that refusal by the arithmetic COPT uses,
  `detail::iis_self_infeasible_column`, over the bounds and the types it
  reads at the call (`XPRSgetcoltype`,
  present in 45.01 and 47.01, mapping `C`, `I` and `B` and leaving every
  other kind to `other`): the first column that qualifies is the answer,
  `irreducible`, crossed bounds or an integer interval without an integer
  giving `member_both` and a binary bound beyond the domain the bound alone.
  The throw stays where no column qualifies, which a test pins on a
  semi-continuous column with crossed bounds. The types are read at the
  call because Xpress retypes a binary on a bound write, measured with
  `XPRSgetcoltype` on 45.01 and 47.01 on 2026-10-02: a binary given the
  bounds [2, 3], [-3, -2], [-3, 1] or [0, 3] reads type `I` and solves
  optimal, while [2, 1] and [0, -2] keep it `B` and solve infeasible. A
  fractional bound of a binary or integer column is rounded inward when it
  is written, unless the rounded bound would cross the other, which then
  keeps the value written: an integer column in [-10, 0] given the lower
  bound 0.25 reads [0.25, 0], and a binary given 0.25 then 0.75 reads
  [1, 0.75]. A column whose type changes to integer on [0.25, 0.75], as
  `add_integer_variable` does, reads [1, 0]. On a feasible model with no
  row and one integer column in [0, 1], `xpress_milp::compute_iis()`
  answers `undetermined` on 45.01 and `feasible` on 47.01, where
  `xpress_lp` and a continuous column answer `feasible` on both
  (2026-10-02). Under
  the default `IISOPS` both releases abort the process on the 4-row,
  26-binary market split ("Error in calculation of row activities"); the
  wrapper's bits (integrality, general, PWL, SOS and indicator constraints
  fixed, delayed rows left as candidates) complete it in 5.6 s on 47.01
  and 3.8 to 15.1 s on 45.01. Indicator rows list with contype `I` and
  sets with a set index under either `IISOPS`, and are dropped. A
  column-less `0 >= 1` names the row `G`. The routine restores a
  MIP-presolved problem itself, and a time-limited `lpoptimize` leaves the
  LP original on 47.01 and LP-presolved on 45.01, answered on both. The
  second review of 2026-09-29 measured what that LP-presolved state does
  on 45.01: after a stopped solve of a 1901 by 1700 LP, `XPRS_ROWS` and
  `XPRS_COLS` read 1401 and 1400, and so do `num_constraints()` and
  `num_variables()`, a row getter or setter on a late row fails with
  error 406 ("Invalid row range passed to XPRSgetrowtype"), nothing can
  be added while the problem is presolved (documented), and the wrapper's
  tables, sized before the call, threw `std::out_of_range`;
  `XPRSpostsolve` restores the counts and does nothing on an original
  problem (documented). So `xpress_lp::solve()` postsolves after
  `lpoptimize` as `xpress_milp::solve()` did (`3acad2c`, a test pinning
  the rows addressable after a stopped solve on both releases), and
  `_compute_iis()` sizes its tables after `XPRSiisfirst` returns
  (`cd3b3bd`). 45.01
  solves an LP with crossed column bounds and a MIP with an integer column
  holding no integer to optimal where 47.01 says infeasible, a defect the
  suites skip from the case data below 47.1; since `5c2192f` the native
  suite runs those cases on 47.01 too, where it skipped them on every
  release while the wrapper threw. Row bounds (`35eb0af`): a
  ranged row is type `R` with `rhs` the upper side and a non-negative
  range, `XPRSchgrhsrange` normalizing a negative range by moving the rhs
  (documented and measured), so crossed sides throw
  `std::invalid_argument`; Xpress has no sense or rhs getter. The
  community license caps a problem at 5000 rows plus columns (error 120).
  Measured on 45.01.01 and 47.01.01; 46 was not probed.
- **HiGHS.** The routine analyzes "an LP, QP, or the relaxation of a MIP"
  (1.15.1 header). `Highs_getIis` is absent from the 1.8.1, 1.9.0, 1.10.0 and
  1.11.0 headers and entered the C API in 1.12.0, while the validated range
  starts at 1.8.1, so the symbol is optional. It has three regimes. In 1.12
  the strategies are marked work in progress and the statuses are numbered 0,
  1 and 2. 1.13 makes the strategies bitmasks and renumbers the statuses -1, 0
  and 1, so 1.12's constants would read in-conflict as not-in-conflict. From
  1.14, the option `iis_time_limit`, default inf (documented), replaces
  `time_limit` during the search (1.14.0 and 1.15.1 sources, measured), and
  the call zeroes its clocks when the replacement applies. It does not bound
  the whole call: the cheap checks run before it applies, and a post-check
  re-solves the returned set with no limit (1.14.0 and 1.15.1 sources), which
  on 1.15.1 once returned after 16.5 s under a 1 s limit (measured). The
  default strategy, Light, finds only trivial conflicts: on 1.15.1 it found
  nothing on a non-trivially infeasible LP. The 2026-09-22 probe, where
  integer x with `x = 0.5` gave an empty IIS, ran under Light and proved
  nothing. The wrapper therefore sets `iis_strategy` to 6. It copies
  `get_time_limit()` into `iis_time_limit` for each call and restores it
  afterwards (N29 a), since `time_limit` alone had no effect on 1.14.0 and
  1.15.1 (measured). A stop returns -1 with nothing written when it comes
  early, and 1 with a partial set, all its entries "maybe", later (on 1.15.1,
  after an elasticity-filter stop, the whole model). On every 0 or 1 return
  `Highs_getIis` overwrites every status entry (1.14.0 and 1.15.1 sources),
  so the wrapper's -1 pre-fill is only a default; after a failed post-check
  the 1 return copies from HiGHS's own status vectors, which the failure
  left empty, an out-of-bounds read inside HiGHS worth an upstream report
  that no pre-fill on the wrapper's side can absorb. On 1.14.x, a stop in the
  elasticity filter returns -1 with model status 8, `kInfeasible`, which
  N26's mapping alone would turn into a throw. Under N34 (a), which amends
  N26, a -1 return is `undetermined` with `time_limit` when the time measured
  around the call reached the limit copied into `iis_time_limit`, whatever the
  model status, and throws `solver_error` otherwise. The same elapsed-time
  test attributes `time_limit` to a "maybe" answer. The C API exposes no IIS
  status, so the elapsed time is the only signal that separates a limit stop
  from an error. With the full strategy, 1.15.1 crashed inside `Highs_getIis`
  on an integer-infeasible MIP. `highs_lp` and `highs_qp` get `compute_iis()`,
  and `highs_milp` gets none, pinned by `static_assert(!has_iis<highs_milp>)`.
  Below 1.14.0, `compute_iis()` throws `solver_error` with no fallback (N26
  and Q1 b). An empty `library_version()` requires the symbol and assumes the
  newest regime. The row arrays take the row counts, although the 1.12.0
  header documents column counts (`highs_c_api.h:2428-2442`).
  Wave 3 fixed two native answers on 2026-09-28. HiGHS stores the matrix
  row-wise when a row brings more nonzeros than it holds, and the post-check
  of `Highs_getIis` then reads past an array on an answer of one row and no
  column, since `HighsIis::setLp` writes starts per column only (read,
  identical in 1.14.0 and 1.15.1; valgrind showed the reads, and a clang build
  crashed). The wrapper first calls `Highs_deleteColsByRange(model, 0, -1)`,
  an empty range that makes the matrix column-wise, deletes nothing and
  returns before the model status and basis (`5d05cb0`); an all-zero mask
  would reach an assertion in `HighsHessian::deleteCols` on an assert-enabled
  build. And for a crossed row without terms, `Highs_getIis` reports both
  sides before its empty-row check (`HighsIis::trivial`), which is not
  minimal, so the decoding keeps the side that 0 violates (`496f19e`). Both
  defects, and the cumulative clock of the forwarding bullet, are worth
  upstream reports.
- **SCIP.** No routine up to 9.2.1. SCIP 10.0 added IIS finder plugins whose
  answer is a sub-SCIP flagged infeasible and irreducible. Mapping it back
  to handles was not examined, so `scip_milp` starts on the free function.
- **The others.** MOSEK, Clp, Cbc, GLPK and SoPlex have no IIS routine;
  MOSEK's infeasibility report and `MSK_primalrepair` are not one.
- **Time limits.** The routines of Gurobi, CPLEX, Xpress and COPT stop
  under the parameter that `set_time_limit` writes: on the wave 5 probes
  within about 1 ms (12 ms once), 0.7 to 15 ms, 10 ms before or after, and
  2.5 to 44 ms of the limit (measured on 2026-09-29; the probes of
  2026-09-27 had seen 0.05 s, 0.43 s, 0.25 s and 25 ms). Gurobi and CPLEX
  document it, the Xpress manual implies it, and COPT does not mention it.
  HiGHS documents and applies its own `iis_time_limit` instead. Each call
  gets a fresh budget (measured on all five), so `solve()` followed by
  `compute_iis()` may take twice the limit. Other limits stop some routines
  too: CPLEX's iteration and node limits (measured) and its memory limit
  (documented), Gurobi's `SoftMemLimit`, which `set_memory_limit` writes
  (documented), Gurobi's `WorkLimit`, which MIP++ does not write (documented
  and measured), Gurobi's `IterationLimit` on `gurobi_lp`, with no answer
  (measured), COPT's `NodeLimit`, which stops the confirming solve but not
  the routine (measured), and on Xpress an interrupt or a native
  `LPITERLIMIT`, without a reason (measured). HiGHS
  lifts its simplex iteration limit during the call (read in the 1.14.0 and
  1.15.1 sources), and since wave 4 a test pins on 1.15.1 that a zero
  iteration limit stops `solve()` but not `compute_iis()`, with a Hessian too
  (measured on 2026-09-29). A stop can leave no answer at all on every backend
  (measured). Under N29 (a), the model's time limit bounds each call as a
  fresh budget, and HiGHS gets it through the per-call copy into
  `iis_time_limit`, restored afterwards. Under N9, the other limits may also
  stop it. The docs state per backend that the budget is per call, which other
  model limits also stop the routine, and that a stop may return late or with
  no answer. `copt_lp` got its time limit in wave 1 (`d205cf6`, N35 a), and
  since wave 5 the model's limit bounds the whole call on both COPT
  classes, the confirming solve and the routine sharing it, the remainder
  written for the second step and restored. An explicit duration per call
  is only a Deferred possibility (N29 b), and leaving the call unbounded
  (N29 c) or to the solver's own rule (N29 d) is rejected.
- **The status after a native call.** A native `compute_iis()` sets the status
  to `unknown` before its first native call, so the status is `unknown`
  whether the call returns or throws (N15). This suspends the 2026-09-22 rule
  that native routines leave the model's status untouched, until a probe
  shows, for a given routine, that the held solution and the status agree.
  That routine then leaves the status untouched, as recommended, and its
  native-suite case changes with it. The probes record their results in this
  note. The maintainer confirmed this reading on 2026-09-27, so a routine
  whose probe passes leaves the status untouched without a new ruling. So
  far, on HiGHS 1.15.1, `Highs_getIis` re-solves: an unsolved feasible LP
  comes back optimal with a primal point, and an infeasible one comes back
  infeasible. On 1.14.0 and 1.15.1 it also resets the run clock (measured).
  The N15 probe of 2026-09-28 on 1.15.1, through `compute_iis()`, found that
  after `Highs_getIis` the model status and the held solution agree on a
  feasible model, solved or not (optimal, `primal_solution_status` 2, with a
  primal point that satisfies every row and bound, and on a solved model the
  solve's own point and objective, bit-identical), and on an infeasible one
  (infeasible, `primal_solution_status` 0 or 1, no feasible point). A zero
  `iis_time_limit` did not stop the probe's feasible models, for two
  different reasons: a solved one never reaches the limit, since HiGHS exits
  early on an optimal status, while an unsolved one is re-solved under the
  copied budget and the probe's tiny LP simply finished before the clock was
  checked (a larger one returns -1 with status 13, `undetermined` with
  `time_limit`); both calls returned `kOk` and `feasible`. On an infeasible
  model a `kError` stop leaves status 8 on a solved model, with the column
  values unchanged and the info invalidated (`primal_solution_status`
  unreadable), and status 13 with an infeasible point on an unsolved one,
  neither claiming a feasible solution. `highs_lp` and `highs_qp` may
  therefore leave the status untouched, which their native-suite case then
  asserts; as of 2026-09-28 both still reset it and the suite asserts
  `unknown`; N46 closed the follow-up, keeping the reset.
  CPLEX's refiner replaces `CPXgetstat` with its conflict statuses. Gurobi's
  `GRBcomputeIIS` overwrites the attributes `Status` and `Runtime`: a stopped
  call turned an infeasible LP's status 3 into 9, the time limit (measured on
  13.0.2). COPT's `COPT_ComputeIIS` leaves `Status`, `LpStatus`, `MipStatus`
  and `HasLpSol` as the solve set them, stopped or not (measured on 8.0.5).
  The wave 5 probes of 2026-09-29 completed the four. CPLEX: the held
  solution survives, but `CPXgetstat` reads the conflict status and
  `dfeasind` flips on a MIP, so status and solution disagree and the reset
  stays. Xpress: after a feasible answer the status attributes still read
  optimal while the objective reads 0 and the solution is gone (422); the
  reset stays. COPT: the routine itself leaves the held solution and every
  status untouched, but the `COPT_Reset` the wrapper needs against the
  answer cache drops them; the reset stays. Gurobi: status and solution
  agree on a model without special constraints, an unsolved feasible one
  solved to its optimum, a solved one keeping its point, an infeasible one
  keeping 3 with `SolCount` 0, and a stop writing the limit code while
  claiming no solution; but on a model with SOS, quadratic or general
  constraints the forcing writes discard the held solution while a cached
  `optimal` would stand, so the reset stays on both classes, and leaving
  the status untouched on a model without such constraints was a follow-up
  like HiGHS's, which N46 closed.
- **Wrappers restore what they set.** Each native wrapper restores its own
  parameters through a small RAII helper: HiGHS `iis_strategy` and
  `iis_time_limit` (N29 a), Xpress `IISOPS`, Gurobi's `IIS*Force` attributes
  (N15), COPT's `TimeLimit` for the second step and, on `copt_milp`, the
  detached callback, the wrapper's own incumbent callback being detached again
  before the confirming solve's return code is checked, and the callbacks
  `cplex_milp` and `xpress_milp` detach for the call, re-registered on every
  exit; CPLEX sets nothing, its groups being arguments of the call, which a
  test pins. The free function's guard saves model data and lives in its
  detail namespace, so it is not reused, which departs from the WP7 guard of
  the N15 recommendation. Since `228d25e` every native guard follows the rule
  of the free function's guard: `restore()` attempts every item and raises the
  first error once all ran, a constructor that fails writes back what it set,
  and the `noexcept` destructor writes back only when `restore()` did not run.
  HiGHS's `restore()` used to stop at its first rejected write, with no
  rollback in its constructor, and COPT's time-limit guard and the three
  callback guards used to retry once in their destructor.
- **Split.** Native, where `has_iis` holds: `gurobi_*`, `cplex_*`, `copt_*`,
  `xpress_*`, `highs_lp` and `highs_qp`. The free function, once each model
  has the capabilities it requires: `clp_lp` (WP8), `cbc_milp` (WP9),
  `highs_lp`, `highs_milp` and `highs_qp` (WP10), `glpk_*` (WP11), `mosek_*`
  (WP12), `scip_milp` (WP13) and `soplex_lp` (WP14). `copt_*`, `cplex_*` and
  `xpress_*` joined it in wave 5, on 2026-09-29, with modifiable row
  bounds, each solver confirmed to store a row's two sides natively
  (N30 b): COPT as two bounds, CPLEX as an `R` row with a range, Xpress as
  type `R` with a range. `gurobi_*` has no modifiable row bounds, since its
  ranges add a slack column, and joined on 2026-10-02 through its rows' sense
  and rhs (N41). `dumb_lp` joined the same day, through the same fallback
  (N24, N43). The work packages are those of
  [iis_pr_plan.md](iis_pr_plan.md). Since wave 3, on 2026-09-28, all eleven
  classes of the free function's list run it, since wave 5 the six COPT,
  CPLEX and Xpress classes too, and since N41 the two Gurobi classes, every
  model class in all; `has_iis` holds on the ten classes of the native list.

## The deletion filter, a public algorithm

- **Two layers, both public.** The engine knows nothing about models, and the
  free function runs it on a model, in place (Q1 b). Under N19 both live in
  `include/mippp/utility/`, beside `column_manager`, today the library's one
  public algorithm over capability concepts: `deletion_filter` in
  `utility/deletion_filter.hpp`, and `compute_iis_by_deletion(model, limits)`
  with its concept `iis_by_deletion_model` in `utility/iis_by_deletion.hpp`.
- **The engine contract.** The engine takes a candidate count, an oracle and
  the limits. It invokes the oracle as an lvalue with the indices of the
  active candidates, and the oracle answers feasible, infeasible or
  inconclusive. Monotonicity is a precondition: every subset of a feasible
  set of candidates is feasible. `inconclusive` is always allowed and never
  deletes a candidate. An exception from the oracle propagates unchanged.
  The result lists the members in ascending order, with the outcome and the
  reason. Without batching, a run makes at most one oracle call per
  candidate, plus one.
- **The free function.** It accepts any model that meets its requirements
  concept: `lp_model`, the entity enumeration, readable and modifiable
  variable bounds, readable and modifiable row bounds (Q3 a), a readable
  objective, `has_status_reset` (N22), and a readable quadratic objective on a
  `qp_model`. `lp_model` already has `set_objective` and
  `set_objective_offset`, so `has_modifiable_objective` is not required. It
  never retypes a column, so `milp_model` needs no branch. It adapts through
  `if constexpr` on capability concepts, never on backend types, and only
  where behavior differs: `qp_model` and `has_time_limit` (Q6 a; the branch
  on the capabilities that can carry background constraints was dropped by
  N36 with `detail::may_carry_background`). Its first mutation of the
  model waits for its first trial. It never touches the sense, the verbosity,
  the tolerances, the other limits or the matrix. A sketch of it compiled
  against main's headers and passed its runtime checks on stub models.
- **In place, on the caller's model.** No copy of the model is built. The
  free function saves what its trials change, runs them on the model, and
  restores everything through an RAII guard on every exit path, including
  exceptions and cancellation. The restore attempts every item and rethrows
  the first error. Special constraints and lazy-constraint machinery stay in
  the model as fixed background, which a copy would have dropped.
- **Saved by value.** Variable bounds and row sides are read as scalars. The
  objective coefficients are copied into owned storage: `get_objective()` is
  a lazy view over the solver's live coefficients on Clp, GLPK, Cbc
  (`cbc_milp.hpp:139-148`) and SCIP (`scip_milp.hpp:219-231`), so a saved
  view would read back the zeros written for the trials. The objective
  offset is saved separately. On a `qp_model` the quadratic part is saved
  too, its Hessian triples copied into owned storage through
  `has_readable_quadratic_objective`, since `set_objective` replaces the
  whole objective. The time limit is saved only when a finite deadline is
  forwarded into it.
- **Deactivate by relaxing, never by removing.** A side is switched off by
  setting it to the backend's own `infinity()` with the matching sign. The
  matrix never changes, so the model is re-solved in place. Row removal and
  slack columns are both rejected: the first would need a
  `has_remove_constraint` capability with row remapping in every backend,
  the second doubles the column count. Warm starts depend on the backend, as
  wave 3 found on 2026-09-28 (measured unless marked). Warm: `glpk_lp`, whose
  setters keep the basis (1 iteration after freeing a row, against 30 from
  `glp_std_basis`); `highs_lp` and `highs_qp`, which start from the saved
  basis (21 pivots after halving a bound, against 149 from scratch, and 0
  after a relaxation), `highs_qp`'s trials being LPs since the first one
  clears the Hessian; and `soplex_lp`, except that a trial which frees a
  nonbasic row reloads the LP and solves cold (1 reload in 6 trials on one
  case, 2 in 5 on the chain, none on bounds against a row, 7.1.3). Cold:
  `glpk_milp`, whose presolve rebuilds the problem on every `glp_intopt`
  (read); `highs_milp`, which runs the whole MIP solver each time (a median of
  2.4 ms over 604 trials, against 0.54 ms for the same continuous model,
  1.10.0); `mosek_lp`, whose default interior-point optimizer cannot
  warm-start (the same iteration count on every re-solve); `mosek_milp`, whose
  root relaxation reruns cold, although MOSEK checks the previous integer
  solution as a starting incumbent (`MSK_IINF_MIO_INITIAL_FEASIBLE_SOLUTION` 0
  on the first solve and 1 after bound changes), which only seeds the
  incumbent; `scip_milp`, whose setters free the transformed problem, so each
  trial presolves again (1 to 24 ms per case on 8.0.4); and `cbc_milp`'s MIP
  trials, which the devel build copies into a fresh `CbcModel` seeded with the
  previous best solution as a checked MIP start, and which run on a fresh
  `Cbc_Model` copy below 3.0. Cbc's LP path through OsiClp forces
  `initialSolve()` after any row-side change, and after a column-bound change
  that touches the basis (read).
- **One side at a time.** Each finite variable bound and each finite row
  side is a candidate, ranged rows included (Q6 a). Row sides are read
  through `has_readable_constraint_bounds` and relaxed through its twin
  `has_modifiable_constraint_bounds<T, M = T>` (Q3 a), so equality and
  ranged rows need no branch. A lower side is a candidate when it is above
  `-infinity()`, and an upper side when it is below `infinity()`. A side at
  the wrong infinity is therefore a candidate, not ignored, although N40
  makes setting one undefined behavior, so the docs no longer present it as
  an input. Crossed bounds are decided by the prechecks below.
- **Trials are feasibility problems.** The objective is set to zero for the
  duration of the loop, so no trial can be unbounded and branch-and-bound
  stops at its first incumbent. The sense is left as the caller set it:
  minimizing or maximizing a zero objective is the same problem. No
  emphasis parameter is touched. The related solver entry points are
  feasibility relaxations (`CPXfeasopt`, `GRBfeasrelax`, `COPT_FeasRelax`,
  `XPRSrepairinfeas`, `Highs_feasibilityRelaxation`, `MSK_primalrepair`)
  and MIP emphasis settings (CPLEX, Gurobi `MIPFocus`, SCIP); none solves a
  feasibility problem other than through a zero objective. A
  `set_feasibility()` operation, if ever added, is sugar over
  `set_objective(zero)`, never a third sense.
- **Only proofs count, read under the zero objective.** A visitor over
  `model_status_t<M>` classifies each trial, since `is_a<status::failed>`
  does not compile on `clp_lp`'s variant. Anything derived from
  `infeasible`, and exactly `infeasible_or_unbounded`, proves infeasibility.
  The latter is a proof because a zero objective cannot be unbounded, and
  SCIP, SoPlex, MOSEK and HiGHS can report it.
  `optimal_infeasible_unscaled`, anything derived from `failed` whatever its
  solution flag, and `unbounded`, which a zero objective makes impossible,
  are inconclusive. Anything else proves feasibility exactly when
  `status::solution_available` holds, so `time_limit{true}` is a proof. An
  inconclusive trial never drops a member, and an inconclusive singleton
  test prevents the irreducible claim.
- **Sound status mappings first.** The proofs are only as sound as each
  backend's status mapping, and three fixes on main came first, landed on
  2026-09-28 as `283a256`, `1a77969` and `daddd47`. GLPK maps
  `GLP_INFEAS` to infeasible. HiGHS reads `psolstatus` uninitialized after a
  load or solve error. Clp reports `optimal` whatever its secondary status
  (`clp_lp.hpp:535-544`). Secondary statuses 2 and 4 leave unscaled primal
  infeasibilities, so a trial could falsely prove feasibility. An infeasible
  model would then end `feasible`, or a removable candidate would stay
  under a false `irreducible` claim. After the N13 fix, Clp reports
  `optimal_infeasible_unscaled` for them, which the classifier treats as
  inconclusive. Secondary status 3 leaves only dual infeasibilities and
  stays `optimal`. Secondary status 1 needs nothing: with status 4 it is
  already `unknown`, and with status 1 it means a dual limit, which
  zero-objective primal trials do not hit.
- **Algorithm.** The deletion filter: prove the model infeasible as it is,
  then test each candidate once. A candidate is dropped only when the
  remainder is still proven infeasible. A feasible witness stays valid when
  more candidates are removed later, so one pass suffices. Batching is ported
  dormant for the moment (N3 b), behind a detail parameter that the free
  function never sets. Candidate ordering is a later refinement.
- **Limits.** A defaulted `iis_limits {max_solves, time_limit, stop_token}`
  is the parameter aggregate of the free function and the limits of the
  engine (Q5 c). `max_solves` counts `solve()` calls. The duration becomes one
  deadline when the call starts. A NaN or negative duration is rejected before
  anything is read, and an infinite or huge one saturates to no deadline.
  Limits act between trials only. Before a trial, a stop request beats the
  deadline, which beats the solve count. A stopped run keeps the last proven
  infeasible subset and reports the stop in the completion status. A proof on
  the last permitted trial is complete.
- **Forwarding the budget.** Where `has_time_limit` holds and the deadline
  is finite, the free function reads the caller's time limit once. It gives
  each trial `std::min(remaining, saved)`, in that argument order, so a NaN
  saved limit forwards `remaining`. It restores `saved` exactly on every
  exit. With the default limits it never calls `get_time_limit` or
  `set_time_limit`. Forwarding relies on the
  [time-limit contract](#library-additions-the-free-function-needs) of
  N4 (A): `get_time_limit()` is never negative. `copt_lp` gained
  `has_time_limit` in wave 1 (`d205cf6`, N35 a), and the last three classes
  without it, `clp_lp`, `glpk_lp` and `glpk_milp`, gained it on 2026-10-02
  (`e2d577f`, `577f90e`): every model class now has it, so the filter
  bounds every trial, and the forwarded-limit cases of
  `IisByDeletionTest`, gated on the concept, reach Clp and GLPK.
  `Clp_setMaximumSeconds` turns a limit into a
  deadline on the process's user CPU clock (`CoinCpuTime`, `getrusage`
  `ru_utime`), counted from the call, and `Clp_maximumSeconds` returns that
  deadline, so `clp_lp` keeps the limit and arms it in each `solve()`; the
  C API has no wall-clock setter, `setMaximumWallSeconds` being
  `ClpModel`'s alone. Clp checks it in the outer loop of its primal
  simplex, at refactorizations, and other threads count: on the `devel`
  build linked to OpenBLAS, an 800-column dense LP under 1 s stopped after
  0.31 to 0.34 s of wall-clock time with 1.22 to 1.31 s of user time
  (four runs), and with `OPENBLAS_NUM_THREADS=1` a 1600-column LP stopped
  at 1.01 s on both clocks (2026-10-02). Clp status 3 is a stop on
  iterations or time and only the secondary status 9 names the time
  (`ClpModel::onStopped`), so `clp_lp` reports `time_limit` there and
  `limit_reached` otherwise, each carrying `Clp_primalFeasible` as its
  solution flag; the three symbols exist in every 1.17 release. GLPK's
  `tm_lim` holds whole wall-clock milliseconds: a finite limit is rounded
  up, since GLPK stops at its first check under 0, the requested duration
  is kept for `get_time_limit()`, `INT_MAX` is no limit, and a negative or
  NaN limit throws `solver_error`, since a negative `tm_lim` makes GLPK
  abort the process (`xerror` in `glp_simplex` and `glp_intopt`).
  `glpk_lp` now reports `GLP_ETMLIM` as `time_limit`. The search of
  `glp_intopt` stops once `tm_lim - 1` ms have passed (`glpios03.c` of
  5.0), so `glpk_milp` passed one millisecond more: without it, a trial
  ended just before the forwarded deadline and the run reported
  `inconclusive_trial` rather than `time_limit`. Before 4.63, `glp_time`
  also truncates to the millisecond, so the check can fire almost one
  millisecond before `tm_lim - 1` have passed: on GLPK 4.59, the floor of
  the validated range, a filter trial with the one millisecond still ended
  up to 0.86 ms before the deadline and the run reported
  `inconclusive_trial`. `glpk_milp` now passes two milliseconds more
  (`fa45782`), so a stop overruns the limit by three at most, and the
  endless-row test passes 10 of 10 on 4.59 and on 5.0 (2026-10-02). The MIP
  presolver checks no clock, then the root LP (`smcp.tm_lim = parm->tm_lim`) and
  the search (its clock started when the tree is created) each get the full
  limit: a 500 by 500 dense integer model under 0.2 s took 2.0 to 2.2 s in all,
  1.7 to 1.8 s of it in the presolver (GLPK 5.0, 2026-10-02), so a
  `glpk_milp` trial can overrun the deadline by its presolve and root LP.
  Forwarding first ran on real solvers
  in wave 3 (2026-09-28, measured): on `cbc_milp`, every trial saw exactly a
  caller's 7 s limit under a 3600 s budget, and more than 3500 s and at most
  3600 s under a caller's 7200 s, both limits restored exactly
  (`cbc_time_limit_test`). HiGHS needed a fix first (`35e6d20`): its time
  limit counted all the time a model had spent in HiGHS, since `Highs_run`
  does not reset the clocks, so a re-solve under a limit below that total
  stopped at once with 0 pivots. Each solve now calls `Highs_zeroAllClocks`,
  present from 1.8.0, so `set_time_limit` bounds each solve on HiGHS, a
  release-note item. MOSEK restores a caller's 3 s exactly, and a fresh task's
  -1 holds +inf after a budgeted call, both meaning no limit. SCIP restores a
  caller's 30 s exactly. The shared case
  `IisByDeletionTest.forwarded_time_limit_is_restored` pins the same two
  checks as `cbc_time_limit_test` on every backend with a time limit: Cbc,
  HiGHS, MOSEK, SCIP and SoPlex. Cbc's limit does not bound its root LP: on the
  devel build a 1500-row dense relaxation ran 8.5 s under a 1e-6 s limit, so a
  Cbc trial can overrun the deadline too.
- **Stop reasons.** After any inconclusive trial that ran, initial or
  singleton, the run ends `inconclusive_trial` (N2 b), or `time_limit` when
  the deadline has passed as the trial returns. `solve_limit` and
  `interrupted` apply only when the engine stops before a trial. With
  `max_solves = 1`, an inconclusive initial trial therefore ends
  `inconclusive_trial`, never `solve_limit` (outcome names as of N44).
- **Prechecks.** One case is decided by arithmetic, without a solve, and it
  is free: it runs and answers whatever the limits (N7 a). On a model with no
  live variable, the first finite row side with lower > 0 or upper < 0 is the
  sole member, irreducible, and none means feasible (N6 b). No solve could
  answer there: Clp, Cbc, HiGHS and SoPlex return `unknown` on a model
  without columns, and MOSEK reported `optimal` until the fix of 1.6. The
  comparison with 0 is exact and documented, and "first" means first in
  enumeration order.
- **Crossed pairs.** A variable whose bounds cross, or a row whose sides
  cross, is a known proof of infeasibility: the free function skips the
  initial trial and the engine continues from that pair, so two singleton
  trials decide it, on every model type and with the background in place
  (N36, amending N7 a2). The solver never receives the crossed pair in a
  trial, since each singleton trial relaxes the other side, and the two trials
  run on an otherwise fully relaxed model. The pair alone is the proof, so the
  answer is a subset of it whatever the rest of the model holds. A crossed row
  needs both trials because one of its sides can be infeasible alone, as on a
  row without terms, and a crossed variable because the background may make
  one side infeasible alone. Under a budget that stops before the trials, the
  pair is reported with the stop's tag and a conflict, and the model
  is left as it was. The former zero-solve claim on crossed variables, and the
  type-level list of background capabilities it needed, are dropped: a
  capability added later could not silently make it unsound. On `scip_milp`
  the trials meet the gap of the next bullet.
- **A documented gap on SCIP.** SCIP creates [0, 1] integer columns as
  `BINARY`, and `scip_milp` keeps them so (N25). The N14 probe of WP13 ran on
  2026-09-28 on SCIP 8.0.4 and 9.2.1, with identical results (measured): SCIP
  neither clamps nor rejects the relaxed bound when it is set. A bound of such
  a column relaxed to `infinity()` is accepted and reads back as the infinity
  written, and the next `SCIPsolve` fails, printing "invalid bounds [-1e+20,1]
  for binary variable" (`var.c:1972`), which `scip_milp` throws as
  `std::runtime_error("scip_milp: error in input data")`. So
  `compute_iis_by_deletion` throws on an infeasible model as soon as a trial
  reaches a bound of a `BINARY` column, after restoring the model: binary x in
  [0, 1] with `x >= 2`, and binaries x and y with `x + y >= 3`, come back with
  their data and still solve infeasible. A feasible model with binaries
  returns `feasible`, and `max_solves = 1` returns `not_proven_minimal` with
  `solve_limit`, without a throw. SCIP rounds integer bounds too: [0.25, 2.75]
  reads [1, 2], and [0.25, 0.75] becomes the crossed [1, 0], typed `BINARY`,
  so `integer_in_a_fractional_interval` meets the gap and skips on SCIP.
  Since N39 the skip is keyed on the case's data: the shared cases ask the
  fixture's optional `iis_case_skip_reason`, and SCIP's answers for any
  integer column whose bounds each round inward to 0 or 1, crossed or not,
  as `var.c` requires for `BINARY`, before anything is built (`5047a77`,
  cherry-picked from `1b91b7d`, then narrowed to that rule by the review of
  wave 4). A non-binary integer column works: integer x in [0, 2] with
  `x >= 3` gives `irreducible`. SCIP 10.0.0's `var.c` has the same check
  (read), and SCIP 10.0.2 behaves the same (measured, after the status fix of
  the SCIP row-sides bullet). The crossed-pair trials of N36 meet the same
  gap.
  `binary_column_with_a_relaxed_bound_fails_to_solve` and
  `deletion_filter_throws_on_a_binary_column` pin it, and
  `docs/solvers/index.md` documents it under "Notable current limitations"
  (`146173a`). The design is centred on linear programs, so the gap is
  documented, not worked around: neither the algorithm nor `scip_milp` changes
  to conform to SCIP.
- **Side effects.** After a run that called `solve()`, `get_status()`
  reports `status::unknown`, exceptions included (Q4 a). The model is back
  to its original data, but the solver holds a trial's solution: a complete
  pass ends on a feasible trial whenever the last candidate is necessary
  (`deletion_filter.hpp:206-235` at `a1a9f11`). Restoring an earlier
  `optimal` would pair it with the wrong solution. A run that solved nothing
  leaves the data and the status as they were: one answered by the column-less
  precheck alone, or stopped before any trial, a crossed pair's continuation
  included, by `max_solves = 0`, a 0 s budget or a stop already requested. The
  free function makes the status `unknown` through
  the model's `reset_status()` (N22). The trials run a registered
  candidate-solution callback, which `gurobi_milp`, `cplex_milp`,
  `xpress_milp` and `copt_milp` have, all four under the filter since wave 5
  and N41: its lazy constraints and rejections are background of every
  trial, while the native routines run without it, and N42 documents the
  difference.

## Library additions the free function needs

Compile-checked on 2026-09-22 against the model classes that the free
function targets first, and on 2026-09-27 for `dumb_lp` (x: satisfied). The
row-bound setters, the entity enumeration and the status reset were concepts
that main did not have, so no model satisfied them. The table is that
pre-wave-1 picture: on 2026-09-28 wave 1 brought the enumeration (`3c6348f`),
the status reset (`0046153`, `ce792ec`) and readable row bounds (`9a852f9`)
to every model, and modifiable row bounds to `clp_lp` (`93063d1`). Wave 3
(items 5.1 to 5.6 of iis_todo.md, `5c4399c` to `9aac8a1`) then brought
modifiable row bounds to the ten other classes of the table, and variable
bounds, read and modify, and a readable objective to `soplex_lp`:

| Model | Variable bounds, read / modify | Row sense and rhs, read / modify | Row bounds, read / modify | Objective, read | Enumeration | Status reset |
| --- | --- | --- | --- | --- | --- | --- |
| `clp_lp` | x / x | x / x | x / . | x | . | . |
| `cbc_milp` | x / x | x / . | x / . | x | . | . |
| `glpk_lp`, `glpk_milp` | x / x | . / . | . / . | x | . | . |
| `scip_milp` | x / x | . / . | . / . | x | . | . |
| `soplex_lp` | . / . | . / . | . / . | . | . | . |
| `mosek_lp`, `mosek_milp` | x / x | . / . | . / . | x | . | . |
| `highs_lp`, `highs_milp`, `highs_qp` | x / x | x / x | . / . | x | . | . |
| `dumb_lp` | x / x | x / x | . / . | x | . | . |

- **Row relaxation.** A row side is relaxed through
  `has_modifiable_constraint_bounds<T, M = T>`, the twin of
  `has_readable_constraint_bounds` (Q3 a). Wave 1 gave it to `clp_lp` and
  wave 3 to the ten other classes of the free function's list. Every
  backend the free function targets stores rows natively as two-sided
  bounds, with a way to set each side: `Clp_rowLower` and `Clp_rowUpper`,
  writable arrays that `set_constraint_rhs` already writes through;
  `Cbc_setRowLower` and `Cbc_setRowUpper`; `glp_set_row_bnds`, which takes a
  bound type; `SCIPchgLhsLinear` and `SCIPchgRhsLinear`; `MSK_chgconbound`,
  which relaxes one side and fixes the bound key, with `MSK_getconbound` and
  `MSK_putconbound` to restore; `Highs_changeRowBounds`. The api objects of
  Clp, Cbc, GLPK, MOSEK and HiGHS already bind these functions, so every
  validated release has them. SCIP must bind its setters, and WP6c its getters
  `SCIPgetLhsLinear` and `SCIPgetRhsLinear`. Once its setters can range a
  row, HiGHS's `get_constraint_sense` throws like Clp and Cbc, and its
  `get_constraint_rhs` throws like Clp (N12 b).
- **Readable row bounds on every model.** Under Q6 (a),
  `has_readable_constraint_bounds` (`model_concepts.hpp:476-483`), with
  `get_constraint_lower_bound` and `get_constraint_upper_bound`, comes to all
  19 model classes and `dumb_lp` in WP6c. Main has it on `clp_lp` and
  `cbc_milp` only. Where the solver has no native ranged rows, as on Gurobi,
  the model's getters derive the bounds from the sense and the rhs: a `<=` row
  reads (`-infinity()`, rhs), a `>=` row (rhs, `infinity()`) and an `==` row
  (rhs, rhs), with the backend's own `infinity()`. The branch lives in the
  model's method, never in the IIS code. SCIP, SoPlex, COPT and Xpress bind a
  row getter first, since none is bound on main. Modifiable row bounds still
  follow N30: only where the solver stores a row's two sides natively, so
  never on Gurobi, whose rows the free function writes through their sense
  and rhs instead (N41).
- **Row sides as wave 3 found them.** Measured on 2026-09-28 unless marked.
  - **Cbc.** An infinity beyond ±1e27 is stored as ±`COIN_DBL_MAX`, which is
    `infinity()`. A row the setters make ranged reads sense `R`.
    `get_constraint_sense` already threw on a ranged row, and
    `get_constraint_rhs`, which returned `Cbc_getRowRHS`, the upper side, now
    throws there too, on rows added ranged included (`c492659`; devel and
    2.10.12). The devel build's `Cbc_addRow` returns without
    adding a row that has no terms, which shifts every later row id, so
    `cbc_milp` throws `solver_error` there (`db9ee56`); 2.10 keeps such rows.
    The same build aborts in `Cbc_status` after a MIP whose relaxation is
    infeasible, unbounded or abandoned, so the outcome queries are read first
    (`5c4399c`). Below 3.0 a MIP solve fixes the model's integer columns at
    the incumbent and carries it unchecked into the next solve (CbcSolver.cpp
    of 2.10.5, read, and three failures on 2.10.12), and
    `Cbc_numberSavedSolutions` is as stale as the incumbent, so each MIP solve
    there runs on a fresh copy of the model (`8b5fad9`).
  - **GLPK.** `glp_set_row_bnds` frees a side at ±`infinity()`, which reads
    back as ±`DBL_MAX`, and equal sides become `GLP_FX`. An IEEE infinity on
    the side it cannot free, as -inf for an upper side, was typed as a real
    side and kept as given: `glp_simplex` then answered `optimal` and
    `glp_intopt`'s presolver failed an assertion (`npp3.c:892`), so every
    side written is clamped to ±`DBL_MAX`, which both solve infeasible.
    `glp_simplex` and `glp_intopt` return `GLP_EBOUND` on crossed bounds or
    sides, so `solve()` then reports `infeasible` when a lower side exceeds its
    upper side (`d8c37b3`), and `failed` otherwise. `glp_intopt` also returns
    `GLP_EBOUND` on any fractional bound of an integer column, and never
    finished `x + y = 1.5` over free integers, so `glpk_milp` rounds the sides
    of integer columns, and of rows whose columns are all integer with
    integral coefficients, within the integrality tolerance before each solve
    and puts them back after (`8ee83d2`). Rows with integral sides and no
    integer point, such as `2x + 2y = 1` over free integers, still branch
    until a time limit stops them, which `glpk_milp` only has since
    `577f90e`: under a 0.5 s budget the filter now answers `undetermined`
    with `time_limit` at 0.50 s (`6bb2208`).
  - **HiGHS.** `Highs_changeRowBounds` accepts crossed sides, and a solve
    reports them infeasible, on the three classes, 1.10.0 and 1.15.1. A side
    of magnitude 1e20 or more reads back as ±`infinity()`, and a lower side of
    +inf throws. `Highs_run` silently repairs, in the model itself, bounds or
    sides crossed by less than `primal_feasibility_tolerance`: x in [1 + 1e-9,
    1] solves optimal and reads [1, 1] afterwards, while the free function's
    exact check reports the pair as an IIS. N12 (b) landed as `b05a026`.
  - **MOSEK.** `MSK_chgconbound` gets `finite = 0` at ±`infinity()` and
    beyond, and derives the bound key: equal sides become `FX`, and crossed
    sides stay `RA` with lb > ub, which solves infeasible. A finite side
    beyond `MSK_DPAR_DATA_TOL_BOUND_INF`, 1e16 by default, is stored free and
    reads ±1e30. An IEEE infinity on the opposite side is rejected (error
    1390), while `infinity()` there stays a real bound. The variable-bound
    setters now pass the same flag (`5e70f3d`), so a side set to `infinity()`
    is freed under a raised tolerance too.
  - **SCIP.** `SCIPchgLhsLinear` and `SCIPchgRhsLinear` ignore a change below
    `numerics/epsilon`, 1e-9, so such a change is written through infinity
    first, and 8.0.4 stores a side beyond the opposite infinity as given, so
    sides are clamped to ±`infinity()`. Sides within 1e-9 of each other cannot
    be read back exactly. The variable-bound getters read the original bounds
    (`88f7d3b`): the global ones follow the presolved copy after a solve, so x
    in [0, 10] with `x <= 4` and `x + y <= 7` read [0, -0] once solved, and
    the filter would have restored presolved bounds. A solve that fails after
    the transformation now leaves `_solved` set, so the next setter frees the
    transform (`9ffeff7`). Crossed bounds and sides solve infeasible; a
    crossed row prints a warning on stderr even when the model is quiet.
    SCIP 10 renumbered `SCIP_STATUS`, which `scip_api` declares with the
    numbering of 8 and 9, so on SCIP 10 every status read wrong, an optimum
    as `interrupted`; `scip_milp` now translates SCIP 10's codes. That bug
    predates the wave, and the compatibility matrix, whose cases never
    checked a SCIP status, missed it. With the fix, the SCIP suites pass on
    10.0.2, the library of the PySCIPOpt 6.2.1 wheel (measured).
- **SoPlex.** It joins last (N11 a), and did in wave 3. The N11 check of
  2026-09-28 read the interface of every release from 6.0.4 to 8.0.3:
  `soplex_interface.h` is byte-identical from 7.0.0 to 8.0.3, and its `.cpp`
  is identical within 7.0.0 to 7.1.3 and within 8.0.0 to 8.0.3. Every symbol
  bound is present from 7.0.0, so the floor stays at 7.1.1, and the suites
  pass on 7.1.1, 7.1.3 and 8.0.3 (measured):

  | Symbol | 6.0.4 | 7.0.0 to 8.0.3 | Bound |
  | --- | --- | --- | --- |
  | `SoPlex_getRowBoundsReal` (N11) | . | x | yes, in WP6c |
  | `SoPlex_changeRowLhsReal`, `SoPlex_changeRowRhsReal` (N11) | . | x | yes |
  | `SoPlex_changeVarLowerReal` (N11) | . | x | yes |
  | `SoPlex_changeVarUpperReal` (N11) | x, moves the lower bound | x | yes, refused without its lower twin |
  | `SoPlex_getObjReal` (N11) | . | x | yes |
  | `SoPlex_getLowerReal` (N11) | . | x | no |
  | `SoPlex_getUpperReal` (N11) | x | x | no |
  | `SoPlex_getRowVectorReal`, `SoPlex_basisRowStatus` | . | x | yes |
  | `SoPlex_clearLPReal` | x | x | yes |
  | `SoPlex_setRealParam` | . | x | yes, for the time limit |

  The wrapper keeps state after all: once a solve has scaled the LP in place,
  `SoPlex_getLowerReal` and `SoPlex_getUpperReal` unscale infinite entries too
  (`getLowerUnscaled` has no infinity check), so ±1e100 reads back as ±1.22e96
  or ±2.56e102. `soplex_lp` reads column bounds from a copy it keeps, which is
  authoritative, and rows from SoPlex, whose per-row getter is right. Wave 3
  also fixed, measured on the three releases: an IEEE infinite column side
  made a bounded LP read unbounded, so every written side is clamped to ±1e100
  (`d7ae123`); `add_column` passed the nonzero count as the column length
  (`2bb9321`); freeing the active side of a nonbasic row leaves a nonbasic
  free row from which a warm solve stops `RUNNING` after an internal XLEAVE04
  or returns a wrong optimum, under every setting tried, and the C API cannot
  drop a basis, so `solve()` reloads the LP when a row reports `ZERO`
  (`08eac9c`); a warm solve after crossing a side returned optimal outside the
  sides, so `solve()` reports infeasible without solving when a lower side
  exceeds its upper side (`a07ac25`); and a fresh `soplex_lp` maximized,
  SoPlex's default, which `LpFuzzyTest` exposed once instantiated (`e963d03`,
  a release-note item).
- **Entity enumeration.** Until `3c6348f` main had no public enumeration of
  live entities:
  `model_base::_variables_range` is protected, and no `variables()` or
  `constraints()` member exists. It comes to every model at once, behind
  `has_enumerable_variables` and `has_enumerable_constraints` (N20). Each call
  returns a sized snapshot in increasing id order that holds no reference to
  the model (N21). `clp_lp` must skip its removed columns, which stay in the
  solver fixed at [0, 0] with their ids in `_free_variable_ids`.
- **Status reset.** Only `solve()` and `refine_lp_status()` write a model's
  private `_status`, and a throwing `solve()` keeps the previous status on
  HiGHS, MOSEK, SCIP, COPT, CPLEX, Gurobi and Xpress. The free function
  therefore requires a public `reset_status() noexcept`, with the concept
  `has_status_reset<T>`, which comes to the 19 models and `dumb_lp` at once
  (N22).
- **Time-limit contract.** Under N4 (A), `get_time_limit()` is never
  negative on a `has_time_limit` model. The 2026-09-27 audit of the 15
  classes found only MOSEK negative on a fresh model, at -1 s, which means
  no limit. Cbc and SoPlex can read back a negative value after the caller
  writes one. SoPlex silently refuses values above 1e100 and negative ones,
  keeps its previous limit and still reports the new value. That is a bug
  on main: after `set(0 s)` then `set(+inf)`, a SoPlex solve still stops
  with `time_limit`. SCIP, with a ceiling of 1e20, and CPLEX, with 1e75,
  throw on +inf. Gurobi (1e100) and COPT (1e20) clamp natively. HiGHS, Cbc,
  MOSEK and Xpress store +inf. No solve changes the value read back.
- **The time-limit mapping.** The MOSEK getter maps a negative value to
  `std::numeric_limits<double>::infinity()`, which round-trips because MOSEK
  11.0 accepts +inf. The MOSEK setter stays pass-through. The SoPlex setter
  throws on a negative value, then writes and records `std::min(t, 1e100)`.
  The Cbc setter throws `solver_error` on a negative value, as HiGHS, Gurobi,
  CPLEX, SCIP and Xpress do natively and the fixed SoPlex setter does, and its
  getter stays transparent (N33 b). The SCIP setter clamps to 1e20 and the
  CPLEX setter to 1e75, the values their fresh models report, so +inf and
  `duration<double>::max()` mean no limit on every backend (N32 a). NaN is
  left to each solver, and `iis_limits` rejects it. MOSEK's -1 s is the only
  aberrant no-limit value the audit found, so the earlier reading of the
  ruling's "solver's infinity()" as the time parameter's own no-limit value is
  dropped as moot (N4).
- **Time-limit tests.** Five new `TimeLimitTest` cases run on all 15 classes:
  `fresh_time_limit_is_unlimited`, `unlimited_time_limit_round_trips`,
  `lifted_time_limit_takes_effect`, `negative_time_limit_is_never_read_back`
  and `forwarded_time_limit_restores_exactly`. On main they fail on MOSEK,
  SCIP, CPLEX, SoPlex and Cbc. With five backend changes, the Cbc one being
  the getter of N33 (a), all six cases, `set_get_time_limit` included, passed
  on all 15 classes. The ruled Cbc setter of N33 (b) was not run.
  `negative_time_limit_is_never_read_back` accepts a setter that throws, so it
  should pass too.
- **Behavioral check.** A shared test that `infinity()` frees a row side,
  mirroring `infinity_removes_a_bound`, which already covers variable
  bounds. Its ranged case is gated on `has_ranged_constraints`, which holds
  only on `clp_lp` and `cbc_milp`.
- **Visibility.** Each capability suite is instantiated wherever the
  capability becomes satisfied, so the feature tables show it. The table
  script drops `highs_qp` rows today, since it sorts by `_lp_` or `_milp_`
  (`tested_features_table.py:31-34`), and the documentation step fixes it.

## Tests and documentation

- **Two suites over shared case bodies.** `IisTest` in
  `test/test_suites/iis.hpp` asserts `has_iis` and runs the native routine.
  `IisByDeletionTest` in `test/test_suites/iis_by_deletion.hpp` asserts the
  free function's concept and runs it. They are labelled "IIS, native" and
  "IIS, deletion filter" (N19). Both are instantiated from each
  `test/solvers/<solver>.cpp`, so the feature tables show each path and the
  `MIPPP_REQUIRED_SOLVERS` skip policy applies. The case bodies are fixture
  member functions in `test/test_suites/iis_cases.hpp`. No separate binary, no
  hard-coded backend list. No archetype exists for `has_iis`, as for every
  capability, so the fixtures assert the concepts, and
  `static_assert(!has_iis<highs_milp>)` pins the MIP case (N10 a).
- **Solver-free tests.** In `mippp_test`, `test/deletion_filter.cpp` tests
  the engine over synthetic oracles, with a fake clock through the engine's
  `Clock` parameter. `test/iis_by_deletion.cpp` tests the free function on
  scripted stub models, `test/iis_snapshot.cpp` the snapshot and
  `test/iis_oracle.cpp` the oracle. A derived probe that hides `solve()` and
  the setters is the deduced `M` of the free function, so its `solve()` runs
  in every trial and asserts per-trial invariants.
- **An independent oracle.** Each case builds its model from its own data,
  so it can rebuild the reported subsystem, check that it is infeasible, and
  check that dropping any single reported side makes it feasible. A row
  reported as `member` is kept whole. The exact oracle lives in
  `test/test_suites/iis_oracle.hpp` and serves both suites.
- **Shared cases.** From the pull request's tests and the 2026-09-22 probes:
  bound-only conflicts; an LP conflict through one side of an equality row;
  integer x with `x = 0.5`, which needs both sides of one row; integers x, y
  with `y = 0` and `x + y = 1.5`, where integrality makes the first row
  redundant; integer y in [1/4, 3/4], where only integrality makes the bounds
  infeasible; ranged rows; redundant rows; two disjoint conflicts; a chain
  where every row is needed; crossing bounds; a feasible model; a model
  modified after an infeasible solve; removed variables on a remapping
  backend; answers unchanged after a later `remove_variable` (N8); the
  published HiGHS vectors, scaled, reversed and sign-flipped; each call
  leaving the model to solve to its previous result; a column-less model. Clp,
  Cbc, HiGHS and SoPlex report a column-less model as `unknown` and MOSEK as
  `optimal`. The free function answers it by the N6 precheck. Native wrappers
  run no precheck (N28 a): the case pins each routine's answer, and a wrapper
  that fails it may call the shared detail arithmetic, which is not the
  deletion filter. On HiGHS 1.15.1, with the only column removed, the row
  0 >= 1 already came back as a lower-side member. Since `916db02`,
  `ranged_row_holding_no_integer`, an integer x in [-10, 10] under
  1.25 <= x <= 1.75, pins in both suites on every MILP backend that the
  row needs both sides, which a routine naming one side gets wrong;
  Gurobi's models cannot build a ranged row and skip it.
- **Filter-only cases.** In `IisByDeletionTest`: a budget sweep, where each
  `max_solves` from 0 up makes at most that many solves and keeps a valid
  answer; a stop requested beforehand; a 0 s budget; the Q4 status rule,
  `unknown` after a run that solved and the pre-call status after a
  precheck-only run; the column-less precheck at zero budget; a crossed pair
  under a zero budget, reported as the proven pair, and under two trials; the
  restoration of everything saved; per-trial invariants through a derived
  probe. Time budgets are 0 s
  only, since the pull request's one wall-clock test needed two timing
  fixes.
- **Native-only cases.** In `IisTest`: the status `unknown` after
  `compute_iis()`, whether it returns or throws, for each routine not yet
  shown to keep a consistent status (N15); indicator constraints kept as
  background on `gurobi_milp` and `cplex_milp`, which first needs the fix of
  native ids after a removal; the both-paths check, which calls the free
  function directly on a model that meets both concepts, and which since wave
  3 runs on `highs_lp` and `highs_qp`, passing on 1.15.1 (2026-09-28),
  since wave 5 on the six CPLEX, Xpress and COPT classes, and since N41 on
  the two Gurobi classes, the ten classes with both concepts; the throw below
  HiGHS 1.14.0. Since `5c2192f`, `IisTest`'s `crossed_variable_bounds` and
  `integer_in_a_fractional_interval` run natively on Xpress 47.01, below
  47.1 keeping the data-keyed skip of the deletion fixture, and Xpress
  tests pin a crossed column beside a row, a binary outside its domain in
  three placements, and the refusal that still throws on a semi-continuous
  column. Since `a6c19f3` the cases each backend had copied are shared:
  `zero_time_limit_is_a_time_limit_stop` on every native class, with an
  integer variant on the MIP classes, `registered_callback_does_not_run`,
  now on `gurobi_milp` too, and, in both suites,
  `time_limit_reads_back_unchanged`, `indicator_constraints_are_background` and
  the status afterwards. In place of the `names_every_side` flag, `validate()`
  checks on both paths that a complete answer has no reason and that a member is
  whole only on an entity with two finite sides; every native routine names the
  side of a column-less row (measured).
- **CI.** Clp, Cbc, GLPK and HiGHS run in CI, so the free function is
  CI-tested. CI always runs at least one of them, and since 2026-10-02
  `dumb_lp`, a user-defined model without row-bound setters, runs the free
  function on Clp too (N24, N43). HiGHS runs the native routine in the
  macOS job, on brew's 1.15.1. The Linux job, on 1.9.0, and the Windows job,
  on 1.13.0, exercise the throw below the 1.14.0 floor. CI thus spans the
  missing symbol, the 1.13 regime and the 1.14+ regime without the
  compatibility matrix, but not 1.12. Only Windows pins its version
  (`c-cpp.yml:307`). Linux takes Ubuntu 25.04's package and macOS brew's
  current bottle (`c-cpp.yml:46, 54, 281`), whose versions above are those of
  main's CI run 35922101900 of 2026-09-23. The macOS case keeps covering the
  1.14+ regime while brew's release stays in the validated range, which ends
  before 1.16. The commercial routines are tested locally.
- **Docs.** `docs/reference/concepts.md` gets `has_iis`, `lp_iis`, the free
  function's concept, `has_modifiable_constraint_bounds`, the enumeration
  concepts and `has_status_reset`. The rows of `has_iis` and of the free
  function's concept say that `irreducible` with zero members means the
  background alone is infeasible. The user guide gets a Solving page on
  reading an IIS, with the basis-like consumer loop, and a page under
  Algorithms on the deletion filter, next to column generation. The IIS page
  restates for IIS the warning of `docs/solvers/index.md` on the native
  handles (N23): background added natively is outside the guarantee, and the
  free function's guard and the entity enumeration do not see native changes.
  It states the SCIP gap (N25). Its paragraph on the time bounds of native
  calls states per backend that the budget is per call, which other model
  limits also stop the routine, and that a stop may return late or with no
  answer (N29 a). It states that the model's time limit bounds a native
  `copt_lp` call as it does on `copt_milp`, through fix 10 (N35 a). The pages
  obtain the IIS type through `model_iis_t<T>` or `auto`, never spell the
  snapshot's template arguments, and test membership with
  `is_a<iis_status::member>` (N37 b, d). Wave 4 wrote them on 2026-09-29.
  `docs/solving/infeasibility.md` runs both paths on one workshop LP and
  covers the choice of path in generic code, the tags and the consumer loop
  over the caller's families or the enumeration, outcomes and reasons,
  `iis_limits` and the native time bound, with the iteration limit that does
  not stop HiGHS's routine, `*_lp` against `*_milp`, the model and its status
  afterwards, the snapshot's handle ids (N8), a per-model table that links the
  limitation bullets, the SCIP gap (N25) among them, and the native-handle
  warning (N23). `docs/algorithms/deletion-filter.md` covers the engine on an
  oracle of the reader's own, monotonicity, limits and reasons, then the free
  function's candidates and trials, what it saves, restores and never touches,
  the trials the forwarded time limit does not bound, warm and cold trials
  per backend, and the same warning. Their code lives in
  `test/doc_snippets/`, compiled and tested in `mippp_test`, and
  `examples/infeasible_transportation` runs the deletion filter, the page
  covering both paths (N45 h).
  What concerns the commercial routines came with wave 5 (`b5014d1`,
  `12bcae7`, `b20e204`, `e507a95`, `818235d`): their tags, with plain
  `member` on the equality rows of Gurobi and CPLEX and on the two-sided
  rows and two-bounded columns of a `copt_milp` that COPT solves as a MIP,
  which other model limits stop each routine, whether a stop returns late or
  with no answer on each, and the `copt_lp` time limit of N35 (a); each
  backend got its rows in the per-model table of the infeasibility page and
  in the `has_iis` row, its paragraph on native time bounds, and its
  limitation bullet. The final review of that wave added the detached
  callbacks of `cplex_milp` and `xpress_milp`, the iteration-limit stop
  CPLEX keeps, the `IsMIP` condition on the `copt_milp` rules, the repair
  section's requirements and Gurobi's way to relax a row, the headers of the
  two filter concepts, and marked the memory limits of Gurobi and CPLEX as
  documented rather than measured.

## Deferred, and how they would come back

- **Dual rays.** A `has_dual_ray<T, M = T>` capability mirroring
  `get_dual_solution`: a handle-indexed mapping, never a vector in insertion
  order. A ray can later seed the free function through
  `if constexpr(has_dual_ray<M>)`: one verifying trial, then the engine's
  continuation. The Clp, SoPlex and MOSEK plumbing from the pull request is
  reusable once reshaped into mappings. The SoPlex part needs an unsubmitted
  8.1.0 patch and waits for upstream. The Clp ray must gate on the cached
  `_status`, which the status reset of N22 keeps honest after a filter run.
- **Elastic relaxation.** Six backends have a native routine: `CPXfeasopt`,
  `GRBfeasrelax`, `COPT_FeasRelax`, `XPRSrepairinfeas`,
  `Highs_feasibilityRelaxation` and `MSK_primalrepair`. The pull request's
  elasticity prefilter returns as a capability with those behind it, not as
  generic code first.
- **Forcing and preferences.** Gurobi forces membership per entity
  (`IISConstrForce`, `IISLBForce`, `IISUBForce`) and CPLEX takes group
  preferences in `CPXrefineconflictext`; Xpress fixes whole classes only
  (`IISOPS`) and COPT 8.0 has neither. Setters on the IIS object, shaped
  like the basis setters, are the natural home. On the deletion path,
  protected sides are a filter over the enumerated sides, behind an additive
  overload of `compute_iis_by_deletion`, beside the narrowing overload of
  N45 (c), which already selects the candidates from an earlier answer.
  Protected sides and candidate order are the first extension after the
  first version (N37 f).
- **Several IISs per model.** Xpress's `XPRSiisnext` and `XPRSiisall`, a
  backend-specific extra, not part of the capability.
- **SCIP 10.** Its IIS finder, once the sub-SCIP answer can be mapped back
  to handles.
- **Filter refinements.** Candidate ordering, and a caller for the dormant
  batching.
- **Integrality as a candidate.** The maintainer's intuition, given with N25,
  is that relaxing an integer or binary variable to continuous may explain a
  MIP better than relaxing its bounds. Xpress's default already lists
  integrality restrictions as removable `I` members. It is outside the first
  version, whose design is centred on linear programs. It comes as a
  per-variable table of its own in the snapshot, read through its own
  accessor, never as new tags: a variable can need both sides and its
  integrality at once, which one tag cannot say (N37 a). It needs a
  variable-type getter and setter, which `milp_model` lacks, and leaves the
  engine untouched, which sees indices only (N37 c).
- **Special constraints as members.** SOS and indicator constraints first need
  handles: no backend implements SOS, and `add_indicator_constraint` returns
  none. They then come as further per-kind tables in the snapshot (N37 a), on
  the native path first, where Gurobi (`IISSOS`, `IISGenConstr`), CPLEX
  (conflict groups) and Xpress (`IISOPS` classes) already report them. The
  deletion path cannot relax an SOS linearly and never removes a row, so it
  keeps them as background.
- **Cancelling a native call.** An additive overload
  `compute_iis(std::stop_token)`, wired to `GRBterminate`, `CPXsetterminate`,
  `XPRSinterrupt`, `COPT_Interrupt` and the HiGHS interrupt callback (N31 b).
  It leaves `model_iis_t` and `has_iis` unchanged. CPLEX documents status 38
  for a user abort of the refiner.
- **A duration for a native call.** An additive overload
  `compute_iis(std::chrono::duration<double>)` that saves the parameter the
  routine reads, writes the argument and restores it, the call without an
  argument keeping N29 (a). It was recommended for later if users ask, like
  N31 (b), and is not ruled: it is a possibility, not a decision (N29 b).
- **A user-defined model in CI.** Done on 2026-10-02, though not as N24
  described it: `dumb_lp` keeps its one-sided rows and has no row-bound
  setters, and runs `IisByDeletionTest` through the sense and the rhs of N41
  (N43).

## What the pull request contributed

- **Kept.** The deletion filter, as the public engine, with its batching
  ported dormant. The idea of a public deletion filter over a user-supplied
  oracle, which Q1 (b) brings forward. The MOSEK fixes that main still
  needs: a destructor that cannot throw, since `~mosek_base()` calls
  `check()`, the handle guard, solution-slot selection, and the getter half
  of the time-limit mapping, which moves to the N4 fix with the author's
  credit. Main already optimizes once per `solve()` since `d8bb08f` and runs
  `TimeLimitTest` on MOSEK. The SoPlex move fix. The Clp, SoPlex and MOSEK
  certificate plumbing, for dual rays later. The exact oracle with the
  published HiGHS vectors, and its list of test situations.
- **Kept in spirit.** The status classification, re-read under the zero
  objective as above: the pull request treated `infeasible_or_unbounded` and
  `primal_and_dual_infeasible` as inconclusive, and a failed solve carrying
  a solution as a feasibility proof.
- **Replaced.** The index-based `linear_system` input, the index-based
  results, the policy and options objects, the report formatter, the
  separate test binary and the seventeen-header layout.
- **Rejected.** The possible-member tag (N1 a), the optimizer-count and
  interior-point tests with their binding (N5 b), and the license hint text
  (N18 a).
- **Outside IIS.** The loader and license diagnostics (`diagnostic_text.hpp`
  and the help text in `solver_library.hpp`) were judged on their own on
  2026-09-27 (N18 a). They become a standalone loader fix, trimmed to the
  platform's search variable, one precedence sentence and the quoted
  `MIPPP_<KEY>_LIBRARY` value. License hint text is rejected, and MOSEK's
  license codes are fixed as a bug in their own commit.

## Rulings of 2026-09-27

The maintainer ruled on 2026-09-27 on questions 1 to 6 of the 2026-09-22
note, written Q1 to Q6, and on questions N0 to N18 of
[iis_pr_plan.md](iis_pr_plan.md). Each bullet names the chosen option, which
the sections above apply. The options behind each letter are in the
pre-ruling versions: this note's at `f5e833f` on main, and the plan's in
[iis_pr_plan_pre_rulings.md](iis_pr_plan_pre_rulings.md), the version
the maintainer ruled on. Each *Confirmed reading* spells out how a short
ruling applies, as the maintainer confirmed on 2026-09-27. N4's reading was
not confirmed: the maintainer left that choice to the assistant, and its
bullet records the decision.

- **Q1. Where the deletion filter lives.** (b) Ruling: "A public free
  function, which needs a public entity enumeration first. For example, the
  IIS engine for `deletion_filter` should be an independant algorithm of the
  library. The concept `has_iis` answers the question 'Does this model has
  native IIS support?', so both IIS computation methods are different
  paths." *Confirmed reading.* The filter has two public layers: an engine
  over a user oracle, and a free function over any model that meets its
  concept. `compute_iis()` is native-only and never runs the filter.
- **Q2. One call or two.** (a) `compute_iis()` returns the snapshot by value
  like `get_basis()`, rather than storing it for a `get_iis()`.
  *Confirmed reading.* The recommended names are adopted: `iis_status`,
  `iis_outcome`, `iis_reason`, `iis_limits`, `lp_iis<I, T>`,
  `model_iis_t<T>` and `has_iis<T>`.
- **Q3. Relaxing a row side.** (a) Row bounds through
  `has_modifiable_constraint_bounds<T, M = T>`, rather than modifiable sense
  and rhs on five more backends, so that the filter stays solver agnostic.
- **Q4. The status after a run.** (a) `status::unknown` after any run that
  called `solve()`, exceptions included, native routines leaving it
  untouched. *Confirmed reading.* A run that solved nothing leaves the data
  and the status as they were.
- **Q5. Parameters.** (c) A solve budget and a stop token plus one duration,
  as a defaulted `iis_limits {max_solves, time_limit, stop_token}`.
  *Confirmed reading.* It is the parameter aggregate of the free function and
  the limits of the engine. Native calls take none (N9).
- **Q6. Ranged rows.** (a) Supported, each finite side a candidate, solver
  agnostic through `if constexpr`. *Confirmed reading.* Ranged rows need no
  branch in the IIS code, and `if constexpr` in the free function tests only
  `qp_model`, `has_time_limit` and the capabilities that can carry background
  constraints. The maintainer added: "Yes, this would imply implementing
  get_constraints_bounds() for all solvers. For those which does not suppport
  nativelly ranged constraints, like Gurobi, the returned pair would contain
  rhs and the solver infinity, the branch is in the model's method, not on the
  IIS side." `has_readable_constraint_bounds` therefore comes to every model
  in WP6c, as [Library additions](#library-additions-the-free-function-needs)
  describes.
- **N0. Handling of pull request #3.** (a) The maintainer reshapes it in
  place, then squash-merges it, rather than the author reshaping it.
- **N1. A possible-member tag.** (a) None: retained, untested and native
  "possible" members get `member_*` under `not_proven_minimal`.
  *Confirmed reading.* `member_*` includes plain `member` where a native
  routine does not name the side. The maintainer added: "Yes, if needed, use
  the pattern variant + tag hierachy to keep details when they are available
  while offering a unfied generalisation." A backend may therefore keep
  details that the unified tags drop through refinement tags that derive from
  the unified ones, listed only in its own status variant, so that `is_a` on a
  unified tag still gives the generalization. It is a guideline for when it is
  needed, and no work is planned for it.
- **N2. A reason for an inconclusive trial.** (b)
  `iis_reason::inconclusive_trial`, set after any inconclusive trial that
  ran, or `time_limit` when the deadline has passed.
- **N3. Batching.** (b) Ported dormant with its tests, rather than a single
  pass only or a public knob. *Confirmed reading.* The batch size is a
  parameter of the engine's detail entry, defaulted to 1, outside `iis_limits`
  and the public engine. The free function never sets it. The maintainer
  confirmed (b) "for the moment", so batching stays dormant for the moment.
- **N4. "No time limit".** (A) Ruling: "`get_time_limit()` is never
  negative. When a backend is proved to return an aberrant value, either use
  the solver's infinity(), numeric_limits::infinity() or
  numeric_limits::max(). Stay transparent when possible." A backend proved
  to return an aberrant value thus maps it to the solver's own infinity(),
  to `std::numeric_limits<double>::infinity()` or to
  `std::numeric_limits<double>::max()`, and stays transparent when possible.
  MOSEK maps its -1 s, and every backend gets `TimeLimitTest` cases.
  *Decision.* Asked to confirm a reading, the maintainer wrote "N4 (A). I am
  not convinced that is the best option, your call." They were not convinced
  and left the call, so it can be revisited. Decision recorded by the
  assistant at the maintainer's request on 2026-09-27: keep (A). The audit of
  2026-09-27 found one aberrant no-limit value, MOSEK's negative one, -1 s
  on a fresh task, and MOSEK's getter reads a negative value as
  `std::numeric_limits<double>::infinity()`. That value round-trips, since
  MOSEK 11.0 accepts +inf (measured) and every setter accepts +inf under
  N32 (a). The other backends stay transparent: Gurobi 1e100, COPT 1e20, SCIP
  1e20, CPLEX 1e75 and SoPlex 1e100 after their clamps, and HiGHS, Cbc and
  Xpress +inf. The earlier reading, that the solver's infinity is the time
  parameter's own no-limit value, is dropped as moot. (A) is kept over (B)
  because the guarantee lives on the `has_time_limit` capability. Any generic
  code, the free function's or a user's, can then forward
  `std::min(remaining, get_time_limit())` without a MOSEK special case,
  whereas (B) would put that special case in every consumer.
- **N5. A MOSEK test calling the vendor API.** (b) The optimizer-count and
  interior-point tests are dropped with their binding, and the
  optimizer-count item of iis_todo.md step 1 is struck.
- **N6. Column-less models.** (b) A zero-solve precheck: the first finite
  row side with lower > 0 or upper < 0 is the sole member, irreducible, and
  none means feasible. *Confirmed reading.* The comparison with 0 is exact and
  documented, and "first" means first in enumeration order.
- **N7. Prechecks and the budget.** (a2) A crossed variable is irreducible at
  zero solves on a model type with no special-constraint or callback
  capability. *Confirmed reading.* (a2) refines (a), so `max_solves` counts
  `solve()` calls only and prechecks are free. *Amended by N36 on
  2026-09-28.* The zero-solve claim and its type-level capability list are
  dropped: a crossed pair, variable or row, is the known proof and two
  singleton trials decide it on every model type. (a) stands: `max_solves`
  counts `solve()` calls only, and the column-less precheck is free.
- **N8. Snapshots and recycled ids.** (a) The snapshot describes the model as
  it was, so a later handle may reuse a removed id.
- **N9. Native routines and `iis_limits`.** Ruling: "Reffers to Q1." Asked
  later that day to choose a wording, the maintainer wrote "(i), match the
  N29 (a) ruling." A native `compute_iis()` takes no `iis_limits`. The
  model's time limit bounds each call as a fresh budget, on HiGHS through a
  per-call copy into `iis_time_limit`. Other model limits may also stop it,
  and the docs list them per backend.
- **N10. An archetype for `has_iis`.** (a) None, like every capability, with
  `static_assert`s in the fixtures.
- **N11. SoPlex.** (a) It joins last, binding eight symbols of 7.1.3 and
  8.0.2 after checking 7.1.1 and 7.1.2, or raising the floor.
- **N12. HiGHS ranged rows.** (b) Once the new setters range a row, the sense
  getter throws like Clp and Cbc, and the rhs getter like Clp.
- **N13. Clp's secondary status.** (a) and (b). `Clp_secondaryStatus` is
  bound, secondary statuses 2 and 4 map to `optimal_infeasible_unscaled` in
  the public variant (a), and the IIS computation classifies both as
  inconclusive (b). *Confirmed reading.* With no Clp member under Q1 (b), the
  generic classifier delivers (b), since it treats that tag as inconclusive.
- **N14. SCIP binary columns.** As recommended: probe whether a relaxed bound
  on a `BINARY` column stays satisfiable, keep such bounds as candidates if it
  does, and otherwise retype the columns as integer inside the guard.
  *Confirmed reading.* That fallback is unavailable. Retyping inside the guard
  needs a type getter, which `milp_model` lacks, and SCIP converts [0, 1]
  integer columns to `BINARY` when it creates them. N25 has since kept
  `BINARY` and documents the gap, so the probe only characterizes it.
- **N15. Native side effects.** Ruling: "As recommended. For the moment,
  status should be considered unknown after IIS computation." The
  recommendation: the status stays untouched where a probe shows that the
  held solution and the status agree, and the Q4 rule applies elsewhere.
  Either way, the wrapper restores what it sets through the WP7 guard.
  *Confirmed reading.* Every native `compute_iis()` sets `unknown` before its
  first native call, superseding Q4's "native routines leaving it
  untouched" until the routine's probe passes. No new ruling is needed per
  routine. Under Q1 (b) the free function's guard saves model data and lives
  in its detail namespace, so each wrapper uses its own RAII helper instead,
  which departs from the recommendation. The wave 5 probes of 2026-09-29
  (see Native routines): CPLEX's, Xpress's and COPT's wrapped calls fail the
  probe, and Gurobi's passes on models without special constraints only, so
  every `compute_iis()` keeps the reset; leaving Gurobi's status untouched
  on such models is a follow-up like HiGHS's, closed by N46.
- **N16. IIS and LP basis support.** (b) Ruling: "Basis implementation will
  handle the factorization burden if needed." IIS comes first, its storage a
  `detail` template that basis support reuses.
- **N17. Release gating.** (a) No tag between the merge of pull request #3
  and WP8, and none is planned before WP8.
- **N18. Loader and license diagnostics.** (a) A standalone loader fix,
  trimmed to the platform's search variable, one precedence sentence and the
  quoted `MIPPP_<KEY>_LIBRARY` value. License hint text is rejected, and
  MOSEK's license codes are fixed as a bug.

Later on 2026-09-27 the maintainer accepted the recommendations of N19, N20,
N21, N22 and N26, whose entries in the plan keep their options and evidence.

- **[N19](iis_pr_plan.md#ruled-later-on-2026-09-27). Headers and names.** (a)
  Everything in `include/mippp/utility/`, namespace `mippp`, each new name
  prefixed with `deletion_` or `iis_` except the verb
  `compute_iis_by_deletion`. The engine is `deletion_filter`,
  `deletion_verdict`, `deletion_oracle` and `deletion_filter_result` in
  `utility/deletion_filter.hpp`. `iis_limits`, `iis_outcome` and `iis_reason`
  sit in the std-only `utility/iis_outcome.hpp`, which `model_concepts.hpp`
  and the engine include. The free function is `compute_iis_by_deletion`, with
  its concept `iis_by_deletion_model`, in `utility/iis_by_deletion.hpp`. The
  suites are `IisTest`, labelled "IIS, native", and `IisByDeletionTest`,
  labelled "IIS, deletion filter". The snapshot is
  `iis_snapshot<Variable, Constraint, VariableStatus, ConstraintStatus>`.
  Outcome and reason are `enum class` (e), and `deletion_filter_result`
  carries no call count (f).
- **[N20](iis_pr_plan.md#ruled-later-on-2026-09-27). Enumeration names.** (a)
  The members `variables()` and `constraints()`, the concepts
  `has_enumerable_variables` and `has_enumerable_constraints`, and the suite
  `EnumerableEntitiesTest`, labelled "Enumerate variables and constraints".
  SCIP's protected data members `variables` and `constraints` are renamed.
- **[N21](iis_pr_plan.md#ruled-later-on-2026-09-27). Enumeration contract.**
  (a) A snapshot of the call in increasing id order, random-access and sized,
  that holds no reference to the model: the lazy `entity_range` where nothing
  is removed, and a `std::vector` on `clp_lp`, `dumb_lp` and the three
  remapping bases, sorted only after a perforating removal. The concepts take
  `<T, M = T>` and require `std::ranges::sized_range`. The enumeration stays
  out of `lp_model` until 2.0.
- **[N22](iis_pr_plan.md#ruled-later-on-2026-09-27). Resetting the status.**
  (B1) A public `void reset_status() noexcept` on the 19 models and `dumb_lp`,
  with the concept `has_status_reset<T>`, which the free function's concept
  requires. The shared case `LpModelTest.reset_status_reports_unknown` pins
  it, and `concepts.md` gets a row. MOSEK's slot cache resets inside it, wired
  by whichever of WP2 and WP6b lands second.
- **[N26](iis_pr_plan.md#ruled-later-on-2026-09-27). The HiGHS native floor.**
  (a3) Native `compute_iis()` on HiGHS throws `solver_error` below 1.14.0,
  naming the loaded version, the library path and the floor. No query API is
  added. An empty `library_version()`, as on a devel build, requires the
  symbol and assumes the newest regime. Tests skip on the version, never on
  catching the exception. An empty answer is `feasible` when the model status
  is optimal or unbounded, and `undetermined` otherwise. An error with model
  status `kTimeLimit` is `undetermined` with `time_limit`, and a `kWarning`
  return is `undetermined`. N34 (a) later amended the mapping of a -1 return.

Later the same day the maintainer ruled on N23 to N25, N27, N28 and N30 to
N33, and asked two questions on N29, which then stayed open with the new
N34. The entries of the [plan](iis_pr_plan.md#ruled-later-on-2026-09-27)
keep the options and evidence of each.

- **[N23](iis_pr_plan.md#ruled-later-on-2026-09-27). Background added through
  `native_model()`.** Ruling: "Add to the documentation, if not already
  present, a warning that states that some features are invalidated if user
  modifies the model through the native handle." Recorded as (a): constraints
  added through `native_model()` are outside the guarantee. The background
  check that (a) shaped, which ignored `has_native_handles`, went away with
  N36 on 2026-09-28, and the warning stands. The general warning is written in
  `docs/solvers/index.md`, committed with `25b4530`, and the IIS page of WP17
  restates it for IIS.
- **[N24](iis_pr_plan.md#ruled-later-on-2026-09-27). A user-defined model in
  CI.** Ruling: "Not a priority, CI always have at least one open source
  solver (Clp, CBC or HiGHS)." No `dumb_lp` instantiation of
  `IisByDeletionTest` for now. It moves to Later. Its premise, `dumb_lp`
  with two-sided rows and modifiable row bounds, no longer holds: since
  2026-10-02 `dumb_lp` keeps its one-sided rows and has no row-bound
  setters, runs the filter through the sense and the rhs of N41, and
  `IisByDeletionTest` is instantiated on it (N43), in CI on Clp.
- **[N25](iis_pr_plan.md#ruled-later-on-2026-09-27). SCIP's `BINARY`
  columns.** (c) in effect. Ruling: "For binary and integer variables, my
  intuition is that relaxing them to continuous is more interesting than
  relaxing their bounds, but I am not used to mixed integer IIS. Either waay,
  the current IIS design being centered on linear programs, the gap should be
  documented instead of modifying the algorithm to conform to SCIP."
  `scip_milp` keeps `BINARY`, and neither the algorithm nor the backend
  changes. The N14 probe only characterizes the gap for the documentation.
  Integrality as a candidate is deferred.
- **[N27](iis_pr_plan.md#ruled-later-on-2026-09-27). The reason on an unproven
  native answer.** (a) No reason: `get_reason()` is empty, meaning the
  routine itself did not prove minimality, and `time_limit` is still set on a
  stop attributed to the time limit.
- **[N28](iis_pr_plan.md#ruled-later-on-2026-09-27). Native wrappers and the
  column-less precheck.** (a) No precheck by default. The shared column-less
  case pins each routine's answer, and a wrapper that fails it may call the
  shared detail arithmetic.
- **[N30](iis_pr_plan.md#ruled-later-on-2026-09-27). Row bounds on the
  commercial backends.** (b) Ruling: "When row bounds are trully supported
  natively (not like Gurobi that is only emulating them)." Row bounds come to
  COPT, CPLEX and Xpress as optional commits of WP16, only where the solver
  stores a row's two sides natively, to be confirmed per solver. Gurobi's
  ranges add a slack column, an emulation that does not qualify. Readable row
  bounds reach every model through WP6c (Q6 a), so N30 governs the modifiable
  half.
- **[N31](iis_pr_plan.md#ruled-later-on-2026-09-27). Cancelling a native
  call.** (b) Later, as an additive overload `compute_iis(std::stop_token)`,
  not in the first version.
- **[N32](iis_pr_plan.md#ruled-later-on-2026-09-27). Unlimited time limits.**
  (a) `set_time_limit` accepts +inf and `duration<double>::max()` on every
  backend: the SCIP setter clamps to 1e20 and the CPLEX setter to 1e75, the
  values their fresh models report.
- **[N33](iis_pr_plan.md#ruled-later-on-2026-09-27). Negative time limits.**
  (b) The Cbc setter throws `solver_error` on a negative value, and its getter
  stays transparent. NaN is left to each solver.

A third message that day ruled on N29 and N34, and on the wording of N9,
which the N9 bullet above records.

- **[N29](iis_pr_plan.md#ruled-later-on-2026-09-27). Bounding a native
  `compute_iis()` in time.** (a) Ruling: "Ok for (a)." It answers the bound
  only, so N29's second choice, a time limit on `copt_lp`, became N35, ruled
  (a) below. The model's time limit bounds each native call as a fresh budget.
  Gurobi, CPLEX, Xpress and `copt_milp` need no code, because their routines
  already read the parameter that `set_time_limit` writes. HiGHS copies
  `get_time_limit()` into `iis_time_limit` for each call, restored afterwards
  (WP15). The docs state per backend that the budget is per call, which other
  model limits also stop the routine, and that a stop may return late or with
  no answer. Option (b), an explicit duration as an additive overload, was
  recommended for later if users ask and is not ruled: it stays a Deferred
  possibility, not a decision. Leaving the call unbounded (c) and the solver's
  own rule (d) are rejected.
- **[N34](iis_pr_plan.md#ruled-later-on-2026-09-27). A HiGHS 1.14.x stop in
  the elasticity filter.** (a) N26's mapping is amended: a -1 return from
  `Highs_getIis` is `undetermined` with `time_limit` when the time measured
  around the call reached the limit copied into `iis_time_limit`, whatever
  the model status, and throws `solver_error` otherwise. The same
  elapsed-time test attributes `time_limit` to a "maybe" answer.

A fourth message that day ruled N35. It also confirmed each *Confirmed
reading* above, adding to Q6, N1 and N3, and left the choice on N4 to the
assistant.

- **[N35](iis_pr_plan.md#ruled-later-on-2026-09-27). A time limit on
  `copt_lp`.** (a) Ruling: "N35. You proved that putting set_time_limit in
  copt_base respects all contracts thus yes." `set_time_limit` and
  `get_time_limit` move from `copt_milp` into `copt_base`, and `TimeLimitTest`
  is instantiated for `COPT_lp`, as the plan's fix 10, now a firm fix before
  WP16. Under N29 (a), a native `copt_lp` IIS is then bounded by the model's
  time limit, like `copt_milp`.

What the rulings overturn:

- **Two paths.** Q1 (b) replaces "a capability, not a separate library" and
  the recommended protected helper per backend.
- **No native `iis_limits`.** N9 replaces the recommended forwarding of
  `iis_limits` into native wrappers, meant for parity.
- **Native status.** N15 suspends the rule that native calls keep the status.
- **Column-less models.** N6 (b) replaces the 2026-09-22 undetermined outcome.
- **Time limits.** N4 (A) replaces the recommended (B), where MOSEK stayed
  transparent and forwarding treated a negative saved limit as no cap.
- **Batching.** N3 (b) replaces the recommended single pass.
- **Clp.** N13 brings the public mapping (a) forward, to a fix before WP8.
- **Parameters.** Q5 (c) adds a duration to the recommended budget and token.
- **SCIP binaries.** N25 replaces N14's recommended fallback, retyping the
  columns inside the guard, by a documented gap.
- **A user model in CI.** N24 defers the recommended `dumb_lp` instantiation
  of `IisByDeletionTest` to Later; it came on 2026-10-02 through N41's
  fallback, without the two-sided rows the recommendation assumed (N43).
- **Gurobi row bounds.** N30 (b), with its condition, turns the recommended
  "Gurobi not yet" into never, since its ranges are emulated.
- **Native column-less answers.** N28 (a) drops the "shared by native
  wrappers" of the recommendation that came with N6.
- **HiGHS stop mapping.** N34 (a) amends N26: a -1 return is judged by the
  time measured around the call, not by the model status `kTimeLimit`.

## Rulings of 2026-09-28

A simplification review before wave 2, asked by the maintainer once wave 1
was on main and green, weighed what is done and the target design against the
basic needs of operations-research users and the later extension to special
MILP variables and constraints. The maintainer agreed with every point. One
planned piece is dropped, and the rest is confirmed with two rules for how
extensions attach.

- **N36. Crossed variable bounds.** Amends N7 (a2). No zero-solve claim: a
  crossed pair, of a variable or of a row, is the known proof, the free
  function skips the initial trial, and the engine continues from the pair
  with two singleton trials, on every model type. The type-level list of
  background capabilities, `detail::may_carry_background<M>` in the plan, is
  dropped with the claim: a special-constraint capability added later would
  have had to be added to that list by hand, and a missed entry would have
  made an `irreducible` claim silently unsound. The cost is two solves of a
  fully relaxed model, and no backend receives crossed bounds in a trial,
  which no shared test exercises today. The column-less arithmetic of N6 (b)
  stays the only precheck that answers without a solve. Under a budget that
  stops before the two trials, the proven pair is reported
  `not_proven_minimal` with the stop's reason, never `irreducible` at zero
  solves.
- **N37. Extension rules.** (a) New candidate kinds, integrality first, then
  SOS and indicator constraints once they have handles, come as new per-kind
  tables in the snapshot with their own accessor, never as new tags: the five
  tags stay, since a variable can need both sides and its integrality at once.
  (b) The snapshot's template arguments are not a commitment: docs and tests
  obtain the type through `model_iis_t<T>` or `auto` and never spell
  `iis_snapshot<...>`, and `lp_iis<I, T>` checks members, not the shape, so the
  shape can move to one entry per kind without a break. (c) The engine stays
  kind-agnostic, indices and an oracle: a new kind is a new item kind in the
  free function. (d) The tag hierarchy stays over an enum, for consistency
  with `status` and `basis_status` and for the refinement door of N1; the docs
  always test membership with `is_a<iis_status::member>`. (e) Batching stays
  dormant (N3 b). (f) Protected sides and candidate order are the first
  extension after the first version: native on Gurobi and CPLEX, a filter over
  the enumerated sides behind an additive overload on the deletion path; not
  in the first version. (g) The rest of the plan stands: the guard and the
  saved objective, which make MIP trials stop at their first incumbent; the
  time forwarding, which makes a time limit hard when one trial is a MIP; the
  outcome and reason split; the status reset; the member counts.

What these rulings change: N36 replaces N7 (a2)'s zero-solve claim by the
crossed-pair continuation, which was already the rule for crossed rows, and
retires the background check that N23 (a) shaped. Nothing published on main
or in pull request #3 changes.

## Rulings of 2026-09-29

The maintainer ruled on the three questions wave 3 left, N38 to N40, on
2026-09-29.

- **N38. Cbc 2.10 on integer-infeasible rows.** (b), the wrong answer stays
  documented, within a wider ruling on Cbc. Cbc "is in a weird spot overall":
  its released C API flushes the matrix "on every row entry", which makes
  model creation particularly slow "with no possible workaround". Its support
  in MIP++ "should be considered as experimental only", and its limitations
  and bugs "should be documented but not addressed". So (a), an integrality
  check on the points `cbc_milp` returns, is rejected, and no other workaround
  is added. The docs mark Cbc experimental wherever they list the backends to
  users: the backend table of `docs/solvers/index.md`, the support tables of
  the README and the home page, and the per-model table of the infeasibility
  page. One note of `docs/solvers/index.md` says why and gathers the Cbc
  limitations already documented, and the existing workarounds stay: native
  parameters that do not reach MIP solves, which run on a private copy below
  3.0, the
  devel build's rows without terms or with all-zero coefficients, the
  integrality proofs over unbounded integer columns, and the time limit that
  did not bound the root LP. `integers_summing_to_one_half` keeps its skip on
  every Cbc, a documented wrong answer on 2.10 and a search without end on the
  devel build.
- **N39. A skip keyed on a solve-time error.** Neither (a), keeping the skip
  keyed on the test name and the error, nor (b), expecting the throw in that
  case: the maintainer agreed with keying the SCIP skip on the case's data
  rather than on the error text. `1b91b7d`, cherry-picked as `5047a77`, does
  it. A fixture may provide `static std::optional<std::string>
  iis_case_skip_reason(const iis_case &)`, which the shared cases call before
  building a case, and SCIP's skips any case with an integer column whose
  bounds each round inward to 0 or 1, which SCIP types `BINARY` and then
  rejects once a trial relaxes a bound. A crossed domain such as [2, 0.5]
  rounds to [2, 0], stays `INTEGER` and runs. A different failure carrying
  the same text is no longer hidden, and the two SCIP tests that pin the
  rejection stay, so a change in SCIP still fails loudly.
- **N40. A side at the opposite infinity.** A lower side set to `+infinity()`,
  or an upper side set to `-infinity()`, which HiGHS and MOSEK reject in some
  forms while the other backends accept it, is "an obviously wrong use of the
  API". The maintainer's habit is "to document that such obviously wrong
  usages result in undefined behavior instead of adding guards that bloat both
  the source code and the binary output", with guards left to the
  assistant's judgment where one is important, and preferring documented
  undefined behavior to defensive guards against misuse is the general
  policy. No guard is added here. GLPK's clamp of infinite sides (`6774049`)
  stays under that latitude: it writes the valid infinite sides too, and
  without it an opposite infinity fails an assertion in the presolver of
  `glp_intopt`, which aborts the process.
  `docs/reference/concepts.md` calls such a side undefined behavior in the
  rows of `has_modifiable_variable_bounds` and
  `has_modifiable_constraint_bounds`, and `docs/algorithms/deletion-filter.md`
  no longer presents it as a candidate. The enumeration still tests each side
  against its own infinity only, and its comment names only the trap of a NaN
  side, never a candidate, not the opposite infinity as supported input.

What these rulings change: no library code, beyond the reworded comment of the
enumeration. N38 turns the pin that iis_todo.md 5.1 awaited into a documented
wrong answer on an experimental backend (`e2d443d`). N39 replaces the skip
keyed on a test name and an error message described under "A documented gap on
SCIP" (`5047a77`). N40 amends "One side at a time": a side at the wrong
infinity is still a candidate in the code, but no longer an input the docs
present (`1c0e50e`).

## Rulings of 2026-10-02

- **N41. The deletion filter on Gurobi.** The maintainer proposed it on
  2026-10-02: Gurobi "can model has_readable_constraint_bounds without lying
  by returning a pair (-inf, rhs) or (rhs, +inf). If constexpr can then be
  used to replace set_constraint_bounds by set_constraint_rhs", and, after the
  assessment, "Ok, implement it." `iis_by_deletion_model` now accepts, in
  place of `has_modifiable_constraint_bounds`, `has_modifiable_constraint_sense`
  and `has_modifiable_constraint_rhs` on a model without ranged rows
  (`detail::iis_rows_as_sense_and_rhs`). Each row of such a model has one side
  or the two equal sides of an `==` row, and so does every state it passes
  through while the filter relaxes and restores its sides, so one sense and
  one rhs express each state: both sides of an `==` row b give (`==`, b), a
  lower side l alone (`>=`, l), an upper side u alone (`<=`, u), and no side
  (`<=`, `infinity()`). The guard writes such a row whole, from the wanted
  state of both its candidates; a model with row-bound setters keeps the bound
  path unchanged. Q3 (a) stands, row bounds being the main interface and this
  a fallback chosen on capabilities, never on a backend type, and N30 (b)
  stands too: Gurobi still has no modifiable row bounds. Measured on
  2026-10-02 on Gurobi 11.0.3, 12.0.1 and 13.0.2 through the model's own
  setters: an rhs of ±1e100, Gurobi's `infinity()`, frees a side, a sense
  switch drops one side of an `==` row, the readable bounds follow exactly,
  and writing the sense and the rhs back restores the model exactly. Relaxing
  a binary variable's bound does not widen its domain, so the filter, like
  the native routine, never names the bounds of a binary variable, and no
  solve throws, unlike SCIP's (N25). `IisByDeletionTest` runs 22 cases on
  `gurobi_lp` and 25 on `gurobi_milp` on the three releases, the skips being
  the integer cases on the LP class and the cases that need ranged rows, and
  `both_paths_find_valid_iis` now runs on Gurobi. Three stub cases in
  `test/iis_by_deletion.cpp` pin the fallback on a model that holds its rows
  as a sense and an rhs. Gurobi 10 is not installed, so the floor rests on
  11.0.3.
- **N42. Callbacks under the filter, documented.** A review of the same day
  found that the filter's trials call `solve()` with a registered
  candidate-solution callback attached, so its lazy constraints and
  rejections act as background of every trial, while the native
  `compute_iis()` of `cplex_milp`, `xpress_milp` and `copt_milp` detaches it
  for the call and Gurobi's routine never fires it (measured on 11.0.3,
  12.0.1 and 13.0.2). Ruling: "documentent the callback effect". The filter is
  the path that honours lazy constraints. Measured with a callback that
  rejects every candidate, on integers x and y in [0, 5], under a 10 s
  `time_limit`: on the feasible x + y <= 8 every native routine answers
  `feasible`, while the filter answers `irreducible` with the lower bounds of
  x and y on Gurobi (36 callback calls), `irreducible` without a member on
  Xpress, and runs to its deadline on CPLEX and COPT (293,313 and about 1.2
  million callback calls), since a trial that relaxes an integer bound
  searches an unbounded domain whose every point the callback rejects. On
  the infeasible x + y >= 11 the native routines answer {x upper, y upper,
  the row}, the filter {the row} on Gurobi and no member on Xpress. A
  callback that only counts gives `feasible` on both paths. On a
  `cplex_milp` whose columns are all continuous, `solve()` refuses to run
  while a callback is registered (`cplex_milp.hpp:381-385`), so the filter
  throws there and `compute_iis()` answers. No code changes: the algorithm
  page has a Callbacks section, and the infeasibility page, the concepts and
  the limitations point to it. The same pass documented, without a ruling,
  two more facts measured on the paths. MIP starts: on `cplex_milp` a start
  (3, 2) of the user's becomes {(3, 2), (0, 0)} after the filter on a
  feasible model, a trial's incumbent added, {(0, 0)} after `compute_iis()`,
  and {(0, 11)}, a trial point, after the filter on an infeasible one, while
  Gurobi's `Start` attribute survives both paths. The bounds of a binary z
  under the row z >= 2, on 2026-10-02: neither path names them on
  `gurobi_milp` (11.0.3, 12.0.1, 13.0.2) or `cplex_milp` (22.1.1, 22.1.2);
  on `copt_milp` (8.0.5) the filter does not and the routine names z as a
  plain member; `xpress_milp` (45.01, 47.01) names the upper bound on both
  paths, and the filter does on `highs_milp` (1.10, 1.15.1), `mosek_milp`
  (11.0), `cbc_milp` (2.10.11 and a `devel` build) and `glpk_milp` (5.0);
  `scip_milp` throws (N25). An integer z in [0, 1] has its upper bound named
  everywhere but by COPT's routine, which names z plainly, and on SCIP,
  which types it binary. The infeasibility page states it once, under LP or
  MILP.
- **N43. No public row writer.** The review asked whether a model without
  row-bound setters should get `set_constraint_lower_bound` and
  `set_constraint_upper_bound` written through its sense and rhs, so that the
  docs' repair code compiled on Gurobi. Ruling: "set_constraint_bounds should
  not be implemented on models that does not support it, use
  get_cosntraint_sense and set_constraint_rhs instead." No public function
  writes row bounds on such a model. Code that relaxes one of its rows reads
  the sense with `get_constraint_sense` and writes `set_constraint_rhs`,
  writing `set_constraint_sense` only when an `==` row loses one side or a row
  changes side. The filter's writer now does so: it reads the current sense,
  computes the wanted state, writes the sense only when it differs, then the
  rhs, so a one-sided row is relaxed and restored through its rhs alone, and a
  row left with no side keeps its sense under an infinite rhs, an `==` row
  becoming (`<=`, `infinity()`), which amends the free state N41 gave.
  `detail::iis_rows_as_sense_and_rhs` now needs a readable sense and no
  row-bound setters, since `has_ranged_constraints` only says that a ranged
  row can be added and the concept held on HiGHS and CPLEX, whose setters
  range a row; static assertions over the real models pin it. The
  infeasibility page's `relax_row`, on which its repair loop rests, its
  `rerun_on_members`, which saves and restores such a row as a sense and an
  rhs, and the transportation example take the same path where the setters are
  missing, with no general writer of two sides, as N43 rules, and run on
  `gurobi_lp`, `gurobi_milp` and `dumb_lp`. `dumb_lp` runs `IisByDeletionTest`
  (20 passed, 10 skipped: three need a `milp_model`, five ranged rows, two a
  time limit), and the page's snippet tests run on it, so CI exercises the
  fallback on Clp. The code review of 2026-10-02 found that on this path an
  `==` row whose rhs lies at the backend's infinity, a side at the opposite
  infinity, comes back from the run as a `>=` or `<=` row with that
  infinite rhs, still infeasible; the finding was declined, since N40 makes
  such a side documented undefined behavior.

## Rulings of 2026-10-05

The maintainer asked on 2026-10-05 whether the IIS feature follows the
patterns of the library's other features, and whether simplifications would
give a simpler API. The answer, with the options weighed, the probes and the
measured prototypes, is [iis_api_review.md](iis_api_review.md). The maintainer
ruled on its completion report the same day: "implement the iis_outcome tag
hierarchy". Most of its other recommendations were ruled later that day, as
N45 below; the two left are listed under
[Pending decisions](iis_api_review.md#pending-decisions).

- **N44. How an IIS run ends.** The pair of `enum class iis_outcome` and
  `std::optional<iis_reason>` gives way to one `std::variant` per path over a
  tag hierarchy of its own, in namespace `iis_outcome`, as the solve status
  reports how a solve ended. `any` carries `conflict_available`; `completed`
  holds `irreducible` and `feasible`; `incomplete` holds `inconclusive_trial`
  and `stopped`, and `stopped` holds `interrupted` and `limit_reached`, over
  `time_limit`, `solve_limit`, `iteration_limit`, `node_limit` and
  `memory_limit`. `iis_outcome::conflict_available(o)` reads the flag, as
  `status::solution_available` reads a status, and replaces the test
  `irreducible || not_proven_minimal`. `get_reason()`, `iis_reason` and
  `deletion_filter_result::reason` are removed. The concept `lp_iis_outcome`
  requires `incomplete`, `irreducible` and `feasible` in every list, and
  `lp_iis` requires `get_outcome()` to satisfy it. `iis_snapshot` takes the
  outcome type as a fifth template argument whose first alternative must be
  `incomplete`, so that a value-initialized outcome claims nothing, as
  `absent` does for the member statuses. The tags are types of their own
  rather than `status::` tags: a reused tag satisfies
  `variant_of<status::any>`, so `classify_deletion_trial` would read an
  `irreducible` outcome as a feasible trial, and `status::solution_available`
  a proven conflict as a primal point (measured with a probe against the real
  headers). No release carries IIS yet, v1.0.0 predating it, so no published
  API breaks.
  - (a) Overturned: N19 (e), "outcome and reason are `enum class`", for both
    enums. Its premise, closed flat sets without payload
    (iis_pr_plan.md:105-110), failed three ways: N2 had already reopened the
    set, the causes refine one another as `time_limit` refines
    `limit_reached`, and the flag is a payload. The "outcome and reason
    split" item of N37 (g) is overturned with it; the rest of N37 (g) stands.
  - (b) Amended in spelling only: N27 (a). Its empty reason, a native answer
    whose routine proved no minimality, is the `incomplete` tag, which is not
    a limit, so its point that a missing proof does not imply a limit now
    holds in the type. N2 (b) is kept as `inconclusive_trial`, under
    `incomplete` rather than `stopped`, since the filter's pass still runs to
    its end. `cancelled` becomes `interrupted`, after `status::interrupted`
    and N31 (b).
  - (c) Lists per base. Each native base declares `iis_outcome_type` beside
    its snapshot type, `incomplete` first, shared by its `*_lp` and `*_milp`
    classes as the member-status variants of Gurobi and CPLEX already are,
    with one comment line where a tag is reachable on one class only. HiGHS
    lists `incomplete`, `irreducible`, `feasible` and `time_limit`, and no
    other stop tag, since a warning cannot be told from a failed post-check.
    Gurobi adds `stopped`, `interrupted`, `limit_reached`,
    `iteration_limit` and `memory_limit`; CPLEX adds `interrupted`,
    `limit_reached`, `iteration_limit`, `node_limit` and `memory_limit`;
    Xpress adds `stopped`; COPT adds `interrupted` and `node_limit`. The
    filter's `deletion_filter_outcome` lists `incomplete`, only as the value
    of a result no run wrote, `irreducible`, `feasible`,
    `inconclusive_trial`, `interrupted`, `time_limit` and `solve_limit`. A
    cause a routine reports without a listed tag folds into its nearest
    listed ancestor explicitly at the producer, never through the converting
    constructor of `std::variant`, as the backends' solve statuses fold a
    native outcome without a tag of its own.
  - (d) Causes the wrappers dropped now have tags. CPLEX's conflict statuses
    33 to 38 map to `time_limit`, `iteration_limit`, `node_limit`,
    `limit_reached` (objective), `memory_limit` and `interrupted`, 39
    (deterministic time) to `limit_reached`, and 32, the contradiction that
    follows an iteration-limit stop on the unchanged problem, to
    `incomplete`, since nothing outside that call cut it short. Status 31
    with a possible flag is `incomplete` with a conflict. Gurobi's `Status` is
    read only after a stop that left no subsystem, where the IIS attributes
    are unavailable: 9, 7, 17 and 11 give `time_limit`, `iteration_limit`,
    `memory_limit` and `interrupted`, 15 and 16 `limit_reached`, and any other
    code, as the 1 a stopped MIP reads, `time_limit` when the clock reached
    the budget and `stopped` otherwise. It is never read on an answered path,
    where it names no stop: a complete answer keeps the last solve's code and
    a partial one reads 3 (measured on 12.0.1). Xpress's `stopped` bit,
    computed and discarded before, gives `stopped` unless the clock, with its
    20 ms slack, blames the time limit. COPT's pre-solve `NODELIMIT` and
    `INTERRUPTED`, and the `INTERRUPTED` of the confirming `COPT_SolveLp`, get
    their tags.
    `set_iteration_limit` on `cplex_lp` and `gurobi_lp` and `set_node_limit`
    on `cplex_milp` thus stop `compute_iis()` with their own tag, where the
    tests had pinned the loss (cplex.cpp:328 and 370, gurobi.cpp:145 at
    `b9d2834`).
  - (e) Decided here, where the prototype differed: on HiGHS, an answer
    without a member whose model status is neither optimal nor unbounded
    goes through the clock too, `time_limit` or `incomplete` without a
    conflict, since the code at `b9d2834` dropped the stop there
    (highs_base.hpp:893-900).
  - (f) Tests read an outcome through `test/iis_outcome_assert.hpp`:
    `outcome_is<Tag>(o)` and `outcome_is<Tag>(o, conflict)` match the exact
    tag, a tag the path does not list fails to compile, and the failure
    message names the actual outcome; a `PrintTo` per tag serves GoogleTest's
    own printing of an outcome variant. The shared fixture's
    runtime check of invalid (outcome, reason) pairs goes, since the type
    cannot hold them.
  - (g) Decided in the review of the implementation: on 45.01, a feasible
    `xpress_milp` with one integer column in [0, 1] and no row makes
    `XPRSiisfirst` return `p_status` 3 with `STOPSTATUS` at
    `XPRS_STOP_MIPGAP`, which the new mapping read as `stopped`, a stop that
    nothing caused (47.01 answers `feasible`). `_compute_iis` now answers a
    model without rows, sets, general or PWL constraints, whose columns are
    continuous, integer or binary, from its columns, as COPT does: a column
    whose bounds admit no value is the IIS, otherwise the model is feasible.
    The check runs after the call, which restores a problem left presolved
    through the native handle, so the counts and bounds it reads are the
    original ones. Both releases now answer `feasible` there (probe,
    2026-10-05).

What this ruling changes: `utility/iis_outcome.hpp`, `model_concepts.hpp`,
the snapshot, the engine, the free function and the five native bases; the
shared suites, the per-backend pins and the doc snippets; the infeasibility
page, which now shows the tree, the meaning of each tag, the lists per path
and the tag each limit gets per model; the deletion-filter page, the concepts
reference, the solver limitations and the transportation example. The
per-backend pins whose meaning changed were confirmed against the real solvers
on 2026-10-05 (HiGHS 1.15.1 for the HiGHS pins); Gurobi's 11, 15 and 17 and
CPLEX's 36 to 38 rest on the solvers' documentation, no run reaching them,
and the mapping functions of both are pinned by tests that need no licence.

## Rulings of 2026-10-05, continued

Once N44 was implemented, the maintainer asked, on 2026-10-05: "implement the
factorizations you spotted then update the documentation with the fixes you
found and update the IIS examples to make then as short and understandable as
possible, minimizing boilerplate." The work landed on branch
`feat/iis-factorizations`, on top of `feat/iis-outcome-tags`, as 54 commits
from `9dcbbd6` to `4058e69`, none pushed.

- **N45. The review's factorizations, fixes and doc corrections.** The
  instruction rules the review's [Factorizations that overturn no
  ruling](iis_api_review.md#factorizations-that-overturn-no-ruling), its
  [Backend data and correctness](iis_api_review.md#backend-data-and-correctness)
  and its [Documentation errors](iis_api_review.md#documentation-errors). The
  review's [Rejected](iis_api_review.md#rejected) list and its
  [Rulings the review would
  keep](iis_api_review.md#rulings-the-review-would-keep) stand as it
  recommended: batching stays dormant (N3 b, N37 e), the guard keeps its `Clock`
  parameter, COPT keeps its `GetSOSIIS` and `GetIndicatorIIS` bindings, and
  the snapshot gets no member lists, no `possible_*` tags and no refresh of
  the status after a native call, the filter no role callback and no
  order-preserving pass.
  - (a) `detail::restore_guard` (`9dcbbd6`), in `detail/restore_guard.hpp`
    with a solver-free test: a `[[nodiscard]]` constructor taking the
    write-back, a `restore()` that disarms and writes back once, its error
    propagating, and a destructor that writes back only while still armed,
    after an exception, swallowing its error. It replaces the seven guard
    classes: HiGHS's option guard (`1a67c2d`), Gurobi's `iis_force_guard`
    over an `iis_forced` record that saves an array only once its force
    succeeded, since the write-back discards the held solution (`db0ee87`),
    the callback reattachment of `cplex_milp` (`4de6a3e`), `xpress_milp`
    (`db3171d`) and `copt_milp` (`9ce568b`), each constructed after a
    successful detach, Xpress's `IISOPS` guard (`1d2904c`), and COPT's
    time-limit guard, now the helper `_iis_within_time_limit` at the MIP's
    `ComputeIIS` and the LP's confirming `SolveLp` (`4a24571`). A guard is
    armed before the writes when there are several and after the single
    write otherwise, and a write-back of several items attempts each before
    raising the first error. Two fixes outside IIS come with it:
    `refine_lp_status` on `gurobi_lp` restores `DualReductions` when the
    re-solve throws (`30fc38a`), and on `cplex_lp` writes both parameters
    back on every exit, where three paths left one changed (`73bca7b`; tests
    `032f497`, `4e5554f`, the second making `CPXprimopt` throw on a problem
    turned MIP through the native handle, error 1017 on 22.1.2).
  - (b) `detail::iis_answer` (`bd3b8db`), in the detail section of
    `iis_snapshot.hpp`: the two tables, `flag_variable` and
    `flag_constraint`, which leave an entity flagged on neither side absent,
    and `finish(outcome)`, which hands the tables over. The review's three
    verbs are gone, since N44's tags make every `finish(outcome)` valid. The
    five bases build every native exit through it: HiGHS (`3170051`),
    Gurobi (`65ac219`), CPLEX (`c17e723`, which also collects a column's two
    sides as flags instead of reading the table back), Xpress (`7e79741`)
    and COPT (`92d29fc`). `iis_sides` and `iis_status::sides_of`
    (`44aae0c`), in `model_concepts.hpp`, read a status as {lower, upper,
    whole}: `member_both` both sides, a sided tag its side, plain `member`
    both sides with `whole`, which marks them as one unit, and `absent`
    nothing, by derivation, so refinement tags read as the tag they refine.
    The oracle's `membership_of` stays independent, and a test checks that
    the two agree on the five tags. The free function's fold merges a side
    into its entity through it, in any order (`ac5c3b4`).
  - (c) The narrowing overload, the mechanism N37 (f) names on the deletion
    path, shipped now under the instruction (`2b8ce68`):
    `compute_iis_by_deletion(model, within, limits = {})`, for any `within`
    with `lp_iis<Iis, M>`, takes as candidates the finite sides `within`
    names, a plain member naming every finite side of its entity, and keeps
    every other side relaxed in every trial through the existing guard. One
    detail runner serves both overloads. The N36 crossed-pair start looks
    only at the selected sides, so a crossed pair named in part or not at
    all stays relaxed, and the N6 column-less answer reads only the selected
    sides. The IIS found is an IIS of the model, since each trial reads its
    active sides and the background alone; the first trial checks the named
    sides, so `within` may come from either path or be stale, and is read by
    handle id on this model. Named sides that hold together answer
    `incomplete` without a conflict, never `feasible`, which says nothing of
    the model, as the comments of `deletion_filter_outcome` and of the
    `incomplete` tag say (`77beda6`). A braced or `iis_limits` argument still
    picks the overload without an answer, pinned by static assertions. It
    narrows a stopped run and refines the plain-member `==` rows of Gurobi
    and CPLEX into their side, in one solve per named side plus one: on the
    workshop the
    native answers of `gurobi_lp` and `cplex_lp`, and of their `*_milp`
    classes, narrow to the five sides, the orders on their lower sides
    (probe, 2026-10-05). Stub cases pin the crossed pairs inside and outside
    the answer (`607bb73` adds one named whole), the incomplete answer, a
    whole row refined on the sense/rhs stub and a column-less run; the
    shared case narrows a `max_solves` stop for 1 to 9 solves to the full
    run's IIS on every backend. The shared native case runs only where a
    routine can name an entity whole (Gurobi, CPLEX, `xpress_milp`,
    `copt_milp`): on the sided routines narrowing repeats the stopped-run
    case, and the suite's HiGHS fixture has no 1.14 floor skip, so the
    review declined widening it. Protected sides, candidate order and
    native forcing stay deferred under N37 (f). A narrowing run on a MILP
    relaxes the bounds of every integer column the answer leaves out from
    its first trial: on `scip_milp` that throws at the first trial when a
    binary column's bound is left out, on an infeasible and on a feasible
    model, the bounds restored (probe on the local SCIP, documented under
    N25's limitation); on `cbc_milp` and `glpk_milp` such a trial branches
    without end on `2 x0 + 2 x1 == 1` until the forwarded time limit stops
    it (measured with GLPK 5.0, Cbc 2.10.11 and a Cbc `devel` build after the
    merge, pinned by `narrowing_returns_on_an_endless_integer_row` and
    documented on the infeasibility page).
  - (d) The Xpress clear goes (`89524d1`): `XPRSiisfirst` clears the
    previous IIS itself, so `XPRSiisclear`, after a decoded subsystem and on
    the column-answered path of N44 (g), only made `XPRSgetiisdata`,
    `XPRSiiswrite`, `XPRSiisisolations` and a continuing `XPRSiisnext`
    unreachable through `native_model()`. Both calls and the binding go,
    and a test reads IIS 1 through `native_api().getiisdata` after the
    call: its counts match the members, its entries name the row's lower
    side and the column's upper bound, and both Farkas multipliers are
    nonzero, on 45.01 and 47.01. `IISOPS` is restored before the call
    returns, so a native `XPRSiisnext` searches under the caller's
    `IISOPS`. The shared scan
    `detail::iis_first_self_infeasible_column<C, I, B>` and
    `iis_column_kind_of<C, I, B>` (`fc93846`) serve Xpress (`76481af`) and
    COPT (`3036592`).
  - (e) Smaller changes. `handle_status_table` stores a
    `std::vector<Status>` (`57052c6`), dropping `is_tag_variant_v`,
    `_make_status` and `_make_derived_mask` at one byte more per id;
    `count_a` requires a tag some alternative derives from, and `get()` is
    `noexcept` when copying is (`b513b28`). `iis_sided_status` joins its two
    siblings in `detail`, and `iis_limits` moves unchanged to
    `deletion_filter.hpp`, amending N19 (a)'s placement, so that
    `model_concepts.hpp` no longer includes `<stop_token>` (`ec90b49`).
    `iis_column_less_precheck` requires readable row bounds only, and is
    used by COPT alone, while HiGHS keeps `iis_side_violated_by_zero`. The
    free function loses `iis_deletion_guard::solved()` and
    `iis_deletion_time_limit_slot`, keeps the saved limit as an optional
    `duration<double>`, and answers a column-less model through the
    enumeration and the fold, a NaN limit still throwing before any read
    (`ac5c3b4`, `ccadb7e`); new tests pin the lower side winning when 0
    violates both sides of a column-less row, and a guard on a fake clock
    past its deadline answering `inconclusive_trial` with no solve and the
    data restored, which is the `Clock` parameter's reason to exist. N37
    (b)'s ways to obtain the type gain `iis_by_deletion_t<M>`.
  - (f) HiGHS's decode names sides for Lower, Upper and Boxed only
    (`4e0c296`): Dropped (−1), Null (0) and any later code make no member,
    where they read as `member_both`. Static assertions pin each code, since
    no answer a test can produce lists the others.
  - (g) COPT minimality against special constraints (`953b821`): COPT's
    routine counts native SOS and indicator constraints among its
    candidates, so `IsMinIIS` proves minimality against the ones it names.
    `copt_milp` reports `irreducible` only when the answer names every one
    of them or no linear member, and otherwise takes the clock-attributed
    `incomplete` with a conflict. Measured on 8.0.5 (`03d8a4a`): an
    indicator left out of the IIS that is the only constraint on its column
    makes the routine flag both bounds of that column without counting them
    in `IISCols`, so the count check answers `incomplete` without a conflict
    before the downgrade; the test's unrelated indicator acts on x instead,
    `IISIndicators` 1, `IsMinIIS` 1, and r flagged on its upper side, and
    the COPT note documents the quirk.
  - (h) The pages and the example (`d588145`, `c59b6bd`, `9f435b7`,
    `8327315`, `18c460e`, `b02269d`, `e9e3945`, `dfe5423`, then the review
    of the round: `3167b5a`, `56c4870`, `00c01ef`, `cbc116c`, `685cd83`,
    `5a77e00`, `68f0ab8`, `421980e`, `7cb96e6`, `4058e69`). The code reads
    sides through `sides_of`, the side-name visitor, `named_sides` and the
    55-line `rerun_on_members` go for a three-line narrowing section, and
    the repair loop runs inline. N43's page code changes with it:
    `relax_members` writes the row-bound setters, and a sentence gives the
    sense-and-rhs recipe for `gurobi_lp` and `gurobi_milp`, freeing a `<=`
    row with `infinity()` and a `>=` row with `-infinity()`, instead of a
    Gurobi branch; the `dumb_lp` repair tests went with that branch. The
    example runs only the filter, its header pointing to the page for both
    paths, and repairs the plan through a route's variable bound, which every
    model class can write. Corrected claims: the two paths may return
    different types, the same on `highs_lp`, `highs_qp`, `xpress_lp` and
    `copt_lp`, and a printer is a template because one Gurobi, CPLEX or
    `xpress_milp` answer holds two status variants; the Two paths table
    names `iis_by_deletion_t<M>`; `diagnose` also falls back at run time,
    on `solver_error` only; a plain member is the entity's finite sides as
    one unit; a write-back that fails while a trial's exception propagates
    is dropped, the restoration claims pointing to What a run changes; the
    filter's guarantee needs a callback whose decision depends on the
    candidate point alone (N42); `highs_qp` restores its objective through
    the quadratic terms and `set_quadratic_objective`; each Native IIS note
    says which routine settings reach `compute_iis()`; the C API sentence is
    true for Xpress now and names Gurobi's discarded attributes; "its
    routine" replaces "its path". The review's feature-table item did not
    hold: `tested_features_table.py` already had the rows "IIS, native" and
    "IIS, deletion filter", and `make features_tables` changed no pixel.
  - (i) Measured line deltas from `691f0be`: the headers +615/−721 over 21
    files, net −106; the core +305/−159 (`restore_guard.hpp` 41 lines new,
    `handle_status_table.hpp` 91 to 58, `iis_by_deletion.hpp` 509 to 541
    with the overload) and the five bases +310/−562 (HiGHS −32, Gurobi −41,
    CPLEX −30, Xpress −73, COPT −76). The non-blank lines inside the
    infeasibility page's snippet regions go from 240 to 112, the
    deletion-filter page's from 57 to 53, and the example from 189 lines to
    119 (172 to 108 non-blank). The tests grow by +966/−271.
  - (j) Verification. The integrated branch at `afc162e`: gcc15_c++26 with
    every solver, 518 of 525 ctest entries passed and 7 skipped (the
    `*_api` version tests and the native HiGHS entries below the 1.14
    floor); HiGHS 1.15.1, 84 of 85, the entry skipped running only below the
    floor; Xpress 45.01 through `xpressmp_old`, the IIS suites 6 of 6;
    gcc14_c++23 and clang18_c++23 on Clp, Cbc, GLPK, HiGHS and dumb, 237 of
    243 each, 6 skipped; `TEST_SANITIZE=address,undefined` on Clp, HiGHS
    and dumb, 171 of 176 and the HiGHS suites on 1.15.1 84 of 85. After the
    review's fixes, at `4058e69`: gcc15_c++26 525 entries with no failure
    and HiGHS 1.15.1 85 with no failure; format, includes, doc snippets and
    `zensical build --clean` clean. Five integration commits fixed what the
    matrix found (`dfe5423`, `03d8a4a`, `032f497`, `8622ecd`, `afc162e`):
    a page claim, the COPT and CPLEX test models, and three `-Wshadow`
    warnings. Gurobi discards `IISConstr` and `IISUB` after `compute_iis()`
    on a model with an indicator (error 10005), measured; the error paths
    of `refine_lp_status` where `optimize` or `primopt` throws rest on
    `restore_guard`'s unit test, except the CPLEX one `4e5554f` reaches.
    Still pending: the library-wide reset of `_status` first in every
    `solve()`, or a documented rule that a throwing solve keeps the previous
    status, and the optional rename of `solve_limit` and `max_solves` to
    `trial_limit` and `max_trials`; the review's other findings outside IIS
    have no ruling.

What this ruling changes: `detail/restore_guard.hpp`,
`detail/handle_status_table.hpp`, `detail/iis_arithmetic.hpp`,
`model_concepts.hpp`, the snapshot, the outcome header, the engine and the
free function; the five native bases and `refine_lp_status` on `gurobi_lp`
and `cplex_lp`; the unit tests, the shared deletion suite and the
per-backend pins; the infeasibility and deletion-filter pages, the concepts
reference, the solver notes, the examples page and the transportation
example. Nothing is published on main.

## Rulings of 2026-10-05, after the merge

The maintainer merged `feat/iis-factorizations` (CI green) and ruled the
review's two pending decisions, the N15 follow-up and the findings outside
IIS:

- **N46. The review's last decisions.**
  - (a) Decision 10: every `solve()` starts with `reset_status()`, so a solve
    that throws leaves `unknown` rather than the status of the solve before
    it, the rule `compute_iis()` already followed. Only MOSEK reset first
    before; HiGHS, Clp, Cbc and SoPlex reset only on a model without
    columns. Gurobi and CPLEX tests make the native solve throw after an
    optimal one (`322a254`).
  - (b) Decision 11: `iis_limits::max_solves` becomes `max_trials` and
    `iis_outcome::solve_limit` becomes `trial_limit`, coherent with
    `inconclusive_trial`: the engine counts oracle calls, one `solve()` each
    on a model (`fa37f8c`).
  - (c) N15's follow-up is closed, left to the assistant: every native
    `compute_iis()` keeps resetting the status. Leaving it untouched can
    contradict what the solver holds after a routine that re-solves, as
    HiGHS's does; refreshing it from the solver reads a stop's limit code
    where a solve had proven infeasibility (Gurobi, 3 turned 9), reads
    nothing after COPT's `COPT_Reset`, and differs between HiGHS releases
    (1.10 sets `kNotset` after a feasible elasticity filter, 1.15.1 reports
    optimal). With (a), every call that runs the solver starts from
    `unknown`, and only `solve()` writes a status back.
  - (d) The other findings: `has_time_limit` checking a `seconds` setter
    while the filter writes a `duration<double>` is not a defect, the
    conversion is expected; Gurobi's single error message per environment is
    documented in one sentence of its IIS note rather than guarded
    (`d02c474`); `highs_lp` no longer lists `status::solution_limit`, which
    no LP status maps to (`c98f381`); the callback handle stays the one
    nested public type of a model, as the ruling of 2026-09-15 decided since
    no call returns it, and the two pages that denied any public member type
    say so (`5767626`).

## Open questions

None remains: N46 ruled the last recommendations of the review of
2026-10-05 on that day, see
[Pending decisions](iis_api_review.md#pending-decisions). The
outward steps of WP1 are done: the documents are committed (`25b4530`, on
the pull request's branch), the reply is posted, `a1a9f11` is tagged
`archive/pr3-a1a9f11` on origin, and pull request #3 is a draft.
