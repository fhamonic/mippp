# IIS computation: design decisions

Design note for the infeasibility-diagnosis feature (roadmap item "IIS").
It records the rulings taken on 2026-09-22 while reviewing the external IIS
pull request, those of 2026-09-27 on [iis_pr_plan.md](iis_pr_plan.md), the
plan for adapting it, and the amendments of 2026-09-28 from a simplification
review held before wave 2, so that the feature is coded once, in the
library's own shape. A *Confirmed reading* spells out a short ruling, as the
maintainer confirmed it on 2026-09-27. It is not user documentation. The
implementation order is in [iis_todo.md](iis_todo.md), and
[Open questions](#open-questions) records that none remains.

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
  `detail::handle_status_table<Status>`, one byte per handle id, where ids
  past the bound read as the variant's alternative 0 (N16 b). The snapshot
  template requires `iis_status::absent` as that alternative, and
  `get_basis()` later instantiates the same table with `basis_status`
  variants. A backend whose routine has details that the unified tags drop,
  such as a native "possible member" flag or a side the unified set cannot
  name, may keep them through the variant and tag hierarchy (N1 a): refinement
  tags that derive from the unified ones, listed only in that backend's status
  variant, so that `is_a` on a unified tag still gives the generalization.
  This is a guideline for when it is needed, and no work is planned for it.
  The template's arguments are not a commitment: docs and tests obtain the
  type through `model_iis_t<T>` or `auto` and never spell `iis_snapshot<...>`,
  and `lp_iis<I, T>` checks members, not the shape, so that later candidate
  kinds, integrality first, can come as further per-kind tables with their own
  accessors, never as new tags (N37 a, b).
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
- **A completion status of its own.** `iis_outcome` distinguishes an
  irreducible conflict (`irreducible`), a conflict not proven minimal
  (`not_proven_minimal`), a feasible model (`feasible`) and an undetermined
  outcome (`undetermined`). `iis_reason` gives the reason when the proof
  stopped early: `solve_limit`, `time_limit`, `cancelled`, or
  `inconclusive_trial` when a trial could not decide (N2 b). Every native
  routine can end partially: Gurobi `IISMinimal`, COPT `IsMinIIS`, CPLEX's
  abort statuses and "possible member" flags, Xpress `IISSOLSTATUS`, HiGHS's
  "maybe in conflict". The unified tags have no possible-member tag (N1 a).
  Retained, untested and native "possible" members get `member_*` under
  `not_proven_minimal`, plain `member` included where the routine does not
  name the side. A native answer not proven minimal while no limit stopped it
  carries no reason: an empty reason means that the routine itself did not
  prove minimality (N27 a). `time_limit` is still set on a stop attributed to
  the time limit. Only CPLEX and Xpress name a limit stop in their answer
  (measured). HiGHS attributes one by the time measured around the call
  (N34 a), and Gurobi and COPT need the same test (N29 evidence).
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
  headers, but their effect on special constraints was not probed. A wrapper
  that cannot keep them out must not claim that the reported rows and bounds
  conflict on their own. On either path, `irreducible` with zero members means
  that the background alone is infeasible.
- **Names.** No generic identifiers such as `result`, `options` or `member`
  in a namespace users are told to open. The tags live in `iis_status`. The
  other types are `iis_outcome`, `iis_reason`, `iis_limits`, `lp_iis<I, T>`,
  `model_iis_t<T>` and `has_iis<T>`, the names that came with the
  recommendation of Q2. Under N19, the names of the engine, the free function,
  its concept and the snapshot start with `deletion_` or `iis_`, except the
  verb `compute_iis_by_deletion`, and outcome and reason are `enum class`.

## Native routines

| Solver, probed release | Entry point | Explains | Row sides | Bound sides | Partial answer | Stopped by `set_time_limit` |
| --- | --- | --- | --- | --- | --- | --- |
| Gurobi 12.0.1 | `GRBcomputeIIS` | the MIP | membership only | `IISLB`, `IISUB` | `IISMinimal` | yes, documented and measured |
| CPLEX 22.1.2 | `CPXrefineconflictext` | the MIP | membership only | lower, upper | abort statuses, "possible" flags | yes, documented and measured |
| COPT 8.0 | `COPT_ComputeIIS` | the MIP | per side, reliable on LPs only | per side | `IsMinIIS` | on `copt_milp`, measured, undocumented, and on `copt_lp` once fix 10 adds the setter (N35 a) |
| Xpress 47.01 | `XPRSiisfirst`, `XPRSgetiisdata` | the MIP | `L`, `G`, or `E` for both | `L`, `U` | `IISSOLSTATUS` | yes, measured, implied by the manual |
| HiGHS 1.15.1, routine from 1.12.0, floor 1.14.0 | `Highs_getIis` | the relaxation | per side | per side | "maybe in conflict" | no, `iis_time_limit` replaces it, the option documented, the replacement read in the sources and measured |
| SCIP 10.0 sources | `SCIPgenerateIIS`, `SCIPgetIIS` | not examined | a sub-SCIP | a sub-SCIP | irreducible flag | not examined |

The last column comes from the time-limit probes of 2026-09-27, on the
releases the introduction lists.

- **Gurobi and CPLEX.** Neither names the side of a row. An inequality row
  gets its side from its sense; an equality row is reported as `member`.
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
  model (measured).
- **COPT.** On MIPs it flags a single side of an equality row even when both
  are needed: integer x with `x = 0.5`, integer x with `2x = 1.5`, and
  integers x, y with `x + y = 1.5` each got one side only. An LP IIS never
  needs both sides of one row, so the flags are consistent on LPs. `copt_milp`
  therefore reports equality rows as `member`. `copt_lp` lacks
  `has_time_limit` on main, although `COPT_SolveLp` honors `TimeLimit`
  (measured on 8.0.5). Under N35 (a), the plan's fix 10 moves the setter into
  `copt_base`, and under N29 (a) the model's time limit then bounds a native
  `copt_lp` call as it does on `copt_milp`. On 8.0.5, a stop before any
  subsystem leaves `HasIIS` at 0, and a feasible model returns the generic
  code 3 (measured). Two measured findings need care in WP16. On a
  time-limited MIP IIS equal to the whole model, 101 rows and 200 columns, the
  per-entity getters flagged two rows and one column, against `IISRows` and
  `IISCols`. Separately, one `IsMinIIS` = 1 answer on a 20-column, 10-row
  binary model was feasible when re-solved, which is unexplained.
- **Xpress.** By default integrality restrictions are removable candidates,
  listed as `I` members. With integers x, y and rows `y = 0` and
  `x + y = 1.5`, the default returned both rows, which is not irreducible once
  integrality is background; with the `IISOPS` integrality bits set it
  returned `x + y = 1.5` alone. `IISOPS` bits mark element classes as fixed,
  and fixed elements are still listed, so the wrapper sets the bits and drops
  the `I` entries. Rows map `L` to `member_upper`, `G` to `member_lower` and
  `E` to `member_both`, to confirm at 45.1, the range floor. On 45.01, the MIP
  IIS of a 4-row, 26-column market split aborted twice with SIGABRT under the
  default `IISOPS`, and completed with `IISOPS` = 17, which keeps integrality
  fixed (measured). A stop returns `p_status` 3, and `NUMIIS` 0 then means no
  answer (measured on 45.01 and 47.01).
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
- **SCIP.** No routine up to 9.2.1. SCIP 10.0 added IIS finder plugins whose
  answer is a sub-SCIP flagged infeasible and irreducible. Mapping it back
  to handles was not examined, so `scip_milp` starts on the free function.
- **The others.** MOSEK, Clp, Cbc, GLPK and SoPlex have no IIS routine;
  MOSEK's infeasibility report and `MSK_primalrepair` are not one.
- **Time limits.** The routines of Gurobi, CPLEX, Xpress and `copt_milp` stop
  under the parameter that `set_time_limit` writes, within 0.05 s, 0.43 s,
  0.25 s and 25 ms of the limit (measured). Gurobi and CPLEX document it, the
  Xpress manual implies it, and COPT does not mention it. HiGHS documents and
  applies its own `iis_time_limit` instead. Each call gets a fresh budget
  (measured on all five), so `solve()` followed by `compute_iis()` may take
  twice the limit. Other limits stop some routines too: CPLEX's iteration and
  node limits (measured) and its memory limit (documented), Gurobi's
  `SoftMemLimit`, which `set_memory_limit` writes (documented), and Gurobi's
  `WorkLimit`, which MIP++ does not write (documented and measured). HiGHS
  lifts its iteration limit during the call (read in the 1.15.1 sources). A
  stop can leave no answer at all on every backend (measured). Under N29 (a),
  the model's time limit bounds each call as a fresh budget, and HiGHS gets it
  through the per-call copy into `iis_time_limit`, restored afterwards. Under
  N9, the other limits may also stop it. The docs state per backend that the
  budget is per call, which other model limits also stop the routine, and that
  a stop may return late or with no answer. `copt_lp` has no time limit on
  main. Fix 10 gives it one (N35 a), which then bounds its call as on
  `copt_milp`. An explicit duration per call is only a Deferred possibility
  (N29 b), and leaving the call unbounded (N29 c) or to the solver's own rule
  (N29 d) is rejected.
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
  `unknown`, the switch being a follow-up.
  CPLEX's refiner replaces `CPXgetstat` with its conflict statuses. Gurobi's
  `GRBcomputeIIS` overwrites the attributes `Status` and `Runtime`: a stopped
  call turned an infeasible LP's status 3 into 9, the time limit (measured on
  13.0.2). COPT's `COPT_ComputeIIS` leaves `Status`, `LpStatus`, `MipStatus`
  and `HasLpSol` as the solve set them, stopped or not (measured on 8.0.5).
  That is half of the probe, since whether the held solution survives was not
  checked. Xpress is unprobed.
- **Wrappers restore what they set.** Each native wrapper restores its own
  parameters through a small RAII helper: HiGHS `iis_strategy` and
  `iis_time_limit` (N29 a), Xpress `IISOPS`, Gurobi's `IIS*Force` attributes
  (N15). The free function's guard saves model data and lives in its detail
  namespace, so it is not reused, which departs from the WP7 guard of the N15
  recommendation.
- **Split.** Native, where `has_iis` holds: `gurobi_*`, `cplex_*`, `copt_*`,
  `xpress_*`, `highs_lp` and `highs_qp`. The free function, once each model
  has the capabilities it requires: `clp_lp` (WP8), `cbc_milp` (WP9),
  `highs_lp`, `highs_milp` and `highs_qp` (WP10), `glpk_*` (WP11), `mosek_*`
  (WP12), `scip_milp` (WP13) and `soplex_lp` (WP14). `copt_*`, `cplex_*` and
  `xpress_*` join it once they get modifiable row bounds, only where the
  solver stores a row's two sides natively, to be confirmed per solver
  (N30 b). `gurobi_*` does not, since its ranges add a slack column. `dumb_lp`
  joins later (N24). The work packages are those of
  [iis_pr_plan.md](iis_pr_plan.md).

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
  the second doubles the column count. Warm starts depend on the backend:
  simplex LP backends reuse their basis, but SCIP drops its transformed
  problem on every modification and GLPK's MIP runs with presolve on, so
  their trials are cold solves.
- **One side at a time.** Each finite variable bound and each finite row
  side is a candidate, ranged rows included (Q6 a). Row sides are read
  through `has_readable_constraint_bounds` and relaxed through its twin
  `has_modifiable_constraint_bounds<T, M = T>` (Q3 a), so equality and
  ranged rows need no branch. A lower side is a candidate when it is above
  `-infinity()`, and an upper side when it is below `infinity()`. A side at
  the wrong infinity is therefore a candidate, not ignored. Crossed bounds
  are decided by the prechecks below.
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
  N4 (A): `get_time_limit()` is never negative. `clp_lp` and `glpk_*` lack
  `has_time_limit`, and so does `copt_lp` until fix 10 lands (N35 a), so one
  trial can overrun the deadline there. The first real forwarding runs on Cbc.
- **Stop reasons.** After any inconclusive trial that ran, initial or
  singleton, the reason is `inconclusive_trial` (N2 b), or `time_limit` when
  the deadline has passed as the trial returns. `solve_limit` and
  `cancelled` apply only when the engine stops before a trial. With
  `max_solves = 1`, an inconclusive initial trial therefore ends
  `undetermined` with `inconclusive_trial`, never `solve_limit`.
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
  pair is reported `not_proven_minimal` with the stop's reason, and the model
  is left as it was. The former zero-solve claim on crossed variables, and the
  type-level list of background capabilities it needed, are dropped: a
  capability added later could not silently make it unsound. On `scip_milp`
  the trials meet the gap of the next bullet.
- **A documented gap on SCIP.** SCIP creates [0, 1] integer columns as
  `BINARY`, and `scip_milp` keeps them so (N25). Whether SCIP rejects or
  clamps a relaxed bound on such a column is unprobed. If it clamps, the
  relaxed bound stays in force, so the filter may drop a bound the conflict
  needs, and the answer holds only with the column's [0, 1] domain as
  background. The crossed-pair trials of N36 meet the same gap there. If it
  rejects, the trial throws (inferred). The design is centred on linear
  programs, so the gap is documented, not worked around: neither the algorithm
  nor `scip_milp` changes to conform to SCIP. The N14 probe of WP13
  characterizes the gap for the documentation.
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
  the model's `reset_status()` (N22). None of the open-source backends has a
  candidate-solution callback today. On a model that has one, the trials run
  it.

## Library additions the free function needs

Compile-checked on 2026-09-22 against the model classes that the free
function targets first, and on 2026-09-27 for `dumb_lp` (x: satisfied). The
row-bound setters, the entity enumeration and the status reset were concepts
that main did not have, so no model satisfied them. The table is that
pre-wave-1 picture: on 2026-09-28 wave 1 brought the enumeration (`3c6348f`),
the status reset (`0046153`, `ce792ec`) and readable row bounds (`9a852f9`)
to every model, and modifiable row bounds to `clp_lp` (`93063d1`):

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
  `has_readable_constraint_bounds` (Q3 a). No backend has it yet. Every
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
  never on Gurobi.
- **SoPlex.** It joins last (N11 a). The C API of 6.0.4 has
  `SoPlex_changeVarBoundsReal` but sits below the 7.1.1 floor. 7.1.3
  declares every symbol N11 needs, `SoPlex_getRowBoundsReal` included, so
  the wrapper keeps no state. The eight symbols are bound once 7.1.1 and
  7.1.2 are checked, or the floor is raised: `SoPlex_getRowBoundsReal` in
  WP6c, the seven others in WP14.
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
  0 >= 1 already came back as a lower-side member.
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
  function directly on a model that meets both concepts; the throw below HiGHS
  1.14.0.
- **CI.** Clp, Cbc, GLPK and HiGHS run in CI, so the free function is
  CI-tested. CI always runs at least one of them, so no user-defined model
  runs the free function for now (N24). HiGHS runs the native routine in the
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
  `is_a<iis_status::member>` (N37 b, d).

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
  overload of `compute_iis_by_deletion`. Protected sides and candidate order
  are the first extension after the first version (N37 f).
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
- **A user-defined model in CI.** `dumb_lp` with two-sided rows and modifiable
  row bounds, its readable ones coming from WP6c, running `IisByDeletionTest`
  (N24). CI always runs at least one open-source solver, so it is not a
  priority.

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
  which departs from the recommendation.
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
  `IisByDeletionTest` for now. It moves to Later.
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
  of `IisByDeletionTest` to Later.
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

## Open questions

None, as of 2026-09-28. N37, the last, was agreed that day. The outward steps
of WP1 are done: the documents are committed (`25b4530`, on the pull
request's branch), the reply is posted, `a1a9f11` is tagged
`archive/pr3-a1a9f11` on origin, and pull request #3 is a draft.
