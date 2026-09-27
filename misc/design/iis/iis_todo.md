# IIS implementation order

Companion to [iis.md](iis.md), which holds the design, its rulings and the evidence behind them, and to
[iis_pr_plan.md](iis_pr_plan.md), whose work packages carry the details. The steps follow the revised package
order of 2026-09-27. Items 0.1 and 1.6 and those of steps 2 to 6 name their package, except 6.2. Items 1.1 to
1.5 are the plan's standalone fixes, and step 7 is WP17. Each step lands and is tested on its own. Q1 to Q6
and N0 to N35 refer to the Rulings of 2026-09-27 in iis.md, and no question is open.

- **Two paths.** Under Q1 (b), `has_iis<T>` means that the model has a native IIS routine, reached through a
  `compute_iis()` member. The deletion filter is a separate public algorithm: an engine over a user oracle,
  and a free function over any model that meets its concept.
- **Names.** The names below are those the maintainer accepted under N19 to N22 on 2026-09-27, with the IIS
  vocabulary of Q2.
- **Marks.** A ticked item is done on main. A struck item is closed by a ruling. The maintainer confirmed the
  readings of the short rulings on 2026-09-27, and N4's was dropped as moot, so no item marks a reading.
- **Critical path.** Steps 0, 2, 3 and 4, then items 5.1 to 5.3. Each fix of step 1 lands before the first
  item that needs it. Step 6 can start once step 2 merges, HiGHS first.

## 0. Before any code

Q1 to Q6 and N0 to N35 were ruled on 2026-09-27. What remains are the outward steps of WP1, none of them done.

- [ ] **0.1. Commit the documents (WP1).** The three documents were revised on 2026-09-27 in the working tree
  and are not committed. iis.md records the rulings, those of later that day included, with the readings the
  maintainer confirmed, and lists no open question. The plan's corrections are applied, and this file is
  rewritten. Commit iis.md, iis_todo.md, iis_pr_plan.md and iis_pr_plan_pre_rulings.md on main. The last one
  is the plan the maintainer ruled on, which keeps the options behind each letter readable.
- **0.2. No open question.** N35, the last question, was ruled (a) on 2026-09-27, and the same message
  confirmed the readings of the short rulings and left the choice on N4 to the assistant.
- [ ] **0.3. Answer the pull request and act on N0 (a).** Tag `a1a9f11` on origin, for example
  `archive/pr3-a1a9f11`, before any reshaping push, and convert pull request #3 to a draft. Post the plan's
  reply, which asks for the author's consent, asks them to stop pushing, and offers leaf work off the critical
  path.

Done when the four documents of 0.1 are on main, the reply is posted, and pull request #3 is a draft.

## 1. Fixes on main

Each fix is its own pull request on main, independent of the IIS types. Two items of the former MOSEK step are
on main, and its optimizer-count test is struck. The plan also tracks three fixes outside this order: the
loader message (N18 a), the destructors of COPT, CPLEX, Gurobi and Xpress that can throw, and a time limit on
`copt_lp` (N35 a).

- [x] **One optimization per `solve()`.** Done in `d8bb08f`: `mosek_lp` and `mosek_milp` call
  `MSK_optimizetrm` once per solve (`mosek_lp.hpp:102-108`).
- [x] **The MOSEK time limit under test.** `TimeLimitTest` runs for `mosek_lp` and `mosek_milp` on main
  (`test/solvers/mosek.cpp:23, 39`).
- [ ] ~~**A regression test counting optimizer runs per solve.**~~ Struck by N5 (b). The optimizer-count and
  interior-point tests are dropped with their binding, since no test calls a vendor API.
- [ ] **1.1. Moves keep the status, and HiGHS initializes `psolstatus`.** The move constructors of `clp_lp`,
  `cbc_milp`, `scip_milp` and `soplex_lp` copy `_status`, pinned by `LpModelTest.move_preserves_status`. The
  three HiGHS classes initialize `psolstatus`, which a load or solve error leaves unwritten.
  `LpStatusTest.constant_row_without_variables_is_not_optimal` checks a term-less row 0 >= 1. Needed by 5.3.
- [ ] **1.2. GLPK status mapping.** `glpk_lp` maps `GLP_INFEAS` to `infeasible` and lets `glp_simplex` errors
  fall through. The fix is noted as a mapping change in the release notes of v1.1.0, since main has no
  changelog file. Needed by 5.2.
- [ ] **1.3. Native ids after removal.** Gurobi's indicators and type setters, and CPLEX's indicators, pass
  the handle id instead of the native id. Two `RemoveVariableTest` cases pin the fix. Needed by 6.3 and 6.4.
- [ ] **1.4. Clp's secondary status (N13 a).** Bind `Clp_secondaryStatus`, a mandatory symbol. Status 0 with
  secondary status 2 or 4 becomes `optimal_infeasible_unscaled`, and secondary 3 stays `optimal`. The wider
  public variant of `clp_lp` gets a note in the release notes of v1.1.0. The classifier of 4.1 treats the tag
  as inconclusive, which gives N13 (b). Needed by 4.2.
- [ ] **1.5. The time-limit contract (N4 A, N32 a, N33 b).** `get_time_limit()` never returns a negative value
  on a `has_time_limit` model. The MOSEK getter maps its -1 s to `std::numeric_limits<double>::infinity()`,
  porting the getter half of the pull request's change with the author's credit, and the MOSEK setter stays
  transparent. The SoPlex setter throws on a negative value and writes `std::min(t, 1e100)`, so that
  `set(+inf)` lifts an earlier limit. The Cbc setter throws `solver_error` on a negative value, and its getter
  stays transparent (N33 b). The SCIP and CPLEX setters clamp to 1e20 and 1e75, their fresh values, so +inf
  and `duration<double>::max()` mean no limit (N32 a). Five new `TimeLimitTest` cases, from
  `fresh_time_limit_is_unlimited` to `forwarded_time_limit_restores_exactly`, run on all 15 classes.
  `docs/solving/status-and-limits.md` states the contract, and `docs/solvers/index.md` corrects its SoPlex
  sentence. Needed by 5.1, 5.4 and 5.6.
- [ ] **1.6. MOSEK fixes (WP2).** The pull request's handle guard, a destructor that cannot throw, and slot
  selection on the LP status and every getter of both classes. The pick is cached and reset with `_status`.
  The status is reset before optimizing, and a column-less task maps MOSEK's own status. License codes map to
  `license_error` in their own commit, without hint text (N18 a). Needed by 5.4. It lands before or after 3.3,
  and whichever of the two lands second resets the slot cache inside the MOSEK `reset_status()`.

Done when each fix's suites pass, in CI for Clp, Cbc, GLPK and HiGHS and locally for the other backends.

## 2. Engine and IIS types, in pull request #3

The maintainer reshapes pull request #3 in place under N0 (a), after 0.3, in the four commits of the plan's
handling section. Nothing here touches a model.

- [ ] **2.1. The public deletion engine (WP4).** `deletion_filter` in `utility/deletion_filter.hpp`, namespace
  `mippp`. Its oracle receives the active candidate indices and answers feasible, infeasible or inconclusive.
  The result lists the members in ascending order, with the outcome and the reason. The detail entry adds the
  continuation from a known proof, a `Clock` parameter, and a batch size that defaults to 1. Batching is
  ported dormant for the moment, with its tests (N3 b), and the free function never sets the batch size, which
  stays outside `iis_limits` and the public entry. Tests in `test/deletion_filter.cpp` run on a fake clock,
  and `test/iis.cpp:1009-1022` at `a1a9f11` is inverted per N2 (b).
- [ ] **2.2. Limits, outcomes and reasons (WP5).** `iis_limits {max_solves, time_limit, stop_token}`,
  `iis_outcome` and `iis_reason`, including `inconclusive_trial`, in the std-only `utility/iis_outcome.hpp`
  that `model_concepts.hpp` and the engine include. Outcome and reason are `enum class` (N19 e). It lands in
  the engine's commit or before it. The duration becomes one deadline, NaN and negative durations are
  rejected, and an infinite one means no deadline. Prechecks are free (N7 a), and a proof on the last
  permitted trial is complete. A stop request beats the deadline, which beats the solve count. A native answer
  not proven minimal while no limit stopped it has no reason (N27 a).
- [ ] **2.3. Status tags, snapshot and concept (WP5).** The `iis_status` tags next to `basis_status`, with
  `member_lower`, `member_upper` and `member_both` deriving from `member`. The public snapshot template
  `iis_snapshot` sits over `detail::handle_status_table<Status>`, which basis support reuses later (N16 b).
  The item also adds `lp_iis<I, T>`, `model_iis_t<T>` and a single-parameter `has_iis<T>`, documented as "the
  model has a native IIS routine" (Q1 b). No archetype (N10 a) and no possible-member tag (N1 a). A comment
  says that the snapshot describes the model as it was, so a later handle may reuse a removed id (N8 a).
- [ ] **2.4. The exact oracle and the published vectors (WP5).** The oracle in
  `test/test_suites/iis_oracle.hpp` and the HiGHS vectors with their attribution, shared later by both IIS
  suites. Self-tests in `test/iis_oracle.cpp` and `test/iis_snapshot.cpp` run in `mippp_test`.

Done when pull request #3 is green and squash-merged with the author's credit, and no model satisfies
`has_iis` yet. Under N17 (a), no tag is cut between this merge and 4.2.

## 3. Capabilities the free function needs

Each item is its own pull request on main, and all four can run beside step 2. None of them mentions IIS.
Items 3.1 to 3.3 add their `concepts.md` rows and features-table labels, and 3.4 updates the existing ones.

- [ ] **3.1. Row bounds on Clp (WP6).** `has_modifiable_constraint_bounds<T, M = T>` next to its readable
  twin (Q3 a). `clp_lp` writes `Clp_rowLower` and `Clp_rowUpper`, as `set_constraint_rhs` already does.
  `ModifiableConstraintBoundsTest` covers each side of `<=`, `>=`, `==` and ranged rows, gating the ranged
  case on `has_ranged_constraints` (Q6 a). Its case `infinity_frees_a_row_side` mirrors
  `infinity_removes_a_bound`.
- [ ] **3.2. Entity enumeration on every model (WP6a).** `variables()` and `constraints()` on the 19 model
  classes and `dumb_lp`, with the concepts `has_enumerable_variables` and `has_enumerable_constraints`, which
  take `<T, M = T>`, require `std::ranges::sized_range` and stay out of `lp_model` until 2.0. Each returns a
  sized snapshot in increasing id order that holds no reference to the model (N21 a). `model_base` gives the
  default, and the remapping bases, `clp_lp` and `dumb_lp` override it. SCIP's protected `variables` and
  `constraints` members are renamed first (N20 a). `EnumerableEntitiesTest` runs on every model fixture, and
  `LpFuzzyTest` checks the enumeration after each operation.
- [ ] **3.3. Status reset on every model (WP6b).** A public `reset_status() noexcept` on the 19 models and
  `dumb_lp`, with the concept `has_status_reset<T>` (N22 B1). The case
  `LpModelTest.reset_status_reports_unknown` solves, resets, reads `unknown`, then re-solves to the same
  status. It lands before or after 1.6, and whichever of the two lands second resets MOSEK's slot cache inside
  it.
- [ ] **3.4. Readable row bounds on every model (WP6c).** `has_readable_constraint_bounds`, that is
  `get_constraint_lower_bound` and `get_constraint_upper_bound`, on the 19 model classes and `dumb_lp` (Q6 a).
  Main has it on `clp_lp` and `cbc_milp` only. Where the solver has no native ranged rows, as on Gurobi, the
  model's getters derive the bounds from the sense and the rhs: a `<=` row reads (`-infinity()`, rhs), a `>=`
  row (rhs, `infinity()`) and an `==` row (rhs, rhs), with the backend's own `infinity()`. The branch lives in
  the model's method, never in the IIS code. `scip_milp` binds `SCIPgetLhsLinear` and `SCIPgetRhsLinear`, and
  `soplex_lp` binds `SoPlex_getRowBoundsReal` after checking it in 7.1.1 and 7.1.2, or raising the floor
  (N11 a). COPT and Xpress bind a row getter too, since none is bound on main. `ReadableConstraintBoundsTest`
  runs on every model fixture, and the `concepts.md` row and the feature tables follow. Modifiable row bounds
  still follow N30. Needed by 5.2 to 5.6 and by the row-bound commits of 6.4 to 6.6.

Done when the four suites pass on Clp in CI. The enumeration, reset and readable-bounds suites also pass on
every other model, in CI on Cbc, GLPK, HiGHS and `dumb_lp` and locally on the rest.

## 4. The free function and its Clp slice

The first end-to-end slice. It needs step 2, items 3.1 to 3.3, and 1.4. `clp_lp` has no time limit, so budget
forwarding is tested on stubs here, and first runs on a real solver in 5.1.

- [ ] **4.1. The public free function (WP7).** `compute_iis_by_deletion(model, limits)` in
  `utility/iis_by_deletion.hpp`, with the requirements concept `iis_by_deletion_model`. It returns the
  snapshot of 2.3 by value (Q2 a). It adapts through `if constexpr` on capability concepts only, never on
  backend types. No backend gains a member for it, and no protected hook or friend declaration is needed.
  - [ ] **Requirements.** `lp_model`, the enumeration, readable and modifiable variable bounds and row
    bounds, a readable objective, the status reset, and a readable quadratic objective on a `qp_model`.
  - [ ] **Classifier.** A visitor over `model_status_t<M>`. Anything derived from `infeasible`, and exactly
    `infeasible_or_unbounded`, proves infeasibility. `optimal_infeasible_unscaled`, anything derived from
    `failed`, and `unbounded` are inconclusive. Anything else proves feasibility exactly when
    `solution_available` holds.
  - [ ] **Guard and applier.** The guard saves by value the finite candidate sides, the objective
    coefficients and offset, and the Hessian triples on a `qp_model`. The first mutation waits for the
    first trial. Trials zero the objective and leave the sense, verbosity, tolerances, other limits and the
    matrix alone. `restore()` attempts every item and rethrows the first error, and the `noexcept`
    destructor restores only if `restore()` did not run. The applier sets one active set, and restoring
    applies the full set.
  - [ ] **Forwarding.** Only with a finite deadline and `has_time_limit`, the guard reads the caller's
    limit once and each trial gets `std::min(remaining, saved)`, in that argument order. `saved` is
    restored exactly on every exit. With the default limits, `get_time_limit` and `set_time_limit` are
    never called.
  - [ ] **Prechecks.** A model with no live variable is decided by N6 (b). The comparison with 0 is exact and
    documented. A variable whose bounds cross is irreducible at zero solves, unless
    `detail::may_carry_background<M>` holds over the special-constraint and callback capabilities (N7 a2).
    `has_native_handles` is not background, so constraints added through `native_model()` are outside the
    guarantee (N23 a). Crossed rows get two singleton trials through the engine's continuation. Prechecks run
    whatever the limits.
  - [ ] **Status.** `unknown` after any run that called `solve()`, exceptions included (Q4 a).
    A run answered by prechecks alone, or stopped before its first trial, leaves the data and the status as
    they were.
  - [ ] **Stub tests.** `test/iis_by_deletion.cpp` in `mippp_test`, over a scripted stub model and a
    derived probe that asserts per-trial invariants. They cover every row of the classification, and a
    throw at trial k and during the restore. They also cover lazy and quadratic objectives, prechecks at
    zero budget, and the forwarding cases of the time-limit audit.
- [ ] **4.2. `IisByDeletionTest` on Clp (WP8).** The suite in `test/test_suites/iis_by_deletion.hpp`, over the
  shared case bodies of `test/test_suites/iis_cases.hpp`, written as fixture member functions. It is
  registered in `all.hpp` and instantiated for Clp in the same pull request. `dumb_lp` is not instantiated for
  now (N24). `has_iis<clp_lp>` stays false. The column-less case asserts the N6 answer. The status case
  asserts `unknown` after a run that solved, and the previous status after a run decided by prechecks.
  Filter-only cases cover the budget sweep, a stop requested beforehand and a 0 s budget. Answers must not
  change after a later `remove_variable` (N8 a). The `concepts.md` row of the free function's concept says
  that zero members with `irreducible` means the background alone is infeasible.

Done when `IisByDeletionTest` passes on Clp in CI, including the case that re-solves the model to its
previous result.

## 5. Widen the free function

CI-tested backends come first. Each item adds modifiable row bounds and any other capability the model lacks,
instantiates the capability suites it newly satisfies, then `IisByDeletionTest`. No item adds `compute_iis()`,
an enumeration, a status reset or readable row bounds, which 3.2 to 3.4 already provide.

- [ ] **5.1. `cbc_milp` (WP9).** Row bounds through `Cbc_setRowLower` and `Cbc_setRowUpper`, which `cbc_api`
  already binds. It is the first MIP under the free function, so it runs the integer-only cases. It also runs
  the first real time-limit forwarding, so it needs 1.5.
- [ ] **5.2. `glpk_lp` and `glpk_milp` (WP11).** Modifiable row bounds through `glp_set_row_bnds`, which
  switches the bound type rather than storing an infinite value, over the readable ones of 3.4. GLPK has no
  time limit, so limits act between trials only. `glpk_milp` trials are cold solves. Needs 1.2 and 3.4.
- [ ] **5.3. `highs_lp`, `highs_milp` and `highs_qp` (WP10).** Modifiable row bounds in `highs_base` through
  `Highs_changeRowBounds`, over the readable ones of 3.4. N12 (b) is its own commit: on a ranged row,
  `get_constraint_sense` throws like Clp and Cbc, and `get_constraint_rhs` throws like Clp. The free function
  saves the Hessian of `highs_qp` through `qp_model`, not through backend code. It carries the
  removed-variable case on a remapping backend. Needs 1.1 and 3.4.
- [ ] **5.4. `mosek_lp` and `mosek_milp` (WP12).** Modifiable row bounds through `MSK_chgconbound`, over the
  readable ones of 3.4, which read `MSK_getconbound`. The time limit needs no special case, since 1.5 makes
  the getter non-negative. Needs 1.5, 1.6 and 3.4.
- [ ] **5.5. `scip_milp` (WP13).** Bind `SCIPchgLhsLinear` and `SCIPchgRhsLinear`, checked across the
  validated range from 8.0.4, since 3.4 binds `SCIPgetLhsLinear` and `SCIPgetRhsLinear`. It rebases onto the
  SCIP rename of 3.2 and onto 3.4. Every trial is a cold solve, so its cases stay small. `scip_milp` keeps its
  `BINARY` columns, and nothing changes to conform to SCIP (N25). The N14 probe, relaxing a bound of a
  `BINARY` column, only characterizes the gap, which this item documents under "Notable current limitations"
  in `docs/solvers/index.md`.
- [ ] **5.6. `soplex_lp` last (WP14).** Under N11 (a), check the seven symbols of 7.1.3 and 8.0.2 that 3.4
  leaves in 7.1.1 and 7.1.2, or raise the floor. Then bind them, and add variable bounds, a readable objective
  and modifiable row bounds. `SoPlex_getRowBoundsReal`, bound by 3.4, reads row sides back, so the wrapper
  keeps no state. Needs 3.4, 5.5 and the SoPlex setter of 1.5.

Done when `IisByDeletionTest` passes on each backend, in CI for Cbc, GLPK and HiGHS and locally for the
rest.

## 6. Native routines

Under Q1 (b), these items add `compute_iis()` members, which never run the deletion filter. The step needs
step 2 only, so it can run beside steps 3 to 5. HiGHS (6.1) goes first and creates the native suite `IisTest`,
so that it runs in CI before any backend tested only locally. Each `compute_iis()` takes no argument (N9). The
model's time limit bounds each call as a fresh budget (N29 a): four routines already stop under it, and HiGHS
through a per-call copy into `iis_time_limit`. Other model limits may also stop some routines, and the docs
list them per backend. It sets the status to `unknown` before its first native call (N15). Once 3.3 has
landed, it may do so through `reset_status()`. It restores every parameter it sets through its own small RAII
helper. It runs no column-less precheck (N28 a), and gives a reason only on a stop attributed to the time
limit (N27 a).

- [ ] **6.1. HiGHS, first (WP15).** `Highs_getIis` in `HIGHS_OPTIONAL_FUNCTIONS`, used on `highs_lp` and
  `highs_qp` only, with `static_assert(!has_iis<highs_milp>)`. Below 1.14.0 (N26 a3), `compute_iis()` throws
  `solver_error`, naming the loaded version, the library path and the floor, with no fallback (Q1 b). An empty
  `library_version()`, as on a devel build, requires the symbol and assumes the newest regime. The wrapper
  sets `iis_strategy` to 6 rather than the default Light, copies `get_time_limit()` into `iis_time_limit` for
  each call (N29 a), and restores what it sets. Row arrays take the row count. Members decode from the bound
  codes through `_var_handle`. "Maybe" entries give `not_proven_minimal`, and a `kWarning` return gives
  `undetermined`. An empty answer is `feasible` when the model status is optimal or unbounded, and
  `undetermined` otherwise. A -1 return is `undetermined` with `time_limit` when the time measured around the
  call reached the copied limit, whatever the model status, and throws `solver_error` otherwise (N34 a,
  amending N26). This covers the -1 with model status 8 that a stop in the elasticity filter gives on 1.14.0
  (measured). The same test attributes `time_limit` to a "maybe" answer. A probe records in iis.md whether
  `Highs_getIis` keeps the held solution and whether the status agrees with it (N15). It re-solves an unsolved
  model and resets the run clock (measured).
  - [ ] **`IisTest`.** The suite in `test/test_suites/iis.hpp` shares the case bodies of 4.2. The fixtures
    `highs_lp_iis_test` and `highs_qp_iis_test` skip below 1.14.0 on `library_version()`, and
    `TEST(HiGHS_lp, compute_iis_below_native_floor_throws)` expects the throw there.
  - [ ] **Both paths.** After 5.3, `both_paths_find_valid_iis` calls the free function directly on models that
    meet both concepts, and the oracle validates both answers.
- [ ] **6.2. Confirm each routine at its range floor.** `COPT_ComputeIIS` in COPT 7.2, `XPRS_IISOPS` in Xpress
  45.1, and the `IIS*Force` attributes in Gurobi 10. Probe each routine on a feasible model, where Gurobi
  gives error 10015, CPLEX conflict status 30 and COPT code 3. The time limit was probed on 2026-09-27 on
  Gurobi 11.0.3 to 13.0.2, CPLEX 22.1.1 and 22.1.2, Xpress 45.01 and 47.01 and COPT 8.0.5 (N29 evidence). At
  Gurobi 10 and COPT 7.2, check that the routine stops under the model's time limit and how it reports the
  stop. At 45.1, run the MIP case that aborted Xpress 45.01 under the default `IISOPS`, with the wrapper's
  bits. Record whether the held solution survives the call and the status agrees with it. Where it does, that
  routine leaves the status untouched (N15).
- [ ] **6.3. Gurobi (WP16).** `GRBcomputeIIS`, with `IISSOSForce`, `IISQConstrForce` and `IISGenConstrForce`
  set to 1 and restored. Inequality rows get their side from the sense, and equality rows are `member`.
  `IISMinimal` = 0 gives `not_proven_minimal` (N1 a). A forced-only answer is irreducible with zero members.
  Error 10005 on `IISMinimal` after a stop means no answer, never an empty IIS. Needs 1.3. Gurobi gets no
  modifiable row bounds, since its ranges add a slack column (N30), and reads its row bounds through 3.4.
- [ ] **6.4. CPLEX (WP16).** `CPXrefineconflictext` and `CPXgetconflictext`, with one group per row and per
  bound side, never the deprecated `CPXrefineconflict`. Indicators stay outside the groups. Status 31 is
  `irreducible` and 30 `feasible`. Abort statuses 32 to 39 and "possible" flags give `member_*` under
  `not_proven_minimal`, and status 33 is a time-limit stop. Status 33 with every group excluded is no answer.
  Rows get their sides as on Gurobi. Needs 1.3. Modifiable row bounds, over the readable ones of 3.4, are an
  optional commit under N30 (b), once CPLEX is confirmed to store ranged rows natively.
- [ ] **6.5. Xpress (WP16).** The `IISOPS` integrality and special-constraint bits are set before
  `XPRSiisfirst` and restored after. `I` entries are dropped. Rows map `L` to `member_upper`, `G` to
  `member_lower` and `E` to `member_both`, to confirm at 45.1. `IISSOLSTATUS` gives the outcome, with
  `p_status` 3 and `NUMIIS` 0 read as a stop with no answer. Modifiable row bounds, over the readable ones of
  3.4, are an optional commit under N30 (b), once Xpress is confirmed to store ranged rows natively.
- [ ] **6.6. COPT (WP16).** `COPT_ComputeIIS` and the four `Get*IIS` calls. Per-side flags on `copt_lp`, and
  equality rows as `member` on `copt_milp`. `IsMinIIS` gives the outcome, and `HasIIS` 0 means no answer.
  Flagged counts that disagree with `IISRows` and `IISCols` give `undetermined` (inferred), a disagreement
  measured on a time-limited MIP IIS on 8.0.5. A binary model whose `IsMinIIS` = 1 answer re-solved feasible
  needs its own probe first. Under N29 (a), `copt_lp` bounds the call once fix 10 gives it a time limit
  (N35 a). Modifiable row bounds, over the readable ones of 3.4, are an optional commit under N30 (b), COPT's
  rows being two-sided natively per its documentation, to confirm.

Done when `IisTest` passes in CI and CI shows HiGHS below and above the native floor. Locally, the suite
passes on each commercial backend at its range floor and latest release, with indicators and a removed
variable on Gurobi and CPLEX.

## 7. Documentation

The user pages come once the CI backends run both paths, after 4.2, 5.1 to 5.3 and 6.1. Rows and labels that a
package adds come with that package.

- [ ] **7.1. Reading an IIS.** `docs/solving/infeasibility.md` covers the `has_iis` gate and the free
  function, the consumer loop, the tags of each path, outcomes and reasons, and `*_lp` against `*_milp`. It
  states the status afterwards: `unknown` after a run that solved, and after every native call for now. It
  explains N8. A per-model table shows `has_iis`, the free function's concept and the HiGHS floor, and later
  packages keep it current. It restates for IIS the warning of `docs/solvers/index.md` on the native handles
  (N23): background added natively is outside the guarantee, and the free function's guard and the entity
  enumeration do not see native changes. It links the SCIP gap of 5.5 (N25). Its paragraph on the time bounds
  of native calls states per backend that the budget is per call, which other model limits also stop the
  routine, and that a stop may return late or with no answer (N29 a). It states that the model's time limit
  bounds a native `copt_lp` call as on `copt_milp`, through fix 10 (N35 a). It replaces the pull request's
  `docs/iis.md`.
- [ ] **7.2. The deletion filter as an algorithm.** A page under Algorithms in `zensical.toml`, next to column
  generation. It covers the engine's oracle contract, monotonicity, limits and reasons. It then covers the
  free function's requirements, what it saves and never touches, and the native-handle warning. It warns that
  one trial can overrun the budget on models without `has_time_limit`.
- [ ] **7.3. Concepts and feature tables.** `concepts.md` rows for every new concept. The labels
  `("IisTest", "IIS, native")` and `("IisByDeletionTest", "IIS, deletion filter")`, next to those of
  `EnumerableEntitiesTest` and `ModifiableConstraintBoundsTest`. `tested_features_table.py` gets the fix
  that makes `highs_qp` rows appear, then the tables are regenerated.
- [ ] **7.4. README and `coming-from.md`.** The README roadmap row, and the list of missing features in
  `docs/getting-started/coming-from.md`.

Done when `zensical build --clean` passes and every snippet compiles.

## Later

The deferred items of iis.md, in no fixed order. WP3 writes a note for each of the pull request's deferred
ideas, pointing into the archive tag of `a1a9f11`.

- **Certificates.** Dual rays, and the ray seeds they allow.
- **Native extras.** Elastic relaxation, forcing and preferences, several IISs per model on Xpress, SCIP 10's
  IIS finder, and cancelling a native call through `compute_iis(std::stop_token)` (N31 b). An explicit
  duration for a native call, if users ask, is a possibility that is not ruled (N29 b).
- **Filter refinements.** Candidate ordering, and a caller for the dormant batching.
- **Integrality as a candidate.** Relaxing an integer or binary variable to continuous rather than its bounds,
  the maintainer's intuition given with N25. It is outside the LP-centred first version.
- **A user model in CI.** `dumb_lp` with two-sided rows and modifiable row bounds under `IisByDeletionTest`
  (N24).
