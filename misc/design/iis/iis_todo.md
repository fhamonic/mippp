# IIS implementation order

Companion to [iis.md](iis.md), which holds the design, its rulings and the evidence behind them, and to
[iis_pr_plan.md](iis_pr_plan.md), whose work packages carry the details. The steps follow the revised package
order of 2026-09-27. Items 0.1 and 1.6 and those of steps 2 to 6 name their package, except 6.2. Items 1.1 to
1.5 are the plan's standalone fixes, and step 7 is WP17. Each step lands and is tested on its own. Q1 to Q6
and N0 to N43 refer to the Rulings of 2026-09-27, 2026-09-28, 2026-09-29 and 2026-10-02 in iis.md, the third on the three
questions wave 3 left.

- **Two paths.** Under Q1 (b), `has_iis<T>` means that the model has a native IIS routine, reached through a
  `compute_iis()` member. The deletion filter is a separate public algorithm: an engine over a user oracle,
  and a free function over any model that meets its concept.
- **Names.** The names below are those the maintainer accepted under N19 to N22 on 2026-09-27, with the IIS
  vocabulary of Q2.
- **Marks.** A ticked item is done on main, or on the branch the item names: pull request #3's,
  `docs/iis-wave4` for step 7, or `feat/iis-wave5` for 6.2 to 6.6. A struck item is closed by a ruling. The maintainer confirmed the readings of
  the short rulings on 2026-09-27, and N4's was dropped as moot, so no item marks a reading.
- **Critical path.** Steps 0, 2, 3 and 4, then items 5.1 to 5.3. Each fix of step 1 lands before the first
  item that needs it. Step 6 can start once step 2 merges, HiGHS first.
- **Waves.** Wave 1, published on main on 2026-09-28 with CI green, is steps 1 and 3 on main and step 2 on
  pull request #3. Wave 2, 2.5 then step 4 and 6.1, landed on 2026-09-28: the merge as `e322ef1`, then the
  five commits `ad0105a` to `19803cc`, all but the both-paths case of 6.1, which waited for 5.3. Wave 3, step
  5 and with it that case, landed on 2026-09-28 and is published on main with CI green (run 36472057802): the
  33 commits `5c4399c` to `9aac8a1`, three commits of the final review (GLPK's infinite sides, SCIP 10's
  status codes, the serial HiGHS timing tests), the record of iis.md and this file, then `d3312b8`, which
  checks the forwarded time limit on every backend with one. It left three questions, N38 to N40, ruled on
  2026-09-29. Wave 4, step 7, landed on 2026-09-29 on `docs/iis-wave4`, locally, awaiting the push: the 22
  commits `1c6256a` to `b4e90d5`, then the record of iis.md and this file. The rulings followed on the same
  branch: the SCIP skip keyed on data (`5047a77`, cherry-picked from `1b91b7d`), Cbc marked experimental
  (`e2d443d`), the opposite infinity documented as undefined behavior (`1c0e50e`), and their record
  (`7d0fd90`). Five commits then answered a reader's review of the infeasibility page, `ccf9c94` to `b95fc8e`
  (see 7.1), and a last one closed the review of this follow-up. Wave 5, 6.2 to 6.6, landed on 2026-09-29 on
  `feat/iis-wave5`, locally, awaiting the push: the four backend branches integrated in the order Gurobi
  (`70374bf`, `4b04d37`), CPLEX (`3e0634b` to `2d3d909`), Xpress (`cd3b3bd` to `35eb0af`) and COPT
  (`f73ac7a` to `c2af30c`), the example fixed to read a member's side through a visitor (`708b645`), the user
  pages and the feature tables (`b5014d1` to `c8e562b`), then the record of iis.md, this file and the plan. A
  second review of the backend branches, folded in the same day, pinned CPLEX's conflict-preference counter
  across a move, sized Xpress's tables after the routine and made `xpress_lp::solve()` postsolve a stopped
  solve (`3acad2c`), and on COPT stopped the confirming solve at its first incumbent, reported two-bounded
  continuous columns whole and a binary column moved outside [0, 1] through its needed bound. A final
  review of the integrated branch, folded the same way, merged an Xpress column listed once per bound into
  `member_both`, detached a registered candidate-solution callback for `compute_iis()` on `xpress_milp`, whose
  routine ran it, and on `cplex_milp`, whose refiner refuses to run with one, answered a column-less model
  on COPT from its rows since the routine leaks there, and pinned the iteration-limit stop CPLEX keeps.
  Every suite passed locally under gcc 15, gcc 14 and clang 18 on the releases step 6 names. The sanitized
  build's ctest, on the CI pins, exercises Gurobi and Xpress and skips every CPLEX and COPT suite; the
  same sanitizer binary run directly against CPLEX 22.1.2, COPT 8.0.5 and Xpress 47.01 passed their
  suites with no sanitizer report once the COPT leak was bypassed. N41, on 2026-10-02, brought the
  deletion filter to `gurobi_lp` and `gurobi_milp` through their rows' sense and rhs, on
  `feat/iis-gurobi-filter`. The review of 2026-10-02 followed, with N42 and N43, and the coherence pass it
  opened landed on `iis/coherence`: the sense-and-rhs fallback narrowed to models without row-bound setters
  (`d9225a9`), the filter's writer reading a row's sense and writing it only on a change of side (`0f2c064`),
  the infeasibility page's repair code and the transportation example running on every model class, Gurobi's
  included (`55160c2`), `IisByDeletionTest` on `dumb_lp` (`e9474f7`), callbacks, MIP starts and the bounds
  of binary variables documented (`e049b88` to `bd3e0e4`), time limits on `clp_lp`, `glpk_lp` and
  `glpk_milp` (`e2d577f` to `6bb2208`), so that every model class has one, the arithmetic of the wrappers
  shared in `detail/iis_arithmetic.hpp` (`2e52aa8`), and on Xpress ranged rows reported whole on a MIP and
  refused searches answered from a column's own bounds (`916db02`, `5c2192f`); the user pages, iis.md, this
  file and the feature tables followed on `iis/coherence-docs`. The shared tests and the guard cleanup land
  on a parallel branch, `<branch>`, as `<commits>`: `<what they change>`.

## 0. Before any code

Q1 to Q6 and N0 to N35 were ruled on 2026-09-27, N36 and N37 on 2026-09-28, and N38 to N40 on 2026-09-29. The
outward steps of WP1 are done.

- [x] **0.1. Commit the documents (WP1).** Done on 2026-09-28 as `25b4530` on pull request #3's branch, which
  reaches main with the squash-merge of 2.5. iis.md records the rulings with the readings the maintainer
  confirmed, and lists no open question; iis_pr_plan_pre_rulings.md is the plan the maintainer ruled on, which
  keeps the options behind each letter readable. The amendments of 2026-09-28 (N36 and N37) follow the same
  route.
- [x] **0.2. Open questions.** N35, the last question before any code, was ruled (a) on 2026-09-27, and the
  same message confirmed the readings of the short rulings and left the choice on N4 to the assistant. The
  three questions wave 3 left, N38 to N40, were ruled on 2026-09-29, recorded under
  [Rulings of 2026-09-29](iis.md#rulings-of-2026-09-29) in iis.md, and none remains open.
- [x] **0.3. Answer the pull request and act on N0 (a).** Done on 2026-09-28: `a1a9f11` is tagged
  `archive/pr3-a1a9f11` on origin, the reply is posted, and pull request #3 is a draft. The maintainer does
  the rework, so the reply offered the author no work.

Done when the four documents of 0.1 are on main, the reply is posted, and pull request #3 is a draft.

## 1. Fixes on main

Each fix is its own commit on main, independent of the IIS types, all landed on 2026-09-28 in wave 1. Two items
of the former MOSEK step were already on main, and its optimizer-count test is struck. The plan's three fixes
outside this order landed too: the loader message (N18 a, `e81fdc5`), the destructors of COPT, CPLEX, Gurobi
and Xpress that could throw (`e43b639`), and a time limit on `copt_lp` (N35 a, `d205cf6`).

- [x] **One optimization per `solve()`.** Done in `d8bb08f`: `mosek_lp` and `mosek_milp` call
  `MSK_optimizetrm` once per solve (`mosek_lp.hpp:102-108`).
- [x] **The MOSEK time limit under test.** `TimeLimitTest` runs for `mosek_lp` and `mosek_milp` on main
  (`test/solvers/mosek.cpp:23, 39`).
- [ ] ~~**A regression test counting optimizer runs per solve.**~~ Struck by N5 (b). The optimizer-count and
  interior-point tests are dropped with their binding, since no test calls a vendor API.
- [x] **1.1. Moves keep the status, and HiGHS initializes `psolstatus`.** The move constructors of `clp_lp`,
  `cbc_milp`, `scip_milp` and `soplex_lp` copy `_status`, pinned by `LpModelTest.move_preserves_status`. The
  three HiGHS classes initialize `psolstatus`, which a load or solve error leaves unwritten.
  `LpStatusTest.constant_row_without_variables_is_not_optimal` checks a term-less row 0 >= 1. Needed by 5.3.
  Done in `1a77969`.
- [x] **1.2. GLPK status mapping.** `glpk_lp` maps `GLP_INFEAS` to `infeasible` and lets `glp_simplex` errors
  fall through. The fix is noted as a mapping change in the release notes of v1.1.0, since main has no
  changelog file. Needed by 5.2. Done in `283a256`.
- [x] **1.3. Native ids after removal.** Gurobi's indicators and type setters, and CPLEX's indicators, pass
  the handle id instead of the native id. Two `RemoveVariableTest` cases pin the fix. Needed by 6.3 and 6.4.
  Done in `23c61b2`.
- [x] **1.4. Clp's secondary status (N13 a).** Bind `Clp_secondaryStatus`, a mandatory symbol. Status 0 with
  secondary status 2 or 4 becomes `optimal_infeasible_unscaled`, and secondary 3 stays `optimal`. The wider
  public variant of `clp_lp` gets a note in the release notes of v1.1.0. The classifier of 4.1 treats the tag
  as inconclusive, which gives N13 (b). Needed by 4.2. Done in `daddd47`.
- [x] **1.5. The time-limit contract (N4 A, N32 a, N33 b).** `get_time_limit()` never returns a negative value
  on a `has_time_limit` model. The MOSEK getter maps its -1 s to `std::numeric_limits<double>::infinity()`,
  porting the getter half of the pull request's change with the author's credit, and the MOSEK setter stays
  transparent. The SoPlex setter throws on a negative value and writes `std::min(t, 1e100)`, so that
  `set(+inf)` lifts an earlier limit. The Cbc setter throws `solver_error` on a negative value, and its getter
  stays transparent (N33 b). The SCIP and CPLEX setters clamp to 1e20 and 1e75, their fresh values, so +inf
  and `duration<double>::max()` mean no limit (N32 a). Five new `TimeLimitTest` cases, from
  `fresh_time_limit_is_unlimited` to `forwarded_time_limit_restores_exactly`, run on all 15 classes.
  `docs/solving/status-and-limits.md` states the contract, and `docs/solvers/index.md` corrects its SoPlex
  sentence. Needed by 5.1, 5.4 and 5.6. Done in `a8ca259`.
- [x] **1.6. MOSEK fixes (WP2).** The pull request's handle guard, a destructor that cannot throw, and slot
  selection on the LP status and every getter of both classes. The pick is cached and reset with `_status`.
  The status is reset before optimizing, and a column-less task maps MOSEK's own status. License codes map to
  `license_error` in their own commit, without hint text (N18 a). Needed by 5.4. It lands before or after 3.3,
  and whichever of the two lands second resets the slot cache inside the MOSEK `reset_status()`. Done in
  `d8fdbb8` and `917fbf6`; 3.3 landed second, and `ce792ec` resets the slot cache inside `reset_status()`.

Done when each fix's suites pass, in CI for Clp, Cbc, GLPK and HiGHS and locally for the other backends.

## 2. Engine and IIS types, in pull request #3

The maintainer reshaped pull request #3 in place under N0 (a), after 0.3, in the four commits of the plan's
handling section, pushed on 2026-09-28 with CI green at `e11c0f5`. Nothing here touches a model.

- [x] **2.1. The public deletion engine (WP4).** `deletion_filter` in `utility/deletion_filter.hpp`, namespace
  `mippp`. Its oracle receives the active candidate indices and answers feasible, infeasible or inconclusive.
  The result lists the members in ascending order, with the outcome and the reason. The detail entry adds the
  continuation from a known proof, a `Clock` parameter, and a batch size that defaults to 1. Batching is
  ported dormant for the moment, with its tests (N3 b), and the free function never sets the batch size, which
  stays outside `iis_limits` and the public entry. Tests in `test/deletion_filter.cpp` run on a fake clock,
  and `test/iis.cpp:1009-1022` at `a1a9f11` is inverted per N2 (b). On the branch as `f9e82ec`.
- [x] **2.2. Limits, outcomes and reasons (WP5).** `iis_limits {max_solves, time_limit, stop_token}`,
  `iis_outcome` and `iis_reason`, including `inconclusive_trial`, in the std-only `utility/iis_outcome.hpp`
  that `model_concepts.hpp` and the engine include. Outcome and reason are `enum class` (N19 e). It lands in
  the engine's commit or before it. The duration becomes one deadline, NaN and negative durations are
  rejected, and an infinite one means no deadline. Prechecks are free (N7 a), and a proof on the last
  permitted trial is complete. A stop request beats the deadline, which beats the solve count. A native answer
  not proven minimal while no limit stopped it has no reason (N27 a). On the branch, between `b6e2132` and
  `e11c0f5`.
- [x] **2.3. Status tags, snapshot and concept (WP5).** The `iis_status` tags next to `basis_status`, with
  `member_lower`, `member_upper` and `member_both` deriving from `member`. The public snapshot template
  `iis_snapshot` sits over `detail::handle_status_table<Status>`, which basis support reuses later (N16 b).
  The item also adds `lp_iis<I, T>`, `model_iis_t<T>` and a single-parameter `has_iis<T>`, documented as "the
  model has a native IIS routine" (Q1 b). No archetype (N10 a) and no possible-member tag (N1 a). A comment
  says that the snapshot describes the model as it was, so a later handle may reuse a removed id (N8 a).
  On the branch as `62adb20`.
- [x] **2.4. The exact oracle and the published vectors (WP5).** The oracle in
  `test/test_suites/iis_oracle.hpp` and the HiGHS vectors with their attribution, shared later by both IIS
  suites. Self-tests in `test/iis_oracle.cpp` and `test/iis_snapshot.cpp` run in `mippp_test`. On the
  branch as `e11c0f5`.
- [x] **2.5. Squash-merge pull request #3 into main.** With a `Co-authored-by` trailer for the author (N0 a),
  once the amendments of 2026-09-28 to the three documents are on its branch. The squash carries `25b4530` and
  the four commits, so main gets the documents, the engine, the types and the oracle at once. Then
  `git worktree prune` locally. Done on 2026-09-28 as `e322ef1`, after a local run of the merged tree with every
  solver source; the wave-1 worktrees and branches are gone.

Done when pull request #3 is green and squash-merged with the author's credit, and no model satisfies
`has_iis` yet. Under N17 (a), no tag is cut between this merge and 4.2.

## 3. Capabilities the free function needs

Each item landed as its own commit on main on 2026-09-28, in wave 1. None of them mentions IIS.
Items 3.1 to 3.3 add their `concepts.md` rows and features-table labels, and 3.4 updates the existing ones.

- [x] **3.1. Row bounds on Clp (WP6).** `has_modifiable_constraint_bounds<T, M = T>` next to its readable
  twin (Q3 a). `clp_lp` writes `Clp_rowLower` and `Clp_rowUpper`, as `set_constraint_rhs` already does.
  `ModifiableConstraintBoundsTest` covers each side of `<=`, `>=`, `==` and ranged rows, gating the ranged
  case on `has_ranged_constraints` (Q6 a). Its case `infinity_frees_a_row_side` mirrors
  `infinity_removes_a_bound`. Done in `93063d1`.
- [x] **3.2. Entity enumeration on every model (WP6a).** `variables()` and `constraints()` on the 19 model
  classes and `dumb_lp`, with the concepts `has_enumerable_variables` and `has_enumerable_constraints`, which
  take `<T, M = T>`, require `std::ranges::sized_range` and stay out of `lp_model` until 2.0. Each returns a
  sized snapshot in increasing id order that holds no reference to the model (N21 a). `model_base` gives the
  default, and the remapping bases, `clp_lp` and `dumb_lp` override it. SCIP's protected `variables` and
  `constraints` members are renamed first (N20 a). `EnumerableEntitiesTest` runs on every model fixture, and
  `LpFuzzyTest` checks the enumeration after each operation. Done in `3c6348f`, after the SCIP rename
  `a848855`.
- [x] **3.3. Status reset on every model (WP6b).** A public `reset_status() noexcept` on the 19 models and
  `dumb_lp`, with the concept `has_status_reset<T>` (N22 B1). The case
  `LpModelTest.reset_status_reports_unknown` solves, resets, reads `unknown`, then re-solves to the same
  status. Done in `0046153`, after 1.6, and `ce792ec` resets MOSEK's slot cache inside it.
- [x] **3.4. Readable row bounds on every model (WP6c).** `has_readable_constraint_bounds`, that is
  `get_constraint_lower_bound` and `get_constraint_upper_bound`, on the 19 model classes and `dumb_lp` (Q6 a).
  Main has it on `clp_lp` and `cbc_milp` only. Where the solver has no native ranged rows, as on Gurobi, the
  model's getters derive the bounds from the sense and the rhs: a `<=` row reads (`-infinity()`, rhs), a `>=`
  row (rhs, `infinity()`) and an `==` row (rhs, rhs), with the backend's own `infinity()`. The branch lives in
  the model's method, never in the IIS code. `scip_milp` binds `SCIPgetLhsLinear` and `SCIPgetRhsLinear`, and
  `soplex_lp` binds `SoPlex_getRowBoundsReal` after checking it in 7.1.1 and 7.1.2, or raising the floor
  (N11 a). COPT and Xpress bind a row getter too, since none is bound on main. `ReadableConstraintBoundsTest`
  runs on every model fixture, and the `concepts.md` row and the feature tables follow. Modifiable row bounds
  still follow N30. Needed by 5.2 to 5.6 and by the row-bound commits of 6.4 to 6.6. Done in `9a852f9`.

Done when the four suites pass on Clp in CI. The enumeration, reset and readable-bounds suites also pass on
every other model, in CI on Cbc, GLPK, HiGHS and `dumb_lp` and locally on the rest.

## 4. The free function and its Clp slice

The first end-to-end slice. It needs 2.5, since step 2 sits on pull request #3's branch, and items 3.1 to 3.3
and 1.4, which are on main. `clp_lp` had no time limit until 2026-10-02, so budget
forwarding is tested on stubs here, and first runs on a real solver in 5.1.

- [x] **4.1. The public free function (WP7).** `compute_iis_by_deletion(model, limits)` in
  `utility/iis_by_deletion.hpp`, with the requirements concept `iis_by_deletion_model`. It returns the
  snapshot of 2.3 by value (Q2 a). It adapts through `if constexpr` on capability concepts only, never on
  backend types. No backend gains a member for it, and no protected hook or friend declaration is needed.
  - [x] **Requirements.** `lp_model`, the enumeration, readable and modifiable variable bounds and row
    bounds, a readable objective, the status reset, and a readable quadratic objective on a `qp_model`.
  - [x] **Classifier.** A visitor over `model_status_t<M>`. Anything derived from `infeasible`, and exactly
    `infeasible_or_unbounded`, proves infeasibility. `optimal_infeasible_unscaled`, anything derived from
    `failed`, and `unbounded` are inconclusive. Anything else proves feasibility exactly when
    `solution_available` holds.
  - [x] **Guard and applier.** The guard saves by value the finite candidate sides, the objective
    coefficients and offset, and the Hessian triples on a `qp_model`. The first mutation waits for the
    first trial. Trials zero the objective and leave the sense, verbosity, tolerances, other limits and the
    matrix alone. `restore()` attempts every item and rethrows the first error, and the `noexcept`
    destructor restores only if `restore()` did not run. The applier sets one active set, and restoring
    applies the full set.
  - [x] **Forwarding.** Only with a finite deadline and `has_time_limit`, the guard reads the caller's
    limit once and each trial gets `std::min(remaining, saved)`, in that argument order. `saved` is
    restored exactly on every exit. With the default limits, `get_time_limit` and `set_time_limit` are
    never called.
  - [x] **Precheck and crossed pairs.** A model with no live variable is decided by N6 (b), whatever the
    limits. The comparison with 0 is exact and documented. A crossed pair, a variable whose bounds cross or a
    row whose sides cross, is the known proof: the initial trial is skipped and the engine continues from the
    pair with two singleton trials, on every model type (N36). No type-level background check exists, and
    constraints added through `native_model()` are outside the guarantee (N23 a). Under a budget that stops
    before the two trials, the pair is reported `not_proven_minimal` with the stop's reason.
  - [x] **Status.** `unknown` after any run that called `solve()`, exceptions included (Q4 a).
    A run answered by the column-less precheck alone, or stopped before its first trial, a crossed pair's
    continuation included, leaves the data and the status as they were.
  - [x] **Stub tests.** `test/iis_by_deletion.cpp` in `mippp_test`, over a scripted stub model and a
    derived probe that asserts per-trial invariants. They cover every row of the classification, and a
    throw at trial k and during the restore. They also cover lazy and quadratic objectives, the column-less
    precheck and a crossed pair at zero budget, a crossed pair decided by two trials, and the forwarding cases
    of the time-limit audit. Done in `ad0105a`: 52 stub cases in `mippp_test`, green under gcc 15, gcc 14 and
    clang 18.
- [x] **4.2. `IisByDeletionTest` on Clp (WP8).** The suite in `test/test_suites/iis_by_deletion.hpp`, over the
  shared case bodies of `test/test_suites/iis_cases.hpp`, written as fixture member functions. It is
  registered in `all.hpp` and instantiated for Clp in the same pull request. `dumb_lp` is not instantiated for
  now (N24). `has_iis<clp_lp>` stays false. The column-less case asserts the N6 answer, and the crossed cases
  the two-trial answer of N36. The status case asserts `unknown` after a run that solved, and the previous
  status after a run decided by the column-less precheck. The suite obtains the IIS type through `auto` and
  never spells the snapshot's template arguments (N37 b). Done in `ac8696c` (29 cases, 25 running on Clp, the
  integer ones and the time-limit one skipping) and `19803cc` (the concept rows and the labels).
  Filter-only cases cover the budget sweep, a stop requested beforehand and a 0 s budget. Answers must not
  change after a later `remove_variable` (N8 a). The `concepts.md` row of the free function's concept says
  that zero members with `irreducible` means the background alone is infeasible.

Done when `IisByDeletionTest` passes on Clp in CI, including the case that re-solves the model to its
previous result.

## 5. Widen the free function

CI-tested backends come first. Each item adds modifiable row bounds and any other capability the model lacks,
instantiates the capability suites it newly satisfies, then `IisByDeletionTest`. No item adds `compute_iis()`,
an enumeration, a status reset or readable row bounds, which 3.2 to 3.4 already provide.

- [x] **5.1. `cbc_milp` (WP9).** Row bounds through `Cbc_setRowLower` and `Cbc_setRowUpper`, which `cbc_api`
  already binds. It is the first MIP under the free function, so it runs the integer-only cases. It also runs
  the first real time-limit forwarding, so it needs 1.5. Done in `5c4399c` to `851b6e9`, and `9aac8a1`.
  `IisByDeletionTest` runs 26 of its 29 cases on Cbc 2.10.11, as in CI, and 20 on the local devel build, which
  drops rows without terms. On both, `integers_summing_to_one_half` skips by name. That skip is not a
  capability gap: on 2.10 it hides a known wrong answer, `optimal` with x0 = 1.5 over integers, and on the
  devel build a search without end. N38 keeps it so: Cbc is experimental, and its limitations and bugs are
  documented, not addressed, so `cbc_milp` gets no integrality check (`e2d443d`). Two fixes came first: the
  devel build aborted in `Cbc_status` after an infeasible relaxation, and below 3.0 a re-solved MIP kept the
  previous incumbent, so each MIP solve there runs on a copy.
- [x] **5.2. `glpk_lp` and `glpk_milp` (WP11).** Modifiable row bounds through `glp_set_row_bnds`, which
  switches the bound type rather than storing an infinite value, over the readable ones of 3.4. GLPK had no
  time limit then, so limits acted between trials only, until both classes got one on 2026-10-02.
  `glpk_milp` trials are cold solves. Needs 1.2 and 3.4. Done in `f766e9c` to `5d1db49`.
  Crossed bounds or sides, which GLPK refuses with `GLP_EBOUND`, now solve
  `infeasible` rather than `failed`, and `glpk_milp` rounds the sides of integer columns and integral rows,
  which `glp_intopt` also refuses when fractional. 23 cases run on `glpk_lp` and 26 on `glpk_milp`. The final
  review found that an IEEE infinity on the side it cannot free, as -inf for an upper side, was written as a
  real side, so `glpk_lp` answered `optimal` and `glpk_milp` aborted in GLPK's presolver; every side written
  is now clamped to ±`DBL_MAX` first. A column added with bounds at `infinity()` is now typed by finiteness
  too, as the setters type it: typed double-bounded before, it solved to ±1.8e308 (4 of 1500 fuzzed LPs).
- [x] **5.3. `highs_lp`, `highs_milp` and `highs_qp` (WP10).** Modifiable row bounds in `highs_base` through
  `Highs_changeRowBounds`, over the readable ones of 3.4. N12 (b) is its own commit: on a ranged row,
  `get_constraint_sense` throws like Clp and Cbc, and `get_constraint_rhs` throws like Clp. The free function
  saves the Hessian of `highs_qp` through `qp_model`, not through backend code. It carries the
  removed-variable case on a remapping backend. Needs 1.1 and 3.4. Done in `35e6d20` to `e1b6124`, N12 (b) as
  `b05a026`. HiGHS's clock had counted every solve of a model, so each solve now zeroes it (`35e6d20`), and
  two native answers were fixed on the way (`5d05cb0`, `496f19e`). 26, 29 and 26 cases run on the three
  classes, on 1.10.0 and 1.15.1.
- [x] **5.4. `mosek_lp` and `mosek_milp` (WP12).** Modifiable row bounds through `MSK_chgconbound`, over the
  readable ones of 3.4, which read `MSK_getconbound`. The time limit needs no special case, since 1.5 makes
  the getter non-negative. Needs 1.5, 1.6 and 3.4. Done in `35b2a55` to `5e70f3d`; the last frees a column
  side set to `infinity()` under a raised `MSK_DPAR_DATA_TOL_BOUND_INF`. 24 cases run on `mosek_lp` and 27 on
  `mosek_milp`, on 11.0.14.
- [x] **5.5. `scip_milp` (WP13).** Bind `SCIPchgLhsLinear` and `SCIPchgRhsLinear`, checked across the validated
  range from 8.0.4, since 3.4 binds `SCIPgetLhsLinear` and `SCIPgetRhsLinear`. It rebases onto the SCIP rename
  of 3.2 and onto 3.4. Every trial is a cold solve, so its cases stay small. `scip_milp` keeps its `BINARY`
  columns, and nothing changes to conform to SCIP (N25). The N14 probe, relaxing a bound of a `BINARY` column,
  only characterizes the gap, which this item documents under "Notable current limitations" in
  `docs/solvers/index.md`. Done in `88f7d3b` to `146173a`: the variable-bound getters read the model's bounds,
  no longer the presolved ones, and a failed solve frees the transform. The N14 probe found that SCIP accepts
  the relaxed bound and fails the next solve, so the free function throws after restoring the model, which
  iis.md and index.md record. 26 cases ran on 8.0.4, 9.2.1 and 10.0.2 then; since `d3312b8` the suite has 30
  tests, of which 27 run on 9.2.1 and 3 skip: the two `remove_variable` cases and
  `integer_in_a_fractional_interval`, which skips from its data since N39. The SCIP fixture's
  `iis_case_skip_reason` answers for any integer column whose bounds each round inward to 0 or 1, as SCIP
  requires for `BINARY`, before the case is built (`5047a77`, narrowed to that rule by the review of wave 4). SCIP 10 was first checked from its sources
  only; the run on 10.0.2, the library of the PySCIPOpt 6.2.1 wheel, found that SCIP 10 renumbered
  `SCIP_STATUS`, which every SCIP 10 solve misread since 1.0, and the final review fixed it, so the suites pass
  there too. The monthly compatibility matrix covers the other 10.0 releases.
- [x] **5.6. `soplex_lp` last (WP14).** Under N11 (a), check the seven symbols of 7.1.3 and 8.0.2 that 3.4
  leaves in 7.1.1 and 7.1.2, or raise the floor. Then bind them, and add variable bounds, a readable objective
  and modifiable row bounds. `SoPlex_getRowBoundsReal`, bound by 3.4, reads row sides back, so the wrapper
  keeps no state. Needs 3.4, 5.5 and the SoPlex setter of 1.5. Done in `2bb9321` to `ab846b7`. The check found
  every symbol in 7.0.0 to 8.0.3, so the floor stays at 7.1.1. Contrary to this item, the wrapper keeps the
  column bounds, which SoPlex misreads after a scaled solve (iis.md, SoPlex). A fresh `soplex_lp` now
  minimizes, and `LpStatusTest` and `LpFuzzyTest` run on it. 24 cases run on 7.1.1, 7.1.3 and 8.0.3.

Done when `IisByDeletionTest` passes on each backend, in CI for Cbc, GLPK and HiGHS and locally for the
rest. Locally done on 2026-09-28, with the sanitized build on Cbc 2.10.11, and in CI on main since run
36472057802.

The release notes of v1.1.0 get the behaviour changes of this step: HiGHS's `set_time_limit` bounds each
solve; a fresh `soplex_lp` minimizes; GLPK solves crossed bounds and sides to `infeasible`; `glpk_milp` rounds
the sides of integer columns and integral rows; `cbc_milp` runs each MIP solve below Cbc 3.0 on a copy, so a
parameter set through `native_api()` on `native_model()` no longer reaches MIP solves and a native read of
their solution sees the unsolved model, and it refuses rows without terms on a devel build; the `scip_milp`
variable-bound getters read the model's bounds after a solve; and a MOSEK column side set to `infinity()` is
freed. Existing getters and setters change too: `cbc_milp::get_constraint_rhs` throws on a ranged row, rows
of `add_ranged_constraint` included, as `clp_lp`'s does, where it returned the upper side; on HiGHS,
`get_constraint_sense`, `get_constraint_rhs`, `get_constraint`, `set_constraint_rhs` and
`set_constraint_sense` throw on a ranged row (N12 b); and a Cbc MIP stopped by a limit no longer reports an
earlier solve's incumbent as available. Fixes worth a line: `soplex_lp::add_column` passed the nonzero count
as the column length, an IEEE infinite column side made a bounded SoPlex LP read unbounded, and GLPK typed a
column added with bounds at `infinity()` as double-bounded and an IEEE infinity on the wrong side of a row or
column as a real side, which answered `optimal`. Above all, `scip_milp` misread every status of SCIP 10, which
renumbered `SCIP_STATUS`, since 1.0: an optimum read `interrupted`.

Wave 5 adds to those notes: `compute_iis()` on `gurobi_lp`, `gurobi_milp`, `cplex_lp`, `cplex_milp`,
`xpress_lp`, `xpress_milp`, `copt_lp` and `copt_milp`, whose api objects now bind the IIS symbols as
mandatory; `set_constraint_lower_bound` and `set_constraint_upper_bound` on the CPLEX, Xpress and COPT
classes, which then satisfy `iis_by_deletion_model`, a side that would cross the other throwing
`std::invalid_argument` on CPLEX and Xpress; on CPLEX, `get_constraint_sense`, `get_constraint_rhs`,
`get_constraint`, `set_constraint_rhs` and `set_constraint_sense` throw `std::runtime_error` on a ranged row,
whose sense read `>=` before; `xpress_lp::solve()` postsolves a solve stopped by a limit, as
`xpress_milp::solve()` did, so that the rows stay addressable on Xpress 45.01; `compute_iis()` on `copt_milp`,
`cplex_milp` and `xpress_milp` detaches a registered candidate-solution callback for the call, and on
`copt_milp` runs the model's own solve up to an incumbent; and the transportation example reads a member's side through a visitor, since `is_a` of a tag a
status variant does not list does not compile.

N41 adds: `compute_iis_by_deletion` on `gurobi_lp` and `gurobi_milp`, which writes each row through its sense
and rhs during the run and writes both back; `iis_by_deletion_model` accepts a modifiable sense and rhs on a
model without ranged rows in place of modifiable row bounds.

N43 adds: that fallback also needs a readable sense, and holds only on a model without row-bound setters; the
filter reads a row's sense there and writes it only when the row changes side, so a one-sided row is relaxed
and restored through its rhs alone; the infeasibility page's repair code and the transportation example run
on `gurobi_lp` and `gurobi_milp`; and `dumb_lp` runs `IisByDeletionTest`, in CI on Clp.

The coherence pass adds: `clp_lp`, `glpk_lp` and `glpk_milp` gain `set_time_limit` and `get_time_limit`, so
every model class satisfies `has_time_limit`, a negative or NaN limit throwing `solver_error` there; Clp's
limit counts the process's user CPU time, GLPK's the wall clock; the status variant of `clp_lp` gains
`limit_reached` and `time_limit`, each carrying Clp's primal feasibility as its solution flag, and that of
`glpk_lp` gains `time_limit`, where a GLPK time-limit stop read `limit_reached`; `glpk_milp` gives GLPK one
millisecond more than the limit set, since GLPK's search stops a millisecond early; `xpress_milp::compute_iis()`
reports a ranged row its routine flags on one side as a plain `member`, its row status gaining that tag, and
`compute_iis()` on both Xpress classes answers a column whose bounds admit no value from those bounds, where
it threw `solver_error`, throwing still on a column of a kind MIP++ never creates; and, as N43 above says,
the filter's writer on Gurobi reads a row's sense and writes its rhs.

## 6. Native routines

Under Q1 (b), these items add `compute_iis()` members, which never run the deletion filter. The step needs
2.5 only, so it can run beside steps 4 and 5. HiGHS (6.1) goes first and creates the native suite `IisTest`,
so that it runs in CI before any backend tested only locally. Each `compute_iis()` takes no argument (N9). The
model's time limit bounds each call as a fresh budget (N29 a): four routines already stop under it, and HiGHS
through a per-call copy into `iis_time_limit`. Other model limits may also stop some routines, and the docs
list them per backend. It sets the status to `unknown` before its first native call (N15), through
`reset_status()` now that 3.3 has landed. It restores every parameter it sets through its own small RAII
helper. It runs no column-less precheck (N28 a), and gives a reason only on a stop attributed to the time
limit (N27 a).

- [x] **6.1. HiGHS, first (WP15).** `Highs_getIis` in `HIGHS_OPTIONAL_FUNCTIONS`, used on `highs_lp` and
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
  model and resets the run clock (measured). Done in `70384a9`; the probe of 2026-09-28 on 1.15.1 is recorded
  in iis.md (`2e23f9a`): status and held solution agree, so leaving the status untouched is a follow-up, and
  `compute_iis()` still resets it.
  - [x] **`IisTest`.** The suite in `test/test_suites/iis.hpp` shares the case bodies of 4.2. The fixtures
    `highs_lp_iis_test` and `highs_qp_iis_test` skip below 1.14.0 on `library_version()`, and
    `TEST(HiGHS_lp, compute_iis_below_native_floor_throws)` expects the throw there. Done in `ac8696c`, with
    four HiGHS-only cases on the options, the zero budget, the status and the Hessian; locally 28 native cases
    pass on 1.15.1 and 16 skip for capabilities, and the throw is exercised on 1.10.0. Since wave 3, 38 run
    and 6 skip, 19 and 3 on each of `highs_lp` and `highs_qp`, since the ranged cases build through modifiable
    row bounds.
  - [x] **Both paths.** After 5.3, `both_paths_find_valid_iis` calls the free function directly on models that
    meet both concepts, and the oracle validates both answers. Done with 5.3 in wave 3: it runs on `highs_lp`
    and `highs_qp` and passes on 1.15.1.
- [x] **6.2. Confirm each routine at its range floor.** `COPT_ComputeIIS` in COPT 7.2, `XPRS_IISOPS` in Xpress
  45.1, and the `IIS*Force` attributes in Gurobi 10. Probe each routine on a feasible model, where Gurobi
  gives error 10015, CPLEX conflict status 30 and COPT code 3. The time limit was probed on 2026-09-27 on
  Gurobi 11.0.3 to 13.0.2, CPLEX 22.1.1 and 22.1.2, Xpress 45.01 and 47.01 and COPT 8.0.5 (N29 evidence). At
  Gurobi 10 and COPT 7.2, check that the routine stops under the model's time limit and how it reports the
  stop. At 45.1, run the MIP case that aborted Xpress 45.01 under the default `IISOPS`, with the wrapper's
  bits. Record whether the held solution survives the call and the status agrees with it. Where it does, that
  routine leaves the status untouched (N15). Done on 2026-09-29, as far as the local installs allow: the COPT
  7.2.5 header declares every IIS symbol and the library loads and binds them, but the local license refuses
  its environment, so nothing ran on 7.2; Gurobi 10 is not installed, and its 10.0 documentation lists
  `GRBcomputeIIS` and the six `IIS*Force` attributes; Xpress 45.01.01 ran the whole suite, market split
  included, under the wrapper's bits. The feasible-model answers (10015, 30, code 3, and Xpress `p_status` 1
  with `NUMIIS` 0) and the status after each call are recorded in iis.md: no wrapped routine passes N15,
  Gurobi's only on models without special constraints, so every `compute_iis()` resets the status.
- [x] **6.3. Gurobi (WP16).** `GRBcomputeIIS`, with `IISSOSForce`, `IISQConstrForce` and `IISGenConstrForce`
  set to 1 and restored. Inequality rows get their side from the sense, and equality rows are `member`.
  `IISMinimal` = 0 gives `not_proven_minimal` (N1 a). A forced-only answer is irreducible with zero members.
  Error 10005 on `IISMinimal` after a stop means no answer, never an empty IIS. Needs 1.3. Gurobi gets no
  modifiable row bounds, since its ranges add a slack column (N30), and reads its row bounds through 3.4.
  Done in `70374bf` and `4b04d37`: the force attributes at 1 behave as planned (measured), a stop is
  attributed to the time limit by the measured time, a binary's bounds are never members (Gurobi's rule), and
  the iteration limit of `gurobi_lp` stops the routine with no answer. Validated on 11.0.3, 12.0.1 and 13.0.2.
- [x] **6.4. CPLEX (WP16).** `CPXrefineconflictext` and `CPXgetconflictext`, with one group per row and per
  bound side, never the deprecated `CPXrefineconflict`. Indicators stay outside the groups. Status 31 is
  `irreducible` and 30 `feasible`. Abort statuses 32 to 39 are `undetermined` with no member, and status 33 a
  time-limit stop: the "possible" flags of a stop were meant to give `member_*` under `not_proven_minimal`,
  but the probes p10 and p13 showed that they prove nothing (a feasible model stopped by the limit gets the
  same flags, and a node-limit stop's exclusions re-solve feasible), see "Native routines" in iis.md. Rows
  get their sides as on Gurobi. Each call passes a fresh equal preference to defeat the resume-after-abort
  defect, from a counter that is a member of the model and moves with it (`48a59cd`). A registered
  candidate-solution callback is detached for the call, since the refiner returns 1811 with one registered
  (`f4e844e`, pinned in `48a59cd`); a 34 stop outlives its limit until a bound or a side is written, which
  the user page states and the iteration-limit test pins. Needs 1.3.
  Modifiable row bounds, over the readable ones of 3.4, are done as native 'R' rows;
  crossed sides are unrepresentable there and throw `std::invalid_argument`.
- [x] **6.5. Xpress (WP16).** The `IISOPS` integrality and special-constraint bits are set before
  `XPRSiisfirst` and restored after. `I` entries are dropped. Rows map `L` to `member_upper`, `G` to
  `member_lower` and `E` to `member_both`, confirmed at 45.01 and 47.01: `E` comes only where integrality
  needs both sides, and a fixed column's `F` maps to `member_both`. `IISSOLSTATUS` gives the outcome, with
  `p_status` 3 and `NUMIIS` 0 read as a stop with no answer, `time_limit` only when the measured time plus a
  20 ms slack reached the limit, since `p_status` 3 also follows an interrupt or a native iteration limit. A
  column whose bounds cross or hold no integer makes the routine refuse the search: the wrapper threw
  `solver_error` there until `5c2192f`, which answers from that column's bounds, and since `916db02`
  `xpress_milp` reports a ranged row flagged on one side as a plain `member`. A column listed once per bound,
  `U` then `L`, merges into `member_both`, and `xpress_milp` detaches a registered candidate-solution
  callback for the call, which the routine's MIP solves ran. Done in `cd3b3bd` and `a12f76d`; the modifiable
  row bounds, once Xpress was confirmed to store a ranged row natively
  (type `R`, rhs the upper side, a non-negative range), in `35eb0af`, crossed sides throwing
  `std::invalid_argument`. The answer's tables are sized after the routine returns, and `xpress_lp::solve()`
  postsolves after a stopped solve (`3acad2c`), since 45.01 leaves the LP presolved, where the counts, the row
  accessors and the additions see the presolved rows. Validated on 45.01.01 and 47.01.01.
- [x] **6.6. COPT (WP16).** `COPT_ComputeIIS` and the four `Get*IIS` calls. Per-side flags on `copt_lp`, and
  equality rows as `member` on `copt_milp`. `IsMinIIS` gives the outcome, and `HasIIS` 0 means no answer.
  Flagged counts that disagree with `IISRows` and `IISCols` give `undetermined` (inferred), a disagreement
  measured on a time-limited MIP IIS on 8.0.5. The binary probe ran: 6 of 176 random answers re-solved
  feasible with only the flagged sides and none with the flagged columns whole, and 8.0.5 flags one bound of
  a two-bounded continuous column where both are needed, so `copt_milp` reports two-sided rows and
  two-bounded columns, continuous too, as `member`. The call resets the solver first and, on `copt_milp`,
  runs the model's own callback-free solve up to an incumbent, since COPT caches its answers and flags a
  whole unsolved feasible MIP; a code 3 on `copt_lp` is confirmed by a solve under the remaining budget; a
  registered callback is detached for the call, a binary column moved outside [0, 1] is a member
  through the bound beyond its domain, and a column-less model is answered from its constant rows, since
  the routine leaks memory there. `copt_lp` has its time limit since wave 1 (N35 a), and both classes
  bound the whole call with it. Done in `f73ac7a` and `8baba3f`; the modifiable row bounds, COPT's rows
  confirmed two-sided natively, in `c2af30c`. Validated on 8.0.5 only, 7.2.5 refusing the local license.

Done when `IisTest` passes in CI and CI shows HiGHS below and above the native floor. Locally, the suite
passes on each commercial backend at its range floor and latest release, with indicators and a removed
variable on Gurobi and CPLEX. CI has shown HiGHS on both sides of the floor since wave 3. Locally, on
2026-09-29, `IisTest` passes on Gurobi 11.0.3, 12.0.1 and 13.0.2, CPLEX 22.1.1 and 22.1.2, Xpress 45.01.01
and 47.01.01 and COPT 8.0.5, with indicators and a removed variable on Gurobi and CPLEX; Gurobi 10 and CPLEX
22.1.0 are not installed, and COPT 7.2.5 refuses the local license.

## 7. Documentation

The user pages come once the CI backends run both paths, after 4.2, 5.1 to 5.3 and 6.1. Rows and labels that a
package adds come with that package.

- [x] **7.1. Reading an IIS.** `docs/solving/infeasibility.md` covers the `has_iis` gate and the free
  function, the consumer loop, the tags of each path, outcomes and reasons, and `*_lp` against `*_milp`. It
  states the status afterwards: `unknown` after a run that solved, and after every native call for now. It
  explains N8. A per-model table shows `has_iis`, the free function's concept and the HiGHS floor, and later
  packages keep it current. It restates for IIS the warning of `docs/solvers/index.md` on the native handles
  (N23): background added natively is outside the guarantee, and the free function's guard and the entity
  enumeration do not see native changes. It links the SCIP gap of 5.5 (N25). Its paragraph on the time bounds
  of native calls states per backend that the budget is per call, which other model limits also stop the
  routine, and that a stop may return late or with no answer (N29 a). It states that the model's time limit
  bounds a native `copt_lp` call as on `copt_milp`, through fix 10 (N35 a). It replaces the pull request's
  `docs/iis.md`. It obtains the IIS type through `model_iis_t<T>` or `auto` and never spells the snapshot's
  template arguments, and it tests membership with `is_a<iis_status::member>` (N37 b, d). Done in `cf09c4f`,
  with its code in `1c6256a` and its links in `4c6034c`; the review added `8a3e694` and `b893392`, with the
  test `aeb1de7`, which pins on 1.15.1 that the iteration limit, the only other limit of `highs_lp` and
  `highs_qp`, does not stop the routine. A reader's review then added the header's includes as a compiled
  snippet (`ccf9c94`); a section "Repairing the model", which relaxes the members of each IIS until the model
  is feasible, on the workshop and on a variant with two disjoint conflicts (`5862a31`); what Gurobi, CPLEX,
  Xpress and COPT users can do until their native routines are bound (`f57fa63`); `rerun_on_members`, which
  narrows a partial answer's next run to its members (`31c8a1b`); and smaller answers: the HiGHS library
  override in the below-floor error, the 1.14 floor on `coming-from.md`, names for printing member handles,
  and the report's output stream (`b95fc8e`). What names a commercial routine came with wave 5 (`b5014d1`,
  `12bcae7`, `b20e204`, `e507a95`): the rows of the per-model table, a paragraph per backend on native time
  bounds and the other limits that stop a routine, the plain `member` of the equality rows of Gurobi and
  CPLEX and of the two-sided rows and columns with two finite bounds, continuous too, of a `copt_milp` that
  COPT solves as a MIP, the visitor that reads a side on any path, and COPT's confirming solve; the
  concepts rows, the limitation bullets and the feature tables followed. The final review of wave 5 added
  the detached callbacks of `cplex_milp` and `xpress_milp`, the iteration-limit stop CPLEX keeps, the
  repair section's requirements and Gurobi's way to relax a row, and the headers of the filter concepts.
- [x] **7.2. The deletion filter as an algorithm.** A page under Algorithms in `zensical.toml`, next to column
  generation. It covers the engine's oracle contract, monotonicity, limits and reasons. It then covers the
  free function's requirements, what it saves and never touches, and the native-handle warning. It warns that
  one trial can overrun the budget on models without `has_time_limit`. Done in `8013cb4`, with its code in
  `bce799e` and its links in `0297e02`; the review corrected it in `8a3e694`, and its tests exposed a false
  gcc 14 warning in the engine, fixed in `b4e90d5`. Its table of warm and cold trials rests on the
  measurements of wave 3, which no test pins, and leaves out `clp_lp`, `glpk_milp` and `cbc_milp` without
  integer columns, not measured.
- [x] **7.3. Concepts and feature tables.** `concepts.md` rows for every new concept. The labels
  `("IisTest", "IIS, native")` and `("IisByDeletionTest", "IIS, deletion filter")`, next to those of
  `EnumerableEntitiesTest` and `ModifiableConstraintBoundsTest`. `tested_features_table.py` gets the fix that
  makes `highs_qp` rows appear, then the tables are regenerated. The labels came in wave 2, in `19803cc`. Done
  in `569703a`, which also measures the drawn width, since a longer label cut the last column, and in
  `24004ff`, which shortens the IIS rows and adds `deletion_oracle`; `f5b8a9a` restores the sentence on
  `irreducible` with zero members. Regenerated again in wave 5 (`c8e562b`), the IIS rows ticking Gurobi,
  Xpress and COPT.
- [x] **7.4. README and `coming-from.md`.** The README roadmap row, and the list of missing features in
  `docs/getting-started/coming-from.md`. Done in `808e318` and `1817a90`, with the feature lists of the home
  pages in `87e728b` and `5b23b3b`.

Two additions beyond the plan, agreed with the maintainer before the wave. The code of the pages is compiled
and tested (`1c6256a`, `bce799e`): each page has one source in `test/doc_snippets/`, a core source of
`mippp_test`, whose marked sections the page includes through `pymdownx.snippets`, and whose tests run them on
HiGHS and Clp and compare the output the page shows. Since `b134b7b`, CI checks the markers and builds the
docs on every pull request. `examples/infeasible_transportation` (`687ed3e`, `bfa4684`, `c331928`) diagnoses
an infeasible plan through both paths, and with its alias changed runs on each of the 11 classes of the
deletion filter.

Done when `zensical build --clean` passes and every snippet compiles. Done on 2026-09-29: the build reports no
issue, and the snippets compile and pass under gcc 15, gcc 14 and clang 18 on HiGHS 1.10.0, and under gcc 15
on 1.15.1 too.

At the version bump of v1.1.0, the example must require `mippp/1.1.0` and `find_package(mippp 1.1 ...)`, with
the other examples: a package built from the v1.0.0 tag has no IIS header, so the example fails there on a
missing header rather than on the version.

## Later

The deferred items of iis.md, in no fixed order. WP3 writes a note for each of the pull request's deferred
ideas, pointing into the archive tag of `a1a9f11`.

- **Certificates.** Dual rays, and the ray seeds they allow.
- **Native extras.** Elastic relaxation, forcing and preferences, several IISs per model on Xpress, SCIP 10's
  IIS finder, and cancelling a native call through `compute_iis(std::stop_token)` (N31 b). An explicit
  duration for a native call, if users ask, is a possibility that is not ruled (N29 b).
- **Protected sides and candidate order.** The first extension after the first version (N37 f): native on
  Gurobi and CPLEX, and on the deletion path a filter over the enumerated sides behind an additive overload of
  `compute_iis_by_deletion`. A caller for the dormant batching stays a filter refinement.
- **Integrality as a candidate.** Relaxing an integer or binary variable to continuous rather than its bounds,
  the maintainer's intuition given with N25. It is outside the LP-centred first version. It comes as a
  per-variable table of its own in the snapshot, never as new tags, needs a variable-type getter and setter on
  `milp_model`, and leaves the engine untouched (N37 a, c).
- **Special constraints as members.** SOS and indicator constraints once they have handles, as further per-kind
  tables in the snapshot, on the native path first (N37 a).
- **A user model in CI.** Done on 2026-10-02 (N43), though not as N24 put it: `dumb_lp` keeps one-sided rows
  and no row-bound setters, and runs `IisByDeletionTest` through the sense-and-rhs fallback of N41.
