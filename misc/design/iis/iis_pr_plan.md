# IIS pull request #3: adaptation plan

Plan for turning pull request #3 (branch `astra/iis-support`, HEAD `a1a9f11`) into the feature designed in
[iis.md](iis.md), in the order of [iis_todo.md](iis_todo.md). It rests on an inventory of the pull request
dated 2026-09-27, whose verdicts it follows unless a reason is given. The pull request adds 7,433 lines over
48 files in 21 commits, two of them merges of main. It still has the shape iis.md lists as "Replaced": an
index-based utility, no `has_iis`, a separate test binary. Kept: the deletion engine, now public with its
batching dormant, its limit semantics, the exact oracle with the published HiGHS vectors, the MOSEK guard and
slot selection, the getter half of the MOSEK time-limit mapping, the SoPlex move fix, the case list, and the
ideas that also work in place. The maintainer ruled on the decisions below on 2026-09-27, as recorded in
iis.md under [Rulings of 2026-09-27](iis.md#rulings-of-2026-09-27). Of the questions these rulings raise, N19
onward, the maintainer accepted the recommendations of N19 to N22 and N26 later the same day, then ruled on
N23 to N25, N27, N28 and N30 to N33, then on N29, N34 and the wording of N9, and last on N35, a time limit on
`copt_lp` split from N29. That last message also confirmed the readings of the short rulings, leaving the
choice on N4 to the assistant, and no question of the plan is pending. Wave 3 raised N38 and N39, open in
iis.md. Line numbers in **Port** and **Drop** bullets are
at `a1a9f11`. Elsewhere they are on main at `f5e833f` unless marked "at `a1a9f11`".

## Decisions

Q1 to Q6 are the open questions 1 to 6 of the 2026-09-22 iis.md, and N0 to N18 are this plan's. The maintainer
ruled on all of them on 2026-09-27. iis.md's [Rulings of 2026-09-27](iis.md#rulings-of-2026-09-27) names each
chosen option in words, and its design sections apply them. The options themselves are in the pre-ruling
version of this plan, kept as [iis_pr_plan_pre_rulings.md](iis_pr_plan_pre_rulings.md). The work packages
below carry the consequences. The questions N19 to N33 arise from these rulings, N34 from the time-limit
probes of 2026-09-27, and N35 from the ruling of N29. All were
[ruled later the same day](#ruled-later-on-2026-09-27), N35 last, so [Pending](#pending) is empty.

### Readings confirmed on 2026-09-27

The maintainer confirmed on 2026-09-27 every reading that this section listed for confirmation, now marked
*Confirmed reading* in iis.md's [Rulings of 2026-09-27](iis.md#rulings-of-2026-09-27), with the three
additions below, except N4's, where they left the choice to the assistant.

- **Q6, readable row bounds on every model.** The maintainer wrote "Yes, this would imply implementing
  get_constraints_bounds() for all solvers. For those which does not suppport nativelly ranged constraints,
  like Gurobi, the returned pair would contain rhs and the solver infinity, the branch is in the model's
  method, not on the IIS side." The reading becomes: ranged rows need no branch in the IIS code, and
  `if constexpr` in the free function tests only `qp_model`, `has_time_limit` and the capabilities that can
  carry background constraints. `has_readable_constraint_bounds` comes to every model in
  [WP6c](#wp6c-readable-row-bounds-on-every-model), where a model without native ranged rows derives its
  bounds from the sense and the rhs.
- **N1, a design guideline.** The maintainer wrote "Yes, if needed, use the pattern variant + tag hierachy to
  keep details when they are available while offering a unfied generalisation." When a backend has details
  that the unified tags drop, such as a native "possible member" flag or a side the unified set cannot name,
  it may keep them through the variant and tag hierarchy: refinement tags deriving from the unified ones,
  listed only in that backend's status variant, so that `is_a` on a unified tag still gives the
  generalization. This applies only if needed, and no work is planned for it.
- **N3, "for the moment".** The maintainer wrote "N3 (b). Yes, for the moment." Batching stays dormant for the
  moment.
- **N4, a choice left to the assistant.** The maintainer wrote "N4 (A). I am not convinced that is the best
  option, your call." They were not convinced and left the call, so it can be revisited. Decision recorded by
  the assistant at the maintainer's request on 2026-09-27: keep (A). The only aberrant no-limit value found
  by the audit of 2026-09-27 is MOSEK's negative one, -1 s on a fresh task. MOSEK's getter reads a
  negative value as `std::numeric_limits<double>::infinity()`. That value round-trips, since MOSEK 11.0
  accepts +inf (measured) and every setter accepts +inf under N32 (a). All other backends stay transparent:
  Gurobi 1e100, COPT 1e20, SCIP 1e20, CPLEX 1e75 and SoPlex 1e100 after their clamps, and HiGHS, Cbc and
  Xpress +inf. The earlier reading, "the solver's infinity is the time parameter's own no-limit value", is
  dropped as moot. (A) is kept over (B) because the guarantee lives on the `has_time_limit` capability. Any
  generic code, the free function's or a user's, can then forward `std::min(remaining, get_time_limit())`
  without a MOSEK special case, whereas (B) would put that special case in every consumer.

### Ruled later on 2026-09-27

Later on 2026-09-27 the maintainer wrote "Ok with your recommendations for N19, N20, N21, N22 and N26." A
second message the same day ruled on N23 to N25, N27, N28 and N30 to N33, and asked two questions on N29,
which was then revised. A third wrote "N29. Ok for (a). N34. (a). N9. (i), match the N29 (a) ruling." Its
"(a)" for N29 answers the bound only, so the revised entry's second choice, a time limit on `copt_lp`, became
N35. A fourth message ruled N35 and confirmed the readings [above](#readings-confirmed-on-2026-09-27). Each
entry starts with the ruling, quoting the maintainer where they wrote more than a letter, and keeps its
options and evidence. [iis.md](iis.md#rulings-of-2026-09-27) records each ruling. Line numbers are on main at
`f5e833f` unless marked.

**N9, ruled.** Wording (i), which replaces the reading this plan had listed for confirmation: "A native
`compute_iis()` takes no `iis_limits`. The model's time limit bounds each call as a fresh budget, on HiGHS
through a per-call copy into `iis_time_limit`. Other model limits may also stop it, and the docs list them per
backend." The rejected wording (ii) matched N29 (d). The earlier reading, that native calls honor only the
model's own time limit, was wrong on its "only". CPLEX's refiner also stops on the iteration limit of
`cplex_lp` and the node limit of `cplex_milp` (measured, statuses 34 and 35), and on the tree memory limit
(documented). Gurobi documents `SoftMemLimit`, which `set_memory_limit` writes (`gurobi_base.hpp:622-624`),
and `WorkLimit` as IIS termination parameters. HiGHS lifts `simplex_iteration_limit` during the call (read in
the 1.15.1 sources). "Bounds" also means "stops searching", not "returns within": HiGHS 1.15.1 returned 15.5 s
late, and a stop can leave no answer on every backend. The budget is per call, whereas the free function
forwards `std::min(remaining, saved)` from one deadline.

- **N19. Headers, and the names of the engine, the free function, the snapshot template and the suites.**
  Ruled: the recommendation. The IIS vocabulary is not open: `iis_status`, `iis_outcome`, `iis_reason`,
  `lp_iis`, `model_iis_t` and `has_iis` come with Q2, `iis_limits` with Q5, and
  `iis_reason::inconclusive_trial` with N2. (a) Everything in `include/mippp/utility/`, namespace `mippp`,
  each new name prefixed with `deletion_` or `iis_`, except the verb `compute_iis_by_deletion`. The engine is
  `deletion_filter`, `deletion_verdict`, `deletion_oracle` and `deletion_filter_result` in
  `utility/deletion_filter.hpp`. The ruled `iis_limits`, `iis_outcome` and `iis_reason` go in a std-only
  `utility/iis_outcome.hpp`, which `model_concepts.hpp` and the engine include. The free function is
  `compute_iis_by_deletion` with the concept `iis_by_deletion_model`, in `utility/iis_by_deletion.hpp`. The
  snapshot is `iis_snapshot<Variable, Constraint, VariableStatus, ConstraintStatus>`. The suites are
  `IisTest`, labelled "IIS, native", and `IisByDeletionTest`, labelled "IIS, deletion filter". (b) A new
  `include/mippp/algorithm/` directory, as the pull request has. (c) A sub-namespace `mippp::iis` for the
  engine only, with short names, like `colgen`. The ruled `iis_*` types stay in `mippp`, and short names such
  as `result` in a namespace users open go against the Names rule of iis.md. (d) As (a), but the free function
  is `compute_iis(model, limits)`, next to the member, which goes against the spirit of Q1 (b)'s "different
  paths". Two sub-points: (e) `iis_outcome` and `iis_reason` as `enum class`, or as tag namespaces with a
  variant like `status`. (f) Whether `deletion_filter_result` carries the number of oracle calls. Recommended:
  (a), `enum class` for (e), and no call count for (f). Evidence: `column_manager`, main's one public
  algorithm over capability concepts, lives in `utility/` (`utility/column_manager.hpp:20`), with its page
  under `docs/algorithms/` (`zensical.toml:33-36`). `colgen` is a sub-namespace only because its vocabulary is
  generic (`utility/column_generation.hpp:20-23`), and a `mippp::iis` next to the `mippp::iis_status` tags
  would read badly. With (d), `compute_iis(model)` and `model.compute_iis()` would read as one operation
  although Q1 (b) made them two paths. Outcome and reason are closed, flat sets without payload, like the
  public `enum class constraint_sense` (`linear_constraint.hpp:11`), whereas `iis_status` needs tags because
  `member_*` refine `member`. The idea `run-statistics` was rejected, and an oracle can count its own calls.
  The suite name `IisByDeletionTest` is preferred to `DeletionFilterTest`, which reads like the solver-free
  engine tests of `test/deletion_filter.cpp`. The pull request's names are
  `mippp::iis::{result, options, feasibility, termination}` (`algorithm/deletion_filter.hpp:25-62`,
  `algorithm/iis_limits.hpp:15-34` at `a1a9f11`). The engine header of the sketch uses only the standard
  library and the three IIS types. Blocked pull request #3 (WP4, WP5), WP7, the suite names, the
  features-table labels and WP17.
- **N20. Names of the entity enumeration.** Ruled: the recommendation. Members: (a) `variables()` and
  `constraints()`, (b) `get_variables()` and `get_constraints()`, (c) `all_variables()` and
  `all_constraints()`, as in JuMP, (d) `variable_handles()` and `constraint_handles()`. Concepts: (a)
  `has_enumerable_variables` and `has_enumerable_constraints`, (b) `has_variable_enumeration` and
  `has_constraint_enumeration`, (c) one `has_enumerable_entities`. Suite: (a) `EnumerableEntitiesTest`,
  labelled "Enumerate variables and constraints", (b) one suite per concept, (c) `EntityEnumerationTest`,
  labelled "Entity enumeration". Recommended: members (a), concepts (a), suite (a). `variables()` pairs with
  `num_variables()` and with the vocabulary of `variables_range`. Its one cost is renaming SCIP's two
  protected data members, about 32 mechanical lines. Members (c) avoid that rename. Concepts (a) parallel
  `has_named_variables` and `has_named_constraints`, and a basis dump needs only variables, so one concept per
  entity kind beats (c). Evidence: `scip_milp.hpp:41-42` declare protected members `variables` and
  `constraints`, and GCC 15 rejects the class once the using-declarations are added ("conflicts with a
  previous declaration"). No other header declares either name as a member. The parameters named `variables`
  in `remove_variables` compile under `-Wshadow`. Blocked WP6a, and through it WP7 and WP13.
- **N21. Contract of the entity enumeration.** Ruled: the recommendation. (a) A snapshot of the call in
  increasing id order, as a random-access sized range: the lazy O(1) `entity_range` where nothing is ever
  removed, and a `std::vector` on `clp_lp`, `dumb_lp` and the three remapping bases, sorted only after a
  perforating removal. The range holds no reference to the model, so it survives later additions, removals and
  a move. (b) Lazy views that reference the model, in native order, invalidated by any addition or removal.
  (c) (a) without the order guarantee. Two sub-points: the concepts take `<T, M = T>`, or a single parameter,
  and they require `std::ranges::sized_range` or not. Recommended: (a), `<T, M = T>`, sized. The enumeration
  stays out of `lp_model` until 2.0, so that user-defined models keep satisfying it. Evidence: a prototype
  compiled on all 19 model classes and `dumb_lp` under the project's full GCC 15 warning set and under clang
  18. A simulation copying the HiGHS and CPLEX removal code ran 600 runs of 400 random operations under ASan
  and UBSan with no mismatch. The id bound exceeded the live count by up to 124. Remapping backends recycle
  freed handles on single additions and append on range additions (`remapping_model_base.hpp:55-82`), so
  native order differs from id order. A lazy view capturing the native count reads `_handle_ids_map` past its
  shrunk size after a removal (`highs_base.hpp:233`), which is undefined behavior.
  `docs/reference/concepts.md:64-68` keeps whole-model concepts single-parameter, while the 2026-09-16 ruling
  gives `<T, M = T>` to concepts a handle might satisfy, and Q3 used that form for
  `has_modifiable_constraint_bounds`. No callback handle satisfies the concepts today, and inside an Xpress
  callback `XPRS_COLS` counts presolved columns (`xpress_milp.hpp:73-78`). The last element of a sorted
  snapshot gives the id bound that sizes the IIS snapshot. Blocked WP6a, and the snapshot sizing of WP7.
- **N22. How the free function makes `get_status()` report `unknown`.** Ruled: the recommendation. (A) A
  detail access key that the 19 models befriend, calling a private `_reset_status() noexcept`. (B1) A public
  `void reset_status() noexcept` with a concept `has_status_reset<T>`, required by the free function's
  concept. (B2) The same, applied through `if constexpr`, so a model without it keeps the last trial's status.
  (C) Every mutator of model data makes the status `unknown`, library-wide, with the flag moved into the seven
  shared bases. (D) Re-solve after the restore. (E) Nothing. Recommended: (B1). It is the only option that
  makes Q4 hold for every model the function accepts, `dumb_lp` included, and it gives a user-written
  algorithm the same tool as the library's. It costs one line per model and in `dumb_lp`, one concept, one
  shared `LpModelTest` case (`reset_status_reports_unknown`: solve, reset, read `unknown`, re-solve to the
  same status) and one `concepts.md` row. MOSEK's slot cache from WP2 resets inside it, wired by whichever of
  WP2 and WP6b lands second, and native `compute_iis()` members may call it under N15 once WP6b has landed.
  (C) can be ruled later on its own, and (B1) stays harmless under it. (A) works, but a user model gets no
  reset unless it befriends a detail name, and a `requires` check cannot see it. (D) costs a full solve and
  contradicts Q4. Evidence: all 19 private `_status` members are written only by `solve()` and
  `refine_lp_status()` (for example `clp_lp.hpp:533, 551-566`, `highs_lp.hpp:56, 95-102`,
  `cbc_milp.hpp:481, 531-539`). A throwing `solve()` keeps the previous status on HiGHS, MOSEK, SCIP, COPT,
  CPLEX, Gurobi and Xpress. `dumb_lp.hpp:83-86` also caches its solution. About 150 mutators live in shared
  bases whose leaves hold differently typed variants. The access key was compiled and run on a library model
  and a derived probe. Blocked WP6b, WP7 and the Q4 wording of WP17. Native members write their own `_status`
  (N15), so WP15 and WP16 never depended on it.
- **N23. Background added through `native_model()`, and the zero-solve claim of N7 (a2).** Ruled: (a),
  ignoring `has_native_handles`, plus a documentation warning. On 2026-09-28, N36 dropped the zero-solve claim
  and its background check; the warning stands. The maintainer wrote "Add to the documentation,
  if not already present, a warning that states that some features are invalidated if user modifies the model
  through the native handle." The background check ignores `has_native_handles`, and constraints added through
  `native_model()` are outside the guarantee. The general warning is written in `docs/solvers/index.md`,
  committed with `25b4530`, at the end of the "Solver-specific parameters" bullet: handles and `native_id()` may designate
  the wrong entity, counts can disagree on SCIP and Cbc, `get_status()` keeps reporting the last `solve()`
  made through MIP++, and features built on this bookkeeping lose their guarantees. WP17's IIS page restates
  it for IIS: background added natively is outside the guarantee, and the free function's guard and the entity
  enumeration do not see native changes. (a) Ignore `has_native_handles`, and document that constraints added
  through `native_model()` are outside the guarantee. (b) Treat `has_native_handles` as possible background,
  which disables the claim on every library model. Recommended: (a). Every backend has native handles, so
  under (b) the claim of N7 (a2) would never apply, and the escape hatch is already outside the portable
  scope. Evidence: `model_concepts.hpp:727-734`, and the Scope bullet of iis.md, where SOS exist only through
  `native_model()`. Blocked WP7's background check and WP17.
- **N24. A user-defined model run in CI by the free function.** Ruled: not a priority, so `IisByDeletionTest`
  gets no `dumb_lp` instantiation now, and the item moves to Later. The maintainer wrote "Not a priority, CI
  always have at least one open source solver (Clp, CBC or HiGHS)." (a) Extend `test/dumb_lp.hpp` with
  two-sided rows and readable and modifiable row bounds, and instantiate `IisByDeletionTest` for it. It
  already has the enumeration and the status reset from WP6a and WP6b. (b) A separate test model. (c) Stub
  models only. Recommended: (a). `dumb_lp` is the reference user model, runs in CI, and already decides
  column-less models arithmetically. `LpFuzzyTest` could then mutate row bounds too. Evidence:
  `test/dumb_lp.hpp:63-68` (row data), `:422-437` (no columns), `:518-535` (capability `static_assert`s), and
  `test/solvers/dumb.cpp`. Blocked WP8's coverage of user models.
- **N25. SCIP creates [0, 1] integer columns as `BINARY`.** Ruled: (c) in effect, keeping `BINARY` and
  documenting the gap of `scip_milp`. The maintainer wrote "For binary and integer variables, my intuition is
  that relaxing them to continuous is more interesting than relaxing their bounds, but I am not used to mixed
  integer IIS. Either waay, the current IIS design being centered on linear programs, the gap should be
  documented instead of modifying the algorithm to conform to SCIP." Neither the algorithm nor `scip_milp`
  changes to conform to SCIP. The N14 probe is still useful, only to characterize the gap for the
  documentation. The intuition becomes the Deferred item "Integrality as a candidate" of iis.md: relaxing an
  integer or binary variable to continuous may explain a MIP better than relaxing its bounds, and Xpress's
  default already lists integrality restrictions as removable `I` members. It is outside the LP-centred first
  version. (a) N14's probe: relax one bound of a `BINARY` column, and keep such bounds as candidates if the
  single side stays satisfiable. (b) If the probe fails, create binary and [0, 1] integer columns, then call
  `SCIPchgVarType` to `SCIP_VARTYPE_INTEGER`, which in the problem stage does not convert them back. (c) Keep
  `BINARY`, and document it as a conformance gap of `scip_milp`. Recommended: the probe, then (b) if it fails.
  Creating the column as `SCIP_VARTYPE_INTEGER` alone does not work, and retyping inside a guard is not
  available to a solver-agnostic function. Evidence: scipoptsuite 9.2.1 `scip/src/scip/var.c:1957-1963`
  converts [0, 1] integer variables to binary at creation, `scip_var.c:8330-8334` changes the type without
  converting in the problem stage, and `scip_milp.hpp:261-271` create the columns, and `:281-293` already
  retype existing ones through `SCIPchgVarType`. `milp_model` has type setters but no type getter
  (`model_concepts.hpp:214-248`). Blocked WP13, and the N7 (a2) claim on `scip_milp`.
- **N26. `compute_iis()` on HiGHS when the routine is missing or too old.** Ruled: the recommendation, whose
  mapping of a -1 return N34 (a) later amended. (a1) Throw `solver_error` when `Highs_getIis` does not
  resolve, and decode three regimes. (a2) Throw below 1.13, leaving two regimes, and on 1.13 the model's time
  limit applies to each internal solve. (a3) Throw below 1.14.0, leaving one regime. (b) A public runtime
  query member. (c) Raise HiGHS's validated floor for every feature. (d) Return an undetermined snapshot with
  a new reason. Recommended: (a3). The message names the loaded version, the library path and the 1.14 floor.
  No query API is added, since `api.library_version()` is already public, and the free function covers older
  releases after WP10. If `library_version()` is empty, as on a devel build, the wrapper requires the symbol
  and assumes the newest regime. Tests skip on the version, never on catching the exception. With the routine
  present, an empty answer is `feasible` when the model status is optimal or unbounded and `undetermined`
  otherwise, an error with model status `kTimeLimit` is `undetermined` with `time_limit`, and a `kWarning`
  return is `undetermined`. Evidence: `Highs_getIis` is absent from the 1.8.1, 1.9.0, 1.10.0 and 1.11.0
  headers, and `dlsym` finds nothing in 1.10.0. The 1.12.0 header marks the strategies as work in progress and
  numbers the statuses InConflict 0, NotInConflict 1 (`c_api_1.12.0.h:147-158`). 1.13.0 renumbers them
  NotInConflict -1, InConflict 1 and makes the strategies bitmasks (`c_api_1.13.0.h:147-162`), so 1.12's
  constants would read in-conflict as not-in-conflict. From 1.14.0, `iis_time_limit` replaces `time_limit`
  during the search (`HighsInterface.cpp:1981-1983`), though not for the whole call: the cheap checks run
  before it and the post-check re-solves with no limit (N29 evidence). The default strategy, Light, found
  nothing on a non-trivially infeasible LP on 1.15.1. The call-time precedents for optional symbols throw
  (`highs_milp.hpp:50-51`, `soplex_lp.hpp:268-271`). Gurobi's optional env symbols instead fall back to one
  another and throw at load only when both are missing (`gurobi_api.hpp:268-273, 305-321`). Under (a1) to
  (a3), `has_iis<highs_lp>` holds at compile time even when the loaded library has no usable routine, which
  sits uneasily with Q1's "Does this model has native IIS support?". (c) removes that state, and (d) keeps it
  without a throw. CI covers the three cases, with Windows pinned to 1.13.0 (`c-cpp.yml:307`). Linux and macOS
  are unpinned: Linux installs Ubuntu 25.04's package (`c-cpp.yml:46, 54`) and macOS brew's current bottle
  (`c-cpp.yml:281`), 1.9.0 and 1.15.1 in main's CI run 35922101900 of 2026-09-23. Blocked WP15, the HiGHS rows
  of WP17's path table, and the done-when of iis_todo.md step 6.
- **N27. The reason on a native answer that is not proven minimal while no limit stopped it.** Ruled: (a), no
  reason. (a) No reason: `get_reason()` is empty, meaning the routine itself did not prove minimality. (b)
  Reuse `iis_reason::inconclusive_trial`. (c) A reason for native paths only. Recommended: (a). The ruled
  reasons describe the filter's own stops, and `time_limit` is still set whenever a routine reports a
  time-limit stop. Evidence: N2 covers the engine's trials only. HiGHS documents answers with "maybe" entries
  as an infeasible set that is not irreducible (1.15.1 `HighsInterface.cpp:1852-1860`). With
  `iis_time_limit = 0` on an unsolved model, 1.14.0 and 1.15.1 returned -1 with model status 13, `kTimeLimit`.
  After a solve, 1.15.1 returned 1 with the whole model marked "maybe", and 1.14.0 returned -1 with model
  status 8 (measured, see N34). A missing proof does not imply a limit either: Gurobi 13.0.2 ended an
  unlimited IIS with `IISMinimal` = 0 after numerical trouble (measured). Only CPLEX and Xpress name a limit
  stop in their answer, so HiGHS attributes it by the time measured around the call (N34 a), and Gurobi and
  COPT need the same test (N29 evidence). Blocked WP5's reason contract softly, the outcome mapping of WP15
  and WP16, and WP17's reading table.
- **N28. Native wrappers and the N6 column-less precheck.** Ruled: (a), no precheck by default. (a) No
  precheck by default. The shared column-less case pins each routine's answer, and a wrapper that fails it may
  call the shared detail arithmetic, which is not the deletion filter. (b) Every native wrapper runs the
  arithmetic first. (c) Native wrappers never treat column-less models specially. Recommended: (a). Evidence:
  on HiGHS 1.15.1, with the only column removed, the row 0 >= 1 came back as a lower-side member, which is
  correct, whereas `highs_lp::solve()` returns `unknown` without columns (`highs_lp.hpp:95-98`). The
  recommendation that came with N6 said "shared by native wrappers", which the ruling does not repeat. Gurobi,
  CPLEX, COPT and Xpress are unprobed. Blocked WP15, WP16 and the shared column-less case.
- **N29. Bounding a native `compute_iis()` in time.** Ruled: (a), the model's time limit bounding each call as
  a fresh budget. The maintainer wrote "Ok for (a)." That answers the bound only, so the second choice of this
  entry, a time limit on `copt_lp`, became N35, ruled (a) below. Gurobi, CPLEX, Xpress and `copt_milp` need no
  code. HiGHS copies `get_time_limit()` into `iis_time_limit` for each call, restored afterwards (WP15). The
  docs state per backend that the budget is per call, which other model limits also stop the routine, and that
  a stop may return late or with no answer. (b), recommended for later if users ask, is not ruled: it stays a
  Deferred possibility in iis.md, not a decision. (c) and (d) are rejected. The maintainer had asked: "It is
  not obvious that set_time_limit() should set the time limit used by IIS computation, is it documented for
  most solver ?" *Answer.* It is mostly not documented. Gurobi and CPLEX state it outright, the Xpress manual
  only implies it, COPT is silent, and HiGHS documents the opposite, a separate `iis_time_limit` that replaces
  `time_limit` during the call. Yet four of the five routines stop under the parameter that `set_time_limit`
  writes, and HiGHS only if MIP++ copies it over (measured). On Gurobi and CPLEX the real choice is therefore
  whether MIP++ keeps or lifts a limit that the vendor documents as covering the IIS (inferred).

  (a) The model's time limit bounds each call, as a fresh budget. Gurobi, CPLEX, Xpress and `copt_milp` need
  no code, because their routines already read the parameter that `set_time_limit` writes. HiGHS copies
  `get_time_limit()` into `iis_time_limit` for each call, as WP15 plans. The docs state per backend that the
  budget is per call, which other model limits also stop the routine, and that a stop may return late or with
  no answer. (b) An explicit duration, as an additive overload `compute_iis(std::chrono::duration<double>)`
  that saves the parameter the routine reads, writes the argument and restores it. The call without an
  argument behaves as in (a). (c) Unbounded: every wrapper saves the solve limit, writes its no-limit value,
  calls the routine and restores the limit. (d) The solver's own rule, documented per backend: bounded on four
  backends, and unbounded on HiGHS, whose `iis_time_limit` stays at its default. Recommended: (a) now, and (b)
  later if users ask, as an additive overload like N31 (b). (a) is what two vendors document and what four
  routines already do. (c) goes against the documented Gurobi and CPLEX semantics, needs the no-limit values
  of N32, and on CPLEX still leaves the iteration and node limits in force. (d) makes HiGHS the one unbounded
  routine. Evidence. *Parameters:* `set_time_limit` writes `TimeLimit` on the model's env on Gurobi
  (`gurobi_base.hpp:613-615`), `CPXPARAM_TimeLimit` on CPLEX (`cplex_base.hpp:554-556`), `TIMELIMIT` on Xpress
  (`xpress_base.hpp:425-426`, `xpress_milp.hpp:176-177`), `TimeLimit` on `copt_milp` only
  (`copt_milp.hpp:164-166`), and `time_limit` on HiGHS (`highs_base.hpp:629-631`). *Documented:* Gurobi's
  `GRBcomputeIIS` reference (docs.gurobi.com, 11.0 to 13.0) says "Termination parameters such as TimeLimit,
  WorkLimit, MemLimit, and SoftMemLimit are considered when computing an IIS." CPLEX's time-limit page
  (22.1.2) says "This time limit applies also to the conflict refiner" and that the limit "applies to each
  call individually". The Xpress 47.01 manual lists only `XPRSlpoptimize` and `XPRSmipoptimize` under
  `TIMELIMIT`, while its `IISSOLSTATUS` 3 means "either because of hitting the time limit or because of a user
  interrupt". COPT 8.0 documents `TimeLimit` as "Time limit of the optimization", and its IIS chapter names
  only `IISMethod`. HiGHS 1.14.0 added a "dedicated time-out during IIS calculation (using HiGHS option
  `iis_time_limit`)", default inf, which replaces `time_limit` during the call after zeroing the clocks
  (1.15.1 `HighsInterface.cpp:1984-1986`). *Measured,* with the limit written by the same call as
  `set_time_limit`: Gurobi 13.0.2 stopped an LP IIS at 2.000 s under 2 s (20.9 s unlimited) and a MILP IIS at
  1.04 s under 1 s, and 11.0.3 and 12.0.1 agree. CPLEX 22.1.1 stopped at 0.573 s under 0.5 s (2.7 to 6.2 s
  unlimited), and its iteration and node limits stopped the refiner too, with statuses 34 and 35. Xpress 47.01
  and 45.01 stopped at 6.23 s and 6.11 s under 6 s (7.8 s unlimited). COPT 8.0.5 stopped at 2.000 s under 2 s
  (18.1 s unlimited) on an LP, and at 1.013 s under 1 s on a MIP. HiGHS 1.14.0 and 1.15.1 ignored `time_limit`
  in every case. `iis_time_limit` stopped them at 0.50 s and 0.52 to 0.55 s under 0.5 s, but 1.15.1 once
  returned after 16.5 s under 1 s, because its post-check re-solves the returned set with no limit (1.15.1
  `HighsInterface.cpp:1877`). Every routine restarted the budget at each call. The four commercial routines
  return a success code on a stop, while HiGHS returns -1 with nothing written on an early stop and 1 with a
  partial set later. A stop may leave no answer at all: Gurobi `IISMinimal` unavailable (error 10005), CPLEX
  status 33 with every group excluded, Xpress `NUMIIS` 0, COPT `HasIIS` 0. Only CPLEX and Xpress report a
  limit stop as such, so Gurobi, COPT and HiGHS need an elapsed-time test to set `time_limit` under N27 (a),
  which N34 (a) rules for HiGHS. Under (a), `solve()` followed by `compute_iis()` may take twice the limit.
  Blocked WP15's `iis_time_limit` item, the stop mapping of WP15 and WP16, WP17's limits paragraph, and the
  wording of N9.
- **N30. Row bounds on the commercial backends.** Ruled: (b), with a condition. The maintainer wrote "(b) When
  row bounds are trully supported natively (not like Gurobi that is only emulating them)." Modifiable row
  bounds come to COPT, CPLEX and Xpress as optional commits of their WP16 pull requests, only where the solver
  stores a row's two sides natively, which each commit confirms first for its solver. Gurobi's ranges add a
  slack column, an emulation that never qualifies. Readable row bounds reach every model through WP6c (Q6 a),
  so N30 governs the modifiable half. (a) None: the free function stays on the open-source backends and HiGHS.
  (b) Row bounds on COPT, CPLEX and Xpress, as optional commits in their WP16 pull requests, and Gurobi not
  yet. (c) Full parity, including Gurobi through sense switching, with its setter throwing on a finite ranged
  state. Recommended: (b). The enumeration already reaches every backend through WP6a. Evidence: Gurobi and
  CPLEX have readable and modifiable sense and rhs but no row bounds, and COPT and Xpress have neither
  (compile-checked concept table). COPT rows are two-sided natively, and CPLEX and Xpress rows can be ranged,
  according to their documentation, unprobed. Gurobi ranges a row only through a slack column, which the
  in-place ruling rejects. Commercial backends are tested locally only. Blocked the scope of WP16, and the
  both-paths check beyond HiGHS.
- **N31. Cancelling a native call.** Ruled: (b), later, as an additive overload, not in the first version. (a)
  Never. (b) Later, an additive overload `compute_iis(std::stop_token)`, wired to `GRBterminate`,
  `CPXsetterminate`, `XPRSinterrupt`, `COPT_Interrupt` and the HiGHS interrupt callback. (c) Now. Recommended:
  (b), not in the first version. The overload leaves `model_iis_t` and `has_iis` unchanged. Evidence: under
  N9, native calls take no `iis_limits`. HiGHS 1.15.1 keeps its interrupt callback active during
  `Highs_getIis`. CPLEX documents status 38 when `CPXsetterminate` stops the refiner. Blocked nothing.
- **N32. `set_time_limit` with an unlimited value on every backend.** Ruled: (a), +inf and
  `duration<double>::max()` accepted on every backend, through the SCIP and CPLEX clamps. (a) Accept +inf and
  `duration<double>::max()` everywhere: SCIP clamps to 1e20 and CPLEX to 1e75, the values their fresh models
  report, as Gurobi and COPT already do natively. (b) The contract is only a non-negative getter and an exact
  `set(get())` round trip. The docs name the fresh value or `std::chrono::seconds::max()`, 9.2e18 s and under
  every ceiling, as the portable "no limit", and the new cases use `seconds::max()` only. Recommended: (a). It
  is two one-line clamps, and users write "no limit" as +inf or `duration::max()`. The getters stay
  transparent because each clamp target is the fresh value. Evidence: SCIP 8.0.4 and 9.2.1 throw
  `SCIP_PARAMETERWRONGVAL` on +inf, `DBL_MAX`, 1e30 and 1e100 (`set.c:1625`, range [0, 1e20]) and print four
  `ERROR` lines to stderr despite quiet mode. CPLEX 22.1.2 throws error 1015. Gurobi 12 and 13 read back 1e100
  after `set(+inf)`, and COPT 8.0 reads 1e20. HiGHS 1.10.0, Cbc, MOSEK 11.0 and Xpress 47.01 read back +inf.
  Blocked the SCIP and CPLEX parts of the time-limit fix, and the cases `unlimited_time_limit_round_trips` and
  `lifted_time_limit_takes_effect`.
- **N33. Negative and NaN values written by the caller.** Ruled: (b), the Cbc setter throws `solver_error` on
  a negative value, and the getter stays transparent. (a) The Cbc getter returns `std::max(t, 0.0)`, and NaN
  is left to each solver. 0 is not among the targets that N4 (A) lists, so (a) needs an amendment of that
  ruling. (b) The Cbc setter throws `solver_error` on a negative value, as HiGHS, Gurobi, CPLEX, SCIP and
  Xpress already do natively and as the fixed SoPlex setter does, and the getter stays transparent. (c) A
  negative Cbc value reads as an infinity, one of the ruling's targets. (d) Every setter throws `solver_error`
  on NaN. Recommended: (b). It meets every clause of N4 (A): no negative value can be stored, so none is read
  back, and the getter stays transparent. (c) would misstate Cbc, which stops at once under a negative limit,
  and (a) would need N4 (A) amended. NaN is not negative, and `iis_limits` already rejects it. Evidence: Cbc
  2.10.11 stops a branching MILP after 0.011 s with `time_limit` under -1 s, as under 0 s. Cbc stores any
  value (`CbcModel.hpp:585-589`), and main's setter passes it through (`cbc_milp.hpp:425-427`). The time-limit
  audit ran with (a). `negative_time_limit_is_never_read_back` accepts a setter that throws, so (b) should
  pass it too, but that variant was not run. On NaN, SCIP and Xpress throw, Gurobi reads back 1e100, SoPlex
  treats it as no limit, and the others read NaN back. The free function writes `std::min(remaining, saved)`,
  so a NaN saved value forwards `remaining`. Blocked the Cbc part of the time-limit fix, the case
  `negative_time_limit_is_never_read_back`, and one WP7 stub test.
- **N34. HiGHS 1.14.x returning -1 with model status 8 after a stop in the elasticity filter.** Ruled: (a),
  N26's mapping amended by the time measured around the call. A -1 return from `Highs_getIis` is
  `undetermined` with `time_limit` when the time measured around the call reached the limit copied into
  `iis_time_limit`, whatever the model status, and throws `solver_error` otherwise. The same elapsed-time test
  attributes `time_limit` to a "maybe" answer. N26 as ruled makes an error with model status `kTimeLimit`
  `undetermined` with `time_limit`, and a `kWarning` return `undetermined`. On 1.14.0, a stop by
  `iis_time_limit` in the elasticity filter instead returns -1 with model status 8, `kInfeasible`, and writes
  nothing (measured). As ruled, the wrapper would then throw `solver_error` on a limit the user asked for. At
  the same point 1.15.1 returns 1 with the whole model "maybe", which N26 already maps. (a) Amend N26's
  mapping: a -1 return is `undetermined` with `time_limit` when the time measured around the call reached the
  limit copied into `iis_time_limit`, whatever the model status, and throws otherwise. (b) Treat -1 with model
  status 8 as `undetermined` with `time_limit` on 1.14.x, with no elapsed-time test. (c) Leave N26 as ruled,
  so the wrapper throws there. Recommended: (a). The C API exposes no IIS status, so the elapsed time is the
  only signal that separates a limit stop from an error. The same test attributes `time_limit` to a "maybe"
  answer, which the vendor's partial result returns with the same code when no limit stopped it. (b) would
  label a real error with status 8 as a limit stop, and (c) turns a limit the user asked for into an
  exception. The case arises only if N29 bounds HiGHS, under (a) or (b), since under (c) and (d)
  `iis_time_limit` stays at inf, and the ruled N29 (a) makes it arise. Evidence, measured on 1.14.0 and
  1.15.1: on a 4051-row, 3051-column LP, `iis_time_limit` at 0.5 s and 1.5 s gave -1 with model status 8 on
  1.14.0, after 0.501 to 1.627 s, and 1 with the whole model "maybe" on 1.15.1. After a prior solve,
  `iis_time_limit = 0` gave -1 with status 8 on 1.14.0. On an unsolved model, a stop while re-establishing
  infeasibility gave -1 with status 13 on both releases, which N26 maps. Every measured stop came at or after
  the limit. The call zeroes the HiGHS clocks, so a wall-clock measure around it reads at least HiGHS's own
  time (inferred). Blocked WP15's stop mapping.
- **N35. A time limit on `copt_lp`.** Ruled: (a). The maintainer wrote "N35. You proved that putting
  set_time_limit in copt_base respects all contracts thus yes." `set_time_limit` and `get_time_limit` move
  from `copt_milp` into `copt_base`, and `TimeLimitTest` is instantiated for `COPT_lp`, as fix 10, now a firm
  standalone fix before WP16. Under N29 (a), a native `copt_lp` IIS is then bounded by the model's time limit,
  like `copt_milp`. It split from N29 on 2026-09-27, since the ruling "Ok for (a)" answers only N29's bound.
  The maintainer had asked, with N29: "Does COPT even respect the time limit when solving LPs ?" *Answer.*
  Yes. On 8.0.5, `COPT_SolveLp` stopped with `LpStatus` 8, `TIMEOUT`, under every usable `LpMethod`: dual
  simplex and the automatic choice within 15 ms, and barrier only between iterations, up to 0.75 s late
  (measured). `copt_lp` lacks the setter apparently by accident (inferred). (a) Move `set_time_limit` and
  `get_time_limit` from `copt_milp` into `copt_base`, and instantiate `TimeLimitTest` for `COPT_lp`, as a
  standalone fix before WP16. (b) Keep `copt_lp` without a limit, and correct
  `docs/solving/status-and-limits.md:87`, which lists COPT under `has_time_limit` without saying `copt_milp`
  only. Recommended: (a), on its LP merits alone. Evidence. *Documented:* COPT documents
  `COPT_LPSTATUS_TIMEOUT` as "The LP optimization is stopped because of time limit." *Measured* on 8.0.5, on a
  sparse packing LP of 20,000 columns and 10,000 rows that dual simplex solves in 885 s without a limit: under
  a 2 s limit, dual simplex stopped at 2.015 s with `LpStatus` 8, and the automatic choice picked simplex and
  stopped at 2.010 s. Barrier, concurrent and the other usable methods also ended with `LpStatus` 8. Under a
  0.5 s limit, simplex stopped at 0.504 s and barrier at 1.251 s, before its first iteration. A dense LP
  shaped like `TimeLimitTest`'s stopped at 0.213 s under the suite's 0.2 s limit, so the suite would very
  likely pass on `COPT_lp` (inferred). `copt_lp.hpp:45` already maps that status. *Inferred:* `copt_lp` lost
  its limit by accident. `d583b34` moved Gurobi's setter into its base but left COPT's in `copt_milp`
  (`copt_milp.hpp:164-170`), and `f48c68f` and `d8bb08f` instantiated `TimeLimitTest` for the other LP models
  only, with no recorded reason. *Not covered:* COPT 7.2.5 was not probed, since the local license refuses it.
  Blocked fix 10 of the order table, and the COPT part of WP16 softly.

### Amended on 2026-09-28

A simplification review before wave 2, recorded in iis.md as N36 and N37. N36 drops the zero-solve claim on
crossed variables and `detail::may_carry_background<M>`: every crossed pair seeds the engine's continuation and
two singleton trials decide it. N37 fixes how extensions attach: new candidate kinds as new per-kind tables, the
snapshot's template arguments never spelled in docs or tests, the engine kind-agnostic, the tag hierarchy kept,
batching dormant, and protected sides with candidate order as the first extension after the first version. WP7
and the ideas table are amended below; the rest of the plan stands.

### Pending

Every question of this plan is ruled as of 2026-09-27, N35 last. Wave 3 raised two on 2026-09-28, N38 and
N39, stated under [Open questions](iis.md#open-questions) in iis.md.

## Corrections to iis.md and iis_todo.md

The revision of iis.md and iis_todo.md of 2026-09-27 applies these in the working tree, not yet committed. An
item is ticked when the revised text applies it. Line numbers are those of iis.md on main, and step numbers
those of iis_todo.md on main.

- [x] **Rulings of 2026-09-27.** Insert the section after "What the pull request contributed", and replace
  open questions 1 to 6 by the new questions N19 to N33. Those ruled later that day join the rulings, N29, N34
  and N35 included, so no open question remains, and each reading of a short ruling is marked as confirmed,
  N4's choice as a decision. The introduction names both ruling dates and the checks of 2026-09-27.
- [x] **Shape of the API, first bullet** (iis.md:17-21). "A capability, not a separate library" and "callers
  never see which path produced the answer" fall with Q1 (b). Two bullets replace it: native routines behind
  `has_iis`, and the public deletion filter.
- [x] **Split** (iis.md:107-111). Native `has_iis` on `gurobi_*`, `cplex_*`, `copt_*`, `xpress_*`, `highs_lp`
  and `highs_qp`. The free function on Clp, Cbc, GLPK, the three HiGHS classes, MOSEK, SCIP and SoPlex as
  WP8 to WP14 land, and on COPT, CPLEX and Xpress only through N30 (b). `dumb_lp` waits (N24).
- [x] **MOSEK step 1** (iis.md:295-301). Items 1 and 3 are done on main: `d8bb08f` optimizes once
  (`mosek_lp.hpp:102-108`), and `TimeLimitTest` runs (`test/solvers/mosek.cpp:23, 39`). Tick both.
- [x] **Step 1 items 2 and 4.** The signature fix is cosmetic, `MSKrestrmcode` being an `int` enum
  (`mosek_api.hpp:27, 165, 183`). The MILP status already picks a slot. The gap is the LP status and every
  getter of both classes (`mosek_lp.hpp:41-60`). Item 4 is struck (N5 b).
- [x] **Lazy objectives** (iis.md:121-124). Cbc (`cbc_milp.hpp:139-148`) and SCIP (`scip_milp.hpp:219-231`)
  are lazy too.
- [x] **SoPlex** (iis.md:202-205, step 5). 6.0.4 has `SoPlex_changeVarBoundsReal` and sits below the 7.1.1
  floor. 7.1.3 declares every N11 symbol; wave 3 found that the wrapper must still keep the column bounds
  (iis.md, SoPlex).
- [x] **Row side symbols** (iis.md:198, step 5). SCIP also needs `SCIPgetLhsLinear` and `SCIPgetRhsLinear`,
  unbound like its setters. MOSEK's per-side setter is `MSK_chgconbound`, not `MSK_putconbound`.
- [x] **Limits** (iis.md:126-127, 166-168, 286-288, step 4). Replace with the Q5 (c) contract and its
  forwarding rule: one deadline, `std::min(remaining, saved)` only with a finite deadline, `saved` read once
  and restored exactly. `clp_lp` and `glpk_*` lack `has_time_limit`, and so does `copt_lp` until fix 10
  (N35 a), so one trial can overrun there, and forwarding first runs on Cbc.
- [x] **Column-less models** (iis.md:229-231, step 5). Clp, HiGHS and SoPlex also return `unknown`, and MOSEK
  returns `optimal` (`mosek_lp.hpp:107`). The free function answers by the N6 (b) precheck, and native
  wrappers run none by default (N28 a).
- [x] **Archetype** (step 2). No capability has one. Per N10 (a), fixtures assert the concepts instead.
- [x] **Snapshot translation** (iis.md:34-41). Handle results translate lazily today (`highs_lp.hpp:104-130`),
  so the snapshot is the first eager one. `clp_lp` is not on the remapping layer. The id bound is
  `_remap_ids ? _native_ids_map.size() : N`. Add N8 (a).
- [x] **Scope and cases** (iis.md:54-62, 227-229). No backend implements `add_sos1_constraint` or
  `add_sos2_constraint` (`has_sos1_constraints` holds nowhere). Indicators exist only on `gurobi_milp` and
  `cplex_milp`, and combined with removals they hit a main bug, fixed first.
- [x] **Only proofs count** (iis.md:152-160). This needs sound mappings, which GLPK's `GLP_INFEAS`, HiGHS's
  uninitialized `psolstatus` and Clp's secondary status break. The classifier is a visitor over
  `model_status_t<M>`, and Clp reports `optimal_infeasible_unscaled` after the N13 fix.
- [x] **Side effects** (iis.md:172-176). Replace with Q4 (a) for the free function, and with `unknown` after
  every native call for now (N15).
- [x] **HiGHS** (iis.md:98-99, 232-234, step 6). `Highs_getIis` is absent from 1.8.1 to 1.11.0, with 1.11.0
  now checked. It has three regimes. 1.12 marks the strategies as work in progress and numbers the statuses 0,
  1 and 2. 1.13 makes the strategies bitmasks and renumbers the statuses -1, 0 and 1. From 1.14,
  `iis_time_limit` replaces `time_limit` during the search, though not for the whole call (N29 evidence). The
  `x = 0.5` probe ran under the Light default, which finds only trivial conflicts, so it proved nothing.
  `highs_milp` gets no `compute_iis()`, and the native floor is 1.14.0 (N26). The row arrays take the row
  counts, though the 1.12.0 header documents column counts (`highs_c_api.h:2428-2442`). CI covers the
  missing-symbol case (Linux, 1.9.0), the 1.13 regime (Windows, 1.13.0) and the 1.14+ regime (macOS, 1.15.1),
  but not 1.12. Only Windows is pinned. Linux takes Ubuntu 25.04's package and macOS brew's current bottle,
  whose versions are those of main's CI run 35922101900 of 2026-09-23. The macOS case keeps covering the 1.14+
  regime while brew's release stays in the validated range, which ends before 1.16.
- [x] **The deletion filter** (iis.md:112-176). Rename "The generic fallback" to "The deletion filter, a
  public algorithm", open with the engine contract, then the free function. Batching is ported dormant (N3 b),
  and the N6 and N7 prechecks get their own bullet.
- [x] **Library additions** (iis.md:178-210). Rename the section for the free function, add the columns
  "Enumeration" and "Status reset", and point the row relaxation at Q3 (a).
- [x] **Two test suites** (iis.md:214-231). `IisTest` asserts `has_iis`, and `IisByDeletionTest` asserts the
  free function's concept. They share case bodies and the exact oracle. The cases split into shared,
  filter-only and native-only.
- [x] **Deferred** (iis.md:254-260). Rename the bullet "Enumeration" (Xpress `XPRSiisnext`, `XPRSiisall`) to
  "Several IISs per model", so it cannot be confused with the entity enumeration. The public filter and
  batching leave the list.
- [x] **Certificate plumbing** (iis.md:242-243, 301-302). The SoPlex part needs an unsubmitted 8.1.0 patch and
  waits for upstream. The Clp ray must gate on the cached `_status`, which the N22 reset keeps honest after a
  filter run.
- [x] **Outside IIS** (iis.md:309-311). Record N18 (a).
- [x] **Feature tables** (step 7). `highs_qp` rows never appear, as the script sorts by `_lp_` or `_milp_`
  (`tested_features_table.py:31-34`). WP17 fixes the script.
- [x] **Time-limit probes of 2026-09-27** (working-tree iis.md). The native routines ran under a time limit,
  and each result goes where it applies. In iis.md's native routines: a time-limit column and bullet, CPLEX
  statuses 32 to 39 with the empty status-33 answer, the COPT and Xpress findings, the N15 results for Gurobi
  and COPT, and the corrected HiGHS sentence, since `iis_time_limit` does not bound the whole call. In this
  plan: the N27 evidence, N29 revised, the new N34, and WP15 and WP16.

## Handling of pull request #3

- **State on 2026-09-27.** Open, not a draft, mergeable, maintainer edits allowed. On 2026-09-23 the
  maintainer wrote that IIS "will fit well alongside LP basis support in a future v1.1.0 release", and that
  the design notes will serve "to merge your IIS PR in v1.1.0". The author answered that they can work on it
  only on Friday nights. On 2026-09-26 they wrote that Astra "implemented the items in the TODO", which the
  branch does not bear out (see the reply below).
- **Handling, ruled (N0 a).** The maintainer reshapes it in place, then squash-merges it. This follows the
  stated intent and keeps the author's credit. About 7,000 of the author's lines leave their branch, so the
  reply asks for their agreement before the first reshaping push. Re-landing through the author's pull
  requests would have tied the critical path to Friday nights.
- **Mechanics.** Tag `a1a9f11` on origin, for example `archive/pr3-a1a9f11`, before the first reshaping push,
  since `refs/pull/3/head` follows the branch. Convert the PR to draft with the GraphQL call of
  `misc/contributor_pr_cheatsheet.md` section 1, as gh 2.4.0 has no `--undo`. The reply asks the author to
  stop pushing. Then follow the cheatsheet loop: `git pull --rebase` before each push, never `--force`.
- **Commits.** (1) `refactor(iis)`: remove the replaced surface per the file map, restoring main's versions of
  the other files. (2) `feat(iis)`: the std-only header of `iis_limits`, `iis_outcome` and `iis_reason`, with
  the public engine (WP4). (3) `feat(model)`: the `iis_status` tags, the snapshot template and the concepts
  (WP5). (4) `test(iis)`: the oracle and vectors (WP5). On green CI, `gh pr ready 3` and a squash-merge whose
  body lists the ported parts. Under N17 (a) the types are public at merge.
- **Credit.** Check on GitHub who a squash commit is attributed to, and word the trailer and the reply to
  match. Their commits use two email addresses, so the reply asks which one to credit. Every later PR porting
  their code carries the same `Co-authored-by` trailer.
- **Later PRs.** Fresh branches from main, one topic each, porting with `git show a1a9f11:<path>`, never
  cherry-picks. The body ends with "Ported from a1a9f11 (PR #3)". MOSEK, SCIP, SoPlex and commercial PRs
  attach a local ctest log, built one at a time with `-c tools.build:jobs=4`.
- **Who does what.** The maintainer owns the critical path. The author is offered leaf work: WP2 and WP12 if
  they have MOSEK, WP3, the loader fix as ruled (N18 a), Gurobi and CPLEX in WP16 if they still have Gurobi 13
  and CPLEX 22.2 (commits `637d166`, `9b36c46`), the SoPlex proposal upstream, the harness later. WP12 no
  longer forwards a time limit specially, since the free function does it for every model. Their
  `public-deletion-filter` idea ships in pull request #3. An offer unclaimed after two Fridays goes to the
  maintainer.
- **Order.** The standalone fixes open now, the destructor fix after WP2. After WP1, pull request #3, WP2,
  WP3, WP6, WP6a, WP6b and WP6c run in parallel. WP2 and WP6b land in either order, and whichever lands second
  wires MOSEK's slot cache into `reset_status()`, so WP2 stays off the critical path. The critical path is
  WP1, pull request #3, WP6 with WP6a and WP6b, WP7, WP8, then WP9 to WP11, with WP6c before WP10 and WP11.
  Native packages wait for WP5 and the native suite, WP15 first, so that `IisTest` runs in CI before any
  backend tested only locally. No tag is cut between the merge of pull request #3 and WP8.

| File at `a1a9f11` | Destination |
| --- | --- |
| `algorithm/deletion_filter.hpp`, `algorithm/iis_limits.hpp` | WP4 public engine in `utility/deletion_filter.hpp` with batching dormant in its detail entry, WP5 types in `utility/iis_outcome.hpp`, ordering and `phase_budget` to WP3 |
| `algorithm/{elasticity_filter,ray_support}.hpp`, `utility/{native_seed,elastic_lp}.hpp`, `solvers/clp/*` | WP3 notes |
| `utility/{linear_iis_model,deletion_workspace,cold_model_workspace,prepared_linear_system,iis_limits}.hpp` | WP7 hunks and ideas, rewritten over the enumerated handles, files dropped |
| `utility/{linear_iis,linear_iis_types,iis_report,iis_statistics}.hpp`, `algorithm/iis_messages.hpp`, `infeasibility_certificate.hpp` | dropped, checklists to WP3, warnings to WP17 |
| `solvers/mosek/*` | WP2, without the certificate, license hint and optimizer-count hunks, the time-limit getter to the time-limit fix |
| `solvers/soplex/*`, `patches/*`, `solvers/{cplex,gurobi}/*_api.hpp`, `model_concepts.hpp` | move fix to the standalone fixes, the rest dropped, Farkas API proposed upstream |
| `detail/{solver_library,diagnostic_text}.hpp`, `test/dynamic_library.cpp` | loader fix (N18 a), trimmed |
| `test/iis.cpp`, `test/iis_diagnostics.cpp` | WP2, WP4, WP5, WP7 and WP8 tests, classification inverted, optimizer-count tests dropped (N5 b), the rest dropped |
| `test/iis_vectors.cpp`, `test/data/iis/*`, `test/CMakeLists.txt` | WP5 oracle and data, README replaced by a provenance comment, WP8 shared cases, no separate binary |
| `docs/iis.md`, `docs/iis-implementation-status.md` | WP17 prose for two pages and WP3 notes, audit quoted in the thread |
| `benchmarks/iis.cpp`, `CMakeLists.txt`, `.gitignore`, `README.md`, `coming-from.md`, `zensical.toml` | reverted, harness out of the repository, docs redone in WP17 |

## Work packages

Each package appears after everything it needs, except inside pull request #3, where the std-only types header
of WP5 lands in WP4's commit or before it. The Step column refers to the rewritten iis_todo.md. Rows named
"Fix" are standalone fixes on main, listed here because packages depend on them. A need named for one part
blocks only that part.

| Order | ID | Title | Todo step | Owner | Lands in | Needs |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | WP1 | Rulings, corrections, reply | 0 | maintainer | a commit on main | the rulings of 2026-09-27 |
| 2 | Fix | Moves keep the status, HiGHS `psolstatus` initialized, constant-row status case | 1.1 | maintainer | own PR | none |
| 3 | Fix | GLPK status mapping | 1.2 | maintainer | own PR | none |
| 4 | Fix | Native ids after removal (Gurobi, CPLEX) | 1.3 | maintainer | own PR | none |
| 5 | Fix | Clp secondary status (N13) | 1.4 | maintainer | own PR | none |
| 6 | Fix | Time-limit contract (N4, N32, N33) | 1.5 | maintainer | own PR | none |
| 7 | Fix | Loader message (N18) | outside IIS | offered to the author | own PR | none |
| 8 | WP2 | MOSEK fixes | 1.6 | author if licensed | own PR | WP1 |
| 9 | Fix | Destructors that cannot throw (COPT, CPLEX, Gurobi, Xpress) | outside IIS | maintainer | own PR | WP2 |
| 10 | Fix | Time limit on `copt_lp` (N35 a) | outside IIS | by license | own PR | none |
| 11 | WP3 | Deferred notes | Later | author | docs PR | WP1 |
| 12 | WP4 | Public deletion engine | 2.1 | maintainer | PR #3 | WP1, the std-only types header of WP5 |
| 13 | WP5 | IIS types, snapshot template, oracle | 2.2-2.4 | maintainer | PR #3 | WP1 |
| 14 | WP6 | Row-bound concept on Clp | 3.1 | maintainer | own PR | none |
| 15 | WP6a | Entity enumeration on every model | 3.2 | maintainer | own PR | none |
| 16 | WP6b | Status reset on every model | 3.3 | maintainer | own PR | none |
| 17 | WP6c | Readable row bounds on every model | 3.4 | maintainer | own PR | none |
| 18 | WP7 | Public free function | 4.1 | maintainer | own PR | PR #3, WP6, WP6a, WP6b |
| 19 | WP8 | Clp slice and `IisByDeletionTest` | 4.2 | maintainer | own PR | WP7, Clp secondary status fix |
| 20 | WP9 | Cbc | 5.1 | maintainer | own PR | WP8, time-limit fix |
| 21 | WP11 | GLPK | 5.2 | maintainer | own PR | WP8, WP6c, GLPK status fix |
| 22 | WP10 | HiGHS row bounds, three classes | 5.3 | maintainer | own PR | WP8, WP6c, `psolstatus` fix |
| 23 | WP12 | MOSEK | 5.4 | author if licensed | own PR | WP2, WP6c, WP8, time-limit fix |
| 24 | WP13 | SCIP | 5.5 | maintainer | own PR | WP8, WP6a, WP6c |
| 25 | WP14 | SoPlex | 5.6 | maintainer | own PR | WP6c, WP13, time-limit fix |
| 26 | WP15 | HiGHS native and `IisTest` | 6.1 | maintainer | own PR | PR #3, and WP10 with WP6a for the both-paths case only |
| 27 | WP16 | Gurobi, CPLEX, Xpress and COPT native | 6.2-6.6 | by license | a PR per solver | WP15, native-id fix, fix 10 for the COPT part, and WP6c with a per-solver check under N30 for row bounds |
| 28 | WP17 | Documentation | 7 | maintainer | own PR | WP8-WP11, WP15 |

- **Critical path.** WP1, then pull request #3 (WP4 and WP5), WP7, WP8, and WP9 to WP11. WP6, WP6a and WP6b
  run in parallel with pull request #3, and all three must land before WP7. WP6c runs beside them and lands
  before WP10 to WP14 and the row-bound commits of WP16.
- **Native path.** WP15 needs only pull request #3, so it can start as soon as that merges and run in parallel
  with WP6 to WP8. It goes first among the native packages, so that `IisTest` runs in CI, on macOS with HiGHS
  1.15.1, before any backend tested only locally. Its both-paths case follows WP10 and WP6a.
- **Release.** Under N17 (a), no tag is cut between the merge of pull request #3 and WP8, and none is planned
  before WP8. WP5's types are therefore public when pull request #3 merges. v1.1.0, which also carries the
  rest-or fixes, is tagged after WP8 at the earliest.
- **Inside pull request #3.** The std-only header of `iis_limits`, `iis_outcome` and `iis_reason` lands in the
  engine's commit or before it, since the engine uses those types.
- **Later.** Outside the first version, each with a Deferred bullet in iis.md: `dumb_lp` under
  `IisByDeletionTest` (N24), cancelling a native call through `compute_iis(std::stop_token)` (N31 b), and
  integrality as a candidate (N25). An explicit duration for a native call (N29 b) has a Deferred bullet too,
  as a possibility that is not ruled. The notes of WP3 cover the pull request's deferred ideas.

### WP1. Rulings, corrections, reply

The three documents were revised on 2026-09-27 in the working tree, and none of it is committed. iis.md
records the rulings, those of later that day included, with the readings the maintainer confirmed, and lists
no open question. The corrections above are applied, and iis_todo.md follows the table above.

- [ ] Commit iis.md, iis_todo.md, this plan and iis_pr_plan_pre_rulings.md on main. The pre-ruling plan
  keeps the options behind letters such as "N7 (a2)" and "N14 as recommended" readable.
- [ ] Post the reply of [Reply to the author](#reply-to-the-author).
- [ ] Act on N0 (a): tag `a1a9f11` on origin and convert pull request #3 to a draft, as
  [Handling of pull request #3](#handling-of-pull-request-3) describes.
- **Done when** the four documents are on main, the reply is posted, and pull request #3 is a draft.

### WP2. MOSEK fixes

- **Port.** `mosek_handle_guard.hpp:1-31`, namespace renamed `detail`. `mosek_base.hpp:35, 43-80, 104-131`.
  `mosek_api.hpp:189`. `mosek_lp.hpp:38-100, 107-143`. `mosek_milp.hpp:86-173, 182-194`, with 174-180 replaced
  by `_status = _get_status(trm);` unless the probe below says otherwise. Tests from
  `test/iis.cpp:251-263, 299-323` and `test/iis_diagnostics.cpp:163-176`.
- **Drop.** `mosek_lp.hpp:7, 145-175`, `infeasibility_certificate.hpp`, `mosek_api.hpp:477-479`, the
  `static_assert` at `test/iis.cpp:324`. Under N5 (b), `mosek_api.hpp:139-145, 443` with
  `test/iis.cpp:758-831, 940-956`. `mosek_base.hpp:471-485` is not ported here. Its getter half moves to the
  time-limit fix, with the author's credit. Its setter half, which writes +inf as -1, is dropped, so that the
  setter stays transparent, as N4 (A) asks when possible.
- [ ] The guard, a destructor that cannot throw, and slot selection on the LP status and every getter. A
  primal-feasible slot ranks above a dual-only one, which the pull request ties. The pick is cached and reset
  with `_status`. WP2 needs only WP1: if WP6b has landed first, WP2 also resets the cache inside the
  `reset_status()` of `mosek_lp` and `mosek_milp`, and otherwise WP6b does.
- [ ] Status reset before optimizing, `trm` initialized, signature. A column-less task maps MOSEK's own
  status. Probe MOSEK 11 on empty LP and MIP tasks, which `LpModelTest.solve_empty_*` solves. If one gets no
  slot, keep an empty-task path, as `_get_status` then gives `unknown` where main reports `optimal`.
- [ ] Own commit, per N18 (a): `_check` maps every license code (1000-1028 in MOSEK 11) to `license_error`,
  without hint text.
- **Tests.** Plain `TEST`s in `test/solvers/mosek.cpp` for the guard (fake api), the slot helper (derived
  probe), and a term-less row 0 >= 1 not being `optimal` until the shared case lands. **Done when** the MOSEK
  suites pass locally.

### WP3. Deferred notes

- **Port** as prose with `git show a1a9f11:<path>` pointers under iis.md's Deferred bullets: ordering
  (`deletion_filter.hpp:124-135, 174-205`, `test/iis.cpp:1264-1336`), seed outcomes
  (`iis_statistics.hpp:10-27`), verified seeds (`linear_iis.hpp:221-263`), `phase_budget`
  (`iis_limits.hpp:69-98`), MOSEK certificates (`mosek_lp.hpp:145-175`), the Clp ray (`clp_lp.hpp:596-606`),
  the elasticity loop (`elasticity_filter.hpp:43-88`), the benchmark (`benchmarks/iis.cpp:30-116, 148-253`).
  Batching and the public filter are no longer notes, since WP4 ports both.
- [ ] One note of 2-4 lines per item, with its return condition, plus two rules. Stop reasons come only from
  the caller's `iis_limits`. `has_dual_ray` gates on the cached status, which the N22 reset keeps honest after
  a filter run.
- [ ] Point at the archive tag of `a1a9f11`, and link the SoPlex proposal once opened.
- **Done when** every archived row of the file map has a note.

### WP4. Public deletion engine

- **Port** into `include/mippp/utility/deletion_filter.hpp`, namespace `mippp`:
  `deletion_filter.hpp:64-70, 87-123, 136-173, 206-242, 247-257` (oracle concept, initial proof and
  continuation, dormant batching, swap/pop pass, entry), with `:45-62` as the public result. From
  `algorithm/iis_limits.hpp`, `:41-54` (validation, saturating conversion to a deadline) without
  `iis_messages.hpp`, and `:60-67` (stop precedence). Tests into `test/deletion_filter.cpp`:
  `test/iis.cpp:352-391, 989-1007, 1213-1262, 1338-1409, 1411-1480`, `:1009-1022` inverted, `:1024-1049` on a
  fake clock.
- **Drop.** `deletion_filter.hpp:26-44, 55, 72-78, 124-135, 174-205` (statistics, ordering, archived by WP3),
  `iis_limits.hpp:39-40, 55-58, 69-98`, `iis_messages.hpp`, `test/iis.cpp:211-228, 1264-1336`, the
  absolute-deadline parts of `:958-987`, and the names `mippp::iis`, `result`, `options`, `termination`,
  `feasibility` and `input_order`. `deletion_filter.hpp:51` (`solve_count`) goes too (N19 f). WP5 takes
  `iis_limits.hpp:15-34`.
- [ ] Public: the entry `deletion_filter`, the verdict `deletion_verdict` (feasible, infeasible,
  inconclusive), the oracle concept `deletion_oracle`, and `deletion_filter_result`. The result lists the
  members in ascending order, with the `iis_outcome` and the `iis_reason`. The oracle is invoked as an lvalue
  with the active candidate indices. Monotonicity is a precondition, `inconclusive` never deletes a candidate,
  and oracle exceptions propagate unchanged. No member is reported before the initial trial proves
  infeasibility. Without batching, the oracle runs at most candidates + 1 times.
- [ ] Detail: the budget built once from `iis_limits`, the entry with the continuation from a known proof, the
  batch size defaulted to 1 (N3 b), and a `Clock` template parameter, `steady_clock` by default. This replaces
  the earlier `deletion_limits {max_solves, deadline, stop_token}`.
- [ ] Reasons per N2 (b), replacing the misattribution of `deletion_filter.hpp:119, 241`. After an
  inconclusive trial that ran, the reason is `inconclusive_trial`, or `time_limit` when the deadline has
  passed as the trial returns. `solve_limit` and `cancelled` apply only before a trial, where a stop request
  beats the deadline, which beats the solve count. A proof on the last permitted trial is complete. Tests:
  `max_solves = 1` with an inconclusive initial trial ends `undetermined` with `inconclusive_trial`. One
  candidate with `max_solves = 2` and an inconclusive singleton ends `not_proven_minimal` with
  `inconclusive_trial`.
- **Tests.** `mippp_test` in every CI job, ASan/UBSan included. The exhaustive test runs over the single pass
  and the dormant batch sizes through the detail entry, with the ordering loop removed, and builds its trace
  only on failure. **Done when** CI is green, the header is self-contained and includes only the standard
  library and the types header, and every public name carries `deletion_` or `iis_` (N19 a).

### WP5. IIS types, snapshot template and oracle

- **Port.** `algorithm/iis_limits.hpp:15-34`, split into `iis_outcome`, `iis_reason` and `iis_limits`. The
  validation assertions of `test/iis.cpp:958-987`, retargeted at `iis_limits::time_limit`.
  `test/iis_vectors.cpp:53-113` into `test/test_suites/iis_oracle.hpp`, keyed by case-local sides. `:27-51`
  (vectors, with attribution) into `test/test_suites/iis_vectors.hpp`, included by `test/iis_oracle.cpp` and
  `test/test_suites/iis_cases.hpp`. `:134-165` (self-test) into `test/iis_oracle.cpp`.
  `test/data/iis/HIGHS-LICENSE.txt`.
- **Drop.** `linear_iis_types.hpp`, `iis_statistics.hpp`, `iis_limits.hpp:26-27, 29-30` (absolute deadline,
  batch size), and `test/data/iis/README.md`, whose lines 18-21 claim a proof the witnesses do not give. A
  provenance comment in the vectors header replaces it.
- [ ] The std-only header `utility/iis_outcome.hpp`, which `model_concepts.hpp` and the engine include. It
  holds `iis_limits {max_solves, time_limit, stop_token}` defaulted to no limit (Q5 c), `iis_outcome`
  (`irreducible`, `not_proven_minimal`, `feasible`, `undetermined`) and `iis_reason` (`solve_limit`,
  `time_limit`, `cancelled`, `inconclusive_trial`). Outcome and reason are `enum class` (N19 e).
- [ ] `iis_status` tags next to `basis_status`, refinements deriving from `member`, and no possible-member tag
  (N1 a). A status concept that accepts `absent` and `member`. `lp_iis<I, T>` checks both `get_status`
  overloads against it, plus the outcome, the reason and the member counts. `model_iis_t<T>`, and a
  single-parameter `has_iis<T>` documented as "native routine" (Q1 b). No archetype (N10 a).
- [ ] The public snapshot template `iis_snapshot<Variable, Constraint, VariableStatus, ConstraintStatus>`,
  over `detail::handle_status_table<Status>` (N16 b). One byte per handle id, ids past the bound reading as
  alternative 0, and `iis_status::absent` required as alternative 0. Counts are built once. The N8 comment
  says that the snapshot describes the model as it was.
- [ ] The contract: `max_solves` counts `solve()` calls and prechecks are free (N7), the N2 attribution rules
  apply, a stop keeps the last proven subset, and zero members with `irreducible` means the background alone
  is infeasible. A native answer not proven minimal while no limit stopped it has no reason (N27 a).
- [ ] Oracle rules: `member_lower` and `member_upper` select one side, `member_both` both, each dropped in
  turn, a `member` row is kept or dropped whole. At most 6 variables per case.
- **Tests.** `test/iis_snapshot.cpp` and `test/iis_oracle.cpp` in `mippp_test`, and a `static_assert` that the
  snapshot satisfies `lp_iis`. **Done when** PR #3 is green and squash-merged, with these types public
  (N17 a). No model satisfies `has_iis` yet.

### WP6. Row-bound concept on Clp

Q3 (a) and Q6 (a) are ruled. Nothing is ported, since the pull request never relaxed a row in place.

- [ ] `has_modifiable_constraint_bounds<T, M = T>` next to its readable twin (`model_concepts.hpp:476-483`).
- [ ] `clp_lp` setters writing `rowLower` and `rowUpper` as `set_constraint_rhs` does (`clp_lp.hpp:413-445`),
  fetching the arrays on each call.
- [ ] `ModifiableConstraintBoundsTest`: each side on `<=`, `>=`, `==` and ranged rows, the ranged case gated
  on `has_ranged_constraints`, `infinity_frees_a_row_side`, warm re-solves, a sense read after re-tightening a
  one-sided row, none on a ranged row.
- [ ] The concept in `docs/reference/concepts.md`, the setters in `docs/solving/updates.md`, the
  features-table label.
- **Done when** the suite passes on Clp in CI.

### WP6a. Entity enumeration

Owner the maintainer, in its own pull request on main, in parallel with pull request #3 and WP6, before WP7
and WP13. Nothing is ported, since the pull request never enumerates a live model. N20 (a) sets the names, and
N21 (a) the contract: a snapshot of the call in increasing id order, random-access and sized, that holds no
reference to the model.

- [ ] Two concepts in `model_concepts.hpp` after `has_num_nonzeros` (`:250-252`), `has_enumerable_variables`
  and `has_enumerable_constraints`, taking `<T, M = T>` and requiring `std::ranges::sized_range` (N21). They
  stay out of `lp_model` until 2.0.
- [ ] `model_base` defaults. `variables()` returns `entity_range(variable(0), num_variables())`, with a
  `static_assert` that fires when a backend with `has_remove_variable` forgets to override it. `constraints()`
  returns `entity_range(constraint(0), num_constraints())` on every model, since none removes constraints.
- [ ] A protected non-template `_live_variables` helper in `remapping_model_base`, walking native ids through
  `_var_handle` and sorting only when remapped, plus one public line in `highs_base`, `gurobi_base` and
  `cplex_base`. A deducing-this hook there fails to compile from `highs_lp` without
  `friend remapping_model_base<...>` in the three remapping bases, and the non-template helper needs none.
  Overrides in `clp_lp`, masking `_free_variable_ids`, and in `dumb_lp`.
- [ ] Rename SCIP's protected `variables` and `constraints` members (`scip_milp.hpp:41-42`), about 32
  mechanical lines, since GCC 15 rejects the class once the using-declarations are added. Using-declarations
  of the `model_base` members in `cbc_milp`, `glpk_base`, `soplex_lp`, `mosek_base`, `copt_base`,
  `xpress_base` and `scip_milp`.
- [ ] Docs: two rows in `concepts.md`, and both names in its list of two-parameter concepts. One sentence in
  `docs/solving/updates.md`, and one paragraph in `docs/modeling/variables.md`.
- **Tests.** `EnumerableEntitiesTest` in `test/test_suites/enumerable_entities.hpp`, instantiated for all 19
  model fixtures and `dumb`, labelled "Enumerate variables and constraints". Its cases are
  `empty_model_lists_nothing`, `lists_every_addition_in_id_order`, `snapshot_ignores_later_additions`,
  `listed_handles_reach_their_entities`, `skips_removed_variables`, `lists_recycled_ids_in_order`,
  `solution_reads_every_listed_variable` and `removing_while_iterating_empties_the_model`. `LpFuzzyTest` gains
  an enumeration check, and `ColumnManagerTest` an oracle. **Done when** the suite passes in CI on Clp, Cbc,
  GLPK, HiGHS and `dumb`, and locally on the rest.

### WP6b. Status reset

N22 (B1) is ruled. Owner the maintainer, in its own pull request, before or after WP2. If WP2 has landed,
MOSEK's `reset_status()` also resets its slot cache, and otherwise WP2 adds that line when it lands. Nothing
is ported.

- [ ] `void reset_status() noexcept` on the 19 models and `dumb_lp`, one line each, plus MOSEK's slot cache
  if WP2 has landed.
- [ ] The concept `has_status_reset<T>`, which the free function's concept requires, and its `concepts.md`
  row.
- **Tests.** `LpModelTest.reset_status_reports_unknown`: solve, reset, read `unknown`, then re-solve to the
  same status. **Done when** the case passes in CI on Clp, Cbc, GLPK, HiGHS and `dumb`, and locally on the
  rest.

### WP6c. Readable row bounds on every model

Q6 (a) is ruled, and the maintainer's confirmation of 2026-09-27 extends it to every model. Owner the
maintainer, in its own pull request on main. It needs nothing and comes before WP7 in the order. It takes over
the readable half of the row bounds that WP10 to WP14 had planned. Nothing is ported.

- [ ] `has_readable_constraint_bounds` (`model_concepts.hpp:476-483`), that is `get_constraint_lower_bound`
  and `get_constraint_upper_bound`, on all 19 model classes and `dumb_lp`. `clp_lp` and `cbc_milp` already
  satisfy it (`clp_lp.hpp:493-498`, `cbc_milp.hpp:374-379`).
- [ ] Where the solver has no native ranged rows, as on Gurobi, the getters derive the bounds from the sense
  and the rhs. A `<=` row reads (`-infinity()`, rhs), a `>=` row (rhs, `infinity()`), and an `==` row (rhs,
  rhs), with the backend's own `infinity()`. The branch lives in the model's method, never in the IIS code.
  `gurobi_base` and `dumb_lp` derive from the sense and the rhs they already read (`gurobi_base.hpp:567-580`,
  `test/dumb_lp.hpp:377-380`). So does `cplex_base` (`cplex_base.hpp:535-544`), since MIP++ only creates `L`,
  `E` and `G` rows there (`cplex_base.hpp:39-44`).
- [ ] Backends with two-sided rows read both sides through symbols bound on main: `highs_base` through its
  private `_row_bounds` over `Highs_getRowsByRange` (`highs_base.hpp:519-526`), `glpk_base` through
  `glp_get_row_type`, `glp_get_row_lb` and `glp_get_row_ub`, and `mosek_base` through `MSK_getconbound` and
  its bound key. A missing side reads as the backend's `infinity()` with its sign.
- [ ] Getters that main does not bind: `SCIPgetLhsLinear` and `SCIPgetRhsLinear` on `scip_milp`, checked
  across the validated range from 8.0.4, and `SoPlex_getRowBoundsReal` on `soplex_lp`, one of the eight
  symbols of N11 (a), checked in 7.1.1 and 7.1.2, or the floor raised. `copt_base` and `xpress_base` bind a
  row getter too, none being bound on main, checked at their range floors, and follow whichever rule above
  fits how the solver stores a row.
- [ ] Modifiable row bounds still follow N30: only where the solver stores a row's two sides natively, so
  never on Gurobi. They come with WP6 on Clp, WP9 to WP14, and the optional commits of WP16.
- [ ] Docs: the `concepts.md` row of `has_readable_constraint_bounds` (`docs/reference/concepts.md:133`),
  which says "Satisfied by `clp_lp` and `cbc_milp`", names every model. The feature tables are regenerated,
  under the existing label "Read constraint bounds" (`tested_features_table.py:45`).
- **Tests.** `ReadableConstraintBoundsTest` (`test/test_suites/readable_constraint_bounds.hpp`), instantiated
  on main for Clp and Cbc only (`test/solvers/clp.cpp:21`, `test/solvers/cbc.cpp:25`), is instantiated on
  every model fixture and `dumb`. Its ranged case stays gated on `has_ranged_constraints`. **Done when** the
  suite passes in CI on Clp, Cbc, GLPK, HiGHS and `dumb`, and locally on the rest.

### WP7. Public free function

`compute_iis_by_deletion(model, limits)` in `include/mippp/utility/iis_by_deletion.hpp`, with its requirements
concept `iis_by_deletion_model`. It is neither a protected helper nor a deducing-this member of any backend.
It needs pull request #3, WP6, WP6a and WP6b.

- **Port** as rewrites over the enumerated handles: `deletion_workspace.hpp:99-111` (bitset diff),
  `linear_iis_model.hpp:73-92` (visit shape), `utility/iis_limits.hpp:34-59` (never loosen),
  `cold_model_workspace.hpp:64-72, 80-88` (zero objective, crossing scan), `prepared_linear_system.hpp:24-103`
  (a candidate per finite side). Stubs after `test/iis.cpp:1076-1089, 1119-1162, 1954-1981` and
  `test/iis_diagnostics.cpp:151-161, 178-197`, inverted.
- **Drop.** The rest of those files, with the factory, `set_minimization`, dummy and slack columns, and
  `elastic_lp.hpp`, `native_seed.hpp`, `linear_iis.hpp`. Q1 (b) also retires the protected live-handle hooks,
  `_compute_iis_by_deletion`, the per-backend `compute_iis(this auto & self)` overload and the remapping
  friend.
- [ ] The requirements concept: `lp_model`, the entity enumeration, readable and modifiable variable bounds,
  readable and modifiable row bounds (Q3 a), a readable objective, the status reset (N22), and a readable
  quadratic objective on a `qp_model`. The function never retypes a column, so `milp_model` needs no branch.
  It branches only through `if constexpr` on `qp_model` and `has_time_limit`. The third branch of the
  2026-09-27 plan, `detail::may_carry_background<M>` over the special-constraint and callback capabilities,
  was dropped on 2026-09-28 with the zero-solve claim it served (N36).
- [ ] The classifier, a visitor over `model_status_t<M>`, since `is_a<status::failed>` does not compile on
  `clp_lp`'s variant. Anything derived from `infeasible`, and exactly `infeasible_or_unbounded`, proves
  infeasibility. `optimal_infeasible_unscaled`, anything derived from `failed` whatever its flag, and
  `unbounded` are inconclusive. Anything else proves feasibility exactly when `solution_available` holds, so
  `time_limit{true}` is a proof.
- [ ] The guard, saving by value the finite sides of live candidates, the objective through `materialize()`
  with its offset, and the time limit only when a finite deadline is forwarded. `materialize()` is linear-only
  and no owning quadratic expression exists, so on a `qp_model` the guard copies the triples of
  `get_quadratic_objective()` into owned `detail` storage. It zeroes the objective and leaves the sense,
  verbosity, tolerances, other limits and the matrix alone. The first mutation waits for the first trial.
  `restore()` runs on every normal exit, attempts every item and rethrows the first error. The `noexcept`
  destructor restores only if `restore()` did not run. The status reset follows whenever a `solve()` ran.
- [ ] The applier over one bitset, the restore being apply(full set). With a finite deadline and
  `has_time_limit`, each trial gets `std::min(remaining, saved)` in that argument order, `saved` being read
  once. With the default limits, `get_time_limit` and `set_time_limit` are never called. The oracle returns
  inconclusive past the deadline. The builder keys the answer by the enumerated handles and folds sides into
  `member_lower`, `member_upper` or `member_both` (N1 a). Handles the enumeration skips read `absent`.
- [ ] The column-less precheck, free under N7 (a) and run whatever the limits: on a model with no live
  variable, the N6 arithmetic, with an exact comparison with 0 and the first side in enumeration order. Every
  crossed pair, of a variable or of a row, seeds the engine's continuation and two singleton trials decide, on
  every model type (N36 of 2026-09-28, amending N7 a2: the zero-solve claim on crossed variables and the
  background check are dropped). Constraints added through `native_model()` are outside the guarantee (N23 a).
- **Tests.** `test/iis_by_deletion.cpp` in `mippp_test`, replacing `test/iis_fallback.cpp`, since nothing is a
  fallback under Q1 (b). A scripted stub model with a lazy objective, optional `has_time_limit` and a QP
  objective; the indicator member that triggered the background check went with N36. A derived probe hides
  `solve()` and the setters and asserts per-trial invariants. Cases: every row of the classification
  (`failed{true}` inconclusive, `time_limit{true}` feasible, `infeasible_or_unbounded` a proof), a throw at
  trial k and during the restore, lazy and QP objectives restored, the column-less precheck and a crossed pair
  at zero budget with the status untouched, a crossed pair decided by two trials, and handles skipped by the
  enumeration reading `absent`.
- **Forwarding tests,** from the time-limit audit. Stub getters return +inf, `DBL_MAX`, 1e100, 1e75, 1e20,
  2 s, 0 s and NaN. Default limits make no `set_time_limit` call. Every write is `std::min(remaining, saved)`
  in that order and never above `saved`, and the last write equals `saved` after a throw. +inf and
  `duration<double>::max()` mean no deadline, and NaN and negative durations are rejected.
- **Done when** CI is green, and review finds no copy, factory, row removal, slack column, sense or verbosity
  change, and no branch on a backend type.

### WP8. Clp slice and `IisByDeletionTest`

It needs WP7 and the Clp secondary status fix.

- **Port** as case bodies of `test/test_suites/iis_cases.hpp`, run by `IisByDeletionTest<T>` in
  `test/test_suites/iis_by_deletion.hpp`, from `test/iis.cpp:1493-1527` (verbosity probe), `:1560-1575`
  (redundant), `:1677-1720` (crossing, ranged, `x = 0.5`), `:1737-1746` (column-less), `:1766-1803` (modified
  after an IIS), `:1983-2002` (32 redundant rows). From `test/iis_vectors.cpp:173-195, 214-253` (transforms,
  integer case). The families of `benchmarks/iis.cpp:30-116` at 4-8 rows. The filter-only cases stay in
  `IisByDeletionTest` itself: `test/iis.cpp:1590-1616, 1641-1674, 1854-1895` (budget sweep, stop requested
  beforehand).
- **Drop.** The backend list (`test/iis.cpp:1483-1491`), the policy, factory, elastic and seed cases, and the
  `PublishedIis` harness (`test/iis_vectors.cpp:167-172, 196-212`).
- [ ] No `clp_lp::compute_iis`, and `has_iis<clp_lp>` stays false. `IisByDeletionTest` is registered in
  `all.hpp` and instantiated for Clp in this PR, since a typed test body is only checked when instantiated.
  `dumb_lp` is not instantiated for now (N24). The fixture asserts the free function's concept.
- [ ] New cases: one side of an equality row, `y = 0` with `x + y = 1.5`, modified after an infeasible solve,
  answers unchanged after a later removal (N8), re-solve to the previous result and getters, a sign flip of
  one-sided rows, a 0 s budget. The column-less case asserts the N6 precheck answer. The status case asserts
  `unknown` after a run that solved, and the pre-call status after a precheck-only run. The integer-only case
  evaluates its witnesses and accepts the precheck or the trial path, since SCIP may round fractional bounds
  of integer columns into a crossing.
- [ ] A derived probe, the deduced `M` of the free function, asserting per-trial invariants (zero objective,
  sense and verbosity kept, time limit never above the caller's, solve count) and the restore after an
  injected throw. No `TYPED_TEST_P` is named after an IIS tag, and oracle and probe helpers are fixture member
  functions, not generic lambdas.
- [ ] The `IisByDeletionTest` label, and the `concepts.md` row of the free function's concept, saying that
  zero members with `irreducible` means the background alone is infeasible.
- **Tests.** The Clp, sanitizer and macOS jobs. A backend rejecting crossed bounds or term-less rows at build
  time gets a `GTEST_SKIP` keyed on the error. **Done when** `IisByDeletionTest` passes on Clp in CI,
  including the previous-result case.

### WP9 to WP14. Widening the free function

Each package adds modifiable row bounds and any other missing capability, instantiates every capability suite
newly satisfied, then `IisByDeletionTest`. None adds `compute_iis()`, the enumeration, a status reset or
readable row bounds, which WP6a, WP6b and WP6c already provide. Nothing is ported.

- [ ] **WP9, Cbc.** Setters through `Cbc_setRowLower` and `Cbc_setRowUpper`, both bound
  (`cbc_api.hpp:51-52, 139-140`). `setRowLower` is already used at `cbc_milp.hpp:340`. It runs the first real
  time-limit forwarding, so it needs the time-limit fix, and the integer-only cases. Done when green in CI.
- [ ] **WP10, HiGHS.** Modifiable row bounds in `highs_base` through `Highs_changeRowBounds`
  (`highs_base.hpp:562-596`), over the readable ones of WP6c, for `highs_lp`, `highs_milp` and `highs_qp`.
  N12 (b) lands in its own commit, replacing the `greater_equal` of `highs_base.hpp:553-559`. `set_objective`
  clears the `highs_qp` Hessian (`highs_qp.hpp:47-51`), and the free function saves it through `qp_model`, not
  through backend code. It needs WP6c and the `psolstatus` fix. Done when green on Linux, macOS and Windows.
- [ ] **WP11, GLPK.** Modifiable row bounds through `glp_set_row_bnds`, switching the type like
  `_set_col_bnds` (`glpk_base.hpp:169-181`), over the readable ones of WP6c. Limits act between solves only,
  and `glpk_milp` trials are cold. It needs WP6c and the GLPK status fix. Done when green in the GLPK job.
- [ ] **WP12, MOSEK.** Modifiable row bounds through `MSK_chgconbound`, with finite = 0 on an infinite value,
  over the readable ones of WP6c, which read `MSK_getconbound`. The free function and `reset_status()` handle
  forwarding and the status, so nothing else is MOSEK-specific. It needs WP2, WP6c and the time-limit fix.
  Done when the suites pass locally.
- [ ] **WP13, SCIP.** Rebased onto WP6a, which renames SCIP's members, and onto WP6c, which binds
  `SCIPgetLhsLinear` and `SCIPgetRhsLinear`. Bind the two side setters `SCIPchgLhsLinear` and
  `SCIPchgRhsLinear`, checked across the validated range (8.0.4 up to 10.0.4), then add modifiable row bounds.
  `scip_milp` keeps its `BINARY` columns, and neither the free function nor the backend changes to conform to
  SCIP (N25). Run the N14 probe on a column typed `BINARY` (`scip_milp.hpp:270`) only to characterize the gap:
  whether SCIP rejects or clamps a relaxed bound, and what the filter then reports. Document the gap under
  "Notable current limitations" in `docs/solvers/index.md`, and WP17's pages link it. The restore needs no
  `_solved` handling. Done when small cases pass locally on SCIP 8.0.4, 9 and 10, or the compatibility matrix
  covers 8.0.4.
- [ ] **WP14, SoPlex.** Per N11 (a), check the seven symbols of 7.1.3 and 8.0.2 that WP6c leaves (bound and
  objective accessors, row side setters) in 7.1.1 and 7.1.2, or raise the floor. WP6c binds the eighth,
  `SoPlex_getRowBoundsReal`. Bind the seven, and add variable bounds, a readable objective and modifiable row
  bounds. It needs WP6c, WP13, and the time-limit fix for the SoPlex setter. Done when the suites pass locally
  on 7.1 and 8.0 with `MIPPP_SOPLEX_LIBRARY` set.

### WP15. HiGHS native and `IisTest`

It needs pull request #3, and WP10 with WP6a for the both-paths case only. N29 (a) rules the `iis_time_limit`
copy, and N34 (a) the elapsed-time test of the stop mapping.

- [ ] `Highs_getIis` in `HIGHS_OPTIONAL_FUNCTIONS` (`highs_api.hpp:325-334`). It is absent up to 1.11.0,
  checked on 2026-09-27. `compute_iis()` on `highs_lp` and `highs_qp` only, with
  `static_assert(!has_iis<highs_milp>)`, since the routine analyzes the relaxation of a MIP.
- [ ] The member sets the status to `unknown` first (N15) and takes no `iis_limits` (N9). Below 1.14.0
  (N26 a3) the member throws `solver_error`, naming the loaded version, the library path and the floor, with
  no fallback (Q1 b). An empty `library_version()` requires the symbol and assumes the newest regime. It
  saves, sets and restores `iis_strategy` (6) through a small RAII helper, and `iis_time_limit` too, copied
  from `get_time_limit()` for each call and restored afterwards (N29 a). `time_limit` alone had no effect on
  `Highs_getIis` on 1.14.0 and 1.15.1 (measured). The copy never goes into `set_time_limit()`, since releases
  before 1.14 reject the option, measured as -1 on 1.10.0.
- [ ] The row arrays are sized by the row count despite the header (`highs_c_api.h:2428-2442`). Members are
  decoded from the bound codes (Free `absent`, Lower `member_lower`, Upper `member_upper`, Boxed
  `member_both`), with columns through `_var_handle`. "Maybe" entries give `not_proven_minimal` (N1 a), and a
  `kWarning` return gives `undetermined`. An empty answer is `feasible` when the model status is optimal or
  unbounded and `undetermined` otherwise (N26). A -1 return is `undetermined` with `time_limit` when the time
  measured around the call reached the limit copied into `iis_time_limit`, whatever the model status, and
  throws `solver_error` otherwise (N34 a, amending N26). This covers the -1 with model status 8 that a stop in
  the elasticity filter gives on 1.14.0 (measured). A "maybe" answer has no reason unless the time limit
  stopped it (N27 a), which the same elapsed-time test decides, since the C API exposes no IIS status. The
  post-check re-solves with no limit (1.14.0 and 1.15.1 sources), so a stopped call may return late, on 1.15.1
  once after 16.5 s under a 1 s limit (measured). In the 1.15.1 sources, a failed post-check returns
  `kWarning` before the status arrays are filled (`HighsInterface.cpp:1898-1906`), so the wrapper initializes
  them (inferred). No precheck runs (N28 a).
- [ ] Probe whether `Highs_getIis` keeps the held solution and whether the status agrees with it (N15), and
  record the result in iis.md. The routine re-solves an unsolved model, leaving model status 8 or 13, and
  resets the run clock (measured on 1.14.0 and 1.15.1).
- **Tests.** `IisTest` in `test/test_suites/iis.hpp` over the shared case bodies, creating `iis_cases.hpp` if
  WP8 has not. In `test/solvers/highs.cpp`, the fixtures `highs_lp_iis_test` and `highs_qp_iis_test`, whose
  `SetUp` skips below 1.14.0 on `library_version()`, and
  `TEST(HiGHS_lp, compute_iis_below_native_floor_throws)`. The both-paths case `both_paths_find_valid_iis`
  calls the free function directly on models meeting both concepts, and the oracle validates both answers.
  **Done when** CI is green and shows HiGHS below and above the floor.

### WP16. Gurobi, CPLEX, Xpress and COPT native

Each leaf member sets the status to `unknown` first (N15) and takes no `iis_limits` (N9). On Gurobi, CPLEX,
Xpress and `copt_milp`, the model's time limit bounds each call as a fresh budget with no code, since their
routines already read the parameter that `set_time_limit` writes (N29 a). CPLEX and Xpress name a limit stop
in their answer, and Gurobi and COPT need a test of the time measured around the call, like the one N34 (a)
rules for HiGHS (N29 evidence). It confirms its entry point at the range floor, translates through
`_var_handle`, and restores what it sets (Xpress `IISOPS`, Gurobi `IIS*Force`) through its own RAII helper. It
runs no column-less precheck (N28 a), and gives a reason only on a stop attributed to the time limit (N27 a).
Modifiable row bounds on COPT, CPLEX and Xpress are optional commits under N30 (b), over the readable ones of
WP6c, each only where the solver stores a row's two sides natively, which the commit confirms first. Gurobi
gets none, since its ranges add a slack column. A bounded native call on `copt_lp` needs fix 10 (N35 a): under
N29 (a), a native `copt_lp` IIS is then bounded by the model's time limit, like `copt_milp`. Without
modifiable row bounds, the both-paths check runs on HiGHS only. It needs WP15 and the native-id fix, but no
longer WP8. Nothing is ported.

- [ ] Probes on each solver: the answer on a feasible model (Gurobi error 10015, CPLEX conflict status 30,
  COPT code 3), and whether the held solution survives and the status agrees with it (N15). The time-limit
  probes of 2026-09-27 answered the rest on the probed releases (N29 evidence), so only Gurobi 10 and COPT 7.2
  still need the time-limit check at their floor: the routines of Gurobi, CPLEX, Xpress and `copt_milp` stop
  under the model's time limit, and only CPLEX and Xpress name the stop. Gurobi overwrites `Status` and
  `Runtime`, and COPT leaves `Status`, `LpStatus` and `HasLpSol` untouched (measured). The results go into
  iis.md.
- [ ] **Gurobi.** `GRBcomputeIIS` with the three `IIS*Force` attributes at 1, whose effect is unprobed.
  `IISConstr`, `IISLB`, `IISUB`, rows sided by sense, equality rows `member`, and `IISMinimal` = 0 giving
  `not_proven_minimal`. A forced-only answer is irreducible with zero members. After return code 0,
  `IISMinimal` is read without throwing: error 10005 means that a stop left no subsystem, which is no answer,
  never an empty IIS (measured). `IISMinimal` = 0 also follows numerical trouble with no limit (measured on
  13.0.2).
- [ ] **CPLEX.** `CPXrefineconflictext` and `CPXgetconflictext`, a group per row and bound side, indicators
  outside. Status 31 is irreducible and 30 feasible. Aborts and "possible" flags give `member_*` under
  `not_proven_minimal` (N1 a). `CPXgetstat` is read right after the call. Statuses 32 to 39 end the refinement
  early (documented), and 33, 34 and 39 were measured on 22.1.1, and 33 and 35 on 22.1.2. Status 33 names a
  time-limit stop. An abort with no member and no possible member, measured as status 33 under a limit of 0 s
  on both releases and of 0.001 s on 22.1.1, is no answer, never an empty IIS.
- [ ] **Xpress.** `IISOPS` integrality and special bits, `XPRSiisfirst`, `XPRSgetiisdata`, `I` entries
  dropped. Row `L` is a `<=` side (`member_upper`), `G` is `member_lower`, `E` is `member_both`, to confirm in
  45.1. `IISSOLSTATUS` gives the completion status, but not alone, since it reads 0 when a stop hits the
  initial LP. `p_status` 3 is a stop by a limit or an interrupt, and `NUMIIS` 0 after it means no answer
  (measured on 45.01 and 47.01). On 45.01 the default-`IISOPS` MIP IIS of a 4-row, 26-column market split
  aborted with SIGABRT, and `IISOPS` = 17 did not (measured), so the floor check of todo 6.2 runs that case
  with the wrapper's bits.
- [ ] **COPT.** `COPT_ComputeIIS` and the four `Get*IIS` calls, per side on `copt_lp`, equality rows `member`
  on `copt_milp`, `IsMinIIS`. `HasIIS` is checked before any getter, and 0 means no answer. A feasible model
  returns the generic code 3, so the wrapper confirms feasibility rather than mapping the code blindly. The
  flagged counts are compared with `IISRows` and `IISCols`: on a time-limited MIP IIS equal to the whole
  model, 101 rows and 200 columns, the getters flagged two rows and one column (measured on 8.0.5). On a
  mismatch the wrapper reports `undetermined`, never the getters' output (inferred). One `IsMinIIS` = 1 answer
  on a 20-column, 10-row binary model was feasible when re-solved (measured, unexplained), which needs its own
  probe before `copt_milp` claims `irreducible`.
- [ ] **Modifiable row bounds, optional (N30 b).** Per solver, first confirm that rows are two-sided natively:
  COPT per its documentation, CPLEX and Xpress ranged rows, all unprobed. Then add modifiable row bounds over
  the readable ones of WP6c, switching those getters to the native sides where WP6c derives them from the
  sense and the rhs, instantiate `ModifiableConstraintBoundsTest` and `IisByDeletionTest`, and run the
  both-paths check. Never on Gurobi.
- **Done when** `IisTest` passes locally at the floor and the latest release, with indicators and a removed
  variable on Gurobi and CPLEX.

### WP17. Documentation

It needs WP8 to WP11 and WP15.

- **Port** prose from `docs/iis.md:3-7, 21-22, 52-55, 59-76, 248-287, 900-903` (definition, the `x == 0.5`
  example on a `*_milp` model, reading table, limits, reference) and the warnings of
  `iis_report.hpp:141-161, 267-269`.
- [ ] `docs/solving/infeasibility.md`, on reading an IIS: the `has_iis` gate and the free function, the
  consumer loop, the tags of each path, outcomes and reasons, `*_lp` against `*_milp`, snapshot validity (N8),
  zero members, verbosity. The status is `unknown` after a run that solved, and after every native call for
  now. A per-model path table shows `has_iis`, the free function's concept and the HiGHS native floor. Links
  to repository files use absolute GitHub URLs. The page restates for IIS the warning of
  `docs/solvers/index.md` on the native handles (N23): background added through `native_model()` is outside
  the guarantee, and the free function's guard and the entity enumeration do not see native changes. It links
  the SCIP gap that WP13 documents (N25). Its paragraph on the time bounds of native calls states per backend
  that the budget is per call, which other model limits also stop the routine, and that a stop may return late
  or with no answer (N29 a). It states that the model's time limit bounds a native `copt_lp` call as on
  `copt_milp`, through fix 10 (N35 a).
- [ ] A page under Algorithms in `zensical.toml`, next to column generation, on the deletion filter. For the
  engine, the oracle contract, monotonicity, limits and reasons. For the free function, its requirements, what
  is saved and never touched, the native-handle warning, and one trial overrunning the budget on models
  without `has_time_limit`. It says, as `concepts.md` does, that a failed restore leaves the model data
  unspecified and the status `unknown`.
- [ ] `concepts.md` rows for every new concept. The labels `("IisTest", "IIS, native")` and
  `("IisByDeletionTest", "IIS, deletion filter")`. Fix `tested_features_table.py:31-34` so `highs_qp` rows
  appear, then regenerate the tables. Links from `status-and-limits.md` and `solutions.md:85`, and the README
  row at :187 and `coming-from.md:123` once IIS ships on the CI backends.
- **Done when** `zensical build --clean` passes and every snippet compiles.

## Standalone fixes on main

They are listed in the order of the work package table.

- [ ] **Moves drop the status.** `_status(other._status)` in the move constructors of `clp_lp`, `cbc_milp`,
  `scip_milp` and `soplex_lp` (from `soplex_lp.hpp:66-67` at `a1a9f11`), plus
  `LpModelTest.move_preserves_status`. The same PR initializes HiGHS's `psolstatus` (`highs_lp.hpp:68`,
  `highs_milp.hpp:119`, `highs_qp.hpp:221`), which errors leave unwritten. It also adds
  `LpStatusTest.constant_row_without_variables_is_not_optimal` on a term-less row 0 >= 1, after probing each
  backend at hand, fixing or gating any failure with a comment. Maintainer, now.
- [ ] **GLPK status mapping.** `glpk_lp.hpp:79-103` maps `GLP_INFEAS` to `infeasible` and lets `glp_simplex`
  errors fall through. Own PR before WP11, noted as a mapping change in the release notes of v1.1.0, since
  main has no changelog file and releases are described on GitHub.
- [ ] **Native ids after removal.** `gurobi_milp.hpp:48-56, 69` and `cplex_milp.hpp:82` pass `v.id()`. Own PR
  with two `RemoveVariableTest` cases, before WP16.
- [ ] **Clp secondary status (N13 a).** Bind `Clp_secondaryStatus` next to `clp_api.hpp:75` and `:137`. The
  symbol exists at tag `releases/1.17.4` and in every Clp library at hand, so the binding is mandatory. Map
  status 0 with secondary 2 or 4, which leave unscaled primal infeasibilities, to
  `optimal_infeasible_unscaled` in `_get_status()` (`clp_lp.hpp:535-544`), widening the variant. Secondary 3
  leaves only dual infeasibilities and stays `optimal`. Add a note on the wider public variant to the release
  notes of v1.1.0, and a test. Maintainer, before WP8.
- [ ] **Time-limit contract (N4 A, N32 a, N33 b).** Maintainer, in its own PR now, needed by WP9, WP12 and
  WP14. The MOSEK getter maps a negative value to `std::numeric_limits<double>::infinity()`
  (`mosek_base.hpp:431-436`), porting the getter half of `mosek_base.hpp:471-485` at `a1a9f11` with the
  author's credit. SoPlex silently refuses values above 1e100 and negative ones, keeps its previous limit and
  still reports the new one, so after `set(0 s)` then `set(+inf)` a solve still stops with `time_limit`. Its
  setter now throws on a negative value, then writes and records `std::min(t, 1e100)`
  (`soplex_lp.hpp:268-273`). Under N33 (b), the Cbc setter throws `solver_error` on a negative value
  (`cbc_milp.hpp:425-427`), and the getter stays transparent. Under N32 (a), the SCIP setter clamps to 1e20
  (`scip_milp.hpp:419-421`) and the CPLEX setter to 1e75 (`cplex_base.hpp:554-556`). Five `TimeLimitTest`
  cases go after `set_get_time_limit` in `test/test_suites/time_limit.hpp`, registered before
  `interrupts_long_solve`: `fresh_time_limit_is_unlimited`, `unlimited_time_limit_round_trips`,
  `lifted_time_limit_takes_effect`, `negative_time_limit_is_never_read_back` and
  `forwarded_time_limit_restores_exactly`. The audit ran the Cbc getter of N33 (a) with the four other backend
  changes, and all six cases passed on all 15 classes. The ruled Cbc setter was not run, and
  `negative_time_limit_is_never_read_back` accepts a setter that throws. Docs: a contract paragraph in
  `docs/solving/status-and-limits.md` after line 99, and a corrected SoPlex sentence at
  `docs/solvers/index.md:27`.
- [ ] **Loader message (N18 a).** `solver_library.hpp:300-301` names `LD_LIBRARY_PATH` everywhere. Port the
  platform variable, precedence and quoting of `solver_library.hpp:129-170`, `diagnostic_text.hpp:15-44` and
  `test/dynamic_library.cpp:203-239, 243-245` at `a1a9f11`. Only the `MIPPP_<KEY>_LIBRARY` value is printed,
  never the search-path variable's, so the test drops that check. No `version_warning_help` or license text.
  Offered to the author.
- [ ] **Throwing destructors.** `copt_base.hpp:61-63`, `cplex_base.hpp:63-64`, `gurobi_base.hpp:79-80` and
  `xpress_base.hpp:69-71` call `check()`. Own PR after WP2, reusing its guard.
- [ ] **Time limit on `copt_lp` (N35 a).** Move `set_time_limit` and `get_time_limit` from `copt_milp`
  (`copt_milp.hpp:164-170`) into `copt_base`, and instantiate `TimeLimitTest` for `COPT_lp`, before WP16. The
  five cases of the time-limit fix then run on `copt_lp` too. `COPT_SolveLp` honors `TimeLimit`, and
  `copt_lp.hpp:45` already maps the timeout (measured on 8.0.5). `docs/solving/status-and-limits.md:87`, which
  lists COPT under `has_time_limit`, becomes true of both classes. Under N29 (a), a native `copt_lp` IIS is
  then bounded by the model's time limit, like `copt_milp`. By license.
- [ ] **SCIP exception type,** optional. `scip_milp::check` (`scip_milp.hpp:137-140`) throws
  `std::runtime_error` where other backends throw `mippp::solver_error`. The time-limit audit found it outside
  N4, for the maintainer to accept or not.
- [ ] **Elsewhere.** MOSEK's license codes and column-less status in WP2, and HiGHS ranged rows (N12 b) in
  WP10.

## The pull request's ideas

Every idea is placed.

| Idea | Placement | Where and why |
| --- | --- | --- |
| `batched-deletion` | now, dormant | WP4 under N3 (b): `deletion_filter.hpp:136-170` and the tests `test/iis.cpp:1338-1409` at `a1a9f11`, behind a batch size of the engine's detail entry, defaulted to 1. It is outside `iis_limits` and the public engine, and the free function never sets it. |
| `candidate-priority-order` | later | WP3 note. It returns with preference setters on the snapshot or with weight ordering, as an internal comparator of the engine's detail entry, never as public order types. |
| `public-deletion-filter` | now | Q1 (b): WP4 publishes the engine over a user oracle in `utility/deletion_filter.hpp`, in pull request #3, and WP7 adds the free function over models, named per N19 (a). Credited to the author in the reply. |
| `possible-member-tag` | rejected | N1 (a): retained, untested and native "possible" members get `member_*` under `not_proven_minimal`. |
| `inconclusive-trial-reason` | now | N2 (b): `iis_reason::inconclusive_trial` in WP5, attribution rules in WP4, `test/iis.cpp:1009-1022` inverted. |
| `continuation-from-known-proof` | now | WP4, as the detail entry that starts from a proven set, first used by WP7 on every crossed pair (N36). |
| `crossing-sides-shortcut` | now, as two trials | WP7. Every crossed pair, variable or row, is the known proof and two singleton trials decide it through the continuation (N36 of 2026-09-28, amending N7 a2); the zero-solve claim on crossed variables is dropped. |
| `column-less-model-arithmetic` | now | WP7 precheck under N6 (b), with an exact comparison with 0. Native wrappers do not run it by default (N28 a), and one that fails the shared case may call it. |
| `mosek-empty-model-status` | now | WP2, with the shared `LpStatusTest` case in the move-fix pull request. |
| `diff-based-trial-application` | now | WP7 applier over the active set, where the restore applies the full set. |
| `restore-guard-exception-policy` | now | WP7 guard: `restore()` attempts every item and rethrows the first error, the noexcept destructor restores only if `restore()` did not run, then the status reset of N22 when a `solve()` ran. WP17 and `concepts.md` document a failed restore. |
| `overall-time-budget` | now | Q5 (c): `iis_limits::time_limit`, turned into one deadline in WP4, forwarded by WP7 as `std::min(remaining, saved)` only with a finite deadline. WP17 documents the overrun on models without `has_time_limit`. |
| `limits-contract-for-completion-status` | now | WP5 contract with the N2 and N7 rules, WP4 engine tests, WP7 stub tests, WP8 suite cases, WP17 page. |
| `soplex-time-limit-runtime-probe` | rejected | Capabilities are compile-time. `SoPlex_setRealParam` exists in every validated release (floor 7.1.1), and the free function forwards a limit only with a finite deadline, so no probe is needed. |
| `verified-ray-seed` | later | After `has_dual_ray`, as an `if constexpr(has_dual_ray<M>)` step of the free function on `*_lp` models: one verifying trial, then the engine's continuation. |
| `weight-ordered-deletion` | later | After candidate ordering and ray seeds, if the out-of-repository harness shows fewer solves. |
| `bound-pruning-by-column-multipliers` | later | With `has_dual_ray`, bound multipliers as a variable mapping. |
| `shared-phase-budget` | later | With the first seeding phase. Stop reasons come only from the caller's `iis_limits`, a rule WP3 records. |
| `seed-outcome-checklist-for-dual-rays` | later | Written by WP3 as a checklist for suite cases with seeding, never an enum. |
| `dual-ray-accessors` | later | `has_dual_ray<T, M = T>` over handle mappings on `*_lp` models, gated on the cached `_status`, which the N22 reset keeps honest after a filter run. |
| `elastic-engine-over-native-feasrelax` | later | A `has_feasibility_relaxation` capability, after probing that each routine leaves the model unchanged. `GRBfeasrelax` modifies the model, and a `GRBcopymodel` workaround is a copy needing its own ruling. |
| `upstream-soplex-farkas` | later | Proposed upstream by the author, bound once a SoPlex release ships it. |
| `optional-symbol-stock-test` | now | WP15: version-keyed HiGHS fixtures skip below the 1.14.0 native floor (N26 a3), and a plain `TEST` expects the throw there. CI covers 1.9.0, 1.13.0 and 1.15.1 with no extra environment variable, Windows pinned and Linux and macOS as their package managers ship. |
| `lp-relaxation-domain` | rejected | `*_milp` explains the MIP, and `highs_milp` gets no native routine. WP17 says to rebuild on an `*_lp` model to explain a relaxation. |
| `time-limit-unlimited-contract` | now | N4 (A), N32 (a) and N33 (b): the time-limit standalone fix, with the MOSEK getter, the SoPlex setter, the SCIP and CPLEX clamps, the throwing Cbc setter and five `TimeLimitTest` cases. |
| `nonthrowing-destructors-all-backends` | now | Standalone pull request after WP2, reusing its guard, for COPT, CPLEX, Gurobi and Xpress. |
| `move-preserves-status` | now | Standalone fix for Clp, Cbc, SCIP and SoPlex, with `LpModelTest.move_preserves_status`. |
| `loader-help-current-values` | now, trimmed | N18 (a): standalone loader fix with the platform's search variable, one precedence sentence and the quoted `MIPPP_<KEY>_LIBRARY` value. No dumps of search-path variables, no license text, no `version_warning_help`. |
| `license-help-per-backend` | rejected | N18 (a): license hint text is rejected. MOSEK's license codes are fixed as a bug in WP2's own commit. |
| `iis-error-context` | rejected | Exceptions propagate after the restore, and trials never grow the model. |
| `run-statistics` | rejected | Member counts remain the only summary of the snapshot. Under N19 (f), the engine's result carries no call count either, since an oracle can count its own calls. |
| `empty-iis-background-infeasible` | now | WP5 contract, WP4 zero-candidate test, the `concepts.md` rows of the free function's concept (WP8) and of `has_iis` (WP15), the WP16 mapping of forced-only answers, and the WP17 page. |
| `exhaustive-monotone-oracle-engine-test` | now | WP4, in `test/deletion_filter.cpp`, over the single pass and the dormant batch sizes through the detail entry, with the ordering loop removed. |
| `fourier-motzkin-exact-oracle` | now | WP5, in `test/test_suites/iis_oracle.hpp`, shared by `IisTest` and `IisByDeletionTest`. |
| `published-highs-vectors` | now | WP5 data, a shared case run first by WP8, and the WP15 both-paths case. |
| `transform-invariance-cases` | now | Shared case bodies in `iis_cases.hpp`, first run by WP8, on the vector case only. |
| `integer-only-bound-conflict` | now | Shared case body with evaluated witnesses, accepting the precheck or the trial path. First run on Cbc in WP9 through `IisByDeletionTest`, and natively in WP16. |
| `budget-sweep-property` | now | `IisByDeletionTest` only, since native calls take no `iis_limits` (N9). Accounting per N7 (a): prechecks are free. |
| `probe-model-tests` | now | WP7 stubs and WP8 suite. The derived probe is the deduced `M` of the free function, so its hidden `solve()` runs in every trial. Helpers are fixture member functions. |
| `derived-probe-for-protected-helpers` | now, narrowed | Only WP2's slot helper. The WP15 both-paths case calls the free function directly, and no protected IIS helper remains. |
| `optimizer-run-counting` | rejected | N5 (b): the optimizer-count and interior-point tests are dropped with their binding, and the optimizer-count item of iis_todo.md step 1 is struck. |
| `benchmark-families-as-suite-cases` | now | Shared case bodies at 4 to 8 rows, first run by WP8. |
| `out-of-repo-strategy-benchmark` | later | A separate repository like `mippp_nqueens`, when a caller for the dormant batching is considered. |
| `per-model-path-table-in-docs` | now | WP17: per model, whether `has_iis` holds, whether the free function's concept holds, and the HiGHS native floor. Updated by WP8 to WP16. |
| `human-readable-report` | rejected | Replaced by iis.md, and its warnings become prose in WP17. |

## Confirmed defects

The refuted report, on the CPLEX 1016 license help, needs nothing. The time-limit probes of 2026-09-27 found
vendor behavior outside the pull request, handled in WP15 and WP16 rather than here: the unlimited post-check
of HiGHS, which overran on 1.15.1, the COPT getters disagreeing with `IISRows` and `IISCols`, the COPT
`IsMinIIS` answer that re-solved feasible, and the Xpress 45.01 abort. On main,
`docs/solving/status-and-limits.md:87` lists COPT under `has_time_limit` without saying `copt_milp` only,
which fix 10 makes true of both classes (N35 a).

| Defect | Resolution |
| --- | --- |
| [engine-limits] A failed status with a solution flag counts as a feasibility proof | WP7 visitor classifier, which compiles over every backend's status variant, `failed` inconclusive whatever its flag |
| [engine-limits] A phase's local solve cap is reported as the user's solve limit | dropped with `phase_budget`, rule kept by WP3 |
| [engine-limits] An inconclusive trial in a completed pass is blamed on the solve budget or cancellation | WP4 under N2 (b), `inconclusive_trial`, with `test/iis.cpp:1009-1022` inverted |
| [oracles-orchestration] Failed solves that carry a solution flag are taken as feasibility proofs | WP7 visitor classifier |
| [oracles-orchestration] infeasible_or_unbounded and primal_and_dual_infeasible discarded although every trial has a zero objective | WP7 visitor classifier, both are proofs |
| [oracles-orchestration] Seeded runs report solve_limit when the initial proof is merely inconclusive | dropped with seeding, the WP4 budget-1 test pins the initial trial |
| [accelerators-certificates] SoPlex Farkas symbols bound with an invented ABI inside upstream's SoPlex_ prefix | dropped, upstream first |
| [accelerators-certificates] Clp ray accessor checks solver state, not the cached status, and can return a stale ray | archived, the WP3 note requires the cached status, which the N22 reset keeps honest after a filter run |
| [accelerators-certificates] clp_lp move constructor drops _status (pre-existing on main) | standalone fix |
| [backend-changes] MOSEK optimize-once regression tests never run by default | counting tests dropped under N5 (b), with the binary |
| [backend-changes] The destructor assertion does not test the fix | `static_assert` dropped, fake-api test kept (WP2) |
| [backend-changes] The version-warning help always prints MIPPP_NO_VERSION_WARNING as not set | dropped with `version_warning_help` under N18 (a) |
| [backend-changes] Pre-existing on main: moving a Clp, Cbc or SCIP model drops its status (the PR fixed only SoPlex) | standalone fix, four backends |
| [backend-changes] Pre-existing on main: MOSEK raises license_error only for an expired license | WP2's own commit under N18 (a) |
| [tests] IIS test selection is a cached configure-time snapshot of MIPPP_REQUIRED_SOLVERS covering only four solvers | binary dropped, `IisTest` and `IisByDeletionTest` instantiated per solver file |
| [tests] Classification pins failed{true} as a feasibility proof | stub table test in `test/iis_by_deletion.cpp` (WP7), inverted |
| [tests] Wall-clock deadline test with a history of flakiness | the engine's `Clock` parameter and a fake clock in `test/deletion_filter.cpp` (WP4), suite budgets of 0 s only |
| [tests] IIS binary escapes the sanitizer job and the TEST_SOURCE build reduction | binary dropped, tests in `mippp_test` and solver files |
| [tests] Filter hard-codes typed-test positions | dropped with the filter |
| [tests] Vacuous 'independent proof' in the integer-bound vector | the shared case body evaluates the witnesses (WP8), WP5 drops the README claim |
| [docs-reporting-build] Status audit falsely says the README distinguishes utility from planned IIS support | audit dropped, gap table quoted in the thread |
| [docs-reporting-build] README roadmap drops IIS while the PR's docs say model-level and native IIS remain planned | reverted in PR #3, WP17 updates it |
| [docs-reporting-build] Status audit credits the PR with the MOSEK TimeLimitTest registration that main already has | audit dropped, WP1 ticks the item |
| [docs-reporting-build] docs/iis.md links point outside the docs directory and will 404 on the published site | pages replaced, WP17 uses absolute URLs |
| [docs-reporting-build] Design notes list MOSEK items that main already fixed | WP1 corrections |

## Reply to the author

- **Thank you, and what lands with your name.** The deletion engine and its exhaustive test, now a public
  algorithm as your `public-deletion-filter` idea proposed, with its batching ported dormant. The limit
  semantics and the inconclusive-trial reason, the diff-based updates, the crossing and constant-row ideas,
  the vectors and oracle, the MOSEK guard, slot selection and time-limit getter, the SoPlex move fix, and the
  case list.
- **The gap, plainly.** At `a1a9f11` the branch has the shape iis.md lists as "Replaced": `linear_system` with
  a model factory, slack columns, public `mippp::iis::{result, options, member}`, a separate `mippp_iis_test`.
  It has no model-level `compute_iis()` (so no `has_iis`), snapshot, row-bound capability or native routine,
  as your own `docs/iis.md:9-12` says. Nor does it have the entity enumeration that the public free function
  now needs. This is the API contract, not style.
- **What happens to PR #3.** I would like to reshape it in place, then squash-merge it (N0 a). That means
  pushing, without force, a cleanup, the engine as the public algorithm `mippp::deletion_filter` with your
  tests and dormant batching, and the IIS types with your vectors and oracle. The types land public, since no
  tag is cut before WP8. The squash-merge carries your credit. Please say whether that suits you, and stop
  pushing to `astra/iis-support` meanwhile. `a1a9f11` gets an archive tag before the first reshaping push,
  so removed code stays reachable there and in the Deferred notes.
- **Ruled, and already on main.** Link the rulings commit. Q1 to Q6 and N0 to N35 are ruled, and no question
  is open. `d8bb08f` already brought the single MOSEK optimization and the time limit.
- **Defects found, bugs surfaced.** The port fixes `failed{true}` taken as feasible, `infeasible_or_unbounded`
  discarded, the limit attribution, the sleeping deadline test and the vacuous integer check. It drops the
  SoPlex bindings and the Clp ray accessor, and records the gate rule for `has_dual_ray`. The main bugs your
  work surfaced are fixed with credit to you.
- **Offers that fit your available time,** all off the critical path: the MOSEK fixes and MOSEK row bounds if
  you have MOSEK, the Deferred notes, the loader fix as ruled, Gurobi and CPLEX native if still licensed, the
  SoPlex Farkas proposal upstream, and the benchmark harness later.
- **How to work.** Fresh branches from main, `git show a1a9f11:<path>` rather than cherry-picks, "Ported from
  a1a9f11 (PR #3)" in the body. Point your agent at iis.md, with its Rulings section, and the rewritten
  iis_todo.md: no generic public name, no vendor call in tests, no separate test binary, no branch on a
  backend type in the free function.
- **Deferred, not rejected.** Ordering, ray seeds, native elastic relaxation, cancelling native calls (N31 b),
  integrality as a candidate (N25), a duration for native calls if users ask (N29 b, not ruled) and the
  harness each get a return path in iis.md. The rejected ideas carry their reasons, and three of them follow
  from the rulings. The possible-member tag: partial answers keep their side under `not_proven_minimal`
  (N1 a). Optimizer counting: no test calls a vendor API (N5 b). License help text: vendor text does not
  belong in a generic header (N18 a).
- **Questions.** Which licenses and versions you have, which offers you take, and which email addresses GitHub
  should credit, as your commits use two.
