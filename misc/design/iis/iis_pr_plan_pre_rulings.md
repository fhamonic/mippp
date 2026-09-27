> Superseded by [iis_pr_plan.md](iis_pr_plan.md). This is the plan as the maintainer ruled on it on
> 2026-09-27, kept so that the options behind each ruling letter stay readable.

# IIS pull request #3: adaptation plan

Plan for turning pull request #3 (`mheyman:astra/iis-support`, HEAD `a1a9f11`) into the feature designed in
[iis.md](iis.md), in the order of [iis_todo.md](iis_todo.md). It rests on an inventory of the pull request
dated 2026-09-27, whose verdicts it follows unless a reason is given. The pull request adds 7,433 lines over
48 files in 21 commits, two of them merges of main. It still has the shape iis.md lists as "Replaced": an
index-based utility, no `has_iis`, a separate test binary. Kept: the deletion engine and its limit semantics,
the exact oracle with the published HiGHS vectors, the MOSEK guard and slot selection, the SoPlex move fix,
the case list, and the ideas that also work in place. Line numbers in **Port** and **Drop** bullets are at
`a1a9f11`. Elsewhere they are on main at `f5e833f` unless marked "at `a1a9f11`".

## Decisions to take first

None of these is ruled. Each item gives the options, a recommendation, the evidence and what it blocks. WP1
records each ruling in iis.md with its date, or defers it to a named package. Q1 to Q6 are iis.md's open
questions, N0 to N18 are new.

- **N0. Handling of pull request #3.** (a) The maintainer reshapes it in place, then squash-merges it. (b) The
  author reshapes it. (c) It is closed, and the reusable parts re-land in fresh pull requests. Recommended:
  (a). Maintainer edits are allowed, the author can work on it only on Friday nights, and the 2026-09-23 comment
  states the intent to merge it in v1.1.0. About 7,000 of the author's lines would leave their branch, so the
  reply asks first.
  Blocks the reply, WP4 and WP5.
- **Q1. Where the fallback lives.** (a) A protected helper per backend and a standalone engine, as iis.md
  recommends. (b) A public free function, which needs a public entity enumeration first. (c) (a), with the
  engine in `detail/deletion_filter.hpp` and the guard, applier, classifier and snapshot builder as a `detail`
  function template over the model reference. Recommended: (c). Each backend declares one
  `compute_iis(this auto & self, const iis_limits & = {})`, so stubs test the template and a derived probe's
  `solve()` runs in trials. A deducing-this hook in `remapping_model_base` fails to compile from `highs_lp`
  (checked 2026-09-27), so the three remapping bases need `friend remapping_model_base<...>`. Evidence: the
  engine is standard-library only (`deletion_filter.hpp:64-70` at `a1a9f11`). Blocks WP7 onward.
- **Q2. One call or two.** (a) `compute_iis()` returns the snapshot by value like `get_basis()`, or (b) it
  stores it for a `get_iis()`. Recommended: (a), with the names `iis_status`, `iis_outcome` (`irreducible`,
  `not_proven_minimal`, `feasible`, `undetermined`), `iis_reason` (`solve_limit`, `time_limit`, `cancelled`),
  `iis_limits`, `lp_iis<I, T>`, `model_iis_t<T>`, `has_iis<T>`. Evidence: the pull request returns by value
  (`linear_iis.hpp:391-409` at `a1a9f11`), under generic names in an opened namespace. Blocks WP5.
- **Q3. Relaxing a row side.** (a) Row bounds through `has_modifiable_constraint_bounds<T, M = T>`, or (b)
  modifiable sense and rhs on five more backends, which cannot relax one side of a ranged row. Recommended:
  (a). Evidence: the pull request's candidates are sides (`prepared_linear_system.hpp:100-101` at `a1a9f11`),
  and five fallback backends already bind side setters. Blocks WP6 onward.
- **Q4. The status after a fallback run.** (a) `status::unknown` after any run that called `solve()`,
  exceptions included, native routines leaving it untouched. (b) The pre-call status. (c) The last trial's.
  (d) `unknown` after every call. Recommended: (a), resetting MOSEK's cached slot too. SCIP needs nothing,
  since the restore's mutators run `_free_transform()`, and `_solved` is never cleared by hand
  (`scip_milp.hpp:163-171, 517`). The suite asserts "unknown or the pre-call status" and the re-solve result.
  Evidence: a complete pass ends on a feasible trial whenever the last candidate is necessary
  (`deletion_filter.hpp:206-235` at `a1a9f11`). Blocks WP7, WP8, WP12 and WP17.
- **Q5. Parameters.** (a) None. (b) A solve budget and a stop token, as iis.md recommends. (c) (b) plus one
  duration. (d) The pull request's options (`algorithm/iis_limits.hpp:23-34` at `a1a9f11`), adding a deadline
  and a batch size. Recommended: (c), a defaulted `iis_limits {max_solves, time_limit, stop_token}`. The
  duration becomes one deadline, NaN and negative values are rejected, limits act between trials, and a trial
  gets min(remaining, saved caller limit) where `has_time_limit` holds. Evidence: iis.md:126-127 and 166-168
  forward a budget its Q5 lacks. Sub-decisions: N4, N7, N9. Blocks WP4, WP5 and WP7.
- **Q6. Ranged rows.** (a) Supported, each finite side a candidate, or (b) refused at runtime. Recommended:
  (a). Evidence: `add_ranged_constraint` exists only on `clp_lp` and `cbc_milp`, the first fallback backends,
  and the pull request tests ranged rows (`test/iis.cpp:1687-1698` at `a1a9f11`). Blocks WP6, WP8 and WP9.
- **N1. A possible-member tag.** (a) None: retained, untested and native "possible" members get `member_*`
  under `not_proven_minimal`. (b) `iis_status::possible_member`, outside `member`, where reported.
  Recommended: (a). The engine keeps a state per candidate, so (b) stays possible, but one tag would lose the
  side. A new alternative breaks exhaustive visitors, so rule before the first release with `has_iis`.
  Evidence: the engine only counts unresolved members (`deletion_filter.hpp:223-234` at `a1a9f11`). Blocks WP5
  softly, and WP7 (snapshot builder), WP8 (first model with `has_iis`), WP15 and WP16.
- **N2. A reason for an inconclusive trial.** (a) None, a missing reason meaning "a solve could not decide".
  (b) `iis_reason::inconclusive_trial`, where the pull request sets `termination::indeterminate`. (c) (b) with
  the trial's status. Recommended: (b), which extends the ruled reasons. After any inconclusive trial that
  ran, initial or singleton, the reason is `inconclusive_trial`, or `time_limit` past the deadline.
  `solve_limit` and `cancelled` apply only when the engine stops before a trial. Evidence:
  `deletion_filter.hpp:119, 241` misattribute (at `a1a9f11`). `test/iis.cpp:1009-1022` pins the completed-pass
  case, and with `max_solves = 1` an inconclusive initial trial reports `solve_limit`. Blocks WP4 and WP5.
- **N3. Batching.** (a) Single pass, re-porting `deletion_filter.hpp:136-170` (at `a1a9f11`) later behind an
  internal threshold. (b) Ported dormant with its tests. (c) A public knob. Recommended: (a), with (b)
  acceptable, since iis.md:161-165 rules one pass and a dormant path has no caller. A loop simulation, not a
  solver run, gives 278 trials at batch 64 against 10,001 for 10,000 candidates with a scattered 10-member
  conflict (732 with 50 members). It doubles the trials on an all-essential chain. Blocks the scope of WP4.
- **N4. "No time limit".** (A) `get_time_limit()` is never negative, MOSEK maps its -1 s
  (`mosek_base.hpp:471-485` at `a1a9f11`), and every backend gets `TimeLimitTest` cases. (B) MOSEK stays
  transparent, and forwarding treats a negative saved limit as no cap. Recommended: (B), two lines in the
  guard. Evidence: main's MOSEK getter returns -1 s on a fresh task (`mosek_base.hpp:428-435`), which
  min(remaining, current) would write. Blocks WP7 and WP12.
- **N5. A MOSEK test calling the vendor API.** (a) An exception to the 2026-09-23 conformance rule, confined
  to `test/solvers/mosek.cpp`. (b) Drop the optimizer-count and interior-point tests and strike step 1 item 4.
  (c) Count log headers, unprobed. Recommended: (b), since `solve()` is main's only `MSK_optimizetrm` call.
  Evidence: `test/iis.cpp:758-831, 940-956` at `a1a9f11`. Blocks the WP2 tests.
- **N6. Column-less models.** (a) Undetermined, per iis.md:229-231. (b) A zero-solve precheck: the first
  finite row side with lower > 0 or upper < 0 is the sole member, irreducible, and none means feasible. The
  pull request's dummy column (`cold_model_workspace.hpp:102-107` at `a1a9f11`) is excluded by the in-place
  ruling. Recommended: (b), shared by native wrappers, status untouched. Sub-question: compare with 0 exactly,
  which calls 0 >= 1e-9 irreducible although solvers accept it, or against `get_feasibility_tolerance()`,
  which only `clp_lp`, `cbc_milp`, `glpk_lp` and `scip_milp` have (checked 2026-09-27). Recommended: exact and
  documented. Evidence: Clp, Cbc, HiGHS and SoPlex return `unknown` without solving, and MOSEK returns
  `optimal`. Until ruled, the suite accepts both answers. Blocks WP7 and WP8.
- **N7. Prechecks and the budget.** (a) `max_solves` counts `solve()` calls, so prechecks are free. (b) Every
  oracle call counts, as `test/iis_diagnostics.cpp:275-282`, `docs/iis.md:259-261` and
  `algorithm/iis_limits.hpp:24` pin (at `a1a9f11`). Recommended: (a). For crossed sides, (a1) two singleton
  trials decide, or (a2) a crossed variable is irreducible at zero solves on a model type with no
  special-constraint or callback capability (iis.md:139-140). Recommended: (a2), since each bound alone is
  satisfiable, integrality included, except binary-typed SCIP columns until N14 is settled. Crossed rows keep
  their trials, since a term-less side can be infeasible alone. Only `cbc_milp` and the HiGHS classes read row
  terms, not `clp_lp`, `glpk_*`, `scip_milp`, `soplex_lp` or `mosek_*` (checked 2026-09-27). Blocks WP5, WP7
  and WP8.
- **N8. Snapshots and recycled ids.** (a) Document that the snapshot describes the model as it was, so a later
  handle may reuse a removed id. (b) Generation stamps. (c) Invalidation on change. Recommended: (a), with a
  case checking answers after a later `remove_variable`. Evidence: `clp_lp.hpp:160-178` and
  `remapping_model_base.hpp:55-70` recycle ids. Blocks WP5, WP8 and WP17.
- **N9. Native routines and `iis_limits`.** (a) The model's own time limit only, per iis.md. (b) Wrappers
  forward min(remaining, saved) and check the stop token first, `max_solves` being fallback-only. Recommended:
  (b), so callers cannot tell the paths apart (iis.md:17-21). `copt_lp` then needs a time limit or an
  "unbounded" note. The pull request has no native routine. Blocks WP15 and WP16.
- **N10. An archetype for `has_iis`.** (a) None, like every capability, with `static_assert`s in the fixture.
  (b) The first capability archetype. Recommended: (a), as archetypes cover only expressions, constraints and
  the `model_*_t` anchor. Blocks WP5.
- **N11. SoPlex.** (a) It joins last, binding eight symbols of 7.1.3 and 8.0.2 (bound and objective accessors,
  row side setters, `SoPlex_getRowBoundsReal`) after checking 7.1.1 and 7.1.2, or raising the floor. (b) It
  stays out. Recommended: (a). Blocks WP14.
- **N12. HiGHS ranged rows.** Once the new setters range a row, the sense and rhs getters (a) keep reporting
  `greater_equal` (`highs_base.hpp:553-559`), or (b) throw from the sense getter like Clp and Cbc, and from
  the rhs getter like Clp. Recommended: (b), as only the new setters can range a HiGHS row. Blocks WP10.
- **N13. Clp's secondary status.** Main maps status 0 to `optimal` whatever the secondary status
  (`clp_lp.hpp:535-544`). With secondary 2 or 4, unscaled primal infeasibilities remain, so a trial falsely
  proves feasibility. An infeasible model then ends `feasible`, or a removable candidate is kept under a false
  `irreducible` claim. (a) Bind `Clp_secondaryStatus`, checking the 1.17.4 floor, and map both to
  `optimal_infeasible_unscaled`, which widens a public variant and breaks exhaustive visitors. (b) Bind it,
  and let `clp_lp::compute_iis` classify both as inconclusive privately. (c) Document the gap. Recommended:
  (b) in WP8, (a) later if wanted. Secondary 1 needs nothing: with status 4 it is already `unknown`, and with
  status 1 it means a dual limit, which zero-objective primal trials do not hit. Blocks WP8.
- **N14. SCIP binary columns.** `add_binary_variable` creates `SCIP_VARTYPE_BINARY` columns
  (`scip_milp.hpp:261-270`). Whether SCIP rejects or clamps a relaxed bound on them is unprobed. (a) Create
  [0, 1] integer columns as `SCIP_VARTYPE_INTEGER`, or retype them inside the guard, so every bound stays a
  candidate. (b) Keep their bounds as candidates if the WP13 probe shows each single bound stays satisfiable.
  (c) Exclude them, which widens the scope ruling and needs its own ruling. Recommended: the probe, then (b),
  else (a) inside the guard. Blocks WP13, and N7 (a2) on `scip_milp`.
- **N15. Native side effects.** iis.md:172-174 says native routines leave the status untouched. Whether
  `Highs_getIis`, `GRBcomputeIIS`, `CPXrefineconflictext` and `XPRSiisfirst` keep the held solution is
  unprobed. The wrappers also write `IISOPS`, `IIS*Force` and, under N9 (b), a time limit. (a) Status
  untouched, once a probe per routine shows that `get_solution` and the status still agree. (b) The Q4 rule on
  native paths too. Recommended: (a) where the probe passes, (b) elsewhere. Either way the wrapper restores
  what it sets through the WP7 guard. Blocks WP15 and WP16.
- **N16. IIS and LP basis support.** The 2026-09-23 comment says IIS "will fit well alongside LP basis support
  in a future v1.1.0 release". The basis concepts exist (`model_concepts.hpp:576-634`), but no backend
  implements `get_basis`, so the IIS snapshot is the first handle-keyed one. (a) Design the per-entity storage
  jointly with `get_basis`. (b) IIS first, its storage a `detail` template that basis support reuses. (c)
  Independent designs. Recommended: (b), which keeps basis work off the critical path. Blocks WP5.
- **N17. Release gating.** v1.1.0 also carries the rest-or fixes (ruling of 2026-09-23). If PR #3 merges
  before that tag with public names, v1.1.0 freezes WP5's surface before WP7 and WP8 validate it. (a) No tag
  between the PR #3 merge and WP8. (b) WP5's types stay in `detail` until WP8 publishes them with `has_iis` on
  Clp. (c) Tag v1.1.0 first, and IIS ships in v1.2.0. Recommended: (b), which decouples the release from the
  critical path, or (a) if no tag is planned before WP8. Blocks the PR #3 merge.
- **N18. Loader and license diagnostics.** iis.md:309-311 leaves them "to be judged on its own". (a) A
  standalone loader fix, trimmed to the platform's search variable, one precedence sentence and the quoted
  `MIPPP_<KEY>_LIBRARY` value. License hint text is rejected, and MOSEK's license codes are fixed as a bug.
  (b) The pull request's diagnostics as they are. (c) Nothing. Recommended: (a), the inventory's majority.
  Evidence: main names `LD_LIBRARY_PATH` on every platform (`solver_library.hpp:300-301`), and MOSEK raises
  `license_error` for an expired license only. Blocks the loader fix and WP2's license commit.

## Corrections to iis.md and iis_todo.md

WP1 applies these.

- [ ] **MOSEK step 1** (iis.md:295-301). Items 1 and 3 are done on main: `d8bb08f` optimizes once
  (`mosek_lp.hpp:102-108`), and `TimeLimitTest` runs (`test/solvers/mosek.cpp:23, 39, 40`). Tick both.
- [ ] **Step 1 items 2 and 4.** The signature fix is cosmetic, `MSKrestrmcode` being an `int` enum
  (`mosek_api.hpp:27, 165, 183`). The MILP status already picks a slot. The gap is the LP status and every
  getter of both classes (`mosek_lp.hpp:41-60`). Item 4 breaks the 2026-09-23 rule that no test calls a vendor
  API, so reword it per N5.
- [ ] **Lazy objectives** (iis.md:121-124). Cbc (`cbc_milp.hpp:139-148`) and SCIP (`scip_milp.hpp:219-231`)
  are lazy too.
- [ ] **SoPlex** (iis.md:202-205, step 5). 6.0.4 has `SoPlex_changeVarBoundsReal` and sits below the 7.1.1
  floor. 7.1.3 declares every N11 symbol, so the wrapper keeps no state.
- [ ] **Row side symbols** (iis.md:198, step 5). SCIP also needs `SCIPgetLhsLinear` and `SCIPgetRhsLinear`,
  unbound like its setters. MOSEK's per-side setter is `MSK_chgconbound`, not `MSK_putconbound`.
- [ ] **Limits** (iis.md:126-127, 166-168, 286-288, step 4). The note forwards a budget Q5 does not define.
  `clp_lp`, `glpk_*` and `copt_lp` lack `has_time_limit`, so forwarding first runs on Cbc and HiGHS, and
  native COPT LP runs unbounded.
- [ ] **Column-less models** (iis.md:229-231, step 5). Clp, HiGHS and SoPlex also return `unknown`, and MOSEK
  returns `optimal` (`mosek_lp.hpp:107`). Rewrite per N6.
- [ ] **Archetype** (step 2). No capability has one. Reword per N10.
- [ ] **Snapshot translation** (iis.md:34-41). Handle results translate lazily today (`highs_lp.hpp:104-130`),
  so the snapshot is the first eager one. `clp_lp` is not on the remapping layer. The id bound is
  `_remap_ids ? _native_ids_map.size() : N`. Add N8.
- [ ] **Scope and cases** (iis.md:54-62, 227-229). No backend implements `add_sos1_constraint` or
  `add_sos2_constraint` (`has_sos1_constraints` holds nowhere). Indicators exist only on `gurobi_milp` and
  `cplex_milp`, and combined with removals they hit a main bug, fixed first.
- [ ] **Only proofs count** (iis.md:152-160). This needs sound mappings, which GLPK's `GLP_INFEAS`, HiGHS's
  uninitialized `psolstatus` and Clp's secondary status break.
- [ ] **Side effects** (iis.md:172-176). Whether native routines keep the held solution is unprobed, and their
  wrappers set parameters. Rewrite per N15.
- [ ] **HiGHS** (iis.md:98-99, 232-234, step 6). 1.11.0 was not checked. CI already spans the switch, with
  Windows on 1.13.0 and macOS on brew. The row arrays take the row counts, though the 1.12.0 header documents
  column counts (`highs_c_api.h:2428-2442`).
- [ ] **Certificate plumbing** (iis.md:242-243, 301-302). The SoPlex part needs an unsubmitted 8.1.0 patch and
  waits for upstream. The Clp ray must gate on the cached `_status`.
- [ ] **Outside IIS** (iis.md:309-311). Record the N18 ruling.
- [ ] **Feature tables** (step 7). `highs_qp` rows never appear, as the script sorts by `_lp_` or `_milp_`
  (`tested_features_table.py:31-34`). WP17 fixes the script.

## Handling of pull request #3

- **State on 2026-09-27.** Open, not a draft, mergeable, maintainer edits allowed. On 2026-09-23 the
  maintainer wrote that IIS "will fit well alongside LP basis support in a future v1.1.0 release", and that
  the design notes will serve "to merge your IIS PR in v1.1.0". The author answered that they can work on it
  only on Friday nights. On 2026-09-26 they wrote that Astra "implemented the items in the TODO", which the
  branch does not bear out (see the reply below).
- **Recommended handling, pending N0.** Reshape it in place, then squash-merge. This follows the stated intent
  and keeps the author's credit. Re-landing through author PRs would tie the critical path to Friday nights.
- **Mechanics.** Tag `a1a9f11` on origin, for example `archive/pr3-a1a9f11`, before the first reshaping push,
  since `refs/pull/3/head` follows the branch. Convert the PR to draft with the GraphQL call of
  `misc/contributor_pr_cheatsheet.md` section 1, as gh 2.4.0 has no `--undo`. The reply asks the author to
  stop pushing. Then follow the cheatsheet loop: `git pull --rebase` before each push, never `--force`.
- **Commits.** (1) `refactor(iis)`: remove the replaced surface per the file map, restoring main's versions of
  the other files. (2) `feat(iis)`: the engine (WP4). (3) `feat(model)`: the IIS types (WP5). (4) `test(iis)`:
  oracle and vectors (WP5). On green CI and under the N17 ruling, `gh pr ready 3` and a squash-merge whose
  body lists the ported parts.
- **Credit.** Check on GitHub who a squash commit is attributed to, and word the trailer and the reply to
  match. Their commits use two email addresses, so the reply asks which one to credit. Every
  later PR porting their code carries the same `Co-authored-by` trailer.
- **Later PRs.** Fresh branches from main, one topic each, porting with `git show a1a9f11:<path>`, never
  cherry-picks. The body ends with "Ported from a1a9f11 (PR #3)". MOSEK, SCIP, SoPlex and commercial PRs
  attach a local ctest log, built one at a time with `-c tools.build:jobs=4`.
- **Who does what.** The maintainer owns the critical path. The author is offered leaf work: WP2 and WP12 if
  they have MOSEK, WP3, the loader fix once N18 is ruled, Gurobi and CPLEX in WP16 if they still have Gurobi 13 and
  CPLEX 22.2 (commits `637d166`, `9b36c46`), the SoPlex proposal upstream, the harness later. An offer
  unclaimed after two Fridays goes to the maintainer.
- **Order.** The move, GLPK-status and native-id fixes open now. The loader fix waits for N18, and the
  destructor PR follows WP2. After WP1, WP2, WP3, WP6 and PR #3 run in parallel. The critical path is WP1,
  PR #3, WP7, WP8, then WP9-WP11. Native packages wait for WP8, so `IisTest` runs in CI before any local-only
  backend.

| File at `a1a9f11` | Destination |
| --- | --- |
| `algorithm/deletion_filter.hpp`, `algorithm/iis_limits.hpp` | WP4 engine and WP5 types, ordering, batching and `phase_budget` to WP3 |
| `algorithm/{elasticity_filter,ray_support}.hpp`, `utility/{native_seed,elastic_lp}.hpp`, `solvers/clp/*` | WP3 notes |
| `utility/{linear_iis_model,deletion_workspace,cold_model_workspace,prepared_linear_system,iis_limits}.hpp` | WP7 hunks and ideas, files dropped |
| `utility/{linear_iis,linear_iis_types,iis_report,iis_statistics}.hpp`, `algorithm/iis_messages.hpp`, `infeasibility_certificate.hpp` | dropped, checklists to WP3, warnings to WP17 |
| `solvers/mosek/*` | WP2, without the certificate and license hint hunks |
| `solvers/soplex/*`, `patches/*`, `solvers/{cplex,gurobi}/*_api.hpp`, `model_concepts.hpp` | move fix to the standalone fixes, the rest dropped, Farkas API proposed upstream |
| `detail/{solver_library,diagnostic_text}.hpp`, `test/dynamic_library.cpp` | loader fix after N18, trimmed |
| `test/iis.cpp`, `test/iis_diagnostics.cpp` | WP2, WP4, WP5, WP7 and WP8 tests, classification inverted, the rest dropped |
| `test/iis_vectors.cpp`, `test/data/iis/*`, `test/CMakeLists.txt` | WP5 oracle and data, README replaced by a provenance comment, WP8 cases, no separate binary |
| `docs/iis.md`, `docs/iis-implementation-status.md` | WP17 prose and WP3 notes, audit quoted in the thread |
| `benchmarks/iis.cpp`, `CMakeLists.txt`, `.gitignore`, `README.md`, `coming-from.md`, `zensical.toml` | reverted, harness out of the repository, docs redone in WP17 |

## Work packages

The Step column maps each package to its iis_todo.md step, with item numbers.

| WP | Content | Step | Owner | Lands in | Needs |
| --- | --- | --- | --- | --- | --- |
| WP1 | Rulings, corrections, reply | 0 | maintainer | commit on main | none |
| WP2 | MOSEK fixes | 1.2, 1.4 | author if licensed | own PR | WP1, N5, N18 |
| WP3 | Deferred notes | Later | author | docs PR | WP1 |
| WP4 | Deletion engine | 4.1 | maintainer | PR #3 | N0, Q5, N2, N3 |
| WP5 | IIS types, snapshot, oracle | 2.1-2.3 | maintainer | PR #3 | N0, Q2, Q5, N1, N2, N7, N8, N10, N16, N17 |
| WP6 | Row-bound concept on Clp | 3 | maintainer | own PR | Q3, Q6 |
| WP7 | In-place fallback machinery | 4.2-4.5 | maintainer | own PR | PR #3, WP6, Q1, Q4, N1, N4, N6, N7 |
| WP8 | Clp slice and shared suite | 2.4, 4.6 | maintainer | own PR | WP7, N1, N13 |
| WP9 | Cbc | 5.1 | maintainer | own PR | WP8 |
| WP10 | HiGHS fallback | 5.3 | maintainer | own PR | WP8, N12 |
| WP11 | GLPK | 5.2 | maintainer | own PR | WP8, GLPK status fix |
| WP12 | MOSEK fallback | 5.4 | author if licensed | own PR | WP2, WP8 |
| WP13 | SCIP | 5.5 | maintainer | own PR | WP8, N14 |
| WP14 | SoPlex | 5.6 | maintainer | own PR | WP13, N11 |
| WP15 | HiGHS native | 6.1, 6.6 | maintainer | own PR | WP10, N1, N9, N15 |
| WP16 | Gurobi, CPLEX, Xpress, COPT native | 6.1-6.5 | by license | a PR per solver | WP8, native-id fix, N1, N9, N15 |
| WP17 | Documentation | 7 | maintainer | own PR | WP8-WP11 |

### WP1. Rulings, corrections, reply

- [ ] Rule Q1-Q6 and N0-N18 in a dated "Rulings" section of iis.md, or defer each to a named package.
- [ ] Apply the corrections, rewrite iis_todo.md in the order of the table above, post the reply, and act on
  N0.
- **Done when** every question has a dated ruling or a named deferral, and the reply is posted.

### WP2. MOSEK fixes

- **Port.** `mosek_handle_guard.hpp:1-31`, namespace renamed `detail`. `mosek_base.hpp:35, 43-80, 104-131`.
  `mosek_api.hpp:189`. `mosek_lp.hpp:38-100, 107-143`. `mosek_milp.hpp:86-173, 182-194`, with 174-180 replaced
  by `_status = _get_status(trm);` unless the probe below says otherwise. Tests from
  `test/iis.cpp:251-263, 299-323` and `test/iis_diagnostics.cpp:163-176`.
- **Drop.** `mosek_lp.hpp:7, 145-175`, `infeasibility_certificate.hpp`, `mosek_api.hpp:477-479`, the
  `static_assert` at `test/iis.cpp:324`. Also `mosek_base.hpp:471-485` unless N4 is (A), and
  `mosek_api.hpp:139-145, 443` with `test/iis.cpp:758-831, 940-956` unless N5 is (a).
- [ ] The guard, a destructor that cannot throw, and slot selection on the LP status and every getter. A
  primal-feasible slot ranks above a dual-only one, which the pull request ties. The pick is cached and reset
  with `_status`.
- [ ] Status reset before optimizing, `trm` initialized, signature. A column-less task maps MOSEK's own
  status. Probe MOSEK 11 on empty LP and MIP tasks, which `LpModelTest.solve_empty_*` solves. If one gets no
  slot, keep an empty-task path, as `_get_status` then gives `unknown` where main reports `optimal`.
- [ ] Own commit, per the N18 ruling: `_check` maps every license code (1000-1028 in MOSEK 11) to
  `license_error`, without hint text.
- **Tests.** Plain `TEST`s in `test/solvers/mosek.cpp` for the guard (fake api), the slot helper (derived
  probe), and a term-less row 0 >= 1 not being `optimal` until the shared case lands. **Done when** the MOSEK
  suites pass locally.

### WP3. Deferred notes

- **Port** as prose with `git show a1a9f11:<path>` pointers under iis.md's Deferred bullets: batching and
  ordering (`deletion_filter.hpp:124-205`, `test/iis.cpp:1264-1409`), seed outcomes
  (`iis_statistics.hpp:10-27`), verified seeds (`linear_iis.hpp:221-263`), `phase_budget`
  (`iis_limits.hpp:69-98`), MOSEK certificates (`mosek_lp.hpp:145-175`), the Clp ray (`clp_lp.hpp:596-606`),
  the elasticity loop (`elasticity_filter.hpp:43-88`), the benchmark (`benchmarks/iis.cpp:30-116, 148-253`).
- [ ] One note of 2-4 lines per item, with its return condition, plus two rules: stop reasons come only from
  the global budget, and `has_dual_ray` gates on the cached status.
- [ ] Point at the archive tag of `a1a9f11`, and link the SoPlex proposal once opened.
- **Done when** every archived row of the file map has a note.

### WP4. Deletion engine

- **Port** into `include/mippp/detail/deletion_filter.hpp`:
  `deletion_filter.hpp:64-70, 87-123, 173, 206-242, 247-257` (oracle concept, initial proof and continuation,
  swap/pop pass, entry), with `:45-62` as the detail outcome, and `algorithm/iis_limits.hpp:60-67` (stop
  precedence). `:136-170` only under N3 (b). Tests into `test/deletion_filter.cpp`:
  `test/iis.cpp:352-391, 989-1007, 1213-1262, 1411-1480`, `:1009-1022` inverted, `:1024-1049` on a fake clock.
- **Drop.** `deletion_filter.hpp:26-44, 55, 72-78, 124-135, 171-172, 174-205` (statistics, ordering),
  `iis_limits.hpp:39-40, 55-58, 69-98`, `iis_messages.hpp`, `test/iis.cpp:211-228, 1264-1336`, the
  absolute-deadline parts of `:958-987`, and the names `mippp::iis`, `result`, `options`, `termination`,
  `feasibility`. WP5 takes `iis_limits.hpp:41-54`.
- [ ] Detail types with specific names: a trial verdict, `deletion_limits {max_solves, deadline, stop_token}`,
  and an outcome with one state per candidate (dropped, necessary, unresolved, untested). No member is
  reported before the initial trial proves infeasibility.
- [ ] A clock template parameter, `steady_clock` by default. Reasons per the N2 ruling, with a test that
  `max_solves = 1` and an inconclusive initial trial never end with `solve_limit`. The continuation entry
  stays in `detail`.
- **Tests.** `mippp_test` in every CI job, ASan/UBSan included, with the exhaustive test's trace built only on
  failure. **Done when** CI is green, the header is self-contained, and no generic public name remains.

### WP5. IIS types, snapshot and oracle

- **Port.** `algorithm/iis_limits.hpp:15-34`, split into outcome, reason and `iis_limits`. If the Q5 ruling
  keeps a duration, `:41-54` (validation, saturating conversion to a deadline) without `iis_messages.hpp`.
  `test/iis_vectors.cpp:53-113` into `test/test_suites/iis_oracle.hpp`, keyed by case-local sides. `:27-51`
  (vectors, with attribution) into `test/test_suites/iis_vectors.hpp`, included by `test/iis_oracle.cpp` and
  `test/test_suites/iis.hpp`. `:134-165` (self-test) into `test/iis_oracle.cpp`.
  `test/data/iis/HIGHS-LICENSE.txt`. Under the same condition, the validation assertions of
  `test/iis.cpp:958-987`, retargeted at `iis_limits::time_limit`.
- **Drop.** `linear_iis_types.hpp`, `iis_statistics.hpp`, `iis_limits.hpp:26-27, 29-30` (absolute deadline,
  batch size), and `test/data/iis/README.md`, whose lines 18-21 claim a proof the witnesses do not give. A
  provenance comment in the vectors header replaces it.
- [ ] `iis_status` tags next to `basis_status`, refinements deriving from `member`, a status concept,
  `lp_iis<I, T>`, `model_iis_t<T>`, `has_iis<T>`, in the namespace the N17 ruling picks.
- [ ] Outcome and reason tags documenting the contract: the budget counts what the N7 ruling says, a proof on
  the last permitted solve is complete, cancellation beats time which beats solves, a stop keeps the last
  proven subset, and zero members with `irreducible` means the background alone is infeasible.
- [ ] The snapshot template, shaped per the N16 ruling: one code per entity in vectors sized by the handle-id
  bounds, `absent` beyond them, counts built once, the N8 comment.
- [ ] Oracle rules: `member_lower` and `member_upper` select one side, `member_both` both, each dropped in
  turn, a `member` row is kept or dropped whole. At most 6 variables per case.
- **Tests.** `test/iis_snapshot.cpp` and `test/iis_oracle.cpp` in `mippp_test`, and a `static_assert` that the
  snapshot satisfies `lp_iis`. **Done when** PR #3 is green and squash-merged under the N17 ruling. No model
  satisfies `has_iis` yet.

### WP6. Row-bound concept on Clp

Nothing is ported, since the pull request never relaxed a row in place.

- [ ] `has_modifiable_constraint_bounds<T, M = T>` next to its readable twin (`model_concepts.hpp:476-483`).
- [ ] `clp_lp` setters writing `rowLower` and `rowUpper` as `set_constraint_rhs` does (`clp_lp.hpp:413-445`),
  fetching the arrays on each call.
- [ ] `ModifiableConstraintBoundsTest`: each side on `<=`, `>=`, `==` and ranged rows,
  `infinity_frees_a_row_side`, warm re-solves, a sense read after re-tightening a one-sided row, none on a
  ranged row.
- [ ] The concept in `docs/reference/concepts.md`, the setters in `docs/solving/updates.md`, the
  features-table label.
- **Done when** the suite passes on Clp in CI.

### WP7. In-place fallback machinery

- **Port** as rewrites over live handles: `deletion_workspace.hpp:99-111` (bitset diff),
  `linear_iis_model.hpp:73-92` (visit shape), `utility/iis_limits.hpp:34-59` (never loosen),
  `cold_model_workspace.hpp:64-72, 80-88` (zero objective, crossing scan), `prepared_linear_system.hpp:24-103`
  (a candidate per finite side). Stubs after `test/iis.cpp:1076-1089, 1119-1162, 1954-1981` and
  `test/iis_diagnostics.cpp:151-161, 178-197`, inverted.
- **Drop.** The rest of those files, with the factory, `set_minimization`, dummy and slack columns, and
  `elastic_lp.hpp`, `native_seed.hpp`, `linear_iis.hpp`.
- [ ] Classifier under the zero objective: `derived_from<infeasible>` or exactly `infeasible_or_unbounded`
  proves infeasibility. The `failed` branch and `optimal_infeasible_unscaled` are inconclusive. Otherwise
  `solution_available` proves feasibility, and the rest is inconclusive.
- [ ] Guard saving by value the finite sides of live candidates, the objective through `materialize()` with
  its offset, and a forwarded time limit. `materialize()` is linear-only and no owning quadratic expression
  exists. On a `qp_model` the guard keeps the `get_quadratic_objective()` view, which owns its Hessian arrays
  on `highs_qp` (`highs_qp.hpp:116-170`), or copies triples into a small `detail` container. It zeroes the
  objective and leaves sense and verbosity alone. `restore()` runs on every normal exit and rethrows the first
  error. The `noexcept` destructor restores only if `restore()` did not run.
- [ ] Applier over one bitset, restore being apply(full set). Prechecks per the N6 and N7 rulings. An oracle
  returning inconclusive past the deadline, forwarding the limit per the N4 ruling. A builder folding sides
  into `member_lower`, `member_upper` or `member_both`, per the N1 ruling.
- [ ] Protected live-handle hooks: `model_base` over [0, `num_variables()`), `remapping_model_base` over
  native i through `_var_handle(i)` with the Q1 friend, never through `_native_id`, and `clp_lp` over [0,
  `getNumCols`) minus `_free_variable_ids`. A protected `_compute_iis_by_deletion` lets probes reach the
  fallback on native backends.
- **Tests.** `test/iis_fallback.cpp` in `mippp_test`, on stubs: every status tag (`failed{true}` inconclusive,
  `time_limit{true}` feasible), a throw at trial k and during restore, lazy and QP objectives restored,
  forwarding, prechecks at zero budget. Hook probes as plain `TEST`s in the Clp and HiGHS solver files:
  `clp_lp` after removal and recycling, `highs_lp` after a non-tail removal. **Done when** CI is green and
  review finds no copy, factory, row removal, slack column, sense or verbosity change.

### WP8. Clp slice and shared suite

- **Port** as cases of `IisTest<T>` in `test/test_suites/iis.hpp`, from `test/iis.cpp:1493-1527` (verbosity
  probe), `:1560-1575` (redundant), `:1677-1720` (crossing, ranged, `x = 0.5`), `:1737-1746` (column-less),
  `:1766-1803` (modified after an IIS), `:1590-1616, 1641-1674, 1854-1895` (budget sweep, stop requested
  beforehand), `:1983-2002` (32 redundant rows). From `test/iis_vectors.cpp:173-195, 214-253` (transforms,
  integer case). The families of `benchmarks/iis.cpp:30-116` at 4-8 rows.
- **Drop.** The backend list (`test/iis.cpp:1483-1491`), the policy, factory, elastic and seed cases, and the
  `PublishedIis` harness (`test/iis_vectors.cpp:167-172, 196-212`).
- [ ] `clp_lp::compute_iis` as one overload, setting the status per the Q4 ruling and reading secondary
  statuses per the N13 ruling. `IisTest` registered and instantiated for Clp in this PR, since a typed test
  body is only checked when instantiated.
- [ ] New cases: one side of an equality row, `y = 0` with `x + y = 1.5`, modified after an infeasible solve,
  answers unchanged after a later removal, re-solve to the previous result and getters, a sign flip of
  one-sided rows. The integer-only case evaluates its witnesses and accepts the precheck or the trial path,
  since SCIP may round fractional bounds of integer columns into a crossing.
- [ ] A derived probe asserting per-trial invariants (zero objective, sense and verbosity kept, time limit
  never above the caller's, solve count) and restore after an injected throw. No `TYPED_TEST_P` is named after
  an IIS tag, and oracle and probe helpers are fixture member functions, not generic lambdas.
- [ ] The `IisTest` label, and the `has_iis` row in `concepts.md`, saying that zero members with `irreducible`
  means the background alone is infeasible.
- **Tests.** The Clp, sanitizer and macOS jobs. A backend rejecting crossed bounds or term-less rows at build
  time gets a `GTEST_SKIP` keyed on the error. **Done when** `IisTest` passes on Clp in CI, including the
  previous-result case.

### WP9 to WP14. Widening the fallback

Each package adds the row-bound capability, instantiates every capability suite newly satisfied, then
`IisTest`. Nothing is ported.

- [ ] **WP9, Cbc.** Setters through `Cbc_setRowLower` and `Cbc_setRowUpper`, both bound
  (`cbc_api.hpp:51-52, 139-140`). `setRowLower` is already used at `cbc_milp.hpp:340`. It runs the first
  time-limit forwarding and integer-only cases. Done when green in CI.
- [ ] **WP10, HiGHS.** Public row bounds in `highs_base` through `_row_bounds` and `Highs_changeRowBounds`
  (`highs_base.hpp:519-526, 562-596`), N12 in its own commit, `compute_iis` on the three classes. The
  `highs_qp` guard saves the Hessian, which `set_objective` clears (`highs_qp.hpp:47-51`). Its view translates
  through `_var_handle` lazily, which is safe as trials remove nothing. Done when green on Linux, macOS and
  Windows.
- [ ] **WP11, GLPK.** Row bounds through `glp_get_row_type`, `lb`, `ub` and `glp_set_row_bnds`, switching the
  type like `_set_col_bnds` (`glpk_base.hpp:169-181`). Limits act between solves only, and `glpk_milp` trials
  are cold. Done when green in the GLPK job.
- [ ] **WP12, MOSEK.** Row bounds through `MSK_getconbound` and `MSK_chgconbound` with finite = 0 on an
  infinite value. `compute_iis` applies the Q4 status rule, resets the slot cache and forwards per the N4
  ruling. Done when the suites pass locally.
- [ ] **WP13, SCIP.** Bind the four linear-constraint side functions, checked across the validated range
  (8.0.4 up to 10.0.4), then row bounds and `compute_iis`. Probe relaxing a bound of a column typed `BINARY`
  (`scip_milp.hpp:270`) and apply the N14 ruling. The restore needs no `_solved` handling. Done when small
  cases pass locally on SCIP 8.0.4, 9 and 10, or the compatibility matrix covers 8.0.4.
- [ ] **WP14, SoPlex.** Check the N11 symbols in 7.1.1 and 7.1.2, bind them, add variable bounds, a readable
  objective and row bounds, then `compute_iis`. Done when the suites pass locally on 7.1 and 8.0 with
  `MIPPP_SOPLEX_LIBRARY` set.

### WP15. HiGHS native

- [ ] `Highs_getIis` in `HIGHS_OPTIONAL_FUNCTIONS` (`highs_api.hpp:325-334`), after confirming it is absent
  from 1.11.0, which is not yet checked.
- [ ] Used on `highs_lp` and `highs_qp` when it resolves. Row arrays take the row counts despite the header
  (`highs_c_api.h:2428-2442`). Columns translate through `_var_handle`. "Maybe in conflict" maps per the N1
  ruling and forbids an irreducible claim. A missing symbol, or an empty IIS for an infeasible model, falls
  back. Status and parameters per the N15 ruling.
- [ ] A derived probe runs both paths, and the oracle validates both answers.
- **Done when** CI is green and the compatibility matrix shows HiGHS before and after 1.12.

### WP16. Gurobi, CPLEX, Xpress and COPT native

Each solver confirms its entry point at the range floor, translates through `_var_handle`, forwards per the N9
ruling, and handles status and parameters per the N15 ruling. These backends lack row bounds, so the
both-paths cross-check runs on HiGHS only (WP15). Nothing is ported.

- [ ] **Gurobi.** `GRBcomputeIIS` with the three `IIS*Force` attributes at 1, whose effect is unprobed.
  `IISConstr`, `IISLB`, `IISUB`, rows sided by sense, equality rows `member`, `IISMinimal`. A forced-only
  answer is irreducible with zero members.
- [ ] **CPLEX.** `CPXrefineconflictext` and `CPXgetconflictext`, a group per row and bound side, indicators
  outside. Status 31 is irreducible, 30 feasible, aborts and "possible" flags not minimal.
- [ ] **Xpress.** `IISOPS` integrality and special bits, `XPRSiisfirst`, `XPRSgetiisdata`, `I` entries
  dropped. Row `L` is a `<=` side (`member_upper`), `G` is `member_lower`, `E` is `member_both`, to confirm in
  45.1. `IISSOLSTATUS` gives the completion status.
- [ ] **COPT.** `COPT_ComputeIIS` and the four `Get*IIS` calls, per side on `copt_lp`, equality rows `member`
  on `copt_milp`, `IsMinIIS`.
- **Done when** `IisTest` passes locally at the floor and the latest release, with indicators and a removed
  variable on Gurobi and CPLEX.

### WP17. Documentation

- **Port** prose from `docs/iis.md:3-7, 21-22, 52-55, 59-76, 248-287, 900-903` (definition, the `x == 0.5`
  example on a `*_milp` model, reading table, limits, reference) and the warnings of
  `iis_report.hpp:141-161, 267-269`.
- [ ] `docs/solving/infeasibility.md`, about 120 lines: the `has_iis` gate, the consumer loop, tags, `*_lp`
  against `*_milp`, outcomes and limits, the state afterwards, snapshot validity, zero members, verbosity, and
  a per-model path table that native packages update. Links to repository files use absolute GitHub URLs.
- [ ] The limits text says that one trial can overrun the budget on `clp_lp` and `glpk_*`, which lack
  `has_time_limit`. The page and `concepts.md` say that a failed restore leaves the model data unspecified,
  with the status per the Q4 ruling.
- [ ] Fix `tested_features_table.py:31-34` so `highs_qp` rows appear, then regenerate the tables. Navigation
  under Solving, links from `status-and-limits.md` and `solutions.md:85`, and the README row at :187 and
  `coming-from.md:123` once `has_iis` ships on the CI backends.
- **Done when** `zensical build --clean` passes and every snippet compiles.

## Standalone fixes on main

- [ ] **Moves drop the status.** `_status(other._status)` in the move constructors of `clp_lp`, `cbc_milp`,
  `scip_milp` and `soplex_lp` (from `soplex_lp.hpp:66-67` at `a1a9f11`), plus
  `LpModelTest.move_preserves_status`. The same PR initializes HiGHS's `psolstatus` (`highs_lp.hpp:68`,
  `highs_milp.hpp:119`, `highs_qp.hpp:221`), which errors leave unwritten. It also adds
  `LpStatusTest.constant_row_without_variables_is_not_optimal` on a term-less row 0 >= 1, after probing each
  backend at hand, fixing or gating any failure with a comment. Maintainer, now.
- [ ] **Loader message.** `solver_library.hpp:300-301` names `LD_LIBRARY_PATH` everywhere. Per the N18 ruling,
  port the platform variable, precedence and quoting of `solver_library.hpp:129-170`,
  `diagnostic_text.hpp:15-44` and `test/dynamic_library.cpp:203-239, 243-245` at `a1a9f11`. Only the
  `MIPPP_<KEY>_LIBRARY` value is printed, never the search-path variable's, so the test drops that check. No
  `version_warning_help` or license text. Offered to the author.
- [ ] **GLPK status mapping.** `glpk_lp.hpp:79-103` maps `GLP_INFEAS` to `infeasible` and lets `glp_simplex`
  errors fall through. Own PR before WP11, noted in the changelog as a mapping change.
- [ ] **Native ids after removal.** `gurobi_milp.hpp:48-56, 69` and `cplex_milp.hpp:82` pass `v.id()`. Own PR
  with two `RemoveVariableTest` cases, before WP16.
- [ ] **Throwing destructors.** `copt_base.hpp:61-63`, `cplex_base.hpp:63-64`, `gurobi_base.hpp:79-80` and
  `xpress_base.hpp:69-71` call `check()`. Own PR after WP2, reusing its guard.
- [ ] **Elsewhere.** MOSEK's license codes and column-less status in WP2, Clp's secondary status per N13 in
  WP8, HiGHS ranged rows per N12 in WP10.

## The pull request's ideas

| Idea | Placement | Where and why |
| --- | --- | --- |
| `batched-deletion` | needs ruling | N3 recommends (a): later, behind an internal threshold set by the harness. Under (b), WP4 ports it dormant with its tests |
| `candidate-priority-order` | later | WP3 note, returns with preferences or weights as an internal comparator |
| `public-deletion-filter` | later | WP3 note, a thin wrapper once the WP4 signature survives WP9-WP14 |
| `possible-member-tag` | needs ruling | N1 recommends none in v1. Under (b), WP4's per-candidate state feeds a `possible_member` tag |
| `inconclusive-trial-reason` | needs ruling | N2 recommends (b), a new reason in WP4 and WP5. Under (a), a missing reason carries it |
| `continuation-from-known-proof` | now | WP4 entry, first used by the WP7 crossing precheck |
| `crossing-sides-shortcut` | now | WP7 precheck on variables and rows, accounting per the N7 ruling |
| `column-less-model-arithmetic` | needs ruling | N6 recommends (b), a WP7 precheck amending iis.md:229-231. Under (a), the outcome stays undetermined |
| `mosek-empty-model-status` | now | WP2, with the shared `LpStatusTest` case in the move-fix PR |
| `diff-based-trial-application` | now | WP7 applier |
| `restore-guard-exception-policy` | now | WP7 guard, a failed restore documented by WP17 and `concepts.md` |
| `overall-time-budget` | needs ruling | Q5 recommends (c), one duration turned into one deadline. WP17 documents overruns on Clp and GLPK |
| `limits-contract-for-completion-status` | now | WP5 contract, WP4 tests, WP8 cases, WP17 page |
| `soplex-time-limit-runtime-probe` | rejected | capabilities are compile-time, WP14 checks `setRealParam` privately |
| `verified-ray-seed` | later | after `has_dual_ray`, one verifying trial then the WP4 continuation |
| `weight-ordered-deletion` | later | after ordering and ray seeds, if the harness shows fewer solves |
| `bound-pruning-by-column-multipliers` | later | with `has_dual_ray`, bound multipliers as a variable mapping |
| `shared-phase-budget` | later | with the first seeding phase, reasons from the global budget only |
| `seed-outcome-checklist-for-dual-rays` | later | written by WP3, suite cases with seeding, never an enum |
| `dual-ray-accessors` | later | `has_dual_ray<T, M = T>` over handle mappings, gated on `_status`, on `*_lp` models only |
| `elastic-engine-over-native-feasrelax` | later | a `has_feasibility_relaxation` capability, after probing that each routine leaves the model unchanged. `GRBfeasrelax` modifies it, and a `GRBcopymodel` workaround is a copy needing its own ruling |
| `upstream-soplex-farkas` | later | proposed upstream by the author, bound once a release ships it |
| `optional-symbol-stock-test` | now | WP15: the suite across HiGHS releases, no extra variable |
| `lp-relaxation-domain` | rejected | `*_milp` explains the MIP, WP17 says to rebuild on `*_lp` |
| `time-limit-unlimited-contract` | needs ruling | N4 recommends (B), two lines in the WP7 guard. Under (A), WP12 changes MOSEK's getter |
| `nonthrowing-destructors-all-backends` | later | standalone PR after WP2 |
| `move-preserves-status` | now | standalone fix, four backends |
| `loader-help-current-values` | needs ruling | N18 recommends a trimmed standalone fix, printing only `MIPPP_<KEY>_LIBRARY` |
| `license-help-per-backend` | needs ruling | N18 recommends rejection: vendor text in a generic header. MOSEK's codes are fixed in WP2 |
| `iis-error-context` | rejected | exceptions propagate after the restore, trials never grow the model |
| `run-statistics` | rejected | member counts are the only summary, probes count solves |
| `empty-iis-background-infeasible` | now | WP5 contract, WP4 test, WP8 `concepts.md` row, WP16 mapping, WP17 page |
| `exhaustive-monotone-oracle-engine-test` | now | WP4, single pass |
| `fourier-motzkin-exact-oracle` | now | WP5 |
| `published-highs-vectors` | now | WP5 data, WP8 case, WP15 cross-check |
| `transform-invariance-cases` | now | WP8, vector case only |
| `integer-only-bound-conflict` | now | WP8 with evaluated witnesses, either path accepted, first run in WP9 |
| `budget-sweep-property` | now | WP8, accounting per the N7 ruling |
| `probe-model-tests` | now | WP7 and WP8 through the deducing-this member, helpers as fixture members |
| `derived-probe-for-protected-helpers` | now | WP2, WP7 hook probes, WP15 both-path check |
| `optimizer-run-counting` | needs ruling | N5 recommends (b), dropped. Under (a), a MOSEK-only test binds `MSK_putcallbackfunc` |
| `benchmark-families-as-suite-cases` | now | WP8 |
| `out-of-repo-strategy-benchmark` | later | separate repository like `mippp_nqueens`, when batching returns |
| `per-model-path-table-in-docs` | now | WP17, updated by WP15 and WP16 |
| `human-readable-report` | rejected | replaced by iis.md, warnings become prose in WP17 |

## Confirmed defects

The refuted report, on the CPLEX 1016 license help, needs nothing.

| Defect | Resolution |
| --- | --- |
| [engine-limits] A failed status with a solution flag counts as a feasibility proof | WP7 classifier, `failed` branch inconclusive |
| [engine-limits] A phase's local solve cap is reported as the user's solve limit | dropped with `phase_budget`, rule kept by WP3 |
| [engine-limits] An inconclusive trial in a completed pass is blamed on the solve budget or cancellation | WP4 attribution per the N2 ruling, `test/iis.cpp:1009-1022` inverted |
| [oracles-orchestration] Failed solves that carry a solution flag are taken as feasibility proofs | WP7 classifier |
| [oracles-orchestration] infeasible_or_unbounded and primal_and_dual_infeasible discarded although every trial has a zero objective | WP7 classifier, both are proofs |
| [oracles-orchestration] Seeded runs report solve_limit when the initial proof is merely inconclusive | dropped with seeding, the WP4 budget-1 test pins the initial trial |
| [accelerators-certificates] SoPlex Farkas symbols bound with an invented ABI inside upstream's SoPlex_ prefix | dropped, upstream first |
| [accelerators-certificates] Clp ray accessor checks solver state, not the cached status, and can return a stale ray | archived, WP3 note requires the cached status |
| [accelerators-certificates] clp_lp move constructor drops _status (pre-existing on main) | standalone fix |
| [backend-changes] MOSEK optimize-once regression tests never run by default | binary dropped, WP2 tests under the MOSEK fixtures, counting per the N5 ruling |
| [backend-changes] The destructor assertion does not test the fix | `static_assert` dropped, fake-api test kept (WP2) |
| [backend-changes] The version-warning help always prints MIPPP_NO_VERSION_WARNING as not set | dropped with `version_warning_help` |
| [backend-changes] Pre-existing on main: moving a Clp, Cbc or SCIP model drops its status (the PR fixed only SoPlex) | standalone fix, four backends |
| [backend-changes] Pre-existing on main: MOSEK raises license_error only for an expired license | WP2, own commit per the N18 ruling |
| [tests] IIS test selection is a cached configure-time snapshot of MIPPP_REQUIRED_SOLVERS covering only four solvers | binary dropped, `IisTest` per solver file |
| [tests] Classification pins failed{true} as a feasibility proof | WP7 table test, inverted |
| [tests] Wall-clock deadline test with a history of flakiness | WP4 fake clock, suite budgets of 0 s only |
| [tests] IIS binary escapes the sanitizer job and the TEST_SOURCE build reduction | binary dropped, tests in `mippp_test` and solver files |
| [tests] Filter hard-codes typed-test positions | dropped with the filter |
| [tests] Vacuous 'independent proof' in the integer-bound vector | WP8 evaluates the witnesses, WP5 drops the README claim |
| [docs-reporting-build] Status audit falsely says the README distinguishes utility from planned IIS support | audit dropped, gap table quoted in the thread |
| [docs-reporting-build] README roadmap drops IIS while the PR's docs say model-level and native IIS remain planned | reverted in PR #3, WP17 updates it |
| [docs-reporting-build] Status audit credits the PR with the MOSEK TimeLimitTest registration that main already has | audit dropped, WP1 ticks the item |
| [docs-reporting-build] docs/iis.md links point outside the docs directory and will 404 on the published site | page replaced, WP17 uses absolute URLs |
| [docs-reporting-build] Design notes list MOSEK items that main already fixed | WP1 corrections |

## Reply to the author

- **Thank you, and what lands with your name.** The deletion engine and its exhaustive test, the limit
  semantics, the diff-based updates, the crossing and constant-row ideas, the vectors and oracle, the MOSEK
  guard and slot selection, the SoPlex move fix, and the case list.
- **The gap, plainly.** At `a1a9f11` the branch has the shape iis.md lists as "Replaced": `linear_system` with
  a model factory, slack columns, public `mippp::iis::{result, options, member}`, a separate `mippp_iis_test`.
  It has no `has_iis`, `compute_iis` member, snapshot, row-bound capability or native routine, as your own
  `docs/iis.md:9-12` says. This is the API contract, not style.
- **What happens to PR #3, per the N0 ruling.** Under (a): I would like to push, without force, a cleanup, the
  engine in `mippp::detail` with your tests, and the IIS types with your vectors, then squash-merge it with
  your credit. Please say whether that suits you, and stop pushing to `astra/iis-support` meanwhile. `a1a9f11`
  is tagged, and removed code stays there and in the Deferred notes. Under (b) or (c), this paragraph states
  that path instead.
- **Ruled, open, and already on main.** Link the rulings commit. Open questions stay open until ruled, so
  please do not build against them. Your view on N1, N2, N3, N5 and N6 is welcome. `d8bb08f` already brought
  the single MOSEK optimization and the time limit.
- **Defects found, bugs surfaced.** The port fixes `failed{true}` taken as feasible, `infeasible_or_unbounded`
  discarded, the limit attribution, the sleeping deadline test and the vacuous integer check. It drops the
  SoPlex bindings and the Clp ray accessor, and records the gate rule for `has_dual_ray`. The main bugs your
  work surfaced are fixed with credit to you.
- **Offers that fit your available time,** all off the critical path: the MOSEK fixes and fallback if you have
  MOSEK, the Deferred notes, the loader fix as ruled, Gurobi and CPLEX native if still licensed, the SoPlex
  Farkas proposal upstream, and the benchmark harness later.
- **How to work.** Fresh branches from main, `git show a1a9f11:<path>` rather than cherry-picks, "Ported from
  a1a9f11 (PR #3)" in the body. Point your agent at iis.md, with its Rulings section, and the rewritten
  iis_todo.md: no generic public name, no vendor call in tests, no separate test binary.
- **Deferred, not rejected.** Ordering, the public filter, ray seeds, native elastic relaxation and the
  harness each get a return path in iis.md. Batching joins them if N3 is ruled (a). The rejected ideas carry
  their reasons.
- **Questions.** Which licenses and versions you have, which offers you take, and which email addresses GitHub
  should credit, as your commits use two.