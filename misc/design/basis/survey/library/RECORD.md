# LP basis support: library-side survey

Survey of 2026-10-09 for the design of `get_basis()` / `set_basis()`. The
repository was only read; nothing in it was edited, built or tested.

**Provenance.** The checked-out tree is local `main` at `04431ca`, which is
13 commits behind `origin/main` = `fix/review-rulings` = `506af98` (MEASURED:
`git log main..origin/main`). Those 13 commits carry what this survey must build
on: `feature_unavailable_error`, `supported_versions`, `highs_api::native_iis_release`,
N46 (every `solve()` starts with `reset_status()`), `trial_limit`, the
`release lacks: ` skip in `SkipOnLicenseError`. **The implementation must branch
from `origin/main` (`506af98`).** Unless marked `[local 04431ca]`, every
`path:line` below is on `506af98`, read from `git archive 506af98` extracted to
`survey/library/origin_main/`. Files the 13 commits do not touch have the same
lines in both trees, among them `include/mippp/model_concepts.hpp`,
`include/mippp/utility/iis_snapshot.hpp`,
`include/mippp/detail/handle_status_table.hpp`,
`include/mippp/utility/column_generation.hpp`,
`include/mippp/utility/column_manager.hpp`, `test/column_manager.cpp`,
`test/dumb_lp.hpp` (MEASURED: `git diff --stat 04431ca 506af98`).

**Legend.** MEASURED: a probe I compiled or ran (the probe is
`survey/library/probe/basis_concepts_probe.cpp` and
`probe/unlisted_tag_hard_error.cpp`, built with `/opt/gcc-15/bin/g++
-std=c++26` and `-std=c++23`, and `clang++-18 --gcc-toolchain=/opt/gcc-15
-std=c++23 -fsyntax-only`, against `origin_main/include`). DOCUMENTED: read in
the repository at the cited `path:line` (the library's own source and docs; no
vendor manual was consulted for this survey). INFERRED: my reading.

---

## 1. Concepts

### 1.1 The draft (DOCUMENTED, `include/mippp/model_concepts.hpp`)

- 605-607 banner "Basis"; 609-617 `namespace basis_status`: `basic`,
  `nonbasic`, `nonbasic_free : nonbasic`, `nonbasic_at_bound : nonbasic`,
  `nonbasic_at_lower_bound`, `nonbasic_at_upper_bound`, `nonbasic_fixed`, the
  last three `: nonbasic_at_bound`.
- 619-621 `lp_basis_status<S>` = `variant_containing_a<S, basic> &&
  variant_containing_a<S, nonbasic>`.
- 623-628 `lp_basis<B, T>`: `requires(B & basis, model_variable_t<T> v,
  model_constraint_t<T> c) { basis.get_status(v) -> lp_basis_status;
  basis.get_status(c) -> lp_basis_status; }`.
- 630-631 `model_basis_t<T> = std::decay_t<decltype(std::declval<T &>().get_basis())>`.
- 633-634 `has_lp_basis<T> = lp_basis<model_basis_t<T>, T>`.
- 636-657 `has_modifiable_lp_basis<T>`: `has_lp_basis<T> && requires(T & model,
  model_basis_t<T> b, model_variable_t<T> v, model_constraint_t<T> c,
  model_scalar_t<T> s)`: `b.set_basic(v)` (642), `b.set_nonbasic(v, s)` with the
  comment "snaps to lower/upper/free within tolerance" (643), `b.set_status(v,
  tag)` for the five leaves `basic`, `nonbasic_free`, `nonbasic_at_lower_bound`,
  `nonbasic_at_upper_bound`, `nonbasic_fixed` (644-648); the same for `c`
  (650-656), with the comment "s: row activity (a·x) to compare w/ bounds" (651).
- 659-662 `model_variable_basis_status_t<T>` and 664-667
  `model_constraint_basis_status_t<T>` = `std::decay_t<decltype(std::declval<T>()
  .get_basis().get_status(std::declval<model_variable_t<T>>()))>` (resp.
  constraint).
- 669-672 `has_lp_basis_warm_start<T> = has_lp_basis<T> && requires(T & model,
  model_basis_t<T> b) { model.set_basis(b); }`.

### 1.2 Side by side with the IIS trio and the handle concepts (DOCUMENTED)

| Point | Basis draft | IIS (implemented) | Other handle-keyed concepts |
| --- | --- | --- | --- |
| Status concept | `lp_basis_status` 619-621: contains `basic` and `nonbasic` | `lp_iis_status` 688-690: contains `absent` and `member` | `model_status_t` / `status::` tags |
| Object concept | `lp_basis<B, T>` 623-628, `B & basis` (non-const) | `lp_iis<I, T>` 733-741, `const I & iis`, plus `get_outcome()` and two `std::size_t` counts | n/a |
| Type alias | `model_basis_t<T>` 630-631, `declval<T &>()` | `model_iis_t<T>` 743-744, `declval<T &>()` | `model_variable_t<M>` 29-30, `model_constraint_t<M>` 108-111, `model_constraint_lhs_range_t<T, M>` 452-455, all `declval<M &>()` / `declval<T &>()` |
| Per-status alias | `model_*_basis_status_t<T>` 659-667, **`declval<T>()`** (rvalue) | none in the library; the test suite spells `native_iis_outcome_t<M>` with `declval<const model_iis_t<M> &>()` (`test/test_suites/iis.hpp:18-20`) | n/a |
| Model concept | `has_lp_basis<T>` 633-634, one parameter | `has_iis<T>` 746-749, one parameter | `has_dual_solution<T, M = T>` 289-293, `has_reduced_costs<T, M = T>` 295-299, `has_enumerable_*<T, M = T>` 258-268, bounds and rows readers 431-534, `has_lazy_constraints<T, M = T>` 538-543 |
| Writer | `has_modifiable_lp_basis<T>` mutates the snapshot `b` | the snapshot has no setter; native answers are built through `detail::iis_answer` and handed over (`iis_snapshot.hpp:119-159`) | `has_modifiable_*<T, M = T>` mutate the model |
| Consumer | `has_lp_basis_warm_start<T>`: `model.set_basis(b)` | `compute_iis_by_deletion(model, within)` reads any `lp_iis<Iis, M>` (N45 c, `misc/design/iis/iis.md` N45 (c)) | n/a |

The `<T, M = T>` rule (DOCUMENTED, `docs/reference/concepts.md:41-72`, memory
"handle-concepts-checked-type-first", ruling 2026-09-16): concepts that "make
sense on a handle" take the checked type first and the model second; "the others
describe whole models and stay single-parameter". `iis.md:36-39`: "`has_iis`
stays a single-parameter whole-model concept, like `has_lp_basis`." So the basis
concepts are the precedent the IIS was told to copy; `lp_basis<B, T>` /
`lp_iis<I, T>` already put the checked type first.

### 1.3 Inconsistencies

- **C1. Const-ness of the read.** `lp_basis` checks `B & basis` (625), `lp_iis`
  `const I & iis` (735). `test/iis_snapshot.cpp:68-71, 198` pins
  `!lp_iis<answer<flaw::mutable_status>, handles_only_model>`. MEASURED: a type
  whose two `get_status` are non-const satisfies `lp_basis` (probe item 2). A by-value
  snapshot has const getters (as `iis_snapshot.hpp:49-54`), so the draft is only
  looser than needed; a getter that queries the live solver would pass it.
- **C2. `declval<T>()` vs `declval<T &>()`.** `model_variable_basis_status_t`
  and `model_constraint_basis_status_t` (660-667) call `get_basis()` on an
  rvalue model; every other alias in the file uses an lvalue. MEASURED (probe
  item 7): with `B get_basis(this auto & self)`, `model_basis_t` and
  `has_lp_basis` still hold, but `model_variable_basis_status_t<M>` is not
  formed. `model_base`'s public members use `this auto & self`
  (`include/mippp/solvers/model_base.hpp:59, 69, 76, 156-240`), so a
  `get_basis` placed in a base with deducing-this would make the alias fail
  while the concept holds. With a plain member function (as every
  `get_solution`/`get_reduced_costs` is, `highs_lp.hpp:110-136`) both work
  (MEASURED, probe item 6). The aliases also call `get_status` on a prvalue
  snapshot; a const lvalue (`declval<const model_basis_t<T> &>()`) matches C1.
- **C3. Five leaf tags required on every modifiable basis.**
  `has_modifiable_lp_basis` (644-648, 652-656) requires `set_status` with
  `basic`, `nonbasic_free`, `nonbasic_at_lower_bound`, `nonbasic_at_upper_bound`
  and `nonbasic_fixed`. `handle_status_table::set` is deleted for a tag the
  variant does not list (`handle_status_table.hpp:36-41`, pinned at
  `test/iis_snapshot.cpp:296-299`). MEASURED (probe items 4-5 and
  `unlisted_tag_hard_error.cpp`): a snapshot over a HiGHS-like variant without
  `nonbasic_fixed` (HiGHS has `Lower`, `Basic`, `Upper`, `Zero`, `Nonbasic`
  and no fixed code, `include/mippp/solvers/highs/impl/v1/highs_api.hpp:203-209`)
  - satisfies the concept when `set_status` is an unconstrained template
    forwarding to the table, then fails with "use of deleted function" when
    `set_status(v, nonbasic_fixed{})` is instantiated: the concept lies;
  - does not satisfy it when `set_status` is constrained on what the table
    accepts;
  - satisfies it when the variant lists all five leaves.
  So either every modifiable snapshot lists five leaves (a "writable" variant
  wider than what the solver reports, against the per-routine-variant rule of
  IIS, `iis.md:59-82`), or `set_status` maps a tag onto a listed one (dropping a
  refinement, the very thing the table's deleted overload forbids,
  `handle_status_table.hpp:36-37`: "A tag the variant does not list would
  convert to the base alternative it derives from, silently dropping its
  refinement"), or the concept shrinks.
- **C4. `set_basic(x)` duplicates `set_status(x, basic{})`** (642 vs 644, 650 vs
  652). Surface redundancy; nothing else in the library has such a pair
  (INFERRED).
- **C5. No base tags in the writer.** The writer accepts only leaves. Neither
  `nonbasic` nor `nonbasic_at_bound` can be written, although they are what a
  user knows when he does not know the side (INFERRED). They would be the
  natural spelling of "nonbasic, let the backend pick the side from the bounds
  at `set_basis` time" (see 1.4), mirroring IIS's plain `member` ("its finite
  sides form one unit", `docs/reference/concepts.md:134`).
- **C6. Unused parameters.** `has_modifiable_lp_basis` declares `T & model`
  (639) and never uses it; the requirement is entirely on the snapshot
  (DOCUMENTED). It signals that the drafter anticipated a model-side setter.
- **C7. `set_basis` takes a non-const lvalue.** 672 checks
  `model.set_basis(b)` with `b` a non-const `model_basis_t<T>` lvalue: a
  `set_basis(B &)` would satisfy it and reject a `const` snapshot. MEASURED: a
  `set_basis(const B &)` satisfies it (probe item 3). Checking `const
  model_basis_t<T> &` would pin the intent.
- **C8. No completion/validity report.** `lp_iis` requires `get_outcome()`
  (738); `lp_basis` has nothing. A model may have no basis to report: before any
  solve, after a solve that used interior point without crossover
  (`include/mippp/solvers/mosek/impl/v1/mosek_base.hpp:76-77`: "The basic slot
  is missing when basis identification is off, and a stopped solve can leave the
  two slots in different states"), after a stop, after a modification that
  invalidates it, after SoPlex's reload "without one"
  (`include/mippp/solvers/soplex/impl/v1/soplex_lp.hpp:439-451, 515`). An
  all-alternative-0 snapshot is not a basis. Options: `get_basis()` throws
  `solver_error` (the solver rejects the request, binding ruling); a validity
  flag or optional; documented UB (misuse ruling). To decide.
- **C9. Doc vs header.** `docs/reference/concepts.md:127` says `set_nonbasic(v,
  value)` "snaps to the nearest bound"; the header says "snaps to
  lower/upper/free within tolerance" (643). `concepts.md:128` promises "a backend
  may accept other basis sources as well, such as a basis view built from
  lambdas" — nothing checks it (an api-doc-sync item).
- **C10. Handle applicability.** `has_lp_basis` is single-parameter by the
  `iis.md:39` precedent. The only handle that could carry a basis is a
  node-relaxation callback handle (`has_node_relaxation_callback`,
  `model_concepts.hpp:789-796`), which no backend implements
  (`docs/reference/concepts.md:189`). `has_dual_solution` /
  `has_reduced_costs`, the other LP-solution reads, are `<T, M = T>` (289-299).
  Keeping one parameter matches `has_iis`; `<T, M = T>` would let a future
  node handle claim it without a break. To decide (INFERRED: one parameter now
  costs nothing until such a handle exists, and adding a defaulted `M` later is
  source-compatible for single-argument uses).
- **C11. `lp_basis_status` pins no alternative 0.** `iis_snapshot`'s
  `requires` pins `absent` as alternative 0 of both status variants and
  `incomplete` of the outcome (`iis_snapshot.hpp:20-28`), and its tests pin the
  rejection (`test/iis_snapshot.cpp:272-294`). A basis snapshot template would
  have to pin it too, and differently per entity kind (see 2.2).
- **C12. `colgen::variable_status<Status>` value-initializes `Status`.**
  `column_generation.hpp:69-75, 194-203`: a column's last known basis status
  starts as alternative 0 of the model's variable status variant (INFERRED from
  `property_initial_value`). Whatever alternative 0 the snapshot pins is also
  what a fresh master column reads before the first `master_refreshed`.

### 1.4 Requirements a by-value snapshot cannot meet without a model reference

- `b.set_nonbasic(v, s)` (643): "snaps to lower/upper/free within tolerance"
  needs the variable's two bounds and a tolerance, plus the knowledge that the
  variable is free; a by-value snapshot holds statuses only. Same for
  `b.set_nonbasic(c, s)` (651) with the row's two sides and the row activity.
  The library's bounds are read through `has_readable_variable_bounds` /
  `has_readable_constraint_bounds` (431-446, 499-516), i.e. from the model.
- The draft also says nothing about *which* infinity: bounds read back "exactly
  as the solver holds them", and `is_infinite` compares magnitude against
  `infinity()` (`model_base.hpp:54-62`), so a snapping rule needs the model's
  `infinity()` too.
- Options (to decide):
  (a) drop `set_nonbasic` from the concept, keep `set_status(x, tag)`, and give
  generic code a free helper over the model, e.g. a status-from-value function
  taking `(model, v, value, tolerance)`;
  (b) move the snapping to the model: `set_basis` resolves a written base tag
  (`nonbasic`, `nonbasic_at_bound`, C5) against the bounds current at
  `set_basis` time;
  (c) the snapshot keeps a model pointer: breaks the "holds no reference to the
  model" contract the enumeration and the IIS snapshot share
  (`docs/reference/concepts.md:90`, `iis.md:85-104`, N21 `iis_pr_plan.md`), and
  dangles once the model is moved (every model is move-constructible and not
  move-assignable, e.g. `highs_base.hpp:68-80`, `clp_lp.hpp:57-72`; the 11
  backends grep identically) or destroyed;
  (d) capture bounds and sides at `get_basis()` time: 16 more bytes per entity
  (two `double`), stale after any bound change, and wrong for an entity added
  later.
- The draft's own docs (`concepts.md:128`) already decouple warm start from
  modifiability ("Warm-starting does not require that type to be modifiable").

### 1.5 Probe results (MEASURED, GCC 15 `-std=c++26` and `-std=c++23`, Clang 18 `-std=c++23`, all static_asserts pass)

`probe/basis_concepts_probe.cpp`:

1. a by-value snapshot of two `handle_status_table`s with const `get_status`
   satisfies `has_lp_basis<M>`;
2. `lp_basis` accepts a mutable-only getter (C1);
3. `set_basis(const B &)` satisfies `has_lp_basis_warm_start` (C7);
4. a modifiable snapshot over a variant without `nonbasic_fixed`, whose
   `set_status` is an unconstrained template, satisfies
   `has_modifiable_lp_basis`;
5. the same with `set_status` constrained on the table's accepted tags does not;
   with all five leaves listed it does (C3);
6. `model_variable_basis_status_t` / `model_constraint_basis_status_t` give the
   two variants on a plain-member `get_basis`;
7. on `B get_basis(this auto & self)`, `model_basis_t` and `has_lp_basis` hold
   but `model_variable_basis_status_t` is not formed (C2);
8. the commented `column_manager.hpp:170-176` snippet,
   `master_refreshed<model_variable_basis_status_t<M>>{rc, value,
   basis.get_status(var)}`, compiles against such a snapshot and has type
   `master_refreshed<variable status variant>`;
9. at run time: `get` past the bound reads alternative 0 (a variable variant
   whose alternative 0 is `nonbasic_at_lower_bound` reads it at id 7; a row
   variant whose alternative 0 is `basic` reads `basic` at id 5); `set(2, …)` on
   a table of bound 2 throws `std::out_of_range`; `count_a<nonbasic>()` over 4
   ids with one `basic` written returns 3 (unwritten ids count).

`probe/unlisted_tag_hard_error.cpp`: forwarding `nonbasic_fixed` into a table
whose variant lacks it is "error: use of deleted function
`handle_status_table<…>::set(std::size_t, Tag)`".

### 1.6 Every statement about the basis in `misc/design/iis/*.md` (DOCUMENTED, quoted)

- `iis.md:38-39`: "`has_iis` stays a single-parameter whole-model concept, like
  `has_lp_basis`."
- `iis.md:59-62`: "**An IIS object shaped like a basis.** Both paths return the
  same snapshot template by value, like `get_basis()`, and nothing is stored in
  the model (Q2 a). Its `get_status(v)` and `get_status(c)` return a variant over
  the tags of an `iis_status` namespace, mirroring `basis_status`".
- `iis.md:68-74`: "The per-entity storage is `detail::handle_status_table<Status>`,
  a `std::vector` of the variant itself, two bytes per handle id for every IIS
  status variant (N45 e), where ids past the bound read as the variant's
  alternative 0 (N16 b). The snapshot template requires `iis_status::absent` as
  that alternative, and `get_basis()` later instantiates the same table with
  `basis_status` variants."
- `iis.md:85-87`: "Handles carry no reverse index to the caller's keys, so the
  consumer iterates its own variable and constraint families and queries each
  entity, as for a basis."
- `iis.md:578-581` (HiGHS): `Highs_deleteColsByRange(model, 0, -1)` "deletes
  nothing and returns before the model status and basis (`5d05cb0`)"; and in
  `highs_base.hpp:793-801` the wrapper relies on it leaving "the data, the
  Hessian, the basis and the model status alone".
- `iis.md:762-785`: warm starts per backend, measured for the deletion filter
  (see section 3).
- `iis.md:1128, 1143`: SoPlex binds `SoPlex_basisRowStatus`; "the C API cannot
  drop a basis, so `solve()` reloads the LP when a row reports `ZERO`
  (`08eac9c`)".
- `iis.md:1299-1301`: the IIS page is "reading an IIS, with the basis-like
  consumer loop".
- `iis.md:1361-1363` (Deferred, forcing): "Setters on the IIS object, shaped like
  the basis setters, are the natural home."
- `iis.md:1457-1458` (Q2): "`compute_iis()` returns the snapshot by value like
  `get_basis()`, rather than storing it for a `get_iis()`."
- `iis.md:1584-1586`: "**N16. IIS and LP basis support.** (b) Ruling: "Basis
  implementation will handle the factorization burden if needed." IIS comes
  first, its storage a `detail` template that basis support reuses." (INFERRED:
  "factorization" is used in these documents for code factoring, as in N45's
  "factorizations", so the ruling licenses the basis work to refactor the shared
  storage it needs.)
- `iis.md:1788-1790` (N37 d): "The tag hierarchy stays over an enum, for
  consistency with `status` and `basis_status` and for the refinement door of
  N1".
- `iis_pr_plan_pre_rulings.md:141-145`: "**N16. IIS and LP basis support.** The
  2026-09-23 comment says IIS "will fit well alongside LP basis support in a
  future v1.1.0 release". The basis concepts exist (`model_concepts.hpp:576-634`),
  but no backend implements `get_basis`, so the IIS snapshot is the first
  handle-keyed one. (a) Design the per-entity storage jointly with `get_basis`.
  (b) IIS first, its storage a `detail` template that basis support reuses. (c)
  Independent designs. Recommended: (b), which keeps basis work off the critical
  path. Blocks WP5." (Line numbers inside are stale; the draft is at 604-672.)
- `iis_pr_plan_pre_rulings.md:35` and `iis_pr_plan.md` (Q2): "returns the
  snapshot by value like `get_basis()`".
- `iis_pr_plan_pre_rulings.md:201-202`, `iis_pr_plan.md:512-513`: "On 2026-09-23
  the maintainer wrote that IIS "will fit well alongside LP basis support in a
  future v1.1.0 release"".
- `iis_pr_plan.md:125-126` (N20): "a basis dump needs only variables, so one
  concept per entity kind beats (c)".
- `iis_pr_plan.md:718, 722-724` (WP5): "`iis_status` tags next to
  `basis_status` …", "over `detail::handle_status_table<Status>` (N16 b). One
  byte per handle id, ids past the bound reading as alternative 0" (one byte
  became two under N45 e).
- `iis_todo.md:159-161` (2.3, done): "The public snapshot template `iis_snapshot`
  sits over `detail::handle_status_table<Status>`, which basis support reuses
  later (N16 b)."
- `iis_api_review.md:68-74`: "`has_iis`, `model_iis_t` and `lp_iis<I, T>`
  (model_concepts.hpp:692-709) copy the basis trio one for one: `lp_basis`,
  `model_basis_t` and `has_lp_basis` (623-634). … The library returns numeric
  per-entity results as mappings (`solution[x]`) and categorical ones as an
  object with `get_status(v)` and `get_status(c)` over tag variants (the basis).
  An IIS is categorical, so `iis_snapshot` (iis_snapshot.hpp:50-55) takes the
  basis shape." (The IIS line numbers cited there are stale: 733-749 now.)
- `iis_api_review.md:510` (Rejected): "The basis has no member list either".
- `iis_api_review.md:513` (Rejected): "The library keeps numbers apart from
  categories, as the basis keeps duals out of `basis_status`".
- No statement in `iis_todo.md` or the review assigns the basis any work item
  beyond reusing the table.

---

## 2. Reuse

### 2.1 What is reusable as is (DOCUMENTED unless marked)

- `detail::handle_status_table<Status>` (`include/mippp/detail/handle_status_table.hpp:15-56`):
  `std::vector<Status>`; `id_bound()`; `set(id, status)`; `get(id)`, `noexcept`
  when copying is (43-46); `count_a<Tag>()` (50-55). It is pinned for basis
  variants already: `TEST(handle_status_table, holds_basis_statuses)`
  (`test/iis_snapshot.cpp:372-383`) builds a table over `variant<nonbasic_at_lower_bound,
  basic, nonbasic_at_upper_bound>`, reads `basic`, reads alternative 0 at an
  unwritten id and at id 7 past the bound, and counts `count_a<nonbasic>() == 2`.
- `is<T>` / `is_a<T>` and the `variant_*` concepts (`include/mippp/utility/variant.hpp:31-54`);
  `test/variant.cpp:7` states "the one include is the point: the basis API needs
  these without status.hpp".
- The sizing and translation pattern of native snapshots:
  `detail::iis_answer<…> answer(_handle_id_bound(num_col), num_row)`
  (`highs_base.hpp:769`, `gurobi_base.hpp:765`, `cplex_base.hpp:722`), the native
  index translated through `_var_handle(col).uid()` (`highs_base.hpp:856-860`),
  with `_handle_id_bound` returning `_remap_ids ? _native_ids_map.size() :
  num_native_ids` (`remapping_model_base.hpp:87-93`: "a table sized by the native
  count would drop the handles above it"). Non-remapping backends size by the
  native column count (`xpress_base.hpp:647`, `copt_base.hpp:527-529`). Rows:
  no backend removes constraints (`has_remove_variable` only,
  `model_concepts.hpp:599-603`), so constraint ids are dense and the row table
  is sized by `num_constraints()`.
- The snapshot's shape: `get_status(Variable) const noexcept` and
  `get_status(Constraint) const noexcept` reading `.uid()`
  (`iis_snapshot.hpp:49-54`), `requires(!std::same_as<Variable, Constraint>)` (20),
  copyable (`TEST(iis_snapshot, copies_answer_alike)`, `test/iis_snapshot.cpp:472-479`).
- The decoder helper pattern: `detail::iis_row_status_by_sense<LessEqual,
  GreaterEqual>(sense)` (`iis_snapshot.hpp:91-103`) maps a native sense to a
  tag; a basis decoder for solvers that report a row as "nonbasic" without side
  would have the same shape (INFERRED; which solvers do so is a vendor-survey
  question).
- The "five native bases build every exit through one builder" factoring (N45 b)
  is a pattern, not code: `iis_answer` itself is IIS-specific
  (`flag_variable` / `flag_constraint` with sides and `whole`,
  `finish(Outcome)`, `iis_snapshot.hpp:119-159`), so it does not transfer
  (INFERRED).
- `detail::restore_guard` (`include/mippp/detail/restore_guard.hpp`, N45 a) for
  any solver option a `set_basis` must set and restore.

### 2.2 What blocks reuse

- **`set()` past the bound throws.** `handle_status_table.hpp:31-35` ("The bound
  is fixed at construction: an id past it throws std::out_of_range rather than
  growing the table"), pinned by `TEST(handle_status_table,
  set_past_the_bound_throws)` (`test/iis_snapshot.cpp:355-359`). MEASURED
  (probe 9). The column-generation warm start is exactly that case:
  `get_basis()`, `add_column()`, then `b.set_status(new_var, …)` throws. Options
  (to decide): (a) the table grows on `set` (the IIS contract changes, and the
  test above changes with it); (b) a separate growable write path or a basis
  table type; (c) `set_basis` never needs the user to write new entries,
  because it reads alternative 0 for them (next bullet).
- **`get()` past the bound returns alternative 0.** `handle_status_table.hpp:13-14,
  43-46`. If `set_basis(b)` walks the model's live entities (`variables()` /
  `constraints()`, `_live_variables` on remapping backends,
  `remapping_model_base.hpp:72-86`; `clp_lp.hpp:82-91`) and reads
  `b.get_status(e)` for each, a snapshot shorter than the model pads the new
  entities with alternative 0 and a snapshot longer than the model ignores
  removed ids (INFERRED). Then no explicit write is needed for added columns or
  rows, provided alternative 0 is the right default **per entity kind**:
  - a new column: nonbasic at a bound (the test already pins
    `nonbasic_at_lower_bound` as alternative 0 of a variable variant,
    `test/iis_snapshot.cpp:373-381`);
  - a new row: its slack `basic`.
  Both keep the basis square (INFERRED, standard LP: one more column nonbasic,
  or one more row with one more basic). So unlike `iis_snapshot`, whose
  `requires` pins one tag (`absent`) for both tables (`iis_snapshot.hpp:23-26`),
  a basis template must pin two different alternatives 0. Trap for the vendor
  survey: "nonbasic at lower bound" is not a valid status for a column whose
  lower bound is infinite (a free column, or one bounded above only): either the
  backend corrects it at `set_basis` time from the current bounds (option (b)
  of 1.4) or the solver must accept/repair it.
- **`set()` is deleted for tags the variant does not list** (36-41). Good for
  reading (no silent drop); a basis write may legitimately want a drop (write
  `nonbasic_fixed` on HiGHS, which has no fixed code) — see C3.
- **`count_a` counts unwritten ids below the bound** ("even when alternative 0
  derives from Tag", 48-49). MEASURED (probe 9: 3 of 4). On a remapping backend
  the variable table's bound includes free (removed) handle ids
  (`remapping_model_base.hpp:87-93`), and on `clp_lp` / `dumb_lp` removed ids stay
  in the native numbering (`clp_lp.hpp:273-288`, `test/dumb_lp.hpp:246-254`), so
  `count_a<nonbasic>()` over a variable table counts dead entities when
  alternative 0 is a nonbasic tag. `count_a<basic>()` stays exact as long as no
  removed id is written `basic` and alternative 0 of the row table is the only
  `basic` default (rows are never removed). INFERRED consequence: on a valid
  basis, `count_a<basic>` over both tables equals `num_constraints()`, a free
  validity check for tests; member counts in the snapshot (as `iis_snapshot`
  precomputes, 43-46) are not required by `lp_basis`.
- **Keyed by uid, N8.** `iis_snapshot.hpp:15-17`: "a handle created afterwards
  may reuse the id of a removed entity, and then reads that entity's entry";
  pinned for IIS by `check_answer_survives_a_later_removal`
  (`test/test_suites/iis_cases.hpp:650-674`). For a basis this is worse than for
  an IIS: `set_basis(b)` after `remove_variable(x)` + `add_variable()` (recycled
  id) gives the new column `x`'s old status, possibly `basic` (INFERRED). Ids are
  recycled on `clp_lp`, `dumb_lp` and the three remapping bases
  (`docs/solving/updates.md:80`; `remove_variable.hpp:471-482`
  `enumeration_lists_recycled_ids_in_order`). Under the misuse ruling, this is a
  documented "describes the model as it was" (N8 a) unless the consequence is
  judged severe.
- **Memory.** Two bytes per id for these empty-tag variants (`iis.md:68-70`, N45 e).
  Fine for a basis (INFERRED).

---

## 3. Column generation

### 3.1 What the library already anticipates (DOCUMENTED)

- `colgen::variable_status<lp_basis_status Status>` (`column_generation.hpp:194-203`):
  "last known basis status of a master column", a property whose handler copies
  `e.get<variable_status<Status>>()`.
- `colgen::master_refreshed<lp_basis_status Status> = event<reduced_cost,
  variable_value, variable_status<Status>>` (`column_generation.hpp:247-251`):
  "a restricted master solve refreshed a master column, exposing its LP primal
  value and basis status alongside its reduced cost".
- The event list in the header comment: "master_refreshed  a master solve
  refreshed a master column" (`column_generation.hpp:49-51`).
- The commented example (`column_manager.hpp:168-176`):
  `auto solution = model.get_solution(); auto reduced_costs =
  model.get_reduced_costs(); auto basis = model.get_basis();
  mgr.update_master_columns([&](const auto &, const auto & var) { return
  master_refreshed<model_variable_basis_status_t<M>>{reduced_costs[var],
  solution[var], basis.get_status(var)}; });`, and the comment at 150-154: "an
  update may carry a reduced cost, but also a basis status, a primal value, or
  any user-defined payload".
- `test/column_manager.cpp:29-36`: `fake_basis_status = variant<basic,
  nonbasic_free, nonbasic_at_lower_bound, nonbasic_at_upper_bound,
  nonbasic_fixed>` "spelled out over the leaf statuses since the fake model
  carries no real basis"; `manager` lists `variable_status<fake_basis_status>`
  in the master state (38-43, 86-87); `master_refresh_updates_value_and_basis_status`
  (157-180) broadcasts `master_refreshed<fake_basis_status>{-4.0, 2.5 *
  var.id(), nonbasic_at_upper_bound{}}`.
- `test/test_suites/column_manager.hpp` (the shared `ColumnManagerTest`) and
  `test/test_suites/cutting_stock.hpp` use no basis: solve, read duals, price,
  `manage_columns`, loop (`column_manager.hpp:102-138`; `cutting_stock.hpp:69`).
  `ColumnManagerTest` requires `has_dual_solution`, `has_column_generation`,
  `has_remove_variable`, `has_enumerable_variables` (18-26) and is instantiated
  on `clp_lp`, `cplex_lp`, `gurobi_lp`, `highs_lp`, `highs_qp`
  (`test/solvers/*.cpp`, grep). `examples/cutting_stock/main.cpp:58` solves in a
  loop without a basis.

### 3.2 Would the snippets compile against a real basis type?

MEASURED (probe 8): yes, against a by-value snapshot with
`get_status(variable) const` returning the model's variable status variant, as
long as `model_variable_basis_status_t<M>` is formed, i.e. `get_basis()` is not
a deducing-this member (C2). `master_refreshed` takes `Status` by value, so
only the returned variant matters, never the snapshot type. A real backend's
variant would be per backend (as IIS's are), not `fake_basis_status`, and the
`colgen::variable_status` property is then per model type: generic colgen code
spells it through `model_variable_basis_status_t<M>`, which therefore must stay
a public alias (INFERRED).

### 3.3 The column-generation workflow and whether it needs `set_basis`

- README: "Since a MIP++ model *is* the solver's native model, re-solves after
  in-place modifications (adding rows, changing bounds) almost always warm-start
  from the last basis implicitly. The roadmap item above is about explicit basis
  get/set — transferring a basis between models or storing one for later."
  (`README.md:191-192`).
- Contradicted by `docs/algorithms/column-generation.md:93`: "LP-basis warm
  starts (`has_lp_basis_warm_start`) — so each master re-solve starts from the
  previous basis instead of from scratch — are specified but not yet
  implemented". The README's claim is the measured one for some backends: the
  deletion-filter table (`docs/algorithms/deletion-filter.md:200-215`;
  `iis.md:762-785`) measured warm re-solves on `glpk_lp` ("GLPK's setters keep
  the basis", 1 iteration against 30), `highs_lp` / `highs_qp` (21 pivots
  against 149), `soplex_lp` "warm, mostly" (a trial that frees the active side
  of a nonbasic row reloads, `soplex_lp.hpp:439-451, 515`), and cold
  `mosek_lp` ("interior point, does not warm-start"); `clp_lp`, Gurobi, CPLEX,
  Xpress, COPT were not measured (`deletion-filter.md:215`). `clp_lp.hpp:608-612`:
  dropping the scale factors "forces a recompute and keeps the warm-start
  basis". A doc fix is due whatever the design (api-doc-sync).
- So for an in-place colgen loop (solve, price, `add_column`, solve), an explicit
  `set_basis` is **not** needed on the backends measured warm (INFERRED from the
  above). Explicit get/set buys:
  1. reading statuses to drive pricing/eviction (`variable_status`, e.g.
     "never evict a basic column"); `manage_columns` evicts then activates in one
     call (`column_manager.hpp:226-292`), so a basis read before it is stale on
     both sides afterwards (INFERRED);
  2. restoring a basis after a dive or a branch-and-price node changed bounds
     and the solver's own basis moved on;
  3. transfer between models: models are non-copyable and move-only (all 11
     backends delete copy and move-assignment, e.g. `highs_base.hpp:68-80`),
     so a second master (rebuilt, per thread, per node) can only be seeded by
     a basis keyed by handle ids that match (INFERRED: two models built in the
     same order give the same ids);
  4. backends that do not warm-start by default: MOSEK's interior point
     (`deletion-filter.md:210`), which also leaves no basic slot
     (`mosek_base.hpp:76-77`) — vendor survey to say whether `set_basis` + the
     simplex optimizer fixes that;
  5. after `remove_variables`: whether each solver keeps a valid internal basis
     when a column is deleted (and what happens if it was basic) is a vendor
     question to list (INFERRED that the colgen eviction predicates
     `above<reduced_cost>` / `window_above` (`column_generation.hpp:315-354,
     478-488`) select columns with positive reduced costs, which are nonbasic at
     an optimal basis of a minimization, so evictions usually keep the basis
     square).
- The workflow a colgen user needs (INFERRED): `solve()` → `get_basis()` (and
  duals, reduced costs) → feed `master_refreshed` → `manage_columns` (adds
  columns, removes some) → optional `set_basis(b)`: entities removed since are
  ignored by id, entities added since read alternative 0 (column nonbasic at a
  bound, row basic) → `solve()`.

---

## 4. Tests

### 4.1 How a capability suite is written (DOCUMENTED)

- `test/test_suites/all.hpp`:
  - `INSTANTIATE_TEST(model_name, test_suite, model_type)` (13-15),
    `TEST_EPSILON` (17);
  - `is_required_solver` / `MIPPP_REQUIRED_SOLVERS` (24-41);
  - `backend_unavailable = "backend unavailable: "` (46) and `release_lacks =
    "release lacks: "` (49), both matched verbatim by `misc/tools/compat_matrix.py`;
  - `MIPPP_API_VERSION_TEST(prefix, Api, key)` (71-94);
  - `model_test<Api, Model>` (96-173): `construct_api(solver_key, args...)`
    once per process from `SetUpTestSuite()` (110-140: a license error or a
    load failure becomes `unavailable_reason`; a required solver's load failure
    becomes `missing_required_api`; a probe `Model(*api)` checks the license);
    `SetUp()` fails/skips every test (146-150); `new_model()` (152);
    `SkipOnLicenseError(f)` (160-172) skips on `license_error`, and on
    `feature_unavailable_error` skips with `release_lacks` unless
    `Api::support_of(api->library_version()) == release_support::validated`,
    where it FAILS ("a validated release lacks a member").
    `[local 04431ca]`: `SkipOnLicenseError` at `all.hpp:118` catches only
    `license_error`.
  - the suite headers are included at 175-211, alphabetically.
- Templates to copy: `ReducedCostsTest` (`test/test_suites/reduced_costs.hpp:10-111`):
  fixture `template <typename T> struct X : public T { using typename
  T::model_type; static_assert(has_reduced_costs<model_type>); }`,
  `TYPED_TEST_SUITE_P`, `GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST`, every
  body in `this->SkipOnLicenseError([this]() { using namespace operators; auto
  model = this->new_model(); … })`, `REGISTER_TYPED_TEST_SUITE_P`.
  `DualSolutionTest` (`dual_solution.hpp`) is the same over constraint handles.
  `LpStatusTest` (`lp_status.hpp:10-16`) asserts `lp_model` only.
  `IisTest` (`iis.hpp:17-279`) is the template for a suite with fixture member
  functions shared with a second path (`iis_cases::fixture<T, Path>`,
  `iis_cases.hpp:473-…`), skip-by-capability with `if constexpr(!has_x<M>)
  GTEST_SKIP() << "no x"` (`iis_cases.hpp:620-624, 650-654`), and
  `skip_reason` keyed on data (561-569). Its removal cases
  (`check_removed_variable_is_skipped`, `check_answer_survives_a_later_removal`,
  620-674) are the model for "the snapshot describes the model as it was".
- Per-backend instantiation: `test/solvers/<solver>.cpp`, e.g. `highs.cpp:17-46`:
  `#include "test_suites/all.hpp"`, `MIPPP_API_VERSION_TEST(HiGHS_api,
  highs_api, "HIGHS")`, `struct highs_lp_test : public model_test<highs_api,
  highs_lp> { static void SetUpTestSuite() { construct_api("HIGHS"); } };`, one
  `INSTANTIATE_TEST(HiGHS_lp, Suite, highs_lp_test);` line per suite.
  Capability pins sit next to them: `static_assert(has_iis<highs_lp>)`,
  `static_assert(!has_iis<highs_milp>)` with the comment "the routine explains
  the relaxation only" (`highs.cpp:182-199`), the outcome variant pinned with
  `std::same_as<native_iis_outcome_t<highs_lp>, std::variant<…>>` (185-190);
  the same in `gurobi.cpp:124-143`, `cplex.cpp:140-157`, `xpress.cpp:294-304`,
  `copt.cpp:104-116`, and `!has_iis` pins in `glpk.cpp:171, 235`, `cbc.cpp:68`,
  `scip.cpp:49`, `clp.cpp:65`, `mosek.cpp:143, 235`, `soplex.cpp:17`. A basis
  suite would pin `static_assert(!has_lp_basis<*_milp>)` on every MILP class
  (ruling "*_milp models expose pure MILP features only").
- A backend's own tests key a release skip on the floor with the same prefix:
  `highs_iis_test<Model>::SetUp()` skips `release_lacks << "compute_iis() needs
  HiGHS …"` below `highs_api::native_iis_release` (`highs.cpp:217-234`), and
  `TEST(HiGHS_lp, compute_iis_below_native_floor_throws)` (288-…). CONTRIBUTING:
  "A backend's own tests, in `test/solvers/<name>.cpp`, may key a skip on the
  release, through the api's floor constant and with the same prefix"
  (`CONTRIBUTING.md:473-476`); shared suites never skip on a solver or a release
  (465-466, `.claude/skills/solver-backend-conformance/SKILL.md:125-128`).
- One suite per capability concept (`SKILL.md:49-50`: "a backend claims a
  capability by instantiating the suite, and only then"); IIS has two suites for
  two concepts (`IisTest`, `IisByDeletionTest`); readers and writers are
  separate suites elsewhere (`ReadableVariablesBoundsTest` /
  `ModifiableVariablesBoundsTest`). So `has_lp_basis` and
  `has_lp_basis_warm_start` (and `has_modifiable_lp_basis` if kept) would be
  separate suites (INFERRED, to decide).
- Unsupported suites stay as commented `// INSTANTIATE_TEST(...)` lines
  (`SKILL.md:71-74, 113-115`; e.g. `highs.cpp:85`).
- Solver-agnostic rule (memory, ruling 2026-10-06): no per-solver skip for a
  behaviour difference; the wrapper adapts, keyed on version if needed.

### 4.2 Registration (DOCUMENTED)

- ctest registers one test per GoogleTest suite automatically, from
  `--gtest_list_tests` after each build (`test/cmake/mippp_suite_tests.cmake:1-43`,
  `test/cmake/mippp_list_suite_tests.cmake:1-93`); a new typed suite needs no
  CMake change. Exit 77 = all skipped (`mippp_suite_tests.cmake:30`).
- Only timing-dependent tests go in `MIPPP_SERIAL_TESTS`
  (`test/CMakeLists.txt:126-142`; `CONTRIBUTING.md:499-504`).
- A new solver-free unit test file (e.g. a basis snapshot test beside
  `test/iis_snapshot.cpp`) must be added to `add_executable(mippp_test …)`
  (`test/CMakeLists.txt:35-61`), as must a new `doc_snippets/<page>.cpp`
  (58-59). Solver files are listed in `MIPPP_TEST_ALL_SOLVER_SOURCES` (7-20) and
  filtered by `TEST_SOURCE` (21-33).
- The header self-containment check compiles every public header alone
  (`test/CMakeLists.txt:144-166`): a new `utility/<x>.hpp` gets it for free.

### 4.3 `test/dumb_lp.hpp` as a reference basis model (DOCUMENTED / INFERRED)

- `dumb_lp` stores the model in plain structures and, in `solve()`, builds a
  fresh Clp model, `loadProblem`, `primal`, copies the solution and duals, then
  `deleteModel` (`test/dumb_lp.hpp:446-525`): no basis survives a solve, so
  every re-solve is cold today.
- `clp_api` already binds `Clp_setColumnStatus`, `Clp_getColumnStatus`,
  `Clp_setRowStatus`, `Clp_getRowStatus` as required symbols
  (`include/mippp/solvers/clp/impl/v1/clp_api.hpp:65-68, 132-135`).
- `dumb_lp`'s handle uid is the Clp column index (removed columns are kept,
  zeroed and fixed, ids recycled: `dumb_lp.hpp:246-257, 104-114`), so
  `get_basis()` can read the statuses before `deleteModel` into a cached pair of
  tables, and `set_basis(b)` can store statuses to replay with
  `Clp_setColumnStatus` / `Clp_setRowStatus` before the next `primal`
  (INFERRED). That makes `dumb_lp` a reference `has_lp_basis` +
  `has_lp_basis_warm_start` model, matching the rule "New suites should run
  against it first" (`SKILL.md:51-53, 89-91`). It is instantiated on Clp in CI
  (`test/solvers/dumb.cpp:7-26`) and excluded from the feature tables
  (`tested_features_table.py:13-14`). Its capability pins live at the end of
  `dumb_lp.hpp:542-563`.
- For engine-level tests that need no solver at all, the precedent is
  `test/iis_snapshot.cpp` (snapshot + table over synthetic tables and a
  `handles_only_model`, 28-33) and `test/column_manager.cpp`'s `fake_model`
  (22-27).

---

## 5. Feature tables and matrix

### 5.1 `docs/assets/features_tables/tested_features_table.py` (DOCUMENTED)

- Walks `test/`, skipping `dumb.cpp` (9-17); regex
  `^INSTANTIATE_TEST\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\);` (27) — anchored at
  line start, so commented lines do not count; the third argument decides the
  table: `_lp_` or `_qp_` → LP table, `_milp_` → MILP table (37-40); the first
  argument is the column, `_lp`/`_milp` stripped and `_qp` → " QP" (20-23).
- Rows come from the instantiated suites; `formated_test_names` (45-83) gives
  order and label ("IIS, native" for `IisTest`, 59); a suite missing from that
  list is appended at the end under its raw name (86-97). So a new
  `LpBasisTest` becomes a row by instantiation alone, and gets a label and a
  position by adding one tuple to `formated_test_names` (INFERRED).
- The features are read from instantiations, not from concepts (the skill says
  so: `SKILL.md:9-13`). `make features_tables` runs it (`Makefile`), needs
  `xelatex` and `pdftocairo` (192-205), and the PNGs
  `docs/assets/features_tables/{lp,milp}_table_{light,dark}.png` are committed.
  N45 (h) recorded that "`make features_tables` changed no pixel" for IIS since
  the rows already existed.

### 5.2 `misc/tools/compat_matrix.py` and `compat_manifest.json` (DOCUMENTED)

- Each manifest solver has one `gtest_filter` by prefix: `Clp*:clp*`,
  `Cbc*:cbc*`, `GLPK*:glpk*`, `HiGHS*:highs*`, `SCIP*:scip*`,
  `SoPlex*:soplex*`, `Gurobi*:gurobi*`, `MOSEK*:mosek*`, `Xpress*:xpress*`,
  `COPT*:copt*`, `CPLEX*:cplex*` (DOCUMENTED, `misc/tools/compat_manifest.json`, read with a JSON loader). A new suite
  instantiated as `HiGHS_lp/LpBasisTest/0` or a `highs_lp_test.*` TEST_F is
  selected with no manifest edit.
- `CORE_SUITES = {"LpModelTest", "MilpModelTest", "QpModelTest"}`
  (`compat_matrix.py:89-94`): a `release lacks: ` skip there breaks the claim;
  anywhere else it makes the row ☑️ partial (81-88). A basis suite is not core,
  so a `feature_unavailable_error` from `get_basis`/`set_basis` on a partial
  release makes the row partial, as intended (`CONTRIBUTING.md:343-347, 361-364, 465-475`).
- `VERSION_TEST` and `CLAIM_PROPERTIES` (63-79) read the version test's
  properties; nothing basis-specific.

---

## 6. Docs

### 6.1 Every mention of the basis (DOCUMENTED, `506af98`)

- `README.md:181`: "| 🔴 | **LP basis warm-starts** | Concepts specified
  (`has_lp_basis`, `has_lp_basis_warm_start`); no backend implements
  `get_basis`/`set_basis` yet |".
- `README.md:191-192` (NOTE): "Since a MIP++ model *is* the solver's native
  model, re-solves after in-place modifications (adding rows, changing bounds)
  almost always warm-start from the last basis implicitly. The roadmap item
  above is about explicit basis get/set — transferring a basis between models or
  storing one for later."
- `README.md:194`: "The first three items are what build-bound, re-solve-heavy
  research code wants most."
- `paper/paper.md:139-142` (`[local 04431ca]` 153-156): "several features useful
  to re-solve-heavy research code — explicit LP basis warm-starts, SOS
  constraints, user-cut callbacks, and heuristic-solution injection — are on the
  roadmap rather than in the current release". (JOSS submission is pending,
  memory "joss-submission-blockers": a paper edit is a JOSS-visible change.)
- `docs/reference/concepts.md:95`: "`is`, `is_a` and the `variant_*` concepts
  behind them live in `utility/variant.hpp` and serve the basis statuses too."
- `docs/reference/concepts.md:101` (`has_status_reset`): "Only the reported
  status changes: the model's data and the solver's warm start stay."
- `docs/reference/concepts.md:126`: "| `has_lp_basis` | `get_basis()`, whose
  `get_status(v)` / `get_status(c)` report the LP basis as a variant over the tags
  of namespace `basis_status`. *(no backend yet)* |"
- `docs/reference/concepts.md:127`: "| `has_modifiable_lp_basis` | `has_lp_basis`,
  and the basis returned by `get_basis()` can be edited in place: `set_basic`,
  `set_nonbasic(v, value)` (snaps to the nearest bound) and `set_status(v, tag)`,
  each for variables and for constraints. *(no backend yet)* |"
- `docs/reference/concepts.md:128`: "| `has_lp_basis_warm_start` | `has_lp_basis`,
  plus `set_basis(b)` accepting the basis type of `get_basis()`. Warm-starting
  does not require that type to be modifiable, and a backend may accept other
  basis sources as well, such as a basis view built from lambdas. *(no backend
  yet)* |"
- `docs/reference/concepts.md:74-80`: the "Concept declared ≠ backend provides"
  note on *(no backend yet)* rows.
- `docs/solving/solutions.md:83-85`: "## Sensitivity and warm starts / LP basis
  access (`has_lp_basis`) and basis warm starts (`has_lp_basis_warm_start`) are
  specified in `model_concepts.hpp` — including a full `basis_status` hierarchy
  — but **no backend implements them yet**; they are the top item on the
  roadmap. Until then, a re-solve after a model change starts from whatever the
  backend itself keeps internally".
- `docs/solving/solutions.md:72-81` (Reduced costs): `if(rc[x] > tolerance) { /*
  x is nonbasic at its lower bound */ }` — a basis reading spelled through
  reduced costs.
- `docs/solving/updates.md:98`: "For LPs, basis warm starts are specified but not
  yet implemented on any backend — see Solutions."
- `docs/algorithms/column-generation.md:93` (What's next): quoted in 3.3; it
  contradicts `README.md:192`.
- `docs/getting-started/index.md:90`: "LP basis warm starts and user-cut callbacks
  are specified as concepts but not yet implemented by any backend … Column
  generation without basis warm starts leaves performance on the table".
- `docs/solvers/index.md:106` (`[local 04431ca]` 95): "**SOS constraints and LP
  basis warm starts** — specified as concepts, not yet implemented by any
  backend."
- `docs/solving/status-and-limits.md:105`: `reset_status()` "changes nothing
  else: the model's data stay, and so does what the solver keeps for a warm
  start."
- `docs/solving/infeasibility.md:283`: the filter's "trials use the solver's MIP
  starts, basis and incumbent as any `solve()` does".
- `docs/algorithms/deletion-filter.md:24` (oracle keeps "a warm start"), `:129`
  ("MIP starts, basis and incumbent | used by each trial … | what the trials
  left"), `:200-215` (warm/cold table).
- No mention in `docs/getting-started/coming-from.md`,
  `docs/solvers/compatibility.md`, `CONTRIBUTING.md`, `CITATION.cff` (grep).

### 6.2 Conventions of `docs/reference/concepts.md` rows (DOCUMENTED)

- Sections with two-column tables `| Concept | Provides |`
  (`concepts.md:82-210`); the basis rows sit under "## Solution information"
  (120-128); IIS under "## Infeasibility analysis" (130-140).
- A row states the members, the return types' concept (`lp_iis`), the
  `model_*_t` alias as "the only portable spelling" with `auto`
  (`concepts.md:135, 140`), the satisfying classes by name ("Satisfied by
  `gurobi_lp`, …"), version gates ("throws `mippp::feature_unavailable_error`
  when the HiGHS loaded at runtime is older than `highs_api::native_iis_release`,
  1.14"), status effects ("afterwards `get_status()` reports `unknown`") and a
  link to the guide section. Undefined behaviour is stated in one line (misuse
  ruling). *(no backend yet)* marks a specified-only row.

### 6.3 How the IIS feature was documented (DOCUMENTED)

- User page `docs/solving/infeasibility.md`: problem first ("A first IIS",
  7-51, a full program assembled from snippet sections), then "Two paths"
  table (53-80), "Reading the answer" (82), "Repairing the model" (124), "How a
  run ends" (154), "Limits" (209), "Narrowing an answer" (241), "LP or MILP"
  (257), "The model afterwards" (281), "A snapshot of the model as it was"
  (302), "Support by model" (306, per-model table linking limitation bullets),
  "Native changes" (325), "Next" (330). Algorithm page
  `docs/algorithms/deletion-filter.md`. Nav entries in `zensical.toml:29-37`.
- Code is never inline: `test/doc_snippets/infeasibility.cpp` holds sections
  `// --8<-- [start:<name>]` … `[end:<name>]` (e.g. 22-25, 39-47), included by
  the page with `--8<-- "test/doc_snippets/infeasibility.cpp:<name>"`
  (`infeasibility.md:15-33`); expected output in `.txt` files beside
  (`infeasibility_workshop.txt`, `infeasibility_repair.txt`); tests below the
  sections run them on a real backend through a `model_test` fixture
  (`CONTRIBUTING.md:506-531`). `make check-snippets` runs
  `misc/tools/check_doc_snippets.py` (one start and one end per section,
  `check_doc_snippets.py:1-52`); CI runs it and `zensical build --clean`.
- Example program: `examples/infeasible_transportation/main.cpp`, registered in
  `examples/CMakeLists.txt:10-13`, each example a self-contained project with
  its own `CMakeLists.txt` and `conanfile.py`, listed in `examples/README.md`.
- Per-backend limitations: bullets with anchors in `docs/solvers/index.md:100-…`
  ("Native IIS on HiGHS" … `{ #limitation-… }`), and the release gate sentence
  in `docs/solvers/index.md:60-62`.

### 6.4 Where basis docs would go (proposal, INFERRED)

- Reading a basis: replace the stub `docs/solving/solutions.md:83-85`
  ("Sensitivity and warm starts") with a section on `get_basis()`, the tags,
  the consumer loop over the caller's families (as `iis.md:85-87` says the IIS
  copied from the basis), alternative 0 for entities added later, N8.
- Setting a basis: `docs/solving/updates.md:84-98` ("Giving the solver a
  starting point", beside MIP starts), replacing line 98.
- Or a page of its own, "Bases and warm starts", in the nav between
  "Solutions, duals and reduced costs" and "Re-solving and model updates"
  (`zensical.toml:31-32`), with `test/doc_snippets/<page>.cpp` (+ `.txt`) added
  to `test/CMakeLists.txt:58-59`.
- Rows `concepts.md:126-128` rewritten to the implemented contract (drop *(no
  backend yet)*, name classes, fix C9).
- `column-generation.md:91-93` rewritten: a `master_refreshed` example with the
  real basis, and the README's correct "in-place re-solves already warm-start"
  statement; `README.md:181` row removed or moved, `README.md:175` core list,
  `docs/solvers/index.md:106`, `docs/getting-started/index.md:90`,
  `docs/solving/updates.md:98`, and `paper/paper.md:140-141` (JOSS-visible)
  updated in the same change (api-doc-sync skill).
- Per-backend limitations (e.g. a backend that cannot set a basis, MOSEK slots,
  SoPlex's reload of a nonbasic free row) as anchored bullets in
  `docs/solvers/index.md`.

---

## 7. Versioning

### 7.1 `feature_unavailable_error` (DOCUMENTED, `506af98` only)

- Type: `include/mippp/utility/solver_exceptions.hpp:17-24`: "A member the
  loaded release lacks, as releases a backend supports only partially may (see
  `<solver>_api::supported_versions`); the model itself stays usable."
  `class feature_unavailable_error : public solver_error`.
- Rules (`CONTRIBUTING.md:432-476`, "Supporting a release partially"): such a
  member throws `feature_unavailable_error`, "never a plain `solver_error`, and
  throws it before it changes the model's data"; its message "names what is
  missing, the library's path and the release it reports"; it "keys on the entry
  point when the entry point is what is missing (`find_function` returned
  `nullptr`) … and on a release floor otherwise. A floor is a public `static
  constexpr solver_version` of the api class, as `highs_api::native_iis_release`
  is, never a literal in the model code, and a `static_assert` beside it checks
  that it is at or below the `from` of every range of `validated_versions`"; it
  "is never called from a core suite".
- The HiGHS floor example: `highs_api.hpp:357-372`: `static constexpr
  solver_version native_iis_release{1, 14};`, `validated_versions =
  {{1, 14}, {1, 16}}`, `supported_versions = {{1, 7, 2}, {1, 16}}`, and
  `static_assert(std::ranges::all_of(validated_versions, … native_iis_release <=
  r.from), "a validated release never lacks native IIS")`. The throw:
  `highs_base.hpp:747-766` (`if(loaded && *loaded <
  highs_api::native_iis_release) throw feature_unavailable_error(concat_str(
  "mippp: compute_iis() needs HiGHS ", …, " or later, the loaded library '",
  library_path, "' reports ", …))`, then a second throw when `Highs->getIis ==
  nullptr`). An empty `library_version()` (devel build) "requires the symbol and
  assumes the newest regime" (N26).
  `[local 04431ca]`: `highs_base.hpp:707` has a private
  `_iis_native_floor{1, 14, 0}` and throws `solver_error`.
- The entry-point example: `highs_milp.hpp:47-66` (`_add_mip_start`): `if(!Highs->setSparseSolution)
  throw feature_unavailable_error("mippp: add_mip_start() needs
  Highs_setSparseSolution, which the loaded library '…' (reporting …) does not
  export")`, thrown before `_reset_cache()` touches anything.

### 7.2 Optional symbols, `probe_symbols`, `support_of` (DOCUMENTED)

- Optional entry points: a second X-macro list resolved with
  `lib.find_function<F>("name")`, `nullptr` when absent:
  `HIGHS_OPTIONAL_FUNCTIONS` (`highs_api.hpp:336-338`: `Highs_setSparseSolution`,
  `Highs_getIis`), `GRB_OPTIONAL_FUNCTIONS` (`gurobi_api.hpp:285-298`),
  `SOPLEX_OPTIONAL_FUNCTIONS` (`soplex_api.hpp:111-129`, including
  `SoPlex_basisRowStatus`, 120). Required ones use `lib.get_function` and make
  the api fail to construct (`CONTRIBUTING.md:557-566`).
- `probe_symbols` is library *acceptance*, not feature gating:
  `try_load_solver_library` rejects a candidate that lacks any probe symbol
  (`include/mippp/detail/solver_library.hpp:214-229`); only Cbc declares one
  (`cbc_api.hpp:190-192`, "Ubuntu ships a libCbc.so without the C API"). Ruling
  D1 (memory "two-tier-version-support"): refuse only a release whose api cannot
  be built — so a basis symbol must never become a probe symbol or a required
  symbol unless every supported release exports it.
- `solver_api<Derived>` (`solver_library.hpp:405-535`): `supported_ranges()`
  (487-492), `support_of(release)` (496-506, `untested` for an empty version),
  `check_library_version` (457-468), `library_version()` (532-534).
- `library_version()` gating in tests: `highs.cpp:217-234` (skip below the floor
  in the backend's own fixture), `all.hpp:160-172` (shared wrapper).

### 7.3 What the basis needs, from the library side (DOCUMENTED / INFERRED)

- Already bound, as **required** symbols (so every supported release has them):
  HiGHS `Highs_getBasis` / `Highs_setBasis` and the `BasisStatus` enum
  (`highs_api.hpp:203-214, 332-333`); COPT `COPT_GetBasis` / `COPT_SetBasis`
  (`copt_api.hpp:187-189, 300-301`); Clp `Clp_get/setColumnStatus`,
  `Clp_get/setRowStatus` (`clp_api.hpp:65-68, 132-135`); GLPK
  `glp_get/set_row_stat`, `glp_get/set_col_stat` (`glpk_api.hpp:89-92, 244-247`).
- Optional: SoPlex `SoPlex_basisRowStatus` only (`soplex_api.hpp:26, 61, 120`),
  used to detect a nonbasic free row (`soplex_lp.hpp:443-450`).
- Gurobi reads/writes `VBasis` / `CBasis` through the generic
  `GRBgetintattrarray` / `GRBsetintattrarray` it binds
  (`gurobi_api.hpp:134-137, 262-263`; used for IIS at `gurobi_base.hpp:700-712,
  811-819`).
- CPLEX, Xpress, MOSEK: nothing basis-specific bound (grep).
- So on HiGHS no symbol gate is needed; any gate would be on behaviour across
  1.7.2–1.15 (the vendor survey's question). A new symbol for another backend
  goes in that backend's `*_OPTIONAL_FUNCTIONS` list if some supported release
  lacks it, with a `feature_unavailable_error` at the call, or in the required
  list if all have it.

---

## 8. Design record

### 8.1 Structure of `misc/design/iis/iis.md` to copy (DOCUMENTED)

- Intro paragraph (`iis.md:1-28`): what the note records (rulings by date), "It
  is not user documentation", where the implementation order lives
  (`iis_todo.md`), what was probed on which releases, and the legend: "Notes on
  these probes mark a run *measured*, a vendor manual *documented*, and a
  reading of either *inferred*", plus "Line numbers are on main at `f5e833f`
  unless marked otherwise".
- "## Shape of the API" (30-170): one bold-titled bullet per decision, each
  ending with the ruling id(s) in parentheses, "(Q1 b)", "(N16 b)".
- "## Native routines" (171-709): a per-solver table (solver and probed
  releases, entry point, semantics, partial answers, time-limit behaviour) then a
  bullet per solver with dated probe results and commit hashes.
- Algorithm and library-addition sections (710-1203), "## Tests and
  documentation" (1204-1343), "## Deferred, and how they would come back"
  (1344-1406), "## What the pull request contributed" (1407-1435).
- "## Rulings of <date>" sections (1436-2307): each N-bullet is `**Nk. Title.**
  (letter) Ruling: "quoted words". *Confirmed reading.* …`, followed by "What
  the rulings overturn" bullets (1732-1756). New rulings append dated sections;
  older entries get "*Amended by Nk on <date>.*".
- "## Open questions" last (2344-…), saying none remains or listing them.
- Companion documents: `iis_pr_plan.md` (questions with options (a)/(b)/(c),
  "Recommended:", "Evidence:", "Blocks WPk", work packages WP1-WP17),
  `iis_pr_plan_pre_rulings.md` (the version the maintainer ruled on, kept),
  `iis_todo.md` (checkbox steps with branch and hashes, "Waves"),
  `iis_api_review.md` (Verdict, What follows the patterns, Rejected table,
  Pending decisions).
- For `misc/design/basis/basis.md`: the same intro + legend, "Shape of the API",
  a per-backend native table, "Tests and documentation", "Deferred", dated
  rulings, "Open questions"; the vendor facts from the other survey go in the
  per-backend table with their probe dates.

### 8.2 Naming and placement rules (DOCUMENTED)

- `iis.md:161-171` (Names): "No generic identifiers such as `result`, `options`
  or `member` in a namespace users are told to open." Tags live in their own
  namespace (`iis_status`, `basis_status`); other names are prefixed (`iis_`,
  `deletion_`) "except the verb `compute_iis_by_deletion`".
- N19 (`iis.md:1597-1610`, `iis_pr_plan.md:86-116`): "(a) Everything in
  `include/mippp/utility/`, namespace `mippp`, each new name prefixed …".
  `colgen` is a sub-namespace "only because its vocabulary is generic"
  (`column_generation.hpp:20-23`). Public templates in `utility/`
  (`utility/iis_snapshot.hpp`), storage in `detail/`
  (`detail/handle_status_table.hpp`), backend helpers in a header's `detail`
  section (`iis_snapshot.hpp:68-161`).
- Already fixed names: `basis_status`, `lp_basis_status`, `lp_basis`,
  `model_basis_t`, `has_lp_basis`, `has_modifiable_lp_basis`,
  `model_variable_basis_status_t`, `model_constraint_basis_status_t`,
  `has_lp_basis_warm_start`, and in colgen `variable_status`, `master_refreshed`.
  A snapshot would follow as `basis_snapshot` (or `lp_basis_snapshot`) in
  `utility/basis_snapshot.hpp` (INFERRED from N19's pattern); N37 (b): docs and
  tests obtain the type through `model_basis_t<M>` or `auto` and never spell the
  template's arguments.
- Suite names: `<Concept>Test` with a label in `formated_test_names` (N19's
  "IIS, native").

### 8.3 Candidate questions for the maintainer (from sections 1-3; not decided here)

- B1 Read const-ness and aliases: tighten `lp_basis` to `const B &` and the
  aliases to `declval<const model_basis_t<T> &>()` / `declval<T &>()` (C1, C2).
- B2 Modifiability: keep `has_modifiable_lp_basis` (five leaves in every
  writable variant, or a mapping setter that drops refinements, or a narrower
  concept), or drop it in favour of `set_basis` reading any `lp_basis` (C3-C6).
- B3 `set_nonbasic(x, value)`: drop, move to the model, model pointer, or
  captured bounds (1.4).
- B4 Alternative 0 per entity kind: variable `nonbasic_at_lower_bound`, row
  `basic`, and who repairs a free column (2.2).
- B5 Growth: may `set` past the bound grow the table (changes the IIS-pinned
  contract) (2.2).
- B6 No basis available: `solver_error`, optional/validity flag, or UB (C8).
- B7 One- or two-parameter `has_lp_basis` (C10).
- B8 Does `set_basis` reset the status (N46 (c): "only `solve()` writes a status
  back"; whether a solver discards its solution on set is the vendor survey's).
- B9 Recycled ids after removal: documented UB / "as it was" (N8 a), or a guard.
- B10 One suite per concept, and which classes (`*_lp`, `highs_qp`; never
  `*_milp`).

---

## 9. Conventions

- **Comment policy.** `CONTRIBUTING.md:533-540` says only "Match the surrounding
  code: naming, header layout, and idioms already in the file take precedence".
  The user-level `comment-discipline` skill (`~/.claude/skills/comment-discipline/SKILL.md`)
  is the default when the project is silent: a comment must name a mistake a
  reader could make (contracts, traps, non-obvious mechanics); no restatement,
  reassurance, history narration, or prose a test already pins. It was applied
  to every header on 2026-09-16 (memory "pre-tag-quality-pass": "do not re-review
  the comment policy on include/"), and the seventh pass set "minimal, only what
  an expert cannot deduce" (memory "v1-tag-pre-release-review"). The in-tree
  style is present-tense invariants, e.g. `handle_status_table.hpp:13-14,
  31-32, 36-37, 48-49`.
- **clang-format.** `.clang-format`: `BasedOnStyle: Google`, `IndentWidth: 4`,
  `UseTab: Never`, `BreakConstructorInitializers: BeforeComma`,
  `PointerAlignment: Middle`, `DerivePointerAlignment: false`,
  `SpaceBeforeParens: Never`, `AccessModifierOffset: -4`, `IncludeBlocks:
  Preserve`. `make check-format` runs `clang-format --dry-run -Werror` over
  `include` and `test`; CI pins clang-format 18.1.8 (`CONTRIBUTING.md:284-286`).
  Concept bodies are wrapped in `// clang-format off` / `on`
  (`model_concepts.hpp:129, 311, 387, …`).
- **Includes.** `make check-includes` runs `misc/tools/check_std_includes.py
  include` (`check_std_includes.py:1-16`): a file must include the standard
  header of every `std::` name it uses, except headers the standard mandates
  (`<ranges>` → `<iterator>` → `<concepts>`); reaching a std header through
  another mippp header does not count. Memory "mechanical-include-insertion-trap":
  `<version>` must precede any `__cpp_lib` gate.
- **Language and compilers.** C++23, GCC 14 and Clang 18 in CI, C++26 features
  only behind a fallback (`CONTRIBUTING.md:543-547`); libc++ fallbacks (memory).
- **MSVC pitfalls** (memory "msvc-template-pitfalls"; MSVC is CI-only):
  explicit-object overloads that differ by a deduced parameter are not partially
  ordered (constrain the deduced one, `model_base.hpp:164-166`); ADL poison
  pills use the one-argument form; enclosing using-directives do not reach a
  generic lambda (in-tree: `test/test_suites/column_manager.hpp:75-79`,
  `test/doc_snippets/infeasibility.cpp:35-37`); `using enum` through a member
  alias of a class template brings nothing in. A `/bigobj` is already set
  (`test/CMakeLists.txt:78-86`).
- **CHANGELOG.** None exists (`find` returned nothing). The release history is
  the commit log and the design notes.
- **Version.** `include/mippp/version.hpp:5-7`: `MIPPP_VERSION_MAJOR 1`,
  `MINOR 0`, `PATCH 0`, the single source parsed by `CMakeLists.txt:3-10` and
  `conanfile.py:37-56`; `CITATION.cff:17` `version: 1.0.0`. v1.0.0 is tagged
  (memory). The maintainer's own words place basis support "in a future v1.1.0
  release" (`iis_pr_plan_pre_rulings.md:141-142`); the rest-or fixes are also a
  minor release (memory). Adding a public feature is additive (minor bump);
  changing the drafted concepts touches public names that shipped in 1.0.0
  without any implementation (INFERRED: no user can have a model satisfying them
  except their own types).
- **Commits.** Memory "no-claude-commit-trailer": no Claude Co-Authored-By
  trailer; the assistant pushes only on request; one build at a time with
  `-c tools.build:jobs=4` (memory "limit-local-build-jobs").

---

## 10. Anything else an implementer must know

- **Branch base.** Start from `origin/main` (`506af98`), not the checked-out
  `04431ca` (see Provenance).
- **Copy/move.** Every model deletes copy construction and both assignments and
  keeps a `noexcept` move constructor (`highs_base.hpp:68-80`, `clp_lp.hpp:57-72`,
  `cbc_milp.hpp:92-114`, `glpk_base.hpp:90-100`, `scip_milp.hpp:104-117`,
  `copt_base.hpp:87-102`, `xpress_base.hpp:84-97`, `soplex_lp.hpp:92-106`,
  `mosek_base.hpp:140-156`, `gurobi_base.hpp:105-121`, `cplex_base.hpp:92-108`).
  A snapshot that references its model would dangle after a move; the IIS
  snapshot holds none (`iis_snapshot.hpp:29-35`).
- **Eager vs lazy translation.** `get_solution()` and `get_reduced_costs()` on
  `highs_lp` capture `this` and translate handle → native lazily at lookup
  (`highs_lp.hpp:110-136`); `iis.md:90-92`: "Handle results translate lazily
  today …, so the snapshot is the first eager one." A basis meant to survive
  later removals and moves must translate eagerly, as the IIS does.
- **Native ids.** `native_id(v)` / `native_id(c)` are public on every model
  (`has_native_handles`, `model_concepts.hpp:837-845`; `highs_base.hpp:98-99`
  returns `_native_id(v)`), so a user can already address the solver's basis
  through `native_api()`; `_native_id` / `_var_handle` / `_handle_id_bound` are
  protected members of `remapping_model_base` (`remapping_model_base.hpp:40-47,
  90-93`), reachable from the backends' bases (`highs_base`, `gurobi_base`,
  `cplex_base`) without a friend. A deducing-this hook *in*
  `remapping_model_base` called from a leaf class needed `friend
  remapping_model_base<...>` (`iis_pr_plan_pre_rulings.md:30-33`).
- **Enumeration.** `variables()` / `constraints()` (`has_enumerable_*`,
  `model_concepts.hpp:255-268`): "Every live entity once, in increasing id order,
  as a snapshot of the call"; the lazy `entity_range` where nothing is removed
  (`model_base.hpp:64-78`, with a `static_assert` that a removing backend
  overrides `variables()`), a `std::vector` on `clp_lp` (`clp_lp.hpp:82-91`),
  `dumb_lp` (`dumb_lp.hpp:108-114`) and the remapping bases
  (`highs_base.hpp:51-53` through `_live_variables`). This is what a generic
  `set_basis(const lp_basis<…> auto &)` would iterate.
- **Removal.** Only `clp_lp`, `cplex_*`, `gurobi_*`, `highs_*` (and `dumb_lp`)
  have `remove_variable` (grep). HiGHS deletes lazily and remaps
  (`highs_base.hpp:202-253`); Clp keeps the column, zeroed and fixed at [0, 0],
  and recycles its id (`clp_lp.hpp:273-288`, `iis.md:93-96`), so its native
  column count includes dead columns that `set_basis` must still give a status
  (INFERRED: nonbasic). No backend removes constraints.
- **Status rules.** N46 (a): every `solve()` starts with `reset_status()`
  (`highs_lp.hpp:97-102`); N46 (c): "every call that runs the solver starts from
  `unknown`, and only `solve()` writes a status back" (`iis.md:2325-2333`);
  `has_status_reset` keeps "the solver's warm start" (`model_concepts.hpp:281-287`,
  `concepts.md:101`). A `get_basis()` runs no solve, so by these rules it leaves
  the status alone; `set_basis` changes solver state without solving (B8).
  `reset_status` on MOSEK also resets a slot cache (N22,
  `mosek_lp.hpp:103`, `mosek_milp.hpp:169`): a MOSEK basis read must use the
  basic slot, which `_pick_continuous_slot` may not pick
  (`mosek_base.hpp:76-91`).
- **SoPlex interplay.** `soplex_lp::solve()` reloads the LP without a basis when a
  row is nonbasic free (`soplex_lp.hpp:439-451, 515`): a user basis with such a
  row would be silently discarded at the next solve (INFERRED).
- **HiGHS IIS interplay.** `_compute_iis()` calls `Highs_deleteColsByRange(model,
  0, -1)`, relied on to leave "the basis and the model status alone"
  (`highs_base.hpp:793-801`); the deletion filter's trials "use the solver's …
  basis … as any `solve()` does" and leave "what the trials left"
  (`deletion-filter.md:129`). INFERRED: a `get_basis()` before and a
  `set_basis()` after `compute_iis_by_deletion` would restore the warm start;
  the filter could do it itself through `if constexpr(has_lp_basis_warm_start<M>)`
  (not in scope; N37 (g) lists what the filter saves).
- **`*_milp` ruling.** Basis members go on `*_lp` and `highs_qp` only (memory
  ruling 2026-07-22); `iis.md:129-134` applies the same rule to IIS
  (`highs_milp` has no `compute_iis()`). Classes: `clp_lp`, `copt_lp`,
  `cplex_lp`, `glpk_lp`, `gurobi_lp`, `highs_lp`, `highs_qp`, `mosek_lp`,
  `soplex_lp`, `xpress_lp` (grep of `class *_lp|_qp`).
- **Native escape hatch warning.** `docs/solvers/index.md` warns that native
  modifications invalidate MIP++ features (N23); a basis set through
  `native_api()` is outside any guarantee.
- **Distinctness checks.** `model_base::_check_distinct` asserts in debug that
  raw entry lists name each entity once (`model_base.hpp:261-283`); a basis
  written from user-supplied pairs would want the same, but a snapshot keyed by
  id has no duplicates by construction (INFERRED).
- **Probes.** Memory "probes-need-process-timeouts": wrap any solver probe in
  `timeout -k`; memory "session-shell-lacks-bashrc-exports": set
  `MIPPP_SOPLEX_LIBRARY` for test runs.

---

## Files of this survey

- `/tmp/claude-1000/-home-plaiseek-Projects-mippp/66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/library/RECORD.md` (this record)
- `/tmp/claude-1000/-home-plaiseek-Projects-mippp/66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/library/probe/basis_concepts_probe.cpp`
- `/tmp/claude-1000/-home-plaiseek-Projects-mippp/66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/library/probe/unlisted_tag_hard_error.cpp`
- `/tmp/claude-1000/-home-plaiseek-Projects-mippp/66409ca5-709c-4c59-b3c1-e7b746a2b680/scratchpad/survey/library/origin_main/` — `git archive 506af98`, the tree the line numbers refer to.
- Commands run: `/opt/gcc-15/bin/g++ -std=c++26 -Wall -Wextra -I origin_main/include basis_concepts_probe.cpp` then the binary with `LD_LIBRARY_PATH=/opt/gcc-15/lib64` (built and run); `/opt/gcc-15/bin/g++ -std=c++23 -fsyntax-only` (passes); `clang++-18 --gcc-toolchain=/opt/gcc-15 -std=c++23 -fsyntax-only` (passes); `/opt/gcc-15/bin/g++ -std=c++26 -fsyntax-only unlisted_tag_hard_error.cpp` (fails as expected, "use of deleted function").
