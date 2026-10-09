# LP basis support: design plan

Plan for implementing the basis concepts drafted in `model_concepts.hpp`
(section "Basis") on every backend that can support them, in the shape of the
IIS feature (`iis_snapshot`, `detail::handle_status_table`, N16 b). It rests on
a survey of 2026-10-09: one read-only agent per LP backend, each with measured
probes against the real libraries, one on the library's own patterns, one on
prior art. Their records, probe sources and outputs are in
[survey/](survey/) (`survey/<backend>/RECORD.md`). A review of this plan by
six reviewers, each finding checked by an independent verifier against those
records, corrected it the same day. Line numbers are on main at `506af98`. A
claim is *measured* when a probe ran it, *documented* when a vendor header,
manual or source says it, *inferred* otherwise; a fact below is measured
unless marked.

Releases probed: Clp 1.17.4, 1.17.6, 1.17.8, 1.17.10, 1.17.11 and devel; GLPK
5.0 (4.59 to 5.0 documented from its NEWS); HiGHS 1.10.0 and 1.15.1; SoPlex
6.0.4 and 7.1.3; Gurobi 11.0.3, 12.0.1 and 13.0.2 (10.x, in the validated
range, is not installed); CPLEX 22.1.1 and 22.1.2; Xpress 45.01 and 47.01;
COPT 8.0.5 (7.2.5 by its exported symbols only, its license is refused here);
MOSEK 11.0.14, 11.1.11 and 11.2.6.

Nothing is implemented yet. Each question gives the options, a
recommendation and the evidence; the [proposed shape](#proposed-shape)
assumes every recommendation. The maintainer ruled B1, B4 to B7 and B9 to B13
on 2026-10-09, asked about B2, B3 and B8, whose entries below were revised
in answer, then ruled them the same day; the rulings are recorded in
[basis.md](basis.md).

## What the survey found

| Model | Read | Write | Row codes name | Fixed code | Code for "nonbasic, at no bound" | "No basis" signal | Warm start under the wrapper's settings |
|---|---|---|---|---|---|---|---|
| `clp_lp` | `Clp_statusArray` (to bind) | `Clp_copyinStatus` (to bind) | the activity | 5 | 0 free, 4 superbasic | no status array: the per-entity getters segfault | yes |
| `glpk_lp` | `glp_get_col_stat`/`row_stat` (bound) | `glp_set_col_stat`/`row_stat` (bound) | the activity | NS | NF, free entities only | none, a basis always exists | yes (presolve off) |
| `highs_lp` | `Highs_getBasis` (bound) | `Highs_setBasis` (bound) | the activity | none | Z3 free, N4 "nonbasic, no side" | info `basis_validity`, also unavailable after any change | yes |
| `soplex_lp` | `SoPlex_basisColStatus`/`RowStatus` (row bound, optional) | **none in the C API** | the activity | 2, once a simplex ran | 3, free entities only | none, a slack basis is fabricated | n/a |
| `gurobi_lp` | `VBasis`/`CBasis` (bound) | the same, plus `GRBreset` (to bind) | **no side** | none | -3 superbasic | error 10005, also after any update that carries a change | yes, once updated and reset |
| `cplex_lp` | `CPXgetbase` (to bind) | `CPXcopybase` (to bind) | the slack: **'L' rows flipped** | none | 3 free or superbasic | error 1262 | yes (`ADVIND` 1) |
| `xpress_lp` | `XPRSgetbasis` (to bind) | `XPRSloadbasis` (to bind) | the slack: **every row flipped** | none | 3 superbasic, also bounded after a stop | error 429 after barrier only; `PRESOLVESTATE` bit 7 | yes |
| `copt_lp` | `COPT_GetBasis` (bound) | `COPT_SetBasis` (bound) | the activity | 4 | 3 superbasic | return code 3; zeros after `COPT_Reset` | yes |
| `mosek_lp` | keys of `MSK_SOL_BAS` via `MSK_getsolution` (bound) | `MSK_putskc`/`putskx` (to bind) | the activity | FIX | SUPBAS, also bounded after a stop | slot undefined, or any `UNK` or `INF` key | **no: interior point by default** |

Facts that shape the questions:

- **Every solver extends its own basis the same way.** A column added to a
  model gets "nonbasic at a bound" and a row gets "basic", so the basis stays
  square. A snapshot extended by the same rule (new column at lower, new row
  basic) is a valid starting basis on every backend that can write one. It
  re-solved in 0 iterations on Clp, HiGHS, Gurobi, CPLEX, Xpress, COPT and
  MOSEK, whose probes added a column that stays nonbasic at the optimum;
  GLPK's probe added a profitable column (optimum 25 to 30), which then
  entered in 2 iterations against 7 from scratch.
- **Removals are where the backends diverge.** No backend removes rows.
  `clp_lp` keeps a removed column in Clp, fixed at [0, 0], until an addition
  reuses its id; its status byte is untouched until the next solve, which
  makes it 5. HiGHS compacts its arrays: removing a basic column leaves fewer
  basic entries than rows, completed at the next solve (1 iteration). CPLEX
  drops the entry and repairs. On Gurobi, `remove_variable` discards the
  basis and the internal warm start (measured on a basic column). A snapshot
  taken before a basic column is removed is one basic short over the live
  entities: Gurobi discards it and starts cold, the others repair it.
- **Sides are advisory, counts are not.** Every solver snaps a side that
  contradicts the bounds (at upper on an infinite upper bound, a wrong side
  on a `<=` row, free on a bounded column), at the next solve or at the
  write. On a boxed entity or a ranged row both sides are legal and the
  written side is kept; a wrong one costs a bound flip, free under the dual
  simplex (HiGHS, COPT, Xpress, MOSEK's dual) and 1 to 3 iterations under the
  primal (Clp, CPLEX on ranged rows). A wrong number of basic entries is
  where they diverge: Gurobi and COPT discard the whole basis and start cold,
  Clp, CPLEX, HiGHS, Xpress and MOSEK repair it, and GLPK fails every later
  solve (`GLP_EBADB`, reported `failed`, or `GLP_ESING`, reported
  `numerical_failure`) until a valid basis is written. GLPK can also abort
  the process (an assertion in its factorizer, `bflib/sgf.c:185`) on a basis
  holding a basic column with no matrix entry.
- **No writer takes a value.** Every native setter takes status codes only:
  a superbasic value is the solver's choice.
- **Writing needs no bound.** The native "at lower" code (HiGHS: "nonbasic",
  code 4) is snapped by every solver to the bound the entity has. CPLEX's row
  encoder needs the row sense, since only ranged rows carry a side there; a
  CPLEX free row (stored as 'L' with rhs 1e20) has no legal nonbasic code.
- **A basis needs a simplex run.** The wrapper's solves end with one on every
  backend except:
  - after presolve proves infeasibility (Gurobi, CPLEX, COPT; Xpress returns
    the slack basis, or the previous solve's basis on a re-solve) or
    unboundedness (Gurobi, CPLEX, Xpress likewise; COPT keeps a basis);
    SoPlex then returns a slack basis it fabricates, measured after
    unboundedness and inferred for infeasibility;
  - on HiGHS, after any end other than optimal of an LP that presolve
    reduced (measured for infeasibility and an iteration limit on 1.10.0 and
    1.15.1, inferred from `Highs.cpp:1680-1691` for the other statuses); an LP
    presolve does not reduce keeps a basis at any limit;
  - after a time limit before the simplex starts (Gurobi, CPLEX, MOSEK,
    Xpress 47.01);
  - after a `solve()` the wrapper answers itself without a simplex: no
    column (`highs_lp`, `clp_lp`, `soplex_lp`), crossed sides (`glpk_lp`
    through `GLP_EBOUND`, `soplex_lp`), bounds that hold no value (`xpress_lp`
    below 46.1.3); the solver keeps whatever basis it held;
  - on `mosek_lp`, whose interior point identifies a basis but ignores one it
    is given.
- **Iteration limit 0 does not prove a warm start.** From an optimal basis it
  reports `iteration_limit` on Gurobi (default method), CPLEX, Xpress, MOSEK's
  primal simplex and GLPK. A limit of 1 reports `optimal` after 0 iterations
  on Gurobi, CPLEX, Xpress and MOSEK, and Clp and HiGHS already do at 0; on
  GLPK it is inferred (only 0 and 5 were run; the limit check precedes
  pricing, `spxprim.c:1273`).

## Proposed shape

```cpp
auto basis = model.get_basis();         // model_basis_t<M>, by value
if(is_a<basis_status::nonbasic_at_bound>(basis.get_status(x))) { ... }
if(is<basis_status::basic>(basis.get_status(row))) { ... }

auto y = model.add_column(entries);     // column generation
// basis.get_status(y) reads nonbasic when y's id is past the snapshot; after
// a removal, y may reuse the removed id and read that entity's entry (N8)
basis.set_status(y, basis_status::nonbasic_at_upper_bound{});

other_model.set_basis(basis);           // any lp_basis source, see B7
other_model.solve();                    // starts from it

model_basis_t<M> slack;                 // default: every column nonbasic,
slack.set_status(x, basis_status::basic{});  // every row basic; grows on
fresh_model.set_basis(slack);           // writes, works on an unsolved model
```

- `get_basis()` returns a snapshot by value over two
  `detail::handle_status_table`s keyed by handle uid, like `iis_snapshot`,
  with `get_status(v)` and `get_status(c)` const and noexcept, and
  `set_status(e, tag)` for edits. Docs and tests spell its type
  `model_basis_t<M>` and its statuses `model_variable_basis_status_t<M>` /
  `model_constraint_basis_status_t<M>`.
- Each backend reports the most specific tag its solver's answer gives, over
  the `basis_status` hierarchy: `basic`; `nonbasic` > `nonbasic_at_bound` >
  {`nonbasic_at_lower_bound`, `nonbasic_at_upper_bound`, `nonbasic_fixed`};
  `nonbasic` > `superbasic` > `nonbasic_free`; bare `nonbasic` for an entity
  whose side the snapshot does not know. Rows report the side of their
  activity `a·x`. Generic code tests with `is_a`: `is_a<nonbasic_at_bound>`
  holds on every backend for an entity at a bound, fixed ones included, and
  `is_a<superbasic>` for one at no bound.
- `set_basis(b)` reads `b.get_status(e)` for every live entity of the target
  model, maps tags to native codes, writes once, and leaves the next `solve()`
  to start from it.
- Members on the `*_lp` classes only (ruling of 2026-07-22): `clp_lp`,
  `glpk_lp`, `highs_lp`, `gurobi_lp`, `cplex_lp`, `xpress_lp`, `copt_lp`,
  `mosek_lp` get both; `soplex_lp` gets `get_basis()` only (B9); `highs_qp`
  gets neither (B10).

## Decisions to take

None is ruled. B1 to B13 deviate from the draft or from an existing pattern,
or trade flexibility, ease of use or performance.

- **B1. The writer of `has_modifiable_lp_basis`.** (a) `set_status(e, tag)`
  only, for the five leaf tags and bare `nonbasic`; `set_basic` and
  `set_nonbasic(e, value)` are dropped. Bare `nonbasic` is written as HiGHS's
  code 4, whose side HiGHS takes from its own solution or the bound nearer
  zero, and as the native at-lower code elsewhere, which the solver snaps
  where no lower bound exists. (b) Keep `set_nonbasic(e, value)`, snapping
  with bounds copied into the snapshot by `get_basis()`: 16 more bytes per
  entity, stale after a bound change, unknown for an entity created later.
  (c) Keep it, store the value and snap at `set_basis` against the bounds the
  model has then: 8 more bytes per entity. Recommended: (a). No native setter
  takes a value, and every solver snaps a side that contradicts the bounds.
  On a boxed entity or a ranged row both sides are legal, so the written side
  is kept: under (a) the caller names it with a leaf tag, and a wrong side
  costs a bound flip at the solve (see the facts). (b) and (c) buy the choice
  of side from a value; under (a) the portable way is a free helper giving a
  status from a value and the model's bounds, deferred unless you want it in
  this change. `set_basic(e)` repeats `set_status(e, basic{})`. A basis meant
  to reach the optimum under an iteration limit of 0 must name the exact
  side of every boxed entity and ranged row. This changes a concept that
  shipped, unimplemented, in 1.0.0. Blocks the core package.
- **B2. What `get_basis()` reports.** *Revised on 2026-10-09 after the
  maintainer asked whether the tag hierarchy is not useful here: it is.*
  (a) The same tags on every backend for the same situation: rows on the side
  of their activity, and `nonbasic_fixed` for every nonbasic entity whose two
  bounds are equal, which costs up to three bulk bound reads per
  `get_basis()` on HiGHS, Gurobi, CPLEX and Xpress, and a comparison with
  cached bounds on SoPlex, whose slack basis reads a fixed column at lower.
  (b) The most specific tag the solver's answer gives, read through the
  hierarchy. Rows are still translated to the side of their activity, which
  is a matter of correctness, not of refinement: flipped on CPLEX's 'L' rows
  and on every Xpress row, derived from the sense on Gurobi (CPLEX and Gurobi
  read the senses, as their IIS decoders do). No bound is read. A fixed
  entity reads `nonbasic_fixed` where the solver has a fixed code (Clp, GLPK,
  COPT, MOSEK, SoPlex once a simplex ran) or the sense says `=` (Gurobi and
  CPLEX rows), and otherwise the side the solver names, by the sign of its
  reduced cost or dual (HiGHS, Xpress, Gurobi and CPLEX columns): both are
  true, and `is_a<nonbasic_at_bound>` holds for both. Each backend's variants
  list the tags it reports plus bare `nonbasic`. Writing needs one policy of
  its own, since `has_modifiable_lp_basis` requires `set_status(e, tag)` for
  every tag while `handle_status_table::set` is deleted for a tag the variant
  does not list: `basis_snapshot::set_status` stores the most specific tag
  the variant lists by derivation, most derived first (`nonbasic_fixed`
  written on HiGHS is stored as bare `nonbasic`). Nothing is lost for the
  solver, which snaps the side of a fixed entity anyway. Recommended: (b).
  It reads nothing beyond what the translation needs, follows the IIS rule
  that a variant lists what its path reports (N1), and lets generic code ask
  the question at the level it cares about. The one loss: whether
  `is<nonbasic_fixed>` holds for a fixed entity depends on the backend, so
  code that needs "fixed" compares the bounds.
- **B3. Free and superbasic.** *Revised on 2026-10-09 after the maintainer
  asked why not add a superbasic tag: added as the parent of
  `nonbasic_free`.* (a) Widen `nonbasic_free` to "nonbasic at no bound". (b)
  `basis_status::superbasic : nonbasic`, "nonbasic at no bound: strictly
  inside its bounds, or free", and `nonbasic_free` re-based on it,
  `nonbasic_free : superbasic`, "nonbasic with no finite bound". A backend
  then reports as specifically as its code allows, with no bound read:
  `nonbasic_free` where the code means free only (GLPK NF, SoPlex 3, HiGHS
  Z3, Clp 0), `superbasic` where one code covers both (Gurobi -3, CPLEX 3,
  Xpress 3, COPT 3, MOSEK SUPBAS, Clp 4); generic code tests
  `is_a<superbasic>` for "at no bound". Recommended: (b). The draft's
  sibling reading, a superbasic tag beside `nonbasic_free`, is what needed
  the bounds to split the shared codes; as a parent it needs none. Re-basing
  `nonbasic_free` changes a tag that shipped, unimplemented, in 1.0.0, like
  B1; `is_a<nonbasic>` still holds for it. That `superbasic` includes the
  free case follows MOI's `SUPER_BASIC`, not MINOS's or Clp's narrower
  term; one sentence of the docs defines it. Superbasics strictly between
  bounds were observed only after a stop (Xpress after an iteration or time
  limit, MOSEK after a dual-simplex iteration limit and in an unbounded
  certificate), never at an optimum.
- **B4. Entities the snapshot does not cover.** (a) The column variant lists
  bare `nonbasic` as its alternative 0 and the row variant lists `basic`, so
  a handle whose id is past the snapshot's bound reads nonbasic (a column) or
  basic (a row), and `set_basis` writes it so; the basis stays square after
  additions alone. Each backend's column variant lists bare `nonbasic`
  first and its row variant `basic` first, then the tags B2 says it reports.
  `set_status` past the table's bound grows it (the IIS table keeps
  throwing), and `basis_snapshot` is publicly default-constructible: an empty
  snapshot is the slack basis, the way to warm-start a model never solved.
  A handle that reuses the id of an entity removed since the snapshot reads
  that entity's entry (N8 a, documented, not guarded). (b) As (a) with
  `nonbasic_at_lower_bound` as the column default, the Coin-OR and MPS
  convention, which reads wrong for a free or upper-bounded column. (c) Exact
  cover, as OR-tools MathOpt: `set_basis` throws unless the basis names every
  entity of the model. Recommended: (a), the rule every solver applies to its
  own basis. The snapshot template requires these two alternatives 0, as
  `iis_snapshot` requires `absent`, and so departs from its single
  alternative 0 for both kinds. Column alternative 0 is also what
  `colgen::variable_status<model_variable_basis_status_t<M>>` holds for a
  master column until its first `master_refreshed`
  (`column_generation.hpp:70-75, 194-203`): predicates that classify fresh
  columns test `!is<basis_status::basic>`, and the docs say so.
- **B5. What `get_basis()` promises.** (a) Right after a `solve()` that ran a
  simplex, before any other call that changes the model or the solver's
  state, `get_basis()` returns the basis that solve ended with, or throws
  `solver_error` where the solver says it holds none: after presolve proves
  infeasibility (Gurobi, CPLEX, COPT, Xpress) or unboundedness (Gurobi,
  CPLEX, Xpress), after a non-optimal end of a presolve-reduced LP on HiGHS,
  after a time limit before the simplex starts (Gurobi, CPLEX, MOSEK, Xpress
  47.01). Everywhere else its result is unspecified, one line per window:
  - before any solve: GLPK, SoPlex and COPT with rows return a slack basis,
    the others throw (Clp's missing status array is the one wrapper-side
    guard: its getters would segfault);
  - after a change of the model: Gurobi, HiGHS and Xpress throw although
    HiGHS and Xpress keep the basis and warm-start from it; the others return
    the basis they hold, which may carry codes the next solve repairs (Clp at
    lower on a new free column, MOSEK `FIX` on an appended column, SoPlex
    `nonbasic_free` on a row whose active side was freed, after which its
    next `solve()` reloads the LP without any basis, `soplex_lp.hpp:515`);
  - after `set_basis()`: HiGHS throws on a model never solved, Xpress was not
    measured, the others return the basis as the solver stored it;
  - after a `solve()` the wrapper answers without a simplex, after
    `refine_lp_status()`, `compute_iis()` or `compute_iis_by_deletion`.

  GLPK and SoPlex always hold a basis and never throw for want of one; on
  SoPlex it may be a slack basis no simplex reached. `soplex_lp::get_basis()`
  throws `feature_unavailable_error` on a library without the getters
  (6.0.4, which loads since the C API reports no version). Two
  sub-decisions: (i) `copt_lp::compute_iis()` calls `COPT_Reset`
  (`copt_base.hpp:539`), after which `COPT_GetBasis` succeeds with all-zero
  arrays, every entity at lower and none basic: `copt_lp::get_basis()`
  counts the basic entries while decoding and throws unless the count equals
  the number of rows, at no extra cost (recommended), or that read is
  documented as unspecified; (ii) on a model without columns, `get_basis()`
  returns, without a native call, the snapshot whose rows all read basic,
  the only square basis, the same on every backend (recommended), or it is
  unspecified. On `clp_lp` whose every column was removed, that empty
  snapshot agrees with what `set_basis` writes for the dead columns (5). (b) As (a), made uniform by wrapper bookkeeping: every mutator
  of every backend clears a "solved" flag that `get_basis()` checks. (c) A
  snapshot that carries an availability flag instead of throwing.
  Recommended: (a), with (i) and (ii). Its signals are the native ones:
  Gurobi 10005, CPLEX 1262, HiGHS `basis_validity`, COPT 3, MOSEK's slot and
  its `UNK` and `INF` keys, Clp's null array, and Xpress `PRESOLVESTATE` bit
  7, chosen over `XPRSgetbasis`'s return code because it is the only Xpress
  signal that excludes the slack basis before a solve and the stale one
  after a presolve-proved re-solve; its cost is HiGHS's, no basis reported
  after a change although the held one still warm-starts.
- **B6. The status after `set_basis`.** (a) `set_basis()` resets the
  reported status to `unknown` on every backend. (b) Only where the native
  write drops the solution: `CPXcopybase`, `COPT_SetBasis` and the
  `GRBreset` that Gurobi needs. (c) Leave the status alone, as every mutator
  does: `get_status()` "keeps reporting the last solve until the next
  `solve()`, even after the model has changed"
  (`docs/solving/status-and-limits.md:98`). Recommended: (c), the documented
  rule. N46 (c) resets the status for calls that run the solver
  (`iis.md:2333-2334`), and `set_basis` runs none. Under (c), on CPLEX, COPT
  and Gurobi the solution getters throw between `set_basis` and the next
  solve while the status still reports the last one, as Gurobi's already do
  after any change (VBasis and X read 10005 while `get_status()` keeps the
  last solve). Under (a) or (b), `set_basis` would be the first call that
  runs no solver yet resets the status, and that page would list it.
- **B7. What `set_basis` accepts.** (a) Any `lp_basis<B, M>`: the model's own
  snapshot, another model's, or a user type with `get_status(v)` and
  `get_status(c)` returning a variant that lists a `basic` tag and a
  `nonbasic` tag. `has_lp_basis_warm_start` checks it, with
  `archetype::lp_basis<T>` (two `get_status` returning
  `std::variant<basis_status::nonbasic, basis_status::basic>`) beside the
  existing archetypes. A tag maps to a native code by derivation, most
  derived first, as `iis_status::sides_of` does: a user refinement writes as
  the tag it refines, `nonbasic_at_bound` writes as bare `nonbasic`, and an
  alternative derived from neither `basic` nor `nonbasic` writes as the
  kind's alternative 0. `set_basis` reads one status per live entity of the
  target, so entries of removed ids are never read. A transfer is meaningful
  when the target's handle ids are the source's: a target built in the same
  creation order from a source that removed nothing, or that replays the
  same removals (only `clp_lp`, `highs_*`, `gurobi_*` and `cplex_*` can);
  otherwise the user passes an `lp_basis` adapter mapping one to the other.
  (b) Only `model_basis_t<M>`. Recommended: (a). Models are move-only, so
  seeding a second model is the transfer case, and the draft docs already
  promise "a basis view built from lambdas"
  (`docs/reference/concepts.md:128`).
- **B8. MOSEK.** (a) `set_basis()` makes the next `solve()` run a simplex
  optimizer and then restores the optimizer setting; later solves run the
  interior point again and start cold. The armed flag is a member of its
  own: `set_basis()` sets it, `solve()` clears it before the native call,
  `reset_status()` never touches it, and the switch sits under
  `detail::restore_guard`, as `refine_lp_status()` does on Gurobi and CPLEX.
  (b) `mosek_lp` uses a simplex optimizer for every solve, so in-place
  re-solves warm-start too, and large cold LPs may get slower. (c)
  `mosek_lp` gets `get_basis()` only, and the user picks a simplex
  optimizer natively. Recommended: (a). The default `MSK_OPTIMIZER_FREE`
  runs the interior point, which ignores the keys (6 iterations with or
  without them; documented: "cannot perform warm-start"). Which simplex is a
  sub-choice: `MSK_OPTIMIZER_DUAL_SIMPLEX`, which the manual calls faster on
  average and which ended optimal after 0 iterations from a written optimal
  basis with presolve off, free or on; or `MSK_OPTIMIZER_FREE_SIMPLEX`,
  which "tries to choose the best optimizer for the given problem and the
  available solution" and picked the primal from a written basis. Every
  measured warm start after a change was dual-feasible (a relaxed bound, a
  tightened row, an unattractive new column); none covers an attractive new
  column, primal- but not dual-feasible, the column-generation case.
  Recommended: the free simplex, confirmed by a probe of that case during
  implementation, the dual otherwise.

  *Answer of 2026-10-09 to the maintainer's question on an algorithm
  selector.* Default LP algorithms, from the survey: only `mosek_lp` runs an
  interior point (MOSEK's automatic choice, with basis identification).
  `gurobi_lp` and `xpress_lp` run the solver's concurrent default, which may
  finish with barrier and crossover; both still end with a basis and honour
  a written one (measured). `highs_lp`, `soplex_lp` and `copt_lp` run the
  dual simplex. Three wrappers hard-code the primal simplex: `clp_lp`
  (`Clp_primal`), `glpk_lp` (`meth = GLP_PRIMAL`) and `cplex_lp`
  (`CPXprimopt`, where `CPXlpopt` would let CPLEX choose, usually the dual).
  An `lp_algorithm` selector, a variant over a tag hierarchy (`simplex` >
  {`primal_simplex`, `dual_simplex`}, `interior_point` refined by whether a
  crossover runs, which decides whether `get_basis()` can answer, and a tag
  for the solver's own choice), would turn those baked-in choices into the
  user's. It is its own change, after the basis: it touches `solve()` on
  nine LP classes, needs its own survey (each backend's knobs, which tags it
  supports, `glp_interior`'s own status codes, HiGHS's PDLP), and raises a
  question of its own, whether `*_milp` classes get it for the root
  relaxation given the ruling of 2026-07-22. The two compose: (a) is defined
  as "when the optimizer is MOSEK's automatic choice, the solve after
  `set_basis()` runs a simplex; an explicit choice is respected". Today no
  explicit choice exists, so it is the plain one-shot, and nothing changes
  when the selector lands.
- **B9. SoPlex.** (a) Read-only: `get_basis()`, no `set_basis()`. (b) Write
  through the exported C++ symbols (`SoPlexBase<double>::readBasisFile` with
  `NameSet`s built by the caller): Itanium ABI only, mangling that changes
  between 7.x and 8.x, a size of `NameSet` unknown without the vendor
  header, and still a temporary file. (c) Write through
  `SoPlex_readBasisFile`: its reader builds cumulative names (`x0x1...xj`),
  so lines over 255 characters limit it to about 80 entities, its cost is
  superquadratic (12 s at 8000x8000 for an empty file), and an unopenable
  path aborts the process. Recommended: (a). Asking upstream for
  `SoPlex_setBasis` and `SoPlex_getBasis` in the C interface would be an
  outward step, yours to take.
- **B10. `highs_qp`.** (a) No basis members on `highs_qp`. (b) Both members,
  documented: `get_basis()` throws after a quadratic solve, and `set_basis()`
  has no effect on the QP solver. Recommended: (a). With a Hessian, HiGHS's
  QP solver leaves `basis_validity` 0 with active-set codes and ignores
  `Highs_setBasis` (measured on 1.10.0 and 1.15.1). A QP hot start (a
  solution and a basis) is a separate feature: 1.10.0 hot-starts with no
  option, but from 1.15.0 it needs `qp_allow_hot_start`, false by default,
  so an in-place re-solve on 1.15.1 starts cold. The concept is per class,
  and a class cannot know whether a Hessian will be set.
- **B11. An invalid basis on GLPK.** (a) Never an error, as elsewhere.
  GLPK's `set_basis` skips its write when a column with no matrix entry is
  marked basic, then calls `glp_factorize` after writing; on either failure
  it installs GLPK's standard basis, so the next solve starts cold, as
  Gurobi and COPT discard such a basis and the others repair it. (b) The
  same checks, but `solver_error` after installing the standard basis. (c)
  Guard the abort only, and document any other invalid basis as undefined
  behaviour: every later solve then reports `failed` (wrong count) or
  `numerical_failure` (singular) until a valid basis is set, and
  `get_basis()` returns the bad one. Recommended: (a). The two checks have
  separate reasons. The empty-column check is a guard under the misuse
  ruling, justified by the process abort; it costs one `glp_get_mat_col`
  count per basic column. The factorization fallback makes `glpk_lp` follow
  the rule the other backends apply natively, and costs nothing on success:
  the factorization is the one the next simplex would compute and reuses
  (`spxprob.c:409-412`). Under (c) a wrong count is undefined behaviour
  whose consequence, a model that fails until repaired, is yours to weigh.
- **B12. Proof of a warm start, and iteration limits.** (a) The shared
  warm-start suite proves that a basis is used only on models with
  `has_iteration_limit` (HiGHS, Gurobi, CPLEX), with a limit of 1 on a dense
  LP presolve cannot reduce (Gurobi's presolve solves small LPs in 0
  iterations, and a given basis turns presolve off). Elsewhere it checks the
  answer and the basis read back after the solve, which a `set_basis` that
  writes nothing would also pass. Iteration limits for `clp_lp`, `glpk_lp`,
  `xpress_lp` and `mosek_lp` stay out of this change (COPT has none). (b) Add
  `set_iteration_limit` to those four classes here, with their
  `IterationLimitTest`; COPT stays unproven. (c) As (a), plus one backend
  test per backend outside the shared proof: a cold solve of a dense LP with
  a nonzero native simplex count, then an identical fresh model given its
  basis, solved with a native count of 0 (Xpress `SIMPLEXITER`, COPT
  `SimplexIter`, Clp `Clp_numberIterations`, GLPK `glp_get_it_cnt`, MOSEK
  `MSK_getintinf` with both simplex and interior-point counts at 0, which
  also proves B8's switch); and echo tests that write a basis into a fresh
  model and read it back before any solve where the native read returns what
  was written (COPT, MOSEK, GLPK for tags consistent with the bounds), which
  pin the write mapping. Recommended: (a) and (c). The shared read suite pins
  the read mapping, row flips included, on a known nondegenerate LP; the
  echo pins the write mapping; the counter pins that the basis is used.
- **B13. A basis short after a removal.** Removing a column that was basic
  at the snapshot leaves the snapshot one basic short over the live entities,
  unless a later column reuses the id; Gurobi then discards it and starts
  cold, and its own re-solve after such a removal is cold too. (a) Document:
  after removing a basic column, `set_basis` cannot keep a Gurobi warm
  start; evict nonbasic columns. The shipped eviction predicates already do
  at an optimum, since a basic column never has a reduced cost above their
  nonnegative threshold (`column_generation.hpp:345-353, 478-488`). (b)
  `set_basis` completes a short count itself, marking nonbasic rows basic,
  as HiGHS does natively, on every backend. (c) On Gurobi only, an update
  and a read-back after the write, so that a discarded basis throws.
  Recommended: (a), under the misuse habit; (c) contradicts B11 (a)'s "never
  an error".

## Decided without a ruling

These follow existing patterns; say so if one should be a question.

- **Concepts.** `lp_basis<B, T>` checks a `const B &`, as `lp_iis` does;
  `model_variable_basis_status_t` and `model_constraint_basis_status_t` call
  `get_status` on `declval<const model_basis_t<T> &>()`, so a deducing-this
  `get_basis()` cannot silently unform them; `has_lp_basis_warm_start`
  checks `set_basis` on a `const model_basis_t<T> &` and on the archetype
  (B7). `has_lp_basis` stays a single-parameter whole-model concept: the
  handle rule of 2026-09-16 lists the `<T, M = T>` concepts by name and kept
  the others single-parameter (`docs/reference/concepts.md:66-72`), no
  backend has a handle that could carry a basis, and a defaulted `M` can be
  added later without breaking single-argument uses.
- **`has_modifiable_lp_basis`** is a property of `basis_snapshot`: it holds
  wherever `has_lp_basis` holds, `soplex_lp` included. It gets no suite of
  its own: `LpBasisTest` asserts it beside `has_lp_basis`, as `IisTest`
  asserts `has_iis`, `test/basis_snapshot.cpp` covers `set_status`
  solver-free, and `LpBasisWarmStartTest` writes through it. The feature
  tables show no row for it: their labels are "LP basis" (`LpBasisTest`,
  `soplex_lp` included) and "LP basis warm start"
  (`LpBasisWarmStartTest`), so no row suggests that SoPlex can write.
- **Names and files.** `basis_snapshot` in `utility/basis_snapshot.hpp`,
  namespace `mippp`, beside `iis_snapshot`; the `set_status` storage policy
  and the tag-to-native-kind classifier in its `detail` section. Suites
  `LpBasisTest` and `LpBasisWarmStartTest` in
  `test/test_suites/lp_basis.hpp`; solver-free unit tests in
  `test/basis_snapshot.cpp`. The shared suites read a basis only after a
  `solve()` on a model with at least one column. Ranged rows sit under
  `if constexpr` on `has_modifiable_constraint_bounds` or
  `has_ranged_constraints`, so `gurobi_lp` runs without them. Under B2 (b)
  and B3 (b), a fixed entity is asserted with `is_a<nonbasic_at_bound>`, a
  free one with `is_a<superbasic>`, every other entity with its exact tag. A
  removal case under `has_remove_variable` checks the optimum only, never an
  iteration count.
- **Native symbols.** Required where every supported release exports them (a
  required symbol refuses the whole release, D1; for these backends the
  supported range is the validated range today): Clp `Clp_statusArray`,
  `Clp_copyinStatus`; GLPK `glp_factorize`, `glp_get_mat_col`,
  `glp_std_basis` (documented across [4.59, 5.1), 5.0 measured); CPLEX
  `CPXgetbase`, `CPXcopybase`; Xpress `XPRSgetbasis`, `XPRSloadbasis`; MOSEK
  `MSK_putskc`, `MSK_putskx`. For the counter tests of B12 (c): Clp
  `Clp_numberIterations`, GLPK `glp_get_it_cnt`, MOSEK `MSK_getintinf`, also
  present in every supported release. Optional, with
  `feature_unavailable_error` when absent: `SoPlex_basisColStatus` (absent
  from 6.0.4, which loads anyway; every SoPlex 7.0 addition is optional,
  `soplex_api.hpp:109-120`) and `GRBreset`, optional only until Gurobi 10.x
  is checked (see the implementation plan). HiGHS and COPT need nothing new
  but constants.
- **Native traps avoided.**
  - Clp: removed columns stay in Clp, so the status buffer spans
    `getNumCols() + getNumRows()` bytes, columns first, and the `get_basis()`
    column table is sized by `getNumCols`. `set_basis` gives every byte a
    code: live ids the mapped tag, every id in `_free_variable_ids` 5, never
    the current byte, which keeps a removed basic column basic. Every byte
    read is masked with `& 7` (the bits above may carry internal flags).
    Clp's per-entity setters are never used: they segfault without a status
    array and overwrite the primal solution.
  - Xpress never gets a null array on a nonzero dimension (segfault);
    `CPXcopybase` and `COPT_SetBasis` never get a null array (error 1004
    measured even for a zero length; COPT's error 3 measured on a nonzero
    dimension), so both get a one-element dummy when a dimension is empty.
  - Gurobi: update, then `GRBreset(model, 0)`, then write: a write on a
    queued model is lost, a write on a solved one is ignored or a no-op
    depending on the release.
  - MOSEK: read the `MSK_SOL_BAS` slot, never the one `solve()` picked.
- **Docs.** A page "Bases and warm starts" under Solving, with compiled
  snippets in `test/doc_snippets/basis.cpp`, including column generation on
  an `*_lp` model; the concept rows, the README roadmap row,
  `docs/solvers/index.md` (per-backend notes: SoPlex read-only, MOSEK's
  one-shot simplex, GLPK's fallback, Gurobi after removals, CPLEX free rows,
  the unspecified windows), `docs/getting-started/index.md`,
  `docs/solving/solutions.md`, `docs/solving/updates.md` and
  `docs/algorithms/column-generation.md` (the eviction flow: take the basis
  after `solve()` and before `manage_columns`, write it after, and B13)
  rewritten; the basis page, the concept rows and `status-and-limits.md`
  say that `get_status()` reports the last solve, `set_basis()` included
  (B6 c); feature tables
  regenerated; `column_manager.hpp`'s commented example checked against the
  real types, and `test/column_manager.cpp`'s `fake_basis_status` given bare
  `nonbasic` as alternative 0. `paper/paper.md` lists basis warm starts as
  roadmap: the edit is yours, since the JOSS submission is pending.
- **Design record.** Rulings recorded in `misc/design/basis/basis.md`, in the
  shape of `iis.md`.

## Implementation plan

1. **Core** (branch from `origin/main`): concepts and archetype,
   `basis_snapshot`, its storage policy and the classifier,
   `test/basis_snapshot.cpp`, the two shared suites, instantiated first on
   `highs_lp` and `clp_lp`.
2. **Backends**, one branch each, in parallel worktrees: native symbols,
   `get_basis()`, `set_basis()`, instantiations, capability pins
   (`!has_lp_basis` on every `*_milp` and `highs_qp`;
   `has_modifiable_lp_basis` and `!has_lp_basis_warm_start` on `soplex_lp`),
   and backend tests for what the shared suites cannot see: B12 (c)'s counter
   and echo tests; Clp before its first solve, and after removing a basic
   column (the dead id written 5, checked through the native status array);
   GLPK's fallback without an abort; COPT after `compute_iis()`; MOSEK's
   optimizer read back after the armed solve and the next solve running the
   interior point; Gurobi's write on a solved model; Xpress's bit 7 after a
   bare `XPRSloadbasis`.
3. **Gurobi 10, before merging.** Confirm `GRBreset` and the update, reset,
   write recipe on Gurobi 10.x through the compatibility matrix, which
   fetches gurobipy 10.0.x (`docs/solvers/compatibility.md:134`; it needs
   your license). If 10.x has it, `GRBreset` becomes required. If not, Gurobi
   10 cannot stay validated (a validated release never lacks a member,
   `CONTRIBUTING.md:452-453`): you choose between narrowing
   `validated_versions` to [11, 14) with `supported_versions` [10, 14), and
   another write path on 10.
4. **Integration**: docs, feature tables, design record.
5. **Matrix**: gcc15 every source, gcc14 and clang18 on the CI subset,
   sanitizers, the other local releases (HiGHS 1.10 and 1.15.1, Gurobi 11,
   12 and 13, CPLEX 22.1.1 and 22.1.2, Xpress 45.01 and 47.01, MOSEK 11.1 and
   11.2, cached Clp releases), the compatibility table re-rendered since the
   new suites change every row's counts; format, includes, snippets,
   zensical.
6. **Review** by several independent reviewers, findings verified, fixes.

Nothing is pushed: branches are left for you to publish.

## Deferred

- A free helper giving the status of an entity from a primal value and the
  model's bounds (B1).
- The deletion filter saving and restoring the caller's basis through
  `has_lp_basis_warm_start` (its trials leave the solver's basis where they
  ended).
- Basis support on `dumb_lp`, the reference model of the shared suites,
  deferred as IIS deferred it (N24, "CI always has Clp, Cbc or HiGHS"): it
  needs new `dumb_lp` members (statuses read before `deleteModel`, replayed
  before the next `primal`). The suites are proved on `highs_lp` and
  `clp_lp`, both in CI. This departs from the conformance workflow's
  "`dumb.cpp` first" step; say so if it should not.
- Iteration limits on `clp_lp`, `glpk_lp`, `xpress_lp`, `mosek_lp` (B12 b).
- An upstream request for a SoPlex C setter (B9).
