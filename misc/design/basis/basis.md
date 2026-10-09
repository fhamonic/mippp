# LP basis support: design decisions

Design note for LP basis support (roadmap item "LP basis warm-starts"). It
records the maintainer's rulings on the questions of
[basis_plan.md](basis_plan.md), whose entries keep the options and evidence
behind each ruling letter, and the survey of 2026-10-09 in
[survey/](survey/). It is not user documentation.

## Rulings of 2026-10-09

- **B1 (a). The writer.** `has_modifiable_lp_basis` requires
  `set_status(e, tag)` only, for the leaf tags and bare `nonbasic`;
  `set_basic` and `set_nonbasic(e, value)` are dropped from the draft. A free
  helper giving a status from a value and the model's bounds stays deferred.
- **B4 (a), documented. Entities the snapshot does not cover.** A column whose
  id is past the snapshot's bound reads bare `nonbasic` (alternative 0 of
  every column variant), a row reads `basic` (alternative 0 of every row
  variant), and `set_basis` writes them so. `set_status` past the bound
  grows the table; `basis_snapshot` is publicly default-constructible, an
  empty snapshot being the slack basis. A handle that reuses a removed id
  reads the removed entity's entry (N8 a). The maintainer asked for this to
  be documented: the docs state the defaults, the slack basis, the recycled
  ids and what `colgen::variable_status` holds before the first refresh.
- **B5 (a) with (i) and (ii). What `get_basis()` promises.** Right after a
  `solve()` that ran a simplex, before any other call that changes the model
  or the solver's state, it returns the basis that solve ended with, or
  throws `solver_error` where the solver says it holds none. Elsewhere its
  result is unspecified and documented per backend. (i) `copt_lp` counts the
  basic entries while decoding and throws unless they equal the rows (the
  zero arrays after `COPT_Reset`). (ii) A model without columns gets, with
  no native call, the snapshot whose rows all read basic.
- **B6 (c). The status after `set_basis`.** Left alone. The maintainer's
  reason: if `set_basis()` reset the status, so should every other setter,
  which is painful; better to document that `get_status()` refers to the last
  solve. The basis page, the concept rows and `status-and-limits.md` say so.
- **B7 (a). What `set_basis` accepts.** Any `lp_basis<B, M>`, checked by
  `has_lp_basis_warm_start` through an archetype; tags map to native codes by
  derivation, most derived first; an alternative derived from neither
  `basic` nor `nonbasic` writes as the kind's alternative 0. A transfer
  between models is meaningful when the handle ids match.
- **B9 (a). SoPlex.** Read-only: `get_basis()`, no `set_basis()`.
- **B10 (a). `highs_qp`.** No basis members.
- **B11 (a). An invalid basis on GLPK.** Never an error: the empty basic
  column guard prevents the abort, and a basis GLPK cannot factorize is
  replaced by GLPK's standard basis, so the next solve starts cold.
- **B12 (a) and (c). Proof of a warm start.** The shared warm-start suite
  proves the basis is used at an iteration limit of 1 where
  `has_iteration_limit` holds (HiGHS, Gurobi, CPLEX); backend tests prove it
  elsewhere by the native simplex count, and echo tests pin the write
  mapping. Iteration limits on other classes stay out of this change.
- **B13 (a). A basis short after a removal.** Documented: after removing a
  column that was basic, Gurobi discards a written basis and starts cold;
  evict nonbasic columns, as the shipped eviction predicates do at an
  optimum.

## Rulings of 2026-10-09, after the maintainer's questions

- **B2 (b). What `get_basis()` reports.** The maintainer asked whether the
  variant over a tag hierarchy is not useful here, then ruled the revised
  option. Each backend reports the most specific tag its solver's answer
  gives. Rows are translated to the side of their activity: flipped on
  CPLEX's 'L' rows and on every Xpress row, derived from the sense on Gurobi.
  No bound is read. A fixed entity reads `nonbasic_fixed` where the solver
  has a fixed code or the sense says `=`, and otherwise the side the solver
  names; `is_a<nonbasic_at_bound>` holds for both. Each backend's variants
  list the tags it reports plus bare `nonbasic`; `set_status` stores the most
  specific tag the variant lists, by derivation, most derived first.
- **B3 (b). Free and superbasic.** The maintainer asked why not add a
  superbasic tag, then ruled: "the shipped basis interface in 1.0.0 was only
  a draft and already modified in the current rulings". The hierarchy
  becomes `superbasic : nonbasic` ("nonbasic at no bound: strictly inside
  its bounds, or free") and `nonbasic_free : superbasic` ("no finite
  bound"). A backend reports `nonbasic_free` where its code means free only
  and `superbasic` where one code covers both; generic code tests
  `is_a<superbasic>`.
- **B8 (a). MOSEK.** `set_basis()` makes the next `solve()` run a simplex
  optimizer, then restores the setting, defined as "when the optimizer is
  MOSEK's automatic choice, the solve after `set_basis()` runs a simplex; an
  explicit choice is respected". The maintainer's question on an algorithm
  selector is answered by a note for a later change,
  [lp_algorithm.md](../lp_algorithm/lp_algorithm.md). Free simplex against
  dual simplex is settled by a probe of the column-generation case.

## Settled during the implementation

- **MOSEK's one-shot runs `MSK_OPTIMIZER_FREE_SIMPLEX`** (B8). Measured on
  MOSEK 11.0.14 over 600 column-generation cases (1 to 10 attractive columns
  added to the dense test LP, 3 to 50 rows each, 10 seeds per shape): 10422
  simplex iterations in all against 13577 for the dual simplex, fewer in 431
  cases and more in 109. An optimal basis written into an identical model
  takes 0 iterations with either.
- **`GRBreset` is required** (implementation plan, step 3). The
  `libgurobi100.so` of the gurobipy 10.0.3 wheel exports it, and the basis
  suites pass on 10.0.3, 11.0.3, 12.0.1 and 13.0.2 alike.
- **COPT's "no basis"** answers with the message of HiGHS and Clp: return
  code 3 and a count of basic entries other than the rows both mean no basis.
- **`is_a<superbasic>` (B3) is a column-level test.** The row statuses of
  `gurobi_lp` and `cplex_lp` list no superbasic tag, since their rows never
  read one (B2 b), so `is_a<superbasic>` does not compile on them: generic
  code over rows tests by derivation, as the docs' `describe` does (pinned
  in test/solvers/gurobi.cpp and cplex.cpp).
- **The explicit choice of B8 (a)** is today an `MSK_IPAR_OPTIMIZER` set
  through `native_api()` to anything but `MSK_OPTIMIZER_FREE`: the one-shot
  switches from `MSK_OPTIMIZER_FREE` only, and restores it.
- **Xpress after a bare `XPRSloadbasis`** (measured on 45.01 and 47.01):
  PRESOLVESTATE bit 7 stays clear on a model never solved, so `get_basis()`
  throws there, and stays set on a solved one, whose `get_basis()` then
  returns the basis just written. B5 leaves that window unspecified; a
  backend test pins the measurement.
- **`clp_lp`'s `set_basis()`** reads live entities only, as every backend
  does: a removed column, which stays in Clp fixed at [0, 0], is written
  fixed without asking the source for its dead handle.
- **`set_basis()` writes what the backend's own snapshot would hold**
  (the maintainer's question of 2026-10-10, removing `detail::basis_kind`).
  A source's status is first converted to the backend's variant as
  `set_status` stores it (B2 b), an alternative that is no basis tag reading
  as alternative 0 (B7), then each listed tag has its native code. A tag the
  variant does not list loses its refinement, as in a snapshot: on
  `highs_lp`, `nonbasic_fixed` and `superbasic` now write `Nonbasic` instead
  of `Lower` and `Zero`, and on `glpk_lp`, `superbasic` writes `GLP_NL`
  instead of `GLP_NF`. GLPK's `glp_set_col_stat` replaces either by the code
  the column's type implies, so the two codes store alike there, and HiGHS
  keeps `Nonbasic` and places the column itself at the solve.
  test/solvers/highs.cpp echoes the code and the warm start; glpk.cpp pins
  what GLPK stores.

## Open questions

None remains on the design. Left to the maintainer outright: an upstream request
for a SoPlex C setter and the `paper/paper.md` edit.
