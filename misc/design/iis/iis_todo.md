# IIS implementation order

Companion to [iis.md](iis.md), which holds the design and the evidence
behind it. The steps are ordered by dependency, and each one can land and be
tested on its own. Question numbers refer to the open questions in iis.md.

## 0. Before any code

- [ ] Rule on open questions 1 to 6. Questions 1 to 3 shape the API and the
  capability work, so nothing from step 2 on starts without them.
- [ ] Answer the pull request with the direction of iis.md, so that the
  author can take part in the steps below instead of reworking the branch.

## 1. MOSEK fixes from the pull request

Independent of the IIS work, so it can land first.

- [ ] One optimization per `solve()`: main calls `MSK_optimize` and then
  `MSK_optimizetrm`, which optimizes again.
- [ ] A destructor that cannot throw, through the pull request's handle
  guard, the `MSK_optimizetrm` signature fix, and solution-slot selection.
- [ ] The MOSEK time limit, with `TimeLimitTest` instantiated in
  `test/solvers/mosek.cpp`.
- [ ] A regression test counting optimizer runs per solve, as the pull
  request's `MosekCertificate` tests do.

Done when the MOSEK suites pass locally.

## 2. Generic API, no backend yet

- [ ] The `iis_status` tags, next to `basis_status` in `model_concepts.hpp`.
- [ ] The snapshot type: `get_status(v)`, `get_status(c)`, member counts, and
  the completion status with its stop reason.
- [ ] `has_iis<T>`, with an archetype and `static_assert`s like the other
  concepts.
- [ ] The shared suite `test/test_suites/iis.hpp`, with the independent
  oracle and the cases listed in iis.md, registered in
  `test/test_suites/all.hpp`. It first runs at step 4.

Done when the new headers pass the header self-containment check and
`make check`.

## 3. Row relaxation on Clp

Assumes question 3 is answered with row bounds.

- [ ] `has_modifiable_constraint_bounds` in `model_concepts.hpp`, with a
  shared suite that includes a test that `infinity()` frees a row side,
  mirroring `infinity_removes_a_bound`.
- [ ] `clp_lp` implements it by writing `Clp_rowLower` and `Clp_rowUpper`,
  as `set_constraint_rhs` already does; the suite is instantiated for Clp.

Done when the suite passes on Clp in CI.

## 4. The fallback on Clp, the first end-to-end slice

- [ ] The deletion filter, ported from the pull request as the engine.
- [ ] The trial classification under the zero objective, as specified in
  iis.md.
- [ ] The RAII guard saving by value: variable bounds, row sides, objective
  coefficients and offset, the quadratic objective on a `qp_model`, and the
  time limit.
- [ ] The crossing-bounds check before any solve, the solve budget and the
  stop token (question 5).
- [ ] The snapshot built in handle space, and the model's status after the
  run (question 4).
- [ ] `clp_lp::compute_iis()` through the fallback (question 1), with the IIS
  suite instantiated for Clp.

Done when the IIS suite passes on Clp in CI, including the case checking
that the model solves to its previous result afterwards.

## 5. Widen the fallback

CI-tested backends come first. Each item adds the row-bound capability and
any other one it lacks, instantiates the capability suites it newly
satisfies, then the IIS suite.

- [ ] `cbc_milp`, through `Cbc_setRowLower` and `Cbc_setRowUpper`, which
  `cbc_api` already binds. It is the first MIP on the fallback, so it runs
  the integer-only cases; a column-less model is expected to end
  undetermined.
- [ ] `glpk_lp` and `glpk_milp`: readable and modifiable row bounds through
  `glp_set_row_bnds`, which switches the bound type rather than storing an
  infinite value.
- [ ] `highs_milp`, through `Highs_changeRowBounds`. It carries the
  removed-variable case on a remapping backend.
- [ ] `mosek_lp` and `mosek_milp`, after step 1, through `MSK_getconbound`
  and `MSK_putconbound`.
- [ ] `scip_milp`, through `SCIPchgLhsLinear` and `SCIPchgRhsLinear`, which
  `scip_api` must bind first. Every trial is a cold solve, so keep its cases
  small.
- [ ] `soplex_lp` last, if at all: SoPlex 6.0 cannot change a variable's
  lower bound, and no release can read a row side back, so this needs
  release-dependent symbols and row bounds kept by the wrapper.

Done when the IIS suite passes on each backend, in CI for the first three.

## 6. Native routines

Each wrapper runs the same IIS suite. Where a backend also meets the
fallback's requirements, the suite checks that both paths return valid IISs,
not necessarily the same one.

- [ ] Confirm the entry points at the start of each validated range, since
  only the probed releases were checked: `COPT_ComputeIIS` in COPT 7.2,
  `XPRS_IISOPS` in Xpress 45.1, and the `IIS*Force` attributes in Gurobi 10.
- [ ] Gurobi: `GRBcomputeIIS`; special constraints forced into the IIS;
  inequality rows get their side from the sense, equality rows are
  `member`; `IISMinimal` for the completion status.
- [ ] CPLEX: `CPXrefineconflictext` with one group per row and per bound,
  never the deprecated `CPXrefineconflict`; abort statuses and "possible"
  flags mean not minimal; conflict status 30 means feasible; sides as for
  Gurobi.
- [ ] Xpress: the `IISOPS` integrality and special-constraint bits set before
  `XPRSiisfirst`; `I` entries dropped; `L`, `G` and `E` mapped to tags;
  `IISSOLSTATUS` for the completion status.
- [ ] COPT: `COPT_ComputeIIS`; per-side flags on `copt_lp`, equality rows as
  `member` on `copt_milp`; `IsMinIIS` for the completion status.
- [ ] HiGHS: `Highs_getIis` as an optional symbol on `highs_lp` and
  `highs_qp`, falling back when it is missing or returns an empty IIS for
  an infeasible model. The index arrays are sized for the whole model,
  since one call returns the counts and fills them.

Done when the suite passes locally on each commercial backend, and the
compatibility matrix shows HiGHS both before and after 1.12.

## 7. Documentation

- [ ] `docs/reference/concepts.md`: `has_iis` and the row-bound concept.
- [ ] One user guide page with the consumer loop, added to the
  `zensical.toml` navigation. It replaces the pull request's `docs/iis.md`.
- [ ] The IIS suite added to
  `docs/assets/features_tables/tested_features_table.py`, and the tables
  regenerated.
- [ ] The README roadmap row, and the list of missing features in
  `docs/getting-started/coming-from.md`.

## Later

The deferred items of iis.md, in no fixed order: dual rays, native elastic
relaxation, forcing and preferences, Xpress enumeration, SCIP 10's IIS
finder, batching and ordering in the fallback, and a public deletion filter.
