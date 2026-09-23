# IIS computation: design decisions

Design note for the infeasibility-diagnosis feature (roadmap item "IIS").
It records the rulings taken on 2026-09-22 while reviewing the external IIS
pull request, so that the feature is coded once, in the library's own shape.
It is not user documentation. The implementation order is in
[iis_todo.md](iis_todo.md), and the points still waiting for a ruling are
listed under [Open questions](#open-questions).

Statements about solvers were checked on 2026-09-22 against the installed C
headers and small runtime probes: Gurobi 12.0.1, CPLEX 22.1.2, COPT 8.0,
Xpress 47.01, HiGHS 1.10.0 and 1.15.1, and the SCIP 9.2.1 and 10.0 sources.
Other releases of the validated ranges were not checked.

## Shape of the API

- **A capability, not a separate library.** `has_iis<Model>` is a concept in
  `model_concepts.hpp` like every other capability. Where no native routine
  is usable, the generic fallback below provides it, so callers never see
  which path produced the answer. Where the fallback lives is open
  question 1.
- **Explicit computation of the model as it is.** `compute_iis()` is a
  distinct call, like `refine_lp_status()`: it is expensive and may be
  partial, so it never runs behind the caller's back. It analyzes the model
  as it currently is and never relies on the status of an earlier solve,
  which is stale as soon as the model is modified. A feasible model is an
  outcome, not an error.
- **An IIS object shaped like a basis.** The result is a snapshot whose
  `get_status(v)` and `get_status(c)` return a variant over the tags of an
  `iis_status` namespace, mirroring `basis_status`: `absent`, `member`, and
  the refinements `member_lower`, `member_upper` and `member_both`. Each
  backend's variant lists only the tags it can report; the native routines
  below differ on row sides.
- **Interpretation through the caller's own ranges.** Handles carry no
  reverse index to the caller's keys, so the consumer iterates its own
  variable and constraint families and queries each entity, as for a basis.
  The snapshot is taken once and keyed by handle id, translated through the
  remapping layer (`_native_id`, `_var_handle`) when it is taken, so that a
  later `remove_variable` cannot shift it. Every query is then a
  constant-time lookup and never a solver call. Member counts are the only
  summary; member ranges are not needed.
- **A completion status of its own.** The object distinguishes an
  irreducible conflict, a conflict not proven minimal, a feasible model and
  an undetermined outcome, with the reason when a limit or a cancellation
  stopped the search. Every native routine can end partially: Gurobi
  `IISMinimal`, COPT `IsMinIIS`, CPLEX's abort statuses and "possible
  member" flags, Xpress `IISSOLSTATUS`, HiGHS's "maybe in conflict".
- **Each model class explains its own problem.** The IIS of a `*_milp`
  model explains the MIP and that of an `*_lp` model the LP, following the
  2026-07-22 ruling that `*_milp` models expose no LP-only feature.
  Integrality is fixed background, never a member, and no domain flag is
  needed. A native routine that only analyzes the relaxation is therefore
  used on `*_lp` models only.
- **Scope: linear rows and variable bounds.** SOS, indicator, quadratic and
  general constraints have no portable handle, so both paths keep them as
  fixed background. The fallback leaves them active. Native wrappers keep
  them out of the candidates: Gurobi's `IISSOSForce`, `IISQConstrForce` and
  `IISGenConstrForce` set to 1, Xpress's `IISOPS` class bits, and CPLEX by
  leaving them out of the conflict groups. These mechanisms exist in the
  headers, but their effect on special constraints was not probed. A
  wrapper that cannot keep them out must not claim that the reported rows
  and bounds conflict on their own.
- **Names.** No generic identifiers such as `result`, `options` or `member`
  in a namespace users are told to open; the tags live in `iis_status`.

## Native routines

| Solver, probed release | Entry point | Explains | Row sides | Bound sides | Partial answer |
| --- | --- | --- | --- | --- | --- |
| Gurobi 12.0.1 | `GRBcomputeIIS` | the MIP | membership only | `IISLB`, `IISUB` | `IISMinimal` |
| CPLEX 22.1.2 | `CPXrefineconflictext` | the MIP | membership only | lower, upper | abort statuses, "possible" flags |
| COPT 8.0 | `COPT_ComputeIIS` | the MIP | per side, reliable on LPs only | per side | `IsMinIIS` |
| Xpress 47.01 | `XPRSiisfirst`, `XPRSgetiisdata` | the MIP | `L`, `G`, or `E` for both | `L`, `U` | `IISSOLSTATUS` |
| HiGHS 1.15.1 | `Highs_getIis` | the relaxation | per side | per side | "maybe in conflict" |
| SCIP 10.0 sources | `SCIPgenerateIIS`, `SCIPgetIIS` | not examined | a sub-SCIP | a sub-SCIP | irreducible flag |

- **Gurobi and CPLEX.** Neither names the side of a row. An inequality row
  gets its side from its sense; an equality row is reported as `member`.
- **CPLEX.** `CPXrefineconflict` is deprecated since 20.1, so the wrapper
  calls `CPXrefineconflictext` with one group per row and per bound; its
  group preferences are also the hook for forcing later. The refiner
  replaces `CPXgetstat` with its own status (31 minimal, 30 feasible), which
  MIP++ does not see because it caches the solve status.
- **COPT.** On MIPs it flags a single side of an equality row even when both
  are needed: integer x with `x = 0.5`, integer x with `2x = 1.5`, and
  integers x, y with `x + y = 1.5` each got one side only. An LP IIS never
  needs both sides of one row, so the flags are consistent on LPs.
  `copt_milp` therefore reports equality rows as `member`.
- **Xpress.** By default integrality restrictions are removable candidates,
  listed as `I` members. With integers x, y and rows `y = 0` and
  `x + y = 1.5`, the default returned both rows, which is not irreducible
  once integrality is background; with the `IISOPS` integrality bits set it
  returned `x + y = 1.5` alone. `IISOPS` bits mark element classes as fixed,
  and fixed elements are still listed, so the wrapper sets the bits and
  drops the `I` entries.
- **HiGHS.** The routine analyzes "an LP, QP, or the relaxation of a MIP"
  (1.15.1 header): on integer x with `x = 0.5` it returns success with an
  empty IIS. It entered the C API in 1.12.0, being absent from 1.10.0 and
  1.11.0, while the validated range starts at 1.8.1, so the symbol is
  optional. `highs_lp` and `highs_qp` use it when it resolves; `highs_milp`
  always uses the fallback.
- **SCIP.** No routine up to 9.2.1. SCIP 10.0 added IIS finder plugins
  whose answer is a sub-SCIP flagged infeasible and irreducible; mapping it
  back to handles was not examined, so SCIP starts on the fallback.
- **The others.** MOSEK, Clp, Cbc, GLPK and SoPlex have no IIS routine;
  MOSEK's infeasibility report and `MSK_primalrepair` are not one.
- **Split.** Native: `gurobi_*`, `cplex_*`, `copt_*`, `xpress_*`, and
  `highs_lp`, `highs_qp` when `Highs_getIis` resolves. Fallback: `clp_lp`,
  `cbc_milp`, `glpk_*`, `scip_milp`, `soplex_lp`, `mosek_*`, `highs_milp`,
  and `highs_lp`, `highs_qp` on HiGHS older than 1.12.

## The generic fallback

- **In place, on the caller's model.** No copy of the model is built. The
  fallback saves what its trials change, runs them on the model, and
  restores everything through an RAII guard on every exit path, including
  exceptions and cancellation. Special constraints and lazy-constraint
  machinery stay in the model as fixed background, which a copy would have
  dropped.
- **Saved by value.** Variable bounds and row sides are read as scalars.
  The objective, coefficients and offset, is copied into owned storage:
  `get_objective()` is a lazy view over the solver's live coefficients on
  Clp and GLPK, so a saved view would read back the zeros written for the
  trials. On a `qp_model` the quadratic part is saved too, through
  `has_readable_quadratic_objective`, since `set_objective` replaces the
  whole objective. The time limit is saved whenever the fallback forwards
  its remaining budget into it.
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
  side is a candidate; the setters that relax a row side are open
  question 3. A variable whose own bounds cross is a conflict by itself and
  is detected before any solve.
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
- **Only proofs count, read under the zero objective.** A trial proves
  infeasibility through `is_a<status::infeasible>` or the exact
  `infeasible_or_unbounded` tag, which becomes a proof because a zero
  objective cannot be unbounded; SCIP, SoPlex, MOSEK and HiGHS can report
  it. A trial proves feasibility through `status::solution_available`,
  except on `optimal_infeasible_unscaled` and in the `failed` branch.
  Everything else is inconclusive, including `unbounded`, which a zero
  objective makes impossible. An inconclusive trial never drops a member,
  and an inconclusive singleton test prevents the irreducible claim.
- **Algorithm.** The deletion filter: prove the model infeasible as it is,
  then test each candidate once; a candidate is dropped only when the
  remainder is still proven infeasible. A feasible witness stays valid when
  more candidates are removed later, so one pass suffices. Batching and
  candidate ordering are later refinements.
- **Limits.** The remaining budget is forwarded into the model's time limit
  where `has_time_limit` holds, never loosening the caller's own limit,
  which is restored afterwards. A solve-count budget and a stop token bound
  the loop; how they are passed is open question 5. A stopped run keeps the
  last proven infeasible subset and reports the stop in the completion
  status.
- **Side effects.** After a fallback run the solver holds the last trial's
  solution; what `get_status()` reports then is open question 4. Native
  routines leave the model's status untouched. None of the fallback
  backends has a candidate-solution callback today; on a user model that
  has one, the trials run it.

## Library additions the fallback needs

Compile-checked on 2026-09-22 against the model classes that use the
fallback (x: satisfied):

| Model | Variable bounds, read / modify | Row sense and rhs, read / modify | Row bounds, read | Objective, read |
| --- | --- | --- | --- | --- |
| `clp_lp` | x / x | x / x | x | x |
| `cbc_milp` | x / x | x / . | x | x |
| `glpk_lp`, `glpk_milp` | x / x | . / . | . | x |
| `scip_milp` | x / x | . / . | . | x |
| `soplex_lp` | . / . | . / . | . | . |
| `mosek_lp`, `mosek_milp` | x / x | . / . | . | x |
| `highs_lp`, `highs_milp`, `highs_qp` | x / x | x / x | . | x |

- **Row relaxation.** Only `clp_lp` and HiGHS can relax a row today. Every
  fallback backend stores rows natively as two-sided bounds, with a way to
  set each side: `Clp_rowLower` and `Clp_rowUpper`, writable arrays that
  `set_constraint_rhs` already writes through; `Cbc_setRowLower` and
  `Cbc_setRowUpper`; `glp_set_row_bnds`, which takes a bound type;
  `SCIPchgLhsLinear` and `SCIPchgRhsLinear`; `MSK_putconbound` and
  `MSK_getconbound`; `Highs_changeRowBounds`. The api objects of Clp, Cbc,
  GLPK, MOSEK and HiGHS already bind these functions, so every validated
  release has them; SCIP's must be added.
- **SoPlex.** The hardest case. The C API of 6.0.4 cannot change a
  variable's lower bound at all; 7.1.3 adds per-column lower bounds and
  per-row side setters; neither can read a row side back, so readable row
  bounds would need state kept by the wrapper.
- **Behavioral check.** A shared test that `infinity()` frees a row side,
  mirroring `infinity_removes_a_bound`, which already covers variable
  bounds.
- **Visibility.** Each capability suite is instantiated wherever the
  capability becomes satisfied, so the feature tables show it.

## Tests and documentation

- **One shared suite.** `test/test_suites/iis.hpp`, instantiated from each
  `test/solvers/<solver>.cpp`, so the feature tables show the capability and
  the `MIPPP_REQUIRED_SOLVERS` skip policy applies. No separate binary, no
  hard-coded backend list.
- **An independent oracle.** Each case builds its model from its own data,
  so it can rebuild the reported subsystem, check that it is infeasible, and
  check that dropping any single reported side makes it feasible. A row
  reported as `member` is kept whole.
- **Cases.** From the pull request's tests and the 2026-09-22 probes:
  bound-only conflicts; an LP conflict through one side of an equality row;
  integer x with `x = 0.5`, which needs both sides of one row; integers x, y
  with `y = 0` and `x + y = 1.5`, where integrality makes the first row
  redundant; ranged rows; redundant rows; crossing bounds; a feasible model;
  a model modified after an infeasible solve; removed variables on a
  remapping backend; indicator constraints kept as background on Gurobi and
  CPLEX; `compute_iis()` leaving the model to solve to its previous result;
  a column-less model, which Cbc reports as `unknown`, so the outcome is
  undetermined; budget exhaustion; cancellation.
- **CI.** Clp, Cbc, GLPK and HiGHS run in CI, so the fallback is CI-tested;
  the commercial routines are tested locally, and the compatibility matrix
  covers the HiGHS switch at 1.12.
- **Docs.** `docs/reference/concepts.md` gets the concepts; the user guide
  gets one page with the basis-like consumer loop.

## Deferred, and how they would come back

- **Dual rays.** A `has_dual_ray<T, M = T>` capability mirroring
  `get_dual_solution`: a handle-indexed mapping, never a vector in insertion
  order. It can later seed the fallback; the Clp, SoPlex and MOSEK plumbing
  from the pull request is reusable once reshaped into mappings.
- **Elastic relaxation.** Six backends have a native routine: `CPXfeasopt`,
  `GRBfeasrelax`, `COPT_FeasRelax`, `XPRSrepairinfeas`,
  `Highs_feasibilityRelaxation` and `MSK_primalrepair`. The pull request's
  elasticity prefilter returns as a capability with those behind it, not as
  generic code first.
- **Forcing and preferences.** Gurobi forces membership per entity
  (`IISConstrForce`, `IISLBForce`, `IISUBForce`) and CPLEX takes group
  preferences in `CPXrefineconflictext`; Xpress fixes whole classes only
  (`IISOPS`) and COPT 8.0 has neither. Setters on the IIS object, shaped
  like the basis setters, are the natural home.
- **Enumeration.** Xpress's `XPRSiisnext` and `XPRSiisall`; a
  backend-specific extra, not part of the capability.
- **SCIP 10.** Its IIS finder, once the sub-SCIP answer can be mapped back
  to handles.
- **Fallback refinements.** Batching, candidate ordering, and a public
  deletion filter over a user-supplied oracle.

## Open questions

1. **Where the fallback lives.** Recommended: a protected helper that each
   backend's `compute_iis()` member calls, with the deletion filter as a
   standalone engine. A member can enumerate the live handles, which no
   public API exposes and which a remapping backend leaves with holes after
   a removal; it can switch to the fallback at runtime when `Highs_getIis`
   or SCIP 10's finder is missing; and only a member can set the cached
   status of question 4. The alternative, a public free function over the
   capability concepts, would also serve user-defined models such as
   `dumb_lp`, but it needs a public enumeration of entities first.
2. **One call or two.** Recommended: `compute_iis()` returns the snapshot by
   value, like `get_basis()`. A separate `get_iis()` would make the fallback
   store its answer in the model.
3. **How a row side is relaxed.** Recommended: through row bounds, with a
   `has_modifiable_constraint_bounds` twin of
   `has_readable_constraint_bounds`. Every fallback backend is two-sided
   natively, only `clp_lp` and HiGHS can modify sense and rhs today, and
   bounds relax equality and ranged rows without touching senses. The
   alternative, modifiable sense and rhs on five more solvers, cannot relax
   a ranged row.
4. **The model's status after a fallback run.** Recommended: `unknown`,
   which already covers "not yet solved" (ruling of 2026-07-22). The model
   is back to its original data, but the solver holds a trial's solution,
   so restoring an earlier `optimal` would pair it with the wrong solution.
5. **Parameters.** Recommended: no required argument in the concept, and an
   optional aggregate carrying the fallback's solve budget and stop token.
   Native routines honor the model's own time limit only.
6. **Ranged rows in the first version.** They come for free if question 3
   is answered with row bounds; otherwise a ranged row cannot be relaxed and
   must be refused at runtime.

## What the pull request contributed

- **Kept.** The deletion filter, as the fallback's engine. The MOSEK fixes:
  on main (`2da7ebe`), `solve()` calls `MSK_optimize` and then
  `_get_status()` calls `MSK_optimizetrm`, which optimizes again, in both
  `mosek_lp.hpp` and `mosek_milp.hpp`; `~mosek_base()` calls `check()`,
  which can throw from a destructor; plus solution-slot selection, the
  handle guard, and a time limit that still needs `TimeLimitTest`
  instantiated for MOSEK. The Clp, SoPlex and MOSEK certificate plumbing,
  for dual rays later. Its list of test situations.
- **Kept in spirit.** The status classification, re-read under the zero
  objective as above: the pull request treated `infeasible_or_unbounded`
  and `primal_and_dual_infeasible` as inconclusive.
- **Replaced.** The index-based `linear_system` input, the index-based
  results, the policy and options objects, the report formatter, the
  separate test binary and the seventeen-header layout.
- **Outside IIS.** The loader and license diagnostics (`diagnostic_text.hpp`
  and the help text in `solver_library.hpp`) are a separate change, to be
  judged on its own.
