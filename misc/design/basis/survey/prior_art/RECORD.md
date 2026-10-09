# Prior art: LP basis access and warm start in modelling layers

Survey for the MIP++ `lp_basis` draft (`include/mippp/model_concepts.hpp`, tags
`basic`, `nonbasic`, `nonbasic_free`, `nonbasic_at_bound` >
{`nonbasic_at_lower_bound`, `nonbasic_at_upper_bound`, `nonbasic_fixed`}).
Date: 2026-10-09. Read only; nothing in the repo was touched.

## Sources and versions (primary sources read, not recalled)

| Source | Version / commit | Where read |
|---|---|---|
| MathOptInterface.jl | v1.51.2 | `~/.julia/packages/MathOptInterface/CnW3S` |
| JuMP.jl | v1.25.0 | `~/.julia/packages/JuMP/RGIK3` |
| HiGHS.jl | v1.17.0 | `~/.julia/packages/HiGHS/xtjuY` |
| Gurobi.jl | v1.9.2 | `~/.julia/packages/Gurobi/K2XSK` |
| CPLEX.jl | v1.1.1 | `~/.julia/packages/CPLEX/5jmjD` |
| Xpress.jl | master a91893d (2026-07-29) | shallow clone in `dl/Xpress.jl` |
| Clp.jl | master cf6f012 (2026-09-01) | shallow clone in `dl/Clp.jl` |
| Coluna.jl | v0.8.1 | `~/.julia/packages/Coluna/eiA4O` |
| OR-tools (math_opt, linear_solver, lp_data, glop) | master 100f66e (2026-09-17) | sparse clone in `dl/or-tools` |
| Osi | 542f2e5 (2025-12-18) | `~/Softwares/cbc/Osi` |
| CoinUtils | ff6121f (2025-12-18) | `~/Softwares/cbc/CoinUtils` |
| Clp | c537deb (2025-12-15) | `~/Softwares/cbc/Clp` |
| HiGHS (native, for HiGHS.jl context) | 73cac48 (2026-08-19) | `~/Softwares/solvers/HiGHS` |
| python-mip | 0ccb811 (2024-05-23) | `~/Softwares/python-mip` |
| linopy | master e0177b2 (2026-10-09) | shallow clone in `dl/linopy` |
| Pyomo, CVXPY, PuLP, Gravity | GitHub master via `gh api` (2026-10-09) | code search + file reads |
| AMPL/ASL `README.suf` | portal.ampl.com | WebFetch |
| Gurobi, Xpress, CPLEX reference pages | current online docs | WebFetch |

## Comparative table

"Activity" = the status names the side of `lb <= a.x <= ub` the row activity
sits on. "Slack/logical" = the status names a slack variable's bound, which
for the `+1`-logical convention is the opposite side.

| Layer | Read | Write / warm start | Tags | Row side | Fixed tag | Free / superbasic | Entities added after snapshot | MIP |
|---|---|---|---|---|---|---|---|---|
| MOI / JuMP | per-entity attributes `VariableBasisStatus`, `ConstraintBasisStatus` (result attributes; JuMP throws `OptimizeNotCalled` once the model is dirty) | none in MOI ("should not implement set or supports"); Gurobi.jl alone implements `MOI.set` | `BASIC`, `NONBASIC`, `NONBASIC_AT_LOWER`, `NONBASIC_AT_UPPER`, `SUPER_BASIC` | activity side for `Interval` rows ("which side of the constraint is active"); one-sided rows are side-less `NONBASIC` | no | `SUPER_BASIC` = "not in the basis but not at one of its bounds", e.g. a free nonbasic at 0 | no snapshot: any modification invalidates results | wrapper-dependent; no layer synthesizes |
| HiGHS.jl | from cached HiGHS solution | none | maps HiGHS `kLower/kBasic/kUpper/kZero/kNonbasic` | no flip (HiGHS codes passed through unflipped here and in MathOpt, which implies activity side; not confirmed against HiGHS itself, see the `survey/highs` probe); one-sided rows collapsed to `NONBASIC` | no (HiGHS folds fixed into `kLower`) | `kZero` -> `SUPER_BASIC` | n/a | throws "no basis is present" when HiGHS has none |
| Gurobi.jl | `VBasis`/`CBasis` per entity | `MOI.set` for both (sticky Gurobi attribute) | VBasis 0/-1/-2/-3 -> BASIC/AT_LOWER/AT_UPPER/SUPER_BASIC; CBasis 0/-1 -> BASIC/NONBASIC | side-less (CBasis has no side); Interval rows unsupported | no | -3 superbasic <-> `SUPER_BASIC` | n/a | not verified |
| CPLEX.jl | `CPXgetbase` full arrays on every single-entity query | none | rstat collapsed to BASIC/NONBASIC | side-less (asserts `CPX_AT_LOWER` for every nonbasic row); Interval unsupported | no | `CPX_FREE_SUPER` -> `SUPER_BASIC` | n/a | not verified |
| Xpress.jl | `XPRSgetbasis` once, cached | none | rows 0 and 2 both -> `NONBASIC` (even Interval), 3 -> `SUPER_BASIC` | no flip, side discarded | no | 3 -> `SUPER_BASIC`; Xpress reports a free nonbasic at 0 as code 0 (at lower) | n/a | not verified |
| Clp.jl | `Clp_getRowStatus` / `Clp_getColumnStatus` | none | `isFree`->`BASIC` (sic), `isFixed`->`NONBASIC`, superBasic->`SUPER_BASIC` | Clp raw status (activity side), one-sided rows: finite side -> NONBASIC, infinite side -> BASIC | `isFixed` collapsed | `superBasic` -> `SUPER_BASIC`, `isFree` -> `BASIC` | n/a | LP only |
| OR-tools MathOpt | `Solution.basis` (`Basis` value struct: `VariableMap<BasisStatus>`, `LinearConstraintMap<BasisStatus>`, `basic_dual_feasibility`) | `ModelSolveParameters.initial_basis` (per-solve, one shot); Gurobi, GLOP (and Xpress code path) | `FREE`, `AT_LOWER_BOUND`, `AT_UPPER_BOUND`, `FIXED_VALUE`, `BASIC` (+`UNSPECIFIED` guard) | activity side; adapters flip slack-sided solvers (Xpress) and derive side from bounds (Gurobi) | yes, "identical finite lower and upper bounds" | `FREE` = "no finite bounds"; no superbasic (Gurobi -3 -> `FREE`, lossy) | strict: `ValidateBasis` requires ids identical to the model's and #nonbasic == #variables; stale basis rejected | LP only (basis filled in the LP solution path) |
| OR-tools MPSolver | `MPVariable::basis_status()`, `MPConstraint::basis_status()` | `MPSolver::SetStartingLpBasis(var_statuses, con_statuses)` (GLOP, Xpress; others `LOG(FATAL)`) | `FREE`, `AT_LOWER_BOUND`, `AT_UPPER_BOUND`, `FIXED_VALUE`, `BASIC` | activity side, documented as the slack status "with AT_UPPER_BOUND and AT_LOWER_BOUND swapped" | yes | `FREE` (Clp superBasic -> FREE) | dense vectors must match current sizes | "only available for continuous problems" |
| Coin-OR Osi | `getBasisStatus(int*,int*)`, `getWarmStart()` -> `CoinWarmStart*` | `setWarmStart(ws)` (used by next solve), `setBasisStatus` (immediate, no pivots) | 0 free, 1 basic, 2 at upper, 3 at lower | slack/logical side: "+1 coefficients, so for a <= constraint the logical will be at lower bound if the constraint is tight" | no (OsiClp maps `isFixed` to upper/lower by reduced-cost sign) | `isFree`; `superBasic` (=4) exists in the enum but cannot be stored in the 2-bit packing | `CoinWarmStartBasis::resize`: new columns `atLowerBound`, new rows `basic`; `deleteRows/Columns` valid only if deleted rows were basic / columns nonbasic, fix-up "left to the client" | the LP relaxation's |
| AMPL / ASL `sstatus` suffix (Pyomo `Suffix(IMPORT_EXPORT)`) | suffix on variables and constraints after solve | same suffix sent to next solve | `none`, `bas`, `sup`, `low`, `upp`, `equ`, `btw` | not stated by README.suf | yes, `equ` | `sup` (superbasic) and `btw` (nonbasic between bounds) separate | `none` = "no status assigned" | driver-dependent |
| Pyomo (Gurobi persistent) | `get_var_attr(v,'VBasis')`, `get_linear_constraint_attr(c,'CBasis')` | `set_var_attr`, `set_linear_constraint_attr` (raw Gurobi codes) | raw solver codes | Gurobi's (side-less CBasis) | no | raw | raw solver | raw |
| python-mip | `.bas` file write (CBC; HiGHS also handles `.bas`) | `.bas` file read | MPS BAS (XU/XL/UL/LL) | file format | no | no | BAS default: rows basic, columns at lower | relaxation basis |
| linopy | `basis_fn` file after solve | `warmstart_fn` basis file (HiGHS `readBasis`, CBC `-basisI`, GLPK `--ini`, ...) | solver-native file | file | file | file | names in file; forces names on | file |
| CVXPY | no | `warm_start=True` sets Gurobi `start` (primal), no basis | none | - | - | - | - | - |
| PuLP | no | no solver-independent basis | none | - | - | - | - | - |
| Gravity | no | Ipopt/Bonmin point warm start only | none | - | - | - | - | - |
| Coluna | no (a source note lists `ConstraintBasisStatus` as something to "make use of") | none | none | - | - | - | - | - |
| ojAlgo | not surveyed | | | | | | | |

## Per-layer notes with citations

### MathOptInterface / JuMP

- `BasisStatusCode` = `BASIC`, `NONBASIC`, `NONBASIC_AT_LOWER`,
  `NONBASIC_AT_UPPER`, `SUPER_BASIC`. Doc: for `ConstraintBasisStatus`,
  `NONBASIC_AT_*` "should be returned only for constraints with the Interval
  set. In this case, they are necessary to distinguish which side of the
  constraint is active. One-sided constraints ... should use NONBASIC". For
  `VariableBasisStatus`, `NONBASIC_AT_*` is returned "regardless of whether the
  alternative bound exists". `SUPER_BASIC`: "not in the basis but is also not at
  one of its bounds. In a linear program, this status occurs when a variable
  with no bounds is not in the basis".
  https://github.com/jump-dev/MathOptInterface.jl/blob/v1.51.2/src/attributes.jl#L2125-L2166
- `VariableBasisStatus`: "Optimizers should implement ... get ... They should
  not implement set or supports."
  https://github.com/jump-dev/MathOptInterface.jl/blob/v1.51.2/src/attributes.jl#L2168-L2205
- `ConstraintBasisStatus` does not apply to `VariableIndex` constraints (bound
  status is read through the variable).
  https://github.com/jump-dev/MathOptInterface.jl/blob/v1.51.2/src/attributes.jl#L2463-L2505
- Both are `is_set_by_optimize` (result attributes).
  https://github.com/jump-dev/MathOptInterface.jl/blob/v1.51.2/src/attributes.jl#L3449-L3474
- Design history. PR #679 removed `VariableBasisStatus` as redundant with the
  basis status of bound constraints
  (https://github.com/jump-dev/MathOptInterface.jl/pull/679); issue #1297
  re-added it because "we can't tell if variables with free bounds with a
  VariablePrimal of 0.0 are in the (degenerate) basis or not", with mlubin:
  "Maybe we should stop fighting against the solver APIs by reviving
  VariableBasisStatus" (https://github.com/jump-dev/MathOptInterface.jl/issues/1297).
  Issue #118 proposed a `VariableBasisStatusStart` for warm start; closed
  without one (https://github.com/jump-dev/MathOptInterface.jl/issues/118).
- JuMP: a modified model throws `OptimizeNotCalled` on any result attribute,
  basis included (outside DIRECT mode).
  https://github.com/jump-dev/JuMP.jl/blob/v1.25.0/src/optimizer_interface.jl#L1160-L1178
- JuMP's basis tutorial describes `ConstraintBasisStatus` as "the basis status
  of the slack variables associated with a constraint" for `A x + I s = b`; for
  one-sided rows the code is side-less so the slack/activity question never
  arises; for `Interval` the spec names the active side of the constraint.
  https://jump.dev/JuMP.jl/stable/tutorials/linear/basis/

### MOI solver wrappers

- HiGHS.jl: `_BASIS_STATUS_MAP` kLower->NONBASIC_AT_LOWER, kBasic->BASIC,
  kUpper->NONBASIC_AT_UPPER, kZero->SUPER_BASIC, kNonbasic->NONBASIC;
  `_replace_basis_status` collapses AT_* to NONBASIC for non-Interval rows; no
  flip. Throws `GetAttributeNotAllowed(attr, "no basis is present")`. No set.
  https://github.com/jump-dev/HiGHS.jl/blob/v1.17.0/src/MOI_wrapper.jl#L2596-L2640
  HiGHS native: `kLower = 0, // (slack) variable is at its lower bound
  [including fixed variables]`, `kZero // free variable is nonbasic and set to
  zero`, `kNonbasic // nonbasic with no specific bound information - useful for
  users and postsolve`.
  https://github.com/ERGO-Code/HiGHS/blob/73cac48c5340d775a477087198611862559be250/highs/lp_data/HConst.h#L249-L257
- Gurobi.jl: get and set for both attributes. CBasis 0->BASIC, -1->NONBASIC;
  VBasis 0/-1/-2/-3 -> BASIC/AT_LOWER/AT_UPPER/SUPER_BASIC; `MOI.set` errors
  on a constraint `NONBASIC_AT_*` and asserts on a variable `NONBASIC`.
  Interval rows are deliberately unsupported because Gurobi's range slack
  columns make indices hard to track.
  https://github.com/jump-dev/Gurobi.jl/blob/v1.9.2/src/MOI_wrapper/MOI_wrapper.jl#L3871-L3956
  https://github.com/jump-dev/Gurobi.jl/blob/v1.9.2/src/MOI_wrapper/MOI_wrapper.jl#L658-L660
  Gurobi doc: CBasis "Possible values are 0 (basic) or -1 (non-basic). A
  constraint is basic when its slack variable is in the simplex basis ... if
  you wish to specify an advanced starting basis, you must set basis status
  information for all constraints and variables"; a valid start disables LP
  presolve unless `LPWarmStart=2`.
  https://docs.gurobi.com/projects/optimizer/en/current/reference/attributes/constraintlinear.html#cbasis
  Whether `set_attribute(x, MOI.VariableBasisStatus(), ...)` round-trips
  through JuMP's CachingOptimizer was not verified; it works on the Gurobi
  `Optimizer` (direct mode).
- CPLEX.jl: calls `CPXgetbase` and allocates the full arrays on every
  single-entity query (O(n) per call, O(n^2) to read the whole basis); rows:
  "CPLEX uses CPX_AT_LOWER regardless of whether it is <= or >=" -> NONBASIC.
  https://github.com/jump-dev/CPLEX.jl/blob/v1.1.1/src/MOI/MOI_wrapper.jl#L3781-L3818
  CPLEX doc: rstat for non-ranged rows is slack-based ("nonbasic at value 0.0");
  ranged rows have AT_LOWER/AT_UPPER of the slack.
  https://www.ibm.com/docs/en/icos/22.1.2?topic=g-cpxxgetbase-cpxgetbase
- Xpress.jl: one `XPRSgetbasis` call cached in a `BasisStatus` struct; rows
  0 and 2 both -> NONBASIC even for Interval (contradicts the MOI spec);
  3 -> SUPER_BASIC.
  https://github.com/jump-dev/Xpress.jl/blob/a91893d34bcdef040e83ea92244adf0e8d5699f8/src/MOI/MOI_wrapper.jl#L3946-L4001
  Xpress doc: rowstat is the "slack, surplus or artificial" status; colstat
  0 = "non-basic at lower bound, or superbasic at zero if the variable has no
  lower bound"; "basis information always refers to the original, unpresolved
  model".
  https://www.fico.com/fico-xpress-optimization/docs/latest/solver/optimizer/HTML/XPRSgetbasis.html
- Clp.jl: `_CLP_BASIS_STATUS` maps isFree (0x00) -> BASIC, isFixed -> NONBASIC;
  `_nonbasic_status` for LessThan returns BASIC when Clp says atLowerBound,
  for GreaterThan BASIC when atUpperBound. Both look like defects (a free
  nonbasic is reported basic).
  https://github.com/jump-dev/Clp.jl/blob/cf6f01207e3f0fe8ff680e7a93bad27ad43e0388/src/MOI_wrapper/MOI_wrapper.jl#L729-L780

### Google OR-tools MathOpt

- `BasisStatusProto`: `BASIS_STATUS_FREE` "free (it has no finite bounds)",
  `AT_LOWER_BOUND` / `AT_UPPER_BOUND` "(which must be finite)", `FIXED_VALUE`
  "identical finite lower and upper bounds", `BASIC`; `UNSPECIFIED` is a guard.
  `BasisProto` = `constraint_status`, `variable_status`
  (`SparseBasisStatusVector`, ids sorted and distinct),
  `basic_dual_feasibility` (ignored on input).
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solution.proto#L160-L237
- `ModelSolveParametersProto.initial_basis`: "Optional initial basis for warm
  starting simplex LP solvers. If set, it is expected to be valid according to
  ValidateBasis".
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/model_parameters.proto#L140-L143
  C++: `std::optional<Basis> initial_basis` in `ModelSolveParameters`;
  `struct Basis { LinearConstraintMap<BasisStatus> constraint_status;
  VariableMap<BasisStatus> variable_status; std::optional<SolutionStatus>
  basic_dual_feasibility; }`.
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/cpp/solution.h#L214-L249
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/cpp/model_solve_parameters.h#L99-L101
- `ValidateBasis`: ids must be identical to the model's variables and linear
  constraints (complete, no extras), and the number of non-BASIC entries must
  equal the number of variables. Bounds/status consistency is not checked.
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/validators/solution_validator.cc#L253-L291
- Gurobi adapter: VBasis superbasic -> FREE (lossy for a bounded superbasic);
  on set FIXED_VALUE -> `GRB_NONBASIC_LOWER`, FREE -> `GRB_SUPERBASIC`. On get,
  a nonbasic row gets its side from the row's bound shape (<= -> AT_UPPER,
  >= -> AT_LOWER, = -> FIXED_VALUE, with a TODO to refine by dual sign); ranged
  rows read MathOpt's own slack column.
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/gurobi_solver.cc#L117-L146
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/gurobi_solver.cc#L741-L855
  Basis only fetched in the LP path (`GetSolutions` dispatches MIP first).
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/gurobi_solver.cc#L1123-L1139
- GLOP adapter: one-to-one with GLOP's own enum (MathOpt's enum is GLOP's).
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/glop_solver.cc#L480-L528
  GLOP: `FIXED_VALUE` "Only possible status of a FIXED_VARIABLE not in the
  basis"; `FREE` "Only possible status of an UNCONSTRAINED non-basic variable.
  Its value should be zero"; ConstraintStatus "is the VariableStatus of the
  slack variable associated to a constraint modulo a change of sign".
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/lp_data/lp_types.h#L194-L240
  `LPSolver::SetInitialBasis`: "for the next Solve() only ... If the set of
  ... BASIC status does not form a basis a warning will be logged and the code
  will ignore it ... disable presolve".
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/glop/lp_solver.h#L76-L91
- HiGHS adapter (read only; `lp_initial_basis_tests` listed as missing): kLower
  -> AT_LOWER ("highs returns lower for fixed ... TODO: investigate returning
  fixed instead"), kZero -> FREE; for kNonbasic it infers the side from the
  value with a hardcoded 1e-10 tolerance and returns `nullopt` (no basis at all)
  when ambiguous: "Mathopt does not [have] an encoding for nonbasic but unknown
  upper/lower". Line 409-410 maps an upper-bounded-only kNonbasic to
  AT_LOWER_BOUND, an apparent sign bug.
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/highs_solver.cc#L370-L414
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/highs_solver_test.cc#L14-L19
- Xpress adapter: explicit row flip ("XPRESS row basis status is that of the
  slack variable. For example, if the slack variable is at LB, the constraint
  is at UB"); `FIXED_VALUE` falls to `default: return XPRS_FREE_SUPER` on set
  (apparent defect); rejects a basis on a presolved problem.
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/xpress_solver.cc#L227-L263
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/math_opt/solvers/xpress_solver.cc#L570-L601

### Google OR-tools linear_solver (MPSolver)

- `enum BasisStatus { FREE = 0, AT_LOWER_BOUND, AT_UPPER_BOUND, FIXED_VALUE,
  BASIC }`, "possible basis status values for a variable and the slack
  variable of a linear constraint"; `SetStartingLpBasis(variable_statuses,
  constraint_statuses)` "takes a starting basis to be used in the next LP
  Solve() call"; GLOP warning to disable presolve. Default interface:
  `LOG(FATAL) << "Not supported by this solver."` (implemented by GLOP and
  Xpress).
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/linear_solver.h#L725-L750
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/linear_solver.h#L1823-L1828
- `MPConstraint::basis_status()`: "if a constraint linear_expression in
  [lb, ub] is transformed into linear_expression + slack = 0 with slack in
  [-ub, -lb], then this status is the same as the status of the slack variable
  with AT_UPPER_BOUND and AT_LOWER_BOUND swapped"; both getters "only
  available for continuous problems".
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/linear_solver.h#L1212-L1218
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/linear_solver.h#L1368-L1380
- Gurobi interface: nonbasic row -> side from sense when `|Slack| <=
  FeasibilityTol` (equality -> AT_UPPER_BOUND, not FIXED_VALUE), otherwise
  FREE. CLP: superBasic -> FREE, isFixed -> FIXED_VALUE. GLPK: GLP_NF -> FREE,
  GLP_NS -> FIXED_VALUE.
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/gurobi_interface.cc#L866-L900
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/clp_interface.cc#L504-L523
  https://github.com/google/or-tools/blob/100f66e6242ab8bf8d32feb8f3bf086db66ae2b5/ortools/linear_solver/glpk_interface.cc#L646-L663

### Coin-OR Osi / CoinWarmStartBasis

- `getBasisStatus(int* cstat, int* rstat)`: "0: free, 1: basic, 2: nonbasic at
  upper bound, 3: nonbasic at lower bound ... Logical variables associated with
  rows are all assumed to have +1 coefficients, so for a <= constraint the
  logical will be at lower bound if the constraint is tight."
  `setBasisStatus`: "Think of it as setWarmStart immediately followed by
  resolve, but no pivots are allowed."
  https://github.com/coin-or/Osi/blob/542f2e5b87f7563ca8b8019bb2bd1eed8e52887b/src/Osi/OsiSolverInterface.hpp#L2032-L2072
- Warm start objects: `getEmptyWarmStart()`, `getWarmStart()` (opaque
  solver-typed `CoinWarmStart*`), `setWarmStart(ws)` returns accepted or not;
  null refreshes from the solver.
  https://github.com/coin-or/Osi/blob/542f2e5b87f7563ca8b8019bb2bd1eed8e52887b/src/Osi/OsiSolverInterface.hpp#L448-L497
- `CoinWarmStartBasis::Status { isFree = 0x00, basic = 0x01, atUpperBound =
  0x02, atLowerBound = 0x03, superBasic = 0x04 }`, packed 2 bits per entry
  (`& 3`), so `superBasic` is not storable.
  https://github.com/coin-or/CoinUtils/blob/ff6121fef357ab666e91e73c4856fe347d37198b/src/CoinWarmStartBasis.hpp#L41-L61
  https://github.com/coin-or/CoinUtils/blob/ff6121fef357ab666e91e73c4856fe347d37198b/src/CoinWarmStartBasis.hpp#L350-L365
- `setSize` discards (all `isFree`, "not a valid basis"); `resize` keeps: "the
  status of new structural variables (added columns) is set to nonbasic at
  lower bound, and the status of new artificial variables (added rows) is set
  to basic. (The basis can be invalid if new structural variables do not have
  a finite lower bound.)"; `deleteRows`: "guaranteed valid only if all deleted
  constraints are slack ... This correction is left to the client";
  `deleteColumns`: valid only if deleted variables are nonbasic; `mergeBasis`
  copies runs between bases; `generateDiff`/`applyDiff`.
  https://github.com/coin-or/CoinUtils/blob/ff6121fef357ab666e91e73c4856fe347d37198b/src/CoinWarmStartBasis.hpp#L177-L254
  https://github.com/coin-or/CoinUtils/blob/ff6121fef357ab666e91e73c4856fe347d37198b/src/CoinWarmStartBasis.cpp#L142-L183
- OsiClp: Clp's own row status is activity-sided, Osi flips it ("Flip slacks",
  `lookupA = {0, 1, 3, 2, 0, 3}`); Clp `isFixed` becomes at-lower or at-upper by
  the sign of the dual / reduced cost; `superBasic` becomes `isFree`. On
  `setBasisStatus` it repairs impossible statuses: both bounds infinite ->
  free, at an infinite bound -> the other side, `isFixed` when lb == ub, status
  0 with finite bounds -> `superBasic`.
  https://github.com/coin-or/Clp/blob/c537deba3e3faef7a621c75a6ac3a82f36d962a6/src/OsiClp/OsiClpSolverInterface.cpp#L5812-L5860
  https://github.com/coin-or/Clp/blob/c537deba3e3faef7a621c75a6ac3a82f36d962a6/src/OsiClp/OsiClpSolverInterface.cpp#L6992-L7110
  Clp's in-memory status on resize uses the same default (new columns 3 =
  atLowerBound, new rows 1 = basic).
  https://github.com/coin-or/Clp/blob/c537deba3e3faef7a621c75a6ac3a82f36d962a6/src/ClpModel.cpp#L1308-L1331
  `ClpSimplex::Status` adds `isFixed = 0x05` ("fixed at lower bound and out of
  basis").
  https://github.com/coin-or/Clp/blob/c537deba3e3faef7a621c75a6ac3a82f36d962a6/src/ClpSimplex.hpp#L112-L123

### AMPL sstatus / Pyomo

- `$sstatus_table`: 0 none "no status assigned", 1 bas, 2 sup "superbasic",
  3 low "nonbasic <= (normally =) lower bound", 4 upp, 5 equ "nonbasic at
  equal lower and upper bounds", 6 btw "nonbasic between bounds"; drivers
  "translate your basis information to values in the outgoing status arrays".
  https://portal.ampl.com/~dmg/netlib/ampl/solvers/README.suf
- Pyomo exchanges it with `model.sstatus = Suffix(direction=IMPORT_EXPORT,
  datatype=INT)` on NL solvers (example uses `gurobi_ampl`, presolve off).
  https://github.com/Pyomo/pyomo/blob/main/examples/pyomo/suffixes/gurobi_ampl_basis.py
- Pyomo GurobiPersistent: `set_var_attr` options include `VBasis`, `PStart`;
  `set_linear_constraint_attr` options `CBasis`, `DStart`, `Lazy`; raw Gurobi
  values. No solver-independent basis API.
  https://pyomo.readthedocs.io/en/stable/api/pyomo.solvers.plugins.solvers.gurobi_persistent.GurobiPersistent.html

### python-mip, linopy, CVXPY, PuLP, Gravity, Coluna

- python-mip: `Model.read/write` with `.bas` ("optimal basis for the linear
  programming relaxation"), CBC via `Cbc_readBasis` / `Cbc_writeBasis`.
  https://github.com/coin-or/python-mip/blob/0ccb81115543e737ab74a4f1309891ce5650c8d5/mip/model.py#L655-L710
  https://github.com/coin-or/python-mip/blob/0ccb81115543e737ab74a4f1309891ce5650c8d5/mip/cbc.py#L1462-L1505
  MPS BAS default: "all rows (auxiliary variables) are assumed to be basic and
  all columns (structural variables) are assumed to be non-basic" at lower
  bound; XU/XL pair a basic column with a nonbasic row.
  http://lpsolve.sourceforge.net/5.5/bas-format.htm
- linopy: `Model.solve(basis_fn=..., warmstart_fn=...)`; names forced on when
  either is given "as basis files refer to names"; HiGHS `readBasis` /
  `writeBasis`, CBC `-basisI`/`-basisO`, GLPK `--ini`/`-w`.
  https://github.com/PyPSA/linopy/blob/e0177b2baa8ba7fdbb36abcb3e7eaf7625071a98/linopy/model.py#L2120-L2147
  https://github.com/PyPSA/linopy/blob/e0177b2baa8ba7fdbb36abcb3e7eaf7625071a98/linopy/solvers.py#L1860-L1882
- CVXPY: `warm_start` for Gurobi sets the variables' `start` from the previous
  solution or user values; no basis anywhere (code search for VBasis: 0 hits).
  https://github.com/cvxpy/cvxpy/blob/master/cvxpy/reductions/solvers/qp_solvers/gurobi_qpif.py
- PuLP: no solver-independent basis (hits are GLPK `glp_adv_basis` and a COPT
  raw `WriteBasis`).
- Gravity: no LP basis API (hits for "basis" are graph cycle bases; warm start
  only sets Ipopt `warm_start_init_point` / Bonmin `warm_start`).
  https://github.com/coin-or/Gravity/blob/master/include/gravity/solver.h
- Coluna: only a note in `src/MathProg/MOIinterface.jl` listing
  `MOI.ConstraintBasisStatus()` among things to "make use of".

## Cross-cutting findings

1. Row side. Every solver-independent surface reports the activity side:
   MOI ("which side of the constraint is active", Interval only), MathOpt and
   GLOP, MPSolver (documents the swap from the slack), AMPL `low`/`upp`. Only
   Osi/CoinWarmStartBasis expose the logical (slack) side and document it as a
   trap. Flips live in adapters for slack-sided natives (OsiClp `lookupA`,
   MathOpt Xpress `isConstraint`). Clp's native row status is activity-sided
   (OsiClp `setBasisStatus` sets the row activity to `rowUpper` for
   `atUpperBound`). HiGHS codes are passed through unflipped by both HiGHS.jl
   and MathOpt, which implies the same; HConst.h says "(slack) variable", so
   this is to be confirmed by the HiGHS probe.
   Several MOI wrappers sidestep the question by collapsing rows to a
   side-less `NONBASIC` (HiGHS.jl for one-sided rows, CPLEX.jl, Xpress.jl,
   Gurobi.jl).
2. Side-less nonbasic is real. MOI `NONBASIC`, Gurobi CBasis -1, HiGHS
   `kNonbasic` ("useful for users"), CPLEX non-ranged rows. MathOpt lacks it
   and its HiGHS adapter drops the whole basis when it cannot pick a side.
3. Free vs superbasic. Names diverge for the same LP case (nonbasic free
   variable at 0): FREE (MathOpt, GLOP, MPSolver, Osi `isFree`, GLPK NF),
   SUPER_BASIC (MOI), kZero (HiGHS), CPX_FREE_SUPER (CPLEX), Gurobi -3, Xpress
   reports it as code 0 "at lower". Bounded-and-between-bounds nonbasic is a
   separate concept only in Clp (`superBasic`), AMPL (`sup`, `btw`), and
   OsiClp's set repair; MathOpt and MPSolver squash it into FREE.
4. Fixed. Own tag in MathOpt, GLOP, MPSolver, Clp (`isFixed`), GLPK (`NS`), AMPL
   (`equ`); absent in MOI, Osi, Gurobi, CPLEX, Xpress, HiGHS (folded into
   lower). Adapters disagree on get: MathOpt-Gurobi FIXED_VALUE for equality
   rows but AT_LOWER for fixed variables; MathOpt-HiGHS AT_LOWER; MPSolver-Gurobi
   AT_UPPER for equality rows; OsiClp picks at-lower/at-upper by dual sign,
   which carries more information than "fixed". On set, `fixed` maps to
   at-lower (MathOpt-Gurobi) or, by a defect, to superbasic (MathOpt-Xpress).
5. Extension. Two schools. Implicit default (CoinWarmStartBasis::resize, Clp's
   own array, MPS BAS): new columns at lower, new rows basic, which keeps
   #basic == #rows. Strict (MathOpt `ValidateBasis`, Gurobi "must set ... all",
   JuMP `OptimizeNotCalled`): a basis must cover exactly the current model.
   AMPL has an explicit "none" status. Deletions are the client's job in Osi.
6. Object shape. MOI: per-entity attributes, no object. MathOpt: value struct
   of two id-keyed maps, freely editable, validated at solve time. MPSolver: two
   dense vectors. Osi: opaque `CoinWarmStart` plus an editable
   `CoinWarmStartBasis` with resize/delete/merge/diff. Nobody exposes a live
   view into the solver. Read performance: Xpress.jl fetches once and caches;
   CPLEX.jl re-fetches the whole arrays per entity.
7. Warm-start trigger. One-shot per-solve parameter (MathOpt `initial_basis`,
   GLOP `SetInitialBasis` "next Solve() only", MPSolver
   `SetStartingLpBasis`); sticky model state (Gurobi VBasis/CBasis via
   Gurobi.jl or Pyomo, Osi `setWarmStart`); immediate (Osi `setBasisStatus`,
   no pivots). Files (python-mip, linopy, AMPL suffix round trip). Persistent
   layers (JuMP, MathOpt IncrementalSolver, python-mip, Coluna) otherwise rely
   on the solver keeping its own basis on the live model. Presolve
   interactions are documented everywhere: GLOP and MPSolver say disable it,
   Gurobi turns LP presolve off unless `LPWarmStart=2`, Xpress refuses a basis
   in presolved state.
8. Ranged rows are where the side matters. MOI asks for `NONBASIC_AT_*` on
   `Interval` rows only. Gurobi.jl and CPLEX.jl refuse `Interval` rows outright
   (Gurobi adds range slack columns that break index tracking). MathOpt-Gurobi
   models a ranged row with its own slack column, reads the side from that
   column, and on set makes "MathOpt slack basic, Gurobi row nonbasic" because
   the two slacks are linearly dependent. CPLEX's ranged-row rstat is
   slack-sided (`CPX_AT_UPPER` = slack at its upper bound). Xpress's rowstat 2
   (slack at upper) is meaningful when the slack has a finite upper bound,
   i.e. a ranged row (inference). Xpress.jl discards the side even there.
9. MIP. No layer synthesizes a basis on a MIP. MathOpt fills `basis` only in the
   LP path; MPSolver documents "only available for continuous problems";
   python-mip calls it the LP relaxation basis. MOI wrappers just query the
   solver (HiGHS.jl throws when absent); Gurobi.jl/CPLEX.jl on MIP not verified.

## Lessons for MIP++

1. Free tag name. `nonbasic_free` matches the majority (MathOpt, GLOP, MPSolver,
   Osi, GLPK). Decide its meaning: strict "no finite bound, value 0" (GLOP,
   MathOpt) or "nonbasic and not at a bound" (MOI `SUPER_BASIC`, OsiClp's
   repair of status 0 with finite bounds to `superBasic`). The broader meaning
   lets backends report Clp/CPLEX/Gurobi/Xpress superbasics without a seventh
   tag; the backend that needs the strict meaning (GLOP-like) coerces on set.
   Recommendation: the broad meaning. MOI chose `SUPER_BASIC` for the same
   case; issue #440 notes it "seems odd when thinking about (MI)LPs" and asked
   the docs to call it a nonlinear case, but MOI kept the name, so this is
   weak support for preferring `nonbasic_free`.
2. Row side. Activity side, as the draft's `set_nonbasic(c, s)` comment already
   assumes (`s` = a.x). Every solver-independent layer does this; the flip
   belongs in the slack-sided backends (Xpress, CPLEX ranged rows, Cbc/Osi).
   Document it once next to the tags, with the `<=` example Osi uses. Ranged
   constraints are where `nonbasic_at_lower/upper_bound` on a constraint earns
   its keep and where backend flips must be tested (finding 8); one-sided rows
   cannot catch a missing flip.
3. Keep the side-less `nonbasic` base tag readable and settable. It is the
   honest answer for Gurobi CBasis -1, HiGHS `kNonbasic`, CPLEX non-ranged
   rows, and the gap MathOpt explicitly regrets. Return the most specific tag a
   backend can derive cheaply (MathOpt derives `<=` -> at upper, `>=` -> at
   lower from the bound shape); tests compare with `is_a<nonbasic>` /
   `is_a<nonbasic_at_bound>`.
4. `nonbasic_fixed` under `nonbasic_at_bound` is the right shape. On set, map it
   to at-lower where the solver has no fixed code. On get, either always
   normalize (lb == ub -> fixed) or document that a fixed entity may report
   at-lower/at-upper; the side carries the dual sign (OsiClp, MathOpt TODO),
   so normalizing loses information. Recommendation: accept `nonbasic_fixed`
   on set, do not normalize on get (backends with a native fixed code report
   it, others report the side); solver-agnostic tests use
   `is_a<nonbasic_at_bound>` for fixed entities.
5. Entities added after the snapshot. Prior art offers the CoinWarmStartBasis /
   MPS-BAS default (new variable nonbasic at its finite bound, new constraint
   basic), which preserves #basic == #rows and is what Clp does internally;
   or MathOpt's strict exact-cover rule. The default is the cheaper and more
   useful one for column/row generation. Recommendation: the default, refined
   where Coin's is known invalid: a new variable without a finite lower bound
   gets at upper, or free when it has no finite bound. Removals: Osi leaves validity to the client
   (removing a basic column or a nonbasic row breaks the count); with
   index-shifting handles a pre-removal snapshot is misaligned, so either
   document set_basis after removal as UB or key the snapshot by stable ids.
6. Snapshot vs editable. A detached value object (MathOpt `Basis`,
   CoinWarmStartBasis) with `get_status` and `set_*` is the established shape;
   no layer gives a live view. Fill it with one bulk call per side (Xpress.jl)
   rather than per-entity solver calls (CPLEX.jl is O(n^2)). Validity checks
   (#basic == #rows) are cheap and MathOpt does them; Osi and GLOP do not
   (GLOP warns and ignores). Recommendation, per the misuse-is-documented-UB
   ruling: no guard, document the count requirement, and let backend errors
   surface through the usual error path.
7. Warm-start trigger. `model.set_basis(b)` maps directly to the sticky native
   setters (Gurobi attributes, HiGHS setBasis, Osi setWarmStart, Xpress
   loadbasis, CPLEX copybase). Recommendation: `set_basis` installs the basis
   as the solver's current basis, the next solve starts from it, and later
   solves start from whatever the solver then holds (the native semantics;
   MathOpt's one-shot parameter exists because its solve is stateless).
   Document presolve interactions. The implicit warm start of re-solving the
   live model needs no API.
8. `set_nonbasic(v, s)` snapping. Precedents: MathOpt-HiGHS uses a hardcoded
   1e-10 and gives up when ambiguous; MPSolver-Gurobi uses `FeasibilityTol` and
   falls back to FREE; OsiClp coerces impossible sides from the bounds. The
   tolerance should be explicit (parameter or the model's feasibility
   tolerance), and the ambiguous outcome needs a rule. Recommendation: within
   tolerance of lb -> at lower (fixed when lb == ub), of ub -> at upper,
   otherwise `nonbasic_free` under the broad meaning of lesson 1 (OsiClp's
   repair does exactly this); a backend without a superbasic code coerces to
   the nearest finite bound.
9. MIP. Exposing basis only on `*_lp` matches every layer surveyed and the
   2026-07-22 ruling that `*_milp` models expose pure MILP features only.

## Defects observed in reference implementations (not design)

- MathOpt HiGHS adapter: upper-bounded-only `kNonbasic` -> AT_LOWER_BOUND
  (highs_solver.cc:409-410).
- MathOpt Xpress adapter: `FIXED_VALUE` -> `XPRS_FREE_SUPER` on set
  (xpress_solver.cc:258-261, default branch).
- Xpress.jl: Interval rows reported `NONBASIC` instead of `NONBASIC_AT_*`.
- Clp.jl: `isFree` -> `BASIC`; one-sided rows on the infinite side -> `BASIC`.
- CPLEX.jl: full `CPXgetbase` per single-entity query.
- MPSolver-Gurobi: nonbasic equality row -> AT_UPPER_BOUND, not FIXED_VALUE.

## Not verified

- ojAlgo (optional, not surveyed).
- JuMP CachingOptimizer round trip of `MOI.set(VariableBasisStatus)` on Gurobi.
- Gurobi.jl / CPLEX.jl / Xpress.jl behaviour when queried on a MIP.
- Whether Pyomo writes `none` (0) for components without an `sstatus` value.
