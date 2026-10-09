# MOSEK LP basis survey (mosek_lp)

Surveyed 2026-10-09 against repo main @ 04431ca (read-only). Every claim is
tagged MEASURED (probe run here), DOCUMENTED (vendor header / manual, cited)
or INFERRED.

Manual citations: `capi.pdf` = `/home/plaiseek/Softwares/solvers/mosek/11.0/doc/capi.pdf`
(MOSEK 11.0 C API manual; page = printed page number). Header citations:
`mosek.h` = `$MOSEK_HOME/h/mosek.h` (11.0.14).

## Probe files (all in this directory)

| file | purpose |
|---|---|
| `probe.cpp` (+ `common.hpp`, its helpers) | C (row/column convention), D (availability), E (warm start, invalid bases with primal simplex), F (partial bases). Output `probe_out.txt` (11.0.14), `probe_out_11.1.11.txt`, `probe_out_11.2.6.txt`, `probe_out_10.2.19.txt` |
| `probe2.cpp` | cold iteration counts, MOSEK log of cold vs warm start, dual-simplex 0-iteration confirmations, invalid bases under dual simplex with WRN/ERR streams linked, dual iteration-limit SUPBAS entries, time-limit key histogram, write-mapping variants on model C, INF key, equal bounds through setters. Output `probe2_out*.txt` |
| `probe3.cpp` | write-side convention under primal simplex (primal point MOSEK builds from the keys), MOSEK log for invalid bases, presolve during a warm start. Output `probe3_out.txt` |
| `probe4.cpp` | keys read through the already declared `MSK_getsolution` equal `MSK_getskc`/`getskx` (optimal C, optimal R, dual iteration limit); wrapper status after a 0-iteration primal / dual warm start. Output `probe4_out.txt` |
| `dl/` | PyPI wheels Mosek 11.1.11, 11.2.6, 10.2.19 (untrusted downloads, only `libmosek64.so.*` used through `MIPPP_MOSEK_LIBRARY`) |
| `capi.txt`, `releasenotes.txt` | pdftotext of the vendor manuals, for grepping |

Build: `flock .../lock0 /opt/gcc-15/bin/g++ -std=c++26 -O0 -I.../include -DMIPPP_INCLUDE_MOSEK_HEADER=1 -I$MOSEK_HOME/h probeN.cpp -o probeN -ldl`.
Run: `MIPPP_NO_VERSION_WARNING=1 timeout -k 5 60 ./probeN` (each model has `set_time_limit(10s)`).

Models used: **C** = 9 rows / 12 columns, built so the optimum (obj 28) has a
tight `<=` row, a tight `>=` row, an `==` row, a ranged row tight at its upper
side and one at its lower side (ranged through `set_constraint_lower_bound` on
a `<=` row, since mosek has no `add_ranged_constraint`), a `<=` row and a `>=`
row with a negative coefficient (`-y6 <= -4`, `-y7 >= -8`), two slack rows, a
column at its upper bound (`[0,4]`), a fixed column (`lb == ub == 2`), a column
at its lower bound, a column `(-inf, 7]` at 7, and an empty free column with
zero cost. **R** = random dense LP, 21 rows (17 `<=`, 3 `>=`, 1 `==` holding a
free column) x 31 columns (6 boxed `[0,3]`, 1 free), seed 12345, optimum
170.594075; cold primal simplex needs 15 iterations, cold dual 24 (presolve OFF).

## A. Wrapper facts

- Class that would get basis support: `mosek_lp` only
  (`include/mippp/solvers/mosek/impl/v1/mosek_lp.hpp:15`,
  `class mosek_lp : public mosek_base`). `mosek_milp` is excluded by the
  MILP-only ruling; there is no MOSEK QP class (`mosek/all.hpp:9-11` exports
  `mosek_api`, `mosek_lp`, `mosek_milp`).
- Bases: `mosek_base : protected model_base<int, double>` (`mosek_base.hpp:39`).
  Not a `remapping_model_base`. `native_id(variable) == v.id()`,
  `native_id(constraint) == c.id()` (`mosek_base.hpp:181-182`). No
  `remove_variable` / `remove_constraint` in the MOSEK wrapper (grep of
  `solvers/mosek` finds none), so handle uid == native index and ids are
  `[0, count)` (`model_base.hpp:63-78`). A snapshot is a plain index-keyed
  table sized `num_variables()` / `num_constraints()`; `_var_handle` /
  `_handle_id_bound` are not needed.
- Constraint handle -> native row: the row index itself. Rows are appended at
  `num_constraints()` (`mosek_base.hpp:392-406`, `450-476`); bounds are stored on
  the row **activity** with a bound key UP / LO / FX and `blc = buc = rhs`
  (`mosek_base.hpp:108-113`, `401-404`, `469-471`). Ranged rows exist only
  through `set_constraint_lower/upper_bound` -> `MSK_chgconbound`
  (`mosek_base.hpp:508-513`); MOSEK then reports bound key RA (MEASURED, dumps
  of model C), or FX when both bounds become equal (MEASURED, probe2 (h)).
  Columns: appended with `MSK_appendvars` then `MSK_putvarbound`
  (`mosek_base.hpp:250-269`, `270-304`).
- `solve()` (`mosek_lp.hpp:110-119`): resets the status, calls
  `MSK_optimizetrm` (solve + termination code), then
  `_solution = _pick_continuous_solution()` (`mosek_base.hpp:76-101`: the slot
  with the higher-ranked solsta, BAS on a tie, ITR when BAS is undefined) and
  maps trm/prosta/solsta to the status (`mosek_lp.hpp:50-95`). No reload.
  `mosek_lp` sets no optimizer, presolve or basis-identification parameter
  (ctor `mosek_lp.hpp:17-18`; `mosek_base` ctor sets only `MSK_IPAR_LOG = 0`,
  `mosek_base.hpp:135-137`), so MOSEK defaults apply: `MSK_IPAR_OPTIMIZER =
  FREE` (MEASURED value 2), which for an LP is the interior-point optimizer
  (DOCUMENTED capi.pdf p.220 "The interior-point optimizer is used by default";
  MEASURED 6 IPM iterations on R), `MSK_IPAR_INTPNT_BASIS = ALWAYS` (MEASURED 1;
  i.e. basis identification/crossover always runs), `MSK_IPAR_PRESOLVE_USE =
  FREE` (MEASURED 2), `MSK_IPAR_SIM_HOTSTART = FREE` (MEASURED 1),
  `MSK_IPAR_SIM_MAX_ITERATIONS = 10000000` (MEASURED). `mosek_milp` sets
  `MSK_OPTIMIZER_MIXED_INT` (`mosek_milp.hpp:22-25`). On branch
  `fix/review-rulings`, `solve()` calls `reset_status()` instead of the two
  resets (git diff), same behaviour.
- Pattern of `get_dual_solution` (`mosek_lp.hpp:131-139`):
  `make_unique_for_overwrite<double[]>(num_constraints())`, one bulk
  `MSK_getsolution(task, _require_solution(), ..., y, ...)` with nullptr for
  every other output, `return constraint_mapping(std::move(array))`.
  `get_reduced_costs` (`mosek_lp.hpp:140-147`): `MSK_getreducedcosts(task,
  slot, 0, numvar, ptr)` -> `variable_mapping`. `_require_solution()` throws
  `solver_error("MOSEK holds no solution")` when no slot was picked
  (`mosek_lp.hpp:96-99`). That slot may be ITR, so a basis getter must not reuse
  it; it must read `MSK_SOL_BAS`.
- Versions: `mosek_api.hpp:453-457`: key `"MOSEK"`, `library_names =
  {"mosek64"}`, `validated_versions = {solver_version_range{{11,0},{11,3}}}`
  (half-open, so 11.0-11.2). No `supported_versions` for MOSEK, neither on
  `main` nor on `fix/review-rulings` (`git grep`: only HiGHS declares one,
  `highs_api.hpp:366` on that branch). No optional-symbol mechanism in
  `mosek_api`: every `MOSEK_FUNCTIONS` entry is bound with `lib.get_function`
  (`mosek_api.hpp:441-445`, `460-462`) and is required. The HiGHS precedent for
  optional symbols (on `fix/review-rulings` only): a separate
  `HIGHS_OPTIONAL_FUNCTIONS` list bound with `lib.find_function`, a nullptr check
  and `feature_unavailable_error` in the member (`highs_api.hpp:336-346`,
  `highs_base.hpp:748-765` on that branch); `feature_unavailable_error` itself
  exists only on that branch (`utility/solver_exceptions.hpp:20` there).
- `mosek_api::_check` (`mosek_api.hpp:469-478`) throws `license_error` /
  `solver_error` on any nonzero code, warnings included. No basis call measured
  here ever returned a warning code, so `check()` is usable as is (MEASURED).
- The api header already declares `MSKstakeye` with the vendor values
  (`mosek_api.hpp:209-217` == `mosek.h:1795-1801`), `MSK_getsolution`
  (`mosek_api.hpp:235-240`, list `:437`), `MSK_solutiondef` (`:219-220`, `:435`),
  `MSK_getsolsta`, `MSK_deletesolution` (`:241`, `:438`). `MSKiparame` holds only
  `MSK_IPAR_LOG = 34` and `MSK_IPAR_OPTIMIZER = 110` (`mosek_api.hpp:141`).
  `MIPPP_INCLUDE_MOSEK_HEADER` switches to the vendor header
  (`mosek_api.hpp:9-11`); the probes build that way.

## B. Native API

Read (bulk, per solution slot):
- `MSKrescodee MSK_getskc(MSKtask_t task, MSKsoltypee whichsol, MSKstakeye * skc)`
  (`mosek.h:4367-4371`; capi.pdf p.378) - NOT declared in the api.
- `MSKrescodee MSK_getskx(MSKtask_t task, MSKsoltypee whichsol, MSKstakeye * skx)`
  (`mosek.h:4387-4391`; p.379) - NOT declared.
- Slices `MSK_getskcslice` / `MSK_getskxslice(task, whichsol, first, last, out)`
  (`mosek.h:4373-4379`, `4393-4399`) - NOT declared.
- `MSK_getsolution(task, whichsol, prosta*, solsta*, skc*, skx*, skn*, xc*, xx*,
  y*, slc*, suc*, slx*, sux*, snx*)` - ALREADY declared
  (`mosek_api.hpp:235-240`, `:437`) and accepts nullptr for unwanted outputs
  (the wrapper does so). Reading therefore needs no new symbol:
  `getsolution(task, MSK_SOL_BAS, nullptr, nullptr, skc, skx, nullptr...)`
  returns arrays identical to `getskc`/`getskx` (MEASURED, probe4: optimal C,
  optimal R, dual iteration limit; 22000 on an unsolved task).

Write:
- `MSKrescodee MSK_putskc(MSKtask_t task, MSKsoltypee whichsol, const MSKstakeye * skc)`
  (`mosek.h:5503-5507`; p.450) - NOT declared, must be added.
- `MSKrescodee MSK_putskx(MSKtask_t task, MSKsoltypee whichsol, const MSKstakeye * skx)`
  (`mosek.h:5517-5521`; p.450-451) - NOT declared, must be added.
- Slices `MSK_putskcslice` / `MSK_putskxslice(task, whichsol, MSKint32t first,
  MSKint32t last, const MSKstakeye *)` (`mosek.h:5509-5515`, `5523-5529`; p.450)
  - NOT declared.
- `MSK_putsolution` (all arrays incl. keys; `mosek.h:5573-5587`; p.453) - NOT
  declared; not needed (keys alone suffice, see E).
- Availability: `MSK_solutiondef(task, MSK_SOL_BAS, &isdef)` and `MSK_getsolsta`
  already declared. Diagnostics only: `MSK_getintinf(task, MSKiinfiteme,
  MSKint32t *)` (`mosek.h:3949-3953`) for `MSK_IINF_SIM_PRIMAL_ITER` (128),
  `SIM_DUAL_ITER` (121), `INTPNT_ITER` (17), `SIM_PRIMAL_HOTSTART` (125),
  `SIM_DUAL_HOTSTART` (118) - NOT declared.
- Parameters a warm start needs (if the wrapper switches the algorithm): the
  already declared `MSK_IPAR_OPTIMIZER` (110) with `MSK_OPTIMIZER_DUAL_SIMPLEX`
  (1), `FREE_SIMPLEX` (3), `PRIMAL_SIMPLEX` (8) (already in
  `mosek_api.hpp:142-152`). Optional knobs, not declared: `MSK_IPAR_SIM_HOTSTART`
  (145), `MSK_IPAR_SIM_MAX_ITERATIONS` (147), `MSK_IPAR_PRESOLVE_USE` (121),
  `MSK_IPAR_INTPNT_BASIS` (17) (`mosek.h:747`, `851`, `875`, `877`). Parameter
  indices are release-series specific: the 11.x numbering passed to 10.2.19 made
  `putintparam(121, 2)` fail with `MSK_RES_ERR_PARAM_IS_TOO_LARGE` (MEASURED).

Release availability (MEASURED, `nm -D --defined-only`): `MSK_getskc`,
`getskx`, `getskcslice`, `getskxslice`, `putskc`, `putskx`, `putskcslice`,
`putskxslice`, `putsolution`, `getsolution`, `solutiondef`, `getintinf`,
`optimizetrm` are exported by libmosek64 10.2.19 (`@@VERS_10.2`), 11.0.14
(`@@VERS_11.0`), 11.1.11 (`@@VERS_11.1`) and 11.2.6 (`@@VERS_11.2`). PyPI lists
no 11.3 yet (latest 11.2.6, MEASURED `pip index versions Mosek`). So every
symbol exists across the whole validated range 11.0-11.2: bind them as required
symbols, no `feature_unavailable_error`. Behaviour is identical too: `probe`
and `probe2` output on 11.1.11 and 11.2.6 is byte-identical to 11.0.14 apart
from the library line (MEASURED, `diff`). (10.2 outputs are meaningless for
behaviour because of the parameter numbering, symbols only.)

Status codes (DOCUMENTED capi.pdf p.612, `mosek.h:1795-1801`; .sol table p.678):
`MSK_SK_UNK = 0` "status unknown", `MSK_SK_BAS = 1` "in the basis",
`MSK_SK_SUPBAS = 2` "super basic", `MSK_SK_LOW = 3` "at its lower bound",
`MSK_SK_UPR = 4` "at its upper bound", `MSK_SK_FIX = 5` "fixed" (sol file: lower
limit identical to upper limit), `MSK_SK_INF = 6` "infeasible in the bounds"
(lower limit greater than upper limit).

Error codes seen (MEASURED): `MSK_RES_ERR_UNDEF_SOLUTION` (22000) when reading
keys of an undefined slot; `MSK_RES_ERR_INV_SKX` (1268) from `putskx` for an
out-of-range code value (7) - nothing of that array is stored.
`MSK_RES_ERR_INV_SKC` (1267) is the row counterpart (DOCUMENTED `mosek.h:1375`,
INFERRED to behave the same). `MSK_RES_WRN_TOO_FEW_BASIS_VARS` (400) /
`TOO_MANY` (405) exist (`mosek.h:1139-1140`) but were never returned.

## C. Row side convention (MEASURED)

Model C, identical codes under: wrapper default (FREE; presolve solved it,
0 iterations), primal simplex presolve OFF and FREE, dual simplex presolve OFF
and FREE (`probe_out.txt` section C):

| entity | bound key / bounds | value or activity | MOSEK key |
|---|---|---|---|
| row `y1 <= 3` | UP [-inf, 3] | 3 | UPR |
| row `y2 >= 2` | LO [2, inf] | 2 | LOW |
| row `y3 == 5` | FX [5, 5] | 5 | FIX |
| row `1 <= y4 <= 6` (tight up) | RA [1, 6] | 6 | UPR |
| row `1 <= y5 <= 6` (tight low) | RA [1, 6] | 1 | LOW |
| row `-y6 <= -4` | UP [-inf, -4] | -4 | UPR |
| row `-y7 >= -8` | LO [-8, inf] | -8 | LOW |
| slack rows `<= 100`, `>= -50` | | 11, 12 | BAS |
| col `x1 in [0,4]` | RA | 4 | UPR |
| col `x2` fixed 2 | FX | 2 | FIX |
| col `x4 >= 0` | LO | 0 | LOW |
| col `x5 in (-inf, 7]` | UP | 7 | UPR |
| empty free col, zero cost | FR | 0 | SUPBAS |
| y1..y7 | LO | >0 | BAS |

#BAS = 9 = #rows. The negative-coefficient rows settle it: a `<=` row whose
activity sits at its upper bound reads UPR and a `>=` row LOW whatever the sign
of the coefficients. **The row key names the side of the ACTIVITY a.x; there is
no slack flip.** DOCUMENTED: MOSEK's internal form is `Ax - x^c = 0`,
`l^c <= x^c <= u^c`, the basis is built from columns of `[A  -I]` and the
logical variable `x^c` is the activity itself (capi.pdf p.147, §9.1.1; p.207
eq. 12.1 "lower/upper limit on the activity for the constraints").

Writing uses the same convention (MEASURED, probe3, primal simplex
`SIM_MAX_ITERATIONS = 0`, reading back the primal point MOSEK built from the
keys): the exact keys give `xc = 3 2 5 6 1 -4 -8 11 12` (the optimum, obj 28);
writing LOW instead of UPR on the ranged row puts its activity at 1 (obj 23,
1 primal iteration to repair); writing the slack-flipped keys (every row LOW
<-> UPR) starts at obj 10 and needs 4 primal iterations. So a library that
wrote slack-side keys would still get the right answer, only a worse start.

Read mapping (rows and columns alike):
- `MSK_SK_BAS` -> `basis_status::basic`
- `MSK_SK_LOW` -> `basis_status::nonbasic_at_lower_bound`
- `MSK_SK_UPR` -> `basis_status::nonbasic_at_upper_bound`
- `MSK_SK_FIX` -> `basis_status::nonbasic_fixed` (seen only on FX entities after
  a solve: `==` rows, `lb == ub` columns, including bounds made equal through the
  wrapper setters - but see G4 for appended columns)
- `MSK_SK_SUPBAS` -> `basis_status::nonbasic_free` (a free nonbasic entity; also
  appears on bounded entities strictly between bounds after a dual-simplex
  iteration limit or in an unbounded certificate, see D)
- `MSK_SK_UNK`, `MSK_SK_INF` -> no tag: the slot holds no usable basis (throw).

Write mapping (all tags writable, MEASURED accepted with rc 0):
- `basic` -> `MSK_SK_BAS`; `nonbasic_at_lower_bound` -> `MSK_SK_LOW`;
  `nonbasic_at_upper_bound` -> `MSK_SK_UPR`; `nonbasic_fixed` -> `MSK_SK_FIX`;
  `nonbasic_free` -> `MSK_SK_SUPBAS`.
- MOSEK treats the side as advisory: a side that does not exist is snapped to
  the existing bound (UPR on a `+inf` upper bound, LOW on a `<=` row, LOW on a
  `(-inf, 7]` column: 0 iterations, OPTIMAL), a wrong side on a boxed entity
  costs a bound flip (free in dual simplex, 1 primal iteration), and FIX on a
  non-fixed column, LOW/UPR on a fixed column or `==` row, LOW/UPR/UNK/FIX on
  the free column, SUPBAS/INF/UNK on one nonbasic bounded column are all
  repaired with 0 iterations (probe2 (c), (f)). So `set_nonbasic(v, s)` snapping
  is harmless but not required on MOSEK.

Codes asked for: fixed column FIX; `==` row FIX; nonbasic free (empty free
column at 0) SUPBAS; basic BAS.

## D. Availability (MEASURED unless tagged)

- Before any solve: `solutiondef(BAS) = 0`; `getskc` / `getskx` return
  `MSK_RES_ERR_UNDEF_SOLUTION` (22000).
- After an optimal default solve (IPM + basis identification): BAS defined,
  solsta OPTIMAL, complete keys, #BAS == #rows (C 9/9, R 21/21). Also when
  presolve alone solves the problem (C: 0 IPM and 0 simplex iterations) the BAS
  slot is defined and complete. The ITR slot also carries keys but they are not
  a basis: no BAS at all, every interior entity SUPBAS (C: #BAS 0, #SUPBAS 10).
  After a simplex solve ITR is undefined (`getskc(ITR)` -> 22000).
- Infeasible LP (FREE / primal / dual x presolve FREE / OFF): BAS defined,
  solsta PRIM_INFEAS_CER, complete keys, #BAS == #rows, only BAS/LOW.
  A column with `lb > ub` reads INF (probe2 (g)).
- Unbounded LP (same 6 settings): BAS defined, solsta DUAL_INFEAS_CER, #BAS ==
  #rows but SUPBAS keys appear (a bounded column at value 1; a row at its bound)
  - the keys describe the certificate.
- Iteration limit: primal simplex `SIM_MAX_ITERATIONS = 3` -> trm
  `MSK_RES_TRM_MAX_ITERATIONS`, solsta PRIM_FEAS, a proper basis (#BAS 21, rows
  BAS 17/LOW 1/UPR 2/FIX 1, columns BAS 4/LOW 27). Dual simplex -> DUAL_FEAS,
  #BAS 21 plus 4 SUPBAS (3 bounded columns at 6.3, 6.8, 8.4 strictly inside
  their bounds, and the free column): not a vertex basis. IPM
  `INTPNT_MAX_ITERATIONS = 2` with the default `INTPNT_BASIS = ALWAYS`: trm
  MAX_ITERATIONS, ITR UNKNOWN, yet the BAS slot is OPTIMAL with the reference
  objective (basis identification and its clean-up ran anyway); the wrapper
  picks BAS and reports `iteration_limit{has_solution = true}`.
- Time limit (1e-4 s on 400x600 with FREE or DUAL; 0.02 s on 800x1200 DUAL):
  trm `MSK_RES_TRM_MAX_TIME`, BAS **defined** but solsta UNKNOWN and every key
  UNK (or 1 BAS + 1 FIX + the rest UNK). Defined does not mean available.
  All three stops happened before the first pivot (0 iterations of anything);
  a time limit firing mid-simplex was not reached here and is INFERRED to leave
  keys like the iteration-limit case (primal: a basis; dual: SUPBAS entries).
- After a modification without re-solve, MOSEK keeps the keys:
  `set_variable_upper_bound` -> solsta UNKNOWN; `set_constraint_upper_bound` ->
  DUAL_FEAS; `set_objective_coefficient` -> UNKNOWN; `add_constraint` -> slot
  extended, new row BAS; `add_variable` -> slot extended, new column **FIX**
  although its bound key is LO (DOCUMENTED cause: `MSK_appendvars` "Appended
  variables will be fixed at zero", capi.pdf p.287, and the wrapper sets bounds
  after appending); after a re-solve that column reads LOW.
  `reset_status()` does not touch MOSEK's slots (BAS still OPTIMAL).
- Interior point without basis identification (`MSK_IPAR_INTPNT_BASIS = NEVER`,
  only reachable through the native handle): BAS undefined, `getskc(BAS)` ->
  22000, wrapper picks ITR.
- #basic == #rows: holds in every optimal, infeasible, unbounded and
  iteration-limited result measured (MEASURED); not documented as a guarantee
  (INFERRED it holds for every simplex/BI result). After a put, MOSEK stores
  whatever was written (#BAS 22 or 52 read back verbatim).
- Native flag: `MSK_solutiondef(task, MSK_SOL_BAS)` (also `getsolsta`,
  `MSK_IINF_SOL_BAS_SOLSTA`). Not sufficient alone: the slot is defined after a
  time-limit stop with UNK keys, and `putskc`/`putskx` on an unsolved model
  define it (solsta UNKNOWN). Key availability on: slot defined AND no UNK/INF
  key (equivalently #BAS == #rows with no UNK).

## E. Warm start (MEASURED on R)

- `putskx` + `putskc` (keys only, no values) on a fresh identical model B:
  rc 0, the BAS slot becomes defined (solsta UNKNOWN), read-back equals written.
- Wrapper defaults (`OPTIMIZER = FREE`): the basis is ignored; IPM runs 6
  iterations then basis identification, result optimal, log shows no hotstart
  line. Same with `OPTIMIZER = INTPNT`; `SIM_MAX_ITERATIONS = 0` is irrelevant.
  DOCUMENTED: the IPM "cannot perform warm-start" (capi.pdf p.220).
- `OPTIMIZER = DUAL_SIMPLEX`, `SIM_MAX_ITERATIONS = 0` (the knob used): trm OK,
  solsta OPTIMAL, 0 dual iterations, `MSK_IINF_SIM_DUAL_HOTSTART = 1`, objective
  = reference, with presolve OFF, FREE or ON. Log: "Optimizer - hotstart : yes",
  "Num. basic : 21  Basis rank : 21". With presolve FREE the warm start skips
  presolve; with ON it presolves (20x30) and still hot-starts with 0 iterations.
- `OPTIMIZER = PRIMAL_SIMPLEX` or `FREE_SIMPLEX` (which picked primal here),
  `SIM_MAX_ITERATIONS = 0`: trm `MSK_RES_TRM_MAX_ITERATIONS`, solsta UNKNOWN,
  but the point is the optimum (objective = reference): an iteration-limit stop
  holding the optimal point. With `SIM_MAX_ITERATIONS = 1` (or no limit):
  OPTIMAL with 0 iterations.
- `MSK_IPAR_SIM_HOTSTART`: FREE (default) and STATUS_KEYS both use the keys;
  NONE ignores them (cold objective 20.25 at 0 iterations). Status keys alone
  suffice (no `xx`/`xc` written).
- Wrapper `solve()` after the put with `OPTIMIZER = DUAL_SIMPLEX`,
  `SIM_MAX_ITERATIONS = 0` set natively: status `optimal`.
- Re-solving the same task with a simplex optimizer hot-starts automatically
  from the stored BAS slot (0 iterations, hotstart flag 1).
- Settings that make MOSEK use a written basis: `MSK_IPAR_OPTIMIZER` must be a
  simplex optimizer and `MSK_IPAR_SIM_HOTSTART` must not be NONE (default FREE
  is fine). Presolve, crash, `SIM_HOTSTART_LU` need no change; no factorisation
  call. The wrapper's defaults are NOT sufficient (FREE -> IPM).
- Basis written after changing a bound: kept. Relax an inactive bound, then put:
  read-back equal, dual `maxit = 0` -> OPTIMAL 0 iterations (also primal maxit=0
  -> limit stop at the optimum). Put, then change the bound: keys survive the
  change. Tighten an active row, then put: dual simplex hot-starts and needs 12
  iterations (cold 24).
- Invalid bases: `putskc`/`putskx` return 0 for every structurally invalid key
  array tried (one basic too few, one too many, all BAS, all UNK, UPR on a `+inf`
  bound, LOW on a basic free column, SUPBAS/FIX/INF/UNK on a nonbasic bounded
  column, a singular basis). Only an out-of-range enum value (7) is rejected
  (`MSK_RES_ERR_INV_SKX`, nothing stored). Nothing is printed on the WRN or ERR
  streams. The next simplex solve repairs silently (log: "Num. basic : 20  Basis
  rank : 20" for one too few) and always ends at the correct optimum - no error,
  no crash, no wrong answer. Cost: 0 iterations (side snaps, single
  SUPBAS/FIX/INF/UNK), 1 dual iteration (count off by one), 25 dual (all UNK or
  slack basis, ~ cold 24), 40 primal (singular). Under `SIM_MAX_ITERATIONS = 0`
  the ones needing an iteration stop with `TRM_MAX_ITERATIONS` (status
  iteration_limit), never a false optimal.

## F. Partial / extended bases (MEASURED)

- Appending rows/columns extends the stored keys automatically: new row BAS,
  new column FIX (see D). A dual simplex `maxit = 0` re-solve of the grown model
  without any put: OPTIMAL, 0 iterations; the new column then reads LOW.
- Explicit extension: old keys + new column LOW + new row BAS written with
  full-size `putskx`/`putskc` -> dual `maxit = 0` OPTIMAL, 0 iterations. (The
  reverse - new column BAS, new row UPR - was also repaired at 0 iterations, but
  only because that new column was empty, hence singular.)
- Full vs subset: `putskc`/`putskx` take full arrays of length
  `numcon`/`numvar` with no length argument (DOCUMENTED signature, p.450); the
  slice functions write `[first, last)` (p.450). On a fresh slot the entries not
  written read back UNK (`putskx` only: all 21 rows UNK; half the columns via
  `putskxslice`: 16/16 unset columns UNK) and are treated as unknown: R with half
  the column keys unknown needed 18 primal iterations vs 15 cold - no gain. On a
  defined slot a one-entry slice replaces only that entry (#BAS became 22, no
  validation).

## G. Gotchas

1. Default LP algorithm is the interior point: `set_basis` is inert and
   re-solves never hot-start unless `solve()` runs a simplex optimizer.
2. Read the basis from `MSK_SOL_BAS`, never from the `_solution` slot (ITR when
   basis identification is off or ranks higher) and never from ITR (its keys
   mark interior entities SUPBAS and hold no BAS).
3. A defined BAS slot is not a basis: time-limit stops leave UNK keys, and a put
   defines the slot. Check for UNK/INF.
4. Columns appended after the last solve read FIX (MOSEK fixes appended
   variables at zero before the wrapper's `putvarbound`); appended rows read
   BAS. Verbatim decoding would report `nonbasic_fixed` for a non-fixed column.
5. After model changes MOSEK keeps the old keys (solsta UNKNOWN / DUAL_FEAS):
   a getter keyed on the native slot returns the last solve's basis.
6. `reset_status()` leaves MOSEK's BAS slot in place (still OPTIMAL);
   `MSK_deletesolution` (already declared) would clear it.
7. Dual-simplex iteration-limit and unbounded results contain SUPBAS on bounded
   entities; IPM iteration-limit with default BI can still yield an OPTIMAL BAS.
8. MOSEK never rejects a structurally invalid basis and prints no warning; only
   enum values outside 0..6 fail. Under the "misuse is documented UB" ruling
   nothing needs guarding, but no `solver_error` will ever surface for misuse.
9. With `SIM_MAX_ITERATIONS = 0` the primal simplex reports an iteration limit
   even from an optimal basis; the dual simplex reports optimal. A shared
   "0 iterations -> optimal" test needs the dual simplex on MOSEK (mosek_lp has
   no iteration-limit setter in any case).
10. After a 0-iteration primal stop at the optimum (solsta UNKNOWN) the wrapper
    reports `iteration_limit` with `solution_available == false`
    (`mosek_lp.hpp:42-48`, `83`) while `get_solution_value()` returns the optimum
    170.594075 (MEASURED, probe4); the dual path reports `optimal`. Another
    reason for the dual simplex in a warm start and in a shared test.
11. The hotstart info flag is 1 even for an all-UNK basis: not proof of reuse.
12. Parameter indices differ between release series (10.2 vs 11.x); any new
    `MSK_IPAR_*` value in the non-header branch is valid for 11.x only.
13. Threads: a task may be shared between threads only one at a time
    (DOCUMENTED capi.pdf p.161). Callbacks must not call any MOSEK function
    (DOCUMENTED p.125), so no basis get/set from a callback; `mosek_lp` exposes
    no callback (INFERRED from the api: `MSK_putcallbackfunc` is declared but not
    bound in `MOSEK_FUNCTIONS`).
14. No lazy updates: task edits take effect immediately; the BAS slot follows
    appends at once (MEASURED D/F).

## H. Recommendation

- Status variants (only what MOSEK reports, never bare `nonbasic`):
  - columns: `std::variant<basis_status::nonbasic_at_lower_bound,
    basis_status::basic, basis_status::nonbasic_at_upper_bound,
    basis_status::nonbasic_fixed, basis_status::nonbasic_free>`
  - rows: `std::variant<basis_status::basic,
    basis_status::nonbasic_at_lower_bound, basis_status::nonbasic_at_upper_bound,
    basis_status::nonbasic_fixed, basis_status::nonbasic_free>`
  Alternative 0 is what `handle_status_table` returns for an id past the bound
  (IIS convention); LOW for a column and BAS for a row are exactly the
  extension MOSEK accepts at 0 iterations (F). If the maintainer prefers one
  variant for both, put `basic` first and let set_basis fill unknown columns
  with LOW explicitly.
- Read mapping: as in C (UNK/INF -> no tag).
- `get_basis()`: require a solve (`_solution` set, so it throws after
  `reset_status()` like `get_solution()`), then `solutiondef(MSK_SOL_BAS)`, else
  `solver_error("MOSEK holds no basis")`; one bulk read through the already
  declared `MSK_getsolution(task, MSK_SOL_BAS, ..., skc, skx, ...)` (or the
  added `MSK_getskc`/`getskx`); any UNK or INF key -> `solver_error`; translate by
  index into two `handle_status_table`s sized by the row/column counts **at the
  last solve** (store them in `solve()`), so appended entities read alternative 0
  rather than MOSEK's FIX artefact. Alternative: size by current counts and
  decode FIX on a non-FX bound key as LOW (needs a bound read).
- `set_basis(b)`: build full `skx`/`skc` arrays of the current size from `b`
  (ids past `b`'s bound -> alternative 0, i.e. column LOW / row BAS), write with
  `putskx` then `putskc` on `MSK_SOL_BAS` through `check()` (new required
  symbols; present in every validated release). Then make the next `solve()`
  run a simplex: recommended a one-shot flag that switches `MSK_IPAR_OPTIMIZER`
  to `MSK_OPTIMIZER_DUAL_SIMPLEX` for that solve and restores the previous value
  (dual: optimal at 0 iterations from an optimal basis, and DOCUMENTED capi.pdf p.220 as
  faster on average). No presolve, hot-start or factorisation change needed.
- Deviations from the drafted concepts / patterns this backend forces:
  1. `set_basis` must change the algorithm of the next solve (the default IPM
     cannot warm-start) - a setter with an algorithmic side effect, or a change
     of mosek_lp's default to a simplex optimizer (affects every LP solve).
  2. `nonbasic_free` doubles as MOSEK's SUPBAS, which after a dual-simplex
     iteration limit or in an unbounded certificate marks bounded entities
     strictly between their bounds - not "free". Accept the wider meaning
     ("nonbasic, not at a bound") or add a superbasic tag.
  3. Availability is not a native flag: the getter must inspect the keys
     (UNK/INF) and throw; after a time-limit stop there is no basis.
  4. Entities appended after the last solve need wrapper-side bookkeeping
     (solve-time counts) to avoid reporting MOSEK's FIX artefact.
  5. MOSEK silently repairs any structurally invalid written basis (wrong
     counts, wrong or missing sides, SUPBAS/FIX/INF/UNK) and returns no error,
     so set_basis can never report misuse; nothing to guard under the UB ruling.
  6. Sides on writes are advisory on MOSEK (snapped/flipped by the solver);
     `set_nonbasic(v, s)` snapping costs nothing but buys nothing here.
  7. No tag is unwritable, no side is unnameable (activity convention, no flip),
     and the basis is available after the default algorithm (BI = ALWAYS) - none
     of these forces a deviation.

## Open questions

- Which variant order / one-or-two variants (H).
- One-shot dual-simplex switch in `solve()` vs a persistent default change for
  mosek_lp vs leaving `set_basis` inert unless the user picks a simplex
  natively.
- Whether `nonbasic_free` may carry MOSEK's superbasic-between-bounds meaning or
  a `superbasic` tag is added.
- Whether get_basis should report entities appended after the last solve as
  alternative 0 (solve-time counts) or decode MOSEK's current keys.
- `MSK_putskc` etc. become required symbols; if a `supported_versions` is ever
  declared for MOSEK below 10.2, re-check (they exist in 10.2.19 already).

## Addendum (implementation, 2026-10-09)

- Line 257, more precisely: after an unbounded solve, a row read SUPBAS has a
  certificate activity at its model lower bound but strictly inside its
  certificate bound 0 (capi.pdf section 12.1.2); the keys describe the
  certificate, not the model.
- The one-shot uses MSK_OPTIMIZER_FREE_SIMPLEX: over 600 column-generation
  cases (impl/sweep.cpp, impl/sweep_out_11.0.txt: 60 shapes x 10 seeds),
  10422 simplex iterations against 13577 for the dual simplex, fewer in 431
  cases and more in 109. The 0-iteration transfer of an optimal basis and the
  counts on 11.1.11 and 11.2.6 are in impl/choice.cpp and impl/choice_out_*.
