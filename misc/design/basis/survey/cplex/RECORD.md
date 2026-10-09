# CPLEX backend (cplex_lp): LP basis survey

Tags: **MEASURED** = a probe I ran; **DOCUMENTED** = vendor header or manual (path:line or file); **INFERRED** = reasoning only.

Environment
- CPLEX 22.1.2 Community (default): `/home/plaiseek/Softwares/solvers/cplex-community/cplex/bin/x86-64_linux/libcplex2212.so`
- CPLEX 22.1.1 (pinned with `MIPPP_CPLEX_LIBRARY=/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/libcplex2211.so`)
- Every probe ran on both releases. The outputs are identical except the iteration counts of the wall-clock time-limit probe (MEASURED: `diff out_*_2212.txt out_*_2211.txt`).
- Repo state: local `main` = 04431ca. It is 13 commits behind `origin/main`, which adds:
  - `reset_status()` at the top of `cplex_lp::solve()`;
  - the two-tier `supported_versions` / `feature_unavailable_error` mechanism.

  CPLEX declares no `supported_versions` there.

Probe files, all in this directory:
- `probe_basis.cpp`: sections C, D, E, F; run as `./probe_basis C|D|E|F`.
- `probe_basis2.cpp`: P1-P10.
- `probe_timelimit.cpp`: P8 with mid-simplex time limits.
- `html2txt.py`: extracts the HTML docs.
- Outputs: `out_{C,D,E,F,P2,TL}_{2212,2211}.txt`.

All probes build their models with MIP++ (`cplex_lp`). They reach `CPXgetbase`, `CPXcopybase`, `CPXcopystart`, `CPXgetitcnt`, `CPXgetmethod`, `CPXbaropt` and `CPXgetslack` by `dlopen(api.library_path(), RTLD_NOLOAD)` and `dlsym`. Each solve has a finite time limit.

---------------------------------------------------------------------------

## A. Wrapper facts

### Classes and bases
- Basis support would go on **`cplex_lp` only** (`include/mippp/solvers/cplex/impl/v1/cplex_lp.hpp:19`, `class cplex_lp : public cplex_base`).
- `cplex_milp` (`cplex_milp.hpp:24`) is excluded by the *_milp ruling.
- There is no `cplex_qp` class.
- `cplex_base` (`cplex_base.hpp:42`) derives `protected remapping_model_base<int, double>`, which derives `protected model_base<int, double>`.

### Variable ids are remapped
- `remapping_model_base` (`remapping_model_base.hpp:16`) is the base.
- `_native_id(v)` is at `remapping_model_base.hpp:40` and `_var_handle(native)` at `:44`. Both are the identity until the first removal.
- `_handle_id_bound(num_native)` (`:90`) is `_native_ids_map.size()` once ids are remapped.
- Variables can be removed:
  - `remove_variable` / `remove_variables` are at `cplex_base.hpp:349-362`.
  - They delete immediately through `_lazily_remove_variables` (`:206-245`, `CPXdelsetcols`), so nothing stays pending.
  - The method is "lazily" in name only.

### Constraints are never removed
- No `remove_constraint` exists on CPLEX (grep).
- A constraint handle id **is** the native row index: `native_id(constraint c)` returns `c.id()` (`cplex_base.hpp:130`).
- New rows get `constraint(num_constraints())` (`:435`, `:497-512`).
- There is no `add_ranged_constraint`. A ranged ('R') row is made by `set_constraint_lower_bound` / `set_constraint_upper_bound` on an inequality (`_set_row_sides`, `:553-579`):
  - always `rhs = lower`, `rngval = upper - lower >= 0`;
  - two equal sides give 'E';
  - two infinite sides give 'L' with `rhs = 1e20` (a "free row").

### solve()
- `cplex_lp::solve()` = `CPXprimopt` only (`cplex_lp.hpp:112-115`). That is primal simplex whatever `CPXPARAM_LPMethod` says, so there is no barrier and no crossover.
- Presolve is CPLEX's default: `PREIND=1`, `ADVIND=1` (MEASURED defaults: `ADVIND=1 PREIND=1 LPMETHOD=0 BARCROSSALG=0 ITLIM=2147483647`).
- There is no reload: CPLEX warm-starts a re-solve from its resident basis (MEASURED F2: re-solve after adding a row and a column took 1 iteration, against 100 cold).
- On `origin/main`, solve() first calls `reset_status()`.
- Related members:
  - `refine_lp_status()` (`:116-139`) re-solves with `CPXPARAM_Advance=0` and `Preprocessing_Reduce=0` (restored by a `restore_guard`), but only when the status is `infeasible_or_unbounded`.
  - `compute_iis()` (`:143-146`) resets the status.
  - `cplex_milp::solve()` (`cplex_milp.hpp:375`) uses `CPXlpopt` on an LP-typed problem.

### Pattern of the existing getters
- `get_dual_solution` (`cplex_lp.hpp:163-169`) and `get_reduced_costs` (`:170-179`) both:
  - allocate a buffer indexed by native index;
  - fill it with `CPXsolution`;
  - wrap it in a lazy `constraint_mapping` / `variable_mapping`, whose lambda translates `_native_id(v)` at **access** time (so it is not stable across a later removal).
- The IIS snapshot is the pattern to mirror instead (`cplex_base.hpp:719-816`):
  - taken by value;
  - `detail::iis_answer` builds `handle_status_table`s sized `_handle_id_bound(num_col)` for variables and `num_row` for rows (`:722`);
  - native column `j` is keyed by `_var_handle(j).uid()` (`:811`);
  - row `i` is keyed by `i`.
- `handle_status_table` (`detail/handle_status_table.hpp`) returns alternative 0 for ids past its bound.

### Versions and optional symbols
- `cplex_api.hpp:378-382`: `library_names = {"cplex2220","cplex2212","cplex2211","cplex2210"}` and `validated_versions = {[22.1.0, 22.2.1)}`. No `supported_versions` (also none on `origin/main`).
- Every CPLEX function is **required**: the `CPLEX_FUNCTIONS` X-macro (`cplex_api.hpp:297-363`) is built with `lib.get_function` (`:368-369`), which throws `symbol_not_found`.
- No CPLEX symbol is optional and there is no `probe_symbols`.
- The optional-symbol precedent is HiGHS on `origin/main`: `HIGHS_OPTIONAL_FUNCTIONS` + `lib.find_function` (nullptr when absent; `detail/dynamic_library.hpp:177`), followed by a nullptr check that throws `feature_unavailable_error` (`highs_base.hpp:750-765` on origin/main).
- The fallback declarations used when the vendor header is not included are at `cplex_api.hpp:12-287`, under `#if MIPPP_INCLUDE_CPLEX_HEADER ... #else`.
- **`CPXgetbase` / `CPXcopybase` / `CPXcopystart` and the `CPX_AT_LOWER`... constants are not declared** in the api header (grep).

---------------------------------------------------------------------------

## B. Native API

### Signatures (DOCUMENTED, `$CPLEX_HOME/include/ilcplex/cplex.h`)
```c
int CPXgetbase (CPXCENVptr env, CPXCLPptr lp, int *cstat, int *rstat);     // cplex.h:645
int CPXcopybase(CPXCENVptr env, CPXLPptr lp, int const *cstat, int const *rstat);  // cplex.h:371
int CPXcopystart(CPXCENVptr env, CPXLPptr lp, int const *cstat, int const *rstat,
                 double const *cprim, double const *rprim,
                 double const *cdual, double const *rdual);                 // cplex.h:433
int CPXgetitcnt(CPXCENVptr, CPXCLPptr);    // cplex.h:855   (diagnostics only)
int CPXgetmethod(CPXCENVptr, CPXCLPptr);   // cplex.h:911   (diagnostics only)
```

### Shape of the calls
- Bulk arrays only, over **all** columns and **all** rows.
- There is no per-entity or subset variant.
- `CPXgetbase` accepts `NULL` for either array (DOCUMENTED `refcallablelibrary/cpxapi/getbase.html`).
- `CPXcopybase` needs both arrays: `NULL` gives error 1004 `CPXERR_NULL_POINTER`. This was MEASURED even when `nrows == 0` (P6) and on an empty model.
- `CPXcopystart` with the basis only (the other arrays NULL) behaves like `CPXcopybase` (MEASURED P10).

### Codes (DOCUMENTED, `cpxconst.h:564-567`)
`CPX_AT_LOWER 0`, `CPX_BASIC 1`, `CPX_AT_UPPER 2`, `CPX_FREE_SUPER 3`.

| Array | Code | Documented meaning |
| --- | --- | --- |
| cstat | 0 | at lower bound |
| cstat | 1 | basic |
| cstat | 2 | at upper bound |
| cstat | 3 | free and nonbasic |
| rstat, non-ranged rows | 0 | the "slack, surplus or artificial variable is nonbasic at value 0.0" |
| rstat, non-ranged rows | 1 | basic |
| rstat, ranged rows | 0 | the slack is nonbasic at its lower bound |
| rstat, ranged rows | 1 | basic |
| rstat, ranged rows | 2 | the slack is nonbasic at its upper bound |

Sources: `/opt/ibm/ILOG/CPLEX_Studio2211/doc/html/en-US/refcallablelibrary/cpxapi/getbase.html`, Tables 1-3; the same tables are in `copybase.html` and `copystart.html`.

### Slack convention (DOCUMENTED, `CPLEX/UsrMan/topics/cont_optim/simplex/slackVariablesAndSolnValues.html`)
- CPLEX's standard form is `Ax + Es = b`, with `0 <= s <= r`:
  - 'L': `E_i = +e_i`, `r = inf`;
  - 'G': `E_i = -e_i`, `r = inf`;
  - 'E': `E_i = +e_i`, `r = 0`;
  - 'R': `E_i = -e_i`, `r = |range|`.
- `CPXgetslack` returns rhs − activity for non-ranged rows, and activity − rhs for ranged rows.

### Other relevant symbols and parameters
- Errors (DOCUMENTED, `cpxconst.h`): `CPXERR_NO_BASIS 1262` (:1714), `CPXERR_NO_SOLN 1217` (:1790), `CPXERR_NULL_POINTER 1004` (:1836).
- Parameters (DOCUMENTED, `cpxconst.h`):
  - `CPXPARAM_Advance` / `CPX_PARAM_ADVIND` 1001 (:2239, :2442);
  - `CPXPARAM_Simplex_Limits_Iterations` 1020 (:2413), already in the api;
  - `CPXPARAM_LPMethod` 1062 (:2270);
  - `CPXPARAM_Preprocessing_Presolve` 1030 (:2377);
  - `CPXPARAM_SolutionType` 1147 (:2426);
  - `CPXPARAM_Barrier_Crossover` 3018 (:2243);
  - `CPX_BASIC_SOLN 1` / `CPX_NONBASIC_SOLN 2` (:459-460).

### Release availability
- MEASURED with `nm -D`: `CPXgetbase`, `CPXcopybase`, `CPXcopystart`, `CPXgetitcnt`, `CPXgetmethod`, `CPXbaropt`, `CPXgetslack`, `CPXsolninfo`, `CPXgetbasednorms`, `CPXcopybasednorms` and `CPXreadcopybase` are exported by both `libcplex2211.so` and `libcplex2212.so`.
- 22.1.0 and 22.2.0 are not installed. INFERRED: present, because these routines are decades-old Callable Library basics with no deprecation note in the 22.1.1 reference.
- So `getbase` / `copybase` can go into the **required** `CPLEX_FUNCTIONS` list. No `feature_unavailable_error` is needed.
- They must also be added to the fallback declarations (`cplex_api.hpp:12-287`), together with the four status constants (and optionally `CPXERR_NO_BASIS`).

---------------------------------------------------------------------------

## C. Row side convention (MEASURED: probe_basis C1-C4, both releases)

### The test model
| Kind | Entities |
| --- | --- |
| Basic columns | y1=3, y2=1, y3=4, y4=4, y5=1, y6=1 |
| Nonbasic columns | u in [0,2] at upper; f1 = 3 fixed (cost +5 in a max); f2 = 2 fixed (cost −5); z free, no rows, cost 0; w in (−inf,5] at upper; v >= 0 at lower |
| Tight rows | R1 'L' `y1+y2<=4`; R2 'G' `y1−y2>=2`; R3 'E' `y3+f1==7` (π<0); R7 'E' `y6−f2==−1` (π>0); R4 'R' `1<=y4+u<=6` tight at upper; R6 'R' `1<=y5<=8` tight at lower |
| Basic rows | R5 'L' loose; R8 free row ('L', rhs 1e20) |

The optimum is objective 15, unique except for z.

### Results
- Every variant gave identical codes:
  - presolve on, where presolve solved the model with itcnt 0 (C1);
  - presolve off (C2);
  - dual simplex (C4);
  - the ranges rewritten natively with negative `rngval`: R4 rhs 6 / rng −5, R6 rhs 8 / rng −7 (C3).
- `#basic == #rows == 8` in all of them.

| Entity | Native code | Activity / value side |
| --- | --- | --- |
| R1 'L' tight | 0 AT_LOWER | activity at its **upper** side |
| R2 'G' tight | 0 AT_LOWER | activity at its lower side |
| R3, R7 'E' (both dual signs) | 0 AT_LOWER | fixed |
| R4 'R' tight at upper (rng 5 or −5) | 2 AT_UPPER | upper |
| R6 'R' tight at lower (rng 7 or −7) | 0 AT_LOWER | lower |
| R5 loose, R8 free row | 1 BASIC | |
| u at upper, w at upper | 2 AT_UPPER | upper |
| v at lower | 0 AT_LOWER | lower |
| f1 fixed, d>0 in the max | 2 AT_UPPER | fixed |
| f2 fixed, d<0 | 0 AT_LOWER | fixed |
| z free nonbasic (value 0) | 3 FREE_SUPER | none |
| basic columns | 1 BASIC | |

### Verdict: **mixed (slack side)**
- Columns name the side of their value.
- A row code names the side of CPLEX's logical variable s in `a·x + E_i s = b`. Activity is `rhs − s` on 'L' and `rhs + s` on 'G' / 'R'.
- So the code **flips relative to the activity on 'L' rows only**:
  - an 'L' row and a 'G' row, both tight, both read AT_LOWER;
  - 'R' rows name the activity side whatever the sign of the range value;
  - 'E' rows only ever read AT_LOWER.
- Translating therefore needs the row sense (`CPXgetsense`, bulk) on every get and set.

### Read mapping (native to tag; report the activity side for rows, the value side for columns)
Columns:
- `BASIC` gives `basic`.
- `FREE_SUPER` gives `nonbasic_free`.
- `AT_LOWER` or `AT_UPPER` with `lb == ub` gives `nonbasic_fixed`. This needs `CPXgetlb`/`CPXgetub`, because CPLEX picks the side of a fixed column by the sign of its reduced cost.
- Otherwise `AT_LOWER` gives `nonbasic_at_lower_bound` and `AT_UPPER` gives `nonbasic_at_upper_bound`.

Rows, all codes being `BASIC` → `basic` plus:
- 'L' with a finite rhs: `AT_LOWER` → `nonbasic_at_upper_bound`.
- 'G' with a finite rhs: `AT_LOWER` → `nonbasic_at_lower_bound`.
- 'E': `AT_LOWER` → `nonbasic_fixed`.
- 'R': `AT_LOWER` → `nonbasic_at_lower_bound` and `AT_UPPER` → `nonbasic_at_upper_bound`. A zero-range 'R' (native only, never made by the wrapper) → `nonbasic_fixed`.
- A free row ('L' with rhs >= 1e20, or 'G' with rhs <= −1e20): any nonbasic code → `nonbasic_free`. This is never produced by a solve; only a verbatim copybase can produce it.

Other codes (AT_UPPER/FREE_SUPER on L/G/E rows, values outside 0..3) exist only verbatim after a copybase of such codes (see E). A `set_basis` that writes only legal codes never produces them.

### Write mapping (tag to native)
Columns:
| Tag | Native |
| --- | --- |
| `basic` | BASIC |
| `nonbasic_at_lower_bound` | AT_LOWER |
| `nonbasic_at_upper_bound` | AT_UPPER |
| `nonbasic_fixed` | AT_LOWER (AT_UPPER is equally accepted; CPLEX re-sides a fixed column by its reduced cost at the next solve, MEASURED E5) |
| `nonbasic_free` | FREE_SUPER |
| bare `nonbasic`, if the design keeps one | finite lb → AT_LOWER, else finite ub → AT_UPPER, else FREE_SUPER. This is exactly what CPLEX itself gives new columns (MEASURED P4). |

Rows, by the current sense:
- `basic` → BASIC on every row.
- 'L': `nonbasic_at_upper_bound` → AT_LOWER.
- 'G': `nonbasic_at_lower_bound` → AT_LOWER.
- 'E': `nonbasic_fixed` → AT_LOWER (also `at_lower` / `at_upper`, which mean the same there).
- 'R': `nonbasic_at_lower_bound` → AT_LOWER and `nonbasic_at_upper_bound` → AT_UPPER.

**Tags with no native meaning** must be mapped to the row's only legal nonbasic code and documented as UB:
- `nonbasic_free` on any row;
- `nonbasic_at_lower_bound` on 'L';
- `nonbasic_at_upper_bound` on 'G';
- `nonbasic_fixed` on L/G/R;
- any nonbasic tag on a free row. Writing AT_LOWER there means activity = 1e20: MEASURED at itlim 0 to give objval 1e+20, then repaired at a full solve with 6 iterations.

For the record, CPLEX accepts all of these and repairs them silently at solve. MEASURED (E5): AT_UPPER on 'L', 'G', 'E' and FREE_SUPER on 'L' were all turned into AT_LOWER, with 0 extra iterations.

---------------------------------------------------------------------------

## D. Availability (MEASURED, probe_basis D1-D9 and probe_basis2 P2-P8)

| State | `CPXgetbase` | Notes |
| --- | --- | --- |
| Before any solve | 1262 "No basis exists." | getstat 0 |
| Optimal | 0 | #basic == #rows |
| Optimal, but presolve solved everything (C1, D8) | 0 | itcnt 0, method 1, full-size arrays in the original space |
| Infeasible, presolve on (**wrapper default**) | **1262** | getstat 3 (INFEASIBLE), no solution (getx 1217) |
| Infeasible, presolve off | 0 | slack basis, pfeas 0, getstat 3 |
| Unbounded, presolve on (default) | **1262** | getstat 4 (INForUNBD) |
| Unbounded, presolve off | 0 | getstat 2, pfeas 1, dfeas 0 |
| After `refine_lp_status()` on the INForUNBD case (P3) | 0 | getstat 2 |
| Iteration-limit stop (itlim 3, during phase 1) | 0 | getstat 10, pfeas 0, #basic == #rows |
| Time limit 0 s / 2 ms / 20 ms (before simplex starts) | **1262** | getstat 11, itcnt 0 |
| Time limit 100/200/300 ms mid-simplex on a 900x900 LP (P8) | 0 | getstat 11, pfeas 1, #basic == #rows |
| After `set_variable_upper_bound`, `add_constraint`, `add_variable`, `set_objective_coefficient` or `remove_variable` without a re-solve (D5) | **0** | The **resident** basis, kept and adjusted (next table). getstat 0, `CPXsolninfo` type 0, getx 1217: **no solution, but a basis**. |
| After `CPXcopybase`, before a solve (E2) | 0 | Returns **exactly what was written**, unvalidated: even codes 7 or −1 and 9 basics for 8 rows come back verbatim. getstat 0. |
| After `compute_iis()` on a feasible or an infeasible model (D9) | 0 | getstat 30/31, but the basis is kept: the re-solve takes 0 iterations |
| Native `CPXbaropt` (BARCROSSALG 0 or −1) | 1262 | soltype 2 (CPX_NONBASIC_SOLN). `CPXbaropt` did not cross over here. |
| Native `CPXlpopt` LPMETHOD=4 (barrier), SolutionType 0 | 0 | crossover ran; method reported 2 |
| Native `CPXlpopt` LPMETHOD=4, SolutionType 2 | 1262 | |
| Native `CPXlpopt` LPMETHOD=0 (auto) or 6 (concurrent) | 0 | dual simplex chosen |
| LP → MILP → LP problem-type switch (native `CPXchgprobtype`, P7) | 0 | basis kept, re-solve takes 0 iterations |

### How CPLEX adjusts the resident basis after modifications (MEASURED D5, P4, P5)
- A new row gets BASIC.
- A new column gets:
  - AT_LOWER if lb is finite (fixed columns included);
  - AT_UPPER if only ub is finite;
  - FREE_SUPER if it is free.
- A deleted column's entry is dropped.
- A column sitting AT_UPPER whose ub becomes +inf is re-sided at once to AT_LOWER.
- A sense change L→G keeps AT_LOWER, so its activity meaning flips.

### Answers
- `#basic == #rows`: true in every state reached through a solve or a modification (MEASURED). INFERRED to be the simplex invariant. It breaks only verbatim after an invalid copybase, until the next solve.
- Native "basis available" flag: **the return code of `CPXgetbase`** (0 vs `CPXERR_NO_BASIS` 1262).
- `CPXsolninfo` `solntype == CPX_BASIC_SOLN` is not a reliable key: it drops to 0 after any modification while a basis is still retrievable.
- The default wrapper path (`CPXprimopt`, presolve on) **gives no basis for presolve-detected infeasible or unbounded LPs**.

---------------------------------------------------------------------------

## E. Warm start (MEASURED: probe_basis E1-E5 and probe_basis2 P1, P9, P10)

### Knob and procedure
- Knob: `CPXPARAM_Simplex_Limits_Iterations` (1020), i.e. the wrapper's `set_iteration_limit`.
- Procedure: solve random LP A (80x60) to optimality (100 iterations, objective 629.6387864), take its basis, build a fresh identical B, call `CPXcopybase`, solve with `cplex_lp::solve()` (CPXprimopt).

### Results
- **ITLIM=0**: status `CPX_STAT_ABORT_IT_LIM` (wrapper `iteration_limit`), itcnt 0, pfeas 1, **dfeas 0**, objval = the optimum, basis = A's basis. Optimality is not certified.
- **ITLIM=1**: status **OPTIMAL**, itcnt 0 (P1); the same with ITLIM 2 and with no limit. A cold solve with ITLIM=1 gives `iteration_limit`.
- The same holds for model C: E3 gives itlim-0 → status 10, x optimal, objective 15.
- Settings:
  - `ADVIND=1` (default) and `ADVIND=2` both use the basis, with `PREIND` 1 or 0 (always 0 iterations);
  - `ADVIND=0` ignores it (all-slack start, 100 iterations).
  - CPLEX skips presolve when it starts from a user basis with ADVIND=1 (DOCUMENTED `CPLEX/Parameters/topics/AdvInd.html`: "If a basis is available only for the original, unpresolved model ... the simplex algorithm will proceed on the unpresolved model").
  - No crash or factorize call is needed.
  - **The wrapper's defaults suffice**: it never changes ADVIND, except inside `refine_lp_status`, where it is restored.
- `CPXcopystart` with only cstat/rstat behaves identically (P10).

### Bound changes
- (a) Solve, change ub(u) 2 → 1.5, copybase the old basis, solve with itlim 0: the basis is **kept**. x follows the new bound (u=1.5, y4=4.5), objective 14.5 = the new optimum, pfeas 1.
- (b) Copybase, then change the bound: getbase is still equal to the written basis, and the solve with itlim 0 gives 14.5.
- (c) Copybase, then add a row: the basis is kept and the new row is BASIC (0 iterations).
- A plain re-solve after modifications warm-starts from the resident basis: 1 iteration against 100 cold (F2).

### Invalid bases (E5, fresh model C; copybase return code, then solve)
**Every case except the NULL pointer: `CPXcopybase` returns 0 and stores the codes verbatim.** The solver repairs silently at solve and the final optimum is always correct, at the cost of a few iterations.

| Case | ITLIM=0 | Full solve |
| --- | --- | --- |
| Too many basic (9) | **objval 1e+20 (garbage)**, pfeas 0, repaired basis | optimal, 4 iterations |
| Too few basic (7) | repaired to the optimal basis | 0 iterations |
| AT_UPPER on a column with ub=+inf | silently re-sided | 0 iterations |
| AT_LOWER / AT_UPPER on a free column | → FREE_SUPER | 0 iterations |
| AT_LOWER on (−inf,5] | → AT_UPPER | 0 iterations |
| FREE_SUPER on [0,2] | → AT_LOWER, value 0, not optimal | 1 iteration |
| FREE_SUPER on [1,5] (P9) | snapped to its lower bound 1, not to 0 | |
| FREE_SUPER on [0,inf) and on a fixed column | → AT_LOWER | |
| AT_LOWER on a fixed column CPLEX reads AT_UPPER (and vice versa) | accepted; CPLEX reports its own side again | |
| AT_UPPER on 'L' / 'G' / 'E' rows, FREE_SUPER on an 'L' row | repaired to AT_LOWER | 0 iterations |
| Wrong side on an 'R' row | **honoured**: at itlim 0, x sits at that side (objval 8 / pfeas 1, objval 1 / pfeas 0) | 1 / 3 iterations |
| AT_LOWER on the free row | objval 1e+20 garbage | 6 iterations |
| Codes 7 (column), −1 (row) | treated as the good codes | |
| Singular basis (two empty columns basic) | slacks substituted | 2 iterations |
| `rstat == NULL` | **1004 CPXERR_NULL_POINTER**, nothing stored | getbase then gives 1262 |

---------------------------------------------------------------------------

## F. Partial or extended bases (MEASURED F1, F2; DOCUMENTED copybase.html)

- `CPXcopybase` takes **full arrays only**.
  - DOCUMENTED: "The length of the array is equal to the number of columns" / "number of rows".
  - There is no count and no index list, so a subset cannot be set.
  - A partial write must read, modify and write. The read fails with 1262 when no basis exists yet, so the wrapper would have to synthesize the missing entries.
- A basis taken from model C and then extended to a model with one more column and one more row:

  | New column | New row | Result at ITLIM=0 |
  | --- | --- | --- |
  | AT_LOWER | BASIC | objective 15 (the optimum), pfeas 1. A valid, optimal start. |
  | AT_LOWER | AT_LOWER | invalid basic count, garbage objective 896, pfeas 0 |
  | BASIC | BASIC | same: invalid count, objective 489 |
  | BASIC | AT_LOWER | same: invalid count, objective 489 |

  All of these are repaired at a full solve (INFERRED from E5).
- So a brand-new column needs a nonbasic code at a finite bound (AT_LOWER if lb is finite, AT_UPPER if only ub is, FREE_SUPER if free), and a brand-new row needs BASIC. These are the same codes CPLEX gives new entities in its resident basis (P4).

---------------------------------------------------------------------------

## G. Gotchas

1. **The row codes depend on the sense.** AT_LOWER means activity-at-upper on 'L' and activity-at-lower on 'G'.
   - A sense change keeps the native code and silently flips its meaning (P5).
   - Translating with the sense current at get/set time is therefore correct, and storing tags rather than native codes in the snapshot is what makes a snapshot survive sense changes.
2. **Fixed columns are reported AT_LOWER or AT_UPPER** by the sign of the reduced cost. Reporting `nonbasic_fixed` needs `CPXgetlb`/`CPXgetub`.
3. **No basis in common default cases**: before any solve; presolve-detected infeasible or unbounded (PREIND=1 is the wrapper's default); a time limit that fires before simplex starts. get_basis must throw there (`check()` turns 1262 into `solver_error("No basis exists.")`), or report the absence otherwise.
4. **A basis without a solution**: after any modification, `CPXgetbase` succeeds with CPLEX's resident, adjusted basis while getstat = 0 and getx fails with 1217.
   - get_basis then returns the basis the next solve will start from, not "the basis of the last solve".
   - The wrapper's cached `_status` still says optimal there (existing behaviour, not basis-specific).
5. **`CPXcopybase` discards the current solution** (getstat 0, getx 1217, E2). `set_basis` should call `reset_status()`, as `compute_iis()` does, or the wrapper keeps reporting `optimal` with no solution behind it.
6. **`CPXcopybase` validates nothing and stores codes verbatim.** get_basis between set_basis and solve returns what was written. Repair happens only at the next solve, silently. A set_basis that writes only legal codes keeps the read mapping total.
7. **Never pass NULL to `CPXcopybase`**, even for zero rows or zero columns (1004). `std::vector<int>::data()` of an empty vector may be NULL, so use a one-element dummy. `CPXgetbase` accepts NULL.
8. **Iteration limit 0 never reports optimal**, even from an optimal basis: the status is `iteration_limit`, dfeas 0, itcnt 0, the primal is optimal. Iteration limit 1 does report optimal with 0 iterations. A solver-agnostic warm-start test should use limit 1 against a cold solve with limit 1, not limit 0.
9. **`ADVIND=0` silently ignores a written basis.**
   - The wrapper never leaves it at 0: `refine_lp_status` sets it temporarily and restores it, which means refine re-solves from scratch and drops a user basis on purpose.
   - A user changing it through the native handle disables set_basis.
10. **`CPXbaropt` (native)** gave no basis here even with crossover on auto. `CPXlpopt` with LPMETHOD=4 crosses over, unless SolutionType=2. `cplex_lp::solve()` always runs `CPXprimopt`, so this is reachable only through the native handle.
11. **Remapping and removal.** The snapshot keyed by uid reads a removed variable's status for a later handle that reuses its id (the same caveat as iis_snapshot). That may write one basic too many, which CPLEX repairs silently (E5).
12. **The vendor-header build** (`MIPPP_INCLUDE_CPLEX_HEADER`) gets `CPX_AT_LOWER`, `CPX_BASIC`, `CPX_AT_UPPER` and `CPX_FREE_SUPER` as macros from cpxconst.h. The constants must be declared only in the `#else` branch and must never be reused as identifiers (INFERRED).
13. No lazy update, no callbacks on `cplex_lp`, one env per model (`cplex_base` constructor): no thread or callback restriction applies (INFERRED from the code). The problem stays CPXPROB_LP on cplex_lp. A native LP→MILP→LP switch keeps the basis (P7).
14. `compute_iis()` keeps the basis. It changes getstat to 30/31 and the re-solve takes 0 iterations (D9).

---------------------------------------------------------------------------

## H. Recommendation

### Status variants (only the tags CPLEX really reports)
- Columns: `std::variant<nonbasic_at_lower_bound, basic, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`.
  - Alternative 0 = `nonbasic_at_lower_bound`: what CPLEX gives a new column with a finite lb, and what an id past the table bound would be written as. CPLEX re-sides it at solve if the bounds disagree.
  - Alternative: put a bare `nonbasic` first, which set_basis resolves from the current bounds. It mirrors CPLEX's own choice (P4) without lying on read, but it is a tag CPLEX never reports itself. **Maintainer's call.**
- Rows: `std::variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed>`, adding `nonbasic_free` only if free rows must be representable (CPLEX never reports it after a solve).
  - Alternative 0 = `basic`: the status of a row unknown to the snapshot, and what CPLEX gives a new row.

### get_basis()
1. `CPXgetbase`, then `check()` (1262 → `solver_error`).
2. Read lb/ub, sense and rhs in bulk (rngval only to detect zero-range 'R').
3. Fill `handle_status_table`s: variables sized `_handle_id_bound(ncols)` and keyed `_var_handle(j).uid()`; rows sized `nrows` and keyed `i`.
4. Return an `lp_basis` snapshot by value, as `_compute_iis` does.

### set_basis(b)
1. Build full cstat/rstat over the native columns and rows from `b.get_status(_var_handle(j))` and `b.get_status(constraint(i))`, using the write mapping with the *current* bounds and senses.
2. Pass non-NULL arrays even when a count is 0.
3. `check(CPXcopybase)`.
4. `reset_status()`.

No parameter, presolve or factorization handling is needed: ADVIND=1 is the default.

### Deviations from the drafted concepts this backend forces (to rule on)
1. **Rows cannot hold `nonbasic_free`**, nor a wrong-side tag on L/G/E rows.
   - The drafted `has_modifiable_lp_basis` requires all five `set_status` tags on constraints.
   - Either the row variant lists them all, and set_basis maps the meaningless ones to the row's only legal nonbasic code (documented UB; CPLEX repairs anyway), or the concept checks only the tags the backend's variant lists.
2. `set_nonbasic(v, s)` and `set_nonbasic(c, s)` (snap by value) need bounds. The snapshot would have to copy lb/ub and row sides at get time (it reads them anyway for the mapping), or keep a model pointer, which contradicts the by-value design.
3. **No basis after presolve-detected infeasible/unbounded or an early time-limit stop** under the default algorithm. get_basis throws there, so tests must not require a basis after such statuses (or must turn presolve off, which the wrapper has no API for).
4. **get_basis after a modification returns CPLEX's resident basis**, with no solution behind it. Accept this as "the basis the next solve starts from", or define get_basis as valid only after a solve.
5. An ITLIM=0 warm start reports `iteration_limit`, not `optimal`. The warm-start test should use iteration limit 1.
6. The fallback api declarations must add `CPXgetbase` and `CPXcopybase` (required symbols, present in every installed release) plus the four status constants. No `feature_unavailable_error` gate is needed for 22.1.x (22.1.0 and 22.2.0 inferred, not probed).

---------------------------------------------------------------------------

## Open questions
- Alternative 0 of the column variant: `nonbasic_at_lower_bound`, or a bare `nonbasic` resolved at set time?
- Should has_modifiable_lp_basis demand every tag on rows? CPLEX can't write `nonbasic_free` on rows.
- Bounds in the snapshot for `set_nonbasic(·, s)`: a copy, or no `set_nonbasic` on CPLEX?
- get_basis after modifications: return the resident basis (natural on CPLEX), or throw?
- "No basis": `solver_error` from `check()`, or a dedicated exception / optional?
- 22.1.0 and 22.2.0 were not available to probe.
