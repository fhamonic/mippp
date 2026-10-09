# HiGHS LP basis survey (highs_lp, highs_qp)

Tags: **MEASURED** (probe run here), **DOCUMENTED** (vendor source/header/docs, path:line), **INFERRED**.

Libraries probed (both pinned with `MIPPP_HIGHS_LIBRARY`):
- 1.15.1 `/home/plaiseek/.local/lib/python3.10/site-packages/highspy/libhighs.so.1.15.1`
- 1.10.0 `/usr/local/lib/libhighs.so.1.10.0`
- Default search (no pin) found **1.15.1** at `/home/plaiseek/Softwares/solvers/HiGHS/build/lib/libhighs.so` (rebuilt 2026-10-06; it comes first on LD_LIBRARY_PATH), not 1.10.0 as the task said (MEASURED, out_lp_C_default.txt).
- HiGHS source tree `/home/plaiseek/Softwares/solvers/HiGHS` is at v1.15.1-7; the C header is `highs/interfaces/highs_c_api.h` (moved from `src/interfaces/` at v1.11). Tags v1.1.1..v1.15.1 were used for release history.

Probe sources (in this directory): `probe_lp.cpp` (sections C, D, E, F; run as `./probe_lp <C|D|E|F>`), `probe_qp.cpp`, `probe_misc.cpp`.
Outputs: `out_lp_{C,D,E,F}_{1151,1100}.txt`, `out_lp_C_default.txt`, `out_qp_libhighs.so.1.{15.1,10.0}.txt`, `out_misc_libhighs.so.1.{15.1,10.0}.txt`.
Every probe ran under `timeout -k 5 60` and exited 0. No probe process survived (checked with ps).

Code legend used below: L0 = kHighsBasisStatusLower(0), B1 = Basic(1), U2 = Upper(2), Z3 = Zero(3), N4 = Nonbasic(4).

---

## A. Wrapper facts

**Checkout note.** Local `main` (04431ca) is 13 commits behind `origin/main` (506af98). The ranges in the task, `supported_versions`, `native_iis_release` and `feature_unavailable_error` exist only on origin/main (commit 00432a9). Line numbers below are for local main, with origin/main given where they differ (read with `git show origin/main:...`).

- **Classes.** `highs_lp : public highs_base` (highs_lp.hpp:19). `highs_qp : public highs_base` (highs_qp.hpp:24). `highs_milp` is excluded by the ruling. `highs_base : protected remapping_model_base<int, double>` (highs_base.hpp:37).
- **Variable ids are remapped.** `_native_id` and `_var_handle` are at remapping_model_base.hpp:40-47. `_handle_id_bound` is at :90-93. The remap turns on at the first deletion that is not a tail deletion (highs_base.hpp:217-252), and freed handle ids are reused (`_free_var_handles`, remapping_model_base.hpp:56-71).
- **Removal.** Variables can be removed: `remove_variable` / `remove_variables` (highs_base.hpp:349-362). Deletion is **not lazy** in this wrapper: both functions call `_lazily_remove_variables()` at once, which issues `Highs_deleteColsBySet` (highs_base.hpp:202-254). So `_var_handles_to_delete` is empty between public calls. Constraints cannot be removed: there is no remove_constraint.
- **Constraint handle to row.** `constraint(constr_id)` with `constr_id = num_constraints()` at insertion (highs_base.hpp:426,440). `native_id(c)` is `c.id()` (highs_base.hpp:99). Rows are never remapped: handle id = uid = native row.
- **Ranged rows.** The wrapper has no `add_ranged_constraint`. A ranged row comes from `set_constraint_lower_bound` / `set_constraint_upper_bound` on an inequality (highs_base.hpp:614-621; test at test/solvers/highs.cpp:47-61). The probe builds its ranged row that way.
- **solve().**
  - On local main (highs_lp.hpp:98-105): if `num_variables()==0` it sets unknown and returns. Otherwise `_run()` = `Highs_zeroAllClocks` + `Highs_run` (highs_base.hpp:674-677), then `_status = _get_status()`.
  - On origin/main (highs_lp.hpp:97-102): `reset_status()` comes first.
  - No option is ever set except `output_flag=false` in the constructor (highs_base.hpp:62), plus time and iteration limits on request. So HiGHS defaults apply: `solver="choose"`, `presolve="choose"`, `run_crossover="on"` (DOCUMENTED HighsOptions.h:749-773 in 1.15.1; the same defaults in v1.7.2 and v1.10.0).
  - For an LP, "choose" runs dual simplex: only "ipm/ipx/hipo" select IPM (DOCUMENTED HighsSolve.cpp:41, HighsOptions.cpp:1169-1181). MEASURED: simplex iterations on every default solve.
  - There is no reload: the model is kept incrementally.
  - `set_iteration_limit` writes `simplex_iteration_limit` (highs_lp.hpp:27-32). On highs_qp it writes both `simplex_iteration_limit` and `qp_iteration_limit` (highs_qp.hpp:178-184).
- **Pattern a basis getter follows.**
  - `get_dual_solution` does one bulk `Highs_getSolution` into a `unique_ptr<double[]>` and returns `constraint_mapping(std::move(arr))`, indexed by row (highs_lp.hpp:123-129; origin/main :120).
  - `get_reduced_costs` / `get_solution` return a `variable_mapping` lambda that indexes the native array with `_native_id(v)`; that is a live translation, not a snapshot (highs_lp.hpp:113-139).
  - The IIS snapshot follows the by-value pattern: `detail::iis_answer<...> answer(_handle_id_bound(num_col), num_row)` (highs_base.hpp:772; origin/main :769), keyed by `_var_handle(col).uid()` (highs_base.hpp:862; origin/main :859), with rows keyed by native row.
- **Versions** (origin/main highs_api.hpp:357-372):
  - `native_iis_release{1,14}`.
  - `validated_versions = {[1.14, 1.16)}`.
  - `supported_versions = {[1.7.2, 1.16)}`.
  - Local main still has `validated_versions = {[1.8.1, 1.16)}` and no supported range (highs_api.hpp:357-358).
- **Optional symbols.**
  - `HIGHS_OPTIONAL_FUNCTIONS(F)` holds `Highs_setSparseSolution` and `Highs_getIis` (highs_api.hpp:335-337). They are resolved with `lib.find_function` (nullptr when absent, :344-345). Required symbols use `lib.get_function` (:342-343), and a missing one refuses the load.
  - Gating pattern on origin/main:
    - `_compute_iis()` throws `feature_unavailable_error` when `library_version() < native_iis_release` or `Highs->getIis == nullptr` (origin/main highs_base.hpp:747-765).
    - `_add_mip_start` throws when `!Highs->setSparseSolution` (origin/main highs_milp.hpp:49-58).
  - `feature_unavailable_error : solver_error` is defined at origin/main utility/solver_exceptions.hpp.
  - `highs_api::_check` throws `solver_error` only on kHighsStatusError (-1). Warnings pass (highs_api.hpp:372-375).
- **Basis declarations already present.**
  - The `BasisStatus` enum is at highs_api.hpp:202-208; origin/main :203-209.
  - `Highs_setBasis` is declared at :209-210 and `Highs_getBasis` at :211-212; origin/main :210-213.
  - Both are in the REQUIRED list `HIGHS_FUNCTIONS` (:331-332; origin/main :332-333), so the api already refuses a library without them.
  - `Highs_getIntInfoValue` is declared too (:168-169).
  - `Highs_getColsByRange` / `Highs_getRowsByRange` are declared (:67-71, :129-133). These are the bulk bound reads needed to tell "fixed" apart.

## B. Native API

All entries below are DOCUMENTED: highs_c_api.h in the 1.15.1 tree, with tag sweeps by `git show <tag>:.../highs_c_api.h`.

- **Bulk read.** `HighsInt Highs_getBasis(const void* highs, HighsInt* col_status, HighsInt* row_status)` (highs_c_api.h:1012). There is no per-entity getter.
  - Implementation (highs_c_api.cpp:571-582): it copies `basis.col_status.size()` / `row_status.size()` entries, not `num_col` / `num_row`, and **always returns kHighsStatusOk**. So the return code says nothing about availability.
  - The internal vectors are padded to the model size on every `returnFromHighs` (`forceHighsSolutionBasisSize`, Highs.cpp:4703-4741), so arrays of `getNumCol` / `getNumRow` entries are filled. MEASURED: no sentinel was ever left unwritten.
- **Bulk write.** `HighsInt Highs_setBasis(void* highs, const HighsInt* col_status, const HighsInt* row_status)` (highs_c_api.h:1264).
  - It reads exactly `Highs_getNumCol` and `Highs_getNumRow` entries (highs_c_api.cpp:668-710), so a too-short array is an out-of-bounds read.
  - A code outside 0..4 returns kHighsStatusError (-1).
  - Otherwise it builds a `HighsBasis` whose default is `alien = true` (HStruct.h:72) and calls `Highs::setBasis`. That function accommodates and factorizes at call time (`accommodateAlienBasis`, HighsSolution.cpp:1914-1973, via Highs.cpp:2734-2776). It returns kError only when that factorization fails.
- **Reset.** `HighsInt Highs_setLogicalBasis(void* highs)` (highs_c_api.h:1274) calls `Highs::setBasis()` = `invalidateBasis()` (Highs.cpp:2807-2817). It is **not declared** in the api: the probe reached it with dlsym.
- **Exist, not needed.** `Highs_getBasicVariables`, `Highs_getBasisInverseRow/Col`, `Highs_getBasisSolve`, `Highs_getBasisTransposeSolve`, `Highs_getReducedRow/Column` (highs_c_api.h:1087-1210). None is declared in the api.
- **Availability flag.** The info item `"basis_validity"` (HighsInfo.h:228-229; `kHighsBasisValidityInvalid=0` / `Valid=1`, highs_c_api.h:60-61) is read through `Highs_getIntInfoValue`, which is already declared. When `info_.valid` is false it returns kHighsStatusWarning(1) and **leaves the value unwritten** (Highs.cpp:367-379, HighsInfo.cpp:277). MEASURED: rc=1 with the sentinel kept.
- **Release availability.** `Highs_getBasis`, `Highs_setBasis`, `Highs_setLogicalBasis` and the inverse functions are present with the same signatures in **every tag from v1.2 to v1.15.1**, including all of [1.7.2, 1.16). The `kHighsBasisStatus*` values are 0..4 in v1.7.2, v1.10.0, v1.14.0 and v1.15.1, and `basis_validity` exists in v1.2, v1.7.2, v1.10.0 and v1.15.1. **No feature gate is needed for get/set.** Only the QP option `qp_allow_hot_start` is 1.15+ (commit 424ac9ffdc, first tag v1.15.0). MEASURED: setting it on 1.10.0 returns rc=-1.
- **Codes** (highs_c_api.h:93-97; HConst.h:249-257; docs/src/structures/enums.md:87-96):
  - 0 kLower = "nonbasic at its lower bound (or fixed value)".
  - 1 kBasic.
  - 2 kUpper = "at its upper bound".
  - 3 kZero = "free variable nonbasic and set to zero".
  - 4 kNonbasic = "nonbasic with no specific bound information - useful for users and postsolve".
  - Rows are described as the "(slack) variable", but the measurement below shows the code names the side of the row activity.

## C. Row side convention (MEASURED, identical on 1.10.0 and 1.15.1)

LP C:

```
min -3x1 - 2x3 + 3x4 + x6
rle:  x1 + x2 + x3 <= 10
rge:  x2 - x3 + x4 >= 1
req:  x3 + x4 + x5 + f == 9
rrng: 2 <= x2 + x5 <= 6
rsl:  x2 + x3 + x4 + x5 + x6 <= 100
x1 in [0, 4], f = 2, w free (in no row), the other columns >= 0
```

The optimum is unique and primal nondegenerate: x = (4, 3, 3, 1, 3, 0, 2, 0), objective -15, every tight row with a nonzero dual. The probe read it with presolve "choose" and "off", minimizing, and maximizing the negated objective.

- **Columns.**
  - x1 at its upper bound: **U2**.
  - x2..x5: **B1**.
  - x6 at lower: **L0**.
  - Fixed f: **U2 when minimizing** (dual -1) and **L0 when maximizing** (dual +1), so the sign of the dual picks the code (HEkk.cpp:1397-1401 `dual >= 0 ? kLower : kUpper`).
  - Free w, nonbasic: **Z3**.
- **Rows.**
  - `<=` tight: **U2**.
  - `>=` tight: **L0**.
  - Ranged row tight at its upper side: **U2**.
  - Slack row: **B1**.
  - `==` row: **U2 (min) / L0 (max)**, by the sign of the dual, like f.
- **Negated rows** (`-x1-x2-x3 >= -10`, `-x2+x3-x4 <= -1`, ranged `[-6,-2]` tight at -6): the codes flip to L0, U2, L0.
  - So the code follows **the side of the ACTIVITY a·x** within [row_lower, row_upper], not the solver's logical. HiGHS's internal logical is sign-flipped (`HEkk::setBasis` maps row kLower to MoveDn while it maps a column's kLower to MoveUp, HEkk.cpp:1238-1241 versus 1215-1218), but the status that HiGHS exposes is already in activity terms.
  - The code does not depend on the sense of the objective either: the same codes appear for min and max, except for fixed entities.
- **No code 4** ever appeared on a validity-1 LP read.

**Read mapping.** It is the same for columns and rows. Rows are on the activity side.
- 1: basic.
- 0: lb == ub ? nonbasic_fixed : nonbasic_at_lower_bound.
- 2: lb == ub ? nonbasic_fixed : nonbasic_at_upper_bound.
- 3: nonbasic_free.
- 4: nonbasic (generic). It is never produced on an LP read with validity 1. It appears only in QP output and in the pre-solve pad from 1.12 on.
- The fixed test needs the bounds: one bulk `Highs_getColsByRange` plus one `Highs_getRowsByRange`.

**Write mapping.** Same for columns and rows.
- basic: 1.
- nonbasic_at_lower_bound: 0.
- nonbasic_at_upper_bound: 2.
- nonbasic_fixed: 0. When lb == ub, HiGHS ignores the side (HEkk.cpp:1213-1214, 1236-1237); 4 would also do.
- nonbasic_free: 3.
- nonbasic (generic): 4. HiGHS picks the side itself (refineBasis, HighsSolution.cpp:1421-1497): from a valid solution when it has one, otherwise the bound nearer zero.
- Every tag can be written. A tag inconsistent with the bounds is repaired silently (see E).

## D. Availability

All MEASURED; "pad" means the codes an absent basis reads as, which differ by release (see below).

- **Before any solve.** `basis_validity` is unavailable (rc=1, unwritten). The codes are the pad: **1.10.0 all L0 for columns and rows (0 basic); 1.15.1 columns N4, rows B1**.
  - The padding changed in commit 9089098cbe, first tag **v1.12.0**. Before that, `resize()` value-initialised to kLower (v1.10.0 Highs.cpp:4229-4247).
  - On an empty model, getBasis writes nothing and returns Ok.
- **Optimal.** validity=1 and #basic == #rows, with simplex, with IPM plus crossover, and when presolve reduces the LP to empty (the basis then comes from postsolve).
- **Infeasible LP.**
  - With presolve "choose" (the wrapper default), presolve proved infeasibility: validity=0 and the codes are the pad.
  - With presolve "off": validity=1 and a real basis (2 basic for 3 rows plus a basic column).
- **Unbounded LP.** validity=1 with a basis under both presolve settings, on this model. A model where presolve detects unboundedness would presumably behave like the infeasible case (INFERRED).
- **Iteration limit 1 and 5** (status 14). validity=1, the basis of the current iterate, #basic == #rows.
- **time_limit=0** (status 13). validity=1, the starting logical basis (all columns L0, rows B1).
- **After a modification without re-solving.** This covers a column bound, a row bound, an added row, 5 added columns, a cost change, removing a nonbasic column and removing a basic column. These runs used presolve="off" (the `fresh` lambda). Basis maintenance does not depend on presolve, and E/B4 shows that presolve="on" still uses the basis.
  - `basis_validity` becomes **unavailable** (rc=1). Every modification calls `invalidateInfo()` (Highs.cpp:3932-3943, HighsInfo.cpp:25).
  - Yet getBasis returns the basis that HiGHS **maintains**:
    - a new row is B1 (appendBasicRowsToBasisInterface, HighsInterface.cpp:1406-1425);
    - a new column gets its code from its bounds (HighsInterface.cpp:1329-1404). MEASURED: [0,inf) gives L0, [-1,3] L0, [-5,3] U2, (-inf,3] U2, free Z3;
    - a bound change keeps definitive sides (setNonbasicStatusInterface, HighsInterface.cpp:1187-1270);
    - a deletion compacts the arrays. Deleting a basic column leaves **#basic < #rows** (4 for 5 rows).
  - The next solve logs "Solving LP with useful basis" and uses it: 0 iterations after an added row, added columns, a bound change or a deleted nonbasic column. After a basic column is deleted, HiGHS completes the basis itself.
- **Non-simplex algorithms.**
  - IPM without crossover, and PDLP: validity=0 and the codes are the pad.
  - 1.10.0 PDLP run after a setBasis: validity=0, but getBasis still returns the **old codes** (stale).
  - From **v1.12.0** (commit ec85bca2b5), `Highs::run` **clears** any basis when `solver` is not simplex or choose (Highs.cpp:1483-1489).
- **QP solver** (highs_qp with a Hessian): validity=0, and the codes are an active set: interior coordinates N4, active bounds L0/U2, active row U2, inactive row B1 (a_quass.cpp:88-124).
- **#basic == #rows guarantee.** Whenever validity==1: MEASURED on every such read, and DOCUMENTED, since `returnFromHighs` checks `debugHighsBasisConsistent` (Highs.cpp ~4967+). It is not guaranteed when validity is 0 or unavailable: the pad of 1.10 has 0 basic, and after deleting a basic column there are fewer than #rows.
- **Native flag to key on.** `Highs_getIntInfoValue(h, "basis_validity", &v)` with v pre-set to 0, since a Warning leaves it unwritten. There is no error code to key on: getBasis is always Ok.
- **Caveat.** Gating on that flag makes `get_basis()` unavailable right after any modification, although HiGHS still holds and will use a basis.

## E. Warm start (MEASURED, identical on both releases unless noted)

**Main experiment.** LP R: 40 rows, 60 columns, max c·x, A ≥ 0, a third of the columns boxed. A cold solve takes 48 simplex iterations, and presolve does not reduce R.

| Run | Model status | Simplex iterations | Note |
|---|---|---|---|
| Control B0: fresh R, no basis, `simplex_iteration_limit=0` | IterationLimit (14) | 0 | log "Presolving model … Not reduced" |
| B1: fresh R, `Highs_setBasis(A's basis)` (rc 0, reads back identical), `simplex_iteration_limit=0` | **Optimal (7)** | **0** | objective = A's |
| B1 without the limit | Optimal | 0 | |

- The knob is `simplex_iteration_limit`, the same one as the wrapper's `set_iteration_limit`.
- B1's log is "Solving LP with useful basis so presolve not used" on 1.15.1, and "Solving LP without presolve, or with basis, or unconstrained" on 1.10.0.
- **No setting is needed, and the wrapper defaults suffice.** With `solver` "choose" or "simplex" and a useful basis, HiGHS skips presolve by itself (Highs.cpp:1466-1548).
  - presolve="on" with a basis: still 0 iterations.
  - simplex_strategy 1 (dual) and 4 (primal): 0 iterations.
  - No factorize or warm-up call is needed, because setBasis factorizes.
- **Algorithm choice.**
  - solver="ipm": the basis is not used. 12 IPM iterations, then crossover gives validity 1.
  - solver="pdlp": not used, validity 0. On 1.10 the old codes are left in place.
- **Bound changes.**
  - setBasis, then change a bound, then solve: 0 iterations, the basis is kept.
  - On a solved A, changing the upper bound of a basic column to 0.1 keeps HiGHS's own basis: getBasis gives the same codes, and info is unavailable.
  - `setBasis(old basis)` after that change returns rc 0 and **is kept** (it reads back identical). The re-solve takes **8 dual simplex iterations, against 44 for a cold solve** of the changed model.
- **On a solved, unmodified model** (probe_misc), setBasis(slack basis) returns rc 0, and then:
  - the model status stays Optimal and basis_validity stays 1;
  - `get_solution()` still reads the old point;
  - getBasis returns the new basis;
  - the re-solve starts from it.
- **Small LP C.** Fresh plus setBasis plus limit 0: Optimal with 0 iterations. The control with limit 0 stops at IterationLimit.

**Invalid bases** (LP C, fresh model, setBasis then solve without a limit):
- **13 basic (all B1).** rc 0. The read-back is repaired by `accommodateAlienBasis`: 3 columns (x1, x6, f) and 2 rows stay basic, the other former basics become N4. Optimal after 5 iterations.
- **0 basic (all L0).** rc 0. The read-back is completed with logicals (every row B1). Optimal after 5 iterations.
- **x6 U2 with an infinite upper bound.** rc 0. It reads back U2 as written; the side is corrected at solve time (initialiseNonbasicValueAndMove, HEkk.cpp:2716-2771). Optimal after 0 iterations, then reported L0.
- **Free w given L0.** rc 0, reads back L0. Optimal after 0 iterations, reported Z3.
- **Boxed x1 given Z3, N4, or L0 (the wrong side).** rc 0. Optimal after 0 iterations: HiGHS corrects the side (N4 through refineBasis) and the dual simplex flips boxed nonbasics without counting iterations (INFERRED mechanism).
- **`<=` row given L0** (its infinite side), or **ranged row given L0** (the wrong finite side): rc 0, Optimal after 0 iterations.
- **Code 5 on a column, or -1 on a row.** **rc = -1 (kHighsStatusError)**, the basis is left unchanged, and the solve is cold (4 iterations). The wrapper's check() would throw `solver_error`.
- **Singular** (the empty column w made basic, rows nonbasic): rc 0. Repaired: w becomes N4 and rle becomes B1. Optimal after 2 iterations.
- No case crashed, and none gave a wrong answer on either release. INFERRED: both are release builds with HiGHS's asserts off. NDEBUG was not checked on either .so; the N4 case never reached `assert(... == kZero)` in HEkk::setBasis anyway, because refineBasis ran first.

## F. Partial and extended bases (MEASURED, both releases)

- **The arrays must be complete.** The C API reads exactly num_col and num_row entries: there is no subset setter and no per-entity setter (DOCUMENTED highs_c_api.cpp:668-710).
- **A model that grew after the get** (+1 column with cost 5 in rsl, +1 slack row): A's basis extended with **new column L0, new row B1** gives rc 0 and **Optimal after 0 iterations with limit 0**.
- **New column B1, new row U2:** rc 0. The basis is repaired at set time (rle becomes B1, rsl becomes N4). Optimal after 3 iterations.
- **The new statuses that keep the basis valid.** A new column must be nonbasic: L0/U2/Z3 by its bounds, or 4 to let HiGHS choose. A new row must be basic. That is exactly what HiGHS does itself on add_column / add_constraint (D).
- **Partial bases.**
  - The wrapper can emulate a partial basis by filling unspecified columns with 4 and unspecified rows with 1. HiGHS documents that a basis "need not be complete" (docs/src/guide/further.md:92-106): it demotes extra basics and completes with logicals.
  - "Basics only, others N4", with limit 0: **IterationLimit** at 0 iterations. refineBasis put x1 and rrng on the bound nearer zero, the wrong side.
  - "Basics only, others L0": limit 0 gives IterationLimit; without a limit, **Optimal after 0 counted iterations** (the boxed sides are flipped).
  - The side of a boxed entity therefore matters only under an iteration limit of 0.
- **setBasis then add a column and a row** (no explicit extension): HiGHS extends the basis itself (new column L0, new row B1). Optimal after 0 iterations with limit 0.
- **setBasis then remove a basic column through the wrapper:** 4 basic for 5 rows. HiGHS uses the "useful basis", completes it, and the solve ends after 1 iteration (that model is Infeasible).

## G. Gotchas

1. **`basis_validity` is unavailable** (Warning, value unwritten) before the first solve, and after any modification. On a fresh model it is also unavailable after setBasis. Yet HiGHS keeps, extends and uses its basis through modifications, so a validity gate is conservative. Pre-initialise the int to 0.
2. **Never infer availability from the codes.** The pad is all L0 before v1.12, and columns N4 / rows B1 from v1.12 on. Stale codes also survive `Highs_setLogicalBasis` (both releases) and a 1.10 PDLP run. getBasis always returns Ok.
3. **`Highs_setLogicalBasis` does not force a cold start on a solved model.** A valid solution makes `Highs::run` rebuild a basis from it (`basisForSolution`, Highs.cpp:1472-1481, since v1.3.0). MEASURED: the re-solve took 0 iterations with "useful basis".
   - basis_validity reads 0 on 1.15.1 but stays 1 on 1.10.0 after setLogicalBasis.
4. **setBasis does not touch the model status or the solution.** On a solved model, Optimal and the old solution remain while getBasis returns the new basis. The wrapper's `_status` would likewise stay stale unless set_basis resets it.
5. **A fixed column or an `==` row reads 0 or 2 by the sign of its dual,** so "fixed" is only known from the bounds.
6. **Boxed sides are soft on warm start:** the dual simplex flips them for free. Under `simplex_iteration_limit=0` the stop happens before any flip, so a test that pins "0 iterations" must write exact sides.
7. **`Highs_setBasis` factorizes immediately** (the alien path): the cost is paid at set time, and the call needs the current dimensions. It also accepts a basis for a model with no row: basic columns are turned into N4 (Highs.cpp:2737-2745; MEASURED).
8. **Presolve.**
   - Presolve is skipped automatically while a useful basis exists, even with presolve="on". A non-simplex solver clears or ignores the basis.
   - Under default settings, presolve can prove infeasibility and leave **no basis** (validity 0). So "no basis after solve" is a normal runtime outcome, not only misuse.
9. **No lazy updates.**
   - HiGHS applies every change at once.
   - The wrapper's "lazy" removal is eager.
   - compute_iis() (deleteColsByRange(0,-1) plus getIis) leaves the basis and its validity intact (MEASURED on 1.15.1; 1.10 refuses compute_iis).
10. **Stale statuses after a remap.** With the remap on, a variable created after the snapshot can reuse the handle id of a removed variable, and then reads that variable's status, possibly `basic`. This is the same caveat as the iis_snapshot header comment. MEASURED consequence: an extra column written B1 (F, "new col B1 / new row U2") gives rc 0. HiGHS repairs the basis at set time (rle becomes B1, rsl becomes N4) and the solve takes 3 iterations instead of 0. The answer is still correct, so documentation suffices.
11. **No solution slots,** and no callback or thread rule specific to the basis. The calls need nothing set beforehand.
12. **highs_qp switches algorithms at run time.**
    - With a Hessian, the QP solver writes an active set (N4 entries, validity 0) that is not a simplex basis.
    - Without a Hessian, simplex gives a regular basis: MEASURED U2 B1 / U2 B1, validity 1.
    - `Highs_setBasis` on a QP is accepted (rc 0, reads back as written) but **ignored** by the QP solver: with limit 0 the status is IterationLimit at x=0.
    - A QP hot start needs a valid solution plus a valid HighsBasis. 1.10 does it automatically: a setSolution plus setBasis, or an in-place re-solve, returns the optimum with 0 QP iterations. **From 1.15.0 it requires `qp_allow_hot_start=true`** (default false), and an in-place re-solve then starts cold (7 QP iterations).
    - A hot start with qp_iteration_limit=0 reports IterationLimit (14) at the optimal point, not Optimal.
13. **Incidental, outside basis:** on 1.10.0, solver="ipm" on a QP returned "Optimal" with objective -4 instead of -0.5. It seems to have dropped the Hessian. The wrapper never sets solver itself.

## H. Recommendation

- **highs_lp** gets `get_basis()` and `set_basis(b)`: has_lp_basis, has_modifiable_lp_basis on the snapshot type, has_lp_basis_warm_start.
- **Column status variant.** Alternative 0 is what a handle id past the snapshot bound reads, that is a column created after the snapshot. Two options:
  - (a) `std::variant<nonbasic, basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`. Generic `nonbasic` as alternative 0 is written as code 4, and HiGHS picks the side from the bounds. It is honest for a column the snapshot never saw, and it also absorbs a code 4 read. The cost: it lists a tag that HiGHS never reports for an LP column that existed at snapshot time.
  - (b) Drop `nonbasic` and use `nonbasic_at_lower_bound` as alternative 0. This is what HiGHS itself assigns to a new [0,inf) column, and it is silently repaired on other bounds. But reading it for a free or upper-bounded new column is not true.
  - Recommended: **(a)**.
- **Row status variant.** `std::variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`, with **basic as alternative 0**. A row added after the snapshot is basic, which matches HiGHS's own extension and keeps the basis square.
  - `nonbasic_free` occurs only for a free row (both bounds set to infinity), which the wrapper allows through set_constraint_lower/upper_bound.
- **get_basis().**
  1. Read `basis_validity` with v pre-set to 0, through `check()`, which lets the Warning through. If v != 1, signal "no basis". This is a design decision for the maintainer: under default settings it happens after a presolve-proven infeasibility, so it is not only misuse.
  2. Make one `Highs_getBasis` call into arrays of `_num_var_native_ids()` / `num_constraints()` entries.
  3. Make one `Highs_getColsByRange(0, n-1)` and one `Highs_getRowsByRange(0, m-1)` call for the bounds, to name nonbasic_fixed.
  4. Fill `handle_status_table`s sized `_handle_id_bound(n)` (keyed by `_var_handle(j).uid()`) and m (keyed by row), exactly like `_compute_iis`.
  5. **Keep the bounds read in step 3 in the snapshot.** The drafted `b.set_nonbasic(v, s)` / `b.set_nonbasic(c, s)` "snaps to lower/upper/free within tolerance" on a by-value snapshot that may outlive model changes, so it needs bounds of its own. Writing code 4 instead throws away the user's value: refineBasis picks the side from HiGHS's own solution or from the bound nearer zero (HighsSolution.cpp:1421-1497), never from `s`.
- **set_basis(b).**
  1. Build complete native arrays: for column j, `b.get_status(_var_handle(j))`; for row i, `b.get_status(constraint(i))`. Map with the write mapping.
  2. Call `check(Highs->setBasis(...))`. It throws solver_error only on an unknown code (unreachable) or a failed factorization.
  3. No flags, no presolve change, no factorize call. The wrapper defaults (solver choose, presolve choose) already make HiGHS use the basis and skip presolve.
  4. Decide whether to `reset_status()`: HiGHS keeps the old status and solution.
  5. Since a basis always comes from the wrapper's own mapping, all six tags are writable. An inconsistent tag is repaired by HiGHS without an error, so obviously wrong use is documented UB and needs no guard.
- **Versions.** Nothing to gate: the symbols are required ones and exist in all of [1.7.2, 1.16). `Highs_setLogicalBasis` is not needed.
- **Deviations to rule on.**
  - (1) HiGHS cannot name "fixed" by code. The wrapper must read bounds, or report fixed entities as at_lower/at_upper (the side chosen by the dual sign).
  - (2) `get_basis()` needs a "no basis" outcome even under default settings, and the gate also reports "no basis" after any modification, although HiGHS still holds a basis.
  - (3) Alternative-0 semantics: generic `nonbasic` for columns and `basic` for rows are new choices, not the IIS `absent`.
  - (4) **highs_qp.** Recommendation: **no basis support**, because the QP solver neither reports a simplex basis (validity 0, active-set codes) nor uses setBasis. A class-level concept cannot depend on whether a Hessian is set. Alternatively the maintainer may allow it, with "no basis after a quadratic solve" and "set_basis ignored by the QP solver" documented. A QP hot start would be a separate feature (setSolution plus basis, `qp_allow_hot_start` from 1.15).
  - (5) Tests that check "0 iterations after set_basis" must use exact sides, or no iteration limit, because of the free bound flips.

## Open questions for the maintainer

1. **How `get_basis()` reports that there is no basis** (basis_validity != 1). It could throw solver_error, throw a dedicated error, or return a snapshot carrying an outcome, as iis_snapshot does. Under default settings this happens after a presolve-proven infeasibility. With the validity gate it also happens after any modification made since the last solve.
2. **Whether `set_basis()` should call `reset_status()`.** HiGHS keeps the old model status and the old solution after setBasis.
3. **Alternative 0 for columns.** Generic `nonbasic` (option a) or `nonbasic_at_lower_bound` (option b). Rows: `basic`.
4. **Where the snapshot's `set_nonbasic(v, s)` gets bounds from.** Recommended: store the bounds read at get time in the snapshot. Writing code 4 instead would ignore `s`.
5. **highs_qp.** No basis support (recommended), or support documented as "unavailable after a quadratic solve and ignored by the QP solver". QP hot start (setSolution plus basis, qp_allow_hot_start from 1.15) would be a separate feature.
6. **Fixed entities.** Report nonbasic_fixed, which needs a bulk bound read on each get_basis, or accept at_lower/at_upper, whose side follows the sign of the dual.
7. **Incidental.** On HiGHS 1.10.0, solver="ipm" on a QP returned Optimal with objective -4 instead of -0.5 (the Hessian seems ignored). It is outside the basis scope, and the wrapper never sets solver.
8. **The local main checkout is behind origin/main** (supported_versions, feature_unavailable_error). The implementation should start from origin/main.
