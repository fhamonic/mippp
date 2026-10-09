# Xpress LP basis survey (xpress_lp)

Survey date 2026-10-09. Releases: 47.01.01 (default, `$XPRESSDIR`) and 45.01.01
(pinned with `MIPPP_XPRESS_LIBRARY=/home/plaiseek/Softwares/solvers/xpressmp_old/lib/libxprs.so.45.01.01`,
current `XPAUTH_PATH`). Repo at `main` 04431ca, read only (clean at the end).

Tags: **MEASURED** = a probe run here, **DOCUMENTED** = vendor header/manual (path:line),
**INFERRED** = reasoning.
Manual = `/home/plaiseek/Softwares/solvers/xpressmp/docs/solver/MD/optimizerC.md` (47.01).
h47 = `/home/plaiseek/Softwares/solvers/xpressmp/include/xprs.h`, h45 = `/home/plaiseek/Softwares/solvers/xpressmp_old/include/xprs.h`.

## Files

- `probe.cpp`: the single probe (sections C D E F G H L P, n/m = NULL-array loads on a non-empty model,
  0-4 = zero-dimension models with NULL / dummy arrays).
  Models are built with MIP++ (`xpress_lp`), natives reached with `dlopen(api.library_path(), RTLD_NOLOAD)` + `dlsym`, never dlclosed.
- Outputs: `out47_<S>.txt` (47.01), `out45_<S>.txt` (45.01), `out47_C_msgs.txt` (full solver log for C),
  `diff_<S>.txt` (47 vs 45, log lines removed). `out47_n.txt`, `out47_m.txt`, `out45_n.txt` are the segfault runs;
  `out47_Z<k>.txt` / `out45_Z<k>.txt` the zero-dimension runs.
- Build: `flock .../survey/lock2 /opt/gcc-15/bin/g++ -std=c++26 -O0 -I/home/plaiseek/Projects/mippp/include probe.cpp -o probe -ldl`.
  Run: `MIPPP_NO_VERSION_WARNING=1 timeout -k 5 60 ./probe <S>`; every model gets `set_time_limit(10s)`.

45.01 vs 47.01: identical on every basis question (C, E, F, G, H, L, P diffs empty). D differs
only in the D6 time-limit timing and in LPSTATUS after addrows (both noted below). **MEASURED**

## A. Wrapper facts

- Class that would get the basis: `xpress_lp` only (`include/mippp/solvers/xpress/impl/v1/xpress_lp.hpp:14`,
  `class xpress_lp : public xpress_base`). There is no xpress QP class; `xpress_milp` is excluded by the
  "*_milp exposes pure MILP" ruling. Bases: `xpress_base : protected model_base<int, double>` (`xpress_base.hpp:35`).
- Not remapping: `remapping_model_base` is used by cplex/gurobi/highs only. No `remove_variable` /
  `remove_constraint` in the xpress tree (grep). Ids are `[0, count)`; `native_id(v) == v.id()`,
  `native_id(c) == c.id()` (`xpress_base.hpp:119-120`).
- Column id = `num_variables()` at creation (`_new_variables`, `xpress_base.hpp:242-251`; `_add_column`
  `:255-266`). Row id = `num_constraints()` at creation, `constraint(constr_id)` (`:329-339`, batch
  `:390-405`); `c.id()` is the Xpress row index (`_row_type` etc. `:422-437`).
- No `add_ranged_constraint` (has_ranged_constraints false). Ranged rows still exist: `_set_row_sides`
  (`:441-462`) writes type 'L' + `XPRSchgrhsrange`, which Xpress holds as 'R'; both sides infinite gives 'N'.
- `solve()` (`xpress_lp.hpp:82-90`): `XPRSlpoptimize(prob, nullptr)` (no flags: default algorithm, which is
  concurrent for a standalone LP, DOCUMENTED manual:20751), status from STOPSTATUS/LPSTATUS, then an
  unconditional `XPRSpostsolve` (an LP stopped presolved, 45.01 does, is put back to original state).
  No presolve, crossover, KEEPBASIS or LPFLAGS control is touched; the constructor sets OUTPUTLOG=0 and a
  message callback (`xpress_base.hpp:72-77`). On branch fix/review-rulings `solve()` also calls
  `reset_status()` and, on releases below 46.1.3 with a column whose bounds hold no value, returns
  `infeasible` WITHOUT calling `XPRSlpoptimize` (`git show fix/review-rulings:.../xpress_lp.hpp:82-88`).
- Pattern a getter follows (`xpress_lp.hpp:111-125`): `make_unique_for_overwrite<double[]>(num_constraints())`,
  one bulk native call `(prob, nullptr, data, 0, n-1)` through `check()`, wrapped in
  `constraint_mapping` / `variable_mapping`. The IIS snapshot (`xpress_base.hpp:587-743`) is sized with
  `num_variables()` / `num_constraints()` read after the native routine, filled by native index (== uid here).
- Versions: `validated_versions = {solver_version_range{{45, 1}, {47, 2}}}` (`xpress_api.hpp:288-289`);
  no `supported_versions`, no `probe_symbols`, no optional symbols. Every function in `XPRESS_FUNCTIONS`
  (`xpress_api.hpp:221-271`) is loaded with `lib.get_function` (`:276-277`), i.e. required: a missing symbol
  refuses the library. The two-tier mechanism (`supported_versions`, `lib.find_function` optional symbols,
  `feature_unavailable_error`) exists only on branch fix/review-rulings (commit 00432a9), used by HiGHS
  (`HIGHS_OPTIONAL_FUNCTIONS`, `highs_api.hpp:336-353` there; throw in `highs_base.hpp:747-760` there).
- `check()` -> `xpress_api::_check` (`xpress_api.hpp:309-319`): non-zero rc -> `solver_error(getlasterror)`
  (license codes -> `license_error`).
- `XPRS_LPITERLIMIT = 8007` is declared (`xpress_api.hpp:150`) but xpress has no `set_iteration_limit`
  (has_iteration_limit, `model_concepts.hpp:313`, is false) and no iteration-count accessor.

## B. Native API

| symbol | signature | api header | 45.01 | 47.01 |
|---|---|---|---|---|
| XPRSgetbasis | `int XPRSgetbasis(XPRSprob, int rowstat[], int colstat[])` | not declared, must be added | h45:2406, dlsym found | h47:2472, dlsym found |
| XPRSloadbasis | `int XPRSloadbasis(XPRSprob, const int rowstat[], const int colstat[])` | not declared, must be added | h45:2340, found | h47:2406, found |
| XPRSgetbasisval | `int XPRSgetbasisval(XPRSprob, int row, int col, int *p_rowstat, int *p_colstat)` (per entity, read only) | not declared | h45:2408 | h47:2474 |
| XPRSgetpresolvebasis | `(XPRSprob, int rowstat[], int colstat[])` | not declared | h45:2456 | h47:2524 |
| XPRSloadpresolvebasis | `(XPRSprob, const int[], const int[])` | not declared | h45:2322 | h47:2388 |

(DOCUMENTED for the line numbers, MEASURED for dlsym on both libraries.) There is no per-entity setter in
either header (DOCUMENTED by absence); loading is bulk only. Both getbasis arrays "may be NULL"
(manual:9617-9628) but loadbasis with a NULL array for a NON-EMPTY dimension segfaults (MEASURED on 47.01 both
arrays, 45.01 rowstat; `out47_n/m.txt`, `out45_n.txt`). A NULL array for a zero-length dimension is fine:
a model with columns and no rows, and an empty model, take getbasis/loadbasis with NULL (what
`std::vector<int>().data()` gives in libstdc++) or a dummy buffer, rc 0, solve unaffected (MEASURED both
releases, `out*_Z0..4.txt`), so `vector::data()` needs no dummy-buffer rule. The pair has existed far longer than the validated range, so both go into
`XPRESS_FUNCTIONS` as required symbols; no feature_unavailable path is needed for 45.1-47.2 (INFERRED for 46.x,
not installed).

Codes (DOCUMENTED h47:2065-2068, h45:2028-2031; manual:9617-9628, 13680-13690):
`XPRS_BASISSTATUS_NONBASIC_LOWER 0`, `BASIC 1`, `NONBASIC_UPPER 2`, `SUPERBASIC 3`. Rows: the status of the
"slack, surplus or artificial" of the row. Columns: 0 = "non-basic at lower bound, or superbasic at zero if the
variable has no lower bound". Arrays are sized `ORIGINALROWS` (1124) / `ORIGINALCOLS` (1214) (h47:580,590);
"The basis information is always for the original (unpresolved) model" (manual:9646). Errors: 140 basis lost
(warning, manual:29786), 266 cannot read basis for presolved problem (:29876), 410/411 invalid row/column basis
type (:29928-29931), 429 no basis is available (:29964).

## C. Row side convention (MEASURED, identical on 45.01 and 47.01, every algorithm)

Model (`small_lp()` in probe.cpp, min, unique optimum -17.5 except the free x6). Read with presolve on and
off, LPFLAGS dual / primal / barrier+crossover; codes identical in all six runs (`out47_C.txt`):

| row | type | rhs | range | activity | slack (XPRSgetslacks) | dual | code |
|---|---|---|---|---|---|---|---|
| r1 a.x <= 8, tight | L | 8 | | 8 | 0 | -0.25 | 0 |
| r2 a.x >= 2, tight | G | 2 | | 2 | 0 | 0.75 | 2 |
| r3 == 7, dual > 0 | E | 7 | 0 | 7 | 0 | 1 | 2 |
| r4 1 <= a.x <= 6, at upper | R | 6 | 5 | 6 | 0 | -2 | 0 |
| r5 <= 100 slack | L | 100 | | 9 | 91 | 0 | 1 |
| r7 5 <= a.x <= 12, at lower | R | 12 | 7 | 5 | 7 | 1 | 2 |
| r8 == 4, dual < 0 | E | 4 | 0 | 4 | 0 | -1 | 0 |
| r9 free | N | 0 | | 9 | -9 | 0 | 1 |
| r10 >= 1 slack | G | 1 | | 8 | -7 | 0 | 1 |

| column | lb | ub | x | dj | code |
|---|---|---|---|---|---|
| x3 at upper | 0 | 4 | 4 | -2 | 2 |
| x4 fixed, dj > 0 | 2 | 2 | 2 | 3 | 0 |
| x12 fixed, dj < 0 | 1 | 1 | 1 | -3 | 2 |
| x6 free, nonbasic, no rows | -inf | inf | 0 | 0 | 3 |
| x7 at lower | 0 | 10 | 0 | 2 | 0 |
| x13 at upper, lb -inf | -inf | 5 | 5 | -1 | 2 |
| x14 at lower -3 | -3 | inf | -3 | 1 | 0 |
| x1 x2 x5(free) x8 x10 x11 basic | | | | 0 | 1 |

Conclusion: slack s = rhs - a.x for every row type (r10: 1 - 8 = -7; r9: 0 - 9 = -9; r7: 12 - 5 = 7), with
s in [0,inf) for L, (-inf,0] for G, [0,0] for E, [0,range] for R, free for N. The row code names the side of
s, which is the OPPOSITE side of the activity: code 0 = s at its lower = activity at its UPPER side; code 2 =
s at its upper = activity at its LOWER side. Columns: code names the value's side.
#basic == ROWS after every optimal solve (10/10 here, 200/200 on the dense model).

Codes asked for: fixed column 0 or 2 by the sign of dj; == row 0 or 2 by the sign of its dual (measured on a
minimization: 0 when dual < 0, 2 when dual > 0, i.e. the side an inequality with that dual would be tight at;
the sign convention of a maximization was not measured); nonbasic free column 3
(superbasic at 0, not 0 as the manual allows); basic 1. N rows are basic. Code 3 also appears on BOUNDED
columns strictly inside their bounds after a stop: LPITERLIMIT=0 gives all 200 columns code 3 at x ~ 200;
5 and 20 iterations leave 2 and 1 such columns (`out47_L.txt`, XPRSgetlpsol values; same on 45.01).

READ mapping (native -> tag), activity side for rows, value side for columns:

- column: 1 basic; 0 nonbasic_at_lower_bound; 2 nonbasic_at_upper_bound; 3 nonbasic_free (superbasic).
  A 0 on a column without lower bound (manual: "superbasic at zero") was never returned: Xpress itself
  reports free columns as 3 and lb=-inf columns at upper as 2, even in the pre-solve slack basis (D1).
- row: 1 basic; 0 nonbasic_at_upper_bound; 2 nonbasic_at_lower_bound; 3 nonbasic_free.
  E rows come back 0/2 by dual sign. 0 on a G row, 2 on an L row, 0/2 on an N row were never returned
  (load repairs them, see E).
- nonbasic_fixed is NOT a native code: reporting it needs the bounds (XPRSgetlb/getub bulk for columns,
  XPRSgetrowtype for E rows: 3 extra bulk calls). Otherwise fixed columns / E rows read at_lower / at_upper by
  the sign of their dj / dual.

WRITE mapping (tag -> native):

- column: basic 1; nonbasic_at_lower_bound 0; nonbasic_at_upper_bound 2; nonbasic_fixed 0 (0 and 2 both
  accepted, 0 iterations either way, H4/P); nonbasic_free 3.
- row: basic 1; nonbasic_at_upper_bound 0; nonbasic_at_lower_bound 2; nonbasic_fixed 0 (E row 0 and 2
  both 0 iterations, H4/P); nonbasic_free 3.
- No row type or bound is needed to write: a code on an infinite side is repaired at load (E below).
- Cannot be written: a superbasic VALUE (loadbasis has no value array; 3 is accepted, the value is the
  solver's). Abstract tags (nonbasic, nonbasic_at_bound) have no code of their own; 0 is the safe choice
  since Xpress repairs it on lb=-inf columns.

## D. Availability (MEASURED, 47.01 = 45.01 unless noted)

| state | getbasis rc | what comes back | KEEPBASIS after |
|---|---|---|---|
| before any solve (D1) | 0 | slack basis: rows 1, columns 0, free columns 3, lb=-inf columns 2 | 0 |
| empty model (D1b) | 0 | nothing | 0 |
| optimal, simplex (C, D0) | 0 | optimal basis, #basic == ROWS | 1 |
| optimal, barrier + default crossover (C, D8) | 0 | crossover basis == simplex basis | 1 |
| optimal, barrier CROSSOVER=0 (D8, D8s, H1, H1b) | **429** "No basis is available", arrays untouched | none, and a previous simplex basis is LOST | 0 (stays 1 if it was 1, H1b) |
| infeasible, presolve proves it, fresh model (D3) | 0 | slack basis (ALGORITHM=1, SIMPLEXITER 0) | 0 |
| infeasible, presolve proves it, re-solve after an optimal one (H2) | 0 | the STALE optimal basis of the previous solve | 1 |
| infeasible, PRESOLVE=0 (D3) | 0 | phase-1 basis, #basic == ROWS | 1 |
| unbounded, presolve (D4) | 0 | slack basis | 0 |
| unbounded, PRESOLVE=0 (D4) | 0 | dual simplex basis (here the slack basis, 0 its) | 1 |
| LPITERLIMIT=5 stop (D5) | 0 | current simplex basis, #basic == ROWS, may hold 3 on bounded columns | 1 |
| LPITERLIMIT=0 stop = presolve only (D5) | 0 | rows 1, every column 3 (superbasic at the crash point) | 1 |
| time limit 1 ms, n=400 (D6) | 0 | 47.01: stopped before the LP (ALGORITHM 1), slack basis; 45.01: LP started, rows 1 / columns 3 | 47: 0, 45: 1 |
| after add row, no re-solve (D7) | 0 | old basis + new row 1, #basic == ROWS | 1 |
| after add column (D7 via add_variable; F1 via add_column) | 0 | old basis + new column 0 | 1 |
| after bound change / rhs change (D7) | 0 | unchanged | 1 |

- Re-solving after each D7 modification takes 0 iterations (KEEPBASIS warm start).
- getbasis works while the LP is still presolved (native lpoptimize stopped, PRESOLVESTATE bit 1 set): it
  returns the original-space basis, identical to the one after XPRSpostsolve (D5n). getpresolvebasis gave
  the same arrays here (nothing was removed by presolve on that model).
- `#basic == ROWS` held after every optimal solve and every stop (C, D, E, F); not applicable after 429.
  DOCUMENTED: KEEPBASIS=2 calls a basis valid when "the number of basic variables must match the number of
  rows" (manual:22564-22585).
- getbasis itself has one error only, 429 (barrier without crossover). The slack basis before a solve,
  the slack basis after presolve proved infeasibility/unboundedness or a stop before simplex, and the stale
  basis after a presolve-proved re-solve all come back with rc 0. KEEPBASIS==0 and ALGORITHM==1 flag the
  fresh cases but not the stale one (H2: KEEPBASIS 1, ALGORITHM 2). DOCUMENTED: KEEPBASIS "will be
  automatically set to 1 after a solve that produced a valid basis" (manual:22566).
- A native flag that does separate them: `PRESOLVESTATE` (1026) bit 7, `XPRS_PRESOLVESTATE_SOLUTIONVALID`
  (`1<<7`, h47:1656, h45:1630, manual:28986 "Solution in memory is valid"). MEASURED on both releases:
  bit 7 CLEAR before any solve (0x280041), after presolve proved infeasible/unbounded (0x21), after the 47.01
  time-limit stop before the LP (0x21) and in the stale H2 case (0x21); bit 7 SET after every simplex result
  (optimal, LPITERLIMIT stops including LPITERLIMIT=0, the 45.01 time-limit stop, PRESOLVE=0 infeasible and
  unbounded) and after barrier without crossover (0x1500a1, where getbasis is 429). So
  `getbasis rc == 0 && (PRESOLVESTATE & 0x80)` means "the basis the last solve produced". Caveat, also
  measured: any modification clears the bit (D7: 0x2c0441 after add row, 0x2c0461 after a bound, rhs or
  column change) although the basis is kept and warm-starts the next solve in 0 iterations; the bit after
  a bare loadbasis was not measured.
- With default controls an optimal LP always ends with a basis: concurrent picks a simplex, and barrier
  crosses over by default on LPs (manual:20450-20470 CROSSOVER default -1; "no crossover on QP/MIQP and after
  the hybrid gradient"). INFERRED from the doc plus measured barrier runs.

## E. Warm start (MEASURED on the dense 200x200 IterationLimitTest model, cold = 118 dual iterations)

Knob for the iteration limit: `XPRS_LPITERLIMIT` (8007) through `setintcontrol`; iterations read from
`XPRS_SIMPLEXITER` (1009). Fresh model B built through MIP++ in the same order, A's arrays loaded with
XPRSloadbasis (rc 0, KEEPBASIS becomes 1, DOCUMENTED manual:22566):

| B setting | status | SIMPLEXITER | LPOBJVAL |
|---|---|---|---|
| cold, LPITERLIMIT=0 | iteration_limit | 0 | 19217474 (crash point) |
| load, LPITERLIMIT=0 | **iteration_limit** (LPSTATUS 4) | 0 | 321053.874 = optimum |
| load, LPITERLIMIT=1 | optimal | 0 | optimum |
| load, unlimited | optimal | 0 | optimum |
| load, PRESOLVE=0, LPITERLIMIT=0 | iteration_limit | 0 | optimum |
| load, LPFLAGS primal, LPITERLIMIT=0 | iteration_limit | 0 | optimum |
| load, LPFLAGS dual, LPITERLIMIT=0 | iteration_limit | 0 | optimum |
| load, LPFLAGS barrier | optimal (barrier ran, crossover) | 0, BARITER 12 | optimum |
| load, then KEEPBASIS=0 | optimal | 118 (basis ignored) | optimum |

The basis is used under the wrapper's defaults (presolve on, no LPFLAGS); no presolve-off, crash-off or
factorization call is needed. With LPITERLIMIT=0 the solver stops before testing optimality, so a loaded
optimal basis reports iteration_limit; LPITERLIMIT >= 1 reports optimal with 0 iterations. Small model
(E3b): loaded basis (with fixed columns, E rows, R rows, N row, free column 3) gives the optimum at 0
iterations with presolve on and off. The basis after the solve equals the loaded one.

Changed bound: after A's optimal solve, halving the upper bound of a basic column and re-solving takes 13
iterations whether A's basis is reloaded (E5) or not (E4, KEEPBASIS auto): the basis is kept, not dropped.
Loading into fresh B then raising a nonbasic column's lower bound to 1e-3 gives 0 iterations (E6).
getbasis right after loadbasis returns the loaded codes (after the load-time repairs below).

Invalid bases (small model, PRESOLVE=0, warnings captured with OUTPUTLOG=1):

| loaded | loadbasis | getbasis right after | solve |
|---|---|---|---|
| one basic too many (x3 2->1) | rc 0, silent | another basic column (x11) demoted to 0 | optimal, 1 iteration |
| one basic too few (x1 1->0) | rc 0 | kept, #basic 9 | warning 140 "Basis lost - recovering", optimal, 1 iteration |
| 2 on x14 (ub = +inf) | rc 0 | repaired to 0 | optimal, 0 |
| 0 or 2 on free x6 | rc 0 | repaired to 3 | optimal, 0 |
| 0 on x13 (lb = -inf, ub 5) | rc 0 | repaired to 2 | optimal, 0 |
| 2 on L row r1 | rc 0 | repaired to 0 | optimal, 0 |
| 0 on G row r2 | rc 0 | repaired to 2 | optimal, 0 |
| 0 on N row r9 (count 9) | rc 0 | repaired to 3 | optimal, 0, no warning |
| 2 on N row + x4 basic (count 10) | rc 0 | N row 3 | warning 140, 1 iteration |
| 3 on bounded x7 | rc 0 | kept 3 | optimal: dual 0, primal 1, default 0 iterations |
| 3 on fixed x4, 3 on L row r1 | rc 0 | kept 3 | optimal, 0 |
| all-slack basis | rc 0 | kept | optimal, 6 (= cold) |
| 7 or -1 on a column | **rc 411** + message; KEEPBASIS reset to 0 | basis PARTIALLY overwritten (entries from the bad one on reset to 0, #basic 7) | optimal, 6 (cold) |
| 7 on a row | **rc 410**; KEEPBASIS 0 | reset to the slack basis | optimal, 6 (cold) |
| NULL rowstat or colstat | **segfault** (47.01 both, 45.01 rowstat) | | |

Wrong side on a BOXED entity (P, both releases): flipping a ranged row, a [0,4] / [0,10] column, a fixed
column or an E row gives 0 iterations and the optimum under dual simplex (the wrapper's path once a basis is
loaded: KEEPBASIS on -> dual, manual:20751), presolve on or off. Only primal simplex (LPFLAGS=2) shows it:
1-2 iterations for the ranged rows and bounded columns, 0 for the fixed column and the E row.

## F. Partial / extended bases (MEASURED)

- Only full arrays: XPRSloadbasis takes ORIGINALROWS + ORIGINALCOLS entries (manual:13680-13690), no
  subset or per-entity setter exists, NULL crashes. A subset update = read the full basis, overlay, load.
- A basis taken before columns/rows were added, extended with new column 0 and new row 1 (the manual's rule,
  manual:13707-13710), loads with rc 0 and the solve takes 0 iterations: small model with a new column via
  add_column and a new slack row (F1, LPITERLIMIT=0 -> optimum value, unlimited -> optimal 0 its), dense
  model with a new slack row (F2), fresh dense model with one extra column and row (F3).
- Without loading, Xpress itself extends the held basis that way (D7: new row 1, new column 0).
- The manual rule for a new column with lb = -inf and finite ub is 2 (manual:13708); writing 0 there is
  repaired to 2 at load (measured on x13), and 0 on a free column to 3.

## G. Gotchas

1. getbasis never fails except with 429 after barrier without crossover; before any solve it returns the
   slack basis, and after a re-solve that presolve ends (infeasible/unbounded) it returns the previous
   solve's basis (stale), rc 0 in both cases (D1, D3, D4, D6, H2). PRESOLVESTATE bit 7 tells these apart
   from a basis the last solve produced, and clears on any modification (D).
2. Presolved state: XPRSloadbasis refuses a presolved problem with 266 (D5n, DOCUMENTED manual:13706).
   The wrapper's unconditional postsolve after `solve()` keeps the problem original, so set_basis after a
   stopped solve works; getbasis is fine in either state.
3. KEEPBASIS is automatic: 1 after a solve that produced a basis, 1 after loadbasis, 0 after a failed load,
   0 after barrier without crossover on a fresh model. Every re-solve warm-starts from the held basis
   (G2: second solve 0 iterations). Explicit KEEPBASIS=0 discards a loaded basis (E2: 118 iterations).
4. LPFLAGS, not DEFAULTALG, picks the standalone LP algorithm on 45/47: DEFAULTALG=3 or 4 still ran
   "concurrent solve with dual" (out47_C_msgs.txt). With KEEPBASIS on the default algorithm is dual
   simplex (DOCUMENTED manual:20751).
5. A loaded optimal basis with LPITERLIMIT=0 reports iteration_limit, not optimal (E2).
6. The basis survives modifications: rows/columns added after loadbasis are appended basic / at lower
   (G1: load, add row, LPITERLIMIT=0 -> the optimum). Changing the objective sense does not touch it (G3).
7. Load-time repairs are silent: wrong count (one extra basic), codes on infinite sides. A side error on a
   boxed entity is invisible under dual simplex. A count short by one gives warning 140, which the wrapper's
   OUTPUTLOG=0 hides.
8. A rejected load (410/411) is not transactional: the held basis is partially overwritten and KEEPBASIS
   reset to 0, so the next solve is cold.
9. Superbasic (3) on bounded columns after a stop, with values strictly inside the bounds (L).
10. getsolution after an iteration-limit stop returns "not available" (status 0, array untouched);
    XPRSgetlpsol does return the current point (L). Irrelevant to the basis itself.
11. On fix/review-rulings, `solve()` on a release below 46.1.3 with a column whose bounds hold no value
    returns `infeasible` without solving: the held basis is whatever was there before.
12. LPSTATUS after addrows: 47.01 resets it to 0 (unstarted), 45.01 keeps 1; the wrapper's cached status
    stays optimal in both. Not a basis question.
13. No callbacks on xpress_lp; the problem object is not thread-safe (INFERRED, generic).

## H. Recommendation

- Classes: `xpress_lp` only.
- api: add `XPRSgetbasis` and `XPRSloadbasis` to `XPRESS_FUNCTIONS` as required symbols (present in 45.01 and
  47.01, MEASURED), plus their declarations in the `#else` branch of `xpress_api.hpp` and the four
  `XPRS_BASISSTATUS_*` constants. No supported_versions / feature_unavailable path needed.
- Column status variant, alternative 0 first so that columns added after `get_basis()` read as the status
  the manual prescribes for new columns:
  `std::variant<nonbasic_at_lower_bound, basic, nonbasic_at_upper_bound, nonbasic_free>`.
- Row status variant, alternative 0 = basic so new rows read basic:
  `std::variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_free>`.
- Read mapping as in C (rows flipped: 0 -> at_upper, 2 -> at_lower). Without nonbasic_fixed (it would cost
  three bulk reads and hide the dj/dual sign Xpress keeps); with it if the maintainer wants it uniform
  across backends.
- get_basis: two `std::vector<int>` (or one buffer) sized `num_constraints()` / `num_variables()`
  (== ORIGINALROWS/COLS whenever the wrapper returns, since solve() postsolves), one `XPRSgetbasis` through
  `check()` (429 -> solver_error), translate into two `handle_status_table`s keyed by native index (== uid),
  return a snapshot by value like `iis_snapshot`.
- set_basis: build full arrays from the current `num_constraints()` / `num_variables()`; entries past the
  snapshot's id bound take the snapshot's alternative 0 (row 1, column 0); write mapping as in C; one
  `XPRSloadbasis` through `check()`. Nothing else: no presolve/crash/flags/factorize call, loadbasis sets
  KEEPBASIS=1 itself, and the problem is never presolved between wrapper calls.
- Deviations to rule on:
  1. Xpress code 3 is "superbasic", reported on bounded columns strictly between their bounds after a stop
     and on free nonbasic columns at optimum. The draft has no superbasic tag: either widen
     `nonbasic_free` to "nonbasic, not at a bound" or add `basis_status::superbasic : nonbasic`.
  2. `nonbasic_fixed` is not native: report it only by deriving it from bounds (extra reads) or drop it from
     the read variant on Xpress; on write it maps to 0 (both sides are accepted).
  3. XPRSgetbasis alone cannot tell a real basis from the slack basis (before a solve, after presolve proves
     infeasible/unbounded on a fresh model, after a stop before the simplex) or from a stale one (after a
     presolve-proved re-solve): only barrier without crossover errors (429). PRESOLVESTATE bit 7 can key an
     availability check natively, but it clears on any model change even though the held basis stays a valid
     warm start: the maintainer must say whether "available" means "produced by the last solve of this exact
     model" (bit 7) or "usable as a warm start" (anything getbasis returns with rc 0), or leave it documented
     as unspecified.
  4. A superbasic value cannot be written (loadbasis takes codes only), so `set_nonbasic(v, s)` with s inside
     the bounds can only write 3; and snapping s needs the bounds, which a detached snapshot does not hold
     (read them at set_basis time from the target model, or store them at get time).
  5. A solver-agnostic warm-start test cannot rely on iteration counts here (no accessor, no
     set_iteration_limit on xpress) and a wrong side on a boxed entity is invisible under dual simplex: the
     row-side convention must be pinned by READ checks on a model with known tight sides.

## Probes (all in probe.cpp, one section per run)

| section | purpose | result |
|---|---|---|
| C | row/column side convention, 6 algorithm/presolve variants | slack = rhs - a.x; row code names the slack's side (0 = activity at upper, 2 = activity at lower); same codes in every variant and on 45.01 |
| D | availability per state | rc 0 everywhere except 429 after barrier w/o crossover; slack basis before solve / presolve-proved; stale after presolve-proved re-solve; PRESOLVESTATE bit 7 separates them |
| E | warm start, changed bound, invalid bases | loaded basis used under default settings (0 its); LPITERLIMIT=0 reports iteration_limit; infinite-side codes and an extra basic silently repaired at load; missing basic -> warning 140; codes outside 0..3 -> 410/411 with a clobbered basis |
| F | extended bases | new column 0 / new row 1 -> 0 iterations; full arrays only |
| G | load then modify; KEEPBASIS auto; sense change | basis survives modifications; second solve 0 its |
| H | stale basis after non-basis solves, valid-side flips, superbasic values | barrier w/o crossover drops the basis (429); E-row/fixed sides interchangeable |
| L | values of code-3 columns after stops | genuinely superbasic (x ~ 200, bounds [0, inf)) |
| P | wrong-side writes under dual vs primal | invisible under dual (0 its), 1-2 its under primal |
| n, m | NULL rowstat / colstat on a non-empty model | segfault |
| 0-4 | zero-dimension models, NULL vs dummy arrays | NULL fine for a zero-length dimension |

## Open questions for the maintainer

1. Code 3 (superbasic) on bounded columns: widen `nonbasic_free` or add a `superbasic` tag?
2. `nonbasic_fixed`: derive it from bounds (3 bulk reads) or leave it out of the Xpress read variant?
3. Availability: PRESOLVESTATE bit 7 ("produced by the last solve of this model") vs anything getbasis returns
   ("usable warm start") vs documented-unspecified; 429 -> solver_error?
4. `set_nonbasic(v, s)`: snapping needs bounds the snapshot does not hold; a superbasic value cannot be
   written on Xpress.
5. Solver-agnostic warm-start test: no iteration accessor or iteration limit on xpress; side errors on boxed
   entities are invisible under dual simplex.
6. 46.x not installed, not measured (INFERRED to behave like 45.01 and 47.01).

## Addendum (implementation, 2026-10-09)

- Bit 7 after a bare XPRSloadbasis, MEASURED on 45.01 and 47.01: on a fresh
  model PRESOLVESTATE stays 0x680041 (bit clear), so get_basis() throws; on a
  solved model it goes 0x1400a1 -> 0xa1 (bit set) and XPRSgetbasis returns
  the arrays just loaded, so get_basis() returns the written basis.
- XPRSloadbasis with an empty colstat (null data()) on a model without
  columns returns 0. An all-basic basis (100 basic over 50 rows) is repaired
  at load.
