# Clp LP basis survey (backend `clp`, class `clp_lp`)

Evidence tags: **MEASURED** (probe run here), **DOCUMENTED** (vendor header or source, path:line), **INFERRED**.

Probe source: `probe_basis.cpp` (this directory), built with the shared lock, run as
`MIPPP_NO_VERSION_WARNING=1 timeout -k 5 60 ./probe_basis`.
Outputs: `out_devel.txt` (coinbrew devel, `Clp_Version()` unparsable, so "untested" tier),
`out_1.17.4.txt`, `out_1.17.6.txt`, `out_1.17.8.txt`, `out_1.17.10.txt`, `out_1.17.11.txt`
(cached conda releases, pinned with `MIPPP_CLP_LIBRARY=<.../libClp.so>` and
`LD_LIBRARY_PATH=<clp lib>:<coin-or-utils-2.11.13 lib>:$LD_LIBRARY_PATH`, the same pairing
misc/tools/compat_matrix.py uses). **All six outputs are identical** apart from the library-path
and symbol-address lines (MEASURED, `diff` after stripping lines 1-2).

Source paths below: `W/` = /home/plaiseek/Projects/mippp/include/mippp/solvers/clp/impl/v1/,
`H` = /home/plaiseek/Softwares/solvers/coinbrew/dist/include/coin-or/Clp_C_Interface.h (devel),
`HR` = .compat-cache/extracted/_deps/coin-or-clp-1.17.{4,11}-*/include/coin/Clp_C_Interface.h,
`S/` = /home/plaiseek/Softwares/solvers/coinbrew/Clp/src/ (devel sources, commit 75ebf2f5, 2025-04-24).

## A. Wrapper facts

- Only one class: `clp_lp : protected model_base<int, double>` (W/clp_lp.hpp:29). No clp_milp or clp_qp, so the basis goes on clp_lp only.
- No remapping: `native_id(variable) / native_id(constraint)` return `id()` (W/clp_lp.hpp:103-104). Not a remapping_model_base.
- Variables can be removed, constraints cannot. `remove_variable` (W/clp_lp.hpp:273-289) keeps the Clp column: it sets obj 0 and bounds [0,0], zeroes its coefficients and pushes the id onto `_free_variable_ids`. The next `add_variable` / `add_column` reuses that id (180-191, 196-198, 240-247). So native column count = `Clp_getNumCols` ≥ `num_variables()`. Tables must be sized by `getNumCols`, as `_snapshot_variables` already is (633-638). A removed column reads code 5 isFixed after a re-solve (MEASURED D8). There is no remove_constraint.
- Constraint handle → row: `constraint(id)` where id is the row index at creation (336, 403-417, 476). Rows are never deleted, so id == native row.
- `solve()` (605-624):
  - returns early when `num_variables()==0`, so Clp is never called and the status array may never exist (MEASURED D9);
  - drops and restores the scaling factors (`Clp_scaling(0)` then the old mode, 615-617), which keeps the basis (comment at 610-614; MEASURED E3, D6c: 0-iteration re-solves);
  - arms `Clp_setMaximumSeconds` (618-619);
  - runs `Clp_primal(model, 0)` (620): primal simplex, ifValuesPass = 0, no presolve, no dual, no barrier, no crossover, no reload;
  - maps the status from `Clp_status / Clp_secondaryStatus / Clp_primalFeasible` (621-623; `_simplex_status` 577-597).
  - On origin/main only (not on local main 04431ca, which is 13 commits behind), `solve()` starts with `reset_status()`.
- Pattern a getter follows: `get_dual_solution` (644-649) copies `Clp_dualRowSolution`, n = `num_constraints()`, into a `unique_ptr<double[]>` and wraps it in a `constraint_mapping`. `get_reduced_costs` / `get_solution` copy `getNumCols` doubles via `_snapshot_variables` (633-652). Everything is copied out by value; Clp's arrays are never exposed.
- api (W/clp_api.hpp:160-178):
  - `validated_versions = {[1.17.4, 1.17.12)}` (169-170). No `supported_versions` is declared, neither locally nor on origin/main.
  - Every function is required: `lib.get_function` (157-158). There is no optional macro and no `probe_symbols`.
  - The optional-symbol template is HiGHS: `HIGHS_OPTIONAL_FUNCTIONS` + `lib.find_function` (null when absent), checked against nullptr at the call site, which throws `feature_unavailable_error` (highs_api.hpp:335-345). `feature_unavailable_error` exists on origin/main (00432a9, utility/solver_exceptions.hpp) but not on local main.
  - Header mode: with `MIPPP_INCLUDE_CLP_HEADER` the prototypes come from `coin/Clp_C_Interface.h` (clp_api.hpp:5-7). Otherwise they are hand-declared (19-90).

## B. Native API

All of these exist in every cached release (1.17.4, 1.17.6, 1.17.8, 1.17.10, 1.17.11) and in devel. MEASURED: `nm -D --defined-only` on each libClp.so, and dlsym in the probe. They are declared in HR at the same lines in 1.17.4 and 1.17.11.

| symbol | signature | in clp_api? | header |
|---|---|---|---|
| Clp_getColumnStatus | `int (Clp_Simplex*, int sequence)` | yes: W/clp_api.hpp:66, 133 | H:284, HR:283 |
| Clp_getRowStatus | `int (Clp_Simplex*, int sequence)` | yes: 68, 135 | H:286, HR:285 |
| Clp_setColumnStatus | `void (Clp_Simplex*, int sequence, int value)` | yes: 65, 132 | H:288, HR:287 |
| Clp_setRowStatus | `void (Clp_Simplex*, int sequence, int value)` | yes: 67, 134 | H:291, HR:290 |
| Clp_statusExists | `int (Clp_Simplex*)` | **no, add** | H:276, HR:275 |
| Clp_statusArray | `unsigned char* (Clp_Simplex*)`, `char[numberRows+numberColumns]` | **no, add** | H:278, HR:277 |
| Clp_copyinStatus | `void (Clp_Simplex*, const unsigned char*)` | **no, add** | H:280, HR:279 |
| Clp_setMaximumIterations | `void (Clp_Simplex*, int)` | no (only needed for an iteration limit or tests) | H:199, HR:198 |
| Clp_numberIterations | `int (Clp_Simplex*)` | no (same) | H:195, HR:194 |
| iteration getter | devel: `Clp_maximumIterations` (H:198); 1.17.x: **unprefixed `maximumIterations`** (HR:197) | no | |

Because every needed symbol exists across the whole validated range, all of them can be required symbols (`get_function`). No `feature_unavailable_error` is needed (INFERRED from the nm survey).

Codes (DOCUMENTED H:281-282 and S/ClpSimplex.hpp:116-123): 0 isFree, 1 basic, 2 atUpperBound, 3 atLowerBound, 4 superBasic, 5 isFixed.

- Storage: one byte per entity, **columns first, then rows** (`status_[seq]` for columns, `status_[seq + numberColumns_]` for rows). The low 3 bits are the code; the getters mask `& 7` (S/ClpSimplex.hpp:1506-1525). Higher bits are internal flags: fake bound (bits 3-4, S/ClpSimplex.hpp:1496-1505), pivoted 32, perturbed 128.
- C setters (S/Clp_C_Interface.cpp:1179-1201): they ignore values outside 0..5. For 3/5 they also write the lower bound into the primal solution array, and for 2 the upper bound.
- `copyinStatus` deletes the array and copies exactly numberRows+numberColumns raw bytes, or sets the array to NULL when given nullptr (S/ClpModel.cpp:3947-3956).
- `statusExists` tests for a non-NULL array (S/ClpModel.hpp:901-904).

## C. Row side convention: **ACTIVITY** (no flip). MEASURED

The LP is in probe_basis.cpp `build()`. Its unique, primal-nondegenerate optimum is obj -15, 7 rows and 7 basics. The only dual degeneracy is on purpose: the empty free column y.

| entity | situation | code |
|---|---|---|
| x1 [0,2] | nonbasic at upper, dj = -2 | 2 atUpper |
| x2 [1,1] | fixed, dj = +2 | 5 isFixed |
| x7 [2,2] | fixed, dj = -3 | 5 isFixed |
| x10 [0,inf) | at lower, dj = +1 | 3 atLower |
| y free, empty column | nonbasic, dj = 0 | 0 isFree |
| x3, x4, x6, x8, x9 (bounded), x5 (free) | basic | 1 |
| row `x3+x1 <= 4` | activity 4 = up | **2 atUpper** |
| row `x4+x2 >= 3` | activity 3 = lo | **3 atLower** |
| row `x5+x1 == 5` (dual +2) and `x8+x1 == 7` (dual -2) | equality | **5 isFixed** (both dual signs) |
| ranged row `1 <= x6 <= 4` | activity at up | 2 atUpper |
| ranged row `2 <= x9 <= 6` | activity at lo | 3 atLower |
| slack row | basic | 1 |

The same codes come back when the LP is solved as max(-obj). The codes name the side of the row activity a·x, not of a slack. This is corroborated by DOCUMENTED S/Clp_C_Interface.cpp:1191-1200: `Clp_setRowStatus` puts the row activity at rowLower for 3/5 and at rowUpper for 2. Also by S/ClpSimplex.cpp:1639-1700: `internalFactorize` sets `rowActivityWork_` from the status the same way.

**Read mapping (columns and rows alike):**
- 1 → basic
- 3 → nonbasic_at_lower_bound
- 2 → nonbasic_at_upper_bound
- 5 → nonbasic_fixed
- 0 → nonbasic_free
- 4 superBasic → no drafted tag. Recommendation: the bare `nonbasic` (see H). Never observed in wrapper flows.
- 6/7 → only reachable by a raw native `copyinStatus`; documented UB.

**Write mapping:**
- basic → 1
- nonbasic_at_lower_bound → 3
- nonbasic_at_upper_bound → 2
- nonbasic_fixed → 5
- nonbasic_free → 0

Every tag can be written, and Clp repairs each one at the next solve if it contradicts the bounds (see E and W below).

After a solve, a fixed column and an == row always read 5, whatever side was written and whatever the dual sign (MEASURED W5-W9 and the main LP). Clp never names a side for a fixed entity.

## D. Availability (MEASURED unless noted)

- **Before any solve:** `statusExists = 0`, `statusArray = NULL`, `Clp_status = -1`. `Clp_getColumnStatus`, `Clp_getRowStatus`, `Clp_setColumnStatus` and `Clp_setRowStatus` all **crash with SIGSEGV** (forked children). The C functions do not test for NULL (S/ClpSimplex.hpp:1506-1525).
- **After an optimal solve:** the array exists and #basic == #rows. MEASURED on the 7-row LP and on a random 40x60 LP. INFERRED: guaranteed. `internalFactorize` caps basics at `maximumBasic_` (DOCUMENTED S/ClpSimplex.cpp:1710). That a short basis is filled with slacks during factorization is INFERRED; it agrees with E5b and E5e.
- **Infeasible LP** (detected at start, 0 iterations): the array exists, #basic = #rows, and it holds the starting slack basis.
- **Unbounded LP:** the array exists, #basic = #rows, and it holds the last basis (2 iterations).
- **Iteration limit 1/3/5/15** (native `Clp_setMaximumIterations`, the wrapper reports limit_reached): the array exists, #basic = #rows, and it holds the basis of the last iterate. That point is primal infeasible (composite objective -1.8e11). Free columns read 0. No superbasic code was seen.
- **Time limit 0 s** (time_limit, secondary 9): the array exists and holds the starting basis.
- **Modifications without a re-solve:**
  - A bound change leaves the array untouched.
  - `add_constraint`, `add_variable` and `add_column` extend it: **new columns read 3 atLower even when free or (-inf,5]**, new rows read 1. Existing entries are preserved.
  - `Clp_status` drops to -1 (S/ClpModel.cpp:1299-1305), but the wrapper's `get_status()` still says optimal.
  - The next solve repairs the codes (new free column → 0, (-inf,5] → 2) in 0 iterations.
  - DOCUMENTED S/ClpModel.cpp:1308-1319: `maximumRows_` stays -1, so the reallocating branch always runs.
- **remove_variable + re-solve:** the removed column reads 5 isFixed ([0,0]).
- **Empty model (no column):** the wrapper skips Clp, so `statusExists` stays 0. If the model has columns but no matrix element (`emptyProblem` path), `statusExists` is 1 and the codes are set (rows basic). Secondary status 6.
- **No non-simplex algorithm:** the wrapper only ever calls `Clp_primal(model, 0)`, so barrier, crossover and presolve-only cases do not arise.
- **"Basis available" flag:** only `Clp_statusExists` or `Clp_statusArray() != NULL`. That flag means "an array exists", which is the basis the next solve starts from. It does not mean "optimal basis of the stored solution". There is no error code anywhere: the getters return int codes and the setters return void.

## E. Warm start (MEASURED; native knob `Clp_setMaximumIterations`)

- A cold start takes 7 iterations. Cold B with max iterations 0 gives Clp_status 3 (limit_reached).
- **B fresh + `Clp_copyinStatus(A's array & 7)` + max iterations 0 + `B.solve()` → Clp_status 0 optimal, 0 iterations, obj -15.** The same holds with A's raw bytes, and with max iterations 1000 (still 0 iterations).
- Per-entity setters also give 0 iterations, but only once `copyinStatus` has created the array.
- No setting is needed. The wrapper defaults already use the basis: `Clp_primal(…, 0)`, no presolve, no crash. `internalFactorize` puts the nonbasic values at their bounds from the codes (S/ClpSimplex.cpp:1620-1790: rows from 1639, columns from 1706), so no primal values need to be written.
- Writing a basis into a solved model takes effect: an all-slack array copied in gave 7 iterations again. Clp keeps no factorization: `primal()` defaults startFinishOptions to 0 (S/ClpSimplex.hpp:348), and `startup()` reuses a factorization only when bit 2 is set (S/ClpSimplex.cpp:9241-9243).
- **Bound change:** the array is kept. A re-solve takes 0 iterations (x1 ub 2 → 1.5, obj -14). Writing A's basis and then moving x3.lb to 3, which makes the basis primal infeasible, gives 1 iteration to the optimum (-13).
- **Invalid bases:** everything is accepted silently. There is no error and no crash, and the result is the right optimum in every case:
  - all 18 entries basic → 7 iterations (cold-like);
  - no basic entry → 7;
  - one basic short → 1;
  - singular basis (x6 and its row's logical both basic) → 2;
  - atUpper on an infinite ub, atLower on a free column, superBasic, isFixed on a non-fixed column, a `<=` row atLower (lo = -inf), a `>=` row atUpper (up = inf) → each repaired, 0 iterations;
  - a ranged row on the wrong finite side is a valid but non-optimal basis: with max iterations 0 it gives limit_reached with that basis's point; with 1000 it gives 1 iteration.
- **Codes outside 0..5:** codes 6 and 7 written by copyin survive the solve as 6/7 (the result is still optimal). Per-entity setter values 7 and -1 are ignored (DOCUMENTED S/Clp_C_Interface.cpp:1181, 1193).
- **W series** (one code changed, max iterations 0):
  - isFree or superBasic on x1 [0,2] → atUpper;
  - isFree or superBasic on x10 [0,inf) → atLower;
  - atLower or atUpper on a fixed column or an == row → isFixed;
  - isFree or superBasic on a `<=` row or a ranged row → atUpper.
  - Explanation (DOCUMENTED): `internalFactorize` turns isFree on a bounded entity into superBasic (S/ClpSimplex.cpp:1603-1606 rows, 1614-1617 columns). It then means to snap to the nearest bound, but compares `fabs(a - lower) < fabs(a - lower)`, which is always false (S/ClpSimplex.cpp:1688-1689 for rows, 1772-1773 for columns). So it always takes the finite upper bound, or the lower bound when the upper is infinite.
  - isFixed on lb < ub → atLower (DOCUMENTED S/ClpSimplex.cpp:1599-1602 rows, 1610-1613 columns). atLower or atUpper on lb == ub reads isFixed after the solve (MEASURED W5-W9). The code that does this was not located; INFERRED: inside primal.

## F. Partial or extended bases (MEASURED)

- A model extended after the get (a new column atLower, a new row basic), with the extended array copied in and max iterations 0 → optimal in 0 iterations. This holds both for the solved model and for a fresh extended model.
- A new **free** column given atLower is repaired to isFree, still 0 iterations.
- Subsets: a per-entity setter changes exactly one byte (F5), but it needs an existing array. `copyinStatus` takes only the full numberColumns+numberRows array. A shorter array is an out-of-bounds read (not run; DOCUMENTED S/ClpModel.cpp:3951-3952).
- Clp's own defaults for new entities are column 3 and row 1, from `createStatus` (S/ClpSimplex.cpp:7608-7634) and `resize` (S/ClpModel.cpp:1313-1314).

## G. Gotchas

1. No status array before the first real solve, so the per-entity C functions segfault. Check `Clp_statusArray() != NULL` (or `statusExists`) first. This also applies after `solve()` on a model with no variable, which never reaches Clp.
2. `Clp_setColumnStatus` / `Clp_setRowStatus` overwrite the primal solution. After `setColumnStatus(x10, atUpper)` with ub = inf, `get_solution()[x10]` is 1.797e308, and x1 drops from 2 to 0, while `get_status()` still says optimal (MEASURED G1). `Clp_copyinStatus` leaves `get_solution()` unchanged (MEASURED X1). Write with `copyinStatus`.
3. The raw bytes carry flag bits. None were seen after `primal` (MEASURED on random LPs), but mask `& 7`.
4. Columns first, rows after. The `statusArray` pointer is invalidated by `copyinStatus` and by any added row or column (the array is reallocated).
5. The array covers removed (recycled) columns: size it by `getNumCols`.
6. Every re-solve already warm-starts from the array Clp keeps. `set_basis` only matters for a fresh model or to override, and takes effect at the next `solve()`. `copyinStatus(nullptr)` clears the basis: the next solve is cold (7 iterations, MEASURED X2).
7. Reading right after a modification returns Clp's "next start" array, invalid entries included (atLower on a free new column). Clp repairs them only at solve.
8. A basis read after an iteration or time limit is a real basis (#basic = #rows) of an infeasible point.
9. Fixed entities never report a side.
10. Written isFree or superBasic codes snap to the upper bound (Clp's self-comparison bug), not to the nearest bound.
11. clp_lp has no iteration limit (`has_iteration_limit` is not satisfied). Its status variant has no `status::iteration_limit`: Clp code 3 maps to limit_reached. A solver-agnostic test that checks "0 iterations after set_basis" cannot run on Clp unless clp_lp gains `set_iteration_limit`. `Clp_setMaximumIterations` exists everywhere, but the getter has a different symbol in 1.17.x and devel, so the wrapper would have to keep the value itself, as it does for `_time_limit`.
12. There are no threads or callbacks in clp_lp. There is no lazy update: changes write Clp's arrays directly.

## H. Recommendation

- **Variants.**
  - Columns: `std::variant<nonbasic_at_lower_bound, basic, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`. Alternative 0 is at-lower, which is Clp's default for a new column.
  - Rows: `std::variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`. Alternative 0 is basic, Clp's default for a new row.
  - Both are read from activity and value sides with the same code table.
  - Code 4 (superBasic, nonbasic strictly between bounds) and codes 6/7 have no drafted tag. They are unreachable through the wrapper (ifValuesPass = 0; never observed in any measured flow) and only appear after a raw native write. Options, for the maintainer:
    - (a) map 4/6/7 to the bare `basis_status::nonbasic` and list it as a sixth alternative. This is honest and legal under `lp_basis_status`. Recommended.
    - (b) leave them out and document them as UB (they appear only after native writes).
    - (c) add a `superbasic : nonbasic` tag.
    - (d) read them as nonbasic_free. This mirrors only the write side: Clp turns isFree on a bounded entity into superBasic. Reading it back mislabels a value that sits between finite bounds, so it is not recommended.
- **get_basis.**
  1. Read `Clp_statusArray` once.
  2. If it is NULL, the model was never solved: throw (or document as UB).
  3. Otherwise build `handle_status_table`s sized `getNumCols` / `getNumRows` from `byte & 7`, columns first.
  4. Return a by-value snapshot.
- **set_basis.**
  1. Build an `unsigned char[getNumCols + getNumRows]`, starting from the current array if one exists, otherwise from Clp's defaults (column 3, row 1). Overwrite only ids below `min(table.id_bound(), getNumCols)` for columns and `min(table.id_bound(), getNumRows)` for rows: a snapshot from a larger model must not index past the array.
  2. Overwrite the ids below each table's bound with the write mapping.
  3. Make one `Clp_copyinStatus` call.
  4. Add no flags, no presolve change, no factorization and no values pass. Never use the per-entity setters (they crash without an array and they clobber the solution).
  5. Leave the wrapper status alone, or reset it: a maintainer decision.
- **api additions.** Add `Clp_statusExists`, `Clp_statusArray` and `Clp_copyinStatus` as required symbols. No `feature_unavailable_error` gate is needed.
- **Deviations to rule on:**
  1. There is no superbasic tag, so a code needs a mapping choice.
  2. Fixed columns and == rows never carry a side.
  3. "Basis available" only means "an array exists": after a modification, an iteration limit or an infeasible or unbounded stop, the basis is not optimal. The status array is the next start, not a certificate.
  4. Writes are never rejected (no `solver_error` possible): an inconsistent basis is silently repaired at the next solve, and wrong basic counts degrade to a near-cold start.
  5. `set_nonbasic(v, s)` with `s` strictly between the bounds cannot be honoured (Clp snaps it to the upper bound), and it needs the entity's bounds at set time.
  6. A warm-start conformance test needs an iteration limit that clp_lp does not have.

## Probes
- probe_basis.cpp: single probe covering B-H. Forked children cover the crash-prone native calls (D0, E5l/m). Outputs are in out_*.txt.
- Label caveat in out_*.txt: in D7a/D7b the column printed as `x[0,3]` is really (-inf,3]. `{.upper_bound = 3.0}` leaves the lower bound at -inf, as the printed lb shows.
