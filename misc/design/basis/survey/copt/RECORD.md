# COPT LP basis survey (copt_lp)

Tags: MEASURED = a probe in this directory, DOCUMENTED = vendor header/manual or repo source (path:line), INFERRED = reasoning.

Environment: COPT 8.0.5 (`/home/plaiseek/Softwares/solvers/copt80/lib/libcopt.so`, pinned with `MIPPP_COPT_LIBRARY`). COPT 7.2.5 was not run (licence refused); its symbols were checked with `nm -D`.

Probe sources (all built with MIP++ `copt_lp`, native calls through `native_api()` / `native_model()`, dlsym for undeclared symbols):
- `probe_basis.cpp` -> `out_80.txt`: sections C (row convention under 11 algorithm configs), D (availability), E (warm start, settings, invalid bases), F (extended / partial).
- `probe_basis2.cpp` -> `out2_80.txt`: stored status after bound edits, compute_iis / Reset, SetBasis on a solved model, TimeLimit=0 on a warm basis, BarIterLimit stop, code histograms after crossover, free row, rows-free model.
- `probe_basis3.cpp` -> `out3_80.txt`: when a basis exists before any solve, an infeasible LP that simplex has to detect, solution reads after SetBasis.
- `doc_*.txt` / `doc72_capiref.txt`: text dumps of the vendor HTML manuals, used only for grepping.

## A. Wrapper facts

- **Classes.** Basis support would go on `copt_lp` only.
  - `copt_lp : public copt_base` (copt_lp.hpp:14).
  - `copt_base : protected model_base<int, double>` (copt_base.hpp:42).
  - `copt_milp : public copt_base` (copt_milp.hpp:21) is excluded by the *_milp ruling.
  - COPT has no *_qp class.
- **Handles are not remapped.**
  - `native_id(variable v)` returns `v.id()` and `native_id(constraint c)` returns `c.id()` (copt_base.hpp:127-128).
  - Handles are positional:
    - a variable's id is `num_variables()` before the add (copt_base.hpp:246, 258);
    - a constraint's id is `num_constraints()` before the add (copt_base.hpp:329, 338, 390, 402-404).
  - Nothing can be removed: the wrapper has no remove_variable, remove_constraint or remapping_model_base (grep of include/mippp/solvers/copt).
  - So handle uid == native column or row index, and `_handle_id_bound` is just Cols/Rows.
- **Rows.**
  - A row is stored natively with two sides LB and UB (`COPT_INFINITY = 1e30`):
    - `<=` gives [-inf, b];
    - `>=` gives [b, inf];
    - `==` gives [b, b] (copt_base.hpp:334-337 passes sense + bound to AddRow).
  - `set_constraint_lower_bound` / `set_constraint_upper_bound` (copt_base.hpp:431-441) turn any row into a ranged row in place.
  - There is no `add_ranged_constraint`. The probe makes ranged rows with these setters.
- **solve().**
  - It is `check(COPT->SolveLp(prob)); _status = _get_status();` (copt_lp.hpp:62-65).
  - It sets no algorithm, presolve or crossover parameter and does no reload. The constructor sets only `Logging=0` (copt_base.hpp:84).
  - Defaults MEASURED: Presolve=-1, LpMethod=-1, Crossover=1, Dualize=-1, DualPerturb=-1, Threads=-1.
  - LpMethod=-1 means "For Linear Programming, choose dual simplex method" (DOCUMENTED copt80/docs/copt-userguide_en/parameter.html:636).
  - `_get_status()` (copt_lp.hpp:35-54) has no case for LpStatus 11 (COPT_LPSTATUS_ITERLIMIT, copt80 copt.h:120), so that status maps to `unknown`.
- **Pattern for the getter.**
  - `get_dual_solution()` and `get_reduced_costs()` (copt_lp.hpp:86-99) do the same three steps:
    1. `make_unique_for_overwrite<double[]>(num_constraints()/num_variables())`;
    2. one bulk `GetLpSolution` call, with nullptr for the unwanted arrays;
    3. `constraint_mapping/variable_mapping(std::move(buffer))`, an id-indexed view (model_base.hpp:33-39).
  - A basis getter would make one bulk `GetBasis` call into two int buffers and wrap them.
- **compute_iis.**
  - `_compute_iis` calls `COPT->Reset(prob, 0)` first (copt_base.hpp:539).
  - `copt_lp::compute_iis()` calls `reset_status()` (copt_lp.hpp:69-73).
- **Versions.**
  - `validated_versions = {solver_version_range{{7,2},{8,1}}}` (copt_api.hpp:331-332).
  - COPT declares no `supported_versions`, neither on main nor on fix/review-rulings.
  - Every symbol in `COPT_FUNCTIONS` is required: `lib.get_function` throws `symbol_not_found` (copt_api.hpp:319-320, dynamic_library.hpp:164-183).
  - There is no optional-symbol list and no nullptr check in the COPT wrapper, and no `probe_symbols`.
  - The optional-symbol pattern exists only for HiGHS, on branch fix/review-rulings:
    - `HIGHS_OPTIONAL_FUNCTIONS` is built with `lib.find_function`;
    - `_compute_iis` checks `Highs->getIis == nullptr` and the version, and throws `feature_unavailable_error` (fix/review-rulings highs_api.hpp:336-376, highs_base.hpp:748-765).
  - main has no `feature_unavailable_error` at all. It lives in fix/review-rulings utility/solver_exceptions.hpp:20.

## B. Native API

The basis codes (DOCUMENTED copt80 copt.h:87-92, same in copt72 copt.h:86-91; meanings in capiref.html:763-797):

| Code | Name | Meaning |
|---|---|---|
| 0 | COPT_BASIS_LOWER | non-basic at its lower bound |
| 1 | COPT_BASIS_BASIC | basic |
| 2 | COPT_BASIS_UPPER | non-basic at its upper bound |
| 3 | COPT_BASIS_SUPERBASIC | "non-basic but not any of its bounds" |
| 4 | COPT_BASIS_FIXED | non-basic and fixed at its bound |

capiref.html:766 says the constraints are treated as slack variables internally. These codes are not declared in copt_api.hpp: they must be added.

| Symbol (signature) | In copt_api.hpp | 7.2.5 | 8.0.5 |
|---|---|---|---|
| `int COPT_GetBasis(copt_prob*, int* colBasis, int* rowBasis)`: bulk, full arrays | yes, :187, F-list :300 | yes | yes |
| `int COPT_SetBasis(copt_prob*, const int* colBasis, const int* rowBasis)`: bulk, full arrays, both required | yes, :188-189, :301 | yes | yes |
| `int COPT_SetSlackBasis(copt_prob*)` | no | yes | yes |
| `int COPT_GetColBasis(copt_prob*, int num, const int* list, int* colBasis)`: subset read, list may be NULL | no | yes | yes |
| `int COPT_GetRowBasis(copt_prob*, int num, const int* list, int* rowBasis)` | no | yes | yes |
| `int COPT_GetBasisHead(copt_prob*, int* bHead)` | no | **no** | yes |
| `int COPT_Reset(copt_prob*, int iClearAll)` | yes, :190, :302 | yes | yes |
| `COPT_ReadBasis` / `COPT_WriteBasis` (file) | no | yes | yes |
| attribute `"HasBasis"` (COPT_INTATTR_HASBASIS), "Whether LP basis is available" | no | copt72 copt.h:247 | copt80 copt.h:297, capiref.html:1512 |
| attribute `"SimplexIter"` | no | yes | copt80 copt.h:288 |

- The 7.2.5 and 8.0.5 columns come from `nm -D` (MEASURED). The vendor headers agree: copt72 copt.h:847-849, 891-892; copt80 copt.h:960-962, 1004-1005, 1070.
- **No setter for a subset exists.** There is no COPT_SetColBasis or COPT_SetRowBasis in either header (DOCUMENTED).
- **No simplex iteration limit exists.**
  - The headers list only BarIterLimit, NLPIterLimit and MipNLPIterLimit (DOCUMENTED copt80 copt.h:204, 208, 229).
  - `SetIntParam` returns 3 for "IterLimit", "SimplexIterLimit" and "LpIterLimit" (MEASURED).
- **There is no warm-start or advanced-basis parameter** (DOCUMENTED: no such parameter in copt.h or parameter.html).
- The GetBasis, SetBasis, SetSlackBasis and Reset docs are the same in 7.2 and 8.0 (DOCUMENTED copt72 capiref.html:5780, 5828; copt80 capiref.html:6191, 6239).
- Every basis function needed exists across the validated range [7.2, 8.1). No feature_unavailable_error gate is needed. GetBasisHead is the only exception (8.0 only) and is not needed.

## C. Row side convention: ACTIVITY (MEASURED)

The LP (min), with x1..x4 >= 0:

- Columns:
  - y in [0,3] with cost -1, nonbasic at upper;
  - z fixed at 2;
  - f free, with no entries and cost 0;
  - l in [0,inf) with cost +1.
- Rows:
  - R1: x1+x2 <= 4, tight;
  - R2: x3+x4 >= 2, tight;
  - R3: x1-x3 == 1;
  - R4: ranged, either [1,3] tight at 3 or [3,7] tight at 3;
  - R5: sum <= 100, slack.
- The optimum is x = (2,2,1,1), unique.
- The duals are nondegenerate:
  - R1 -1, R2 +1, R3 0.5;
  - R4 -1 at the upper side, +1 at the lower side;
  - y reduced cost -1, l +1.

Codes read (identical under every configuration below):

| Entity | Code |
|---|---|
| x1..x4 | BASIC |
| y (at ub 3) | UPPER |
| z (lb == ub) | FIXED |
| f (free, value 0) | SUPERBASIC |
| l (at lb 0) | LOWER |
| R1 `<=` tight (activity 4 = UB) | **UPPER** |
| R2 `>=` tight (activity 2 = LB) | **LOWER** |
| R3 `==` | **FIXED** |
| R4 [1,3] at 3 | **UPPER** |
| R4 [3,7] at 3 | **LOWER** |
| R5 slack | BASIC |
| free row (both sides infinite) | BASIC |

- Configurations: wrapper defaults; Presolve=0; LpMethod=1 with Presolve=0; barrier plus crossover; concurrent (4); auto (5); PDLP plus crossover; the maximize(-obj) variant. #basic = 5 = rows every time.
- The codes name the side of the row activity a.x within [LB, UB]. Nothing flips between `<=` and `>=`, or between min and max.
- `COPT_GetLpSolution`'s "slack" output is the activity: 4, 2, 1, 3, 11 (MEASURED; DOCUMENTED capiref.html:6106 "also known as activities of constraints").

Read mapping (columns and rows alike):

| Native code | Tag |
|---|---|
| 1 | basic |
| 0 | nonbasic_at_lower_bound |
| 2 | nonbasic_at_upper_bound |
| 4 | nonbasic_fixed |
| 3 | nonbasic_free |

- SUPERBASIC was observed only on a free column at 0, including in code histograms after barrier+crossover and PDLP+crossover on a 200x200 LP.
- On a boxed entity it can only come back as an echo of a user write (INFERRED).

Write mapping:

| Tag | Native code |
|---|---|
| basic | 1 |
| nonbasic_at_lower_bound | 0 |
| nonbasic_at_upper_bound | 2 |
| nonbasic_fixed | 4 |
| nonbasic_free | 3 |

- The non-leaf tags `nonbasic` and `nonbasic_at_bound` have no code of their own. If they ever must be written, LOWER is safe on COPT, because COPT repairs sides (see E).
- In the slack basis that COPT itself builds, which SetSlackBasis returns and which exists before any solve, a fixed column gets FIXED, a free column SUPERBASIC, a column with only an upper bound UPPER, and rows BASIC.

## D. Availability (MEASURED unless noted)

| Situation | HasBasis | GetBasis ret | Content |
|---|---|---|---|
| model with >= 1 row, before any solve | 1 | 0 | COPT's slack basis (rows BASIC, columns LOWER/UPPER/FIXED/SUPERBASIC by bounds) |
| model with 0 rows (empty, or columns only), before solve | 0 | 3 (COPT_RETCODE_INVALID) | untouched |
| optimal (simplex, presolve on/off, barrier+crossover, concurrent, PDLP+crossover) | 1 | 0 | optimal basis, #basic == #rows every time (5/5, 200/200) |
| optimal after barrier or PDLP with Crossover=0 | 0 | 3 | none |
| infeasible, found by presolve or the pre-simplex check (2x2 LP, presolve -1 or 0) | 0 | 3 | none |
| infeasible, found by simplex (dense 200x200 + `sum >= 1e6`, 3 simplex iterations) | 1 | 0 | final simplex basis |
| unbounded (presolve -1 or 0) | 1 | 0 | a basis, #basic == rows |
| time limit (TimeLimit 0, 1e-4, 0.01 on 300x300) | 1 | 0 | a valid basis, #basic == rows (slack basis or the current simplex basis) |
| simplex iteration limit | n/a | n/a | the knob does not exist |
| BarIterLimit=2 stop (LpStatus 11, Crossover 0 or 1) | 0 | 3 | none; the wrapper maps the status to `unknown` |
| LpMethod=3 (crossover only) with no solution supplied | n/a | n/a | SolveLp returns 5, internal error, so solver_error |
| after add row / add column / add_column, no re-solve | 1 | 0 | the stored basis is kept and extended (new column LOWER, new row BASIC); LpStatus resets to 0 (UNSTARTED), HasLpSol 0 |
| after a column bound / objective / row bound edit, no re-solve | 1 | 0 | the stored basis is kept (see G.3 for re-derived column codes); **LpStatus stays 1 (OPTIMAL) while HasLpSol drops to 0** |
| after `COPT_Reset(prob, 0 or 1)` | **1** | **0** | **all zeros** (every column and row LOWER, 0 basic): not a basis |
| after `compute_iis()` on an infeasible LP | **1** | **0** | **all zeros**, same cause |
| after `compute_iis()` on a feasible LP | 1 | 0 | an optimal basis from the routine's internal SolveLp, which started cold (3 or 4 iterations, the cold count) |
| after SetBasis, no solve | 1 | 0 | verbatim echo of what was written, even invalid content |

Answers to the specific questions:

- **Is #basic == #rows guaranteed after an optimal solve?** It held in every measured optimal, unbounded and time-limit case (MEASURED). There is no documentation of a guarantee.
- **Is there a native availability flag?**
  - `HasBasis` exists, and `GetBasis` returns COPT_RETCODE_INVALID (3) when HasBasis = 0.
  - **HasBasis is not reliable after Reset**: it stays 1 while the arrays are zeroed.

## E. Warm start (MEASURED)

COPT has no simplex iteration limit, so `SimplexIter` (and BarrierIter / PDLPIter) was read instead. `TimeLimit=0` was also used as a "0 work" knob.

**A and B on a 200x200 dense LP:**

- **A**, cold with defaults: 149 simplex iterations, optimal, obj 321053.874. With Presolve=0: also 149.
- **B**: a fresh identical model, then `SetBasis(A's basis)`:
  - SetBasis returns 0;
  - before the solve, HasBasis = 1, LpStatus = 0, and GetBasis echoes the basis exactly;
  - the solve gives **optimal with SimplexIter = 0** under:
    - defaults;
    - Presolve=0;
    - LpMethod=1;
    - LpMethod=1 with Presolve=0.
  - With **TimeLimit = 0** plus that basis, B still reports **optimal** with 0 iterations, and the basis it reads back equals the one written.

**Settings that ignore the basis:**

| Setting | Result |
|---|---|
| LpMethod=2, barrier | 19 barrier iterations, then crossover |
| LpMethod=4, concurrent | 149 simplex iterations = cold |
| LpMethod=6, PDLP | 3840 PDLP + 19 simplex iterations |

- No extra flag or factorization call is needed. The wrapper's defaults use the basis.

**SetBasis, then a modification, then solve** (B, presolve -1 and 0):

| Modification | Simplex iterations | Verdict |
|---|---|---|
| upper bound on a nonbasic column inf -> 1e6 | 0 | kept |
| upper bound on a basic column inf -> 1e6 | 0 | kept |
| add a slack row | 0 | kept |
| add an empty column | 0 | kept |
| less attractive objective coefficient on a nonbasic column | 0 | kept |
| set_time_limit | 0 | kept |
| basic column's upper bound tightened to half its value | 7 (cold 143) | kept, then repaired |

**Modify then SetBasis on the solved A:**

| Run | Simplex iterations |
|---|---|
| A re-solved with its own retained basis | 7 |
| fresh model with the bound + SetBasis(A's basis) | 7 |
| fresh model with the bound, cold | 143 |

**SetBasis on a solved, unchanged model:**

- LpStatus stays 1 (OPTIMAL) but **HasLpSol drops to 0**.
- `GetLpSolution` then returns 3, so the wrapper's `get_solution()` throws "API call failed: invalid data".
- `get_status()` still says optimal.
- Writing the slack basis there makes the next solve cold (149 iterations).

**Invalid bases** (convention LP; presolve 0 and -1; SetBasis ret, then the echo, then the solve):

| Written | SetBasis | Echo | Solve |
|---|---|---|---|
| valid optimal | 0 | same | optimal, 0 iterations |
| m+1 basic (l BASIC) | 0 | same | optimal, iterations = cold (4 or 3): **silently discarded** |
| m-1 basic (x1 LOWER) | 0 | same | optimal, cold: discarded |
| every column and row BASIC | 0 | same | optimal, cold: discarded |
| UPPER on infinite ub (l) | 0 | same | optimal, 0 iterations: **silently repaired** |
| LOWER, UPPER or FIXED on the free column f | 0 | same | optimal, 0 iterations: repaired |
| SUPERBASIC on l (boxed below) | 0 | same | optimal, 0 iterations: repaired |
| FIXED on non-fixed l | 0 | same | optimal, 0 iterations: repaired |
| LOWER or UPPER on fixed z | 0 | same | optimal, 0 iterations |
| == row R3 as LOWER, UPPER or FIXED | 0 | same | optimal, 0 iterations |
| R2 (>=) UPPER; R1 (<=) LOWER; both flipped (the slack convention) | 0 | same | optimal, 0 iterations: repaired |
| y LOWER instead of UPPER (valid, suboptimal side) | 0 | same | optimal, 0 iterations (the bound flip is not counted) |
| R5 (basic row) SUPERBASIC, so m-1 basic | 0 | same | cold: discarded |
| code 7 or -1 | **3** (solver_error through check) | the previous stored basis is unchanged | normal cold solve |
| extended model, wrong count (new column BASIC, or new row UPPER) | 0 | n/a | cold (149): discarded |

- No case crashed or gave a wrong answer.
- Consequence: an iteration count cannot catch a wrong side mapping on COPT. Only the echo (get after set, before solve) can.

## F. Partial or extended bases (MEASURED)

- **Arrays must be full.**
  - SetBasis always takes arrays of the current Cols and Rows.
  - `SetBasis(cols, nullptr)` and `SetBasis(nullptr, rows)` both return 3.
  - `GetBasis(cols, nullptr)` and `GetBasis(nullptr, rows)` work (ret 0).
  - `COPT_GetColBasis` / `GetRowBasis(num, list)` read a subset (ret 0, values equal the bulk read).
  - No subset setter exists.
- **A model that gained a column and a row after the get:**

  | Written for the new entries | Result |
  |---|---|
  | new column LOWER, new row BASIC | **0 iterations**, optimal |
  | new column SUPERBASIC, new row BASIC | 0 iterations |
  | new column BASIC (m+1 basic) | cold: discarded |
  | new row UPPER (m-1 basic) | cold: discarded |

  This holds both on a fresh model B and on the solved A after the additions.
- **COPT extends its own stored basis on AddCol(s)/AddRow(s).**
  - A new column gets LOWER, or SUPERBASIC if it is free.
  - A new row gets BASIC.
  - So a re-solve after additions warm-starts by itself (0 iterations).
- **The status a brand-new column or row needs:**
  - a column: nonbasic at a bound, LOWER (SUPERBASIC is also accepted);
  - a row: basic.

## G. Gotchas

1. **Reset and compute_iis drop the basis.**
   - COPT_Reset drops the basis (DOCUMENTED capiref.html:6239 "Reset basis and LP/MIP solution ... forces next solve start from scratch"; MEASURED: the next solve is cold).
   - Afterwards GetBasis returns 0, HasBasis = 1, and **all-zero arrays**.
   - The IIS routine calls Reset (copt_base.hpp:539), so get_basis() after compute_iis() on an infeasible LP reads zeros. On a feasible LP it reads the basis of the routine's internal cold SolveLp.
2. **set_basis drops the solution but not LpStatus.**
   - HasLpSol drops to 0 while LpStatus stays OPTIMAL, so the wrapper's cached `_status` lies, and get_solution / get_dual_solution throw solver_error.
   - set_basis should call reset_status(), as compute_iis does.
3. **A column bound edit re-derives that column's status.**
   - Any SetColLower or SetColUpper on a nonbasic column resets its stored status from its bounds:
     - UPPER becomes LOWER, even when the value is unchanged (ub 3 -> 3);
     - FIXED becomes LOWER;
     - SUPERBASIC becomes LOWER when a lower bound appears;
     - LOWER becomes SUPERBASIC when the column becomes free.
   - Basic columns keep BASIC.
   - Row bound edits keep the code unless it becomes impossible:
     - FIXED becomes LOWER when the row becomes ranged;
     - UPPER becomes LOWER when ub -> inf;
     - a <= row read as LOWER becomes UPPER when the ub moves.
   - Consequence: set_basis, then a bound edit, then get_basis does not echo what was written. The warm start still works: 0 iterations, the bound flips are free.
4. **The written side is advisory.**
   - COPT repairs impossible sides and flips boxed nonbasic columns to the dual-feasible side without counting iterations.
   - A wrong count of basic entries is accepted (ret 0) and silently dropped at the solve, which then starts cold. The solve stays correct and only slower.
   - Only out-of-range codes are rejected (ret 3).
5. **The basis is kept across modifications.** COPT keeps and extends its basis, so re-solves warm-start without set_basis. An unchanged re-solve runs 0 iterations.
6. **Availability depends on how the solve ended.**
   - A 0-row model has no basis before the solve. A model with rows has a slack basis before any solve.
   - An infeasible verdict may or may not leave a basis, depending on whether simplex or presolve decided it.
   - Barrier or PDLP with Crossover=0, and a BarIterLimit stop, leave none (ret 3).
7. **No simplex iteration limit.**
   - LpStatus 11 (ITERLIMIT, 8.0 only; absent from the copt72 header) arises only from BarIterLimit.
   - The wrapper maps it to `unknown`, an existing gap.
8. **No lazy updates and no solution slots.** COPT applies edits immediately: Cols and Rows update at once.
9. **No flags needed.** No flag or parameter must be set before reading or writing a basis, and nothing must be factorized first.
10. **LpMethod 2/4/6 ignore the basis.** LpMethod 2, 4 or 6, set by the user through native_api(), ignore a written basis. The wrapper never sets LpMethod.
11. **No problem-type switch.** copt_lp never changes column types: SetColType is called only in copt_milp.hpp:47-57. So no LP to MIP switch can invalidate a copt_lp basis.
12. **No LP callbacks.** Callback contexts are MIP-only (copt_api.hpp:216-219), so copt_lp has no callback restriction (INFERRED).

## H. Recommendation

- **Status variants (index order chosen so that ids past the snapshot's bound read as COPT's own default for a new entity):**
  - Columns: `std::variant<nonbasic_at_lower_bound, basic, nonbasic_at_upper_bound, nonbasic_free, nonbasic_fixed>`.
    - The variant index equals the COPT code (0 LOWER, 1 BASIC, 2 UPPER, 3 SUPERBASIC, 4 FIXED), so the encode is `index()`. The decode from a runtime code still needs a 5-entry table or a switch.
    - A column added after get_basis reads as nonbasic_at_lower_bound, which is what COPT would give it.
  - Rows: `std::variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_free, nonbasic_fixed>`.
    - A row added after get_basis reads as basic, which keeps the basis valid.
    - Decode with a 5-entry table.
  - **This makes the column and row status types different.** handle_status_table's "alternative 0 past the bound" rule forces it if both new entries are to stay valid. Otherwise the snapshot needs a per-kind default.
  - nonbasic_free stays in the row variant only for a faithful echo of a user-written basis. COPT never reports it for a row after a solve, and a free row reads basic.
- **get_basis():**
  1. Read Cols and Rows.
  2. Make one `GetBasis` call into two int buffers.
  3. Throw solver_error through check() on ret 3 (no basis).
  4. Decode into the two tables, or keep the int buffers and decode lazily, as get_dual_solution does.
  - Handle uid == native index, with no remap.
  - **Decision for the maintainer:** after Reset or compute_iis(), GetBasis returns zeros with ret 0. Options:
    - (a) document get_basis after compute_iis without a solve as UB (the misuse ruling);
    - (b) add an O(n+m) check that count(BASIC) == Rows and throw solver_error otherwise.

    (b) also catches any other zeroed state for the price of one pass.
- **set_basis(b):**
  1. `reset_status()` first. This is required: the native call drops the solution while LpStatus stays optimal.
  2. Build full arrays of the current Cols and Rows. Entries past the snapshot's bound come from alternative 0: LOWER for columns, BASIC for rows. Alternatively, start from a GetBasis read of COPT's current stored basis, which gives COPT's bound-derived defaults (SUPERBASIC for a new free column). That costs one extra call and fails on a 0-row model.
  3. Encode tag -> code and make one `SetBasis` call through check(). An out-of-range code cannot arise from tags.
  - No parameter, presolve or factorization handling is needed: the defaults (LpMethod -1 = dual simplex, Presolve -1) use the basis (MEASURED, 0 iterations).
  - Do not force LpMethod.
- **set_nonbasic(v, s):** pick the code from s against the bounds (FIXED if lb == ub, LOWER or UPPER within tolerance, SUPERBASIC otherwise). COPT keeps no value: SetBasis takes codes only.
- **Deviations to rule on:**
  1. COPT never rejects a wrong side or a wrong basic count. It repairs or discards silently, so "a solver rejecting input is a solver_error" never fires except for out-of-range codes, which the tags cannot produce. An invalid user basis costs only speed.
  2. A basis is unavailable after barrier or PDLP without crossover, after a BarIterLimit stop and after some infeasible verdicts. That is solver_error from ret 3, and the wrapper's defaults never reach it except for infeasible.
  3. The column and row variants differ (see above).
  4. get_basis after compute_iis needs either a validity check or a UB note.
  5. Bound edits rewrite stored column statuses, so set, edit, get is not an echo.
  6. The API needs additions:
     - COPT_BASIS_* constants;
     - optionally COPT_INTATTR_HASBASIS, COPT_SetSlackBasis and COPT_GetColBasis/GetRowBasis.

     All exist in 7.2.5 and 8.0.5, so they can be required symbols and no supported_versions or feature_unavailable_error gate is needed.
  7. Test design: iteration counts cannot detect a wrong write mapping on COPT. A get-after-set echo can.
  8. TimeLimit=0 plus an optimal basis reports optimal on COPT. Other solvers may report time_limit, so a solver-agnostic "0 iterations" test needs a per-backend knob. COPT has no iteration limit.

## Open questions

- Does HasBasis or GetBasis behave the same on 7.2.5? Not runnable here (licence). The symbols and docs are identical.
- Can COPT report SUPERBASIC for a boxed column after its own solve (for example after a time-limit stop during crossover)? It was never observed.
- Is #basic == #rows a documented guarantee? It held in every measurement, but no document states it.
