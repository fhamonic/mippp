# GLPK LP basis survey (glpk_lp)

Evidence tags: **MEASURED** (probe run here, GLPK 5.0 = /lib/x86_64-linux-gnu/libglpk.so -> libglpk.so.40.3.1, the only GLPK release on the machine), **DOCUMENTED** (vendor header, manual or source; the source and manual are GLPK 4.65 at ~/Softwares/solvers/coinbrew/ThirdParty/Glpk/glpk, the header is /usr/include/glpk.h 5.0), **INFERRED**.

Repo state: main @ 04431ca, clean, and nothing in the repo was modified. Branch fix/review-rulings holds the two-tier version work, and GLPK declares no supported_versions there either.

Probe sources (all in this directory):
- `probe.cpp` -> `probe.out`: sections C, D1-D7, E1-E3, F, G
- `probe2.cpp` -> `probe2.out`: H1-H5. These cover glp_factorize as a validator, glp_warm_up side effects, term_out, crossed bounds and a time limit stop.
- `probe3.cpp` -> `probe3.out`: F2 and F3, a stale snapshot written after a later solve, and a bigger basis truncated into a smaller model.
- `probe4.cpp` -> `probe4.out`: I1 and I2, glp_factorize on m == 0 and on nnz == 0. The cases run in forked children.
- `probe5.cpp` -> `probe5.out`: I3, which singular bases abort GLPK's factorizer. These cases also run in forked children.

Compile: `flock .../survey/lock0 /opt/gcc-15/bin/g++ -std=c++26 -O0 -I/home/plaiseek/Projects/mippp/include probeN.cpp -o probeN -ldl`.
Run: `MIPPP_NO_VERSION_WARNING=1 timeout -k 5 60 ./probeN`.
Every wrapper solve uses set_time_limit(10 s), and every direct glp_simplex call uses tm_lim = 10000.
The functions the api lacks (glp_get_row_prim, glp_get_it_cnt, glp_warm_up, glp_bf_exists, glp_factorize, glp_std/adv/cpx_basis, glp_interior) are reached with dlsym on `api.library_path()`.

## A. Wrapper facts (DOCUMENTED, repo paths)

- **Classes.** Only `glpk_lp` gets basis support (include/mippp/solvers/glpk/impl/v1/glpk_lp.hpp:18, `class glpk_lp : public glpk_base`).
  - `glpk_milp` (glpk_milp.hpp:23) is excluded by the *_milp ruling.
  - GLPK has no QP class.
  - `glpk_base` (glpk_base.hpp:27) is `protected model_base<int, double>`, not remapping_model_base. Only cplex, gurobi and highs use remapping_model_base.
- **Ids.** There is no remove_variable or remove_constraint(s) anywhere in the GLPK wrapper, so ids are dense and appended.
  - Variables: `_new_variables` uses offset = num_variables() (glpk_base.hpp:259-268), and `_add_column` does the same (272-281).
  - Constraints: `add_constraint` uses id = num_constraints() (346-358), and `_add_constraints` does the same (386-402).
  - `native_id(v) = v.id()+1` and `native_id(c) = c.id()+1` (glpk_base.hpp:117-118). A constraint handle is therefore GLPK row `id()+1`.
  - A snapshot is sized by glp_get_num_cols and glp_get_num_rows and needs no `_var_handle` translation.
- **Ranged rows.** There is no `add_ranged_constraint`. A ranged row comes from set_constraint_lower/upper_bound (glpk_base.hpp:426-431), which goes through `_set_row_bnds` (197-201) and `_bounds_type` (176-183) and gives a GLP_DB row. A row with lb == ub becomes GLP_FX.
- **solve()** (glpk_lp.hpp:132-141) makes one `glp_simplex(model, &model_params)` call.
  - Parameters (glpk_lp.hpp:31-46): meth=GLP_PRIMAL, pricing=GLP_PT_STD, r_test=GLP_RT_STD, presolve=0, excl=0, shift=0, it_lim=INT_MAX, tm_lim=INT_MAX (set by set_time_limit, 60-63), msg_lev=OFF.
  - There is no interior point, no crossover, no reload and no basis reset. The basis lives in glp_prob and persists across solves.
  - On GLP_EBOUND with strictly crossed bounds, solve() reports infeasible (134-137).
  - Status mapping `_simplex_status` (92-123):
    - EBADB, EBOUND and EFAIL become `failed`.
    - ESING and ECOND become `numerical_failure`.
    - EITLIM, EOBJLL and EOBJUL become `limit_reached`.
    - ETMLIM becomes `time_limit`.
  - The GLPK wrapper has no iteration-limit setter (has_iteration_limit is implemented only by cplex, gurobi and highs).
- **Pattern to follow.** `get_dual_solution` (glpk_lp.hpp:153-161) loops `glp_get_row_dual(i)` for i=1..m into `unique_ptr<double[]>` and returns `constraint_mapping`. `get_reduced_costs` (162-170) loops `glp_get_col_dual(j)` and returns `variable_mapping`. These are per-entity native calls with no bulk routine, and a basis getter would loop glp_get_col_stat and glp_get_row_stat the same way.
- **Versions.** glpk_api.hpp:276-280 has key "GLPK", library_names {"glpk"} and `validated_versions = {[4.59, 5.1)}`.
  - There are no `supported_versions` and no `probe_symbols`.
  - Every symbol is required: `CONSTRUCT_GLPK_FUNCTIONS` uses `lib.get_function` (glpk_api.hpp:267-268). There is no optional-symbol mechanism and no nullptr check of api members. HiGHS has one: `HIGHS_OPTIONAL_FUNCTIONS` plus `lib.find_function`, highs_api.hpp:335-345.
  - `feature_unavailable_error` exists only on branch fix/review-rulings (utility/solver_exceptions.hpp), not on main.
- **Already declared.** GLP_BS..GLP_NS (glpk_api.hpp:84-88). `glp_set_row_stat`, `glp_get_row_stat`, `glp_set_col_stat` and `glp_get_col_stat` are declared at 89-92 and in the function table at 244-247 (members `set_row_stat`, `get_row_stat`, `set_col_stat`, `get_col_stat`). `glp_term_out` is declared at 18 and 215.
- **Not declared.** glp_factorize, glp_bf_exists, glp_warm_up, glp_std_basis, glp_adv_basis, glp_cpx_basis, glp_get_it_cnt and glp_get_row_prim.

## B. Native API

**Read and write routines.** These are per entity and 1-based, and GLPK has no bulk array routine (DOCUMENTED /usr/include/glpk.h):

| Signature | glpk.h | Declared in the api? | Since |
|---|---|---|---|
| `int glp_get_row_stat(glp_prob *P, int i)` | :490 | yes, glpk_api.hpp:90 and :245 | 4.16 |
| `int glp_get_col_stat(glp_prob *P, int j)` | :499 | yes, glpk_api.hpp:92 and :247 | 4.16 |
| `void glp_set_row_stat(glp_prob *P, int i, int stat)` | :454 | yes, glpk_api.hpp:89 and :244 | 4.16 |
| `void glp_set_col_stat(glp_prob *P, int j, int stat)` | :457 | yes, glpk_api.hpp:91 and :246 | 4.16 |

The routines became `glp_` in 4.16, after their `lpx_` names (NEWS "GLPK 4.16").

**Helpers.** None of these is declared in the api:

| Signature | glpk.h | Since |
|---|---|---|
| `int glp_factorize(glp_prob *P)` | :613 | 4.18 (NEWS:1261-1262) |
| `int glp_bf_exists(glp_prob *P)` | :610 | 4.18 |
| `int glp_warm_up(glp_prob *P)` | :640 | 4.39 (NEWS:664) |
| `void glp_std_basis(glp_prob *P)` | :460 | 4.31 (NEWS:921) |
| `void glp_adv_basis(glp_prob *P, int flags)` | :463 | 4.31 |
| `void glp_cpx_basis(glp_prob *P)` | :466 | 4.31 |
| `int glp_get_it_cnt(glp_prob *P)` (tests only) | :512 | 4.53 (NEWS:305) |
| `double glp_get_row_prim(glp_prob *P, int i)` | :493 | old |
| `int glp_get_mat_col(glp_prob *P, int j, int ind[], double val[])` (for the empty-column guard in H) | :421 | old |

`glp_get_mat_col` accepts NULL arrays and then only counts the column (4.65 src/api/prob2.c:477-488).

Every symbol is present across the whole validated range [4.59, 5.1). GLPK therefore needs no feature_unavailable_error (DOCUMENTED). Only 5.0 was measured.

**Codes** (DOCUMENTED glpk.h:56-60, manual doc/glpk02.tex:1364-1384 and 2174-2185):

| Code | Value | Meaning |
|---|---|---|
| GLP_BS | 1 | basic |
| GLP_NL | 2 | nonbasic on its lower bound |
| GLP_NU | 3 | nonbasic on its upper bound |
| GLP_NF | 4 | nonbasic free, value 0 |
| GLP_NS | 5 | nonbasic fixed |

**Row variable.** A row's variable is the "auxiliary variable" x_i = sum_j a_ij x_{m+j}, and its bounds are the row bounds (DOCUMENTED doc/glpk01.tex:20-45). So it is the activity itself, not a slack.

**Setter semantics** (DOCUMENTED 4.65 src/api/prob5.c:52-80 and 106-134, manual glpk02.tex:1403-1455; MEASURED E3):
- A nonbasic code is replaced by the code that the entity's type implies:
  - FR becomes NF.
  - LO becomes NL.
  - UP becomes NU.
  - DB keeps NU and turns anything else into NL.
  - FX becomes NS.
- A code outside 1..5, or an index out of range, calls `xerror`, which aborts the process (SIGABRT, MEASURED). Reads with an out-of-range index abort too.
- Flipping whether an entity is basic clears the factorization (`lp->valid = 0`).

## C. Row side convention: activity (MEASURED, probe.out "C/D2" and "C'")

The model C LP is a maximization with objective 25. The basis is unique except for the free column z, whose reduced cost is 0.

| entity | type | value / activity | native |
|---|---|---|---|
| a in [0,4] | DB | 4 | NU(3) |
| b, c(free), d, e, w | LO/FR | 6, -2, 5, 4, 3 | BS(1) |
| f = 2 | FX | 2 | NS(5) |
| z free | FR | 0 | NF(4) |
| g >= 0 | LO | 0 | NL(2) |
| h in [1,7] | DB | 1 | NL |
| k <= 3 | UP | 3 | NU |
| R1 `b+a <= 10` | UP | 10 | NU |
| R2 `c+d >= 3` | LO | 3 | NL |
| R3 `d-e == 1` | FX | 1 | NS |
| R4 ranged [2,6] | DB | 6 (upper) | NU |
| R5 `<= 100` | UP | 6 (slack) | BS |
| R6 ranged [2,9] | DB | 2 (lower) | NL |

Minimizing the negated objective gives identical codes, so the codes do not depend on the objective sense. #basic = 6 = #rows.

A `<=` row tight gives NU and a `>=` row tight gives NL. So the code names the side of the activity a.x, with no flip.

- **Read mapping**, the same for rows and columns:

  | Native | Tag |
  |---|---|
  | GLP_BS | basic |
  | GLP_NL | nonbasic_at_lower_bound |
  | GLP_NU | nonbasic_at_upper_bound |
  | GLP_NS | nonbasic_fixed |
  | GLP_NF | nonbasic_free |

  No other value is ever returned.
- **Write mapping:**

  | Tag | Native |
  |---|---|
  | basic | GLP_BS |
  | nonbasic_at_lower_bound | GLP_NL |
  | nonbasic_at_upper_bound | GLP_NU |
  | nonbasic_fixed | GLP_NS |
  | nonbasic_free | GLP_NF |
  | generic nonbasic / nonbasic_at_bound, if ever allowed | GLP_NL |

  GLPK accepts every tag but rewrites it to the entity's type. The rewrites below were measured (E3):
  - Columns:
    - NU on a LO column becomes NL.
    - NL, NU or NS on a FR column becomes NF.
    - NF or NS on a DB column becomes NL.
    - NL or NU on a FX column becomes NS.
    - NL on an UP column becomes NU.
  - Rows:
    - NF or NL on an UP row becomes NU.
    - NU on a LO row becomes NL.
    - NL on a FX row becomes NS.
    - NF on a DB row becomes NL.

  So every tag can be written, but some states cannot be expressed:
  - **Superbasic:** nonbasic_free on a bounded entity lands on a bound.
  - nonbasic_fixed on a non-fixed entity becomes "at lower".
  - at_upper without a finite upper bound becomes at_lower, or free.
- **Specific codes:**

  | Entity | Code |
  |---|---|
  | fixed column | NS |
  | == row | NS |
  | nonbasic free entity | NF, value 0 |
  | basic entity | BS |

## D. Availability

- **Before any solve** (MEASURED D1): the statuses are defined, not garbage and not an error. This is GLPK's construction basis:
  - All rows are BS (src/api/prob1.c:309).
  - Columns are nonbasic by type. FR gives NF, LO gives NL, UP gives NU and FX gives NS. DB gives NL, or NU when |lb| > |ub| (prob1.c:391 and 639-657).
  - #basic = #rows, bf_exists = 0 and glp_get_status = UNDEF.
- **After optimal** (D2): the final basis. #basic = #rows, bf_exists = 1 and the status is OPT.
- **After infeasible** (D3, wrapper, presolve off): the final phase-1 basis. #basic = #rows, bf_exists = 1 and the status is NOFEAS.
- **After unbounded** (D4): the final basis. #basic = #rows and the status is UNBND.
- **Time and iteration limits** (D5, H5):
  - `set_time_limit(0)` stops at the starting basis: 0 iterations, wrapper `time_limit`, GLPK status FEAS.
  - A 1 ms limit on a 300x400 LP stopped after 49 iterations with #basic = 300.
  - A direct it_lim=5 call returned GLP_EITLIM with #basic = #rows. Resuming took 44 more iterations, so 5 + 44 = 49, the same as from scratch: the stopped basis is reused.
- **GLP_EFAIL:** the basis is not stored back, so the statuses stay as they were before the call (DOCUMENTED src/simplex/spxprim.c:1775). Not measured.
- **Crossed bounds** (EBOUND path, H4): the wrapper reports infeasible and the basis is untouched (MEASURED).
- **EBADB or ESING** (E3): the invalid input basis stays (MEASURED).
- **Modification without re-solve** (D6, MEASURED):
  - **Adding a row** gives the new row BS, so #basic stays equal to #rows, and clears the factorization (bf_exists = 0).
  - **Adding columns** gives them nonbasic by type: DB[-2,5] NL, DB[-5,2] NU, FR NF, FX NS, LO NL, UP NU, and add_column DB[0,1] NL.
  - **Bound change that keeps the type:** the status is kept (a: NU stays NU).
  - **Bound change that changes the type** renormalizes a nonbasic status:
    - DB to LO: NU becomes NL.
    - LO to DB: NL stays NL.
    - UP to DB: NU stays NU.
    - Row UP to FX: NU becomes NS.
    - Row FX to UP: NS becomes NU.
  - **Basic entities stay basic** (b fixed while basic stays BS).
  - **Status after modifications:** glp_get_status stays a stale OPT, and the wrapper status stays optimal.
- **Algorithms other than plain simplex** (D7):
  - glpk_lp runs only glp_simplex with presolve off.
  - glp_interior, which the wrapper never calls, leaves the statuses untouched (MEASURED).
  - **Presolve on** (not reachable through glpk_lp, measured by a direct call):
    - An optimal solve recovers a valid basis of the original problem, here an alternative optimal one (z BS, R5 NU), with bf_exists = 0.
    - An infeasible solve returns GLP_ENOPFS, leaves the input statuses untouched and leaves the status UNDEF.
  - **Trivial LP:** when no constraint coefficient is nonzero (nnz == 0), glp_simplex runs trivial_lp. That routine overwrites the basis: rows BS, columns at the bound the objective favours. A written basis, even an invalid one, is ignored and the solve reports optimal (MEASURED G; DOCUMENTED glpapi06.c:475).
- **glp_factorize on degenerate shapes** (MEASURED probe4):
  - m == 0: returns 0 when no column is basic. When a column is basic it returns GLP_EBADB, yet `solve()` reports optimal anyway, because trivial_lp rewrites the basis.
  - nnz == 0 with rows: returns 0 for a valid basis (rows basic) and GLP_EBADB for a wrong count, while `solve()` reports optimal because the basis is ignored. A right count with a basic column ABORTS.
  - So a validating set_basis would throw, or must guard, on bases that `solve()` would merely ignore when nnz == 0.
- **#basic == #rows after an optimal solve:** always (MEASURED in every solve; DOCUMENTED glpk02.tex:1391-1394 "on normal exit the simplex solver remains the basis valid"). Only a user write can break the count.
- **Native "basis available" flag:** none, because a basis always exists.
  - `glp_bf_exists` reports the factorization only. It is 0 before a solve, after modifications and after a presolved solve.
  - glp_get_status is UNDEF only before the first solve and goes stale after modifications.
  - Validity is checked by `glp_factorize` or `glp_warm_up`, which return GLP_EBADB (wrong count), GLP_ESING or GLP_ECOND.

## E. Warm start (MEASURED E1, E2, E3, H1, H2)

**Transfer to a fresh model:**
- Model C: A takes 6 iterations from scratch. A fresh B, given A's statuses through glp_set_*_stat and solved with the wrapper's `solve()`, reports optimal with objective 25, 0 iterations (glp_get_it_cnt) and the same basis.
- Random 40x60 LP: A takes 49 iterations and B takes 0, with the same objective.

**Iteration limit:**
- The knob is `glp_smcp.it_lim = 0`, through a direct glp_simplex call with the wrapper's parameters mirrored.
- With the basis, the call returns GLP_EITLIM with glp_get_status OPT, the optimal objective and 0 iterations. GLP_DUAL gives the same.
- Without the basis, it returns GLP_EITLIM with INFEAS (model C), or FEAS with objective 0 (random LP).
- The limit check comes before pricing (spxprim.c:1273), so it_lim=0 never reports optimal. The wrapper would map it to limit_reached{true} (glpk_lp.hpp:103).

**Settings:** the wrapper's defaults are enough.
- Presolve must be off, which it is (glpk_lp.hpp:43).
- With presolve on, the basis is ignored. This is DOCUMENTED: glpapi06.c:329 runs `glp_adv_basis(lp, 0)` on the transformed LP, never on the stored basis, and manual glpk02.tex:1698-1700 says "on performing re-optimization the presolver should be disabled". The measurement corroborates it, without proving it alone: 2 vs 2 iterations on model C and 67 vs 67 on the random LP, with and without the written basis.
- GLPK has no crash or hot-start flag, and no factorize call is needed. glp_simplex factorizes when !glp_bf_exists (glpapi06.c:226), and both primal and dual use the given basis.

**In-place re-solve is already warm:** re-solving A takes 0 extra iterations. glp_get_it_cnt is cumulative per problem object.

**glp_warm_up vs glp_factorize:**
- glp_warm_up also starts at 0 iterations, but it overwrites the stored basic solution and status (H2). After std_basis + warm_up the wrapper still said optimal while `get_solution_value()` returned 2 instead of 25. Do not use it in set_basis.
- glp_factorize validates without touching the solution or the status (H2: OPT and 25 kept). The next glp_simplex reuses the factorization, because spxprob.c:409-412 takes P->bfd when it is valid, and needed 0 iterations (H1).

**Writing a basis into A after a bound change** (E2):
- Change that keeps the type (a ub 4 to 3): the basis is kept. 0 iterations, optimal, objective 21.
- Change of type (a ub to inf): NU is written back but reads NL. 1 iteration.
- A basic variable made infeasible (b ub 6 to 5): kept as the starting basis. 1 iteration, optimal, objective 24.

**Invalid bases** (E3, H1):

| Invalid input | glp_factorize / glp_warm_up | glp_simplex | Wrapper status |
|---|---|---|---|
| one basic too many | GLP_EBADB | GLP_EBADB | failed |
| one basic too few | GLP_EBADB | GLP_EBADB | failed |
| right count but singular (a column in no row made basic) | GLP_ESING | GLP_ESING | numerical_failure |

- The bad basis persists, and every later solve fails the same way. It was recovered only after glp_adv_basis.
- **Some singular bases abort the process** (MEASURED probe5 and probe4, GLPK 5.0). GLPK's factorizer stops on `Assertion failed: k1 < k2` (bflib/sgf.c:185, SIGABRT), in glp_factorize and in the wrapper's `solve()` alike. The same xassert is at line 185 of the 4.65 source, so the 4.x range is very likely affected too (INFERRED).
  - **Aborted:** a basis holding a structurally empty column, that is, a variable that appears in no row:
    - {z} on 1 row;
    - {z, R1-slack} on 2 rows;
    - {x basic, row nonbasic} on a model with nnz == 0.
  - **Returned GLP_ESING:** other singular bases:
    - {z, x}, where z is empty;
    - {x, w}, two identical columns;
    - E3's k-empty case on model C.
  - **Why** (INFERRED from sgf.c:85-185): `sgf_reduce_nuc` never queues a column that is empty from the start. The singleton phases can then leave a 1x1 zero nucleus, and `xassert(k1 < k2)` fires. A column whose count falls to 0 during elimination is caught (return 1 or 2, giving ESING). So, as far as this reading goes, the abort needs a basic column that has no nonzero from the start. Rejecting any basis that holds such a column is enough to avoid it.
- glp_adv_basis prints "Constructing initial basis..." whatever msg_lev says, unless `glp_term_out(GLP_OFF)` is called first (H3).
- NU on an infinite upper bound is repaired silently at write time (reads NL). NL on a free column becomes NF. A "superbasic" (NF on a DB column) becomes NL.
- An invalid code (0 or 6) aborts the process (SIGABRT).
- No wrong answer was observed.

## F. Partial and extended bases (MEASURED F, F2, F3)

- **Writes are per entity**, so any subset can be written. Validity (count and nonsingularity) is checked only at the next factorization.
  - A columns-only write over the default all-rows-basic basis gives #basic 11 > 6, so GLP_EBADB.
- **Writing right after additions** works without extra handling.
  - GLPK gives a new row BS (prob1.c:309) and a new column a nonbasic code by type (prob1.c:391, then the set_col_bnds normalization).
  - Writing the old snapshot over the old entries is then valid: 2 iterations instead of 7 from scratch (in place and on a fresh B).
- **A solve in between breaks it** (F2). The new column and the new row both became NU, so an old-entries-only write gives #basic 6 of 7 rows, GLP_EBADB, and the solve reports failed.
- **The explicit extension rule fixes it:** uncovered rows BS and uncovered columns GLP_NL (normalized by type). The basis is valid, and the solve takes 2 iterations to the optimum of 30.
- **Truncating** a bigger model's basis into a smaller model gives GLP_EBADB (F3).

## G. Gotchas

1. The basis is persistent state of the problem object and always exists. It is not a solution item, it has no slot and it needs no flag to read or write. A "never solved" check must use the wrapper's `_status`, which itself is not reset by modifications.
2. GLPK rewrites written codes to the entity's type, silently. A get after a set does not round-trip for tags that disagree with the bounds. Bound edits also rewrite nonbasic codes when the type changes.
3. An invalid code, or an index out of range (for example a foreign handle), aborts the process through xerror. The same is already true of every other GLPK call made with a foreign handle.
4. An invalid or singular basis is not rejected at write time. It surfaces at the next solve as GLP_EBADB, GLP_ESING or GLP_ECOND (wrapper `failed` or `numerical_failure`) and persists until a valid basis is written.
5. glp_warm_up has side effects: it overwrites the solution, primal and dual values and the status.
6. glp_adv_basis prints regardless of msg_lev.
7. Presolve ignores the basis. glpk_lp hard-codes it off; a future presolve option would have to be turned off whenever a basis is set.
8. An LP with no nonzeros (nnz == 0) ignores the basis.
9. glp_get_it_cnt is cumulative. glp_get_status goes stale after modifications.
10. GLP_EFAIL does not store the basis.
11. The wrapper has no iteration limit. A 0-iteration check is only possible natively, and it_lim reports EITLIM, never optimal.
12. GLPK has no callbacks in glpk_lp and nothing thread-specific beyond GLPK's usual one-problem-per-thread rule (INFERRED). glp_term_out is a per-environment setting (INFERRED).
13. GLPK can ABORT the process (sgf.c:185 assertion) on some structurally singular bases: those holding a basic column with no matrix entry. This happens both in glp_factorize and in glp_simplex, so writing such a basis natively, or through an unguarded set_basis, kills the process at the next solve.
14. Code citations are from the 4.65 source. Behaviour was measured on 5.0 only, and releases 4.59 to 4.64 were not available to run.

## H. Recommendation

**Status variants.**
- GLPK reports exactly five tags, for rows and columns alike: basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free. This satisfies lp_basis_status.
- Following the iis_snapshot pattern, alternative 0 is what ids past the table bound read as, meaning handles created after get_basis. Choose it to equal the extension rule:
  - **Rows:** `variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`. Alternative 0 is basic, which is honest.
  - **Columns:** alternative 0 should be nonbasic. There are two options:
    - (a) `nonbasic_at_lower_bound` first. It is written as GLP_NL and GLPK turns it into NF, NU or NS by type. It is correct when written, but on read it is wrong for a new free or upper-only column.
    - (b) Add the generic `basis_status::nonbasic` as alternative 0, used only for ids past the bound, so the read is honest. It is written as GLP_NL. This changes the drafted concept: `has_modifiable_lp_basis` (model_concepts.hpp:637-657) does not list `set_status(v, basis_status::nonbasic{})`, so option (b) needs the concept extended, not only the variant.
  - Either way, a full-table write then gives a valid extended basis without special-casing.

**get_basis().** Loop `glp_get_col_stat(j)` for j=1..n and `glp_get_row_stat(i)` for i=1..m into two handle_status_tables, sized num_variables and num_constraints, with id = native - 1. Map with the read table in C. No gating is needed.

**set_basis(b).** The steps, in order:
1. **Count check, no native call.** Over the current entities, count the snapshot's `basic` entries, with ids past the bound counted through alternative 0. If the count differs from num_constraints, throw `solver_error`. GLPK would answer GLP_EBADB here, and this catches the common mistake before any write.
2. **Empty-column guard.** For each column marked basic, call `glp_get_mat_col(P, j, NULL, NULL)`. If it returns 0, throw `solver_error`. This prevents the measured abort; it costs at most m calls, each O(column length).
3. **Write.** For every current column j and row i, call `glp_set_col_stat` or `glp_set_row_stat` with `code(b.get_status(handle))`. This is n + m calls.
4. **Factorize.** Call `glp_factorize`. If it returns GLP_ESING or GLP_ECOND, restore the previous basis and throw `solver_error`: the solver rejected the basis, and without a restore it poisons every later solve. There are two ways to restore:
   - (i) Keep a saved copy taken before step 3. This costs n + m more get calls on every set_basis.
   - (ii) Call `glp_std_basis` on rejection only. This adds no cost on success, but the user's previous basis is lost.

The factorization from step 4 is reused by the next glp_simplex (spxprob.c:409-412), so validating costs no extra LU. Do not call glp_warm_up, set no flag, and keep presolve off.

When nnz == 0, solve() ignores any basis. Steps 1 and 2 still reject an invalid one there, which is consistent: the basis is invalid even though GLPK would skip it.

New api symbols: `glp_factorize` and `glp_get_mat_col`, plus optionally `glp_std_basis`. All are required symbols, present since 4.31 at the latest, so no feature_unavailable_error is needed.

**Cost summary for the ruling:**
- get_basis: n + m native calls.
- set_basis on success: n + m writes, at most m column-length probes and one LU that the next solve would have done anyway. Option (i) adds n + m reads.
- set_basis on rejection: one extra glp_std_basis with option (ii).

**Deviations from the drafted concepts.**
1. Every tag can be written, but GLPK rewrites tags that disagree with the bounds. It has no superbasic state. set_nonbasic(v, s) with s strictly inside the bounds cannot be honoured; it snaps to a bound, or to NF (value 0) on a free column. So a set followed by a get is not exact.
2. A basis always exists, including before the first solve. There is no "unavailable" state to report.
3. Validating in set_basis needs new api symbols (glp_factorize, glp_get_mat_col), and the maintainer must choose how to restore on rejection. A guard against the factorizer abort is needed, unless "singular basis = documented UB" is accepted with a process abort as its consequence.
4. The row side is the activity side natively, so no flip is needed.
5. The extension default for columns is the open choice (a) or (b) above.

## Open questions

1. Column alternative 0: `nonbasic_at_lower_bound` (an inaccurate read for new free or upper-only columns), or generic `nonbasic` (an extra alternative that GLPK never reports natively)?
2. When glp_factorize rejects a basis in set_basis, should it restore the previous basis from a saved copy (n + m extra reads per call), write glp_std_basis (free on success, loses the old basis), or keep the bad basis? Then it throws solver_error. Should the count check and the empty-column abort guard be added, given that an empty basic column aborts the process inside GLPK (sgf.c:185)?
3. Should set_basis apply the extension rule (rows BS, columns NL) to entities newer than the snapshot? This is needed for validity after an intervening solve (F2).
4. A solver-agnostic "0-iteration warm start" test needs an iteration limit or an iteration count, and glpk_lp has neither. Should glpk_lp gain set_iteration_limit (it_lim)? That would require mapping GLP_EITLIM to status::iteration_limit instead of limit_reached. The alternative is a backend test using glp_get_it_cnt.
5. Should a set-then-get round trip be promised? GLPK cannot promise it for tags that disagree with the bounds.
6. Releases 4.59 to 4.64 were not measured, because only 5.0 is installed.
