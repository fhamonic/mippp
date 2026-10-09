# SoPlex (soplex_lp) LP basis survey

Tags: MEASURED = a probe I ran; DOCUMENTED = vendor source/header (path:line) or URL; INFERRED = reasoning.
Releases: 7.1.3 shared (MIPPP_SOPLEX_LIBRARY, scipoptsuite-9.2.1), 6.0.4 shared (/usr/local/lib),
8.0.2 static `.a` + headers (conan cache, not loadable), master headers downloaded from GitHub on 2026-10-09.
All runs: `MIPPP_NO_VERSION_WARNING=1 timeout -k 5 60 ./probe...`, compiled one at a time through `flock .../lock1`.

## Headline (decides the design)

* **Reading a basis: fully possible from SoPlex 7.0 on** through the per-entity C getters
  `SoPlex_basisColStatus` / `SoPlex_basisRowStatus`. Codes name the ACTIVITY side for rows (no flip).
* **Writing a basis: there is no programmatic setter in the SoPlex C API, in any release up to master**
  (DOCUMENTED: soplex_interface.h has only `SoPlex_readBasisFile`, `SoPlex_basisRowStatus`,
  `SoPlex_basisColStatus`; master header identical to 7.1.3 — MEASURED `diff`). The C++
  `SoPlexBase<double>::setBasis/getBasis/clearBasis/hasBasis` are not exported (MEASURED `nm -D`,
  7.1.3 .so and 8.0.2 .a). The only C route, `SoPlex_readBasisFile` (CPLEX .bas file), is broken for
  anything but tiny models:
  1. its reader names entity j with a stringstream it never resets: column j is `x0x1...xj`, row i is
     `C0C1...Ci` (DOCUMENTED spxbasis.hpp:453-482 in 6.0.4, 7.1.3, 8.0.2 and master);
  2. the MPS line buffer is 256 chars (DOCUMENTED mpsinput.h:65, mpsinput.cpp:83), so only columns up to
     index 86 can appear on a `UL` line (255 chars), rows up to index 81 next to column 0 (252 chars ok,
     258 fails) — lines of at most 255 chars parse, 258 fail (MEASURED probe_name_limit);
  3. building those names costs superquadratic time inside SoPlex on EVERY call, even for an empty
     file: 0.048 s (n=m=1000), 0.26 s (2000), 1.76 s (4000), 12.3 s (8000), against a 0.30 s cold
     solve at 8000 (MEASURED probe_scaling);
  4. a path that cannot be opened throws `strict_fstream::Exception` through the extern "C" function:
     std::terminate, the process aborts (MEASURED probe_missing_file, exit 134).
  Within those limits it works perfectly (warm start OPTIMAL in 0 iterations, MEASURED E1).
* A C++-ABI route (exported `SoPlexBase<double>::readBasisFile(const char*, const NameSet*, const NameSet*)`
  with caller-built `soplex::NameSet`s) works at n=m=8000 in 7 ms, codes identical, OPTIMAL in 0
  iterations (MEASURED probe_cxx_abi) — but it is Itanium-ABI-only, the NameSet ctor mangling changes
  between 7.x (`_ZN6soplex7NameSetC1Eiidd`) and 8.x (`...C1Eimdd`) (MEASURED nm), sizeof(NameSet) is
  unknown without the vendor header, and it still needs a temp file.

## A. Wrapper facts (main @ 04431ca)

* Class: `soplex_lp : protected model_base<int, double>` (include/mippp/solvers/soplex/impl/v1/soplex_lp.hpp:28).
  The only SoPlex class; it is an LP class, so basis support is allowed there (no *_milp class exists).
* No remapping: not a remapping_model_base; variable id == native column index, constraint id == native
  row index (`native_id(v)`/`native_id(c)` return `v.id()`/`c.id()`, soplex_lp.hpp:114-115). No
  remove_variable / remove_constraint in the wrapper (the C API has SoPlex_removeColReal/RowReal since
  7.0, unused). So a basis table is sized `numCols()` / `numRows()` and indexed by native index.
* Constraint handle -> native row: `constraint(num_constraints())` at creation (soplex_lp.hpp:281-293);
  each row is `lhs <= a.x <= rhs`; the absent side is written as IEEE +-infinity (soplex_lp.hpp:269-276),
  bounds later set through the wrapper are clamped to +-1e100 (soplex_lp.hpp:36-38, 358-365). A ranged
  row can be made with `set_constraint_lower_bound` on a `<=` row (no add_ranged_constraint).
* solve() (soplex_lp.hpp:501-549): empty model -> unknown; crossed sides (lb>ub on a column copy or a
  row) -> infeasible without solving (421-432); if any row reads `SOPLEX_BASIS_ZERO` (3) the LP is
  cleared and re-added WITHOUT a basis (`_has_nonbasic_free_row` 437-444, `_reload_without_basis`
  445-498) because SoPlex mis-solves from such a basis; then `SoPlex_optimize`. Algorithm: SoPlex
  defaults (dual simplex ALGORITHM=1, REPRESENTATION auto, SIMPLIFIER=3 internal presolve —
  MEASURED getIntParam on 7.1.3); simplex only, no barrier, no crossover concept.
  Status mapping notes: ABORT_ITER -> `failed`; ABORT_TIME -> `time_limit`; (aside, unrelated to basis:
  ERROR_/NO_RATIOTESTER/NO_PRICER/NO_SOLVER/NOT_INIT fall through to `time_limit`, 528-535).
* get_dual_solution (soplex_lp.hpp:560-566): ONE bulk C call (`SoPlex_getDualReal`) into a
  `make_unique_for_overwrite<double[]>` buffer returned as `constraint_mapping`. There is no
  get_reduced_costs on soplex_lp (SoPlex_getRedCostReal exists since 7.0, undeclared). The basis getter
  cannot follow the bulk pattern: there is no bulk basis call in the C API, it must loop per entity
  (n + m calls; 16000 calls take 0.1 ms, MEASURED probe_scaling).
* Versions: `validated_versions = {[7.1.1, 8.0.4)}` on main (soplex_api.hpp:142-143);
  `{[7.1.1, 8.1.1)}` on branch fix/review-rulings. No `supported_versions`, no `probe_symbols`; the C
  API reports no version so the version guard never fires (soplex_api.hpp:140-141).
* Optional symbols: `SOPLEX_OPTIONAL_FUNCTIONS` (soplex_api.hpp:109-121) built with `find_function`
  (nullptr when absent, 128-129); `basisRowStatus` already declared there (26, 120). Callers check with
  `_optional(fn, name)` throwing `solver_error("X not available.")` (soplex_lp.hpp:40-45) or test for
  nullptr directly (244, 424, 438). `feature_unavailable_error` does NOT exist on main; it exists on
  fix/review-rulings (utility/solver_exceptions.hpp:20 there).

## B. Native API

| purpose | symbol / signature | in api header? | 6.0.4 | 7.0-7.1.x | 8.0.x | master |
|---|---|---|---|---|---|---|
| read row status | `int SoPlex_basisRowStatus(void*, int rowidx)` (soplex_interface.h:172) | yes, optional (soplex_api.hpp:26,120) | absent (MEASURED nm) | present (MEASURED 7.1.3) | present (MEASURED nm 8.0.2 .a) | present (DOCUMENTED) |
| read col status | `int SoPlex_basisColStatus(void*, int colidx)` (soplex_interface.h:182) | NO, must be added (optional) | absent | present | present | present |
| write (file) | `int SoPlex_readBasisFile(void*, const char* filename)` (soplex_interface.h:16) | NO | absent | present | present | present |
| iterations (tests/debug) | `int SoPlex_getNumIterations(void*)` (h:91) | NO | absent | present | present | present |
| solver status | `int SoPlex_getStatus(void*)` (h:85) | NO | absent | present | present | present |
| C setter / bulk getter / clear / hasBasis | none | — | — | — | — | — |

7.0.0 CHANGELOG lists SoPlex_readBasisFile among the C functions added (DOCUMENTED
https://raw.githubusercontent.com/scipopt/soplex/master/CHANGELOG line 233); the status getters are not
listed there but are absent in 6.0.4 and present in 7.1.3 (INFERRED: added in 7.0.0, matching the wrapper
comment soplex_api.hpp:109-110). All validated releases have all three.

Codes (DOCUMENTED soplex_interface.h:165-182, spxsolver.h:195-203 `VarStatus`): 0 ON_UPPER, 1 ON_LOWER,
2 FIXED, 3 ZERO (free, nonbasic at 0), 4 BASIC, 5 UNDEFINED. Row meaning (DOCUMENTED spxbasis.h:127-163):
row P_ON_UPPER means `a.x = u_r`, P_ON_LOWER `a.x = l_r`, P_FIXED `l_r = a.x = u_r`, P_FREE free row at 0.
Code 5 is unreachable through the getters: `basisStatusToVarStatus` maps every dual status including
D_UNDEFINED to BASIC (DOCUMENTED spxsolver.hpp:1792-1828); without a basis the getters fabricate a slack
basis (soplex.hpp:4360-4416).

## C. Row side convention — MEASURED: ACTIVITY side

LP (probe_basis.cpp `build_main`): y1..y4 in [0,10], xup in [0,4], xfix in [2,2], xlow in [1,5],
xfree free and in no row; r_le: y1+y2<=8, r_ge: y1-y2>=2, r_eq: y3+xfix==5, r_rng: 1<=y4+xup<=7,
r_slk: y1+y2+y3+y4+xlow<=100; min -y1-2y2+y3-y4-3xup+2xlow. Unique optimum y=(5,3,3,3), xup=4, xlow=1,
obj -21, nondegenerate duals (-1.5, 0.5, 1, -1, 0).

| entity | code | activity/value |
|---|---|---|
| y1..y4 | 4 BASIC | strictly inside |
| xup | 0 ON_UPPER | at upper 4 |
| xfix | 2 FIXED | 2 |
| xlow | 1 ON_LOWER | at lower 1 |
| xfree (free, empty col) | 3 ZERO | 0 |
| r_le (<=, tight) | 0 ON_UPPER | a.x = 8 = rhs |
| r_ge (>=, tight) | 1 ON_LOWER | a.x = 2 = lhs |
| r_eq (==) | 2 FIXED | 5 |
| r_rng [1,7] tight at 7 | 0 ON_UPPER | a.x = 7 = upper side |
| r_slk | 4 BASIC | 15 < 100 |

Identical codes for: maximize with negated objective (C2), SIMPLIFIER off (C3), REPRESENTATION forced
COLUMN or ROW x ALGORITHM PRIMAL or DUAL (C4, four runs), a tall LP with 20 rows / 8 cols (C5).
#basic == #rows in every run. Conclusion: a row's nonbasic code names the side of the activity; no flip
between <= and >= and no dependence on representation.

Read mapping (columns and rows alike): 4 -> basic; 1 -> nonbasic_at_lower_bound; 0 -> nonbasic_at_upper_bound;
2 -> nonbasic_fixed; 3 -> nonbasic_free; 5 -> unreachable (treat as solver_error or document as UB).
Note: before any solve a fixed column reads 1 (ON_LOWER), not 2 (fabricated slack basis by bounds,
MEASURED D1, DOCUMENTED soplex.hpp:4397-4404); a reader must accept ON_LOWER on lb==ub.

Write mapping (only via a .bas file, SPxBasisBase::readBasis, DOCUMENTED spxbasis.hpp:408-583):
* column basic -> `XU`/`XL <col> <row>` line pairing it with one nonbasic row (any pairing works);
* column nonbasic_at_upper_bound -> `UL <col>`; nonbasic_at_lower_bound -> `LL <col>` or nothing
  (default for a finite lower bound);
* column nonbasic_fixed -> nothing (default P_FIXED when lb==ub, spxbasis.hpp:500-501; `LL` on it also reads
  back FIXED, MEASURED E3g); cannot be written on lb != ub;
* column nonbasic_free -> nothing, only for a free column (default P_FREE); `UL`/`LL` on a free column are
  silently repaired to ZERO (MEASURED E3c/E3d); a bounded column cannot be written free (no superbasic);
* row basic -> nothing (default);
* row nonbasic_at_upper/lower/fixed -> named in an XU/XL line; SoPlex forces the side from the row type
  for `<=`, `>=`, `==` (XL on a <= row reads back ON_UPPER, XU on a >= row ON_LOWER — MEASURED E2e);
  only a ranged row honours the indicator (XU upper, XL lower — MEASURED E2d);
* row nonbasic_free -> CANNOT be written (no indicator) — and SoPlex mis-solves from such a basis anyway
  (MEASURED G1.0: SoPlex_optimize returns RUNNING, obj 2 instead of 31).

## D. Availability — MEASURED (7.1.3, probe_basis.cpp section D)

* Before any solve: no error; a fabricated slack basis: rows 4, columns 1 / 0 / 3 by bounds (fixed column
  reads 1). SoPlex_getStatus = 0 UNKNOWN. Indistinguishable from a real slack basis.
* After optimal: valid optimal basis, #basic == #rows (C1-C5).
* Infeasible LP: simplex's final basis (presolve did not catch it here), #basic == #rows, status 3 (D2a,
  D2b identical). Unbounded LP with presolve: presolve decides in 0 iterations and the getter returns the
  slack basis (`_hasBasis = false`, solvereal.hpp:113-120); with SIMPLIFIER off, the simplex's final basis
  (D3).
* Iteration limit (ITERLIMIT=3, IntParam 5): status -6 ABORT_ITER (wrapper: `failed`), current basis,
  #basic == #rows; a later unlimited solve continues from it (5 vs 8 iterations, D4a/D4d). Time limit 0 s:
  status -7 (wrapper `time_limit`), 0 iterations, a valid starting basis (all columns at upper, rows basic).
* After a modification without re-solve (D5): basis kept and adjusted; SoPlex_getStatus becomes 0 UNKNOWN.
  New row -> 4 BASIC; new column -> by bounds (ON_LOWER if lb finite, ON_UPPER if lb=-inf and ub finite,
  ZERO if free); bound change keeps codes; a nonbasic column whose active bound becomes infinite moves to
  the other side (ON_UPPER->ON_LOWER, ON_LOWER->ON_UPPER); a nonbasic row whose active side is freed
  becomes 3 ZERO (the case the wrapper reloads for).
* Non-simplex: SoPlex has none. A model solved entirely by presolve (D6, 0 iterations): basis available
  and valid, #basic == #rows.
* #basic == #rows after optimal: yes in all runs; SoPlex reports column-basis statuses in both
  representations (DOCUMENTED spxsolver.hpp:1792-1828).
* No "basis available" flag in the C API (SoPlexBase::hasBasis, soplex.hpp:4351, is not wrapped or exported).
  SoPlex_getStatus reports solver status only (UNKNOWN fresh/modified, NO_PROBLEM right after
  readBasisFile, last solve status otherwise).

## E. Warm start — MEASURED (7.1.3)

Setter used: SoPlex_readBasisFile with a file in SoPlex's own writer layout and the reader's cumulative
names; iteration knob: IntParam ITERLIMIT = 5 (soplex.h:1067) set to 0 through `api.setIntParam`.
* E0 control (fresh B, no basis, ITERLIMIT=0): ABORT_ITER (-6), 0 iterations, obj -47.
* E1 fresh B + A's basis: readBasisFile returns 1; codes read back before solving equal A's; ITERLIMIT=0
  solve -> OPTIMAL (1), 0 iterations, obj -21. Also OPTIMAL/0 iterations with SIMPLIFIER off (E1c), with
  primal simplex (E1d), and through wrapper solve() without limit (E1b: optimal, 0 iterations).
* No setting needed: SoPlex skips presolve and scaling whenever it holds a basis (DOCUMENTED
  solvereal.hpp:78-84); the wrapper defaults suffice. Side effect: a poor written basis disables presolve
  (empty file: 7 iterations vs 3 cold, E3j).
* With plain names `x1`/`C1` (what the C header suggests) the read fails at the 2nd entity: return 0,
  "Syntax error in line 3", basis cleared to the slack basis (E3i). A short line ` XU x0 C0` also fails
  (fixed-format heuristic patches the blank, E3h).
* Write into A after changing one bound (xup ub 4 -> 3.5, E2a): kept, OPTIMAL in 0 iterations at the new
  optimum -20. Bound change after the write (E2b, r_slk rhs 100 -> 200): kept, OPTIMAL 0 iterations;
  making the basis primal infeasible (E2c, y1 ub -> 4): 1 iteration vs 2 cold.
* Invalid bases (readBasisFile return / effect / next solve):
  - too many basic (column paired with an already nonbasic row, E3a): returns 1, SoPlex silently loads a
    different default basis (codes differ from the file), solve OPTIMAL in 4 iterations, correct;
  - too few basic (column paired twice, E3b): same silent replacement, OPTIMAL 4 iterations, correct;
  - UL on an infinite upper / LL on an infinite lower of a free column (E3c/E3d): returns 1, repaired to
    ZERO, OPTIMAL in 0 iterations;
  - singular basis (empty column basic, E3f): returns 1, codes as written, solve recovers, OPTIMAL in 4;
  - unknown name (E3e): returns 0, basis cleared, normal solve;
  - superbasic / nonbasic free row: not expressible in the format;
  - missing file: process abort (std::terminate on strict_fstream::Exception), probe_missing_file.
  No crash or wrong answer from a malformed but existing file; only the return value 0 signals a parse
  failure, and wrong counts are repaired silently (return 1).

## F. Partial / extended bases — MEASURED

The format is subset-based by construction: every unnamed column takes its bound default and every
unnamed row is basic. F1: model gained a column (lb 0, obj +1) and a row after the get; A's file loaded:
new column ON_LOWER, new row BASIC, OPTIMAL in 0 iterations. F2: omitting xup's UL line leaves xup at its
default ON_LOWER (read back), and the solve still reports OPTIMAL, obj -21, in 0 iterations (MEASURED;
the mechanism, a dual-simplex bound flip of the boxed column, is INFERRED — codes after that solve were not dumped). Brand-new
column -> nonbasic at lower (or its bound default), brand-new row -> basic. Only full-entity granularity:
a basic column must be written together with a nonbasic row partner.

## G. Gotchas

* The setter is a FILE: temp-file creation, filesystem I/O, and an abort if the file cannot be opened.
* Cumulative default names + 256-char lines: only columns <= 86 (UL line) and rows <= 81 (next to column 0)
  are writable, fewer when a basic column and its row partner share a line (MEASURED); superquadratic
  internal cost per call (12.3 s at n=m=8000 even for an empty file).
* readBasisFile first calls clearBasis() (soplex.hpp:9825-9853): a failed read leaves NO basis (slack).
* SoPlex does not keep a ZERO row safely: freeing a nonbasic row's active side yields code 3, and a
  re-solve from that basis returns RUNNING/wrong optimum (G1.0). The wrapper's reload (G1.2, 31 correct)
  drops every basis, including one the user just set (INFERRED from soplex_lp.hpp:511). An empty .bas
  file also resets it correctly (G1.1: 31) but costs the quadratic name building.
* get_basis right after set_basis returns what SoPlex kept (after its silent repairs), not the input.
* Fabricated slack basis before any solve and after presolve-decided infeasibility/unboundedness:
  get_basis never fails, it may describe a basis no simplex reached.
* A written basis disables presolve for the next solve (solvereal.hpp:78-84).
* No lazy updates, no callbacks, no threads in the C API; modifications apply immediately.
* 6.0.4 (outside the validated range) has none of the basis functions (MEASURED out_604.txt): the getter
  must be gated (nullptr check -> feature_unavailable_error on fix/review-rulings, solver_error on main).

## H. Recommendation

* Report for columns: `std::variant<basis_status::basic, basis_status::nonbasic_at_lower_bound,
  basis_status::nonbasic_at_upper_bound, basis_status::nonbasic_fixed, basis_status::nonbasic_free>`;
  for rows the same five (nonbasic_free on a row only appears in a stale basis after a modification).
  Read mapping: 4 basic, 1 lower, 0 upper, 2 fixed, 3 free; activity side for rows.
* get_basis: loop `SoPlex_basisColStatus` / `SoPlex_basisRowStatus` over `numCols()` / `numRows()` into a
  handle_status_table (no remapping); add `SoPlex_basisColStatus` to SOPLEX_OPTIONAL_FUNCTIONS; throw
  feature_unavailable_error (or solver_error on main) when either pointer is null.
* set_basis / has_lp_basis_warm_start: recommend NOT providing it on soplex_lp (deviation: SoPlex reads
  but cannot write a basis through its C API). Alternatives for the maintainer, in decreasing preference:
  (1) read-only basis on SoPlex; (2) ask upstream for `SoPlex_setBasis(void*, const int* rows, const int*
  cols)` / `SoPlex_getBasis` in soplex_interface.cpp (wraps the existing SoPlexBase<R>::setBasis,
  soplex.hpp:5563) and gate it per release; (3) C++ ABI route (works, 7 ms at 8000, but Itanium-only,
  release-specific mangling, unknown NameSet size, temp file); (4) SoPlex_readBasisFile with cumulative
  names (only entities with index <= ~81-86, superquadratic cost, abort on I/O failure) — not acceptable.
  If any writer is accepted: tags it cannot write = row nonbasic_free, column nonbasic_free on a bounded
  column, column nonbasic_fixed on lb != ub; the side of a one-sided or equality row is forced by its
  type; wrong counts are silently repaired; no flags are needed around the call (presolve is skipped
  automatically while a basis is held).

## Probe sources and outputs

* probe_basis.cpp -> out_713.txt (7.1.3), out_604.txt (6.0.4), err_713.txt (SoPlex stderr: syntax errors)
* probe_missing_file.cpp -> out_missing_file.txt
* probe_name_limit.cpp -> out_name_limit.txt
* probe_scaling.cpp -> out_scaling.txt
* probe_cxx_abi.cpp -> out_cxx_abi.txt
* vendor_dl/: CHANGELOG, soplex_interface_master.h, spxbasis_master.hpp, mpsinput_master.{h,cpp}
  (downloaded from raw.githubusercontent.com/scipopt/soplex/master on 2026-10-09)

## Open questions

* 7.0.x: presence of SoPlex_basisRowStatus/ColStatus is INFERRED (absent 6.0.4, present 7.1.3, not in the
  7.0.0 CHANGELOG list); moot for the validated range, which starts at 7.1.1.
* What basis loadDesc substitutes on a wrong basic count (E3a/E3b): return 1, codes differ from the file,
  looks like a crash/dual-feasible start rather than the slack basis; only characterized as "silent repair".
* handle_status_table reads ids past its bound as alternative 0, but SoPlex's own default for a column added
  after the snapshot is nonbasic at lower (by bounds) and for a row is basic (F1, D5a/D5e): should the column
  and row variants have different alternative orders (column alt 0 = nonbasic_at_lower_bound, row alt 0 =
  basic)?
* Would upstream accept SoPlex_setBasis / SoPlex_getBasis in soplex_interface.cpp (thin wrappers over
  SoPlexBase<R>::setBasis/getBasis, soplex.hpp:4423, 5563)? That is the only clean writer.
* Does a Windows SoPlex DLL export the C++ symbols the ABI route needs? Unmeasured; expected not.
* The nullptr gate throws solver_error on main and feature_unavailable_error on fix/review-rulings; 6.0.4 is
  outside both the validated list and any supported list, so it is not strictly "supported but not validated".
