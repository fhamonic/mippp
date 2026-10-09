# Gurobi LP basis survey (backend `gurobi`, class `gurobi_lp`)

Tags: **MEASURED** (a probe I ran), **DOCUMENTED** (vendor header / manual, cited), **INFERRED**.
Releases measured: 11.0.3, 12.0.1 (default) and 13.0.2, pinned with `MIPPP_GUROBI_LIBRARY`.
Gurobi 10 is inside `validated_versions` but is not installed here, so every 10.x claim is INFERRED.

Repo state: local `main` = 04431ca is 13 commits behind `origin/main` (= `fix/review-rulings`).
`feature_unavailable_error` and `supported_versions` exist only on `origin/main`.

## Probe files (all in this directory)

- `basis_probe.cpp`: one binary with sections P C D E F G H I J (`./basis_probe <section>` or `all`).
  It builds every model through MIP++ `gurobi_lp`, reaches the native calls through
  `native_api()` / `native_model()`, and dlsyms `GRBreset` from `api.library_path()`.
- Outputs: `out_all_1103.txt`, `out_all_1201.txt`, `out_all_1302.txt` (full runs),
  `out_J_{1103,1201,1302}.txt`, and the per-section 12.0.1 runs `out_{C,D,E,F,G,H,I}_1201.txt`.
- Compile: `flock .../survey/lock1 /opt/gcc-15/bin/g++ -std=c++26 -O0 -I/home/plaiseek/Projects/mippp/include basis_probe.cpp -o basis_probe -ldl`
- Run: `MIPPP_NO_VERSION_WARNING=1 [MIPPP_GUROBI_LIBRARY=...] timeout -k 5 60 ./basis_probe all`. A full run takes 0.3 s. No probe process survived (checked with ps).

Test models:
- **core LP** (9 columns, 5 rows): max 2x+3y+u-v+t, with
  - x,y in [0,10], w >= 0, u in [0,1], f in [3,3], v in [1,5], t in (-inf,3];
  - z1 free (appears in the slack row only), z2 free (empty column);
  - rows `le: x+y<=6`, `ge: x-y>=-2`, `eq: y-w=0`, `slack: x+y+u+f+v+t+z1<=100`, `geslack: x+y>=1`.

  The optimum is x=2, y=4, w=4, u=1, f=3, v=1, t=3, obj 19. Its duals are nonzero (2.5 and -0.5), so it is dual-nondegenerate except for the zero-cost free columns.
- **medium LP**: 60 columns in [0,10], 40 rows of mixed senses (every 5th row is `==`), seeded LCG data. The wrapper's default solve takes 39 simplex iterations; obj 629.581256.

---

## A. Wrapper facts

- **Classes.** Only `gurobi_lp` (gurobi_lp.hpp:17, `class gurobi_lp : public gurobi_base`) gets basis support. `gurobi_milp` (gurobi_milp.hpp:21) is excluded by the pure-MILP ruling. There is no Gurobi QP class: all.hpp aliases only gurobi_api / gurobi_lp / gurobi_milp.
- **Base.** `gurobi_base : protected remapping_model_base<int,double>` (gurobi_base.hpp:33).
- **Variable ids are remapped**, but lazily:
  - `remove_variable(s)` (gurobi_base.hpp:403-416) calls `_lazily_remove_variables()` at once (131-181). That function does GRBupdatemodel, then GRBdelvars, then GRBupdatemodel.
  - It switches `_remap_ids` on only when a non-tail column is deleted.
  - Translation goes through `_native_id(v)` / `_var_handle(j)` (remapping_model_base.hpp:40-47). Tables are sized with `_handle_id_bound(_num_var_native_ids)` (remapping_model_base.hpp:90-93).
  - Removed handle ids are recycled through `_free_var_handles` (remapping_model_base.hpp:56-71).
- **Constraints cannot be removed.** There is no `remove_constraint`, and no `add_ranged_constraint` either: `GRBaddrangeconstr` is declared at gurobi_api.hpp:48/246 but never used.
- **A constraint handle is the native row index.**
  - `constraint(_lazy_num_constraints++)` (gurobi_base.hpp:472; bulk version 532-547).
  - `native_id(constraint c)` returns `c.id()` (gurobi_base.hpp:220).
  - `native_id(variable v)` returns `_native_id(v)` (219).
- **Lazy updates.**
  - Adds and attribute writes are queued: `GRBaddvar`, `GRBaddconstr`, `GRBset*attr*`.
  - Getters that read the model call `update_gurobi_model()` (124) first. `num_variables()` (184-193), `num_constraints()` (194-204) and `get_*` bound/obj/name do so.
  - The solution getters do not update: `get_solution` and `get_reduced_costs` use `_num_var_native_ids`.
  - `variables()` and `constraints()` use the counters, so they need no update (63-70).
- **solve().**
  - On main it is `check(GRB->optimize(model)); _status = _get_status();` (gurobi_lp.hpp:93-96). origin/main adds `reset_status()` first.
  - No Method, Presolve, Crossover or LPWarmStart is ever set. The only parameter the wrapper touches is OutputFlag (gurobi_base.hpp:84), plus DualReductions temporarily in `refine_lp_status` (gurobi_lp.hpp:97-112).
  - So the solve runs Gurobi's defaults (MEASURED, section P): Method=-1, Presolve=-1, Crossover=-1, Threads=0, UpdateMode=1. LPWarmStart is 1 on 11.0.3 and 12.0.1, and -1 on 13.0.2.
  - Default Method for an LP is concurrent (DOCUMENTED: parameters page, Method: "Automatic (-1) setting will typically choose non-deterministic concurrent (Method=3) for an LP"). On every probe LP, ConcurrentWinMethod was 1, i.e. dual simplex won (MEASURED).
- **The pattern a basis getter will follow:**
  - `get_dual_solution` (gurobi_lp.hpp:136-143) sizes from `num_constraints()`, which updates. It reads `Pi` with `getdblattrarray` through `check()`, so 10005 becomes `solver_error`, and returns `constraint_mapping(unique_ptr<double[]>)`.
  - `get_reduced_costs` (144-154) and `get_solution` (125-135) size from `_num_var_native_ids` without an update, read `RC` / `X`, and return a `variable_mapping` lambda that indexes by `_native_id(v)`.
  - The IIS snapshot is the closer template, `_compute_iis` at gurobi_base.hpp:760-853. It runs `_lazily_remove_variables()` and `update_gurobi_model()`, sets `num_col=_num_var_native_ids` and `num_row=num_constraints()`, and builds `iis_answer(_handle_id_bound(num_col), num_row)`. Columns are keyed by `_var_handle(j).uid()` (836-838) and rows by `i`. It reads `Sense` with `getcharattrarray` to name a row's side (822-824, 839-845).
- **api.**
  - `library_names = {"gurobi130","gurobi120","gurobi110","gurobi100"}` (gurobi_api.hpp:312-313).
  - `validated_versions = {solver_version_range{{10},{14}}}` (315-316).
  - There is no `supported_versions`, on main or on origin/main. origin/main declares one only for HiGHS (highs_api.hpp:366).
  - Optional symbols use the `GRB_OPTIONAL_FUNCTIONS` X-macro (285-290). Its members are built with `lib.find_function` (297-298), are nullptr when absent, and are checked by hand (322-324: `if(!emptyenvinternal && !emptyenv) throw solver_error`).
  - Required symbols use `lib.get_function`, through `GRB_FUNCTIONS` (228-283).
  - `feature_unavailable_error : solver_error` exists only on origin/main (`git show origin/main:include/mippp/utility/solver_exceptions.hpp`, lines 17-24). Gurobi does not use it.
  - Errors: `_check` (342-346) maps 10009 to `license_error` and anything else to `solver_error(geterrormsg)`. `GRB_ERROR_DATA_NOT_AVAILABLE = 10005` is already declared (71).

## B. Native API

- **Bulk read/write** (all DOCUMENTED, gurobi_c.h 12.0.1:158/161; 11.0.3:157/160; 13.0.2:159/162):
  - `int GRBgetintattrarray(GRBmodel*, const char* attrname, int first, int len, int* values)`
  - `int GRBsetintattrarray(GRBmodel*, const char* attrname, int first, int len, int* newvalues)`
  - Attributes: `"VBasis"` (columns) and `"CBasis"` (rows). Header lines: 12.0.1 at 409/418, 11.0.3 at 401/410, 13.0.2 at 425/434.
  - **Already declared** in the api struct: gurobi_api.hpp:134-137, required in `GRB_FUNCTIONS` at 262-263. Nothing to add for the core read/write.
- **Per-entity:** `GRBget/setintattrelement` (gurobi_api.hpp:130-133, 260-261), already declared. `GRBget/setintattrlist` exist (12.0.1 header:164/167) but are **not** in the struct and are not needed.
- **Also needed** (already declared):
  - `GRBgetcharattrarray` with `"Sense"` (gurobi_api.hpp:179-180/274) to name the row side;
  - `GRBgetdblattrarray` with `"LB"` / `"UB"` (157-158/268; constants 140-141) to detect fixed columns.
- **To add to the struct: `int GRBreset(GRBmodel *model, int clearall)`.** It is in the 11.0.3:970, 12.0.1:1036 and 13.0.2:1109 headers. DOCUMENTED: docs.gurobi.com .../reference/c/solving.html says "Reset the model to an unsolved state, discarding any previously computed solution information ... Pass 0 to just discard the solution". Its presence in 10.x is INFERRED, not verified, because 10.x is not installed. Declare it in `GRB_OPTIONAL_FUNCTIONS` (find_function + nullptr) rather than `GRB_FUNCTIONS`, so that a 10.x library lacking it still loads. `set_basis` then throws when the pointer is null. Confirm on a 10.x library before merging. It is needed by set_basis (see E/G).
- **Constants to add** to the non-header branch of gurobi_api.hpp:
  - `GRB_INT_ATTR_VBASIS "VBasis"` and `GRB_INT_ATTR_CBASIS "CBasis"`;
  - the codes `GRB_BASIC 0`, `GRB_NONBASIC_LOWER -1`, `GRB_NONBASIC_UPPER -2`, `GRB_SUPERBASIC -3` (header 12.0.1:1102-1105, 11.0.3:1039-1042, 13.0.2:1177-1180).
  - Optionally, for docs and tests only: `"LPWarmStart"` (1152), `"Method"` (1142), `"Presolve"` (1289), `"Crossover"` (1162), `"IterCount"` (392).
- **Code meanings.**
  - VBasis (DOCUMENTED, attributes/variable.html): "0 (basic), -1 (non-basic at lower bound), -2 (non-basic at upper bound), and -3 (super-basic)".
  - CBasis (DOCUMENTED, attributes/constraintlinear.html): "0 (basic) or -1 (non-basic). A constraint is basic when its slack variable is in the simplex basis".
  - Both pages: "you must set basis status information for all constraints and variables in the model", "Only available for basic solutions", and "if you provide a valid starting extreme point ... VBasis, CBasis, then LP presolve will be disabled by default ... For presolve to be enabled, the parameter LPWarmStart must be set to 2".
- **Release availability.** VBasis, CBasis, the array accessors and GRBreset exist in 11.0.3, 12.0.1 and 13.0.2. MEASURED: they worked identically on all three. 10.x is INFERRED. No `feature_unavailable_error` gating is needed.

## C. Row side convention (MEASURED, out_all_*: section C, identical on 11/12/13)

Core LP, in six variants: default; primal and dual simplex with Presolve=0; dual with presolve; barrier+crossover with Presolve=0; and the negated-objective minimisation. Every variant reached obj 19 and gave the same column codes, except z1:

| column | bounds | value | VBasis |
|---|---|---|---|
| x | [0,10] | 2 | 0 |
| y | [0,10] | 4 | 0 |
| w | [0,inf) | 4 | 0 |
| u | [0,1] | 1 | -2 (at upper) |
| f | [3,3] | 3 | -1 (**fixed reads -1**) |
| v | [1,5] | 1 | -1 |
| t | (-inf,3] | 3 | -2 |
| z1 | free | 86 | 0 (basic, simplex runs) |
| z1 | free | 0 | **-3** (barrier+crossover run) |
| z2 | free, empty column | 0 | **-3** in every run |

| row | sense, rhs | Slack | CBasis |
|---|---|---|---|
| le | <, 6 | 0 | -1 |
| ge | >, -2 | 0 | -1 |
| eq | =, 0 | 0 | -1 |
| slack | <, 100 | 0 (simplex: z1 basic) / 86 (barrier) | -1 / 0 |
| geslack | >, 1 | **-5** | 0 |

- CBasis was only ever 0 or -1 in every solve, on all three releases. It **never names a side**.
- A row with CBasis -1 always had Slack == 0, for `<`, `>` and `=` alike.
- `Slack = RHS - a·x` for both senses: -5 = 1 - 6 on the loose `>=` row, and 86 = 100 - 14.

So Gurobi's logical is `a·x + s = rhs`, with s >= 0 on a `<` row and s <= 0 on a `>` row. -1 means "slack at 0", i.e. the activity sits at the rhs: its **upper** side on a `<` row and its **lower** side on a `>` row. The code names neither the activity's side nor the slack's own bound side, which flips with the sense. The side can only be **derived from Sense**. This is the same trick as `iis_row_status_by_sense` (iis_snapshot.hpp:93-103).

Ranged rows: the wrapper has none. Natively (G2, MEASURED), `GRBaddrangeconstr` turns the row into `=` and adds a range column `RgXXX` in [0, ub-lb]. The row's CBasis stays -1. The **range column carries the side**: -1 (at 0) means the activity is at the upper side; -2 means it is at the lower side. NumVars grows by one (3 instead of 2), which would desynchronise `_num_var_native_ids` if the wrapper ever used this call.

A fixed column read back as -2 once: on 11.0.3, after a warm lim0 stop from a written -1 (H1 dump). At every optimum it read -1. The read mapping therefore treats -1 and -2 alike when LB == UB.

**Read mapping (native to tag):**
- Columns:
  - 0 → `basic`
  - -1 → `nonbasic_at_lower_bound`, or `nonbasic_fixed` if LB == UB
  - -2 → `nonbasic_at_upper_bound`, or `nonbasic_fixed` if LB == UB
  - -3 → `nonbasic_free`. Seen only on free columns at value 0; Gurobi's meaning is "superbasic".
- Rows:
  - 0 → `basic`
  - -1 with Sense `<` → `nonbasic_at_upper_bound`
  - -1 with Sense `>` → `nonbasic_at_lower_bound`
  - -1 with Sense `=` → `nonbasic_fixed`

**Write mapping (tag to native):**
- Columns:
  - `basic` → 0
  - `nonbasic_at_lower_bound` → -1
  - `nonbasic_at_upper_bound` → -2
  - `nonbasic_fixed` → -1 (-2 is also accepted, MEASURED)
  - `nonbasic_free` → -3
- Rows:
  - `basic` → 0
  - **every nonbasic tag → -1.** The side named by `nonbasic_at_lower_bound`, `nonbasic_at_upper_bound`, `nonbasic_fixed` or `nonbasic_free` cannot be written. Gurobi always puts the activity at the rhs, so a contradictory side such as at_lower on a `<` row cannot be represented.
  - There are no free rows.

Codes reported:

| entity | code |
|---|---|
| fixed column | -1 (or -2) |
| `==` row | -1 |
| nonbasic free column | -3 |
| basic entity | 0 |

## D. Availability (MEASURED; same on 11/12/13 except the PDHG lines)

- **Before any solve:**
  - After an update, both getters return **10005** DATA_NOT_AVAILABLE.
  - Before the update, a length counting the queued entities gives **10006** INDEX_OUT_OF_RANGE, because NumVars and NumConstrs are still 0.
- **After an optimal solve:** available, with **#basic == #rows in every case**. This held for wrapper solve(), primal, dual, concurrent, barrier+crossover, and PDHG+crossover on 12/13.
- **Infeasible LP:**
  - When presolve proved it (Status 3 with Presolve on, trivial model): 10005.
  - When simplex proved it (Presolve=0, or the depot/store conflict even with defaults, 8 iterations): available, #basic == #rows.
- **Unbounded LP:**
  - Default settings give Status 4 INF_OR_UNBD from presolve, and the basis is 10005.
  - After `DualReductions=0`, the re-solve (what `refine_lp_status` does) gives Status 5 with a basis available.
  - With Presolve=0, Status 5 and a basis available.
- **Iteration limit:**
  - Simplex stop (IterationLimit 1 or 5): Status 7 with the current basis available, #basic == #rows.
  - Barrier with IterationLimit 3, which stops in crossover: Status 7, 10005.
  - Barrier with BarIterLimit=2: Status 2, because crossover finished the job, and the basis is available.
- **TimeLimit 0:** Status 9, 10005.
- **Non-simplex algorithms:**
  - Method=2 with Crossover=0: Status 2 OPTIMAL but **10005**.
  - Method=6 (PDHG) with Crossover=0: OPTIMAL, 10005. Method=6 is accepted on 12.0.1 and 13.0.2 and rejected on 11.0.3 (10008, maximum 5).
  - With default crossover, both methods give a basis.
  - The wrapper never changes Method or Crossover, so a successful wrapper solve always ends with a basis (INFERRED from the defaults plus the measurements).
- **After a model modification:**
  - While the change is still queued, reads return the **stale** pre-modification basis. This was seen after a ub change, an added row and an added column. Reading with the new length gives 10006.
  - **After the update** (any getter of the wrapper triggers one), the basis is **discarded: 10005**. This held for a bound change, rhs change, objective coefficient change, add row, add column, and remove_variable (whose update is internal).
  - A parameter change does not discard it, and neither does a wrapper getter that updates with nothing pending.
  - X is discarded together with the basis.
- **MIP:** after a solve with an integer column, 10005.
- **Flag:** there is no "basis available" attribute. The key is the 10005 code of GRBgetintattrarray("VBasis"). DOCUMENTED: "Only available for basic solutions".

## E. Warm start (MEASURED on 11.0.3 / 12.0.1 / 13.0.2)

Knob: `IterationLimit`, a double parameter ("Limits the number of simplex iterations performed ... applies to MIP, barrier crossover, and simplex", DOCUMENTED, parameters page). The wrapper sets it with `set_iteration_limit`.

**Fresh model B, then GRBupdatemodel, then a write of A's optimal VBasis and CBasis:**
- **No limit:** OPTIMAL in **0 iterations**, against 39 cold. This held with Method -1, 0, 1 and 3.
- **IterationLimit=0:**
  - Under the default Method -1, and under Method 1, the result is **Status 7 ITERATION_LIMIT**, IterCount 0, ObjVal equal to the optimum 629.581256. X and Pi are readable but SolCount is 0.
  - So the wrapper reports `iteration_limit{solution_available=false}` while X holds the optimum (H6, J4).
  - Method=0 (primal) reports OPTIMAL at limit 0.
  - **IterationLimit=1 reports OPTIMAL with IterCount 0** (H5, J4b, I25).
- **Method=2 (barrier) ignores the basis:** 7 barrier iterations plus crossover.
- **LPWarmStart:**
  - LPWarmStart=0 ignores the basis: 39 iterations.
  - LPWarmStart=2 uses it: 0 iterations.
  - The default (1, or -1 on 13) uses it **and disables presolve**. On the core LP, the default lim0 cold solve gives OPTIMAL because presolve solves it, but the warm one gives ITERATION_LIMIT. This matches the DOCUMENTED presolve note.
- **No extra setting is needed.** The wrapper's defaults (Method -1, Presolve -1, LPWarmStart default) honour a written basis. No crash or factorize call is needed.

**Gotchas that decide whether a write is honoured:**
- **Write before the first update** of a freshly built model, with the columns still queued: the call returns 0, but the write is **silently lost**. Reading back after the update gives 10005, and the solve is cold (39 iterations, H4).
- **On a model that holds a solution:**
  - The write returns 0 and does not discard X or Pi (G1). After an update the attribute **reads back the written basis** (H7, I26).
  - If nothing else changed, the next GRBoptimize is a **no-op**: Status 2 and 0 iterations, with VBasis still reporting the written basis even though X is the old optimum (H7, I16, I26). This holds on all releases.
  - With a modification queued in the same batch:
    - 11.0.3 / 12.0.1 **ignore** the written basis and warm-start from the internal one: 0 iterations with a written slack basis (H8, H9, I17, I23).
    - 13.0.2 honours it for bound changes: 47 iterations (H8, H9, I23).
    - 13.0.2 still ignores it for an objective-coefficient change in the same batch (I17).
  - If the modification was flushed by an update *before* the write, the write is honoured on all releases (I17c, I24).
- **Recipe that is uniform on 11/12/13:** `GRBupdatemodel` → `GRBreset(model, 0)` → write VBasis + CBasis.
  - The written basis is honoured: slack basis 40 iterations, optimal basis 0 iterations (I19, I25, J1-J3).
  - It survives a later bound change (J2, J3) and a later add_column + add_constraint (J1: 0 iterations; Gurobi extends it with the new column nonbasic and the new row basic).
  - After the recipe, Status is 1 (LOADED) and X is 10005 (J6). The written basis reads back only after an update.
  - GRBreset itself flushes queued changes (J5: NumVars 61 after reset).
  - GRBreset with nothing written also drops the internal warm start: 39 iterations (I21).
  - Writing *then* resetting loses the write (I22).
- **Bound change on a fresh or reset model:**
  - Writing then loosening a bound, in the same batch or after an update, keeps the basis (H11, H12, J2).
  - Loosening then writing also keeps it (H13, H14).
  - Adding a row after the write keeps it: the read gives 10005, but the solve takes 0 iterations (H15).

**Invalid basis** (core LP, section E "invalid"; the setter returned **0 in every case**):

| written | read back after update | solve |
|---|---|---|
| too few basic (a basic column set to -1) | 10005: silently discarded | cold (presolve runs again) |
| too many basic (a nonbasic column set to 0) | 10005: discarded | cold |
| CBasis 1 on a nonbasic row | 10005: discarded | cold |
| -1 on a free column (z2) | accepted, kept | repaired, obj 19 |
| -2 on a free column (ub = +inf) | accepted, kept | repaired, obj 19 |
| -1 on t (lb = -inf) | accepted, kept | repaired, obj 19 |
| -2 on a fixed column | accepted, kept | repaired, obj 19 |
| -3 on a bounded column at ub or at lb | accepted, kept | repaired: after a lim0 stop u reads -1; obj 19 |
| VBasis 1 | normalised to 0 | count unchanged, so valid |
| VBasis -4 | normalised to -1 | — |
| CBasis -2 or -3 | normalised to -1 | — |
| singular basis (empty column z2 basic) | accepted | repaired: lim0 stop shows ObjVal 5e30 garbage and the le row made basic; no limit gives 1 iteration, OPTIMAL 19 |

- An accepted basis that needs repair still solved in **0 iterations without a limit**. Cold solves of the core also take 0 iterations under default presolve, so the evidence runs as follows. The read-back is kept (not 10005), so Gurobi accepted the basis. LPWarmStart then disables presolve, which the same basis confirms by giving ITERATION_LIMIT rather than presolve's OPTIMAL in the `default lim0` run. So the `default nolim` IterCount of 0 is a real warm start. At lim0 the accepted cases report ITERATION_LIMIT with ObjVal -0 and X unavailable, which is the same as the *unmodified* core reference basis (its superbasic z2 entry), so this is not a sign of repair.
- No crash and no wrong answer was seen. Detecting a silently discarded basis needs an update plus a read-back (10005).
- Superbasic entries cost something at lim0. With the core reference basis, whose z2 is -3, the lim0 stop reports X unavailable and ObjVal -0. Without a limit it takes 0 iterations. The barrier-derived basis, with z1 at -3, takes 1 iteration (H1, H2).

## F. Partial / extended bases (MEASURED, section F; core LP plus 1 column and 1 row)

- Writing old-length arrays after the model gained a column and a row leaves the new entries unset. The basis is **silently discarded** (10005, cold).
- Full arrays with **the new column at -1 and the new row at 0** are honoured: 0 iterations.
- Full arrays with the new column basic (too many basic) are discarded.
- **VBasis only** or **CBasis only** (full length) is discarded. This matches the DOCUMENTED "must set ... for all constraints and variables".
- A single `setintattrelement` is discarded.
- Every element set one by one through `setintattrelement` is honoured, which is equivalent to the full arrays.
- So only **complete** VBasis and CBasis arrays work. A subset cannot be set natively; the wrapper must fill every entry.
- Gurobi's own extension of an internal basis, after adding a column and a row to a solved model, is new column -1 and new row 0 (F.h, F.i).
- After the update+reset+write recipe, adding entities is fine: Gurobi extends the written basis itself (J1).

## G. Gotchas

1. **Lazy updates.**
   - `set_basis` must call `GRBupdatemodel` before writing, or a write on queued entities is lost (H4).
   - `get_basis` must update before reading. Otherwise a queued modification makes it return a stale basis of the wrong dimensions, or 10006.
   - Updating with a modification pending discards the basis, which then reads 10005.
2. **Solved-model write.** Without `GRBreset(model,0)` the write is ignored or a no-op, and whether it is depends on the release (see E). Use the recipe. The reset discards X, Pi and Status (LOADED), so `set_basis` should `reset_status()`.
3. **Read-back after set_basis.** `get_basis()` immediately after `set_basis()` returns the written basis only after an update, and returns it even though no solve happened.
4. **Presolve.** It is disabled automatically when a valid start is given (LPWarmStart 1), and re-enabled for a cold solve when the basis was discarded.
5. **Iteration limit 0.** A warm optimal basis under the default Method gives ITERATION_LIMIT, not OPTIMAL, with SolCount 0. A limit of 1 gives OPTIMAL in 0 iterations. Shared tests must not expect `optimal` at a limit of 0 on Gurobi.
6. **remove_variable.** It flushes and discards both the basis and the internal warm start (G4: cold crash basis). A snapshot taken before the removal still holds entries for removed handles. If one of those was basic, the written basis has too few basics and Gurobi silently drops it (cold start).
7. **Remapped ids.** Read through `_var_handle(j).uid()` and size the column table with `_handle_id_bound`; write by walking native j and looking up `_var_handle(j)`. A recycled handle id reads the removed entity's status, the same caveat as the IIS snapshot (iis_snapshot.hpp:15-17).
8. **Ranged rows.** Not used by the wrapper. Natively they add a column, so supporting them would require counting range columns.
9. **MIP.** VBasis is unavailable, which is irrelevant for gurobi_lp. A write on a MIP returns 0.
10. **Callbacks and threads.** Nothing basis-specific was observed. The attribute calls are on the model and are not allowed during optimize (INFERRED; Gurobi's general rule).

## H. Recommendation

- **Column status variant (read):** `std::variant<nonbasic_at_lower_bound, basic, nonbasic_at_upper_bound, nonbasic_fixed, nonbasic_free>`.
  - Alternative 0 is `nonbasic_at_lower_bound`, so that handles past the table bound, i.e. columns created after the snapshot, read and write as -1. That is exactly Gurobi's own extension and keeps #basic valid.
  - The repo already tests such a table in test/iis_snapshot.cpp:372-382 (`holds_basis_statuses`).
- **Row status variant (read):** `std::variant<basic, nonbasic_at_lower_bound, nonbasic_at_upper_bound, nonbasic_fixed>`. Alternative 0 is `basic` (new rows basic). There is no `nonbasic_free` for rows.
- **get_basis:**
  1. `_lazily_remove_variables()` and `update_gurobi_model()`.
  2. Read VBasis[n], CBasis[m], LB[n], UB[n] and Sense[m] with n = `_num_var_native_ids` and m = `_lazy_num_constraints`. Call `check()` so that 10005 becomes `solver_error`, as `get_dual_solution` does.
  3. Translate with the read mapping. Columns go to `handle_status_table(_handle_id_bound(n))`, keyed by `_var_handle(j).uid()`; rows go to `table(m)`, keyed by i.
  4. Returning a snapshot by value mirrors the IIS design.
- **set_basis(b):**
  1. `_lazily_remove_variables()` and `GRBupdatemodel`.
  2. **`GRBreset(model, 0)`**, a new symbol. Make it optional (`GRB_OPTIONAL_FUNCTIONS`) and throw when it is null, until 10.x is checked.
  3. Build full int arrays from the native columns j (status of `_var_handle(j)`, or alternative 0 past the bound) and the rows i (status of i, or basic past the bound).
  4. `GRBsetintattrarray("VBasis")` then `("CBasis")`, through `check()`.
  5. `reset_status()`.
  6. No parameter change.
  7. Optional: `GRBupdatemodel` plus a read-back to turn a silently discarded basis into an error. This costs two extra calls; the alternative is to document it as UB.
- **Deviations to rule on:**
  1. The row side is not native. It is derived from Sense on read, and every nonbasic tag collapses to -1 on write: a side the user names on a row is ignored.
  2. `nonbasic_fixed` is not native for columns. It is derived from LB==UB on read and written as -1.
  3. -3 (superbasic) is mapped to `nonbasic_free`. It is observed only on free columns at 0, but Gurobi uses the same code for "between bounds".
  4. set_basis discards the current solution, because of GRBreset. Without the reset the write is silently ignored on a solved model, with behaviour that depends on the release.
  5. The basis is unavailable after barrier or PDHG without crossover, after presolve-decided INF_OR_UNBD or infeasibility, after a time limit hit before simplex, after any modification, and before any solve. In all these cases the getter would throw `solver_error`.
  6. An invalid basis (wrong number of basics, partial arrays) is silently dropped by Gurobi rather than rejected with an error code.
  7. A warm optimal start at an iteration limit of 0 reports `iteration_limit`, not `optimal`. A test should use a limit of 1, or compare results.
  8. One new api symbol is needed: GRBreset. Declare it optional until 10.x is verified.

## Addendum (implementation, 2026-10-09)

- GRBgetintattrarray / GRBsetintattrarray / GRBgetcharattrarray("Sense") with
  len 0 return 0, with a null pointer as with a dummy, on 10.0.3, 11.0.3,
  12.0.1 and 13.0.2 (MEASURED during the implementation).
- Section D: the update that drops the basis is triggered by get_basis() and
  every getter of model data; the solution getters do not update.
- Gurobi 10.0.3 (gurobipy wheel, libgurobi100.so) exports GRBreset; the basis
  suites pass on it. GRBreset is bound as a required symbol.
- Without GRBreset, a slack basis written over a solved model is ignored (0
  iterations instead of 18) on 12.0.1 and 13.0.2: the reset is load-bearing.
