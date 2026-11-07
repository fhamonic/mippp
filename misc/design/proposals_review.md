# Open proposals of misc/design: review of 2026-10-31

The maintainer asked, before tagging 1.1.0, how the feature proposals recorded
in `misc/design` are best implemented, if implemented at all. This note
records the answer. It is not user documentation.

**Scope.** Most of `misc/design` records finished work: `basis/`, the IIS
design and its two paths, the column manager's L1, L3 to L6, L8 to L11 and X1,
and the rest-or issues in `bugs/mippp.md` but three. The open proposals are:

- the LP algorithm selector of `lp_algorithm/lp_algorithm.md`;
- the dual smoother of `dual_smoothing/`;
- the IIS items under "Deferred, and how they would come back" in
  `iis/iis.md`, and N37 (f);
- the basis helper deferred by ruling B1, and the deferred items at the end of
  `basis/basis_plan.md`;
- issues 8 (in part), 9 (its thread-count aside) and 10 of `bugs/mippp.md`.

The column manager's L2 (no park mode) and L7 are settled; the review only
checked that their reasons still hold.

A helper for manual scaling, asked about after the review, has its own
section, [Manual scaling](#manual-scaling-asked-after-the-review). It was
read and probed directly, without agents.

**Method.** Five agents each read one cluster against the tree of
`feat/lp-basis` (`2dc8083` with the uncommitted 1.1.0 changes), the vendor
headers on this machine and the two downstream users, gecot (all branches) and
rest-or. One of them swept every note for proposals missing from the list
above. Four more each tried to refute one cluster's claims and verdicts.
Interrupting a solve and the thread count had no reader of their own: they
came up through the sweep and the IIS verifier, so their "Today" facts are
shallower than the others'. The thread count was surveyed afterwards, the same
day.
Probes ran under `timeout` against CPLEX 22.1.1 and 22.1.2, Gurobi 11.0.3,
12.0.1 and 13.0.2, MOSEK 11.0, Xpress and HiGHS; they were not kept. No file of
the tree was changed. *Measured* below means a probe ran it, *documented* that
a vendor header, manual or source says it, *inferred* a reading of either.

**Status.** Only question 4 is ruled (2026-10-31): `has_thread_limit` is
built on Gurobi, CPLEX, Xpress, COPT and MOSEK for 1.1.0, uncommitted, and
HiGHS is left out for the moment. Nothing else below is implemented or ruled,
nor 1.1.0 scope, except the paper's roadmap sentence under
[Corrections to the record](#corrections-to-the-record).

## Verdict

No open proposal is a large feature worth building now. Two downstream users
exist, and neither uses most of them:

- **Small fixes for 1.2**, each a few lines:
  - `cplex_lp` honouring a native `LPMethod`;
  - the MIP gap on `glpk_milp` and `mosek_milp`;
  - COPT's barrier iteration-limit status;
  - a docs paragraph on choosing the LP algorithm through `native_api()`;
  - a docs section on scaling a model by hand, which is the helper for
    manual scaling worth having.
- **Ruled and built for 1.1.0:** a thread limit (gecot, rest-or) on Gurobi,
  CPLEX, Xpress, COPT and MOSEK; HiGHS, with a per-thread pool, left out.
- **Features with recorded demand, each needing a ruling:**
  - interrupting a solve (rest-or);
  - the LP algorithm selector (a gecot research note);
  - protected sides on the deletion path (your N37 f ranking).
- **Waiting for demand:** the dual smoother, and every other IIS extension.
  The dual smoother lost both of its field tests.
- **Blocked:** special constraints as IIS members, on SOS and on indicator
  handles.

| Item | Source | Verdict | Size | Demand |
| --- | --- | --- | --- | --- |
| `cplex_lp` runs `CPXlpopt` | lp_algorithm.md | 1.2 | 1-2 lines + 1 test | the escape hatch is broken |
| MIP gap on `glpk_milp`, `mosek_milp` | bugs/mippp.md issue 8 | 1.2 | ~20 lines, suites exist | parity, rest-or |
| COPT `LpStatus` 11 | lp_algorithm.md | 1.2, bundle | ~5 lines + 1 test | none (native-only path) |
| LP algorithm through `native_api()` | lp_algorithm.md | 1.2, docs | ~30 lines + 1 snippet | the answer to "why no selector" |
| Scaling a model by hand | asked after the review | 1.2, docs | ~60 lines + 1 snippet | gecot (C5, `NumericFocus`) |
| `(e <= b) / r`, `r * (e <= b)` | asked after the review | ruling | ~20 lines + tests | one `/ S` per side does it |
| Interrupting a solve | bugs/mippp.md issue 10 | ruling, then survey | per backend | rest-or |
| Thread limit | bugs/mippp.md issue 9 | ruled, built for 1.1.0; HiGHS out | a few lines × 5 backends + 1 suite | gecot (native on Gurobi), rest-or |
| LP algorithm selector | lp_algorithm.md | later | ~700-1000 lines | gecot research note |
| Protected sides, deletion path | iis.md, N37 (f) | later | ~150-250 lines | your ranking |
| Dual smoother | dual_smoothing/ | wait | ~150 lines + tests | lost both field tests |
| Lagrangian bound in the guide | dual_smoothing.md §4.3 | later, docs | ~15 lines | none |
| Elastic relaxation | iis.md | wait | 6 backends | none |
| Several IISs on Xpress | iis.md | wait | 3 bindings | none |
| SCIP 10's IIS finder | iis.md | wait | 1 backend | none |
| `compute_iis(stop_token)`, `(duration)` | iis.md, N29 b, N31 b | with interrupts | small | none |
| Dual rays | iis.md | wait | 7 backends | none |
| Integrality as a candidate | iis.md, N25 | wait | ~1 week | your intuition |
| Batching caller | iis.md, N3 b | wait or remove | small | none |
| Special constraints as members | iis.md | blocked | — | none |
| Basis helper from values | basis.md B1 | wait or drop | small | none |
| Iteration limits on 4 LP backends | basis_plan.md B12 b | wait | small | none |
| Filter restores the basis | basis_plan.md | wait | small | none |

## Small fixes for 1.2

### `cplex_lp` honours a native `LPMethod`

- **Today.** `cplex_lp::solve()` calls `CPXprimopt`
  (`cplex_lp.hpp:125`, and `:147` in `refine_lp_status`).
  - It came with the first `cplex_lp` (`759b20f`), with no stated reason.
  - `cplex_milp` uses `CPXlpopt` on LP-typed problems since `8780fcf`.
  - A native `CPXPARAM_LPMethod` is silently ignored: 2 and 4 both ran the
    primal simplex (measured, 22.1.1 and 22.1.2).
  - `cplex_lp` is the only backend with a native algorithm parameter where
    `native_api()` fails this way.
- **Measured with `CPXlpopt`** on both releases:
  - The 7 `LpStatus` cases give the same statuses, and so do the warm-start
    cases.
  - CPLEX's automatic choice runs:
    - the dual simplex from scratch (25 iterations against 33);
    - the primal after `add_column` (same counts);
    - the dual after `add_constraint` (25/3/2 against 57/8/12).
  - `CPXlpopt` refuses a problem turned MIP with error 1017, like
    `CPXprimopt`, so `test/solvers/cplex.cpp:290-308` stays valid. Only its
    comment changes.
  - `TimeLimitTest`'s size-800 LP still stops at 0.2 s. Its unlimited solve
    drops from 0.677 s to 0.290 s, so the stop margin shrinks from about 3.4×
    to 1.45×. The next size exceeds the Community edition's cap.
- **Options:**
  - **(a)** `CPXlpopt` with CPLEX's default: the vendor's choice, and faster
    here. It changes `cplex_lp`'s default algorithm, gecot's CPLEX runs
    included.
  - **(d)** `solve()` reads `LPMethod`: `CPXprimopt` at 0, `CPXlpopt`
    otherwise. No default changes, but an explicit 0 would mean primal, not
    automatic.
  - **(c)** write `LPMethod` = 1 in the constructor. A user reading the
    parameter back sees a value they never set.
- **Recommendation.** (a), with a *Changed* line in the CHANGELOG; (d) if the
  default must not move in a minor release. Keep `refine_lp_status` on
  `CPXprimopt`, which re-solves without an advanced start and without primal
  or dual reductions to tell infeasible from unbounded. Add one backend test that a native `LPMethod` is honoured.

### The MIP gap on `glpk_milp` and `mosek_milp`

- **Today.**
  - Issue 8 was fixed on every MILP backend but these two.
  - `glpk_milp` hard-codes `mip_gap = 1e-4` (`glpk_milp.hpp:43`).
  - `mosek_milp` has no setter, though its knob, `MSK_DPAR_MIO_TOL_REL_GAP`
    (`mosek.h:519`), is reached through `putdouparam` and `getdouparam`,
    which are already bound.
  - Both suites are commented out (`test/solvers/glpk.cpp:595-596`,
    `mosek.cpp:445-446`).
  - `docs/solving/status-and-limits.md:154` lists the other seven backends.
- **Demand.** rest-or's `require_zero_gap` reaches the gap natively on HiGHS
  only, so its exactness claims on GLPK and MOSEK stop at the default gap.
- **Recommendation.** Implement `set_optimality_tolerance` under the
  2026-09-23 ruling, and enable the suites. `glpk_milp` already reports a
  stop at the gap, `GLP_EMIPGAP`, as `optimal` (`glpk_milp.hpp:146`); MOSEK's
  mapping of a gap stop was not checked. The one doubt is whether leaving
  these two backends out was deliberate.

### COPT's barrier iteration limit

- **Today.** `copt_lp` maps `LpStatus` 11 (`COPT_LPSTATUS_ITERLIMIT`, COPT
  8.0's `copt.h:120`) to `unknown`. Its status variant has no
  `iteration_limit` (`copt_lp.hpp:32-61`).
- **Reach.** Only after a native `LpMethod` of 2, 5 or 6 stops on
  `BarIterLimit`.
- **Fix.** Add the alternative and the case, about 5 lines. Declare the
  constant outside the `MIPPP_INCLUDE_COPT_HEADER` branch, since 7.2's header
  lacks it.
- **Release.** 1.1.0 adds status alternatives in a minor release
  (CHANGELOG, "Status variants"), so this can go into 1.2. Bundle it with the
  next change to `copt_lp`.
- **Dropped:** a `glp_interior` path on `glpk_lp`. It would need a second
  solution store, and `get_basis()` would read stale statuses.

### Docs: choosing the LP algorithm

- **Where.** Under "Solver-specific parameters" in `docs/solvers/index.md`.
- **What it says.**
  - `clp_lp` and `glpk_lp` always run the primal simplex, with no native
    route:
    - Clp's entry point is its only selector, and `dd9c448` chose
      `Clp_primal` for correctness: `Clp_initialSolve` missed
      infeasibilities by scaling.
    - GLPK's `glp_smcp` is a private member, so no native call reaches it.
  - The other seven honour their native knob: `cplex_lp` once the fix above
    lands, and Xpress through its `LPFLAGS` control, not the `flags`
    argument.
  - After a crossover-less interior point, `get_basis()` throws.
- **Example.** One compiled Gurobi snippet.
- **Not a table of raw codes.** MOSEK's optimizer codes are positional,
  HiGHS's `solver` values grew across releases, and stub mode declares
  neither `CPXPARAM_LPMethod` nor `XPRS_LPFLAGS`.

### Manual scaling (asked after the review)

- **Source.** No note of `misc/design` proposes it: the maintainer asked
  after the review whether MIP++ should have a helper for it.
- **Demand.** gecot only, on Gurobi. rest-or has none.
  - `include/gecot/solvers/mip_solver_base.hpp:128-131` (main and every
    `path-rr-*` branch) sets `NumericFocus` 1 through `native_api()`: "with
    scores near 1e11, gurobi's absolute tolerances otherwise prune better
    solutions; scaling the models is the real fix".
  - `misc/design/path_rr.md` §3.9, open item C5 (branch `path-rr-rewrite`).
    It divides the contribution coefficients and constants by S = 1e8 on
    green, and π by S in pricing. Green's matrix range is 2e-4 to 5e11, with
    a right-hand side of 1e13, and the unscaled barrier hits its iteration
    limit.
  - Two failures are recorded there. The prototype did not divide the
    criterion's constants, which gave wrong bounds. The pricing margin then
    reads in scaled units.
- **What manual scaling buys.**
  - Every backend but GLPK already scales internally (*documented*, vendor
    headers): Gurobi `ScaleFlag`, CPLEX `CPXPARAM_Read_Scale`, Xpress
    `SCALING`, COPT `Scaling`, HiGHS `simplex_scale_strategy`, MOSEK
    `MSK_IPAR_SIM_SCALING` and `MSK_IPAR_INTPNT_SCALING`, `Clp_scaling`,
    SoPlex `SCALER`.
  - The GLPK wrappers never call `glp_scale_prob`. That GLPK's simplex then
    runs unscaled is *inferred*, to verify.
  - Internal scaling cannot change what an absolute tolerance means next to
    the magnitudes of the user's model (*documented*):
    - Gurobi states its tolerances are absolute, "they do not depend on the
      scale of the quantities involved", and that "a constraint with a 10^10
      right-hand side value is not going to work well with the default
      10^-6 feasibility tolerance".
    - Gurobi meets its optimality tolerance on the scaled objective:
      objective scaling "can lead to large dual violations on the original,
      unscaled objective" (`ObjScale` reference).
      `ScaleFlag` "may lead to larger constraint violations in the original,
      unscaled model". A Gurobi Knowledge Base article says only the LP part
      is scaled, not the MIP part.
    - Clp, CPLEX (for LPs) and SoPlex report an optimum of the scaled problem
      that violates the original as a status, Clp's
      `optimal_infeasible_unscaled` among them. HiGHS checks its continuous
      solvers' tolerances on the original problem.
  - Measured on gecot's Benders masters (Quebec, Gurobi 12.0.1, five failing
    configurations on four seeds each, 2026-10-31; matrix range 1e-13 to
    1e7, right-hand sides up to 8e9, objective coefficients all 1):
    - every `ScaleFlag` level left 8 to 13 of the 20 runs wrong;
    - qualities scaled by 1e-3 (scores near 1e5) or by 1/max (scores near
      32) made all 20 right, as did `NumericFocus` 1;
    - qualities scaled by 1e-6 (scores near 0.1) left 12 of 20 wrong, one of
      them fixed by tightening Gurobi's default absolute `OptimalityTol`. The
      units must suit every absolute tolerance, from below as well.
  - A row divided by S accepts a violation S times the feasibility
    tolerance in its original units. An objective divided by S does the same
    to the reduced-cost tolerance and any absolute gap. The relative MIP gap
    that `set_optimality_tolerance` sets does not move.
  - So the fix is a change of units. Only the user knows the units.
- **Rejected.**
  - A transparent `scaled_model<M>` adaptor, holding row, column and
    objective factors. It would have to rescale every write and unscale
    every read:
    - bounds and objective coefficients, rows, ranged rows;
    - `add_column` entries, the bound and right-hand-side setters;
    - MIP starts, candidate-solution values, lazy constraints;
    - solution values, duals, reduced costs, the dual bound;
    - read-backs such as `get_constraint_lhs`.

    That is the whole surface of `model_concepts.hpp`, and the adaptor would
    still not know the factors.
  - An equilibrator computing the factors in MIP++. The solvers do it
    already, and doing it outside would silently move the tolerances into
    unit-less coordinates.
- **Today's spelling works** (*measured*, `highs_lp`, HiGHS 1.15.1, probe
  not kept).
  - `set_objective((f + 100) * s)` reads an offset of `100 * s`. Scaling an
    expression scales its constant. Scaling the expression as a whole, never
    term by term, therefore avoids gecot's first failure.
  - A row is scaled per side, `e / S <= b / S`, or as `(e - b) / S <= 0`.
    `(e <= b) / S` does not compile.
  - One maximization LP was solved as written, then with three row factors,
    one column factor and an objective factor. Solution, objective, duals and
    reduced costs agreed after the conversions below.
  - The basis statuses agreed. One more run multiplied a `>=` row by −2,
    which turns it into a `<=` row. That row then read
    `nonbasic_at_upper_bound` where the original read
    `nonbasic_at_lower_bound`.

  Conventions: row i multiplied by r_i, column j replaced by c_j · x′_j, the
  objective multiplied by s.

  | Quantity | Original, from the scaled model | |
  | --- | --- | --- |
  | value of x_j | c_j · x′_j | measured |
  | objective value | obj′ / s | measured |
  | dual of row i | r_i · y′_i / s | measured |
  | reduced cost of x_j | d′_j / (s · c_j) | measured |
  | basis statuses | unchanged for r_i, c_j > 0 | measured |
  | dual bound | bound′ / s | inferred |
  | IIS membership | unchanged | inferred |

- **Recommended: a docs section, 1.2.**
  - **Where.** A section after "Tolerances" in
    `docs/solving/status-and-limits.md`, since the reason for scaling is
    there.
  - **What it says.**
    - When to scale: a coefficient range the solver's log flags (with
      `set_verbose`).
    - Scale whole expressions. Scale a row per side or on one side. Divide
      the three arguments of `add_ranged_constraint` alike.
    - After the build, scale what touches the model:
      - an `add_column` entry by r_i · c_j, and its objective coefficient by
        s · c_j;
      - the right-hand-side and bound setters;
      - MIP starts (x′ = x / c);
      - candidate-solution values and lazy constraints.
    - The table above.
    - Column generation. Price with unscaled duals, or stay in scaled units
      throughout. The column manager's `reduced_cost` is in the units the
      caller feeds it, and margins are stated in one unit.
    - What each tolerance then means.
    - Positive factors only: a negative one flips the row's nonbasic side.
    - The native knobs, through `native_api()`.
  - **Example.** A compiled snippet in `test/doc_snippets` that pins the
    conversions, the probe above turned into a test.
  - **Size.** About 60 lines of docs and an 80-line snippet.
- **Offered, not recommended: scaling a constraint as a whole.**
  - **What.** `(e <= b) / r` and `r * (e <= b)` in `linear_constraint.hpp`,
    a `linear_constraint_view` over the scaled expression. About 20 lines
    with tests.
  - **For.** A whole row cannot lose its right-hand side, which the per-side
    spelling allows.
  - **Against.**
    - One `/ S` per side does the same.
    - The demand is one project.
    - It does not reach `add_ranged_constraint`, which takes its sides as
      arguments.
  - **Ruling needed.** A negative factor: flip the sense (the measured basis
    flip follows), or make r > 0 a documented precondition (question 14).
- **Native user scales: a docs line, no concept.**
  - Three backends of ten have a user objective scale (*documented*):
    - Gurobi: `ObjScale`, a double;
    - HiGHS: `user_objective_scale` and `user_bound_scale`, exponents of 2;
    - Xpress: `OBJSCALEFACTOR`, a power of 2.
  - CPLEX, COPT, MOSEK and Clp's C API have only automatic scaling switches.
  - `ObjScale` does not serve gecot's case (*measured*, same grid): its
    objective coefficients are all 1, the magnitudes are in the right-hand
    sides and the matrix. `ObjScale` 1e6 left 13 of 20 runs wrong, and 1e8
    returned the empty solution as optimal on every `target_benders` run.

## Features with recorded demand, each needing a ruling

### Interrupting a solve (issue 10)

- **Demand.** rest-or cancels jobs only between solves. At shutdown it calls
  `std::quick_exit` past workers stuck in a solve
  (`src/app/main.cpp:88-96`), and its `cancel_job` admits the gap
  (`src/mcp/tools.cpp:242-245`).
  - This is the strongest demand of any item here.
  - It also covers compute_iis(stop_token) (N31 b) and the filter's trials:
    one model capability would serve `solve()`, the native IIS routines and
    the filter.
- **Ruling.** The record of 2026-09-23 says "GLPK/CLP time limits and
  interrupts: nothing to do for now".
  - The time-limit half was reversed since (`e2d577f`, `577f90e`).
  - The review reads the line as about those two backends only, not as a
    refusal of the feature.
- **Today.**
  - `COPT_Interrupt` is bound and used by the wrapper's own callback.
  - `GRBterminate`, `CPXsetterminate` and `XPRSinterrupt` are not bound.
  - HiGHS's callbacks are declared but not loaded.
  - Most status variants already list `status::interrupted`: Cbc, COPT,
    CPLEX, Gurobi, MOSEK, SCIP, `glpk_milp` and `highs_qp`. The Xpress
    models, `highs_lp`, `highs_milp`, `clp_lp`, `glpk_lp` and `soplex_lp`
    would gain it.
- **Shape.** A capability `has_interrupt<T>` with either:
  - an `interrupt()` callable from another thread; or
  - a `solve(std::stop_token)` overload whose `std::stop_callback` calls the
    native entry point.
- **Survey first, like the basis**, for:
  - thread safety, which no local vendor manual covers for Gurobi or COPT.
    Xpress's manual anticipates `XPRSinterrupt` from outside a callback
    (documented);
  - which backends have no entry point (Clp, SoPlex, Cbc not checked;
    GLPK's only from its MIP callback);
  - HiGHS, where only `kCallbackSimplexInterrupt` survives into the IIS
    search (`HighsInterface.cpp:1976-1983`, documented).

### Thread count (issue 9's aside)

- **Demand.**
  - gecot sets Gurobi `Threads` = 1 through `native_api()` for its pricing
    sub-models, MILPs solved in parallel (`tree_formulation_rr.hpp:58-61`).
    On every other backend they run at the solver's default.
  - gecot's `--max-threads` sizes only its TBB pool, so Gurobi used all 16
    cores in the scaling runs of 2026-10-31, and which runs failed depended on
    the core count (measured, see [Manual scaling](#manual-scaling-asked-after-the-review)).
  - rest-or runs several solves at once (`bugs/mippp.md`, issue 9).
- **Today.** Nothing in `include/`. Each model owns its native environment
  (`CPXopenCPLEX`, `MSK_makeenv`, `COPT_CreateEnv` per model, Gurobi's model
  copy of its env, Xpress controls per problem), so a native setting already
  stays with its model.
- **Survey** (2026-10-31, measured on 16 hardware threads). A market split
  MILP (m = 6) under a 2 s limit, and a dense 900 × 900 LP; CPU time over wall
  time during the solve, then the peak OS thread count:

  | Backend | Native control | Automatic | MILP default | MILP at 1 | LP default |
  | --- | --- | --- | --- | --- | --- |
  | Gurobi 12.0.1 | `Threads` | 0 | 14.6, 16 | 1.0, 1 | 3.4, 8 |
  | CPLEX 22.1 | `CPXPARAM_Threads` (1067) | 0 | 12.5, 16 | 1.0, 1 | 1.0, 1 |
  | Xpress 47.01 | `XPRS_THREADS` (8278) | -1 | 12.0, 23 | 1.0, 1 | 1.0, 1 |
  | COPT 8.0 | `Threads` | -1 | 9.0, 43 | 1.0, 2 | 2.4, 8 |
  | MOSEK 11.0 | `MSK_IPAR_NUM_THREADS` (100) | 0 | 5.2, 9 | 1.0, 2 | 2.9, 9 |
  | HiGHS 1.15.1 | `threads` | 0, half the cores | 1.6, 8 | 1.0, 1 | 1.1, 8 |
  | Cbc | `threads`, with `CBC_THREAD` only | none, 0 is serial | serial | 2.3, +1 | — |
  | SCIP 9.2, GLPK, Clp, SoPlex | none | — | serial | — | — |

  - A limit set after a first solve applies to the next solve of the same
    model on the five commercial backends. Xpress and MOSEK keep their pools
    alive (17 and 9 threads) at a CPU ratio of 1.0.
  - `cplex_lp` runs `CPXprimopt`, which is serial; CPLEX threads an LP only
    once the `CPXlpopt` fix above lets `LPMethod` pick the barrier or
    concurrent. Xpress chose a serial dual simplex on this LP.
  - Cbc: `threads` n ≥ 1 turns on parallel branch and bound with n workers,
    and 100 + n makes it repeatable (`CbcParameters.cpp:1700`, master). It
    requests parallelism; it does not cap it. A limit would map to the
    default 0.
  - SCIP: `parallel/maxnthreads` governs `SCIPsolveConcurrent` only, and
    `scip_milp::solve` calls `SCIPsolve` (`scip_milp.hpp:620`).
  - OpenBLAS: the Cbc and SCIP libraries on this machine start a 15-thread
    OpenBLAS pool at load, which raised their CPU ratio to 1.44 and 1.17.
    `OPENBLAS_NUM_THREADS=1` brings both to 1.0 and one thread. That pool is
    process-wide, outside any model's reach.
- **HiGHS needs a contract.** Its scheduler is one per calling OS thread
  (`static thread_local ExecutorHandle` in `HighsTaskExecutor.h`, in 1.10,
  1.12, 1.14 and 1.15, documented), sized by the solve that creates it. From
  1.12, `Highs_destroy` resets the pool of the calling thread
  (`highs_c_api.cpp:176-179`, absent in 1.11), so the pool lives until a HiGHS
  model is destroyed on that thread (documented, measured):
  - 4 threads each solving a default model reached 34 OS threads: 4 pools of
    8 (measured). gecot's 16 TBB workers on HiGHS hold 16 pools of 8
    (inferred).
  - While the pool lives, a solve on the same thread with a different
    nonzero `threads` fails, for another model and for the same model solved
    again; with the first model destroyed, it runs (measured).
    `Highs_run` returns an error and MIP++ throws `HiGHS kHighsStatusError`;
    the reason ("Option 'threads' is set to %d but global scheduler has
    already been initialized", `Highs.cpp:5112-5124`) goes to the log, which
    MIP++ silences.
  - A TBB `parallel_for` runs tasks on its calling thread too, so a master
    solved there at the default, then a sub-model at 1 on the same thread,
    hits this (inferred).
  - `Highs_resetGlobalScheduler(blocking)`, in the C API of 1.14 and 1.15,
    destroys the calling thread's pool; after it the second solve ran
    (measured). `Highs.h:1327` still says it is "not thread-safe to call this
    function while calling run() or presolve() on any other Highs instance in
    any thread", although the handle is now thread-local.
- **Recommendation.** Worth building: two users, five backends that use 5 to
  15 cores by default, one native control each, a few lines per backend.
  - `has_thread_limit`: `set_thread_limit(std::size_t n)` with n ≥ 1, and
    `get_thread_limit()`, 0 meaning the solver's default. A cap on the solver's
    own threads: `copt_milp` and the MOSEK models still run a second thread
    at 1, though on one core in all, and a linked BLAS keeps its pool.
  - Native only, like `has_iis`: Gurobi, CPLEX, Xpress, COPT and MOSEK, LP
    and MILP. The serial backends and Cbc lack it, and code guarded by
    `if constexpr` leaves them serial anyway.
  - HiGHS joins if the backend resets the calling thread's pool when the
    limit differs, after a stress probe shows that reset safe while other
    threads solve. A documented precondition instead would break the TBB
    pattern above.
- **Ruling (2026-10-31).** Built on the five commercial backends, LP and
  MILP; HiGHS left out for the moment. As built, `set_thread_limit(0)`
  restores the default and n ≥ 1 caps; a value above the solver's maximum is
  stored as that maximum, queried on Gurobi and CPLEX, which reject larger
  values. See `docs/solving/status-and-limits.md#threads`.

### LP algorithm selector

- **Demand.** gecot's `path_rr` note, on branch `path-rr-rewrite`, §3.9
  (`44f35ed`, 2026-10-29):
  - A barrier master without crossover (Gurobi `Method` 2, `Crossover` 0)
    gives central duals. It took 1.75 to 2.7× fewer rounds on Quebec, and
    449 to 56 rounds on bike_small.
  - The note says: "Gurobi only (`native_api`) until mippp has an
    lp_algorithm selector".
  - It rates the result a small effect, from prototypes that are "Research,
    not ready to land".
  - The same recipe needs `BarConvTol` and the dual objective, so native
    calls would remain anyway.
  - rest-or sets no algorithm.
- **Verdict.** Later, when gecot lands the barrier master.
- **Shape if built.** A per-backend variant, like `basis_status`:
  - `automatic`;
  - `simplex`, with `primal_simplex` and `dual_simplex` under it;
  - `interior_point`, with `interior_point_without_crossover` under it.
- **Tag set.**
  - The crossover-less tag is the one the demand asks for.
  - It does not break `get_basis()`, which already throws after a
    crossover-less interior point "which only native settings can ask for"
    (`docs/solving/basis.md:95`). Only that clause changes.
  - It needs solution getters that read the interior solution. `mosek_lp`
    already falls back to its interior slot; CPLEX needs `SolutionType` 2
    written by the setter.
  - No `first_order` or `concurrent` tags: crossover defaults differ between
    solvers (Gurobi and COPT cross over, Xpress and HiGHS do not), and Gurobi
    PDHG needs 12.0.
- **Setter only.** Prefer a setter-only `has_lp_algorithm<T>`. A getter must
  either decode native values outside the list (CPLEX sifting, Gurobi
  `Method` 3 to 6) or cache a member that lies after a native write.
- **Per backend.**
  - Clp gains `Clp_dual`, and GLPK a member that `solve()` reads.
  - COPT has no primal simplex value.
  - Keep `automatic` as each backend's current behaviour, so nothing changes
    by default.
- **Inheritance.** The deletion filter's trials and `refine_lp_status` would
  run under the chosen algorithm: cold trials under an interior point.
- **`*_milp` stays out** (ruling of 2026-07-22). A root-relaxation
  algorithm remains native.
- **Dropped questions:**
  - `get_algorithm()` reporting what ran;
  - what `set_iteration_limit` bounds under a non-simplex algorithm, which is
    moot without the selector.

### Protected sides on the deletion path (N37 f)

- **Demand.** None downstream; gecot and rest-or make no IIS call. N37 (f)
  ranks it first among IIS extensions. Whether that means "in 1.2" or only
  "first" is your call.
- **In-library motive.** Keeping binary bounds avoids N25's throw on
  `scip_milp`. Keeping integer bounds avoids endless branching on
  `glpk_milp` and `cbc_milp` (`infeasibility.md:255`).
- **The workaround is costly.** The public engine over a user's own oracle is
  the documented route. Over a MIP++ model, that oracle must redo the guard,
  and the trial classifier is in `detail`.
- **Shape.** An additive overload taking the kept sides as anything with
  `get_status(v)` and `get_status(c)` returning `lp_iis_status`: the protocol
  `set_basis` and the narrowing overload already read, not a new selector
  concept.
  - Kept sides stay enumerated, because `_write_row` reads the other side of
    a sense-and-rhs row from the candidates.
  - They read `absent`, like SOS background.
- **Two rulings to amend.**
  - N36: a kept-kept crossed pair answered without a solve is a second
    zero-solve precheck.
  - The 2026-10-05 rejection of a per-side role callback, one of whose two
    reasons ("ships N37 f early") lapses with 1.1.0.
- **Native half.**
  - Gurobi documents `IISConstrForce`, `IISLBForce` and `IISUBForce` as
    "ignored for LPs". Measured on 11.0.3, 12.0.1 and 13.0.2: ignored on an
    LP under `IISMethod` -1 and 0, honoured on a model with integer columns
    under -1, 0 and 1. So native forcing is `gurobi_milp` with integer
    columns only, never `gurobi_lp`. A `gurobi_milp` without them is an LP to
    Gurobi.
  - On CPLEX, leave the kept sides ungrouped, as the wrapper already does for
    SOS. A preference of 0 flags unneeded groups as possible members, which
    the decoder turns into `incomplete` (measured).
- **Candidate order.** Defer it: CPLEX's preferences had no effect on the
  probe model, and no other routine takes an order.

## Waiting for demand

### Dual smoother

The sketch still builds and runs on the current tree and reproduces §2.3
(measured). It never touches the model, so X1 is no hazard. The evidence
against it:

- **gecot `path_rr`.** Its copy sits on the unmerged branch
  `path-rr-smoothing`. gecot's own note lists it under "measured and set
  aside":
  - it loses on Quebec at every α, and the best fixed α is 0;
  - it multiplies the dual-keyed search reuse's searches by 13.
- **gecot `tree_rr`**, the named first user.
  - α = 0.5 was 11% slower in total (§2.2), with no committed code.
  - Its main branch now skips a pricing whose duals equal those of the last
    unproductive one (`c983ada`), skipping 28 to 46% of pricings on Aix.
    Smoothing moves the duals every round and would defeat that skip.
- **Adaptive α**, with GCG's constants on the sketch's cutting stock, is worse
  than no smoothing on 3 of 7 instances (measured).
- **The bound.** The bound the helper gives for free is about six lines
  gecot already writes.

Wait for demand, or drop. The literature reports wins on other shapes (GAP,
lot sizing), so only a measured win on a real user should reopen it.

**If it is ever built**, these sub-decisions are settled or narrowed:

- **The sense.**
  - Option one: an `update()` returning the round's value, and an explicit
    `move_center()`. That leaves the comparison to the user, as `negative`
    and `positive` do under L3, and needs no sense vocabulary in `colgen`.
  - Option two: a sense input without a default.
  - Not a `Better` parameter.
  - An `enum class objective_sense` in `mippp` would be hidden inside
    `cbc_milp` by its member `objective_sense`.
- **The mispricing schedule.**
  - GCG re-prices at α itself from the moved center. The sketch uses
    `1 − (k+1)(1−α)`.
  - Over 21 configurations, GCG's indexing costs more rounds in 14, and one
    more round at α = 0.5 in 6 of 7 (measured).
  - GCG's indexing also re-prices at the same point when a mispricing round
    keeps the center, on a tie for example.
  - The data leans to the sketch.
- **Adaptive α.** GCG's rule is in `class_stabilization.cpp`:
  - constants: 0.8 at the start, `min(0.9, α + 0.1(1−α))` to increase,
    `max(0, α − 0.1)` to decrease;
  - its comment says the paper has the sign wrong
    (`class_stabilization.cpp:619-621`, documented).
  - The paper itself was not read.
- **α out of range.** Reject α outside [0, 1) at both ends: α < 0
  extrapolates past π_out, and the bound silently stops being one (inferred).
- **Eviction.** Evict only after the mispricing loop, or the X1 guard throws
  on the read after convergence. The guide's activate-then-evict round would
  need that change.

**Independent of the class:** a paragraph on the Lagrangian bound and early
termination in `docs/algorithms/column-generation.md`. Both column generation
examples already apply the bound at termination
(`vehicle_routing_colgen/main.cpp:440-446`,
`travelling_salesman_colgen/main.cpp:230-235`). Low priority.

An objective-sense getter is cheap: every backend has a native one, and HiGHS,
GLPK, CPLEX and MOSEK already load it. But its only consumer would be the
smoother.

### IIS: native extras

- **Elastic relaxation.**
  - Six routines: `CPXfeasopt`, `GRBfeasrelax`, `COPT_FeasRelax`,
    `XPRSrepairinfeas`, `Highs_feasibilityRelaxation`, `MSK_primalrepair`.
  - They share only the weighted linear sum.
  - Four solve and leave the model unchanged: CPLEX and COPT (measured),
    HiGHS (per its source), Xpress (per its manual).
  - `GRBfeasrelax` adds columns and needs an optimize afterwards
    (documented). It is the only one that needs a copy or an undo.
  - `MSK_primalrepair` changes bounds only and does not solve (documented,
    MOSEK 11 C API manual), so the save-and-restore of the deletion filter
    covers it.
  - The concept would be `has_feasibility_relaxation<T>`, single-parameter
    like `has_iis`.
  - Under X1, the call must mark results stale on CPLEX and HiGHS, which
    leave the relaxed point readable.
  - A hand-written elastic model, which MIP++'s modeling makes a few lines on
    every backend, is the answer until a user asks.
- **Several IISs on Xpress.**
  - The docs already show a several-conflicts loop on every backend
    (`infeasibility.md:142-152`).
  - If asked, bind `XPRSiisnext`, `XPRSiisall` and `XPRSiisstatus` for
    `native_api()` only. COPT's `GetSOSIIS` and the unused `CPXfeasopt` set
    that precedent.
  - Drop a wrapped member.
- **SCIP 10's IIS finder.**
  - It is a batched deletion filter, which `scip_milp` already runs.
  - `pub_iisfinder.h` has no variable map. `SCIPcopyOrig` keeps names, so a
    rename-and-match mapping would work, at 2n writes per call.
  - No SCIP 10 shared library is on this machine, though the compat harness
    has run 10.0.0 to 10.1.0.
- **`compute_iis(stop_token)` and `compute_iis(duration)`.**
  - Decide them with interrupting a solve.
  - Two overloads make `compute_iis({})` ambiguous (measured).
  - Today's idiom covers a duration on every native class: save, set and
    restore the time limit.
  - A name `compute_iis_within` would clash with the `within` parameter of
    `compute_iis_by_deletion`.
  - Adding `interrupted` to the HiGHS and Xpress outcome variants is optional:
    N44 (c) folds it into `incomplete` and `stopped`.

### IIS: data extras

- **Dual rays.**
  - No consumer: gecot's Benders branch adds optimality cuts only, and column
    generation starts feasible. For the IIS alone, a ray helps `clp_lp` and
    `mosek_lp` only.
  - If a consumer comes (Benders feasibility cuts, Farkas pricing), add
    `get_dual_ray()` returning `std::optional` of a handle-indexed mapping.
    Not `feature_unavailable_error`: two-tier-version-support reserves it for
    a member the release lacks, and holding no ray is a legitimate outcome.
  - Seed the filter through the narrowing overload, whose first trial already
    verifies the seed.
  - SoPlex stays blocked upstream.
- **Integrality as a candidate.**
  - The setters exist (`model_concepts.hpp:246-248`); only a getter is
    missing.
  - If one comes, make it two-way, `is_integer(v)`. Binary is bounds-derived
    on GLPK (documented), emulated on Cbc, MOSEK and HiGHS, and on Xpress a
    bound write turns `B` into `I` for good (measured).
  - Relaxing a column to continuous cannot cause endless branching, and
    tested before the bounds it avoids N25's throw. Only the demand is
    missing.
- **Batching.**
  - The trade-off is simulated in solve counts only, on a one-IIS oracle: a
    group trial changes up to 64 sides, and real models hold several IISs.
  - Any threshold needs wall-clock runs on real LPs.
  - It conflicts with candidate order.
  - Keep it dormant, or remove it (27 header lines, 96 test lines; the
    archive tag keeps it).

### Smaller deferred items

- **The basis helper from values (B1).**
  - No user; column generation reads statuses from `get_basis()`.
  - A nonbasic side is advisory: a wrong side costs one bound flip. The
    helper also cannot choose the basic entities, which is what a basis built
    from values needs.
  - Wait, or drop.
  - A separate, smaller question: an additive
    `basis_snapshot::set_status(e, status_variant)`. Copying a status between
    snapshots needs `std::visit` today (measured).
  - A request for "a basis from a solution" would instead mean a native LP
    start with crossover, which is a larger proposal (`Highs_setSolution` and
    `Highs_crossover`, Gurobi `PStart`/`DStart`, `CPXcopystart`,
    `XPRScrossoverlpsol`, `COPT_SetLpSolution`).
- **Iteration limits on `clp_lp`, `glpk_lp`, `xpress_lp`, `mosek_lp`
  (B12 b).** Cheap on Clp and GLPK. On Xpress and MOSEK, which limit applies
  depends on the algorithm, which ties them to the selector.
- **The deletion filter restoring the caller's basis.** The documented
  "unspecified after `compute_iis_by_deletion`" covers it.

## Blocked

- **Special constraints as IIS members.**
  - SOS is a roadmap row that no backend implements.
  - `add_indicator_constraint` returns `void` on `gurobi_milp` and
    `cplex_milp`.
  - An indicator handle is cheap and source-compatible on its own, so for
    indicators "blocked" means "not started".
  - COPT's routine already takes native SOS and indicators as candidates
    (`copt_base.hpp:769-773`). Member tables would turn its `incomplete`
    answers into `irreducible` ones, while the change of default answers
    concerns Gurobi, CPLEX and Xpress only.

## Dropped

- Penalty and box-step stabilization (O8): they change the master.
- `set_out_point(model, duals)` (O7): F1 is resolved in 1.1, because every
  model has `constraints()`.
- A wrapped several-IISs member on Xpress.
- Basis support on `dumb_lp`: HiGHS and Clp prove the basis suites in CI.
- The WP3 archive pointers: the tag `archive/pr3-a1a9f11` keeps the code.
- A `scaled_model<M>` adaptor and an equilibrator in MIP++: see
  [Manual scaling](#manual-scaling-asked-after-the-review).

## Closed, confirmed

- **Column manager L2.** `remove_variable` is still on Clp, CPLEX, Gurobi
  and HiGHS only, and the pool state still holds no variable, so a park by
  bound would still leak.
- **Column manager L7.** `<execution>` is not used anywhere; libstdc++ 15
  still needs TBB for a parallel backend.
- **Done as the notes record:** L1, L3 to L6, L8 to L11, X1, `basis/`, both
  IIS paths, and rest-or issues 1 to 7, 9 and 11.

## Corrections to the record

- **The paper.** `paper/paper.md:140-142` lists "explicit LP basis
  warm-starts" as roadmap. 1.1.0 ships them on every LP backend (read-only
  on SoPlex). This is the one item for the tag.
- **iis.md.**
  - `iis.md:1382-1384` and `iis_todo.md:586-587` say `milp_model` lacks a
    variable-type setter. It has three; only a getter is missing.
  - N37 (f) and `iis.md:1359-1363` say native forcing on Gurobi. It holds on
    `gurobi_milp` with integer columns only.
- **Xpress after a deletion run.** `docs/solvers/index.md:132` documents that
  Xpress turns a binary column into an integer one when a written bound
  leaves [0, 1]. Not documented: restoring the bounds to [0, 1] does not turn
  it back (measured). So on `xpress_milp`, a deletion run that relaxed a
  binary column's bound returns it typed `I` on [0, 1]. The feasible set is
  the same, but the docs say the model comes back with its data restored.
  Either add a line to the docs or write the type back.
- **`mosek_lp` after a primal-infeasible solve.** `get_dual_solution()` and
  `get_reduced_costs()` return the Farkas certificate (measured), because the
  slot choice ranks `PRIM_INFEAS_CER` like `OPTIMAL`. `solutions.md:27`
  leaves that read unspecified.
- **dual_smoothing.md.**
  - §3.4's second snippet uses the API removed in `648f637`.
  - §2.1 says the gain is smallest on bin packing, while its table shows
    vehicle routing.
  - F1 is resolved.
- **column_manager_tree_rr.md.** L3 (b) and §5 item 2 still cite the shared
  `Better` that the L3 ruling rejected.
- **glpk_lp's defaults.** `glpk_lp` departs from GLPK's simplex defaults on
  pricing and ratio test (`GLP_PT_STD`, `GLP_RT_STD` against PSE and HAR), not
  on the method.

## Questions for the maintainer

1. `cplex_lp`: change the default to CPLEX's automatic choice (a), or keep
   the primal simplex and honour a native setting only (d)?
2. Was leaving `glpk_milp` and `mosek_milp` without
   `set_optimality_tolerance` deliberate?
3. Does "interrupts: nothing to do for now" (2026-09-23) cover GLPK and Clp
   only, or interrupting a solve in general? If only those two, survey the
   backends for 1.2?
4. Thread limit: build `has_thread_limit` on Gurobi, CPLEX, Xpress, COPT and
   MOSEK? And on HiGHS, whose pool is per calling thread: reset that pool when
   the limit differs (after a stress probe), document the first solve on a
   thread as fixing it, or leave HiGHS out?
   *Ruled 2026-10-31: the five, built; HiGHS left out for the moment.*
5. N37 (f): protected sides in 1.2 regardless of demand, or first only once
   IIS work resumes?
6. Update the paper's roadmap sentence at the 1.1.0 tag, given the pending
   JOSS submission?
7. `mosek_lp`'s Farkas certificate through `get_dual_solution()`: document
   it, leave it unspecified, or guard it?
8. Xpress's `B` to `I` after a deletion run: a docs line, or a type
   write-back in the guard?
9. The dormant batching: keep it, or remove it?
10. The dual smoother: close its note as "set aside", run an experiment on a
    shape that suits it first, or keep it open?
11. Should `add_indicator_constraint` return a handle in 1.2, independently
    of the IIS?
12. Add `basis_snapshot::set_status(e, status_variant)`?
13. Annotate the design notes with the corrections above, or leave them as
    records of their dates?
14. Manual scaling: the docs section only, or also `(e <= b) / r` and
    `r * (e <= b)`? If those, does a negative factor flip the sense, or is
    r > 0 a documented precondition?
