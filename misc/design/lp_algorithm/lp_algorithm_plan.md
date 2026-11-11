> The plan as written before the implementation, kept so that the options and
> evidence behind each ruling letter stay readable.
> [lp_algorithm.md](lp_algorithm.md) records the rulings, the review and what
> the implementation settled: `clp_lp` has no selector, and `copt_lp` no
> crossover-less tag.

# LP algorithm selection: assessment and plan

On 2026-11-02 the maintainer asked how best to add a choice of LP algorithm
(primal simplex, dual simplex, barrier, ...) on the backends that support it.
This note answers. It builds on [lp_algorithm.md](lp_algorithm.md) (the notes
of 2026-10-09) and on the "LP algorithm selector" section of
[../proposals_review.md](../proposals_review.md), and re-measures their
claims. Nothing is implemented or ruled.

**Method.** One probe per LP backend, run against the real libraries, records
which native knob selects which algorithm, witnessed by the solver's own
iteration counters. Each probe also records:

- the status MIP++ maps on a feasible, an infeasible and an unbounded
  120 x 80 LP that presolve does not decide;
- the five LPs of `test/test_suites/lp_status.hpp`;
- which results can be read afterwards;
- whether `set_iteration_limit` binds, and what `set_basis()` does to the next
  solve.

Probes, outputs and releases are in [survey/RECORD.md](survey/RECORD.md).
*Measured* means a probe ran it, *documented* that a vendor header or source
says it, *inferred* a reading of either. Line numbers are on `feat/lp-basis`
with the uncommitted 1.1.0 changes.

## Recommendation in short

- **Shape.** Build it the way `status` and `basis_status` work: a per-backend
  variant over a small tag hierarchy, with
  `set_lp_algorithm(tag)` / `get_lp_algorithm()` on `*_lp` models and one
  concept, `has_lp_algorithm<M, Tag>`.
  - A backend lists only the tags it honours. Any other tag fails to compile;
    there is no fallback to a parent tag. The basis encoders do fall back,
    writing a refinement as its base, because the solver then re-derives the
    true status. Here a fallback would run a different algorithm than the
    one asked for, such as the dual simplex on COPT for `primal_simplex`.
  - The getter decodes the native setting, so a native write stays visible.
- **Tags.**
  - `automatic`;
  - `primal_simplex` and `dual_simplex`, under a parent `simplex`;
  - `interior_point`, which crosses over to a basis;
  - `interior_point_without_crossover`, under `interior_point`.
  - First-order methods, concurrent, network and sifting stay native-only.
- **Where.** Eight LP backends, not SoPlex. Which tags each backend lists is
  in the table below; Clp gets no barrier, COPT has no primal simplex, and
  HiGHS gets no crossover-less tag.
- **Fixes the selector needs.** Four of them, all measured:
  - `gurobi_lp::refine_lp_status()` must re-solve with a simplex;
  - `glpk_lp` must map a dual simplex that proves the dual infeasible;
  - `cplex_lp` must call `CPXlpopt`, so `proposals_review.md` question 1
    comes first;
  - `xpress_lp` must write the `LPFLAGS` control rather than the `flags`
    argument.
- **Size.** About 1000 lines with the tests and docs, in line with the
  review's 700 to 1000 lines.
- **Demand.** One user: gecot's crossover-less barrier master on Gurobi, which
  `interior_point_without_crossover` on `gurobi_lp` covers.

## What the survey found

### Which knob selects what

✓ means three things held: the counters show the asked algorithm ran, the
three 120 x 80 LPs got the right status, and the five `lp_status` LPs passed.
The status columns name what MIP++ reported.

| Model | `solve()` today (default run here) | primal simplex | dual simplex | interior point, crossover | interior point, no crossover |
|---|---|---|---|---|---|
| `clp_lp` | `Clp_primal` (`clp_lp.hpp:663`), the wrapper's choice | `Clp_primal` ✓ | `Clp_dual` ✓ | `Clp_initialBarrierSolve`: **`optimal` on both unbounded suite LPs** | `Clp_initialBarrierNoCrossSolve`: same, and `get_basis()` returns statuses instead of throwing |
| `glpk_lp` | `meth = GLP_PRIMAL` (`glpk_lp.hpp:38`), GLPK's own default | ✓ | `GLP_DUAL`: the unbounded LP reads `unknown` (GLPK says status `GLP_INFEAS`, dual `GLP_NOFEAS`). 1143 iterations against 112, under the wrapper's textbook pricing | `glp_interior` not probed: own status codes and solution store, no basis | — |
| `soplex_lp` | `ALGORITHM` 1, SoPlex's default (dual) | `ALGORITHM` 0 reads back as set, **no effect seen**: equal counts under all three representations | default | — | — |
| `highs_lp` | `solver=choose`: dual simplex | `simplex_strategy` 4 ✓ | `simplex_strategy` 1 ✓ | `solver=ipm`, `run_crossover` on by default ✓ | `run_crossover=off`: **the suite's `max_bounded` reads `unknown`** (model status Unknown, dual point infeasible after postsolve); the unbounded LP makes `Highs_run` return an error, so `solve()` **throws** (model status 9) |
| `gurobi_lp` | `Method` -1: dual | `Method` 0 ✓ | 1 ✓ | 2 ✓ | 2 with `Crossover` 0 ✓, but `refine_lp_status()` throws (below) |
| `cplex_lp` | `CPXprimopt` (`cplex_lp.hpp:125`): primal, whatever `LPMethod` says | `CPXlpopt` + `LPMethod` 1 ✓ | `LPMethod` 2 ✓ | `LPMethod` 4 ✓ | `LPMethod` 4 + `SolutionType` 2 ✓; `Barrier_Crossover` -1 is refused (error 1014, 22.1.2); `CPXbaropt` alone also skips the crossover |
| `xpress_lp` | `XPRSlpoptimize(prob, nullptr)` (`xpress_lp.hpp:102`): dual | `LPFLAGS` 2 or `"p"` ✓ | `LPFLAGS` 1 or `"d"` ✓ | `LPFLAGS` 4 or `"b"` ✓ | plus `CROSSOVER` 0: the 120 x 80 infeasible and unbounded LPs read **`failed`** (`LPSTATUS` 7 unsolved, `SOLVESTATUS` 2 failed); the suite's LPs pass |
| `copt_lp` | `LpMethod` -1: dual | none: `LpMethod` spans [-1, 6] without one | `LpMethod` 1 ✓ | 2 ✓ | 2 with `Crossover` 0: the suite's `max_infeasible` reads **`numerical_failure`** |
| `mosek_lp` | `MSK_OPTIMIZER_FREE`: interior point, then basis identification | `PRIMAL_SIMPLEX` ✓ (`NEW_PRIMAL_SIMPLEX` too) | `DUAL_SIMPLEX` ✓ (`NEW_DUAL_SIMPLEX` too) | `INTPNT` ✓ | `INTPNT` with `INTPNT_BASIS` = `NEVER` ✓ |

More facts behind the table:

- **Xpress `DEFAULTALG` is not a route.** 47.01 ignores it on a cold solve
  and honours it after `set_basis()`; 45.01 honours it in both cases.
  `LPFLAGS` and the `flags` argument work on both releases. `LPFLAGS` is a
  control, so it can be read back: a fresh model reads `LPFLAGS` 0 and
  `CROSSOVER` -1 on both releases.
- **HiGHS's "simplex, either side" is `solver=simplex` with
  `simplex_strategy` 0.** It chose the primal, dual and primal simplex on the
  three 120 x 80 LPs, and the suite's LPs pass. `simplex_strategy` defaults to
  1, the dual simplex, so `automatic` (`solver=choose`) and `dual_simplex`
  (`solver=simplex`) differ by `solver` alone, and the decoder must read both
  knobs.
- **HiGHS's knobs exist across its supported range.** The `simplex` and `ipm`
  values of `solver`, `simplex_strategy`, and `run_crossover` with on, off and
  choose are all in `HighsOptions.h` at v1.7.2, v1.10.0, v1.12.0 and v1.14.0
  (documented, read from git tags). `hipo` is refused by the local 1.15.1
  build.
- **Clp's entry points all exist in 1.17.4.** `Clp_dual`, `Clp_initialSolve`
  and the two barrier entries are in its `Clp_C_Interface.h` (documented):
  the C API has had the barrier since 2006.
  - The barrier misreports on the devel build only; no release is installed.
  - `Clp_initialSolve` passed everything here, but `dd9c448` dropped it for
    missing infeasibilities, and `Clp_initialPrimalSolve` misreports the
    suite's unbounded LPs as the barrier does.
- **The other native values are not candidates.**
  - CPLEX `LPMethod` 3 (network) and Xpress `"n"` fall back to the dual
    simplex on a non-network LP.
  - COPT `LpMethod` 3 (crossover only) throws "internal error" without a
    starting solution.
  - Gurobi `Method` 6 (PDHG) is refused by 11.0.3: "maximum is 5".
- **Every default crossover runs.** With the solver's default crossover, all
  seven interior points end with a basis that `get_basis()` reads.

### The first-order methods

The table leaves these out for these reasons, all measured:

- **Gurobi PDHG** (`Method` 6, 12.0.1) never proves infeasibility or
  unboundedness. The 120 x 80 infeasible and unbounded LPs, and both unbounded
  suite LPs, end at the time limit. Without a limit, the suite run did not
  return in 4 minutes.
- **HiGHS PDLP** never crosses over, even with `run_crossover=on`. It reports
  the infeasible LP as `infeasible_or_unbounded` and `max_bounded` as
  `unknown`.
- **COPT PDLP** (8.0) kills the process with SIGFPE on the suite's
  `max_infeasible` LP, every time, also when that LP runs alone in a process.

### After an interior point, and other interactions

- **Results.** After a crossover-less interior point, the solution, duals and
  reduced costs read fine on all seven (Clp included).
- **`get_basis()` after a crossover-less run.**
  - It throws a raw native message on Gurobi ("Unable to retrieve attribute
    'VBasis'"), CPLEX ("Error 1262: No basis exists.") and Xpress ("?429
    Error: No basis is available").
  - It throws the wrapper's "found no basis" on HiGHS, COPT and MOSEK.
  - On Clp it returns statuses.
- **`set_iteration_limit` bounds simplex iterations only.** On Gurobi, CPLEX
  and HiGHS, a limit of 1 leaves a crossover-less interior point `optimal`.
  With a crossover, Gurobi stops in the crossover (`iteration_limit` after
  8 barrier and 3 crossover iterations), while CPLEX and HiGHS ignore the
  limit.
- **`set_basis()` before an interior point.**
  - Every interior point runs cold after `set_basis()` of an optimal basis,
    without an error: barrier counts are the same as from scratch on Gurobi,
    CPLEX, HiGHS, Xpress, COPT, MOSEK and Clp.
  - `mosek_lp`'s one-shot simplex (`mosek_lp.hpp:129`) keys on `FREE`, so it
    keeps working under `automatic`.
- **`refine_lp_status()` inherits the algorithm.**
  - `gurobi_lp` re-solves under the same `Method` (`gurobi_lp.hpp:110-124`).
    The barrier calls the unbounded LP `infeasible_or_unbounded`.
    - With a crossover, the re-solve refines it to `unbounded`.
    - With `Crossover` 0, it stays `infeasible_or_unbounded`, and the call
      throws "Failed to refine LP status."
  - `cplex_lp`'s refiner calls `CPXprimopt` (`cplex_lp.hpp:147`), so it
    refined in every configuration.
- **Changing the knob after a solve** leaves the results readable on all
  nine. That matches the rule for parameters in
  `docs/solving/solutions.md:31`.
- **The deletion filter** (inferred, not run). Its trials call `solve()`, so
  under an interior point they run cold.
  - Under Xpress's crossover-less barrier, an infeasible trial reads `failed`.
  - `classify_deletion_trial` (`utility/iis_by_deletion.hpp:133`) calls
    `failed` inconclusive, so the filter would end `incomplete`.

## Proposed shape

```cpp
namespace lp_algorithm {
struct automatic {};  // the model's default: what solve() runs without a choice
struct simplex {};
struct primal_simplex : simplex {};
struct dual_simplex : simplex {};
struct interior_point {};  // crosses over to a basic solution
struct interior_point_without_crossover : interior_point {};
struct other {};  // read only: a native setting no tag names
}  // namespace lp_algorithm

template <typename T, typename A>
concept has_lp_algorithm = requires(T & model) {
    model.set_lp_algorithm(A{});
    model.get_lp_algorithm();
} && variant_containing_a<
         std::decay_t<decltype(std::declval<T &>().get_lp_algorithm())>, A>;
```

Tags each backend would list, under the recommendations below. "native" is
where the getter decodes from; `other` is listed wherever a native write can
leave a value no tag names.

| Model | Tags | Native state |
|---|---|---|
| `clp_lp` | `automatic` (primal), `primal_simplex`, `dual_simplex` | a member choosing the entry point |
| `glpk_lp` | `automatic` (primal), `primal_simplex`, `dual_simplex` | `model_params.meth` |
| `highs_lp` | `automatic`, `simplex`, `primal_simplex`, `dual_simplex`, `interior_point`, `other` | `solver`, `simplex_strategy` (0 for `simplex`), `run_crossover` |
| `gurobi_lp` | `automatic`, `primal_simplex`, `dual_simplex`, `interior_point`, `interior_point_without_crossover`, `other` | `Method`, `Crossover` |
| `cplex_lp` | the same | `LPMethod`, `SolutionType` |
| `xpress_lp` | the same | `LPFLAGS`, `CROSSOVER` |
| `copt_lp` | `automatic`, `dual_simplex`, `interior_point`, `interior_point_without_crossover`, `other` | `LpMethod`, `Crossover` |
| `mosek_lp` | `automatic`, `simplex`, `primal_simplex`, `dual_simplex`, `interior_point`, `interior_point_without_crossover`, `other` | `OPTIMIZER`, `INTPNT_BASIS` |
| `soplex_lp` | none | — |

Generic code asks `has_lp_algorithm<M, lp_algorithm::dual_simplex>`, the tag
every listing backend has. The docs say what a tag guarantees and what it
does not:

- A tag names the main algorithm. Solvers may finish with a few iterations
  of another method, for unscaled cleanup or crossover; HiGHS switches to a
  primal phase 2 for cleanup (`HApp.h`).
- After `interior_point_without_crossover`, `get_basis()` throws.

## Decisions to take

- **A1. The tag set.**
  - (a) The six tags above: `automatic`, `simplex` (as a parent, and
    selectable on HiGHS and MOSEK), the two simplex sides, and the two
    interior points.
  - (b) (a) plus `first_order` and `concurrent`.
  - (c) The simplex sides only.

  Recommended: (a). First-order methods hang on Gurobi, crash COPT and never
  cross over on HiGHS. `concurrent`, network and sifting are what `automatic`
  already reaches, or fall back to the dual simplex. (c) drops gecot's demand.
  `simplex` as a selectable tag maps to HiGHS's `solver=simplex` and MOSEK's
  `FREE_SIMPLEX` only; dropping it there costs little.
- **A2. Where `interior_point_without_crossover` is listed.**
  - (a) On every backend whose solver has it: HiGHS, Gurobi, CPLEX, Xpress,
    COPT and MOSEK, with the weak statuses documented.
  - (b) Only where the probes' statuses held: Gurobi, CPLEX and MOSEK.
  - (c) (b) plus Xpress and COPT, documenting `failed` and
    `numerical_failure` on infeasible or unbounded models. Not on HiGHS.

  Recommended: (c). On Xpress and COPT the weak statuses are the solver's own
  verdict on an unsolved problem. HiGHS reports a two-variable feasible LP as
  `unknown`, with dual values from an infeasible dual point, and those duals
  are what the demand is for. (a) would also turn HiGHS's `Highs_run` error
  into a status, which today throws.
- **A3. The getter.**
  - (a) `get_lp_algorithm()` decodes the native state into the backend's
    variant, `other` included (Gurobi `Method` 3 to 6, CPLEX `LPMethod`
    3/5/6, COPT `LpMethod` 3 to 6, Xpress `LPFLAGS` 8, HiGHS `pdlp`, `ipx`
    and the other `solver` values).
  - (b) Setter only, as the review proposed.
  - (c) A getter returning the last tag set, cached.

  Recommended: (a). Every parameter concept in `model_concepts.hpp` has a
  getter, `has_thread_limit` (`:866`) included, and the decoder is the
  encoder table read backwards. (c) lies after a native write. With (b),
  generic code cannot save and restore a choice; the deletion filter of A6
  needs to. On Clp and GLPK the member is the native state.
- **A4. `automatic`, and the order with question 1 of `proposals_review.md`.**
  - `automatic` restores what each model runs by default, as
    `set_thread_limit(0)` restores the default.
  - On Clp and GLPK that is the primal simplex: GLPK's own default, and the
    entry point `dd9c448` chose for Clp.
  - On CPLEX, the selector needs `solve()` to call `CPXlpopt`, or no tag
    reaches the barrier. So rule question 1 first.
    - With its (a), CPLEX's `automatic` is CPLEX's own choice (the dual
      simplex here).
    - With its (d), `automatic` keeps `CPXprimopt`, and only an explicit
      tag calls `CPXlpopt`.

  Recommended: question 1 (a), then `automatic` as described. Its known
  cost, from the review: `TimeLimitTest`'s size-800 LP solves faster under
  `CPXlpopt`, so the 0.2 s stop margin shrinks from about 3.4× to 1.45×.
- **A5. What `set_lp_algorithm` writes.**
  - (a) Every knob of the backend's tuple, at every call: Gurobi `Method` and
    `Crossover`; HiGHS `solver`, `simplex_strategy` and `run_crossover`;
    MOSEK `OPTIMIZER` and `INTPNT_BASIS`; CPLEX `LPMethod` and
    `SolutionType`; COPT `LpMethod` and `Crossover`; Xpress `LPFLAGS` and
    `CROSSOVER`. `automatic` writes each knob's default.
  - (b) Only the primary knob.

  Recommended: (a). Under (b), `interior_point` after
  `interior_point_without_crossover` would keep `Crossover` 0. The cost: a
  native crossover strategy (Gurobi `Crossover` 1 to 4) is reset by the
  setter, which the docs say.
- **A6. The deletion filter.**
  - (a) Trials run under the caller's choice, as documented: cold under an
    interior point, and `incomplete` under Xpress's crossover-less barrier.
  - (b) The guard switches to `dual_simplex` for its trials and restores the
    saved tag at exit. It does so only where the model lists `dual_simplex`
    and the saved value is not `other`.

  Recommended: (b). Trials are then warm and their statuses certified, and
  it needs A3 (a). With A3 (b), take (a).
- **A7. `set_iteration_limit`.**
  - (a) Document it as a limit on simplex iterations, crossover pivots
    included where the solver counts them (Gurobi).
  - (b) Also write the interior-point limits (`BarIterLimit`,
    `Barrier_Limits_Iteration`, `ipm_iteration_limit`).

  Recommended: (a). A barrier iteration is a different unit, a few dozen per
  solve against hundreds or thousands of pivots, so one number cannot bound
  both.
- **A8. Names.** `set_lp_algorithm` / `get_lp_algorithm`, namespace
  `lp_algorithm`, concept `has_lp_algorithm`; or the shorter `set_algorithm`.
  Recommended: the `lp_` names. A root and node algorithm on `*_milp` would
  be a separate concept if ever asked, since `*_milp` stays out under the
  2026-07-22 ruling. Those names leave that room.

## Decided without a ruling

- **`gurobi_lp::refine_lp_status()` re-solves with `Method` 1.** It already
  writes and restores `DualReductions`, and adds `Method` to its guard. This
  fixes the throw measured under `Crossover` 0, and matches `cplex_lp`'s
  refiner, which never depended on the algorithm.
- **`glpk_lp`'s status mapping** (`glpk_lp.hpp:97-128`) reads
  `glp_get_dual_stat`. `GLP_NOFEAS` with no primal verdict becomes
  `infeasible_or_unbounded`, an alternative added to `glpk_lp`'s variant.
  1.1.0 already adds status alternatives in a minor release.
- **`xpress_lp` writes the `LPFLAGS` control** and keeps calling
  `XPRSlpoptimize(prob, nullptr)`. The getter can then read it back, and a
  user's native `LPFLAGS` keeps working.
- **`clp_lp` keeps the barrier entry points out.** No native route reaches
  them, so nothing new to guard.
- **Uniform "no basis" messages.**
  - The three backends that throw a raw message after a crossover-less run
    throw the wrapper's "found no basis": Gurobi when `VBasis` is
    unavailable, CPLEX when `CPXsolninfo` reports a non-basic solution, and
    Xpress on error 429.
  - `docs/solving/basis.md:95` says "which only native settings can ask for".
    It changes to name `interior_point_without_crossover`.
  - `basis.md:124` and `docs/solvers/index.md:110` add that an explicit
    `interior_point` ignores the basis.
- **The X1 rule.** `docs/solving/solutions.md:31` lists the LP algorithm with
  the limits and tolerances, as settings that are not a change.

## Implementation plan and size

1. **Core.** Tags and concept in `model_concepts.hpp` under their own banner,
   about 40 lines.
2. **Shared suite** `test/test_suites/lp_algorithm.hpp`, about 200 lines. For
   each tag `M` lists:
   - the five `lp_status` LPs with the suite's refine step;
   - after `interior_point_without_crossover`, `get_basis()` throws
     `solver_error` while the solution reads;
   - `set_basis()` followed by `interior_point` solves to `optimal`;
   - `set` then `get` returns the tag;
   - results stay readable after the setter.
3. **Backends.** About 40 to 70 lines each, 450 in all:
   - encoder and decoder tables in the style of the basis encoders;
   - the `solve()` changes: Clp's entry point, GLPK's `meth`, CPLEX's
     `CPXlpopt`;
   - the Gurobi refiner and the GLPK mapping above.
4. **Native witness tests.** One per backend, about 30 lines each: the tag's
   native echo plus an iteration counter showing the algorithm ran, as the
   probes do. The shared suite cannot see which algorithm ran.
5. **Docs**, about 120 lines:
   - a section next to "Threads" in `docs/solving/status-and-limits.md`, with
     a support table;
   - `concepts.md`, the `coming-from.md` row, the features table row, the
     CHANGELOG, and the basis and solutions clauses above.

In total about 1060 lines.

## Corrections to the record

Today's probes contradict or sharpen these earlier claims:

- **`lp_algorithm.md`.**
  - It lists the Clp barrier entry points as "to verify". They exist in the
    C interface of every 1.17 release, and on devel they report `optimal` on
    unbounded LPs.
  - "Measured on 45.01 and 47.01: `DEFAULTALG` 3 or 4 still ran the
    concurrent solve": today 47.01 ignores `DEFAULTALG` only on a cold solve,
    and 45.01 honours it.
- **`proposals_review.md`, LP algorithm selector.**
  - "Crossover defaults differ between solvers (Gurobi and COPT cross over,
    Xpress and HiGHS do not)" holds for the first-order methods (HiGHS PDLP
    never crosses over). After a barrier, all seven cross over by default.
  - "It does not break `get_basis()`, which already throws after a
    crossover-less interior point": true of the six backends that would
    list the tag, but three of them throw a raw native message. Clp returns
    statuses, which is moot once its barrier is left out.
  - "CPLEX needs `SolutionType` 2 written by the setter": confirmed, as the
    route through `CPXlpopt`. `Barrier_Crossover` -1 is refused by 22.1.2.
- **`proposals_review.md`, docs section.** "The other seven honour their
  native knob" does not hold for SoPlex: its `ALGORITHM` showed no effect
  here.

## Not verified

- **Releases.**
  - Clp releases: only a devel build is installed, so the barrier misreport
    and `Clp_dual` were not run on 1.17.x.
  - GLPK before 5.0.
  - HiGHS before 1.15.1: knobs read from git, not run.
  - Gurobi 10.
  - COPT 7.2: license refused.
  - CPLEX 22.1.0 and 22.1.1.
  - MOSEK 11.1 and 11.2. Its `MSK_OPTIMIZER_*` values are positional
    (`CONIC` 0 to `PRIMAL_SIMPLEX` 8 in 11.0), so the encoder's constants are
    pinned on 11.0 only.
- **Instances.** Two random LPs and the five suite LPs. No performance claim
  rests on them; the iteration counts only witness which algorithm ran.
- **SoPlex's `ALGORITHM`.** No effect on these instances. It may act on
  warm starts or other shapes; not probed.
- **The deletion filter under an interior point.** Inferred from the measured
  Xpress status and the classifier, not run.
- **HiGHS's crossover-less mode with presolve off.** Not probed: it might
  avoid the `unknown` above, but A2 does not hinge on a non-default presolve.
