# LP algorithm selection: design decisions

Design note for the LP algorithm selector (`set_lp_algorithm`,
`get_lp_algorithm`, `has_lp_algorithm`). It records the maintainer's rulings
on the questions of [lp_algorithm_plan.md](lp_algorithm_plan.md), whose
entries keep the options and evidence behind each letter, the review of
2026-11-11 and what the implementation settled. Probes, outputs and the fuzz
survey are in [survey/](survey/RECORD.md). It is not user documentation. The
notes of 2026-10-09 that started the work are kept at the end.

## Rulings of 2026-11-11

- **A1 (a). Tags.** `automatic`; `simplex` with `primal_simplex` and
  `dual_simplex` under it; `interior_point` with
  `interior_point_without_crossover` under it; `other`, read only. `simplex`
  is settable on HiGHS and MOSEK only. First-order, concurrent, network and
  sifting methods stay native.
- **A2 (c), amended after the fuzz survey below.** The crossover-less tag is
  listed where the solver's verdicts held.
- **A3 (a). The getter decodes the native state**, strictly (see the review).
- **A4, with question 1 (a) of `proposals_review.md`.** `automatic` is each
  model's default. `cplex_lp::solve()` calls `CPXlpopt`, so CPLEX's default
  moves from the primal simplex to its own choice; `refine_lp_status()` stays
  on `CPXprimopt`.
- **A5 (a).** The setter writes every knob of the backend at every call.
- **A6, narrowed (b).** The deletion filter switches to `dual_simplex` for
  its trials only when the saved algorithm is an interior point, and restores
  it. A simplex, `automatic` and `other` are left alone, so `mosek_lp` under
  `automatic` runs its trials as before.
- **A7 (a).** `set_iteration_limit` is documented as a limit on simplex
  iterations.
- **A8.** `set_lp_algorithm`, `get_lp_algorithm`, `namespace lp_algorithm`,
  `has_lp_algorithm`.
- **Decided without a ruling:** every item of the plan, and a new
  `glpk_lp::refine_lp_status()`, accepted with the plan.

## Review of 2026-11-11: what changed from the plan

- **The setter takes the exact tag.** With one overload per tag taken by
  value, `interior_point_without_crossover{}` converts to `interior_point`
  and runs with a crossover on a model that lists the base only, which is
  the fallback the plan forbids. Each backend has one template constrained
  by `detail::settable_lp_algorithm`, and the concept tests the exact
  alternative (`variant_with_alternative`). Every backend's test file pins
  its tag list with positive and negative `static_assert`s.
- **Strict decode, through the encoder.** `detail::decode_lp_algorithm`
  returns the tag whose encoded knobs equal the native ones, and `other`
  otherwise. Setting the algorithm just read then changes nothing, which the
  deletion filter's restore needs: a decoder naming `interior_point` for
  Gurobi `Method` 2 with `Crossover` 3 would have reset the strategy.
- **One-argument concept and deduced alias.**
  `has_lp_algorithm<T, A = lp_algorithm::automatic>` and
  `model_lp_algorithm_t<M>`.
- **`glpk_lp` holds the tag itself.** `meth` reads `GLP_PRIMAL` under
  `automatic` and `primal_simplex` alike.
- **What the suite adds to the plan's:** every case of `LpStatusTest`
  (built by `lp_status_cases`, shared with that suite); an LP that is primal
  and dual infeasible; re-solves after a new row, a new column and a tighter
  bound; the basis after an earlier simplex solve and a crossover-less one;
  native `compute_iis()` and the deletion filter under each tag; the fuzzer
  of `LpFuzzyTest` under each tag.

## Rulings of 2026-11-11, after the fuzz survey

The fuzzer ran 1500 seeds under every tag of every backend
([survey/RECORD.md](survey/RECORD.md)). Four combinations failed, and each
was reproduced by a standalone probe:

- **No selector on `clp_lp`.** `Clp_dual` (devel build) reported a feasible
  LP `infeasible` (seed 1124: two free columns, `x0 + 5 x1 >= 4`,
  `2 x1 == -2`, also on a fresh model) and an unbounded one `optimal` at
  -1.1e11 (seed 409). Without `dual_simplex` the model would list
  `automatic` and `primal_simplex` only, which are the same algorithm.
- **No `interior_point_without_crossover` on `copt_lp`.** COPT 8.0 reported
  LPs without an optimum `optimal`, native `LpStatus` 1 with an objective
  value, in 26 runs.
- **`primal_simplex` stays on `gurobi_lp` and `highs_lp`, documented.**
  Gurobi 12.0.1 reported a bounded LP without rows `unbounded` on a re-solve
  (seed 98; 11.0.3 does the same, 13.0.2 does not). HiGHS 1.15.1 ended
  `unknown` on a re-solve (seed 1090). One run each.
- **The suite's fuzz runs under a fixed seed**, 1, which `MIPPP_FUZZ_SEED`
  overrides.

The selector is therefore on seven models:

| Model | Tags | Native state |
|---|---|---|
| `glpk_lp` | `automatic` (primal), `primal_simplex`, `dual_simplex` | a member, and `model_params.meth` |
| `highs_lp` | `automatic`, `simplex`, `primal_simplex`, `dual_simplex`, `interior_point`, `other` | `solver`, `simplex_strategy`, `run_crossover` |
| `gurobi_lp` | `automatic`, `primal_simplex`, `dual_simplex`, `interior_point`, `interior_point_without_crossover`, `other` | `Method`, `Crossover` |
| `cplex_lp` | the same | `LPMethod`, `SolutionType` |
| `xpress_lp` | the same | `LPFLAGS`, `CROSSOVER` |
| `copt_lp` | `automatic`, `dual_simplex`, `interior_point`, `other` | `LpMethod`, `Crossover` |
| `mosek_lp` | `automatic`, `simplex`, `primal_simplex`, `dual_simplex`, `interior_point`, `interior_point_without_crossover`, `other` | `OPTIMIZER`, `INTPNT_BASIS` |

## Settled during the implementation

- **The encoder is one function template per backend**, a chain of
  `if constexpr(std::same_as<A, tag>)` ending on a dependent
  `static_assert`: a listed tag without a row fails to compile, where
  `detail::overloaded` would have fallen back to a base tag's row.
- **`mosek_lp::solve()` deletes the basic solution before a solve that runs
  no simplex.** After a simplex solve, a new row and a crossover-less solve,
  `get_basis()` returned the first solve's basis: `solutiondef(MSK_SOL_BAS)`
  stays true. Gurobi, CPLEX and Xpress throw there without a change.
- **`get_basis()` messages.** Gurobi's `GRB_ERROR_DATA_NOT_AVAILABLE` on
  `VBasis`, CPLEX's error 1262 and Xpress's error 429 (read from
  `XPRS_ERRORCODE`) throw the wrapper's "found no basis".
- **`gurobi_lp::refine_lp_status()`** re-solves with `Method` 1 and restores
  `Method` with `DualReductions`, the guard armed before the writes.
- **`glpk_lp`.** With a return code of 0, a dual status `GLP_NOFEAS` on a
  basis that is not primal feasible maps to `infeasible_or_unbounded`; so
  does `GLP_ENODFS`, as on `glpk_milp`. `refine_lp_status()` runs
  `glp_simplex` under `GLP_PRIMAL` from that basis. `max x` under
  `x - y >= 1` over `x, y >= 0` is the two-column LP that ends there.
- **Native `compute_iis()` holds under every listed tag** on HiGHS, Gurobi,
  CPLEX, Xpress and COPT, on the suite's four-row conflict, and leaves the
  algorithm as set.
- **The deletion filter needed the switch.** Before it, Gurobi's
  crossover-less barrier ended the four-row conflict on
  `inconclusive_trial`.
- **Witnesses of which algorithm ran**, in each backend's test file:
  iteration counters on Gurobi, COPT, MOSEK and HiGHS; `CPXgetmethod`,
  `CPXgetitcnt`, `CPXgetbaritcnt` and `CPXsolninfo` on CPLEX, resolved from
  the library by the test, so that the api requires no new symbol; the
  `ALGORITHM` attribute on Xpress. HiGHS and GLPK name no simplex side: on
  `build_dense_covering`, whose slack basis is dual feasible, their dual
  simplex takes fewer iterations than their primal one, and on GLPK the
  two-column LP above ends on two different statuses.
- **Threads of `cplex_lp`** (16 hardware threads, the 900-column dense LP):
  `automatic` and both simplex sides used 1.00 core at any limit; the
  barrier, with or without crossover, 8 to 10 cores by default and 1.00
  under a limit of 1.
- **`TimeLimitTest` on `cplex_lp`** still stops: the 800-column LP ends at
  0.21 s under the 0.2 s limit.
- **Released GLPK and HiGHS.** `misc/tools/compat_matrix.py`, run on the
  new binary without publishing its table: GLPK 4.59 to 5.0 pass every
  test, the selector's and the refiner's included; no test of
  `LpAlgorithmTest` fails on HiGHS 1.7.2 to 1.16.0, and
  `compute_iis_under_each_algorithm` skips below 1.14 as the other IIS
  tests do.

## Open questions

Found on the way, in models left at their default algorithm, and not
changed:

- **`unbounded` on an LP that is primal and dual infeasible.** `gurobi_lp`
  reports it after `refine_lp_status()` under every tag, and `mosek_lp`
  under every tag but the crossover-less one
  ([out_both_infeasible.txt](survey/out_both_infeasible.txt)). Gurobi
  documents `UNBOUNDED` as "an unbounded ray exists, which says nothing
  about feasibility", and `mosek_lp` maps `MSK_PRO_STA_DUAL_INFEAS` to
  `unbounded`. The docs call a status a proof. The suite only requires
  such an LP not to read `optimal`.
- **`clp_lp` reports a feasible unbounded LP `infeasible`** when the
  unbounded columns are in no row (the LP of
  [probe_fuzz_copt.cpp](survey/probe_fuzz_copt.cpp), devel build). The fuzz
  suites compare optimality only, so they do not see it.

## Not verified

- COPT 7.2 (the licence is refused here), MOSEK 11.1 to 11.3 at run time
  (the enumerations of the 11.1.11 and 11.2.8 wheels were read as
  identical to 11.0's), CPLEX 22.1.0, 22.1.1
  and 22.2, Gurobi 10, Xpress 45 and 46 under the final suite.
- MSVC, clang and libc++ builds: the suite's fixed seed draws another
  sequence there.
- HiGHS with HiPO, which `solver=ipm` selects when the build has it. The
  upstream 1.15.1 static release that CI links did not link here: it needs
  glibc 2.38.
- The published compatibility table was not regenerated. On the current
  tree two rows would change for reasons older than the selector: HiGHS
  1.7.2 reads as lacking a core test
  (`HiGHS_milp/LpModelTest.model_changes_invalidate_results` needs
  `add_mip_start`), and HiGHS 1.16.0, outside the claim, fails
  `LpModelTest.solve_empty_*`.

## Notes of 2026-10-09

The notes that started the work, written while designing LP basis support
and superseded by the plan and the rulings above.

Notes gathered while designing LP basis support (2026-10-09), for a later
change adding an algorithm selector to the LP models. Nothing here is
designed or ruled; it records what is known, what the basis work fixed in
place, and the questions such a change must answer. Facts marked *measured*
come from the basis survey (`misc/design/basis/survey/<backend>/RECORD.md`);
*documented* from a vendor header or manual cited there; *to verify* were not
checked.

### Why

The maintainer asked, while ruling B8 of the basis plan, whether an
`get_algorithm` / `set_algorithm` pair over a variant of tags (`primal_simplex`
and `dual_simplex` under `simplex`) would be better than letting `set_basis`
change MOSEK's optimizer. The survey showed that the LP models do not all
run a simplex by default, and that three wrappers hard-code an algorithm the
user never chose:

| Model | What `solve()` runs today | Knob |
|---|---|---|
| `clp_lp` | primal simplex, hard-coded: `Clp_primal(model, 0)` | entry points: `Clp_primal`, `Clp_dual`; `Clp_initialSolve` and barrier entry points to verify |
| `glpk_lp` | primal simplex, hard-coded: `glp_smcp.meth = GLP_PRIMAL` | `meth` = `GLP_PRIMAL`, `GLP_DUAL`, `GLP_DUALP`; `glp_interior` is a separate routine with its own status codes |
| `highs_lp` | HiGHS's choice (`solver = "choose"`): dual simplex for an LP | option `solver` (`simplex`, `ipm`, `pdlp`, later releases more), `simplex_strategy` (dual 1, primal 4), `run_crossover` |
| `soplex_lp` | dual simplex (`ALGORITHM` 1) | int parameter `ALGORITHM` (primal, dual) |
| `gurobi_lp` | Gurobi's choice (`Method` -1): concurrent for an LP | `Method` (0 primal, 1 dual, 2 barrier, 3 and 4 concurrent, 5 deterministic concurrent simplex, 6 PDHG from 12.0), `Crossover` |
| `cplex_lp` | primal simplex, hard-coded: `CPXprimopt`, whatever `LPMethod` says | `CPXlpopt` with `CPXPARAM_LPMethod` (0 auto, 1 primal, 2 dual, 4 barrier, 6 concurrent, others to verify), or `CPXprimopt` / `CPXdualopt` / `CPXbaropt`; `CPXPARAM_Barrier_Crossover` |
| `xpress_lp` | Xpress's choice: `XPRSlpoptimize(prob, nullptr)`, concurrent for a standalone LP, dual simplex once a basis is held | the `flags` argument of `XPRSlpoptimize` (primal, dual, barrier; measured on 45.01 and 47.01: `DEFAULTALG` 3 or 4 still ran the concurrent solve), `CROSSOVER` |
| `copt_lp` | COPT's choice (`LpMethod` -1): dual simplex | `LpMethod` (1 dual, 2 barrier, 3 crossover, 4 concurrent, 5 automatic, 6 PDLP), `Crossover` |
| `mosek_lp` | MOSEK's choice (`MSK_OPTIMIZER_FREE`): interior point, with basis identification | `MSK_IPAR_OPTIMIZER` (`FREE`, `INTPNT`, `FREE_SIMPLEX`, `PRIMAL_SIMPLEX`, `DUAL_SIMPLEX`), `MSK_IPAR_INTPNT_BASIS` |

`cplex_lp`'s `CPXprimopt` is a performance choice nobody made explicitly:
`CPXlpopt` would let CPLEX pick, usually the dual simplex (measured with
`LPMETHOD` 0 and 6).

### A first shape

A variant over a tag hierarchy, like `status`, `basis_status` and
`iis_status`, each backend listing the tags it supports:

```
namespace lp_algorithm {
struct automatic {};                  // the solver's own choice
struct simplex {};                    // the solver picks primal or dual
struct primal_simplex : simplex {};
struct dual_simplex : simplex {};
struct interior_point {};             // with a crossover: ends with a basis
struct interior_point_without_crossover : interior_point {};
struct first_order {};                // PDLP, PDHG
struct concurrent {};
}
```

Points to settle:

- **Crossover decides whether `get_basis()` answers.** An interior point
  without crossover, a first-order method without crossover and a barrier
  stopped before its crossover leave no basis on HiGHS, Gurobi, CPLEX,
  Xpress, COPT and MOSEK (measured). So crossover is a refinement of the
  interior-point tag, not an independent switch, and the docs of
  `get_basis()` name the tags after which it throws.
- **The default tag.** `automatic` is what every model starts from and what
  `get_algorithm()` reports until `set_algorithm()` is called. Whether
  `get_algorithm()` should instead report what the last solve actually ran
  (Gurobi's `ConcurrentWinMethod`, CPLEX's `CPXgetmethod`) is a separate
  read, maybe a separate member.
- **Hard-coded wrappers change behaviour.** Under `automatic`, `clp_lp`,
  `glpk_lp` and `cplex_lp` would hand the choice to the solver, which changes
  their default algorithm, or `automatic` keeps today's primal simplex on
  them. Either way it is a visible change of default for those three.
- **`*_milp` classes.** Gurobi's `Method`, CPLEX's `LPMethod` and others
  also pick the algorithm of a MILP's root relaxation. The ruling of
  2026-07-22 keeps LP-only features off `*_milp`; whether a root-relaxation
  algorithm counts as one is a question for the maintainer.
- **Iteration limits.** `set_iteration_limit` writes simplex limits today
  (HiGHS `simplex_iteration_limit`, Gurobi `IterationLimit`, CPLEX
  `Simplex_Limits_Iterations`); an interior point has its own limit on every
  solver (Gurobi `BarIterLimit`, COPT `BarIterLimit`, MOSEK
  `INTPNT_MAX_ITERATIONS`). Which one `set_iteration_limit` bounds under a
  non-simplex algorithm must be decided.
- **Status mapping.** Some routines have their own status codes
  (`glp_interior`; COPT's `LpStatus` 11 from `BarIterLimit`, which
  `copt_lp` maps to `unknown` today).
- **Survey first.** As for the basis: one probe per backend of every tag it
  would list, the knob that selects it, whether the solve then ends with a
  basis, and the release range where each knob exists (HiGHS `solver` values
  grew across releases; Gurobi `Method` 6 needs 12.0, measured rejected on
  11.0.3).

### What the basis change fixed in place

Ruling B8 (a) of 2026-10-09 (`misc/design/basis/basis.md`): on `mosek_lp`,
`set_basis()` makes the next `solve()` run a simplex optimizer, then restores
the optimizer setting. It is defined as "when the optimizer is MOSEK's
automatic choice, the solve after `set_basis()` runs a simplex; an explicit
choice is respected". The one-shot already switches only when
`MSK_IPAR_OPTIMIZER` reads `MSK_OPTIMIZER_FREE` (pinned by
`mosek_lp_test.set_basis_keeps_an_optimizer_chosen_natively`), so an optimizer
set through `native_api()` is today's explicit choice. A selector that writes
`MSK_IPAR_OPTIMIZER` needs no change to the one-shot; what it must settle is
whether `automatic` writes `MSK_OPTIMIZER_FREE` at each solve, and so
overrides a native setting. An explicit `interior_point` keeps ignoring the
basis, as the docs of `set_basis()` must then say.
