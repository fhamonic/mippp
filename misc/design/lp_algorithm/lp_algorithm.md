# LP algorithm selection: notes for a future change

Notes gathered while designing LP basis support (2026-10-09), for a later
change adding an algorithm selector to the LP models. Nothing here is
designed or ruled; it records what is known, what the basis work fixed in
place, and the questions such a change must answer. Facts marked *measured*
come from the basis survey (`misc/design/basis/survey/<backend>/RECORD.md`);
*documented* from a vendor header or manual cited there; *to verify* were not
checked.

## Why

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

## A first shape

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

## What the basis change fixed in place

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
