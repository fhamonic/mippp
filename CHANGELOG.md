# Changelog

All notable changes to MIP++ are recorded in this file. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [1.1.0] - Unreleased

### Added

- **Infeasibility diagnosis.** Two ways to find an irreducible infeasible
  subsystem (IIS) of a model, described in
  [Diagnosing infeasible models](docs/solving/infeasibility.md):
  - `compute_iis()`, the solver's own routine, behind the concept `has_iis`,
    on `gurobi_lp`, `gurobi_milp`, `cplex_lp`, `cplex_milp`, `xpress_lp`,
    `xpress_milp`, `copt_lp`, `copt_milp`, and on `highs_lp` and `highs_qp`
    with HiGHS 1.14 or later;
  - `compute_iis_by_deletion(model, limits)`, the deletion filter of
    `mippp/utility/iis_by_deletion.hpp`, on every model class, with an
    overload `compute_iis_by_deletion(model, within, limits)` that narrows an
    earlier answer. Its engine, `deletion_filter`, also runs over an oracle of
    your own, see [The deletion filter](docs/algorithms/deletion-filter.md).

  Both answers satisfy `lp_iis`: a status per variable and per constraint,
  from the tags of `iis_status` (`iis_status::sides_of` reads which sides it
  holds), and an outcome from the tags of `iis_outcome`. The example
  `examples/infeasible_transportation` diagnoses an infeasible transportation
  plan.
- **LP bases.** `get_basis()` on `clp_lp`, `copt_lp`, `cplex_lp`, `glpk_lp`,
  `gurobi_lp`, `highs_lp`, `mosek_lp`, `soplex_lp` and `xpress_lp`, and
  `set_basis()` on all of them but `soplex_lp`, from which the next solve
  starts. A basis is a `basis_snapshot` whose statuses are the most specific
  `basis_status` tags the solver gives. See
  [Bases and warm starts](docs/solving/basis.md).
- **Branch-and-bound results.** `get_dual_bound()` and `get_node_count()`,
  behind `has_dual_bound` and `has_node_count`, on every `*_milp` model but
  `glpk_milp`. See
  [Dual bound and node count](docs/solving/solutions.md#dual-bound-and-node-count).
- **Row bounds.** `get_constraint_lower_bound(c)` and
  `get_constraint_upper_bound(c)` on every model class, so
  `has_readable_constraint_bounds`, which only `clp_lp` and `cbc_milp`
  satisfied, holds everywhere. `set_constraint_lower_bound(c, s)` and
  `set_constraint_upper_bound(c, s)`, behind the new
  `has_modifiable_constraint_bounds`, on every model class but those of
  Gurobi; on CPLEX and Xpress, a side that would cross the other throws
  `std::invalid_argument`.
- **Entity enumeration.** `variables()` and `constraints()`, behind
  `has_enumerable_variables` and `has_enumerable_constraints`, on every model
  class: a sized snapshot of the live handles, in increasing id order.
- **Status reset.** `reset_status()`, behind `has_status_reset`, on every
  model class: `get_status()` then reports `unknown` and the result getters
  throw until the next solve,
  while the model's data and the solver's warm start stay. See
  [Resetting the status](docs/solving/status-and-limits.md#resetting-the-status).
- **Time limits** on `clp_lp`, `glpk_lp`, `glpk_milp`, `copt_lp` and
  `scip_milp`.
- **Tolerances.** `set_optimality_tolerance`, the relative MIP gap, and
  `set_integrality_tolerance` on `highs_milp`; `set_integrality_tolerance` on
  `xpress_milp`. HiGHS has no integrality tolerance of its own, so the latter
  sets `mip_feasibility_tolerance`, which also bounds the row and bound
  violations its MIP solver accepts.
- **SoPlex.** `soplex_lp` reads and modifies variable bounds, and reads its
  objective.
- **Partial support of solver releases.** A backend may declare
  `supported_versions`, wider than its `validated_versions`. On a release in
  it, the model classes build and solve, and a member the release lacks throws
  `mippp::feature_unavailable_error`, a `solver_error`.
  `<solver>_api::support_of(api.library_version())` returns
  `release_support::validated`, `partial` or `untested`. Only HiGHS declares
  one, see Changed.
- **Column manager.** See Changed for the redesign, and the
  [column manager guide](docs/algorithms/column-generation.md#large-scale-pricing-the-column-manager).
  - `activate_columns(model, predicate, add, selector)` and
    `evict_columns(model, predicate, remove, selector)`, which return how many
    columns they moved and replace `manage_columns`;
  - a `Data` template parameter: data stored with each column, given at
    emplacement and passed in every entry;
  - `colgen::seed_hash`, the default `Hash`, which hashes ranges,
    `std::pair` and `std::tuple` seeds, and `colgen::hash_combine`;
  - `window_below<K>`, the mirror of `window_above<K>`, and
    `age_above_with_backoff{base, step}`, which damps eviction thrashing;
  - `emplace_master_column(model, seed, add)`, which adds an initial column
    through the add callback;
  - `stable_erase_if` on `unordered_dense_map`.

### Changed

- **LP basis concepts.** The draft of 1.0.0, which no backend implemented, is
  settled:
  - `has_modifiable_lp_basis` edits a basis through `set_status(e, tag)`
    alone, without `set_basic` and `set_nonbasic`, and requires the basis to
    be default-constructible, a default one being the slack basis;
  - the new tag `basis_status::superbasic` is the parent of `nonbasic_free`;
  - `has_lp_basis_warm_start` requires `set_basis` to accept any basis whose
    statuses list a basic tag and a nonbasic one.
- **Status mappings.** Code that tests exact tags may get other answers:
  - `scip_milp` reports `optimal` when it stops at the optimality tolerance,
    as every other backend does, where it reported `limit_reached`, and
    `limit_reached` at its stall-node limit, where it reported
    `numerical_failure`;
  - `clp_lp` reports `optimal_infeasible_unscaled`, where it reported
    `optimal`, when Clp's scaled problem is optimal but the point it returns
    violates the model (secondary status 2 or 4);
  - `glpk_lp` no longer reports a basic solution that merely violates a bound
    (`GLP_INFEAS`) as `infeasible`, and maps each `glp_simplex` error code to
    `limit_reached`, `numerical_failure` or `failed`;
  - GLPK models report crossed bounds or sides as `infeasible`, where they
    reported `failed`.
- **Status variants.** `clp_lp` lists `optimal_infeasible_unscaled`,
  `limit_reached` and `time_limit`; `glpk_lp` lists `limit_reached`,
  `time_limit`, `failed` and `numerical_failure`; `glpk_milp` lists
  `infeasible_or_unbounded`. A visitor written as one overload per
  alternative needs overloads for them, or for a tag they derive from.
  `highs_lp` no longer lists `solution_limit`, which none of its solves
  reported, so `is` and `is_a` of `status::solution_limit` on a `highs_lp`
  status no longer compile.
- **A `solve()` that throws** leaves `get_status()` at `unknown`, where it kept
  the previous solve's status.
- **Results after a change (breaking).** `get_solution_value()`,
  `get_solution()`, `get_dual_solution()`, `get_reduced_costs()`,
  `get_basis()`, `get_dual_bound()` and `get_node_count()` read the last
  `solve()` until the model changes, and throw the new
  `mippp::solve_required_error`, a `std::logic_error`, before the first
  `solve()` and after any change until the next one. A change is any call that
  writes the model: variables and constraints added or removed, the objective,
  the sense, bounds, sides, types, names, `set_basis` and `add_mip_start`.
  Limit, tolerance, verbosity and callback setters are not changes.
  `get_status()` reports `unknown` over the same span, where it kept the last
  solve's status, `set_basis` included. `get_basis()` no longer reads back a
  basis written with `set_basis` before the next solve, as it did on
  `cplex_lp`, `copt_lp`, `glpk_lp` and `mosek_lp`. What these getters return is
  unchanged: a snapshot that keeps the values of its solve. The backends
  disagreed: after a change, Gurobi, CPLEX and COPT threw their own errors,
  HiGHS, Xpress and MOSEK returned zeros or wrong values, and SCIP crashed.
  Fetch results right after `solve()`, see
  [When results can be read](docs/solving/solutions.md#when-results-can-be-read).
- **Time limits.** `get_time_limit()` never returns a negative value, and
  `+inf` or `duration<double>::max()` lifts a limit, on every backend.
  HiGHS's limit bounds each solve, where it bounded the time of all the
  model's solves so far.
- **`soplex_lp` minimizes by default**, like every other backend, where it
  maximized an objective set without a sense.
- **`glpk_milp` rounds integral sides** for GLPK: the bounds of integer columns
  and the sides of rows whose columns are all integer with integral
  coefficients. GLPK refused an integer column with a fractional bound, which
  reported `failed`. The model keeps the sides as set.
- **Xpress.** `xpress_milp::solve()` and `xpress_lp::solve()` postsolve a solve
  stopped by a limit, so the columns and rows stay those of the model. On
  `xpress_milp`, a second `solve()` then restarts the search instead of
  resuming it.
- **Cbc below 3.0.** Each MIP solve runs on a fresh copy of the model, so a
  parameter set natively on `native_model()` no longer reaches MIP solves, and
  a native read of their solution sees the unsolved model. The time limit
  counts wall-clock time, as Cbc's master branch does by default, where it
  counted the CPU time of the whole process. It is passed with a 50 ms margin, because Cbc
  2.10 also takes its preprocessing time off the limit, and `get_time_limit()`
  reads it back as set.
- **Cbc `devel` builds.** `cbc_milp` throws `solver_error` on a row without
  terms, which these builds drop silently.
- **Ranged rows.** `get_constraint_sense`, `get_constraint_rhs`,
  `get_constraint`, `set_constraint_sense` and `set_constraint_rhs` throw on a
  ranged row on HiGHS and CPLEX, where CPLEX read its sense as `>=`.
  `cbc_milp::get_constraint_rhs` throws on one, as `clp_lp`'s does, where it
  returned the upper side. The row-bound getters read such rows.
- **Supported solver releases.**
  - HiGHS is validated for 1.14 and 1.15, where it was for 1.8.1 to 1.15,
    and supported partially from 1.7.2. On a partially supported release,
    `compute_iis()` throws `mippp::feature_unavailable_error`, and so does
    `add_mip_start()` on a release without `Highs_setSparseSolution`.
  - SCIP is validated up to 10.1.0, where it was up to 10.0.3.
  - SoPlex is validated up to 8.1.0, where it was up to 8.0.3.
  - A partially supported release loads without the version warning.
- **Native entry points.** Every backend binds new native functions for these
  features. Those of SoPlex, and `Highs_getIis` and `Highs_setSparseSolution`,
  are optional. The others are required, so a solver library that lacks one no
  longer loads.
- **Column manager (breaking).** The redesign fixes `manage_columns`, which
  evicted before activating and so, at convergence, modified the master after
  its last solve: reading the solution then threw on Gurobi and CPLEX and
  returned 0 on HiGHS.
  - Every callback receives an entry, `{seed, state}` or, with a `Data`,
    `{seed, data, state}`, instead of a `std::pair<const Seed &, const State &>`
    (predicates, remove callback), a seed (add callback, `update_pool_columns`)
    or a seed and a variable (`update_master_columns`).
  - Predicates and selectors are separate arguments. `none`, `evict_never`,
    `all<P>`, and the `at_most_k*` strategies that bundled a predicate are
    gone; `at_most_k`, `at_most_k_random` and `at_most_k_best` are selectors.
  - `evict_window_above<K>` and `evict_unattractive_aged`, which assumed a
    minimizing master, are gone.
  - The template parameters are `<Model, Seed, PoolProperties,
    MasterProperties, Data, Hash, KeyEqual>`: `Hash` moved from fifth to sixth.
  - `purge_pool` keeps the order of the columns it spares.

  | 1.0 | 1.1 |
  |---|---|
  | `manage_columns(m, all<P>{...}, add)` | `activate_columns(m, P{...}, add)` |
  | `manage_columns(m, A, E, add, rm)` | `activate_columns(m, a, add)`, then `evict_columns(m, e, rm)` |
  | `at_most_k_best{k, cmp, P{}}` | `activate_columns(m, P{}, add, at_most_k_best{k, cmp})` |
  | `evict_window_above<K>{t}` | `window_above<K>{t}` |
  | `evict_unattractive_aged{k, t}` | `conjunction<above<age>, above<reduced_cost>>{k, t}` |
  | `add(m, seed)` | `add(m, entry)`, with `entry.seed` |
  | `update_pool_columns([](seed) {...})` | `update_pool_columns([](auto && entry) {...})` |
  | `update_master_columns([](seed, var) {...})` | `update_master_columns([](auto && entry) {...})`, with `entry.state.var` |
  | `entry.first`, `entry.second`, `std::get<1>(entry)` | `entry.seed`, `entry.state` |
  | `column_manager<M, S, PP, MP, Hash>` | `column_manager<M, S, PP, MP, no_data, Hash>` |

### Fixed

- `scip_milp` misread every status of SCIP 10, which renumbered `SCIP_STATUS`:
  an optimum read `interrupted`, an infeasible model `node_limit`.
- `xpress_milp::set_optimality_tolerance` set Xpress's integrality tolerance
  (`MIPTOL`) instead of the relative gap (`MIPRELSTOP`).
- After a limit stopped an Xpress MILP search, `get_solution()` overflowed its
  buffer, and the column getters and setters addressed the presolved columns.
  An LP stopped by a limit on Xpress 45.01 kept its rows and columns presolved
  in the same way.
- The stopped statuses of COPT and SCIP models reported no solution even when
  the solver held one.
- A moved `clp_lp`, `cbc_milp`, `scip_milp` or `soplex_lp` reported `unknown`
  instead of its status.
- After `remove_variable`, `set_continuous`, `set_integer`, `set_binary` and
  indicator constraints on `gurobi_milp`, and indicator constraints on
  `cplex_milp`, addressed the next column, or none.
- The destructors of the COPT, CPLEX, Gurobi, Xpress and MOSEK models could
  throw, which terminated the program, and a MOSEK model whose constructor
  threw leaked its environment and task.
- MOSEK raised `license_error` only for an expired license. Every license
  error code raises it now.
- On a Cbc `devel` build, `cbc_milp` aborted the process when the relaxation of
  a MIP was infeasible, unbounded or abandoned.
- A Cbc MIP stopped by a limit before finding an integer point reported an
  earlier solve's incumbent as available.
- After a solve, the variable-bound getters of `scip_milp` returned the
  presolved domain. After a solve that threw, its setters changed only the
  transformed problem and the next solve resumed the stale search.
- `soplex_lp::add_column` dropped the entries of a column below its first
  rows.
- `soplex_lp` read a bounded LP as unbounded when a column side was an IEEE
  infinity rather than SoPlex's own, and reported `optimal` at a point outside
  bounds crossed after a solve.
- GLPK took an IEEE infinity on the side it cannot free as a real side, so
  `glpk_lp` answered `optimal` and the presolver of `glpk_milp` failed an
  assertion, and it typed a column added with bounds at `infinity()` as
  double-bounded.
- A MOSEK column side set to `infinity()` stayed a finite bound under a raised
  `MSK_DPAR_DATA_TOL_BOUND_INF`.
- `refine_lp_status()` on `gurobi_lp` and `cplex_lp` left solver parameters
  changed when it threw.
- The loader's error told every platform to set `LD_LIBRARY_PATH`. It names
  `PATH` on Windows and `DYLD_LIBRARY_PATH` on macOS.
- On `cplex_lp`, `cplex_milp`, `gurobi_lp`, `gurobi_milp`, `highs_lp`,
  `highs_milp` and `highs_qp`, a solution or reduced-cost snapshot taken
  before `remove_variable` read the values of other variables afterwards.

## [1.0.0] - 2026-09-23

First stable release.

[1.1.0]: https://github.com/fhamonic/mippp/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/fhamonic/mippp/releases/tag/v1.0.0
