# Choosing a solver

Every backend exposes the same modeling interface, so the choice of solver is a two-line change. This page lists the available backends, how to switch between them, and what each one supports.

## The backends

Each backend lives in `mippp/solvers/<name>/all.hpp` and provides an api class plus one model class per problem kind. The last column is the range of solver releases each binding has been validated on, see [Solver version compatibility](compatibility.md).

| Solver | Header (`mippp/solvers/…`) | API class | Model classes | Validated releases |
| --- | --- | --- | --- | --- |
| [HiGHS](https://highs.dev) | `highs/all.hpp` | `highs_api` | `highs_lp`, `highs_milp`, `highs_qp` | 1.8.1 – 1.15 |
| [Gurobi](https://www.gurobi.com) | `gurobi/all.hpp` | `gurobi_api` | `gurobi_lp`, `gurobi_milp` | 10 – 13 |
| [CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio) | `cplex/all.hpp` | `cplex_api` | `cplex_lp`, `cplex_milp` | 22.1.0 – 22.2.0 |
| [FICO Xpress](https://www.fico.com/en/products/fico-xpress-optimization) | `xpress/all.hpp` | `xpress_api` | `xpress_lp`, `xpress_milp` | 45.1 – 47.1 (local runs) |
| [COPT](https://www.copt.de) | `copt/all.hpp` | `copt_api` | `copt_lp`, `copt_milp` | 7.2 – 8.0 (local runs) |
| [MOSEK](https://www.mosek.com) | `mosek/all.hpp` | `mosek_api` | `mosek_lp`, `mosek_milp` | 11.0 (local run) |
| [SCIP](https://scipopt.org) | `scip/all.hpp` | `scip_api` | `scip_milp` | 8.0.4 – 10.0.3 |
| [Cbc](https://github.com/coin-or/Cbc) *(experimental)* | `cbc/all.hpp` | `cbc_api` | `cbc_milp` | 2.10.9 – 2.10.13 |
| [Clp](https://github.com/coin-or/Clp) | `clp/all.hpp` | `clp_api` | `clp_lp` | 1.17.4 – 1.17.11 |
| [GLPK](https://www.gnu.org/software/glpk/) | `glpk/all.hpp` | `glpk_api` | `glpk_lp`, `glpk_milp` | 4.59 – 5.0 |
| [SoPlex](https://soplex.zib.de) | `soplex/all.hpp` | `soplex_api` | `soplex_lp` | 7.1.1 – 8.0.3 |

Notes:

- `*_lp` classes model continuous problems; `*_milp` classes add integer and binary variables (a `*_milp` model with only continuous variables is of course a valid LP). SCIP and Cbc expose only a MILP class; Clp and SoPlex only an LP class.
- Quadratic objectives (`*_qp`) are currently supported through HiGHS only.
- SoPlex's C interface gained `SoPlex_setRealParam`, the objective getter and the variable-bound and row-side entry points in 7.0, so on an older SoPlex `set_time_limit`, `get_objective`, `get_objective_coefficient`, the variable-bound setters and the row-side getters and setters throw although their concepts hold. It has no getter for real parameters: `get_time_limit()` returns the last limit SoPlex accepted. A value above 1e100 is stored as 1e100, and a negative one throws.
- `soplex_lp` reads column bounds from its own copy, since SoPlex misreads infinite bounds once a solve has scaled the LP. Change them through the model, never through `native_model()`: a solve that reloads the LP, as it does after a row's active side was freed, writes the copy back.
- Cbc is experimental. The C API of the released Cbc rebuilds the matrix on every row added, so building a model is slow, and nothing on the MIP++ side can avoid it; the Cbc figures of [Performance](../performance.md#limitations) come from an unreleased build. Its known limitations are documented, and no workaround beyond those already in place is planned: native parameters that do not reach MIP solves below Cbc 3.0, and rows without terms or with all-zero coefficients on a `devel` build, in the next two notes; wrong answers and endless branching on some rows over unbounded integer columns, see [Integrality proofs](#limitation-integrality-proofs); and a time limit that did not bound the root LP on a `devel` build, see [Deletion filter](#limitation-deletion-filter).
- Below Cbc 3.0, which covers every validated release, each MIP solve of `cbc_milp` runs on a private copy of the model, since Cbc carries a MIP solve's incumbent unchecked into the next one. A parameter set through `native_api()` on `native_model()` therefore does not reach MIP solves, and a native read of the solution after a MIP solve sees the unsolved model. Use the MIP++ setters and getters there; an LP solve, without integer columns, still runs on `native_model()`.
- A Cbc `devel` build, outside the validated range, drops a row without terms, so there `cbc_milp` throws `solver_error` when such a row is added. It keeps a row whose coefficients are all zero, but once the model has been solved, a re-solve can miss a change that makes that row's sides unsatisfiable and answer `optimal`.
- Xpress leaves a MILP search that stopped at a limit in its presolved form, so `xpress_milp::solve()` postsolves it before returning. The incumbent stays available, but the search tree is dropped: a second `solve()` starts the search over rather than resuming it.

## Switching backends

The examples follow one convention: the backend appears in exactly two places — the include and one alias.

```cpp
#include "mippp/solvers/highs/all.hpp"

using milp_type = highs_milp;
```

Change those to `gurobi`/`gurobi_milp` and recompile: the rest of the program is untouched. There is no linking step to adjust, because solver libraries are loaded at runtime.

### One implementation per solver, and the releases it is validated on

A binding is one implementation that adapts at runtime to a range of solver releases — probing for entry points that appeared or disappeared along the way — and lives in an implementation namespace, `mippp::gurobi::impl::v1` for Gurobi, which is what `all.hpp` aliases into `mippp` as `gurobi_api`, `gurobi_lp` and `gurobi_milp`. Two lists on the api class state what it drives:

- `gurobi_api::library_names` — the library names `gurobi_api::load()` searches for, newest first (`libgurobi130.so`, `libgurobi120.so`, …);
- `gurobi_api::validated_versions` — the release ranges that have passed the full test suite, half-open (`{{10}, {14}}` is every 10.x.y up to 13.x.y): a row of the [compatibility matrix](compatibility.md) or, for the solvers the matrix cannot obtain or license, a maintainer's run recorded in that page's notes.

A loaded release outside the validated ranges is used anyway, with a warning on `stderr` naming the ranges and what the library reported; `MIPPP_NO_VERSION_WARNING` silences it. To load one particular release, pin its file with `MIPPP_GUROBI_LIBRARY` or an explicit path.

A new solver release the implementation still drives is a matrix run and a bump of the validated range (plus a new library name where the solver ships one per release); one it cannot adapt to gets `impl::v2`, and the `mippp` aliases move to it while `impl::v1` stays available unchanged.

To choose the solver at *runtime* — for a `--solver` command-line flag, say — write the model-building code once as a template over the backend and dispatch on the flag; that pattern, and the capability checks that go with it, are the subject of [Writing solver-generic code](generic-code.md).

Only the solvers actually installed on the machine need to be present: a backend fails when its first model is constructed (with a descriptive exception), not at program startup.

## How solver libraries are found

Each backend has an api object, `gurobi_api` say, holding the C entry points the wrapper uses. A model's default constructor obtains it through `gurobi_api::load()`, which opens the solver's shared library through the platform loader (`dlopen` on Linux and macOS, `LoadLibrary` on Windows) and resolves those entry points — nothing is linked, and MIP++ needs no third-party loader library. Resolution order (first match wins):

1. an explicit path passed to `load` — `gurobi_milp model(gurobi_api::load("/opt/gurobi1201/linux64/lib/libgurobi120.so"));`
2. the `MIPPP_<SOLVER>_LIBRARY` environment variable, holding the full path of the exact file to load;
3. a search of the dynamic loader's directories (`LD_LIBRARY_PATH` on Linux, `DYLD_LIBRARY_PATH` on macOS, `PATH` on Windows, then the system library directories) for the conventional names, accepting version-suffixed sonames (`libhighs.so.1.10.0`) when the plain name is absent. The first directory holding any of the names wins, as it would for the loader; when a binding drives several releases (`libgurobi130.so`, `libgurobi120.so`, …) and one directory holds more than one of them, the newest is taken.

An api object is one loaded library file: `library_path()` returns it and `library_version()` the release it reports (empty for a library whose C API has no version call, such as SoPlex, or reporting something that is not a number, such as a Cbc `devel` build). `load` returns the same object whenever it resolves to the same file, and that object lives for the rest of the process: every model of a backend shares one table of entry points, and `model.native_api()` is a reference that cannot dangle. Two explicit paths naming two different files give two independent api objects, each with its own global state, so two versions of the same solver can serve two models in one process. The directory search of step 3 is memoized per solver for the life of the process, so default-constructing many models is cheap; explicit paths and the environment variable are never cached.

A library that exists but lacks the expected entry points (a same-named build without the C API, say) is rejected with the loader's own message rather than half-loaded, and the exception lists every candidate tried. Every loading failure also states this precedence and quotes the current value of `MIPPP_<SOLVER>_LIBRARY`, so an empty value or a stray space shows. See [Installation](../getting-started/installation.md#making-solver-libraries-discoverable) for per-solver environment setup.

## Feature support

Core LP/MILP modeling works on every backend. Optional capabilities — dual solutions, callbacks, MIP starts, SOS and indicator constraints, parameter control — vary; each is a [concept](../reference/concepts.md), so code that needs one states it and the compiler enforces it.

The matrices below record which features are implemented **and tested** per backend (generated from the test suites). Each column is one model class: the solver's `*_milp` class in the first matrix, its `*_lp` class in the second, and `highs_qp` in the HiGHS QP column.

### MILP models

![MILP feature support matrix](../assets/features_tables/milp_table_light.png#only-light)
![MILP feature support matrix](../assets/features_tables/milp_table_dark.png#only-dark)

### LP and QP models

![LP and QP feature support matrix](../assets/features_tables/lp_table_light.png#only-light)
![LP and QP feature support matrix](../assets/features_tables/lp_table_dark.png#only-dark)

Notable current limitations (see the
[roadmap](https://github.com/fhamonic/mippp#roadmap) for what's planned):

- **Callbacks** — candidate-solution callbacks are implemented and validated on Gurobi, CPLEX, COPT and Xpress; SCIP has none yet. Node-relaxation (user-cut) callbacks are specified but not yet implemented.
- **Solve status** — `get_status()` is part of `lp_model`, so every backend reports one, but the set of tags a backend can return varies (it is part of the model type). `refine_lp_status()` — resolving `infeasible_or_unbounded` into one of the two — exists only on `gurobi_lp` and `cplex_lp`, and `glpk_milp` reports an unbounded MIP as `infeasible_or_unbounded`. See [Status, limits and tolerances](../solving/status-and-limits.md).
- **Quadratic objectives** — HiGHS only. Quadratic constraints: none yet.
- **SOS constraints and LP basis warm starts** — specified as concepts, not yet implemented by any backend.
- **Ranged constraints** — `add_ranged_constraint` on Clp and Cbc only (`has_ranged_constraints`). On Cbc, Clp, GLPK, HiGHS, MOSEK, SCIP and SoPlex, `has_modifiable_constraint_bounds` makes any row ranged by giving it two distinct finite sides, and `has_readable_constraint_bounds` reads the sides back on every backend; see [Special constraints](../modeling/special-constraints.md#ranged-constraints).
- **Indicator constraints** — Gurobi and CPLEX only (`has_indicator_constraints`). The call returns no handle, so an indicator constraint cannot be read back or edited once added ([details](../modeling/special-constraints.md)).
- **Deletion filter** — `compute_iis_by_deletion`, one of the two ways to [diagnose an infeasible model](../solving/infeasibility.md), runs on every model class of Cbc, Clp, GLPK, HiGHS, MOSEK, SCIP and SoPlex. On `clp_lp`, `glpk_lp` and `glpk_milp`, which have no time limit, the deadline of `iis_limits` is checked between trials only, so one trial can overrun it. On `cbc_milp` the forwarded time limit does not bound the root LP, so one trial can overrun the deadline there too (measured on a Cbc `devel` build; 2.10 was not measured). HiGHS silently repairs, in the model itself, a variable bound or row side that crosses the other by less than its primal feasibility tolerance, and solves the model, while the filter, whose test for crossed sides is exact, reports such a pair as an IIS.
  { #limitation-deletion-filter }
- **Integrality proofs over unbounded integer columns** — Cbc 2.10.11 and 2.10.12 report `x0 + x1 == 1.5` over free integer columns `optimal`, with `x0 = 1.5`. A Cbc `devel` build finds no proof for that row and branches until a time limit stops it. Cbc 2.10.11, a Cbc `devel` build and `glpk_milp` all branch that way on `2 x0 + 2 x1 == 1`, and `glpk_milp`, which has no time limit, then never returns. The trials of `compute_iis_by_deletion` inherit these answers.
  { #limitation-integrality-proofs }
- **Deletion filter on SCIP binaries** — SCIP types a column `BINARY` when it comes from `add_binary_variable`, or from `add_integer_variable` with bounds that SCIP rounds into {0, 1}, such as [0, 1] or [0.25, 0.75]. It accepts relaxing a bound of such a column to `infinity()`, and the bound reads back as the infinity written, but the next `solve()` then throws `std::runtime_error("scip_milp: error in input data")`. `compute_iis_by_deletion` therefore throws on an infeasible `scip_milp` whenever a trial reaches such a bound, after restoring the model. A feasible model, and a run whose limits stop it before that trial, are unaffected. The same holds on SCIP 8.0.4, 9.2.1 and 10.0.2.
  { #limitation-scip-binaries }
- **Solver-specific parameters** — the uniform interface covers [limits and tolerances](../solving/status-and-limits.md); there is no uniform passthrough for solver-specific knobs such as Gurobi's `MIPFocus` or CPLEX's emphasis settings. The escape hatch is the `has_native_handles` concept, which every model class satisfies: `native_model()` returns the solver's own objects and `native_api()` the loaded `*_api` object, whose members are the solver's raw C functions, so the call is made directly:
  { #limitation-native-handles }

    ```cpp
    gurobi_milp model;
    auto [env, grb_model] = model.native_model();
    model.native_api().setintparam(env, "MIPFocus", 2);
    ```

    What comes back depends on the backend: a `(env, model)` pair where the solver has an environment — `std::pair<GRBenv *, GRBmodel *>` (Gurobi), `std::pair<CPXENVptr, CPXLPptr>` (CPLEX), `std::pair<copt_env *, copt_prob *>` (COPT), `std::pair<MSKenv_t, MSKtask_t>` (MOSEK) — and the single problem object otherwise: `XPRSprob` (Xpress), `SCIP *` (SCIP), `glp_prob *` (GLPK), `Clp_Simplex *` (Clp), `Cbc_Model *` (Cbc), `void *` (HiGHS, SoPlex). To address one variable or constraint through the raw API, `native_id(v)` and `native_id(c)` translate a MIP++ handle into what the solver calls it: the column or row index (1-based on GLPK), or the `SCIP_VAR *` / `SCIP_CONS *` on SCIP. Handles and native indices drift apart once variables have been removed on the backends that delete columns (HiGHS, Gurobi, CPLEX), which is what the translation is for.

    !!! warning "Modifying the model through the native handles invalidates MIP++ features"
        MIP++ does not see what is done through `native_model()` and `native_api()`. Adding, removing or reordering variables or constraints that way, or solving through the raw API, leaves MIP++'s bookkeeping out of date. Handles and `native_id()` may then designate the wrong variable or constraint. The variable and constraint counts can disagree with the solver on the backends that keep them on the MIP++ side, such as SCIP and Cbc. `get_status()` keeps reporting the last `solve()` made through MIP++. Any feature built on this bookkeeping, including those that read or restore the model on your behalf, then loses its guarantees. Keep native calls to parameters and read-only queries. On `cbc_milp` below Cbc 3.0, even those miss MIP solves, which run on a private copy (see the notes under [The backends](#the-backends)).

## Next

[Writing solver-generic code](generic-code.md) — one model builder, every backend, with capability differences resolved at compile time.
