# Choosing a solver

Every backend exposes the same modeling interface, so the choice of solver is a two-line change. This page lists the available backends, how to switch between them, and what each one supports.

## The backends

Each backend lives in `mippp/solvers/<name>/all.hpp` and provides an api class plus one model class per problem kind. The version in the alias points at the solver release the binding targets.

| Solver | Header (`mippp/solvers/…`) | API class | Model classes | Targets |
| --- | --- | --- | --- | --- |
| [HiGHS](https://highs.dev) | `highs/all.hpp` | `highs_api` | `highs_lp`, `highs_milp`, `highs_qp` | v1.10 |
| [Gurobi](https://www.gurobi.com) | `gurobi/all.hpp` | `gurobi_api` | `gurobi_lp`, `gurobi_milp` | v12.0 |
| [CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio) | `cplex/all.hpp` | `cplex_api` | `cplex_lp`, `cplex_milp` | v22.1.2 |
| [FICO Xpress](https://www.fico.com/en/products/fico-xpress-optimization) | `xpress/all.hpp` | `xpress_api` | `xpress_lp`, `xpress_milp` | v45.1 |
| [COPT](https://www.copt.de) | `copt/all.hpp` | `copt_api` | `copt_lp`, `copt_milp` | v7.2 |
| [MOSEK](https://www.mosek.com) | `mosek/all.hpp` | `mosek_api` | `mosek_lp`, `mosek_milp` | v11 |
| [SCIP](https://scipopt.org) | `scip/all.hpp` | `scip_api` | `scip_milp` | v8 |
| [Cbc](https://github.com/coin-or/Cbc) | `cbc/all.hpp` | `cbc_api` | `cbc_milp` | v2.10.12 |
| [Clp](https://github.com/coin-or/Clp) | `clp/all.hpp` | `clp_api` | `clp_lp` | v1.17 |
| [GLPK](https://www.gnu.org/software/glpk/) | `glpk/all.hpp` | `glpk_api` | `glpk_lp`, `glpk_milp` | v5 |
| [SoPlex](https://soplex.zib.de) | `soplex/all.hpp` | `soplex_api` | `soplex_lp` | v6 |

Notes:

- `*_lp` classes model continuous problems; `*_milp` classes add integer and binary variables (a `*_milp` model with only continuous variables is of course a valid LP). SCIP and Cbc expose only a MILP class; Clp and SoPlex only an LP class.
- Quadratic objectives (`*_qp`) are currently supported through HiGHS only.

## Switching backends

The examples follow one convention: the backend appears in exactly two places — the include and one alias.

```cpp
#include "mippp/solvers/highs/all.hpp"

using milp_type = highs_milp;
```

Change those to `gurobi`/`gurobi_milp` and recompile: the rest of the program is untouched. There is no linking step to adjust, because solver libraries are loaded at runtime.

### Versioned namespaces and the `all.hpp` aliases

Each binding lives in a namespace named after the solver release it targets, `mippp::gurobi::v12_0::gurobi_milp` for instance, and `all.hpp` aliases the newest one into `mippp` as `gurobi_milp`. When a binding for a newer solver release is added, the alias moves to it in the next minor release of MIP++, the previous namespace stays available unchanged, the move is announced in the release notes, and the [compatibility matrix](compatibility.md) records which solver builds each namespace loads. Code that must keep loading a given solver release spells the versioned namespace instead of the alias.

To choose the solver at *runtime* — for a `--solver` command-line flag, say — write the model-building code once as a template over the backend and dispatch on the flag; that pattern, and the capability checks that go with it, are the subject of [Writing solver-generic code](generic-code.md).

Only the solvers actually installed on the machine need to be present: a backend fails when its first model is constructed (with a descriptive exception), not at program startup.

## How solver libraries are found

Each backend has an api object, `gurobi_api` say, holding the C entry points the wrapper uses. A model's default constructor obtains it through `gurobi_api::load()`, which opens the solver's shared library through the platform loader (`dlopen` on Linux and macOS, `LoadLibrary` on Windows) and resolves those entry points — nothing is linked, and MIP++ needs no third-party loader library. Resolution order (first match wins):

1. an explicit path passed to `load` — `gurobi_milp model(gurobi_api::load("/opt/gurobi1201/linux64/lib/libgurobi120.so"));`
2. the `MIPPP_<SOLVER>_LIBRARY` environment variable, holding the full path of the exact file to load;
3. a search of the dynamic loader's directories (`LD_LIBRARY_PATH` and system library paths) for the conventional name, accepting version-suffixed sonames (`libhighs.so.1.10.0`) when the plain name is absent.

An api object is one loaded library file, and `library_path()` returns it. `load` returns the same object whenever it resolves to the same file, and that object lives for the rest of the process: every model of a backend shares one table of entry points, and `model.native_api()` is a reference that cannot dangle. Two explicit paths naming two different files give two independent api objects, each with its own global state, so two versions of the same solver can serve two models in one process. The directory search of step 3 is memoized per solver for the life of the process, so default-constructing many models is cheap; explicit paths and the environment variable are never cached.

A library that exists but lacks the expected entry points (a same-named build without the C API, say) is rejected with the loader's own message rather than half-loaded, and the exception lists every candidate tried. See [Installation](../getting-started/installation.md#making-solver-libraries-discoverable) for per-solver environment setup.

## Feature support

Core LP/MILP modeling works on every backend. Optional capabilities — dual solutions, callbacks, MIP starts, SOS and indicator constraints, parameter control — vary; each is a [concept](../reference/concepts.md), so code that needs one states it and the compiler enforces it.

The matrices below record which features are implemented **and tested** per backend (generated from the test suites).

### MILP models

![MILP feature support matrix](../assets/features_tables/milp_table_light.png#only-light)
![MILP feature support matrix](../assets/features_tables/milp_table_dark.png#only-dark)

### LP models

![LP feature support matrix](../assets/features_tables/lp_table_light.png#only-light)
![LP feature support matrix](../assets/features_tables/lp_table_dark.png#only-dark)

Notable current limitations (see the
[roadmap](https://github.com/fhamonic/mippp#roadmap) for what's planned):

- **Callbacks** — candidate-solution callbacks are implemented on Gurobi, CPLEX, COPT and Xpress, and validated on Gurobi, CPLEX and COPT; SCIP has none yet. Node-relaxation (user-cut) callbacks are specified but not yet implemented.
- **Solve status** — `get_status()` is part of `lp_model`, so every backend reports one, but the set of tags a backend can return varies (it is part of the model type). `refine_lp_status()` — resolving `infeasible_or_unbounded` into one of the two — exists only on `gurobi_lp` and `cplex_lp`, and `glpk_milp` cannot yet report `infeasible`. See [Status, limits and tolerances](../solving/status-and-limits.md).
- **Quadratic objectives** — HiGHS only. Quadratic constraints: none yet.
- **SOS constraints and LP basis warm starts** — specified as concepts, not yet implemented by any backend.
- **Ranged constraints** — Clp and Cbc only (`has_ranged_constraints`, with `has_readable_constraint_bounds` to read them back); see [Special constraints](../modeling/special-constraints.md#ranged-constraints).
- **Indicator constraints** — Gurobi and CPLEX only (`has_indicator_constraints`). The call returns no handle, so an indicator constraint cannot be read back or edited once added ([details](../modeling/special-constraints.md)).
- **Solver-specific parameters** — the uniform interface covers [limits and tolerances](../solving/status-and-limits.md); there is no uniform passthrough for solver-specific knobs such as Gurobi's `MIPFocus` or CPLEX's emphasis settings. The escape hatch is the `has_native_handles` concept, which every model class satisfies: `native_model()` returns the solver's own objects and `native_api()` the loaded `*_api` object, whose members are the solver's raw C functions, so the call is made directly:

    ```cpp
    gurobi_milp model;
    auto [env, grb_model] = model.native_model();
    model.native_api().setintparam(env, "MIPFocus", 2);
    ```

    What comes back depends on the backend: a `(env, model)` pair where the solver has an environment — `std::pair<GRBenv *, GRBmodel *>` (Gurobi), `std::pair<CPXENVptr, CPXLPptr>` (CPLEX), `std::pair<copt_env *, copt_prob *>` (COPT), `std::pair<MSKenv_t, MSKtask_t>` (MOSEK) — and the single problem object otherwise: `XPRSprob` (Xpress), `SCIP *` (SCIP), `glp_prob *` (GLPK), `Clp_Simplex *` (Clp), `Cbc_Model *` (Cbc), `void *` (HiGHS, SoPlex). To address one variable or constraint through the raw API, `native_id(v)` and `native_id(c)` translate a MIP++ handle into what the solver calls it: the column or row index (1-based on GLPK), or the `SCIP_VAR *` / `SCIP_CONS *` on SCIP. Handles and native indices drift apart once variables have been removed on the backends that delete columns (HiGHS, Gurobi, CPLEX), which is what the translation is for. Anything done through these handles bypasses MIP++'s bookkeeping (handle remapping, name tracking), so keep it to parameters and read-only queries.

## Next

[Writing solver-generic code](generic-code.md) — one model builder, every backend, with capability differences resolved at compile time.
