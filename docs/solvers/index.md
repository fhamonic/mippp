# Choosing a solver

Every backend exposes the same modeling interface, so the choice of solver is a two-line change. This page lists the available backends, how to switch between them, and what each one supports.

## The backends

Each backend lives in `mippp/solvers/<name>/all.hpp` and provides an api class plus one model class per problem kind. The version in the alias points at the solver release the binding targets.

| Solver | Header (`mippp/solvers/…`) | API class | Model classes | Targets |
| --- | --- | --- | --- | --- |
| [HiGHS](https://highs.dev) | `highs/all.hpp` | `highs_api` | `highs_lp`, `highs_milp`, `highs_qp` | v1.10 |
| [Gurobi](https://www.gurobi.com) | `gurobi/all.hpp` | `gurobi_api` | `gurobi_lp`, `gurobi_milp` | v12.0 |
| [CPLEX](https://www.ibm.com/products/ilog-cplex-optimization-studio) | `cplex/all.hpp` | `cplex_api` | `cplex_lp`, `cplex_milp` | v22.12 |
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

The examples follow one convention: the backend appears in exactly three places — the include and two aliases.

```cpp
#include "mippp/solvers/highs/all.hpp"

using api_type  = highs_api;
using milp_type = highs_milp;
```

Change those to `gurobi`/`gurobi_api`/`gurobi_milp` and recompile: the rest of the program is untouched. There is no linking step to adjust, because solver libraries are loaded at runtime.

To choose the solver at *runtime* — for a `--solver` command-line flag, say — write the model-building code once as a template over the backend and dispatch on the flag; that pattern, and the capability checks that go with it, are the subject of [Writing solver-generic code](generic-code.md).

Only the solvers actually installed on the machine need to be present: a backend fails at api-construction time (with a descriptive exception), not at program startup.

## How solver libraries are found

Constructing the api object loads the solver's shared library through[dylib](https://github.com/martin-olivier/dylib). Resolution order (first match wins):

1. an explicit path passed to the constructor — `gurobi_api api("/opt/gurobi1201/linux64/lib/libgurobi120.so");`
2. the `MIPPP_<SOLVER>_LIBRARY` environment variable, holding the full path of the exact file to load;
3. a search of the dynamic loader's directories (`LD_LIBRARY_PATH` and system library paths) for the conventional name, accepting version-suffixed sonames (`libhighs.so.1.10.0`) when the plain name is absent.

See [Installation](../getting-started/installation.md#making-solver-libraries-discoverable) for per-solver environment setup.

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

- **Callbacks** — candidate-solution callbacks are implemented on Gurobi, CPLEX, COPT, SCIP and Xpress, and validated on Gurobi, CPLEX and COPT. Node-relaxation (user-cut) callbacks are specified but not yet implemented.
- **Solve status** — `solve_status()` is part of `lp_model`, so every backend reports one, but the set of tags a backend can return varies (it is part of the model type). `refine_lp_status()` — resolving `infeasible_or_unbounded` into one of the two — exists only on `gurobi_lp` and `cplex_lp`, and `glpk_milp` cannot yet report `infeasible`. See [Status, limits and tolerances](../solving/status-and-limits.md).
- **Quadratic objectives** — HiGHS only. Quadratic constraints: none yet.
- **SOS constraints and LP basis warm starts** — specified as concepts, not yet implemented by any backend.
- **Indicator constraints** — usable on Gurobi and CPLEX by calling `add_indicator_constraint` directly, but `has_indicator_constraints` is `false` everywhere because those implementations return `void` where the concept expects a constraint handle ([details](../modeling/special-constraints.md#one-model-both-encodings)).

## Next

[Writing solver-generic code](generic-code.md) — one model builder, every backend, with capability differences resolved at compile time.
