<p align="center">
  <img width="256" height="238" src="docs/assets/mippp.png" alt=" MIP++ ">
</p>

#

**One model, any solver — at raw C API speed.**

MIP++ is a header-only C++23 library for linear, mixed-integer, and quadratic programming — the fastest way to write portable, solver-agnostic optimization models in modern C++. It gives you an algebraic modeling syntax as readable as JuMP or Pyomo, but compiles down to direct calls into the solver's own C API. The same model code targets any of **11 solvers** — you choose the backend at compile time, and its shared library is loaded dynamically at runtime, with no link-time solver dependency.

📖 **[Documentation](https://fhamonic.github.io/mippp/)** — start with the [Getting Started guide](https://fhamonic.github.io/mippp/getting-started/).

[![C++23](https://img.shields.io/badge/C++-23-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/23)
[![Conan](https://img.shields.io/badge/Conan-2.0+-blue.svg?style=flat)](https://conan.io)
[![License](https://img.shields.io/badge/license-Boost%20Software%20License-blue)](https://www.boost.org/users/license.html)

## Highlights

- **One model, every solver.** Benchmarking Gurobi vs. CPLEX vs. HiGHS vs. SCIP is a two-line change and a recompile — no `#ifdef` soup, no linking against a solver SDK. ([Why MIP++](https://fhamonic.github.io/mippp/getting-started/))
- **No modeling tax.** A million-variable model built in **73 ms** — **2.2–5.2× faster than the OR-Tools C++ API**, 6–15× faster than JuMP, and orders of magnitude ahead of the Python layers. Build time is noise for a one-shot solve that runs for hours; it bites on very large models and in loops that touch the model constantly — MIP++ is built for those regimes. [See the benchmark ↓](#performance)
- **Built for algorithms, not just models.** [Branch-and-cut callbacks](https://fhamonic.github.io/mippp/algorithms/branch-and-cut/) with lazy constraints, [column generation](https://fhamonic.github.io/mippp/algorithms/column-generation/), dual values, reduced costs, MIP starts, indicator constraints, and [in-place model updates](https://fhamonic.github.io/mippp/solving/updates/) — a MIP++ model *is* the solver's own model, so re-solves and modifications never re-extract anything.
- **A functional, zero-copy expression system.** Objectives and constraint families are composed from C++ ranges with `xsum`; expressions are lazy views that allocate nothing. ([Expressions and constraints](https://fhamonic.github.io/mippp/modeling/expressions/))

## A first model

```cpp
#include <print>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

int main() {
    highs_api api;        // loads the HiGHS C API at runtime
    highs_lp model(api);  // swap for gurobi_*, cplex_*, cbc_*, ...

    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 3});

    model.set_maximization();
    model.set_objective(4 * x1 + 5 * x2);
    model.add_constraint(x1 <= 4);
    model.add_constraint(2 * x1 + x2 <= 9);

    model.solve();
    auto sol = model.get_solution();

    std::println("objective = {}", model.get_solution_value());
    std::println("x1 = {}, x2 = {}", sol[x1], sol[x2]);
    return 0;
}
```

The [Getting Started guide](https://fhamonic.github.io/mippp/getting-started/) walks through this model, the expression system, and solver selection; [the algorithms guides](https://fhamonic.github.io/mippp/algorithms/branch-and-cut/) cover branch-and-cut callbacks and column generation. Runnable examples (N-Queens, sudoku, TSP with lazy constraints, cutting stock) live in [`examples/`](examples/).

## Is MIP++ for you?

MIP++ has a deliberate niche, and it is worth being honest about where its edges are:

- **Optimization embedded in a larger C++ system** — a simulator, a planning service, a shipped binary that must run with whatever solver the user has installed, without recompiling per solver. This is the sweet spot.
- **Cross-solver experiments** — reviewers asking for results on Gurobi *and* an open solver, licenses that differ between your laptop, the cluster, and your coauthors' machines. The two-line backend switch was built for exactly this.
- **Build-bound iterative methods in C++** — column generation, cutting planes, decomposition. In-place updates, `add_column`, and lazy-constraint callbacks are here today; LP basis warm starts and user-cut callbacks are still on the [roadmap](#roadmap), and they matter in precisely this regime — watch the project if you need them.
- **Everyday modeling in Python or Julia, heavy solver-specific parameter tuning, CP or scheduling** — stay with gurobipy, JuMP, or OR-Tools CP-SAT. They are mature, their communities are large, and for a one-shot solve where solver time dominates, the modeling tax rarely matters.

Two adoption realities to weigh: MIP++ requires **GCC 14+ / C++23** and assumes comfort with modern C++ (ranges, concepts, template diagnostics). It is a young, single-maintainer project — contributions welcome ! — thus, pin a version if you build long-lived research code on it.

## Supported solvers

| Backend | LP | MILP | QP |
| --- | :-: | :-: | :-: |
| HiGHS | ✓ | ✓ | ✓ |
| Gurobi, CPLEX, Xpress, COPT, MOSEK, GLPK | ✓ | ✓ | |
| Cbc, SCIP | | ✓ | |
| Clp, SoPlex | ✓ | | |

<sub>Per-feature support (duals, callbacks, MIP starts, …) varies by backend — see the feature matrices in [Choosing a solver](https://fhamonic.github.io/mippp/solvers/).</sub>

## Performance

Time to *build* the N-Queens model (`N²` binary variables, `6N−6` constraints) — MIP++ in milliseconds, the other interfaces as a multiple of it **on the same backend**, lower is better:

| N | **MIP++** Cbc | **MIP++** HiGHS | OR-tools Cbc | OR-tools HiGHS | JuMP Cbc | JuMP HiGHS |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | **0.9 ms** | **1.5 ms** | 4.0× | 2.3× | 14.3× | 43.9× |
| 500 | **18.9 ms** | **36.6 ms** | 4.8× | 2.4× | 13.6× | 7.4× |
| 1000 | **72.8 ms** | **146.2 ms** | 5.2× | 2.6× | 13.7× | 6.5× |

Only model construction is timed, never the resolution. The Python layers (Python-MIP, PuLP) land two to three orders of magnitude above MIP++ under CPython. Note that OR-Tools' `MPSolver` and JuMP fill their own backend-independent structures and defer the native model build to `Solve()`, while the MIP++ timings include it — which is also why OR-Tools comes out ahead with SCIP. N-Queens is a variable-heavy, constraint-light model, so these numbers chiefly stress variable creation and short-row streaming; a companion benchmark with dense, structured constraints is planned.

Full tables, methodology and setup: [Performance](https://fhamonic.github.io/mippp/performance/) — benchmark code and raw per-solver timings in [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens).

## Installation

MIP++ targets **GCC 14 / C++23** but **GCC 15 / C++26** is recommended. Install via Conan or as a CMake subdirectory:

```bash
git clone https://github.com/fhamonic/mippp && cd mippp
conan create . -u -b=missing -pr=<your_conan_profile>
```

Solver shared libraries are discovered at runtime; only the solvers you actually have installed need to be present. Full instructions: [Installation](https://fhamonic.github.io/mippp/getting-started/installation/).

## Roadmap

The modeling core is in place (LP/MILP/QP, lazy-constraint callbacks, column generation, reduced costs, MIP starts, SOS/indicator constraints). Next up, roughly by priority: **LP basis warm-starts**, **user-cut callbacks**, **heuristic-solution injection**, **native-handle access** for setting solver-specific parameters (Gurobi's `MIPFocus`, CPLEX emphasis settings, …) — the `*_api` objects already expose the raw C entry points, but the model classes do not yet hand out their native handles — then QP objectives beyond HiGHS, QCP/SOCP constraints, model file I/O (LP/MPS), infeasibility diagnosis (IIS), solution pools, multi-objective support, semi-continuous variables, logging control, and progress getters.

The first three items are exactly what a build-bound, re-solve-heavy research code wants most, and we know it — until they land, a column-generation or cutting-plane study may still be better served by a mature layer, and the honest comparison is on the [Is MIP++ for you?](#is-mip-for-you) list above.

Contributions are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md), and open an issue to claim an item.

## Acknowledgements

This work is grounded in the PhD thesis and postdoctoral positions of François Hamonic, funded by Région Sud and Natural Solutions (PhD grant), the ERC project SCALED (grant n°949812), the PEPR VDBI project RESILIENCE, and the OASIS project of Aix-Marseille University's ITEM institute (postdoctoral positions).

## Documentation, citing, license

- 📖 Documentation: [fhamonic.github.io/mippp](https://fhamonic.github.io/mippp/) — sources live under [`docs/`](docs/).
- 📄 If you use MIP++ in academic work, please cite it — see [`CITATION.cff`](CITATION.cff).
- ⚖️ Licensed under the [Boost Software License 1.0](LICENSE.md).
