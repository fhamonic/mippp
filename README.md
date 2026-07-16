<p align="center">
  <img width="256" height="238" src="docs/assets/mippp.png" alt=" MIP++ ">
</p>

#

**One model, any solver — at raw C API speed.**

MIP++ is a header-only C++23 library for linear, mixed-integer, and quadratic programming. It gives you an algebraic modeling syntax as readable as JuMP or Pyomo, but compiles down to direct calls into the solver's own C API. The same model code targets any of **11 solvers** — you choose the backend at compile time, and its shared library is loaded dynamically at runtime, with no link-time solver dependency.

📖 **[Documentation](https://fhamonic.github.io/mippp/)** — start with the [Getting Started guide](https://fhamonic.github.io/mippp/getting-started/).

[![C++23](https://img.shields.io/badge/C++-23-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/26)
[![Conan](https://img.shields.io/badge/Conan-2.0+-blue.svg?style=flat)](https://conan.io)
[![License](https://img.shields.io/badge/license-Boost%20Software%20License-blue)](https://www.boost.org/users/license.html)

## Highlights

- **One model, every solver.** Benchmarking Gurobi vs. CPLEX vs. HiGHS vs. SCIP is a two-line change and a recompile — no `#ifdef` soup, no linking against a solver SDK. ([Why MIP++](https://fhamonic.github.io/mippp/getting-started/))
- **No modeling tax.** Model building within **10–20% of hand-written C**, where Python layers (gurobipy, Python-MIP, PuLP) are **40–550× slower**. [See the benchmark ↓](#performance)
- **Built for algorithms, not just models.** [Branch-and-cut callbacks](https://fhamonic.github.io/mippp/advanced/callbacks/) with lazy constraints, [column generation](https://fhamonic.github.io/mippp/advanced/column-generation/), dual values, reduced costs, MIP starts, and SOS/indicator constraints.
- **A functional, zero-copy expression system.** Objectives and constraint families are composed from C++ ranges with `xsum`; expressions are lazy views that allocate nothing. ([Expressions and constraints](https://fhamonic.github.io/mippp/getting-started/expressions/))

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

The [Getting Started guide](https://fhamonic.github.io/mippp/getting-started/) walks through this model, the expression system, and solver selection; [advanced topics](https://fhamonic.github.io/mippp/advanced/callbacks/) cover branch-and-cut callbacks and column generation. Runnable examples (N-Queens, sudoku, TSP with lazy constraints, cutting stock) live in [`examples/`](examples/).

## Supported solvers

| Backend | LP | MILP | QP |
| --- | :-: | :-: | :-: |
| HiGHS | ✓ | ✓ | ✓ |
| Gurobi, CPLEX, Xpress, COPT, MOSEK, GLPK | ✓ | ✓ | |
| Cbc, SCIP | | ✓ | |
| Clp, SoPlex | ✓ | | |

<sub>Per-feature support (duals, callbacks, MIP starts, …) varies by backend — see the feature matrices in [Choosing a solver](https://fhamonic.github.io/mippp/getting-started/solvers/).</sub>

## Performance

Model-creation time for the N-Queens problem (`N²` binary variables, `6N−6` constraints), relative to the Gurobi C API — **lower is better, 1.0× = C speed**:

| N | C | **MIP++** | gurobipy | Python-MIP | JuMP (warm) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 500 | 1.0× | **1.1×** | 289× | 305× | 14.5× |
| 1000 | 1.0× | **1.1×** | 552× | 542× | 24.1× |

Benchmark code, setup, and raw per-solver timings:
[mippp_nqueens](https://github.com/fhamonic/mippp_nqueens).

## Installation

MIP++ targets **GCC 14 / C++23** but **GCC 15 / C++26** is recommended. Install via Conan or as a CMake subdirectory:

```bash
git clone https://github.com/fhamonic/mippp && cd mippp
conan create . -u -b=missing -pr=<your_conan_profile>
```

Solver shared libraries are discovered at runtime; only the solvers you actually have installed need to be present. Full instructions: [Installation](https://fhamonic.github.io/mippp/getting-started/installation/).

## Roadmap

The modeling core is in place (LP/MILP/QP, lazy-constraint callbacks, column generation, reduced costs, MIP starts, SOS/indicator constraints). Next up, roughly by priority: **LP basis warm-starts**, **user-cut callbacks**, heuristic-solution injection, QP objectives beyond HiGHS, QCP/SOCP constraints, model file I/O (LP/MPS), infeasibility diagnosis (IIS), solution pools, multi-objective support, semi-continuous variables, logging control, and progress getters.

Contributions are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md), and open an issue to claim an item.

## Acknowledgements

This work is grounded in the PhD thesis and postdoctoral positions of François Hamonic, funded by Région Sud and Natural Solutions (PhD grant), the ERC project SCALED (grant n°949812), the PEPR VDBI project RESILIENCE, and Aix-Marseille University's ITEM institute (postdoctoral positions).

## Documentation, citing, license

- 📖 Documentation: [fhamonic.github.io/mippp](https://fhamonic.github.io/mippp/) — sources live under [`docs/`](docs/).
- 📄 If you use MIP++ in academic work, please cite it — see [`CITATION.cff`](CITATION.cff).
- ⚖️ Licensed under the [Boost Software License 1.0](LICENSE).
