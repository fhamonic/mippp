<div class="mip-hero" markdown>

![MIP++ logo](assets/mippp.png)

# MIP++

**One model, any solver — at raw C API speed.**

[Get started](getting-started/index.md){ .md-button .md-button--primary }
[View on GitHub](https://github.com/fhamonic/mippp){ .md-button }

---

</div>

MIP++ is a header-only C++26 library for linear, mixed-integer, and quadratic programming. It gives you an algebraic modeling syntax as readable as JuMP or Pyomo, but compiles down to direct calls into the solver's own C API. The same model code targets any of **11 solvers** — you choose the backend at compile time, and its shared library is loaded dynamically at runtime, with no link-time solver dependency.

```cpp
#include <iostream>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

int main() {
    highs_api api;        // loads the HiGHS C API at runtime
    highs_lp model(api);  // swap for gurobi_lp, cplex_lp, clp_lp, ...

    auto x1 = model.add_variable();
    auto x2 = model.add_variable({.upper_bound = 3});

    model.set_maximization();
    model.set_objective(4 * x1 + 5 * x2);
    model.add_constraint(x1 <= 4);
    model.add_constraint(2 * x1 + x2 <= 9);

    model.solve();
    auto sol = model.get_solution();
    std::cout << "x1 = " << sol[x1] << ", x2 = " << sol[x2] << "\n";
}
```

## Where to start

- [**Why MIP++**](getting-started/index.md) — the design choices behind the library, and why they matter for operations-research work. **Start here.**
- [**Installation**](getting-started/installation.md) — requirements, Conan or CMake setup, and making solver libraries discoverable.
- [**A first model**](getting-started/first-model.md) — a guided tour of the program above.
- [**Expressions and constraints**](getting-started/expressions.md) — `xsum`, lambda-indexed variables, and constraint families over ranges.
- [**Choosing a solver**](getting-started/solvers.md) — the 11 backends and what each supports.

For complete, runnable programs (N-Queens, TSP with lazy constraints, cutting-stock by column generation, Sudoku), see the [`examples/`](https://github.com/fhamonic/mippp/tree/main/examples) directory.

## Supported solvers

| Backend | LP | MILP | QP |
| --- | :-: | :-: | :-: |
| HiGHS | ✓ | ✓ | ✓ |
| Gurobi, CPLEX, Xpress, COPT, MOSEK, GLPK | ✓ | ✓ | |
| SCIP, Cbc | | ✓ | |
| Clp, SoPlex | ✓ | | |

Per-feature support (duals, callbacks, MIP starts, …) varies by backend and is detailed in [Choosing a solver](getting-started/solvers.md).

## Performance

Model-creation time for the N-Queens problem (`N²` binary variables, `6N−6` constraints), relative to the raw Gurobi C API — lower is better, 1.0× = C speed:

| N | C | **MIP++** | gurobipy | Python-MIP | JuMP (warm) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 500 | 1.0× | **1.1×** | 289× | 305× | 14.5× |
| 1000 | 1.0× | **1.1×** | 552× | 542× | 24.1× |

Benchmark code, setup, and raw per-solver timings: [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens).

## License and citing

MIP++ is distributed under the [Boost Software License 1.0](https://github.com/fhamonic/mippp/blob/main/LICENSE).
If you use it in academic work, please cite it — see [`CITATION.cff`](https://github.com/fhamonic/mippp/blob/main/CITATION.cff).
Contributions are welcome: [CONTRIBUTING.md](https://github.com/fhamonic/mippp/blob/main/CONTRIBUTING.md).
