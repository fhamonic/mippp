<div class="mip-hero" markdown>

![MIP++ logo](assets/mippp.png)

# MIP++

**One model, any solver — at raw C API speed.**

[Get started](getting-started/index.md){ .md-button .md-button--primary }
[View on GitHub](https://github.com/fhamonic/mippp){ .md-button }

---

</div>

MIP++ is a header-only C++23 library for linear, mixed-integer, and quadratic programming — the fastest way to write portable, solver-agnostic optimization models in modern C++. It gives you an algebraic modeling syntax as readable as JuMP or Pyomo, but compiles down to direct calls into the solver's own C API. The same model code targets any of **11 solvers** — you choose the backend at compile time, and its shared library is loaded dynamically at runtime, with no link-time solver dependency.

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

## Where to start

The documentation follows the path of a modeling study: build a model, solve it, read the results, then wrap an algorithm around it.

**1. Getting started** — read in order, about half an hour.

- [**Why MIP++**](getting-started/index.md) — the design choices behind the library, why they matter for operations-research work, and who the library is (and is not) for. **Start here.**
- [**Installation**](getting-started/installation.md) — requirements, Conan or CMake setup, and making solver libraries discoverable.
- [**A first model**](getting-started/first-model.md) — a guided tour of the program above.
- [**Coming from gurobipy, JuMP or PuLP**](getting-started/coming-from.md) — a translation table, and the six things that differ.

**2. Modeling** — [variables and index sets](modeling/variables.md), [expressions and constraint families](modeling/expressions.md), [objectives](modeling/objectives.md) (including quadratic), [special constraints](modeling/special-constraints.md).

**3. Solving** — [status, limits and tolerances](solving/status-and-limits.md), [solutions, duals and reduced costs](solving/solutions.md), [re-solving and model updates](solving/updates.md).

**4. Algorithms** — [branch-and-cut with lazy constraints](algorithms/branch-and-cut.md), [column generation](algorithms/column-generation.md).

**5. Solvers** — [choosing a solver](solvers/index.md) among the 11 backends, and [writing solver-generic code](solvers/generic-code.md) for cross-solver experiments.

Complete runnable programs — N-Queens, Sudoku, TSP with lazy constraints, cutting stock by column generation — are indexed in [Worked examples](examples.md).

## Supported solvers

| Backend | LP | MILP | QP |
| --- | :-: | :-: | :-: |
| HiGHS | ✓ | ✓ | ✓ |
| Gurobi, CPLEX, Xpress, COPT, MOSEK, GLPK | ✓ | ✓ | |
| Cbc, SCIP | | ✓ | |
| Clp, SoPlex | ✓ | | |

<sub>Per-feature support (duals, callbacks, MIP starts, …) varies by backend — see the feature matrices in [Choosing a solver](https://fhamonic.github.io/mippp/solvers/).</sub>

## Performance

Time to *build* the N-Queens model (`N²` binary variables, `6N−6` constraints) — MIP++ in milliseconds, the other interfaces as a multiple of it **on the same backend**:

| N | **MIP++** Cbc | **MIP++** HiGHS | OR-tools Cbc | OR-tools HiGHS | JuMP Cbc | JuMP HiGHS |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | **0.9 ms** | **1.5 ms** | 4.0× | 2.3× | 14.3× | 43.9× |
| 500 | **18.9 ms** | **36.6 ms** | 4.8× | 2.4× | 13.6× | 7.4× |
| 1000 | **72.8 ms** | **146.2 ms** | 5.2× | 2.6× | 13.7× | 6.5× |

A million binary variables in 73 ms; the Python layers land two to three orders of magnitude above that. Build time is noise when a single solve runs for hours — it matters on very large models and in loops that touch the model constantly, and the [Performance](performance.md) page is explicit about that scope. Full tables, the case where MIP++ *loses* (SCIP), and the methodology are there too — benchmark code and raw per-solver timings in [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens).

## Acknowledgements

This work is grounded in the PhD thesis and postdoctoral positions of François Hamonic, funded by Région Sud and Natural Solutions (PhD grant), the ERC project SCALED (grant n°949812), the PEPR VDBI project RESILIENCE, and Aix-Marseille University's ITEM institute (postdoctoral positions).

## Documentation, citing, license

- 📖 Documentation: [fhamonic.github.io/mippp](https://fhamonic.github.io/mippp/) — sources live under [`docs/`](https://github.com/fhamonic/mippp/tree/main/docs).
- 📄 If you use MIP++ in academic work, please cite it — see [`CITATION.cff`](https://github.com/fhamonic/mippp/tree/main/CITATION.cff).
- ⚖️ Licensed under the [Boost Software License 1.0](https://github.com/fhamonic/mippp/tree/main/LICENSE.md).

