# ![MIP++](misc/logo/mippp_256x.png)

**Write your optimization model once — solve it with any solver, at near the speed of the raw C API.**

MIP++ is a header-only C++26 library for linear, mixed-integer, and quadratic
programming. It gives you an algebraic modeling syntax as readable as JuMP or
Pyomo, but compiles down to direct calls into the solver's own C API. The
same model code targets any of **11 solvers** — you choose the backend at compile
time, and its shared library is loaded dynamically at runtime, with no link-time
solver dependency.

[![C++26](https://img.shields.io/badge/C++-26-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/26)
[![Conan](https://img.shields.io/badge/Conan-2.0+-blue.svg?style=flat)](https://conan.io)
[![License](https://img.shields.io/badge/license-Boost%20Software%20License-blue)](https://www.boost.org/users/license.html)

## Why MIP++ for research

- **One model, every solver.** Your model is written once against MIP++'s
  generic concepts; benchmarking Gurobi vs. CPLEX vs. HiGHS vs. SCIP is a
  one-line type change and a recompile — no `#ifdef` soup and no linking against
  a solver SDK. Templatize over the backend and you can even pick the solver at
  runtime.
- **No modeling tax.** Building an N-Queens model with MIP++ is within **10–20%
  of hand-written C**, while Python layers (gurobipy, Python-MIP, PuLP) are
  **40–550× slower** — the overhead that matters for large models, column
  generation, and repeated re-solves. [See the benchmark ↓](#performance)
- **Built for algorithms, not just models.** Branch-and-cut **callbacks** with
  lazy constraints, **column generation**, **dual values**, and **MIP starts** —
  the building blocks of real OR methods. User cuts and SOS/indicator
  constraints are on the roadmap.
- **A functional, zero-copy expression system.** Objectives and constraint
  families are composed from C++ ranges with `xsum`; expressions are *lazy views*
  that allocate nothing and are streamed into the solver in a single pass. Index
  variables by your problem's own coordinates through a lambda id-map (`x(i, j)`
  is O(1) arithmetic, no hash map), and have names assigned lazily rather than
  up front.
- **Header-only, drop-in.** Solvers are loaded dynamically at runtime, so there
  are no link-time solver dependencies to fight with.

## A first model

```cpp
#include "mippp/solvers/gurobi/all.hpp"
using namespace mippp;
using namespace mippp::operators;

gurobi_api api;         // loads the Gurobi C API at runtime
gurobi_lp model(api);   // swap for highs_lp, scip_milp, cplex_milp, ...

auto x1 = model.add_variable();
auto x2 = model.add_variable({.upper_bound = 3});

model.set_maximization();
model.set_objective(4 * x1 + 5 * x2);
model.add_constraint(x1 <= 4);
model.add_constraint(2 * x1 + x2 <= 9);

model.solve();
auto sol = model.get_solution();
std::print("x1: {}, x2: {}\n", sol[x1], sol[x2]);
```

## A closer look

The design ideas that make the modeling layer pleasant to write research code
against:

**Functional expressions + lambda-indexed variables.** Index variables by your
problem's natural coordinates, then build entire constraint families over ranges:

```cpp
// x(i, j) maps to a variable by plain arithmetic — no hash map, no lookup table.
auto x = model.add_binary_variables(n * n, [n](int i, int j) { return i * n + j; });

// A whole family of constraints, composed functionally over ranges:
model.add_constraints(std::views::iota(0, n), [&](int i) {
    return xsum(std::views::iota(0, n), [&](int j) { return x(i, j); }) == 1;
});
```

`xsum(...)` returns a lazy `linear_expression_view`; no intermediate term vector
is ever materialized, and the model streams each term once into a reused buffer.
Pass a name lambda alongside the id-map and the names are assigned *lazily*
instead of eagerly up front.

**Branch-and-cut via a clean callback wrapper.** The callback receives a typed
handle exposing the incumbent and `add_lazy_constraint`, so lazy subtour
elimination for the TSP stays a handful of lines (Gurobi, CPLEX, COPT):

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto sol = handle.get_solution();
    for(auto && subtour : find_subtours(sol))
        handle.add_lazy_constraint(xsum(subtour, x) <= int(subtour.size()) - 1);
});
```

**Column generation with duals.** Read dual values, price your subproblem, and
inject improving columns — the classic loop, unobscured:

```cpp
for(;;) {
    model.solve();
    auto duals = model.get_dual_solution();
    auto pattern = solve_pricing(duals);            // your subproblem
    if(reduced_cost(pattern) >= -epsilon) break;    // no improving column left
    model.add_column(columns_of(pattern), {.obj_coef = 1});
}
```

For large-scale pricing, a `column_manager` layers on compile-time *column
properties* (reduced-cost windows, age, …) and pluggable activation/eviction
strategies. Full runnable versions of all three live in
[`examples/`](examples/) *(link to docs)*.

## Supported solvers

| Backend | LP | MILP | QP |
| --- | :-: | :-: | :-: |
| HiGHS | ✓ | ✓ | ✓ |
| Gurobi, CPLEX, Xpress, COPT, MOSEK, SCIP, Cbc, GLPK | ✓ | ✓ | |
| Clp, SoPlex | ✓ | | |

<sub>QP is currently supported through HiGHS only. Per-feature support (duals,
callbacks, MIP starts, warm starts, …) varies by backend and is detailed in the
[feature tables below](#feature-support).</sub>

## Performance

Model-creation time for the N-Queens problem (`N²` binary variables, `6N−6`
constraints), relative to the Gurobi C API — **lower is better, 1.0× = C speed**:

| N | C | **MIP++** | gurobipy | Python-MIP | JuMP (warm) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 500 | 1.0× | **1.1×** | 289× | 305× | 14.5× |
| 1000 | 1.0× | **1.1×** | 552× | 542× | 24.1× |

Full tables (all sizes, both Gurobi and Cbc backends) and reproduction steps:
[mippp_nqueens](https://github.com/fhamonic/mippp_nqueens).

## Installation

MIP++ requires **GCC 15 / C++26** (it uses C++26 ranges such as `std::views::concat`).

**Conan (from source):**
```bash
git clone https://github.com/fhamonic/mippp && cd mippp
conan create . -u -b=missing -pr=<your_conan_profile>
```
then add `mippp/<version>` to your `conanfile`.

**CMake subdirectory:**
```cmake
add_subdirectory(dependencies/mippp)
target_link_libraries(<target> INTERFACE mippp)
```

Each solver's shared library is discovered at runtime; only the solvers you
actually have installed need to be present. See [CONTRIBUTING.md](CONTRIBUTING.md)
for details.

## Feature support

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="misc/features_tables/milp_table_dark.png">
    <img alt="MILP feature support matrix" src="misc/features_tables/milp_table_light.png">
  </picture>
</p>

*(Linear programming table: [`misc/features_tables`](misc/features_tables/).)*

## Documentation, citing, contributing

- 📖 Documentation: *(link once the docs site is live)*
- 📄 If you use MIP++ in academic work, please cite it — see [`CITATION.cff`](CITATION.cff).
- 🤝 Contributions welcome: [CONTRIBUTING.md](CONTRIBUTING.md).
- ⚖️ Licensed under the [Boost Software License 1.0](LICENSE).
