# ![MIP++](misc/mippp_thick_outlined_256x.png)
MIP++ (aka MIPpp) is a header‑only C++ library that brings a Pythonic modeling interface to mixed‑integer linear programming. It lets you load multiple solver backends (Gurobi, Cbc, Highs, etc...) at runtime, and uses template metaprogramming to preserve Python‑like syntactic sugar while emitting near‑optimal code at compile time.

Work in progress.

[![Generic badge](https://img.shields.io/badge/C++-20-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/20)
[![Generic badge](https://img.shields.io/badge/CMake-3.12+-blue.svg?style=flat&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI1MDguOTkyIiBoZWlnaHQ9IjU1OC42NTYiIGZpbGw9IiNmZmZmZmYiIHhtbG5zOnY9Imh0dHBzOi8vdmVjdGEuaW8vbmFubyI+PHBhdGggZD0iTTYuMzU3IDQ2My4yOTZDNi43OCA0NjIuMDMyIDIzOS4wMTEtLjE0MiAyMzkuMTUzIDBjLjA2OS4wNjggNC45MzUgNTUuNzAzIDEwLjgxNSAxMjMuNjMybDkuMzg4IDEyNC43MzZjLS43MTYuNjc2LTUzLjc1MiA0NS44NjItMTE3Ljg1OCAxMDAuNDE0TDE1LjUxMyA0NTYuMDQzYy01LjE4NyA0LjQ0MS05LjMwNiA3LjcwNi05LjE1NSA3LjI1NHptNDAxLjAyOC0xMC4wNDlsLTEwMS42NjktNDEuODNjLS4zMzgtLjMzOC0zMy45MTItMzg3Ljk0OS0zMy42MjktMzg4LjIzNy4wOTgtLjA5OSA1My40OTYgMTA1Ljg1OSAxMTguNjYzIDIzNS40NjJsMTE4LjI0MiAyMzUuODg2Yy0uMTM0LjEzNC00NS44NTctMTguNDQzLTEwMS42MDgtNDEuMjgyek0wIDUwOS4zNzRjMy44NTgtMy43MSAxNTAuOTc2LTEyOC40ODQgMTUxLjI3Ni0xMjguMzAxLjIzOS4xNDUgNzAuNDczIDI5LjAwMyAxNTYuMDc1IDY0LjEyOWwxNTUuOTM2IDY0LjE1OWMuMTYyLjE2Mi0xMDQuMDc3LjI5NS0yMzEuNjQzLjI5NVMtLjE2MiA1MDkuNTI5IDAgNTA5LjM3NHoiLz48L3N2Zz4=)](https://cmake.org/cmake/help/latest/release/3.12.html)
[![Generic badge](https://img.shields.io/badge/Conan-2.0+-blue.svg?style=flat&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI0ODEiIGhlaWdodD0iNTEyIiBmaWxsPSIjZmZmIiB4bWxuczp2PSJodHRwczovL3ZlY3RhLmlvL25hbm8iPjxwYXRoIGQ9Ik0xMjEuNzQ1IDQyNS43MjRMLjcwNCAzMzkuOTYxVjIyNi42OTkgMTEzLjQzN2wxMDUuNjQtNTAuODYyTDIyMy43MjMgNi4xMDJsMTEuNzQtNS42MTEgNzguNzU4IDM4LjM1MSAxMjIuMjQ2IDU5LjU0MSA0My40ODggMjEuMTktLjAwNCAxMTcuMzM2LS4wMDQgMTE3LjMzNi02Ni4zNzQgNDMuOTk3LTExOC41ODEgNzguNjIxLTUyLjIwOCAzNC42MjR6bTE4Mi4xNDUgMjQuMDc0bDU0LjU4LTM0LjQ0OC4xNzgtMTA1LjEwNS0uNzA0LTEwNC43NjJjLS40ODUuMTg5LTI2LjU1MyAxNC45NDEtNTcuOTI4IDMyLjc4NGwtNTcuMDQ1IDMyLjQ0MS0uMTc4IDEwOC44NDUtLjE3OCAxMDguODQ1IDMuMzQ3LTIuMDc2IDU3LjkyOC0zNi41MjR6bTExOC41NjItNzYuNTM4bDQzLjM2MS0yOC4xNzEuMDI5LTEwMS42NjUtMS4wNjItMTAxLjI0N2MtLjYuMjMtMjEuMzQ0IDEyLjIyOC00Ni4wOTggMjYuNjYxbC00NS4wMDggMjYuMjQzLS4wMzEgMTA0LjgzMS0uMDMxIDEwNC44MzEgMi43MzktMS42NTZjMS41MDctLjkxMSAyMi4yNTItMTQuMzMzIDQ2LjEtMjkuODI3em0tNzEuNzI3LTE3OS4zODlsMTE1LjE0Ni02Ni4wOTVjMC0uMjQ1LTUxLjg3My0yNi41MDktMTE1LjI3My01OC4zNjZMMjM1LjMyNSAxMS40ODkgMjA4LjUxMyAyNC42MiA5Ny4yNzggNzkuMDlsLTg0LjUxMiA0MS45NDJjLS4wNjMuNDI2IDIyMS4wNjUgMTM3LjkyMSAyMjIuNjM1IDEzOC40MzEuMDk3LjAzMiA1MS45OTMtMjkuNDg1IDExNS4zMjMtNjUuNTkyek0yMTQuODAxIDIwNi4zMWMtMjQuMTIyLTQuMDc2LTUxLjEzNi0xNy44MjctNjcuNjA5LTM0LjQxNi0xMS4xNC0xMS4yMTgtMTUuNjMtMTkuNzQ1LTE2LjM0Ny0zMS4wNDItMS4yODItMjAuMjEyIDE2LjQzNi00MC42OTkgNDkuOTk3LTU3LjgxMSAyMS45NzMtMTEuMjA0IDQxLjA1Mi0xNi43ODcgNjMuOTEtMTguNzAyIDQ0LjE2Mi0zLjcgOTMuODM1IDE2LjQ5OSAxMjkuODEgNTIuNzg3bDYuMjMxIDYuMjg1LTkuNzUzIDUuNjI4Yy0xOC4wMDggMTAuMzkxLTQ2LjQ5MyAyNS4zNDEtNDcuMDc5IDI0LjcwOC0uMTExLS4xMi4zOC0yLjQzNyAxLjA5MS01LjE0OCA1LjAzNC0xOS4xOTUtNS4wNi0zNi4yMzItMjcuNzczLTQ2Ljg3Ni0xMi4xNjEtNS42OTktMjYuMjM2LTguNTczLTQxLjk4NC04LjU3NC0xNi4zNC0uMDAxLTI4LjcxNiAyLjc5My00MS41NTIgOS4zOC0xNy44OTQgOS4xODMtMjkuOTk4IDIyLjYzMS0zMi40NzcgMzYuMDgxLTEuNjg5IDkuMTY1IDIuNTAyIDE4LjY5NyAxMC45OTYgMjUuMDEzIDE0LjI1MyAxMC41OTcgMzkuMDc0IDE2LjI1NiA3MS43MzQgMTYuMzU0bDEyLjc3OC4wMzgtMjcuMTE0IDEzLjY1N2MtMTQuOTEzIDcuNTExLTI3LjU5IDEzLjY0Mi0yOC4xNzEgMTMuNjIzcy0zLjU5Mi0uNDYyLTYuNjkxLS45ODV6Ii8+PC9zdmc+)](https://conan.io/index.html)
[![Generic badge](https://img.shields.io/badge/license-Boost%20Software%20License-blue)](https://www.boost.org/users/license.html)

## Features

- **Header-only**: Dynamically loading the C API of solvers allows MIP++ to stay header-only and easy to integrate to C++ projects.
- **Modern C++20**: Utilizes ranges, lambdas, and template metaprogramming for an elegant and efficient API allowing a Pythonic syntax.
- **Solver Agnostic**: Supports a variety of solvers (Gurobi, CPLEX, Clp, Cbc, Highs, SoPlex, SCIP, MOSEK, COPT, GLPK and more upcoming) with a unified API.
- **High Performance**: Optimized for speed with zero cost abstractions and no unnecessary data copying.


## Getting started

### Local Conan package (from latest commit)

```
git clone 
cd mippp
conan create . -u -b=missing -pr=<your_conan_profile>
```

Then add `mippp/<version>` as a requirement in your conanfile.txt or conanfile.py.


### Conan Center package (upcoming release)

Simply add `mippp/1.0.0` to your `conanfile.txt` or `conanfile.py`.


### Using CMAKE subdirectory

```properties
cd <your_project> && mkdir dependencies
git submodule add https://github.com/fhamonic/mippp dependencies/mippp
```
Then in your CMakeLists.txt:
```cmake
add_subdirectory(dependencies/mippp)
...
target_link_libraries(<some_target> INTERFACE mippp)
```
And ensure that your CMake can find the [Range-v3](https://ericniebler.github.io/range-v3/) and [dylib](https://github.com/martin-olivier/dylib) libraries with `find_package` calls.

    
## Code examples

### Test

```cpp
#include "mippp/mip_model.hpp"
using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;
...
gurobi_api api("gurobi120"); // loads libgurobi120.so C API
gurobi_lp model(api);

auto x1 = model.add_variable();
auto x2 = model.add_variable({.upper_bound=3});

model.set_maximization();
model.set_objective(4 * x1 + 5 * x2);
model.add_constraint(x1 <= 4);
model.add_constraint(2*x1 + x2 <= 9);

model.solve();
auto sol = model.get_solution();

std::cout << std::format("x1: {}, x2: {}", sol[x1], sol[x2]) << std::endl;
```

### Maximum Flow

Using the [MELON library](https://github.com/fhamonic/melon), we can express the Maximum Flow problem as

```cpp
#include "melon/static_digraph.hpp"
using namespace fhamonic::melon;
...
static_graph graph = ...;
arc_map_t<static_graph, double> capacity_map = ...;
vertex_t<static_graph> s = ...;
vertex_t<static_graph> t = ...;
...
gurobi_api api();
gurobi_lp model(api);

auto F = model.add_variable();
auto X_vars = model.add_variables(graph.num_arcs());

model.set_maximization();
model.set_objective(F);

auto flow_conservation_constrs = model.add_constraints(
    graph.vertices(),
    [&](auto && u) {
        return OPT((u == s), xsum(graph.out_arcs(s), X_vars) ==
                            xsum(graph.in_arcs(s), X_vars) + F);
    },
    [&](auto && u) {
        return OPT((u == t), xsum(graph.out_arcs(t), X_vars) ==
                            xsum(graph.in_arcs(t), X_vars) - F);
    },
    [&](auto && u) {
        return xsum(graph.out_arcs(u), X_vars) ==
                xsum(graph.in_arcs(u), X_vars);
    });

for(auto && a : graph.arcs()) {
    model.set_variable_upper_bound(X_vars(a), capacity_map[a]);
}
```

### Shortest Path

```cpp
static_graph graph = ...;
arc_map_t<static_graph, double> length_map = ...;
vertex_t<static_graph> s = ...;
vertex_t<static_graph> t = ...;
...
gurobi_api api();
gurobi_lp model(api);

auto X_vars = model.add_variables(graph.num_arcs(),
    [](arc_t<static_graph> a) -> std::size_t { return a; });

model.set_maximization();
model.set_objective(xsum(graph.arcs(), [&](auto && a){
    return length_map[a] * X_vars(a);
}));
for(auto && u : graph.vertices()) {
    const double extra_flow = (u == s ? 1 : (u == t ? -1 : 0));
    model.add_constraint(xsum(graph.out_arcs(u), X_vars) == 
                         xsum(graph.in_arcs(u), X_vars) + extra_flow);
}
```

### Sudoku

```cpp
auto indices = ranges::views::iota(0, 9);
auto values = ranges::views::iota(1, 10);
auto coords = ranges::views::cartesian_product(ranges::views::iota(0, 3),
                                                ranges::views::iota(0, 3));

gurobi_api api;
gurobi_milp model(api);

auto X_vars =
    model.add_binary_variables(9 * 9 * 9, [](int i, int j, int value) {
        return (81 * i) + (9 * j) + (value - 1);
    });

auto single_value_constrs = model.add_constraints(
    ranges::views::cartesian_product(indices, indices), [&](auto && p) {
        auto && [i, j] = p;
        return xsum(values, [&](auto && v) { return X_vars(i, j, v); }) == 1;
    });
auto one_per_row_constrs = model.add_constraints(
    ranges::views::cartesian_product(values, indices), [&](auto && p) {
        auto && [v, i] = p;
        return xsum(indices, [&](auto && j) { return X_vars(i, j, v); }) == 1;
    });
auto one_per_col_constrs = model.add_constraints(
    ranges::views::cartesian_product(values, indices), [&](auto && p) {
        auto && [v, j] = p;
        return xsum(indices, [&](auto && i) { return X_vars(i, j, v); }) == 1;
    });
auto one_per_block_constrs = model.add_constraints(
    ranges::views::cartesian_product(values, coords), [&](auto && p) {
        auto && [v, b] = p;
        return xsum(coords, [&](auto && p2) {
                    return X_vars(3 * std::get<0>(b) + std::get<0>(p2),
                                    3 * std::get<1>(b) + std::get<1>(p2), v);
                }) == 1;
    });

for(auto [i, j, value] : grid_hints) {
    model.set_variable_lower_bound(X_vars(i, j, value), 1);
}

model.solve();
auto solution = model.get_solution();

for(auto i : indices) {
    for(auto j : indices) {
        for(auto v : values) {
            if(solution[X_vars(i, j, v)]) std::cout << ' ' << v;
        }
    }
    std::cout << std::endl;
}
```
## Roadmap

- Add callbacks for Highs, CPLEX, Cbc, MOSEK and COPT.
- Add support for XPRS
- Add special constraints (sos and indicator constraints)
