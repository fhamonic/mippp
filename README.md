# MIPpp

MIPpp is a attempt to provide a way for efficiently instanciate Mixed Integer Linear Programs in C++ in a [Python-MIP](https://www.python-mip.com/) fashion. Like Python-MIP, the aim is to support several MILP solvers as backends (currently CBC, SCIP and GUROBI).
The use of template metaprogramming allows to retain most of the syntactical sugars available in python while generating near optimal code at compile time.

Work in progress.

[![Generic badge](https://img.shields.io/badge/C++-20-blue.svg?style=flat&logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
[![Generic badge](https://img.shields.io/badge/CMake-3.12+-blue.svg?style=flat&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI1MDguOTkyIiBoZWlnaHQ9IjU1OC42NTYiIGZpbGw9IiNmZmZmZmYiIHhtbG5zOnY9Imh0dHBzOi8vdmVjdGEuaW8vbmFubyI+PHBhdGggZD0iTTYuMzU3IDQ2My4yOTZDNi43OCA0NjIuMDMyIDIzOS4wMTEtLjE0MiAyMzkuMTUzIDBjLjA2OS4wNjggNC45MzUgNTUuNzAzIDEwLjgxNSAxMjMuNjMybDkuMzg4IDEyNC43MzZjLS43MTYuNjc2LTUzLjc1MiA0NS44NjItMTE3Ljg1OCAxMDAuNDE0TDE1LjUxMyA0NTYuMDQzYy01LjE4NyA0LjQ0MS05LjMwNiA3LjcwNi05LjE1NSA3LjI1NHptNDAxLjAyOC0xMC4wNDlsLTEwMS42NjktNDEuODNjLS4zMzgtLjMzOC0zMy45MTItMzg3Ljk0OS0zMy42MjktMzg4LjIzNy4wOTgtLjA5OSA1My40OTYgMTA1Ljg1OSAxMTguNjYzIDIzNS40NjJsMTE4LjI0MiAyMzUuODg2Yy0uMTM0LjEzNC00NS44NTctMTguNDQzLTEwMS42MDgtNDEuMjgyek0wIDUwOS4zNzRjMy44NTgtMy43MSAxNTAuOTc2LTEyOC40ODQgMTUxLjI3Ni0xMjguMzAxLjIzOS4xNDUgNzAuNDczIDI5LjAwMyAxNTYuMDc1IDY0LjEyOWwxNTUuOTM2IDY0LjE1OWMuMTYyLjE2Mi0xMDQuMDc3LjI5NS0yMzEuNjQzLjI5NVMtLjE2MiA1MDkuNTI5IDAgNTA5LjM3NHoiLz48L3N2Zz4=)](https://cmake.org/cmake/help/latest/release/3.12.html)
[![Generic badge](https://img.shields.io/badge/Conan-1.33+-blue.svg?style=flat&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSI0ODEiIGhlaWdodD0iNTEyIiBmaWxsPSIjZmZmIiB4bWxuczp2PSJodHRwczovL3ZlY3RhLmlvL25hbm8iPjxwYXRoIGQ9Ik0xMjEuNzQ1IDQyNS43MjRMLjcwNCAzMzkuOTYxVjIyNi42OTkgMTEzLjQzN2wxMDUuNjQtNTAuODYyTDIyMy43MjMgNi4xMDJsMTEuNzQtNS42MTEgNzguNzU4IDM4LjM1MSAxMjIuMjQ2IDU5LjU0MSA0My40ODggMjEuMTktLjAwNCAxMTcuMzM2LS4wMDQgMTE3LjMzNi02Ni4zNzQgNDMuOTk3LTExOC41ODEgNzguNjIxLTUyLjIwOCAzNC42MjR6bTE4Mi4xNDUgMjQuMDc0bDU0LjU4LTM0LjQ0OC4xNzgtMTA1LjEwNS0uNzA0LTEwNC43NjJjLS40ODUuMTg5LTI2LjU1MyAxNC45NDEtNTcuOTI4IDMyLjc4NGwtNTcuMDQ1IDMyLjQ0MS0uMTc4IDEwOC44NDUtLjE3OCAxMDguODQ1IDMuMzQ3LTIuMDc2IDU3LjkyOC0zNi41MjR6bTExOC41NjItNzYuNTM4bDQzLjM2MS0yOC4xNzEuMDI5LTEwMS42NjUtMS4wNjItMTAxLjI0N2MtLjYuMjMtMjEuMzQ0IDEyLjIyOC00Ni4wOTggMjYuNjYxbC00NS4wMDggMjYuMjQzLS4wMzEgMTA0LjgzMS0uMDMxIDEwNC44MzEgMi43MzktMS42NTZjMS41MDctLjkxMSAyMi4yNTItMTQuMzMzIDQ2LjEtMjkuODI3em0tNzEuNzI3LTE3OS4zODlsMTE1LjE0Ni02Ni4wOTVjMC0uMjQ1LTUxLjg3My0yNi41MDktMTE1LjI3My01OC4zNjZMMjM1LjMyNSAxMS40ODkgMjA4LjUxMyAyNC42MiA5Ny4yNzggNzkuMDlsLTg0LjUxMiA0MS45NDJjLS4wNjMuNDI2IDIyMS4wNjUgMTM3LjkyMSAyMjIuNjM1IDEzOC40MzEuMDk3LjAzMiA1MS45OTMtMjkuNDg1IDExNS4zMjMtNjUuNTkyek0yMTQuODAxIDIwNi4zMWMtMjQuMTIyLTQuMDc2LTUxLjEzNi0xNy44MjctNjcuNjA5LTM0LjQxNi0xMS4xNC0xMS4yMTgtMTUuNjMtMTkuNzQ1LTE2LjM0Ny0zMS4wNDItMS4yODItMjAuMjEyIDE2LjQzNi00MC42OTkgNDkuOTk3LTU3LjgxMSAyMS45NzMtMTEuMjA0IDQxLjA1Mi0xNi43ODcgNjMuOTEtMTguNzAyIDQ0LjE2Mi0zLjcgOTMuODM1IDE2LjQ5OSAxMjkuODEgNTIuNzg3bDYuMjMxIDYuMjg1LTkuNzUzIDUuNjI4Yy0xOC4wMDggMTAuMzkxLTQ2LjQ5MyAyNS4zNDEtNDcuMDc5IDI0LjcwOC0uMTExLS4xMi4zOC0yLjQzNyAxLjA5MS01LjE0OCA1LjAzNC0xOS4xOTUtNS4wNi0zNi4yMzItMjcuNzczLTQ2Ljg3Ni0xMi4xNjEtNS42OTktMjYuMjM2LTguNTczLTQxLjk4NC04LjU3NC0xNi4zNC0uMDAxLTI4LjcxNiAyLjc5My00MS41NTIgOS4zOC0xNy44OTQgOS4xODMtMjkuOTk4IDIyLjYzMS0zMi40NzcgMzYuMDgxLTEuNjg5IDkuMTY1IDIuNTAyIDE4LjY5NyAxMC45OTYgMjUuMDEzIDE0LjI1MyAxMC41OTcgMzkuMDc0IDE2LjI1NiA3MS43MzQgMTYuMzU0bDEyLjc3OC4wMzgtMjcuMTE0IDEzLjY1N2MtMTQuOTEzIDcuNTExLTI3LjU5IDEzLjY0Mi0yOC4xNzEgMTMuNjIzcy0zLjU5Mi0uNDYyLTYuNjkxLS45ODV6Ii8+PC9zdmc+)](https://conan.io/index.html)
[![Generic badge](https://img.shields.io/badge/license-Boost%20Software%20License-blue)](https://www.boost.org/users/license.html)

## How to link

### As git and cmake submodules (recommended)

This library is intended to be added as git and cmake submodules with
```properties
[submodule "dependencies/mippp"]
    path = dependencies/mippp
    url = https://github.com/fhamonic/mippp
```
and
```cmake
add_subdirectory(dependencies/mippp)
...
target_link_libraries(<some_target> INTERFACE mippp)
```
Until C++23, the [Range-v3 library](https://ericniebler.github.io/range-v3/) is mandatory for some ranges functionnalities.
This project use the [Conan C++ package manager](https://conan.io/) is used to automatically resolve this dependency.


### As a single-header
The single header is generated in the single-header folder with
```
make single-header
```
then manage to #include it where needed with the range-v3 library.
    
## Code examples

### Test

```cpp
#include "mippp/mip_model.hpp"
using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;
...
grb_api api("gurobi120"); // loads libgurobi120.so C API
grb_lp_model model(api);

auto x1 = model.add_variable();
auto x2 = model.add_variable({.upper_bound=3});

model.set_maximization();
model.set_objective(4 * x1 + 5 * x2);
model.add_constraint(x1 <= 4);
model.add_constraint(2*x1 + x2 <= 9);

model.optimize();
auto sol = model.get_solution();

std::cout << std::format("x1: {}, x2: {}", sol[x1], sol[x2]) << std::endl;
```

### Maximum Flow

Using the [MELON library](https://github.com/fhamonic/melon), we can express the Maximum Flow problem as

```cpp
#include "melon/static_digraph.hpp"
using namespace fhamonic::melon;

#include "mippp/mip_model.hpp"
using namespace fhamonic::mippp;
using namespace fhamonic::mippp::operators;
...
static_graph graph = ...;
arc_map_t<static_graph, double> capacity_map = ...;
vertex_t<static_graph> s = ...;
vertex_t<static_graph> t = ...;
...
grb_api api();
grb_lp_model model(api);

auto F = model.add_variable();
auto X_vars = model.add_variables(graph.num_arcs());

model.set_maximization();
model.set_objective(F);
for(auto && u : graph.vertices()) {
    if(u == s || u == t) continue;
    model.add_constraint(xsum(graph.out_arcs(u), X_vars) == xsum(graph.in_arcs(u), X_vars));
}
model.add_constraint(xsum(graph.out_arcs(s), X_vars) == xsum(graph.in_arcs(s), X_vars) + F);
model.add_constraint(xsum(graph.out_arcs(t), X_vars) == xsum(graph.in_arcs(t), X_vars) - F);

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
grb_api api();
grb_lp_model model(api);

auto X_vars = model.add_variables(graph.num_arcs(),
    [](arc_t<static_graph> a) -> std::size_t { return a; });

model.set_maximization();
model.set_objective(xsum(graph.arcs(), [&](auto && a){
    return length_map[a] * X_vars(a);
}));
for(auto && u : graph.vertices()) {
    const double extra_flow = (u == s ? 1 : (u == t ? -1 : 0));
    model.add_constraint(
        xsum(graph.out_arcs(u), X_vars) == xsum(graph.in_arcs(u), X_vars) + extra_flow);
}
```

### Sudoku

```cpp
auto indices = std::views::iota(0, 9);  // 0 to 8
auto values = std::views::iota(1, 10);  // 1 to 9
auto 3x3coords = std::views::transform( // (0,0), (0,1), (0,2), (1,0), ...
    indices, [](auto && i) { return std::make_pair(i / 3, i % 3); });

grb_api api;
grb_milp model(api);

auto X_vars =
    model.add_binary_variables(9 * 9 * 9, [](int i, int j, int value) {
        return (81 * i) + (9 * j) + (value - 1);
    });

model.set_maximization();
model.set_objective(xsum(X_vars));
for(auto i : indices) {
    for(auto j : indices) {
        model.add_constraint(
            xsum(values, [&](auto && v) { return X_vars(i, j, v); }) == 1);
    }
}
for(auto v : values) {
    for(auto i : indices) {
        model.add_constraint(
            xsum(indices, [&](auto && j) { return X_vars(i, j, v); }) == 1);
        model.add_constraint(
            xsum(indices, [&](auto && j) { return X_vars(j, i, v); }) == 1);
    }
    for(auto && [bi, bj] : 3x3coords) {
        model.add_constraint(xsum(3x3coords, [&](auto && p) {
                return X_vars(3 * bi + p.first, 3 * bj + p.second, v);
            }) == 1);
    }
}

for(auto && x :
    {X_vars(0, 2, 8), X_vars(0, 4, 1), X_vars(0, 8, 9), X_vars(1, 0, 6),
    X_vars(1, 2, 1), X_vars(1, 4, 9), X_vars(1, 6, 3), X_vars(1, 7, 2),
    X_vars(2, 1, 4), X_vars(2, 4, 3), X_vars(2, 5, 7), X_vars(2, 8, 5),
    X_vars(3, 1, 3), X_vars(3, 2, 5), X_vars(3, 5, 8), X_vars(3, 6, 2),
    X_vars(4, 2, 2), X_vars(4, 3, 6), X_vars(4, 4, 5), X_vars(4, 6, 8),
    X_vars(5, 2, 4), X_vars(5, 5, 1), X_vars(5, 6, 7), X_vars(5, 7, 5),
    X_vars(6, 0, 5), X_vars(6, 3, 3), X_vars(6, 4, 4), X_vars(6, 7, 8),
    X_vars(7, 1, 9), X_vars(7, 2, 7), X_vars(7, 4, 8), X_vars(7, 6, 5),
    X_vars(7, 8, 6), X_vars(8, 0, 1), X_vars(8, 4, 6), X_vars(8, 6, 9)}) {
    model.set_variable_lower_bound(x, 1);
}

model.optimize();
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