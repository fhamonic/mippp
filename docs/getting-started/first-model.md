# A first model

This page walks through the smallest possible MIP++ program — a two-variable LP — and introduces the pieces every model uses: the api/model pair, variable handles, expressions, and solution access.

```text
max   4 x1 + 5 x2
s.t.    x1        <= 4
      2 x1 +  x2  <= 9
      x1 >= 0,  x2 <= 3
```

The full program ([`examples/simple_lp.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/simple_lp.cpp)):

```cpp
#include <print>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using api_type = highs_api;
using lp_type = highs_lp;

int main() {
    api_type api;        // loads the HiGHS C API at runtime
    lp_type model(api);

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

## The include and the two `using namespace`

Each backend ships a single convenience header, `mippp/solvers/<solver>/all.hpp`, that pulls in its api and model classes. `using namespace mippp` brings in the types; `using namespace mippp::operators` is a deliberate **opt-in** for the overloaded operators (`+`, `*`, `<=`, `==`, …) and `xsum` — the algebraic syntax never  eaks into your code unless you ask for it.

Writing the backend choice as two aliases at the top of the file (`api_type` / `lp_type`) is a convention used throughout the examples: they are the only two lines to touch to re-run the program on another solver.

## The api / model split

```cpp
api_type api;        // finds and loads the solver's shared library
lp_type model(api);  // one optimization model using that library
```

The `api` object does the dynamic loading (see [Installation](installation.md#making-solver-libraries-discoverable)) and holds the function pointers of the solver's C API. Constructing it is the expensive step — do it **once** and share it: any number of models, created and destroyed freely, can reference the same api.

## Variables

```cpp
auto x1 = model.add_variable();
auto x2 = model.add_variable({.upper_bound = 3});
```

`add_variable` returns a lightweight, trivially-copyable **handle** (a strongly-typed column index — passing it around costs nothing). Options are given by designated initializers on the `variable_params` struct:

```cpp
model.add_variable({.obj_coef = 1.0, .lower_bound = -2.0, .upper_bound = 3.0});
```

Every field is optional, with one subtlety. A bare `add_variable()` creates the classic LP default: objective coefficient `0`, lower bound `0`, no upper bound. But when a `variable_params` list is given, an **omitted bound is unbounded** — `{.upper_bound = 3}` above declares `-∞ ≤ x2 ≤ 3`; write `{.lower_bound = 0, .upper_bound = 3}` if you want `0 ≤ x2 ≤ 3`. Passing `.obj_coef` at creation is equivalent to (and cheaper than) mentioning the variable in `set_objective` later.

MILP model classes (`highs_milp`, `gurobi_milp`, …) additionally provide `add_integer_variable(s)` and `add_binary_variable(s)`, as well as `set_integer` / `set_binary` / `set_continuous` to change a variable's type afterwards.

Variables are usually created in bulk with `add_variables(count, id_lambda)`, which is where MIP++'s lambda-indexing shines — that is the subject of the [next page](expressions.md).

## Objective and constraints

```cpp
model.set_maximization();
model.set_objective(4 * x1 + 5 * x2);
model.add_constraint(x1 <= 4);
model.add_constraint(2 * x1 + x2 <= 9);
```

`4 * x1 + 5 * x2` is a *lazy expression view* — no vector of terms is allocated; the terms are streamed directly into the solver when the call is made. Comparison operators `<=`, `>=`, `==` between expressions (or an expression and a scalar) produce constraints. `add_constraint` returns a constraint handle, which you can keep to query duals or modify the row later.

## Solving and reading results

```cpp
model.solve();
auto sol = model.get_solution();
double obj = model.get_solution_value();
double v1  = sol[x1];
```

`get_solution()` returns a map-like object indexed by variable handles. Backends also report the solve status:

```cpp
model.solve();
if(model.proven_optimal())         { /* ... */ }
else if(model.proven_infeasible()) { /* ... */ }
else if(model.proven_unbounded())  { /* ... */ }
```

(On Gurobi a richer `termination_reason()` is available; time limits, tolerances, and other controls are listed in the [concepts reference](../reference/concepts.md).)

LP backends supporting dual solutions expose them the same way, indexed by constraint handles:

```cpp
auto duals = model.get_dual_solution();
double y = duals[some_constraint];
```

## Compiling

With the Conan or CMake setup from the [Installation](installation.md) page, there is nothing solver-specific to do — no `-lgurobi`, no `-lhighs`:

```bash
g++-15 -std=c++26 -O3 -I<mippp>/include -I<dylib>/include simple_lp.cpp -o simple_lp
./simple_lp   # libhighs.so must be discoverable at *run* time
```

Next: [Expressions and constraints](expressions.md), where models stop being toy-sized.
