# A first model

This page walks through the smallest possible MIP++ program — a two-variable LP — and introduces the pieces every model uses: the model class, variable handles, expressions, and solution access.

```text
max   4 x1 + 5 x2
s.t.    x1        <= 4
      2 x1 +  x2  <= 9
      x1 >= 0,  x2 <= 3
```

The full program ([`examples/simple_lp/main.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/simple_lp/main.cpp)):

```cpp
#include <print>

#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;

int main() {
    lp_type model;  // loads the HiGHS C API at runtime

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

Writing the backend choice as an alias at the top of the file (`lp_type`) is a convention used throughout the examples: it is the only line to touch to re-run the program on another solver.

## Where the solver comes from

```cpp
lp_type model;  // finds and loads the solver's shared library
```

A model's default constructor obtains the backend's api object — `highs_api` here — which holds the function pointers of the solver's C API. There is one such object per loaded library file, created by the first model that needs it and living for the rest of the process (see [Installation](installation.md#making-solver-libraries-discoverable) for how the file is found); every later model of that backend shares it, so models can be created and destroyed freely, and a missing library is reported by the first model's constructor. To load a specific file instead, pass the api explicitly:

```cpp
lp_type model(highs_api::load("/path/to/libhighs.so.1.10.0"));
```

`model.native_api()` returns the api a model was built from.

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

Variables are usually created in bulk over a range of keys with `add_variables(keys)` — or, when the keys are not worth materialising, with `add_variables(count, id_lambda)` — which is where MIP++'s indexing shines — that is the subject of [Variables and index sets](../modeling/variables.md).

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
const auto & r = model.solve_status();
if(is_a<status::optimal>(r))         { /* ... */ }
else if(is_a<status::infeasible>(r)) { /* ... */ }
else if(is_a<status::unbounded>(r))  { /* ... */ }
```

`solve_status()` returns a `std::variant` of tag types organized in a hierarchy, and `is_a` tests a whole branch of it. (The full hierarchy, time limits, tolerances and the rest are covered in [Status, limits and tolerances](../solving/status-and-limits.md).)

LP backends supporting dual solutions expose them the same way, indexed by constraint handles:

```cpp
auto duals = model.get_dual_solution();
double y = duals[some_constraint];
```

More on reading results — snapshots, reduced costs, evaluating expressions at a solution — in [Solutions, duals and reduced costs](../solving/solutions.md).

## Compiling

With the Conan or CMake setup from the [Installation](installation.md) page, there is nothing solver-specific to do — no `-lgurobi`, no `-lhighs`:

```bash
g++-14 -std=c++23 -O3 -I<mippp>/include main.cpp -o simple_lp
./simple_lp   # libhighs.so must be discoverable at *run* time
```

MIP++ has no library to link against. The only platform detail is that on glibc older than 2.34 (before Debian 12 / Ubuntu 22.04) `dlopen` lives in a separate library, so add `-ldl` there; CMake and Conan consumers get this automatically.

The [`examples/simple_lp/`](https://github.com/fhamonic/mippp/tree/main/examples/simple_lp) folder packages this program with a `CMakeLists.txt` and a `conanfile.py`, ready to be copied as the start of your own project.

## Next

- [Variables and index sets](../modeling/variables.md) — where models stop being toy-sized: whole families of variables indexed by *your* coordinates.
- [Coming from gurobipy, JuMP or PuLP](coming-from.md) — if the program above looked familiar but the spelling did not.
