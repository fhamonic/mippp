# Worked examples

The [`examples/`](https://github.com/fhamonic/mippp/tree/main/examples) directory holds complete, runnable programs. They depend only on MIP++ (no GoogleTest; only the TSP example also uses the [MELON](https://github.com/fhamonic/melon) graph library), and each one is built around a technique from the guides. Each example is an independent project folder — `main.cpp`, a `CMakeLists.txt` and a `conanfile.py` — that can be copied as the starting point of a new program.

| Example | Model | Techniques | Read alongside |
| :--- | :--- | :--- | :--- |
| [`simple_lp/`](https://github.com/fhamonic/mippp/tree/main/examples/simple_lp) | 2-variable LP | the whole build → solve → read cycle | [A first model](getting-started/first-model.md) |
| [`nqueens/`](https://github.com/fhamonic/mippp/tree/main/examples/nqueens) | N-Queens | lambda-indexed variables, constraint families over `iota` ranges | [Variables](modeling/variables.md), [Expressions](modeling/expressions.md) |
| [`sudoku/`](https://github.com/fhamonic/mippp/tree/main/examples/sudoku) | Sudoku | 3-dimensional indexing, families over `cartesian_product` | [Expressions](modeling/expressions.md) |
| [`travelling_salesman_dfj/`](https://github.com/fhamonic/mippp/tree/main/examples/travelling_salesman_dfj) | TSP | branch-and-cut, candidate-solution callback, lazy subtour elimination | [Branch-and-cut](algorithms/branch-and-cut.md) |
| [`cutting_stock/`](https://github.com/fhamonic/mippp/tree/main/examples/cutting_stock) | Cutting stock | column generation: duals by key, `add_column`, a knapsack pricer | [Column generation](algorithms/column-generation.md) |

## Reading them in order

1. **`simple_lp`** — the model class, variable handles, `sol[x]`. Five minutes.
2. **`nqueens`** — the first *real* model: one batch of `n²` binaries with an `(row, col)` id-map, six constraint families built from `iota` and `xsum`. This is the model behind the [benchmark](https://github.com/fhamonic/mippp_nqueens), so it is also the reference for what "no modeling tax" means in practice — see [Performance](performance.md).
3. **`sudoku`** — the same ideas one dimension up, and a good template for assignment-style models: `X(i, j, v)`, families over cartesian products, hints fixed with single constraints.
4. **`travelling_salesman_dfj`** — an algorithm, not just a model: the callback receives a candidate, the code searches it for subtours, and injects the violated constraints. Needs a backend with callback support (Gurobi, CPLEX or COPT).
5. **`cutting_stock`** — the other classic: a restricted master, dual prices read back *by order id*, a dynamic-programming pricer, and columns streamed in as lazy ranges.

Every example selects its backend through the alias at the top of its `main.cpp`:

```cpp
using milp_type = highs_milp;
```

Change it to target another solver — see [Choosing a solver](solvers/index.md).

## Building and running

Each example builds on its own with Conan, once MIP++ has been exported to the cache (see [Installation](getting-started/installation.md#getting-the-headers)):

```bash
cd examples/nqueens
conan build . -of=build -b=missing -pr=<your_profile>
./build/nqueens 8      # board size as an optional argument
```

or with plain CMake against an installed MIP++ (`cmake -S . -B build -DCMAKE_PREFIX_PATH=<install_prefix>`). To build them all from the repository root instead:

```bash
ENABLE_EXAMPLES=ON conan build . -of=build -b=missing -pr=<your_profile> -c tools.build:skip_test=true

./build/examples/simple_lp/simple_lp
./build/examples/nqueens/nqueens 8
```

To start a new project, copy an example folder and rename the target in its `CMakeLists.txt`.

The corresponding solver's shared library must be discoverable at **run** time — see [Installation](getting-started/installation.md#making-solver-libraries-discoverable).

## Beyond the examples

The [test suites](https://github.com/fhamonic/mippp/tree/main/test/test_suites) are the second body of usage code, and they cover features no example does — MIP starts, model modification, reduced costs, and the [`column_manager`](https://github.com/fhamonic/mippp/blob/main/test/test_suites/column_manager.hpp) for large-scale pricing. They are written against the concepts, so each of them is itself an example of [solver-generic code](solvers/generic-code.md).
