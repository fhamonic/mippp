# Worked examples

The [`examples/`](https://github.com/fhamonic/mippp/tree/main/examples) directory holds complete, runnable programs. They depend only on MIP++ (no GoogleTest, and only the TSP example needs a graph library), and each one is built around a technique from the guides.

| Example | Model | Techniques | Read alongside |
| :--- | :--- | :--- | :--- |
| [`simple_lp.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/simple_lp.cpp) | 2-variable LP | the whole build → solve → read cycle | [A first model](getting-started/first-model.md) |
| [`nqueens.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/nqueens.cpp) | N-Queens | lambda-indexed variables, constraint families over `iota` ranges | [Variables](modeling/variables.md), [Expressions](modeling/expressions.md) |
| [`sudoku.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/sudoku.cpp) | Sudoku | 3-dimensional indexing, families over `cartesian_product` | [Expressions](modeling/expressions.md) |
| [`tsp_lazy_constraints.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/tsp_lazy_constraints.cpp) | TSP | branch-and-cut, candidate-solution callback, lazy subtour elimination | [Branch-and-cut](algorithms/branch-and-cut.md) |
| [`cutting_stock.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/cutting_stock.cpp) | Cutting stock | column generation: duals by key, `add_column`, a knapsack pricer | [Column generation](algorithms/column-generation.md) |

## Reading them in order

1. **`simple_lp`** — the api/model split, variable handles, `sol[x]`. Five minutes.
2. **`nqueens`** — the first *real* model: one batch of `n²` binaries with an `(row, col)` id-map, six constraint families built from `iota` and `xsum`. This is the model behind the [benchmark](https://github.com/fhamonic/mippp_nqueens), so it is also the reference for what "no modeling tax" means in practice — see [Performance](performance.md).
3. **`sudoku`** — the same ideas one dimension up, and a good template for assignment-style models: `X(i, j, v)`, families over cartesian products, hints fixed with single constraints.
4. **`tsp_lazy_constraints`** — an algorithm, not just a model: the callback receives a candidate, the code searches it for subtours, and injects the violated constraints. Needs a backend with callback support (Gurobi, CPLEX or COPT).
5. **`cutting_stock`** — the other classic: a restricted master, dual prices read back *by order id*, a dynamic-programming pricer, and columns streamed in as lazy ranges.

Every example selects its backend through the two aliases at the top of the file:

```cpp
using api_type = highs_api;
using milp_type = highs_milp;
```

Change them to target another solver — see [Choosing a solver](solvers/index.md).

## Building and running

```bash
cmake -S . -B build -DENABLE_EXAMPLES=ON
cmake --build build

./build/examples/example_simple_lp
./build/examples/example_nqueens 8      # board size as an optional argument
```

The corresponding solver's shared library must be discoverable at **run** time — see [Installation](getting-started/installation.md#making-solver-libraries-discoverable).

## Beyond the examples

The [test suites](https://github.com/fhamonic/mippp/tree/main/test/test_suites) are the second body of usage code, and they cover features no example does — MIP starts, model modification, reduced costs, and the [`column_manager`](https://github.com/fhamonic/mippp/blob/main/test/test_suites/column_manager.hpp) for large-scale pricing. They are written against the concepts, so each of them is itself an example of [solver-generic code](solvers/generic-code.md).
