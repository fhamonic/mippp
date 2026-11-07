# Worked examples

The [`examples/`](https://github.com/fhamonic/mippp/tree/main/examples) directory holds complete, runnable programs. Each one is built around a technique from the guides and depends only on MIP++, without GoogleTest; `cutting_stock` and the two TSP examples also use the [MELON](https://github.com/fhamonic/melon) graph library. Each example is an independent project folder — `main.cpp`, a `CMakeLists.txt` and a `conanfile.py` — that can be copied as the starting point of a new program.

| Example | Model | Techniques | Read alongside |
| :--- | :--- | :--- | :--- |
| [`simple_lp/`](https://github.com/fhamonic/mippp/tree/main/examples/simple_lp) | 2-variable LP | the whole build → solve → read cycle | [A first model](getting-started/first-model.md) |
| [`nqueens/`](https://github.com/fhamonic/mippp/tree/main/examples/nqueens) | N-Queens | lambda-indexed variables, constraint families over `iota` ranges | [Variables](modeling/variables.md), [Expressions](modeling/expressions.md) |
| [`sudoku/`](https://github.com/fhamonic/mippp/tree/main/examples/sudoku) | Sudoku | 3-dimensional indexing, families over `cartesian_product` | [Expressions](modeling/expressions.md) |
| [`travelling_salesman_dfj/`](https://github.com/fhamonic/mippp/tree/main/examples/travelling_salesman_dfj) | TSP | branch-and-cut, candidate-solution callback, lazy subtour elimination | [Branch-and-cut](algorithms/branch-and-cut.md) |
| [`cutting_stock/`](https://github.com/fhamonic/mippp/tree/main/examples/cutting_stock) | Cutting stock | column generation: duals by key, `add_column`, a knapsack pricer | [Column generation](algorithms/column-generation.md) |
| [`travelling_salesman_colgen/`](https://github.com/fhamonic/mippp/tree/main/examples/travelling_salesman_colgen) | TSP, Held-Karp bound | the `column_manager`: 1-trees as seeds with their length as data, reduced costs from the duals, eviction with back-off | [Column generation](algorithms/column-generation.md) |
| [`vehicle_routing_colgen/`](https://github.com/fhamonic/mippp/tree/main/examples/vehicle_routing_colgen) | VRP with time windows, Solomon's R101 | the `column_manager` fed by a labeling pricer: up to 100 routes per round through `emplace_pool_columns`, with their costs as data, then price-and-branch | [Column generation](algorithms/column-generation.md) |
| [`infeasible_transportation/`](https://github.com/fhamonic/mippp/tree/main/examples/infeasible_transportation) | Transportation | an IIS from the deletion filter, members printed by name with their sides | [Diagnosing infeasibility](solving/infeasibility.md) |

## Reading them in order

1. **`simple_lp`** — the model class, variable handles, `sol[x]`. Five minutes.
2. **`nqueens`** — the first *real* model: one batch of `n²` binaries with an `(row, col)` id-map, six constraint families built from `iota` and `xsum`. This is the model behind the [benchmark](https://github.com/fhamonic/mippp_nqueens), so it is also the reference for what "no modeling tax" means in practice — see [Performance](performance.md).
3. **`sudoku`** — the same ideas one dimension up, and a good template for assignment-style models: `X(i, j, v)`, families over cartesian products, hints fixed with single constraints.
4. **`travelling_salesman_dfj`** — an algorithm, not just a model: the callback receives a candidate, the code searches it for subtours, and injects the violated constraints. Needs lazy constraints in the candidate-solution callback, which Gurobi, CPLEX and COPT provide.
5. **`cutting_stock`** — the other classic: a restricted master, dual prices read back *by order id*, MELON's unbounded knapsack branch and bound as the pricer, and columns streamed in as lazy ranges.
6. **`travelling_salesman_colgen`** — the [`column_manager`](algorithms/column-generation.md) on columns that are combinatorial objects: 1-trees, each a spanning tree of all cities but one plus two edges at that one. The master mixes them so that every city has degree 2, and its optimum is the Held-Karp bound of the TSP. Each round prices the shortest 1-tree under the reduced edge lengths with MELON's Kruskal, recomputes the reduced cost of every known 1-tree from the duals, activates the improving ones, and evicts those that have been in the master long enough and are now unattractive, waiting longer each time one comes back. The number of cities is an optional argument: the rounds grow quickly with it, as column generation tails off. Needs an LP model with `remove_variable`: `highs_lp`, `clp_lp`, `gurobi_lp` or `cplex_lp`.
7. **`vehicle_routing_colgen`** — the [`column_manager`](algorithms/column-generation.md) fed by a pricer that finds many columns at once. A column is a vehicle's route through customers of Solomon's R101 instance, within their time windows and the vehicle's capacity, and the master covers every customer. Each round prices routes by labeling, the classic algorithm for the elementary shortest path problem with resource constraints, emplaces up to 100 improving ones, including the most negative, in the pool with their costs in one call, and activates the improving ones. A MILP over every generated route then partitions the customers among them: price-and-branch, whose plan the LP bound proves optimal at the default 25 customers. The number of customers, up to 100, is an optional argument. Needs an LP model with dual values, and a MILP model with `add_column`: every one but `scip_milp`.
8. **`infeasible_transportation`** — a model with no solution, although its depots hold more than the stores order. The deletion filter narrows it down to four figures, printed through the program's own names with the side of each. Raising one of them, a route's trucks, repairs the plan. With its alias changed, the same program runs on every model class; [Diagnosing infeasibility](solving/infeasibility.md) covers the native routine.

Every example selects its backend through the alias at the top of its `main.cpp`, two of them in `vehicle_routing_colgen`:

```cpp
using milp_type = highs_milp;
```

Change it, and the `mippp/solvers/<solver>/all.hpp` include, to target another solver — see [Choosing a solver](solvers/index.md).

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

The [test suites](https://github.com/fhamonic/mippp/tree/main/test/test_suites) are the second body of usage code, and they cover features no example does, such as MIP starts and the solver's reduced costs (`get_reduced_costs`). They are written against the concepts, so each of them is itself an example of [solver-generic code](solvers/generic-code.md).
