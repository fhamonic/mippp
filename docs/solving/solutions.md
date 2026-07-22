# Solutions, duals and reduced costs

MIP++ returns solution information as **mappings indexed by handles**, never as flat arrays you have to index yourself. That is what lets results be read back in the coordinates the model was written in — `sol[X(i, j)]`, `duals[demand(o)]` — with no dictionary and no bookkeeping of column numbers.

## Primal values

```cpp
model.solve();
double obj = model.get_solution_value();
auto   sol = model.get_solution();

double v = sol[x1];              // by handle
double f = sol[X(i, j)];         // by your own coordinates
```

Three properties of the returned object are worth knowing:

- It is a **snapshot**: the values are read from the solver when `get_solution()` is called. A later `solve()` does not refresh it — call `get_solution()` again.
- It is **move-only and tied to the model**: keep it within the model's lifetime, and pass it by reference to helper functions.
- One call fetches the whole vector, so call it **once per solve**, not once per lookup.

```cpp
auto sol = model.get_solution();                       // ✓ one fetch
for(int i : items) if(sol[X(i)] > 0.5) selected.push_back(i);
```

Reading a solution is only valid when one exists — see [Status, limits and tolerances](status-and-limits.md#the-coarse-status).

### Rounding integers

Solvers return integer variables as floating-point values within the integrality tolerance, so compare against a midpoint rather than testing equality:

```cpp
const bool taken = sol[X(i)] > 0.5;        // ✓
const bool bad   = sol[X(i)] == 1.0;       // ✗ brittle
```

### Evaluating expressions at a solution

`evaluate(expr, sol)` folds any linear expression against the solution — useful to compute a quantity that was never a model row (a separate cost component, a violation, a KPI):

```cpp
double transport_cost = evaluate(
    xsum(arcs, [&](auto a) { return cost[a] * F(a); }), sol);
```

It works with any map indexable by variable handles, not just solutions — including a heuristic's own assignment: wrap raw storage in an `entity_mapping`, or a lambda with `views::mapping_all` (see [Mappings](../reference/mappings.md)).

## Dual values

On backends satisfying `has_dual_solution`, duals are indexed by **constraint handles**:

```cpp
auto duals = model.get_dual_solution();
double y = duals[capacity_row];
```

Combined with [constraint families](../modeling/expressions.md#constraint-families), which stay callable by key, this gives dual prices directly in the problem's coordinates — the interface decomposition algorithms need:

```cpp
auto demand = model.add_constraints(orders, [&](int o) { /* ... */ });
// ... solve ...
auto duals = model.get_dual_solution();
for(int o : orders) price[o] = duals[demand(o)];
```

That pattern is the core of [Column generation](../algorithms/column-generation.md).

!!! note "Duals of a MILP"
    A `*_milp` model with integer variables has no meaningful dual solution. To price on the relaxation, either build the LP model class, or relax the variables with `set_continuous(v)`, solve, read duals, and restore with `set_integer(v)`.

## Reduced costs

On backends satisfying `has_reduced_costs`, reduced costs are indexed by variable handles:

```cpp
auto rc = model.get_reduced_costs();
if(rc[x] > tolerance) { /* x is nonbasic at its lower bound */ }
```

They are the natural companion of duals for pricing schemes that reason on the master's *variables* rather than its rows, and for sensitivity analysis of an LP.

## Sensitivity and warm starts

LP basis access (`has_lp_basis`) and basis warm starts (`has_lp_basis_warm_start`) are specified in [`model_concepts.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/model_concepts.hpp) — including a full `basis_status` hierarchy — but **no backend implements them yet**; they are the top item on the [roadmap](https://github.com/fhamonic/mippp#roadmap). Until then, a re-solve after a model change starts from whatever the backend itself keeps internally (see [Re-solving and model updates](updates.md)).

## Next

[Re-solving and model updates](updates.md) — changing a model between solves without rebuilding it.
