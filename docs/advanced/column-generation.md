# Column generation

Column generation needs three things from a modeling layer: **dual values** from the restricted master problem, a way to **add a column** to a live model, and enough speed that model manipulation doesn't dominate the pricing loop.
MIP++ provides the first two as first-class operations (concepts `has_dual_solution` and `has_add_column`), and its [zero-copy expression system](../getting-started/expressions.md#why-its-fast-expressions-are-views) takes care of the third.

## The classic loop

```cpp
for(;;) {
    model.solve();
    auto duals = model.get_dual_solution();
    auto pattern = solve_pricing(duals);            // your subproblem
    if(reduced_cost(pattern) >= -epsilon) break;    // no improving column left
    model.add_column(columns_of(pattern), {.obj_coef = 1});
}
```

- `get_dual_solution()` returns a map indexed by **constraint handles** —
  and since `add_constraints` returns a range whose constraints are
  [retrievable by key](../getting-started/expressions.md#constraint-families),
  you read duals in your problem's own coordinates.
- `add_column(entries, params)` creates a new variable from a range (or
  initializer list) of `(constraint, coefficient)` pairs — the transpose view
  of `add_constraint` — with the usual `variable_params` for its objective
  coefficient and bounds. It returns the new variable handle.
- `remove_variable(s)` (concept `has_remove_variable`) lets you drop columns
  again if you manage the master's size explicitly.

## Example: cutting stock

[`examples/cutting_stock.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/cutting_stock.cpp) is a complete, runnable implementation — the pricing subproblem (an unbounded knapsack over the current duals) is a small dynamic program, so the example depends only on MIP++. The essential parts:

```cpp
// Master: one demand-covering constraint per order, retrievable by order.
auto demand_constrs = model.add_constraints(orders, [&](int o) {
    return xsum(std::views::iota(0, int(patterns.size())),
                [&, o](int p) { return patterns[p][o] * vars[p]; })
           >= demand[o];
});

for(;;) {
    model.solve();
    auto duals = model.get_dual_solution();

    for(int o : orders) value[o] = duals[demand_constrs(o)];  // duals by key

    auto [best, counts] = price(length, value, roll_length);
    if(best <= 1.0 + 1e-9) break;  // no column with negative reduced cost

    // New pattern: a column with entry counts[o] in each demand row it serves.
    vars.push_back(model.add_column(
        orders | std::views::filter([&](int o) { return counts[o] > 0; })
               | std::views::transform([&](int o) {
                     return std::make_pair(demand_constrs(o),
                                           double(counts[o]));
                 }),
        {.obj_coef = 1, .lower_bound = 0}));
    patterns.push_back(std::move(counts));
}
```

Note how the column's entries are themselves a lazy range pipeline — filter the orders the pattern serves, map each to a `(constraint, coefficient)` pair — streamed straight into the solver.

## Large-scale pricing: the column manager

When the pricing problem generates many candidate columns — more than the master should hold at once — the bookkeeping (which columns exist, which are active in the master, which to evict) becomes its own subsystem. The `mippp::column_generation::column_manager` (in `mippp/utility/column_manager.hpp`) implements it on top of any model satisfying the concepts above:

- columns are identified by a user-chosen **seed** type (e.g. the pattern), deduplicated in a pool;
- compile-time **column properties** (reduced-cost windows, age, …) are attached to pooled and in-master columns via `property_list`;
- pluggable **activation and eviction strategies** decide which columns move between the pool and the master at each round, and the manager reports diagnostics such as columns regenerated while already in the master — a telltale sign of stale duals or cycling.

The `column_manager` test suite ([`test/test_suites/column_manager.hpp`](https://github.com/fhamonic/mippp/blob/main/test/test_suites/column_manager.hpp)) shows complete usage until a dedicated guide lands here.

## What's next

LP-basis warm starts (`has_lp_basis_warm_start`) — so each master re-solve starts from the previous basis instead of from scratch — are on the [roadmap](https://github.com/fhamonic/mippp#roadmap). Variable reduced costs (`get_reduced_costs`, concept `has_reduced_costs`) are already available on the LP backends, for pricing schemes that reason on the master's variables.
