# Column generation

Column generation needs three things from a modeling layer: **dual values** from the restricted master problem, a way to **add a column** to a live model, and enough speed that model manipulation doesn't dominate the pricing loop.
MIP++ provides the first two as first-class operations (concepts `has_dual_solution` and `has_add_column`), and its [zero-copy expression system](../modeling/expressions.md#why-its-fast-expressions-are-views) takes care of the third.

## The classic loop

```cpp
for(;;) {
    model.solve();
    auto duals = model.get_dual_solution();
    auto column = solve_pricing(duals);            // your subproblem
    if(reduced_cost(column) >= -epsilon) break;    // no improving column left
    model.add_column(column, {.obj_coef = 1});
}
```

- `get_dual_solution()` returns a map indexed by **constraint handles** —
  and since `add_constraints` returns a range whose constraints are
  [retrievable by key](../modeling/expressions.md#constraint-families),
  you read duals in your problem's own coordinates.
- `add_column(entries, params)` creates a new variable from a range (or
  initializer list) of `(constraint, coefficient)` pairs — the transpose view
  of `add_constraint` — with the usual `variable_params` for its objective
  coefficient and bounds. It returns the new variable handle.
- `remove_variable(s)` (concept `has_remove_variable`) lets you drop columns
  again if you manage the master's size explicitly.

The master must be an **LP** while you price: duals only exist for the relaxation. The usual endgame is to price to optimality, then set the columns integer (`set_integer`) or hand the generated set to a MILP model and solve once — see [the note on MILP duals](../solving/solutions.md#dual-values).

## Example: cutting stock

[`examples/cutting_stock.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/cutting_stock.cpp) is a complete, runnable implementation — the pricing subproblem (an unbounded knapsack over the current duals) is a small dynamic program, so the example depends only on MIP++.

The master has one demand-covering row per order, kept retrievable by order id:

```cpp
auto demand_constrs = model.add_constraints(orders, [&](int o) {
    return xsum(std::views::iota(0, int(patterns.size())),
                [&, o](int p) { return patterns[p][o] * vars[p]; })
           >= demand[o];
});
```

Each round then reads the duals *by order*, prices a pattern, and — if it improves — streams it in as a new column:

```cpp
for(;;) {
    model.solve();
    auto duals = model.get_dual_solution();

    std::vector<double> value(m);
    for(int o : orders) value[o] = duals[demand_constrs(o)];

    auto [best, counts] = price(length, value, roll_length);
    if(best <= 1.0 + 1e-9) break;   // no pattern beats the cost of a new roll

    auto new_var = model.add_column(
        orders | std::views::filter([&](int o) { return counts[o] > 0; })
               | std::views::transform([&](int o) {
                     return std::make_pair(demand_constrs(o),
                                           double(counts[o]));
                 }),
        {.obj_coef = 1, .lower_bound = 0});

    patterns.push_back(std::move(counts));
    vars.push_back(new_var);
}
```

(The example's `price` returns the knapsack value of the best pattern; since every pattern costs one roll, "improving" means a value above `1`, which is the reduced-cost test written in the units of this model.)

Note how the column's entries are themselves a lazy range pipeline — filter the orders the pattern serves, map each to a `(constraint, coefficient)` pair — streamed straight into the solver with no intermediate vector.

## Large-scale pricing: the column manager

When the pricing problem generates many candidate columns — more than the master should hold at once — the bookkeeping (which columns exist, which are active in the master, which to evict) becomes its own subsystem. The `mippp::column_generation::column_manager` (in `mippp/utility/column_manager.hpp`) implements it on top of any model satisfying the concepts above:

- columns are identified by a user-chosen **seed** type (e.g. the pattern), deduplicated in a pool;
- compile-time **column properties** (reduced-cost windows, age, …) are attached to pooled and in-master columns via `property_list`;
- pluggable **activation and eviction strategies** decide which columns move between the pool and the master at each round, and the manager reports diagnostics such as columns regenerated while already in the master — a telltale sign of stale duals or cycling.

The `column_manager` test suite ([`test/test_suites/column_manager.hpp`](https://github.com/fhamonic/mippp/blob/main/test/test_suites/column_manager.hpp)) shows complete usage until a dedicated guide lands here.

## Performance notes

- **Build the master once and grow it.** `add_column` touches only the new column; nothing is rebuilt. Combined with in-place [model updates](../solving/updates.md), a whole run has exactly one model construction.
- **Read `get_dual_solution()` once per round**, then index it — each call fetches the whole dual vector.
- **Keep the pricing data in your own structures.** The generated columns are your objects; the model only holds handles, which stay valid across re-solves.

## What's next

LP-basis warm starts (`has_lp_basis_warm_start`) — so each master re-solve starts from the previous basis instead of from scratch — are specified but not yet implemented on any backend; they are the top item on the [roadmap](https://github.com/fhamonic/mippp#roadmap). Variable reduced costs (`get_reduced_costs`, concept `has_reduced_costs`) are already available on the LP backends, for pricing schemes that reason on the master's variables.

## Next

[Choosing a solver](../solvers/index.md) — which backends provide duals, `add_column`, and the rest.
