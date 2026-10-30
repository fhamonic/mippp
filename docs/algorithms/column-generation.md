# Column generation

Column generation needs three things from a modeling layer: **dual values** from the restricted master problem, a way to **add a column** to a live model, and enough speed that model manipulation doesn't dominate the pricing loop.
MIP++ provides the first two as first-class operations (concepts `has_dual_solution` and `has_column_generation`), and its [zero-copy expression system](../modeling/expressions.md#why-its-fast-expressions-are-views) takes care of the third.

## The classic loop

```cpp
for(;;) {
    model.solve();
    auto duals = model.get_dual_solution();
    auto column = solve_pricing(duals);            // your subproblem
    if(reduced_cost(column) >= -epsilon) break;    // no improving column left
    model.add_column(column, {.obj_coef = 1, .lower_bound = 0});
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

[`examples/cutting_stock/`](https://github.com/fhamonic/mippp/blob/main/examples/cutting_stock/main.cpp) is a complete, runnable implementation — the pricing subproblem (an unbounded knapsack over the current duals) is a small dynamic program, so the example depends only on MIP++.

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

When the pricing problem generates many candidate columns, more than the master should hold at once, the bookkeeping (which columns exist, which are in the master, which to evict) becomes its own subsystem. `mippp::colgen::column_manager`, in `mippp/utility/column_manager.hpp`, implements it on top of any model satisfying the concepts above:

- a column is identified by a **seed** of your choice (here, the pattern), and the manager deduplicates seeds;
- a column is either in the **pool**, priced but not in the master, or in the **master**, with its variable;
- compile-time **column properties** (reduced cost, age, …) live in each column's state, listed per state in a `property_list`, and react to the events you broadcast;
- a **predicate** chooses which columns to activate or evict at each round, and an optional **selector** narrows that choice down.

### A round

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:includes"
```

The manager is declared with its two property lists, one for each state:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:manager"
```

The rows exist before any column, so the initial columns go through the same add callback as every later one. The callback receives the column's [entry](#entries) and returns the new variable:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:rows"
```

Each round prices a pattern into the pool, refreshes every column's reduced cost, activates the improving columns and evicts the unattractive ones:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:round"
```

(`price` returns the pattern of best knapsack value at these duals, if it beats the cost of a roll.)

- **Read the master right after `solve()`.** Take the solution, the duals, the reduced costs and the basis before `activate_columns` or `evict_columns` modifies the master: after a modification they throw `mippp::solve_required_error` until the next `solve()`, while what they returned keeps the values of the last solve ([When results can be read](../solving/solutions.md#when-results-can-be-read)).
- **Activate, then evict.** At convergence nothing is activated, so the loop leaves the master as it was solved last, and the solution read after the loop is the optimum. Evicting first would modify the master after its last solve.
- `emplace_pool_column` reports a seed already known; `emplace_pool_columns(range)` counts the seeds it inserted, those already in the pool, and those already in the master, which betrays stale duals or a cycling process.
- Eviction removes the variables, so `evict_columns(model, predicate)` needs `has_remove_variable`, which, among the LP models, `gurobi_lp`, `cplex_lp`, `highs_lp` and `clp_lp` model. A third argument replaces that default: `remove(model, entries)` receives the entries of the evicted columns. An evicted column goes back to the pool, and a later round reactivates it without pricing it again.

### Entries

Every callback receives a column's entry: predicates, selector comparators, the add callback, the remove callback (a range of them), and the event makers of `update_pool_columns`, `update_master_columns` and `update_columns`. An entry refers to the column's `seed`, its `data` if the manager [has one](#per-column-data), and its `state`, where `state.var` is the variable of a master column and `state.template get<P>()` reads the property `P`. It destructures as `auto && [seed, state] = entry`, or as `auto && [seed, data, state] = entry`.

### Predicates and selectors

A predicate is any function object callable with an entry and returning a `bool`. The shipped ones read a property: `below<P>{v}` and `above<P>{v}`, `negative<P>{tol}` and `positive<P>{tol}` (below `-tol` and above `tol`), `window_above<K>{v}` and `window_below<K>{v}` on the last `K` reduced costs of `reduced_cost_window<K>`, and `age_above_with_backoff` (see [Eviction](#eviction-age-and-thrashing)). `conjunction` and `disjunction` combine predicates.

None of them assumes an optimization sense. The sign is yours to state:

| Master | Activation | Eviction |
|---|---|---|
| minimizing | `negative<reduced_cost>{tol}` | `positive<reduced_cost>{tol}`, `window_above<K>{tol}` |
| maximizing | `positive<reduced_cost>{tol}` | `negative<reduced_cost>{tol}`, `window_below<K>{-tol}` |

A lambda is a predicate too, which covers tolerances relative to the duals. Build it in each round, capturing that round's duals:

```cpp
auto improving = [&](const auto & entry) {   // a maximizing master
    return entry.state.template get<colgen::reduced_cost>()
           > 1e-7 * duals[target_row(entry.seed)] + 1e-6;
};
columns.activate_columns(model, improving, add_column);
```

Keep the reduced cost the events carry exact. Folding a tolerance into it, as in `priced{rc - 1e-7 * dual}`, also shifts the zero reduced cost of the master's basic columns, which the eviction predicate then reads as unattractive.

The selector comes last and defaults to `all_candidates`. `at_most_k{n}` keeps the first `n` candidates, `at_most_k_random{n, gen}` draws `n` of them, and `at_most_k_best{n, cmp}` keeps the `n` best according to `cmp`, which compares entries, lesser being better.

### Per-column data

Data known when a column is priced, which is no part of its identity, goes in the fifth template parameter, `Data`. The manager stores it beside the column's state, keeps it across activations and evictions, and passes it in every entry. For instance, with a column whose contribution enters its reduced cost:

```cpp
struct contribution { double value; };
colgen::column_manager<Model, tree, colgen::property_list<colgen::reduced_cost>,
                       colgen::property_list<colgen::reduced_cost>, contribution>
    columns;

columns.emplace_pool_column(std::move(t), c);      // constructs the Data from c
columns.update_columns([&](const auto & entry) {
    return colgen::priced{reduced_cost(entry.seed, entry.data.value)};
});
```

- An emplacement constructs the `Data` from its extra arguments only when it inserts: a seed already known keeps the data it was first given.
- With a `Data`, `emplace_pool_columns(range)` takes `(seed, data)` pairs or tuples.
- Seeds that are ranges or `std::pair`/`std::tuple` hash by default through `colgen::seed_hash`, which keeps `std::hash` for the seeds that have one. A seed of your own type needs a `std::hash` specialization, or a `Hash` as the sixth template parameter.

### Order and reproducibility

The manager presents the candidates in the order their seeds were first emplaced: a column keeps its place when it moves between the pool and the master, and `purge_pool` keeps the order of the columns it spares. `all_candidates` and `at_most_k` keep that order, so it is the order in which the columns enter the master, which steers the solver's path. When parallel pricing collects the columns, sorting them before `emplace_pool_columns` makes runs repeatable.

### Eviction, age and thrashing

- `age` counts the events carrying a reduced cost that a column received since it entered its current state. That is one per round only if each round sends the column one such event: `update_columns(price)` followed by `update_master_columns` with a `master_refreshed` ages master columns twice per round.
- `conjunction<above<age>, positive<reduced_cost>>{k, tol}` spares the columns that entered the master less than `k` rounds ago.
- Evicted columns often come back a few rounds later. `age_above_with_backoff{base, step}` holds when `age > base + step * times_activated`, so a column stays longer in the master each time it comes back; `below<times_activated>{n}` stops evicting a column once it has entered the master `n` times. `times_activated` must then be listed in both property lists, or it restarts at every activation.
- `purge_pool(predicate)` erases pool columns, which bounds the pool's memory.

The `column_manager` test suite ([`test/test_suites/column_manager.hpp`](https://github.com/fhamonic/mippp/blob/main/test/test_suites/column_manager.hpp)) runs this loop on every backend with removal.

## Performance notes

- **Build the master once and grow it.** `add_column` touches only the new column; nothing is rebuilt. Combined with in-place [model updates](../solving/updates.md), a whole run has exactly one model construction.
- **Read `get_dual_solution()` once per round**, then index it — each call fetches the whole dual vector.
- **Keep the pricing data in your own structures.** The generated columns are your objects; the model only holds handles, which stay valid across re-solves.

## Bases

A master grown in place usually re-solves from the basis the solver kept, and the columns added since enter it nonbasic. On the LP models, `get_basis()` also tells which master columns are basic, through `model_variable_basis_status_t<M>` and the `variable_status` property that `master_refreshed` updates, and `set_basis(b)`, on every LP model but `soplex_lp`, starts a solve from a basis of your choice, see [Bases and warm starts](../solving/basis.md):

- A master column reads its status from the last `master_refreshed`; until then it holds bare `basis_status::nonbasic`, the status of a column the basis does not cover, so a predicate that classifies fresh columns tests `!is<basis_status::basic>(s)`.
- Take the basis right after `solve()`, and write it with `set_basis` after `activate_columns` and `evict_columns`, in that order. Handles are recycled, so an activated column may take the handle of a column evicted in an earlier round; the basis, taken after that eviction, reads that handle nonbasic, as a new column should be. Evicting before activating in the same round would let a new column read the entry of the column whose handle it takes, basic if that column was. The basis keeps as many basic entities as rows only if no evicted column was basic. At an optimum, `get_reduced_costs()` reads zero on a basic column, so with those reduced costs, `positive<reduced_cost>{tol}` on a minimizing master and `negative<reduced_cost>{tol}` on a maximizing one evict nonbasic columns only; reduced costs recomputed from the duals carry rounding errors of either sign, and then only a tolerance above those errors keeps basic columns.
- On Gurobi, removing a column that was basic discards the basis Gurobi keeps, and the next solve starts cold. A basis taken before that removal is one basic entity short, and Gurobi discards it too; an eviction predicate that selects nonbasic columns only, as in the previous point, avoids both.

Variable reduced costs (`get_reduced_costs`, concept `has_reduced_costs`) are available on every LP backend but `soplex_lp`, for pricing schemes that reason on the master's variables.

## Next

[The deletion filter](deletion-filter.md) — an irreducible infeasible subsystem of a model, or of constraints of your own.
