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

The master must be an **LP** while you price: duals only exist for the relaxation, and no `*_milp` model has them. The usual endgame is to price to optimality, then hand the generated columns to a MILP model and solve once, see [An integer solution at the end](#an-integer-solution-at-the-end).

## Example: cutting stock

[`examples/cutting_stock/`](https://github.com/fhamonic/mippp/blob/main/examples/cutting_stock/main.cpp) is a complete, runnable implementation. Rolls of length 100 are cut into pieces of lengths 14, 31, 36 and 45, of which 211, 395, 610 and 97 are ordered, using as few rolls as possible. The pricing subproblem, an unbounded knapsack over the current duals, is solved by the branch and bound of the [MELON](https://github.com/fhamonic/melon) graph library, `melon::unbounded_knapsack_bnb`, so the example depends on MELON as well as on MIP++.

The master has one demand-covering row per order, kept retrievable by order id. A row starts without a term, `empty_linear_expression<V, S>` being the expression of no term over variables `V` and scalars `S`, and the columns bring their coefficients through `add_column`. The master starts from rolls cut into pieces of a single length:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:classic_master"
```

Each round then reads the duals *by order*, prices a pattern, and — if it improves — streams it in as a new column:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:classic_round"
```

Every pattern costs one roll, so "improving" means a knapsack value above `1`, which is the reduced-cost test written in the units of this model. The loop ends at the optimum of the relaxation, 452.25 rolls.

Note how the column's entries are themselves a lazy range pipeline — each item of the knapsack's solution mapped to a `(constraint, coefficient)` pair — streamed straight into the solver with no intermediate vector.

## Large-scale pricing: the column manager

When the pricing problem generates many candidate columns, more than the master should hold at once, the bookkeeping (which columns exist, which are in the master, which to evict) becomes its own subsystem. `mippp::colgen::column_manager`, in `mippp/utility/column_manager.hpp`, implements it on top of any model satisfying the concepts above:

- a column is identified by a **seed** of your choice (here, the pattern), and the manager deduplicates seeds;
- a column is either in the **pool**, priced but not in the master, or in the **master**, with its variable;
- compile-time **column properties** (reduced cost, age, …) live in each column's state, listed per state in a `property_list`, and react to the events you broadcast;
- a **predicate** chooses which columns to activate or evict at each round, and an optional **selector** narrows that choice down.

### A first example

This is the cutting stock master of [the example above](#example-cutting-stock), with its columns kept by a manager, as a complete program:

```cpp
#include <cstddef>
#include <optional>
#include <print>
#include <ranges>
#include <utility>
#include <vector>

--8<-- "test/doc_snippets/column_generation.cpp:includes"

using namespace mippp;
using namespace mippp::operators;

const int roll_length = 100;
const std::vector<int> length = {14, 31, 36, 45};  // piece lengths
const std::vector<double> demand = {211, 395, 610, 97};

// the pattern of best value at these duals, if it beats the cost of a roll
std::optional<std::vector<int>> price(const std::vector<double> & value) {
    --8<-- "test/doc_snippets/column_generation.cpp:price"
}

int main() {
    using Model = highs_lp;
    Model model;
    --8<-- "test/doc_snippets/column_generation.cpp:first_master"

    --8<-- "test/doc_snippets/column_generation.cpp:first_round"
    std::println("{} rolls", model.get_solution_value());
}
```

- The manager takes the model type, the seed type, here the pattern, then the property list of the pool columns: their reduced cost, which activation reads. Nothing reads a property of a master column here, so the master's list, the fourth template parameter, keeps its default, `property_list<>`.
- `add_pattern` receives a column's [entry](#entries) and returns the new variable: `emplace_master_column` creates the initial columns through it, and `activate_columns` every later one. `demand_constrs` iterates its rows [in key order](../modeling/expressions.md#constraint-families), the order of the lengths in a pattern, so `std::views::zip` pairs each row with the number of pieces the pattern cuts; a zero count becomes a zero coefficient, which `add_column` accepts.
- Each round solves the master, reads its duals, prices a pattern into the pool and refreshes the reduced costs of the pool columns. `activate_columns` then adds to the master the pool columns whose reduced cost is below `-1e-9`, the improving columns of a minimizing master, and returns how many it added. The loop ends on a round that adds none: the master is still as it was last solved, and the program prints the optimum of the relaxation, 452.25 rolls, after three rounds.
- **Read the master right after `solve()`.** `activate_columns` modifies the master, and from then on the getters throw `mippp::solve_required_error` until the next `solve()` ([When results can be read](../solving/solutions.md#when-results-can-be-read)).
- No column leaves the master, so this loop does not need `has_remove_variable`. Evicting columns does.

Each of the next five sections changes a few lines of this example.

### Several columns per round

A pricer often finds several improving columns at once: a dynamic program reads them off its table, a heuristic returns all it met. Activating several per round saves solves of the master, at the price of a larger one. Here `price_several(value)` returns, for each length, the best pattern that cuts at least one piece of it, if it beats the cost of a roll, each pattern once. Every pattern cuts some piece, so the best of them is the best pattern: the pricer stays exact.

The round puts them all in the pool in one call:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:several_emplace"
```

`found` counts the patterns:

- `num_inserted`, the new ones;
- `num_already_in_pool`, those an earlier round left in the pool;
- `num_already_in_master`, those in the master, which betray stale duals or a cycling process: at the master's optimum, no master column improves.

The round then activates the two improving columns of most negative reduced cost:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:several_activate"
```

- `at_most_k_best{n, cmp}` keeps the `n` best candidates according to `cmp`, which compares entries, lesser being better. The other improving columns stay in the pool, where the next round prices them again.
- The loop still stops on the first round that activates nothing. Every improving pattern the pricer returns is in the pool, and `at_most_k_best` keeps at least one candidate, so such a round is one in which the exact pricer found nothing improving.
- On the cutting stock instance, the first round prices three patterns and activates two, and the second round finds the optimum, 452.25 rolls: two rounds instead of three. The third pattern stays in the pool.

### Keeping the master small

In the first example, every activated column stays in the master, which grows each round, and so does the time of each solve. Evicting the columns that have stopped paying off keeps the master small. The master columns then need a reduced cost and an age, so the master's property list names them:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:evicting_manager"
```

Each round refreshes the reduced cost of every column, pool and master alike, with `update_columns`, activates the improving ones, then evicts the columns that have been in the master for more than one round and whose reduced cost is now above `1e-6`:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:evicting_round"
```

- **Activate, then evict.** `evict_columns` modifies the master too, so the round reads all it needs right after `solve()`; what the getters returned keeps the values of that solve. At convergence nothing is activated, so the loop leaves the master as it was solved last, and the solution read after the loop is the optimum. Evicting first would modify the master after its last solve.
- Eviction removes the variables, so `evict_columns(model, predicate)` needs `has_remove_variable`; among the LP models, `gurobi_lp`, `cplex_lp`, `highs_lp` and `clp_lp` have it. A third argument replaces that default: `remove(model, entries)` receives the entries of the evicted columns.
- An evicted column goes back to the pool, and a later round can reactivate it without the pricer finding it again, see [Pricing the pool first](#pricing-the-pool-first).
- The eviction tolerance, `1e-6`, stays above the rounding errors of reduced costs recomputed from the duals on this master. Those errors grow with the master, to about `2e-5` on the 60-city master of `travelling_salesman_colgen`, which uses `1e-4`. A basic column's reduced cost is zero up to those errors, and evicting a basic column can cost the solver its basis, see [Bases](#bases).
- On the cutting stock instance, the loop reaches 452.25 rolls with five columns in the master and one in the pool, where the first example keeps six in the master.

### Evicting nonbasic columns only

The solver's basis tells which columns are basic, with no tolerance to choose, and evicting nonbasic columns only keeps the basis whole, see [Bases](#bases). The master columns then also hold their basis status:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:nonbasic_manager"
```

Each round reads the solver's reduced costs, solution and basis right after `solve()`. `update_columns(pool_event, master_event)` sends the pool columns a `priced` computed from the duals and the master columns a `master_refreshed` read from the solver, and the eviction tests the status:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:nonbasic_round"
```

- Each column receives one event per round, so `age` still counts rounds.
- A column activated in a round has received no `master_refreshed` yet, and holds bare `basis_status::nonbasic`, the status of a column the basis does not cover. `above<age>{1}` spares it anyway.
- `positive<reduced_cost>{1e-6}` still applies, on the solver's reduced costs: a nonbasic column at zero reduced cost is as attractive as a basic one at these duals, and stays.
- On the cutting stock instance, the loop reaches 452.25 rolls with five columns in the master and one in the pool.

### Pricing the pool first

Pricing is often the expensive step, while the reduced cost of a known column is a sum over its entries. Once eviction returns columns to the pool, the pool is worth searching first: each round prices every known column from the duals, activates the improving columns of the pool, and calls the pricer only when the pool has none:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:pool_first_round"
```

- The manager and the eviction are those of [Keeping the master small](#keeping-the-master-small).
- An evicted column stays known, with its seed and its properties. `update_columns` prices it again every round, and when the duals make it improving again, `activate_columns` brings it back without the pricer.
- Without eviction, a pricer that returns one improving column per round leaves nothing in the pool, and searching it first gains nothing.
- The loop stops on a round that activates nothing: neither the pool nor the pricer had an improving column.
- After `emplace_pool_column`, `update_pool_columns` prices the new column and the others again, which changes nothing while the pool's property list holds `reduced_cost` only. An `age` there would count that round twice.
- A heuristic pricer goes between the two: called when the pool has no improving column, it hands over to the exact pricer when it finds none, so the loop still stops only on a round in which the exact pricer found nothing.
- On the cutting stock instance, the run is too short for an evicted column to come back, and ends at 452.25 rolls. The tests also run it on an instance of ten lengths, where evicted columns do come back.

### An integer solution at the end

Column generation solves the relaxation. Price-and-branch, the simplest endgame, solves the integer problem restricted to the generated columns: after the loop, a MILP model, here `highs_milp`, takes the same demand rows and every known column, of the master and of the pool, as an integer variable:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:price_and_branch"
```

- `master_columns()` and `pool_columns()` view the entries of the columns, in the order their seeds were first emplaced. `plan` keeps each pattern with its number of rolls.
- The relaxation bounds every cutting plan from below, so no plan uses fewer than ⌈452.25⌉ = 453 rolls. The MILP's optimum is a plan, so it bounds the integer optimum from above. When both meet, the plan is optimal; otherwise closing the gap takes branching on the master, which the manager does not do.

### Complete programs

[`examples/travelling_salesman_colgen/`](https://github.com/fhamonic/mippp/blob/main/examples/travelling_salesman_colgen/main.cpp) computes the Held-Karp lower bound of a travelling salesman instance by generating 1-trees. It keeps each 1-tree's cost as its data, evicts with back-off, and starts from a nearest-neighbour tour, its initial column and upper bound.

[`examples/vehicle_routing_colgen/`](https://github.com/fhamonic/mippp/blob/main/examples/vehicle_routing_colgen/main.cpp) generates the routes of a vehicle routing problem with time windows, on Solomon's R101 instance. Its pricer, a labeling algorithm for the elementary shortest path problem with resource constraints, finds many routes at once: each round hands up to 100 improving ones, including one of least reduced cost, to `emplace_pool_columns` as `(route, cost)` pairs, the seeds and their [data](#per-column-data), and activates every improving one. The run ends with [price-and-branch](#an-integer-solution-at-the-end), a MILP over the routes of the master and of the pool, whose plan the LP bound proves optimal at the default 25 customers.

The `column_manager` test suite ([`test/test_suites/column_manager.hpp`](https://github.com/fhamonic/mippp/blob/main/test/test_suites/column_manager.hpp)) runs a cutting stock loop, with and without eviction by reduced cost, on every backend with removal.

### Entries

Every callback receives a column's entry: predicates, selector comparators, the add callback, the remove callback (a range of them), and the event makers of `update_pool_columns`, `update_master_columns` and `update_columns`. An entry refers to the column's `seed`, its `data` if the manager [has one](#per-column-data), and its `state`, where `state.var` is the variable of a master column and `state.template get<P>()` reads the property `P`. It destructures as `auto && [seed, state] = entry`, or as `auto && [seed, data, state] = entry`.

### Predicates and selectors

A predicate is any function object callable with an entry and returning a `bool`. The shipped ones read a property: `below<P>{v}` and `above<P>{v}`, `negative<P>{tol}` and `positive<P>{tol}` (below `-tol` and above `tol`), `window_above<K>{v}` and `window_below<K>{v}` on the last `K` reduced costs of `reduced_cost_window<K>`, and `age_above_with_backoff` (see [Eviction](#eviction-age-and-thrashing)). `conjunction` and `disjunction` combine predicates.

None of them assumes an optimization sense. The sign is yours to state:

| Master | Activation | Eviction |
|---|---|---|
| minimizing | `negative<reduced_cost>{tol}` | `positive<reduced_cost>{tol}`, `window_above<K>{tol}` |
| maximizing | `positive<reduced_cost>{tol}` | `negative<reduced_cost>{tol}`, `window_below<K>{-tol}` |

A lambda is a predicate too, which covers tolerances relative to the duals. Build it in each round, capturing that round's duals. Here a pattern must improve by more than `1e-7` of what its pieces are worth at these duals, plus `1e-9`:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:relative_tolerance"
```

Keep the reduced cost the events carry exact. Folding a tolerance into it, as in `priced{rc + 1e-7 * worth}`, also shifts the zero reduced cost of the master's basic columns, which an eviction predicate then reads as unattractive.

The selector comes last and defaults to `all_candidates`. `at_most_k{n}` keeps the first `n` candidates, `at_most_k_random{n, gen}` draws `n` of them, and `at_most_k_best{n, cmp}` keeps the `n` best according to `cmp`, which compares entries, lesser being better, as in [Several columns per round](#several-columns-per-round).

### Per-column data

Data known when a column is priced, which is no part of its identity, goes in the fifth template parameter, `Data`. The manager stores it beside the column's state, keeps it across activations and evictions, and passes it in every entry. A column's cost is the usual case, as the cost of a route in [`examples/vehicle_routing_colgen/`](https://github.com/fhamonic/mippp/blob/main/examples/vehicle_routing_colgen/main.cpp) or the length of a 1-tree in [`examples/travelling_salesman_colgen/`](https://github.com/fhamonic/mippp/blob/main/examples/travelling_salesman_colgen/main.cpp): the pricer computes it, and the add callback and the reduced cost read it. Every pattern costs one roll here. An emplacement passes the cost as its last argument, after the add callback for the initial columns:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:data_manager"
```

Each round passes the cost of the priced pattern, `emplace_pool_column(seed, cost)`, and the reduced cost reads it:

```cpp
--8<-- "test/doc_snippets/column_generation.cpp:data_round"
```

- An emplacement constructs the `Data` from its extra arguments only when it inserts: a seed already known keeps the data it was first given.
- With a `Data`, `emplace_pool_columns(range)` takes `(seed, data)` pairs or tuples.
- Seeds that are ranges or `std::pair`/`std::tuple` hash by default through `colgen::seed_hash`, which keeps `std::hash` for the seeds that have one. A seed of your own type needs a `std::hash` specialization, or a `Hash` as the sixth template parameter.

### Order and reproducibility

The manager presents the candidates in the order their seeds were first emplaced: a column keeps its place when it moves between the pool and the master, and `purge_pool` keeps the order of the columns it spares. `all_candidates`, `at_most_k` and `at_most_k_random` keep that order, `at_most_k_best` does not; with the first three, it is the order in which the columns enter the master, which steers the solver's path. When parallel pricing collects the columns, sorting them before `emplace_pool_columns` makes runs repeatable.

### Eviction, age and thrashing

- `age` counts the events carrying a reduced cost that a column received since it entered its current state. That is one per round only if each round sends the column one such event: `update_columns` with a `priced` followed by `update_master_columns` with a `master_refreshed` ages master columns twice per round, where `update_columns(pool_event, master_event)` ages them once.
- `conjunction<above<age>, positive<reduced_cost>>{k, tol}` spares the columns that entered the master `k` rounds ago or less.
- Evicted columns often come back a few rounds later. `age_above_with_backoff{base, step}` holds when `age > base + step * times_activated`, so a column stays longer in the master each time it comes back; `below<times_activated>{n}` stops evicting a column once it has entered the master `n` times. `times_activated` must then be listed in both property lists, or it restarts at every activation.
- `purge_pool(predicate)` erases pool columns, which bounds the pool's memory.

## Performance notes

- **Build the master once and grow it.** `add_column` touches only the new column; nothing is rebuilt. Combined with in-place [model updates](../solving/updates.md), a whole run has exactly one model construction.
- **Read `get_dual_solution()` once per round**, then index it — each call fetches the whole dual vector.
- **Keep the pricing data in your own structures.** The generated columns are your objects; the model only holds handles, which stay valid across re-solves.

## Bases

A master grown in place usually re-solves from the basis the solver kept, and the columns added since enter it nonbasic. On the LP models, `get_basis()` also tells which master columns are basic, through `model_variable_basis_status_t<M>` and the `variable_status` property that `master_refreshed` updates, and `set_basis(b)`, on every LP model but `soplex_lp`, starts a solve from a basis of your choice, see [Bases and warm starts](../solving/basis.md):

- A master column reads its status from the last `master_refreshed`; until then it holds bare `basis_status::nonbasic`, the status of a column the basis does not cover, so a predicate that classifies fresh columns tests `!is<basis_status::basic>(s)`.
- Take the basis right after `solve()`, and write it with `set_basis` after `activate_columns` and `evict_columns`, in that order. Handles are recycled, so an activated column may take the handle of a column evicted in an earlier round; the basis, taken after that eviction, reads that handle nonbasic, as a new column should be. Evicting before activating in the same round would let a new column read the entry of the column whose handle it takes, basic if that column was. The basis keeps as many basic entities as rows only if no evicted column was basic. At an optimum, `get_reduced_costs()` reads zero on a basic column, so with those reduced costs, `positive<reduced_cost>{tol}` on a minimizing master and `negative<reduced_cost>{tol}` on a maximizing one evict nonbasic columns only; reduced costs recomputed from the duals carry rounding errors of either sign, and then only a tolerance above those errors keeps basic columns. [Evicting nonbasic columns only](#evicting-nonbasic-columns-only) tests the status itself.
- On Gurobi, removing a column that was basic discards the basis Gurobi keeps, and the next solve starts cold. A basis taken before that removal is one basic entity short, and Gurobi discards it too; an eviction predicate that selects nonbasic columns only, as in the previous point, avoids both.

Variable reduced costs (`get_reduced_costs`, concept `has_reduced_costs`) are available on every LP backend but `soplex_lp`, for pricing schemes that reason on the master's variables.

## Next

[The deletion filter](deletion-filter.md) — an irreducible infeasible subsystem of a model, or of constraints of your own.
