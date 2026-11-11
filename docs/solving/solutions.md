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

- It is a **snapshot**: the values are read from the solver when `get_solution()` is called. A later change of the model or `solve()` leaves it as it is, so call `get_solution()` again after each solve. It covers the variables that existed at the solve, those removed since included.
- It is **move-only and tied to the model**: keep it within the model's lifetime, and pass it by reference to helper functions.
- One call fetches the whole vector, so call it **once per solve**, not once per lookup.

```cpp
auto sol = model.get_solution();                       // ✓ one fetch
for(int i : items) if(sol[X(i)] > 0.5) selected.push_back(i);
```

Reading a solution is only valid when one exists — see [Status, limits and tolerances](status-and-limits.md#the-solve-status).

## When results can be read

The result getters, `get_solution_value()`, `get_solution()`, `get_dual_solution()`, `get_reduced_costs()`, `get_basis()`, `get_dual_bound()` and `get_node_count()`, read the last `solve()` until the model changes. Any change ends that: adding or removing a variable or a constraint, or setting the objective, the sense, a bound, a side, a type, a name, a basis or a MIP start. From then until the next `solve()`, `get_status()` reports `status::unknown` and these getters throw `mippp::solve_required_error`, as they do before the first `solve()`. Setting a limit, a tolerance, the LP algorithm, the verbosity or a callback is not a change, and the results stay readable.

Solvers disagree on what they keep after a change: some discard their solution, others return zeros or the values of the model as it was, and SCIP crashes. So fetch what you need right after `solve()`. What the getters returned keeps the values of that solve:

```cpp
model.solve();
const double value = model.get_solution_value();
auto sol = model.get_solution();
model.add_constraint(cut);   // value and sol still read the solve
model.get_solution();        // throws mippp::solve_required_error
```

`solve_required_error` is a `std::logic_error`, not a `solver_error`: it reports a mistake in the calling code, not a failure of the solver.

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

It works with any map indexable by variable handles, not just solutions — including a heuristic's own assignment: wrap raw storage in an `entity_mapping`, or a lambda with `maps::mapping_all` (see [Mappings](../reference/mappings.md)).

## Dual bound and node count

The MILP models but `glpk_milp` satisfy `has_dual_bound` and `has_node_count`: `get_dual_bound()` returns the bound the solve proved on the optimal objective, and `get_node_count()` the number of branch-and-bound nodes it explored:

```cpp
model.solve();
double incumbent  = model.get_solution_value();
double bound      = model.get_dual_bound();   // <= the optimum when minimizing
double gap        = std::abs(incumbent - bound) / std::abs(incumbent);
std::size_t nodes = model.get_node_count();
```

- The bound is in the objective's own sense, a lower bound when minimizing and an upper bound when maximizing, and includes the objective offset.
- After `optimal`, it lies within the [optimality tolerance](status-and-limits.md#tolerances) of the incumbent. After a stop, a limit or an interruption, it is the bound the search had reached, unspecified if the stop came before the root relaxation was solved.
- On a model without integer variables the solve is an LP: the bound is the LP optimum and the node count 0.
- Each solver counts nodes its own way: a search closed at the root reports 0 on some solvers and 1 on others.
- Both are defined after a solve that reports `optimal` or a stop, until the model is modified, and unspecified at other times, as `get_solution_value()` is.

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
    A `*_milp` model has no `get_dual_solution`, even with every variable continuous: the LP model classes and `highs_qp` have it. To price on the relaxation, build the LP model class; [column generation](../algorithms/column-generation.md#an-integer-solution-at-the-end) hands the generated columns to a MILP model at the end.

## Reduced costs

On backends satisfying `has_reduced_costs`, reduced costs are indexed by variable handles:

```cpp
auto rc = model.get_reduced_costs();
if(rc[x] > tolerance) { /* x is nonbasic at its lower bound */ }
```

They are the natural companion of duals for pricing schemes that reason on the master's *variables* rather than its rows, and for sensitivity analysis of an LP.

## Bases

On the LP models, `get_basis()` returns the basis an LP solve ended with, which variables and rows are basic and at which bound the others stand, and `set_basis(b)`, on every LP model but `soplex_lp`, starts the next solve from a basis: see [Bases and warm starts](basis.md).

## Next

[Bases and warm starts](basis.md) — reading the basis of a solve, and starting a solve from one.
