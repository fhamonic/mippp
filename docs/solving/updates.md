# Re-solving and model updates

Most research code solves the same model many times: a parameter sweep, a branching or diving heuristic, a Lagrangian loop, a master problem that grows. MIP++ models are live solver objects, so all of that is done by **modifying in place and calling `solve()` again** — never by rebuilding from scratch.

This page collects the modification operations and the guarantees around handles.

## What stays valid across a solve

- **Variable and constraint handles.** They identify entities of the model, not of a solve, and remain valid until the entity is removed.
- **Limits, tolerances and the sense** set on the model.
- **Solution objects are snapshots** and are *not* refreshed — fetch them again after each solve ([Solutions](solutions.md)).

## Bounds

```cpp
model.set_variable_lower_bound(x, lb);   // has_modifiable_variables_bounds
model.set_variable_upper_bound(x, ub);

double lb0 = model.get_variable_lower_bound(x);   // has_readable_variables_bounds
```

Changing bounds is the cheapest possible model update — the matrix is untouched — which makes it the right tool for:

- **fixing variables** in a diving or rounding heuristic (`lb = ub = value`), then restoring;
- **manual branching**, one child per bound tightening;
- **scenario loops** where only capacities change.

## Objective

`set_objective` replaces the objective, `add_objective` increments it, and `set_objective_coefficient` edits a single column — see [Objectives](../modeling/objectives.md#incremental-changes). Switching the sense (`set_maximization` / `set_minimization`) needs no other change.

## Constraint rows

On backends satisfying the corresponding concepts, a row can be edited in place:

```cpp
model.set_constraint_rhs(row, new_b);                  // has_modifiable_constraint_rhs
model.set_constraint_sense(row, constraint_sense::greater_equal);
model.set_constraint_lhs(row, entries);                // range of (variable, coefficient)
```

and read back:

```cpp
auto c    = model.get_constraint(row);        // a linear_constraint
auto lhs  = model.get_constraint_lhs(row);    // range of (variable, coefficient)
auto sens = model.get_constraint_sense(row);
double b  = model.get_constraint_rhs(row);
```

Right-hand-side updates are the standard way to run an **ε-constraint** or **budget sweep**: build the row once, then move `b` across the loop.

## Adding and removing entities

New variables and constraints can be added to a solved model at any time; that is what makes cutting-plane and column-generation loops possible.

- `add_constraint` / `add_constraints` — add rows (a cut, a lazy constraint added outside a callback, a new period).
- `add_column(entries, params)` — add a *column* from `(constraint, coefficient)` pairs, the transpose of `add_constraint`. See [Column generation](../algorithms/column-generation.md).
- `remove_variable(v)` / `remove_variables(range)` — on backends satisfying `has_remove_variable` (Clp, CPLEX, Gurobi, HiGHS).

!!! warning "Handles after a removal"
    Handles of the **surviving** entities stay valid across `remove_variable` — MIP++ keeps its own handle-to-column mapping, so you do not renumber anything. The handle of a **removed** variable, on the other hand, is dead and may later be recycled for a new variable: drop it from your own containers at the moment you remove it.

!!! note "That guarantee is free until you use it"
    The mapping is **not** maintained eagerly. A model starts in *identity* mode: a variable handle's id **is** the solver's column index, `add_constraint` writes `handle.id()` straight into the index buffer handed to the C API, and neither id map is allocated.

    The maps are built — and the indirection switched on — only by a `remove_variable` / `remove_variables` call that perforates the column numbering. On HiGHS and Gurobi, deleting the *tail* of the columns (the variables added most recently) does not even do that: the identity mapping survives it. Clp goes further and never remaps at all, zeroing the column in place and recycling its id.

    So a model that never removes a variable — which is most models, including every cutting-plane and most column-generation codes — pays **nothing** for handle stability: no indirection per term, no vector, no branch beyond one predictable flag test. After the first perforating removal, each handle→column access costs a single array lookup, and the model stays in that mode for the rest of its life. The mechanics are in [`remapping_model_base.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/solvers/remapping_model_base.hpp).

## Giving the solver a starting point

On backends satisfying `has_mip_start`, a known feasible (or partially assigned) solution can be handed to the solver from `(variable, value)` pairs:

```cpp
model.add_mip_start({{x1, 1.0}, {x2, 0.0}});

model.add_mip_start(std::views::transform(items, [&](int i) {
    return std::pair{X(i), double(heuristic_selection[i])};
}));
```

Backends: Cbc, COPT, CPLEX, Gurobi, HiGHS, MOSEK, Xpress. This is the usual way to feed a constructive heuristic's solution into an exact run, and to make a "warm" second solve of a perturbed instance cheap.

For LPs, basis warm starts are specified but not yet implemented on any backend — see [Solutions](solutions.md#sensitivity-and-warm-starts).

## A benchmark loop

Putting the pieces together: one model, many parameter values, one row of results each.

```cpp
auto model = milp_type(api);
build(model, instance);                       // once
if constexpr(has_time_limit<milp_type>) model.set_time_limit(600s);

for(double budget : budgets) {
    model.set_constraint_rhs(budget_row, budget);
    model.solve();
    auto rec = run_status(model);          // see Status, limits and tolerances
    if(rec.has_solution)
        report(budget, model.get_solution_value(), model.get_solution());
}
```

Every iteration reuses the solver's internal state; only the numbers that actually changed are pushed. Because model construction is not repeated, the measured time is the *solve* time — which is what the experiment is about.

## Next

[Branch-and-cut with lazy constraints](../algorithms/branch-and-cut.md) — modifying the model from inside the search.
