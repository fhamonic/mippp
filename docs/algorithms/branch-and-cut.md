# Branch-and-cut: lazy constraints and user cuts

Many exact methods hinge on adding constraints *during* the branch-and-bound search rather than up front — the classic example being subtour elimination for the TSP, where the exponentially many subtour constraints are generated only as candidate solutions violate them.

The method has the same three parts whatever the problem:

1. a **relaxed model**, holding only the polynomially many constraints;
2. a **separation routine**, which takes a candidate solution and looks for a violated constraint of the omitted family;
3. a **callback** wiring the two together inside the solver's search.

MIP++ provides the third part as one uniform interface — the **candidate-solution callback** — so parts 1 and 2 are the only code you write, and they are backend-independent. A second callback, on the relaxation of a node, takes [user cuts](#user-cuts): constraints the model does not need, added to move the bound faster.

## The interface

The callback fires whenever the solver finds an integer candidate solution, and receives a typed handle whose two essential operations are:

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto sol = handle.get_solution();  // the candidate, indexed by variable
    if(/* sol is valid */) return ;
    // otherwise, sol must be excluded:
    handle.add_lazy_constraint(/* any linear constraint expression */);
});
model.solve();
```

- `handle.get_solution()` returns the candidate values, indexed by variable handles exactly like `model.get_solution()`. The point is a copy that owns its values: you can keep it after the callback returns, and write to it.
- `handle.add_lazy_constraint(c)` adds a constraint the candidate violates, built with the same expression syntax (`xsum`, operators) as regular constraints. If no lazy constraint cuts the candidate off, it is accepted as an incumbent.

Take the handle by `auto &` — its type is the backend's `candidate_solution_callback_handle`, and generic code names it through `candidate_solution_callback_handle_t<Model>`.

The two operations are two concepts, because a solver can fire a candidate callback without letting you cut inside it:

- `has_candidate_solution_callback<Model>` promises the callback and `handle.get_solution()`, indexed by the model's variable handles;
- `has_candidate_solution_rejection<Handle>` promises `handle.reject_solution()`, which refuses the candidate without stating why. It needs no model types, so it takes the handle alone. It can refuse more than the candidate, see [below](#rejecting-without-a-constraint);
- `has_lazy_constraints<Handle, Model>` promises `handle.add_lazy_constraint(c)` on that handle. The second parameter is the model, which supplies the variable and constraint types the handle has no way to declare itself (see [Concepts on callback handles](../reference/concepts.md#concepts-on-callback-handles));
- `has_solution_injection<Handle, Model>` promises `handle.add_solution(point)`, which hands the solver a point to try as an incumbent, see [Repairing a rejected candidate](#repairing-a-rejected-candidate).

The callback, with `get_solution()` and `get_solution_value()`, is validated by the test suite on **Gurobi, CPLEX, COPT and Xpress**; lazy constraints and solution injection on the first three. **Xpress** offers `reject_solution()` instead. SCIP has no callback yet.

## Example: TSP subtour elimination

The heart of [`examples/travelling_salesman_dfj/`](https://github.com/fhamonic/mippp/blob/main/examples/travelling_salesman_dfj/main.cpp), where `X(a)` are binary arc variables constrained so that each city has one incoming and one outgoing activated arc:

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto solution = handle.get_solution();
    auto solution_graph = melon::views::subgraph(
        graph, {}, [&](auto a) { return solution[X_vars(a)] > 0.5; });

    for(auto && tour : melon::traversal_forest(solution_graph)) {
        if(tour.size() == graph.num_vertices()) return;
        auto tour_induced_subgraph =
            melon::views::induced_subgraph(graph, tour);
        handle.add_lazy_constraint(
            xsum(melon::arcs(tour_induced_subgraph), X_vars) <=
            static_cast<int>(tour.size()) - 1);
    }
});
```

A dozen lines of algorithmic code, and nothing solver-specific: retargeting this branch-and-cut from Gurobi to CPLEX or COPT is still just the two-alias change.

Note the shape of the separation: it reads the candidate through the variable range in the model's own coordinates (`X_vars(a)`, an arc), builds the violated inequality from a *range of arcs* with `xsum`, and never materialises a term list.

## Writing the separation routine

A few rules that apply on every backend:

- **The callback runs inside the solver's search.** Keep it fast, and keep it free of side effects your model depends on; it may be called many times, and on some backends from a solver-managed thread.
- **Two callbacks of one model never run at the same time**, this one and a [progress callback](../solving/progress.md) included, so a separator can keep state without locking it. CPLEX calls from several threads at once: there a slow callback holds the other threads up.
- **A constraint the candidate satisfies does not reject it**, so a separator may offer every constraint of its family without testing each. Gurobi keeps such a row. CPLEX and COPT can only take a row by rejecting the candidate, so they are not given it: offer it again when a later candidate violates it. A candidate satisfies a constraint when it is within the solver's feasibility tolerance of it, 1e-6 by default.
- **A callback set after a solve is asked about the solution that solve found.** The next `solve()` starts over where it must, so the callback sees every candidate.
- **Do not modify the model from inside it.** Adding variables, changing bounds or reading `model.get_solution()` is not the callback's job — everything goes through the handle.
- **Rounding matters.** The candidate is integral only up to the integrality tolerance; test `> 0.5`, never `== 1.0`.
- **`evaluate` is available** for checking a candidate against a constraint you keep outside the model:

    ```cpp
    if(evaluate(cut_lhs, solution) > rhs + epsilon)
        handle.add_lazy_constraint(cut_lhs <= rhs);
    ```

- **Add all the violated constraints you find**, not just the first — one call to the callback can inject several, as the loop over subtours above does.

## Repairing a rejected candidate

A lazy constraint rejects the whole candidate. In a Benders decomposition that is more than you mean: the integer part of the candidate is feasible, and only the variable that stands for the cost of the subproblem was priced too low. Whether the solver then finds that integer part again is its own business. On a facility location master with one such variable, CPLEX offered it again in the very next call, at its corrected cost. Gurobi lost 18 to 45 candidates per run that were truly better than its incumbent.

`handle.add_solution(point)` hands the solver a point to try as an incumbent. The candidate that `get_solution()` returns is yours to edit, so the repair is to correct it and hand it back:

```cpp
--8<-- "test/doc_snippets/branch_and_cut.cpp:repair"
```

On that master, the repair brought Gurobi an incumbent within 1% of the best known 2 to 12 times sooner on 4 instances of 5, and a better incumbent at the time limit on 2. It changed nothing measurable on CPLEX.

- **The point is complete.** It gives a value to every variable of the model. Any mapping from the model's variables to values is a point: the candidate itself, or one computed on the fly with `maps::function`.
- **The point must be feasible for the whole problem**, the constraints your callback would add included. COPT makes it the incumbent without showing it to the callback, so a point the callback would reject is undefined behavior: it can come back as the optimum, at `status::optimal`.
- **The solver uses it at once, later or never.** A point that does not improve the incumbent is dropped. Gurobi and CPLEX queue the point and show it to the callback before taking it, so the callback runs twice on a repaired candidate: remember the last answer of a costly subproblem.
- **Hand over a repair, not a guess.** There is no call to hand over an integer part alone and let the solver complete it: Gurobi completes one without the lazy constraint just added, and the same wrong candidate comes back.

## Rejecting without a constraint

`handle.reject_solution()`, which only `xpress_milp` has, is not the constraint-free twin of a lazy constraint. When the candidate is the optimum of a node's relaxation, Xpress drops that node, and with it every point below. With `x` an integer in [0, 5] to maximize and a callback that refuses `x = 5`, `xpress_milp` answers 1 at `status::optimal`, where 4 is feasible; with the row `2 x <= 10` added, it answers `infeasible`. Use it only for a rule that every point below the node breaks too.

## User cuts

A lazy constraint is part of the model: without it, the solver would return a point that breaks it. A *user cut* is not. Every point the solver may return satisfies it already, and it only removes fractional points of the relaxation, so that the bound moves faster. It has its own callback, called on the relaxation of a node, and its own call:

```cpp
model.set_node_relaxation_callback([&](auto & handle) {
    auto point = handle.get_solution();  // the relaxation point, fractional
    // for each inequality of your family that the point violates:
    handle.add_user_cut(/* any linear constraint expression */);
});
model.solve();
```

- `has_node_relaxation_callback<Model>` promises the callback, with `handle.get_solution()` and `handle.get_solution_value()`, the objective at that point. The point is a copy, as a candidate is;
- `has_user_cuts<Handle, Model>` promises `handle.add_user_cut(c)`, and its `distinct_variables` form.

Both are validated by the test suite on **Gurobi, CPLEX and COPT**, whose handle also has `add_solution(point)`: a rounding heuristic hands its point over from here. Xpress, SCIP and GLPK are planned. Cbc, HiGHS and MOSEK have no usable call for it; the [cutting-plane loop](#without-a-callback) below needs none.

### What a user cut promises

- **It holds at every point `solve()` may return**: every integer point within the rows and bounds of the model that the candidate-solution callback accepts. A cut that removes such a point is undefined behavior, even when an optimum survives it: solvers answered with a wrong optimum at `status::optimal`, or `infeasible` on a feasible model.
- **So it is valid everywhere, not only below the current node.** A cut that only holds under the branching decisions of the node loses the optimum on Gurobi and COPT, which have no way to keep it local. Nor is a cut valid only for points better than the incumbent a user cut: write those as lazy constraints.
- **The solver applies it anywhere or nowhere**, and may drop it later, so a later point can violate a cut you already gave. Offer a cut whenever it is violated, rather than remembering which were added.
- **It carries no feasibility.** On several solvers a returned point can violate a user cut that was accepted. What the answer depends on goes through `add_lazy_constraint`.

One separator can serve both callbacks, lazy constraints for feasibility and user cuts for strength:

```cpp
--8<-- "test/doc_snippets/branch_and_cut.cpp:both"
```

### When the callback runs

That is the solver's business, and "at every node" holds on none of them. It may never run: not on a model without integer variables, not when presolve or an integral root relaxation ends the search, and not when the solver's own cut generation is switched off (`CutLevel=0` on COPT, `CUTPASS=-1` on CPLEX). An integer point may never reach it. A separator that needs a call to be correct is wrong: feasibility belongs to the candidate-solution callback.

### What MIP++ does to a cut

Solvers given a valid cut exactly as written lose the optimum, crash or loop, with no error code, on cuts as ordinary as an equality or one with a large coefficient. So `add_user_cut` prepares every cut before the solver sees it:

- the terms of a repeated variable are added up, unless you promise `distinct_variables`;
- zero coefficients are removed, and a cut left without a term is dropped;
- the cut is scaled so that its largest coefficient is 1;
- an equality is replaced by the side the point violates;
- a cut that the point does not violate by more than 1e-6, on that scale, is dropped.

The scaling and the last rule have a price. A cut whose only violated part is a big-M term, `w.x - K y <= r` with `y = 0` at the point, is dropped once `K` exceeds a million times its violation. A user cut may be applied nowhere, so nothing is lost but its strength.

### What it costs on each solver

- **Gurobi** needs `PreCrush`, or it ignores a cut on columns its presolve merged. `solve()` writes the parameter while a node-relaxation callback is set and restores it afterwards. It costs nothing measurable on most models, and 100 to 300 times the nodes on one with parallel general integer columns.
- **CPLEX** calls the callback from several threads at once, and MIP++ lets one call run at a time, so a slow separator holds the search up: with 1 ms per call, 16 threads explored 13% of the nodes they explore without the lock. CPLEX also calls again on a node whose relaxation its last cuts did not move; MIP++ skips that call, so a separator is not asked twice about the same point at the same node. A separator that keeps finding new, barely violated cuts can still hold CPLEX at the root: bound its cut passes through `native_api()` in that case.
- **COPT** calls from its own cut loop, and setting the callback changes its search even when it adds nothing.

The [deletion filter](deletion-filter.md#callbacks) takes the callback away for its trials and puts it back: a cut valid for your model is not valid for a model with a row relaxed.

## Generic branch-and-cut code

An algorithm that needs a callback states it in its signature, so instantiating it on a backend without one is a compile-time error rather than a runtime surprise:

```cpp
template <typename Model>
    requires milp_model<Model> && has_candidate_solution_callback<Model> &&
             has_lazy_constraints<candidate_solution_callback_handle_t<Model>,
                                  Model>
void solve_tsp(Model & model, const instance & data);
```

Inside the callback, the same concept constrains the handle in place. The checked type goes first, so the one-argument shorthand reads as "a handle with lazy constraints for `Model`":

```cpp
model.set_candidate_solution_callback(
    [&](has_lazy_constraints<Model> auto & handle) { ... });
```

See [Writing solver-generic code](../solvers/generic-code.md).

## Watching a search is another callback

To follow a search rather than steer it, use a [progress callback](../solving/progress.md). A candidate-solution callback runs on candidates only, so it says nothing while only the bound moves, and setting one can change the search even when it does nothing. COPT treats it as a request for lazy constraints and turns presolve reductions off: on one knapsack that took it from 28860 nodes to 4806, and on one market split instance from an optimum proved in 17 s to no proof in 40 s. On CPLEX a no-op one also moved the search, and on Gurobi `set_candidate_solution_callback` sets `LazyConstraints`.

## Without a callback

On a backend without user cuts, cuts that must be separated on fractional solutions can be added between solves in a **cutting-plane loop** — solve the relaxation, separate, `add_constraint`, solve again, then solve the MILP with the rows in — which needs no callback support at all (see [Re-solving and model updates](../solving/updates.md#adding-and-removing-entities)).

## Next

[Column generation](column-generation.md) — the dual counterpart: growing the model by columns instead of rows.
