# Branch-and-cut with lazy constraints

Many exact methods hinge on adding constraints *during* the branch-and-bound search rather than up front — the classic example being subtour elimination for the TSP, where the exponentially many subtour constraints are generated only as candidate solutions violate them.

The method has the same three parts whatever the problem:

1. a **relaxed model**, holding only the polynomially many constraints;
2. a **separation routine**, which takes a candidate solution and looks for a violated constraint of the omitted family;
3. a **callback** wiring the two together inside the solver's search.

MIP++ provides the third part as one uniform interface — the **candidate-solution callback** — so parts 1 and 2 are the only code you write, and they are backend-independent.

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

- `handle.get_solution()` returns the candidate values, indexed by variable handles exactly like `model.get_solution()`.
- `handle.add_lazy_constraint(c)` injects a violated constraint, built with the same expression syntax (`xsum`, operators) as regular constraints. If no lazy constraint is added, the candidate is accepted as an incumbent.

Take the handle by `auto &` — its type is the backend's `candidate_solution_callback_handle`, and generic code names it through `candidate_solution_callback_handle_t<Model>`.

Backends supporting this are those satisfying the `has_candidate_solution_callback` concept: implemented on **Gurobi, CPLEX, COPT, SCIP and Xpress**, and validated by the test suite on **Gurobi, CPLEX and COPT**.

## Example: TSP subtour elimination

The heart of [`examples/tsp_lazy_constraints.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/tsp_lazy_constraints.cpp), where `X(a)` are binary arc variables constrained so that each city has one incoming and one outgoing activated arc:

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
- **Do not modify the model from inside it.** Adding variables, changing bounds or reading `model.get_solution()` is not the callback's job — everything goes through the handle.
- **Rounding matters.** The candidate is integral only up to the integrality tolerance; test `> 0.5`, never `== 1.0`.
- **`evaluate` is available** for checking a candidate against a constraint you keep outside the model:

    ```cpp
    if(evaluate(cut_lhs, solution) > rhs + epsilon)
        handle.add_lazy_constraint(cut_lhs <= rhs);
    ```

- **Add all the violated constraints you find**, not just the first — one call to the callback can inject several, as the loop over subtours above does.

## Generic branch-and-cut code

An algorithm that needs a callback states it in its signature, so instantiating it on a backend without one is a compile-time error rather than a runtime surprise:

```cpp
template <typename Model>
    requires milp_model<Model> && has_candidate_solution_callback<Model>
void solve_tsp(Model & model, const instance & data);
```

See [Writing solver-generic code](../solvers/generic-code.md).

## What's next

A node-relaxation callback (for *user cuts*, i.e. cutting fractional relaxation solutions) and heuristic-solution injection are on the [roadmap](https://github.com/fhamonic/mippp#roadmap); the concept `has_node_relaxation_callback` is already specified in [`model_concepts.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/model_concepts.hpp), with no backend implementing it yet.

Until then, cuts that must be separated on fractional solutions can be added between solves in a **cutting-plane loop** — solve, separate, `add_constraint`, solve again — which needs no callback support at all (see [Re-solving and model updates](../solving/updates.md#adding-and-removing-entities)).

## Next

[Column generation](column-generation.md) — the dual counterpart: growing the model by columns instead of rows.
