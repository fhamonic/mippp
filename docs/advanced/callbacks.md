# Branch-and-cut callbacks

Many exact methods hinge on adding constraints *during* the branch-and-bound search rather than up front — the classic example being subtour elimination for the TSP, where the exponentially many subtour constraints are generated only as candidate solutions violate them.

MIP++ wraps the solvers' callback machinery behind one uniform interface: the **candidate-solution callback**. It fires whenever the solver finds an integer candidate solution, and receives a typed handle whose two essential operations are:

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto sol = handle.get_solution();  // the candidate, indexed by variable
    if(/* sol is valid */) return ;
    // otherwise, sol must be excluded:
    handle.add_lazy_constraint(/* any linear constraint expression */);
});
model.solve();
```

- `handle.get_solution()` returns the candidate values, indexed by variable
  handles exactly like `model.get_solution()`.
- `handle.add_lazy_constraint(c)` injects a violated constraint, built with the same expression syntax (`xsum`, operators) as regular constraints. If no lazy constraint is added, the candidate is accepted as an incumbent.

Backends supporting this are those satisfying the `has_candidate_solution_callback` concept — currently **Gurobi, CPLEX, and COPT**.

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

## What's next

A node-relaxation callback (for *user cuts*, i.e. cutting fractional relaxation solutions) and heuristic-solution injection are on the [roadmap](https://github.com/fhamonic/mippp#roadmap).
