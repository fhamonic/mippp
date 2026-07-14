# Branch-and-cut callbacks

Many exact methods hinge on adding constraints *during* the branch-and-bound search rather than up front — the classic example being subtour elimination for the TSP, where the exponentially many subtour constraints are generated only as candidate solutions violate them.

MIP++ wraps the solvers' callback machinery behind one uniform interface: the **candidate-solution callback**. It fires whenever the solver finds an integer candidate solution, and receives a typed handle whose two essential operations are:

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto sol = handle.get_solution();   // the candidate, indexed by variable
    // inspect it, and if it must be excluded:
    handle.add_lazy_constraint(/* any linear constraint expression */);
});
model.solve();
```

- `handle.get_solution()` returns the candidate values, indexed by variable
  handles exactly like `model.get_solution()`.
- `handle.add_lazy_constraint(c)` injects a violated constraint, built with the same expression syntax (`xsum`, operators) as regular constraints. If no lazy constraint is added, the candidate is accepted as an incumbent.

Backends supporting this are those satisfying the `has_candidate_solution_callback` concept — currently **Gurobi, CPLEX, and COPT**.

## Example: TSP subtour elimination

The heart of [`examples/tsp_lazy_constraints.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/tsp_lazy_constraints.cpp), where `X(i, j)` are binary arc variables constrained so that each city has one incoming and one outgoing arc:

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto sol = handle.get_solution();
    // successor[i] = the city visited right after i in the candidate.
    std::vector<int> successor(n);
    for(int i : cities)
        for(int j : cities)
            if(i != j && sol[X(i, j)] > 0.5) successor[i] = j;

    // Walk the successor cycles; each cycle shorter than n is a subtour.
    std::vector<bool> seen(n, false);
    for(int start : cities) {
        if(seen[start]) continue;
        std::vector<int> subtour;
        for(int v = start; !seen[v]; v = successor[v]) {
            seen[v] = true;
            subtour.push_back(v);
        }
        if(int(subtour.size()) == n) return;  // a single tour: accept

        // Arcs inside the subtour must number at most |S| - 1.
        handle.add_lazy_constraint(
            xsum(std::views::cartesian_product(subtour, subtour),
                 [&](auto && p) {
                     auto && [i, j] = p;
                     return X(i, j);
                 }) <= int(subtour.size()) - 1);
    }
});
```

A dozen lines of algorithmic code, and nothing solver-specific: retargeting this branch-and-cut from Gurobi to CPLEX or COPT is still just the two-alias change.

## What's next

A node-relaxation callback (for *user cuts*, i.e. cutting fractional relaxation solutions) and heuristic-solution injection are on the [roadmap](https://github.com/fhamonic/mippp#roadmap).
