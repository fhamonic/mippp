# Coming from gurobipy, JuMP or PuLP

If you already write models in Python or Julia, almost everything transfers: MIP++ has the same vocabulary (variables, expressions, constraints, objective, solve, solution) with different spelling. This page is the translation table, followed by the handful of places where MIP++ genuinely behaves differently.

## The same model, three ways

=== "MIP++"

    ```cpp
    #include "mippp/solvers/highs/all.hpp"
    using namespace mippp;
    using namespace mippp::operators;

    highs_api api;
    highs_milp model(api);

    auto x = model.add_binary_variables(n, [](int i) { return i; });

    model.set_maximization();
    model.set_objective(xsum(items, [&](int i) { return value[i] * x(i); }));
    model.add_constraint(
        xsum(items, [&](int i) { return weight[i] * x(i); }) <= capacity);

    model.solve();
    auto sol = model.get_solution();
    ```

=== "gurobipy"

    ```python
    import gurobipy as gp

    model = gp.Model()

    x = model.addVars(n, vtype=gp.GRB.BINARY)

    model.ModelSense = gp.GRB.MAXIMIZE
    model.setObjective(gp.quicksum(value[i] * x[i] for i in items))
    model.addConstr(
        gp.quicksum(weight[i] * x[i] for i in items) <= capacity)

    model.optimize()
    sol = model.getAttr("X", x)
    ```

=== "JuMP"

    ```julia
    using JuMP, HiGHS

    model = Model(HiGHS.Optimizer)

    @variable(model, x[i in items], Bin)

    @objective(model, Max, sum(value[i] * x[i] for i in items))
    @constraint(model, sum(weight[i] * x[i] for i in items) <= capacity)

    optimize!(model)
    sol = value.(x)
    ```

## Translation table

| Task | gurobipy | JuMP | PuLP | MIP++ |
| :--- | :--- | :--- | :--- | :--- |
| create a model | `gp.Model()` | `Model(Opt)` | `LpProblem()` | `highs_milp model(api);` |
| one variable | `addVar(ub=3)` | `@variable(m, x <= 3)` | `LpVariable("x", upBound=3)` | `add_variable({.lower_bound=0, .upper_bound=3})` |
| indexed variables | `addVars(n, m)` | `@variable(m, x[1:n,1:m])` | `LpVariable.dicts` | `add_variables(n*m, [m](int i,int j){return i*m+j;})` |
| binary / integer | `vtype=GRB.BINARY` | `Bin` / `Int` | `cat="Binary"` | `add_binary_variables`, `add_integer_variables` |
| sum over a set | `quicksum(...)` | `sum(...)` | `lpSum(...)` | `xsum(range, lambda)` |
| objective | `setObjective` | `@objective` | `prob += expr` | `set_objective` + `set_maximization` |
| one constraint | `addConstr` | `@constraint` | `prob += lhs <= rhs` | `add_constraint(lhs <= rhs)` |
| constraint family | `addConstrs(... for i in I)` | `@constraint(m, [i in I], ...)` | loop | `add_constraints(I, generator)` |
| solve | `optimize()` | `optimize!` | `solve()` | `solve()` |
| status | `model.Status` | `termination_status` | `LpStatus` | `is_a<status::optimal>(model.solve_status())` |
| a value | `x.X` | `value(x)` | `x.varValue` | `sol[x]` after `auto sol = model.get_solution();` |
| dual | `constr.Pi` | `dual(c)` | `c.pi` | `duals[c]` after `get_dual_solution()` |
| reduced cost | `x.RC` | `reduced_cost(x)` | `x.dj` | `rc[x]` after `get_reduced_costs()` |
| time limit | `setParam("TimeLimit", 60)` | `set_time_limit_sec` | `PULP_CBC_CMD(timeLimit=)` | `set_time_limit(60s)` |
| MIP start | `x.Start = v` | `set_start_value` | `x.setInitialValue` | `add_mip_start(entries)` |
| lazy constraint | `model.cbLazy(...)` | `MOI.submit(..., LazyConstraint)` | — | `handle.add_lazy_constraint(...)` |
| change solver | rewrite in another API | `set_optimizer(...)` | `prob.solve(SOLVER())` | change two type aliases, recompile |

## Six things that are genuinely different

### 1. Index sets are lambdas, not dictionaries

`x[i, j]` in Python is a hash-table lookup on a tuple key. In MIP++, `X(i, j)` calls *your* id-map — plain arithmetic on a contiguous batch of columns:

```cpp
auto X = model.add_binary_variables(n * n, [n](int i, int j) { return i * n + j; });
```

Nothing is hashed, nothing is allocated per lookup. This is the main reason model building runs at C-API speed, and it means you should think about the *layout* of a variable family once, at creation. See [Variables and index sets](../modeling/variables.md).

### 2. Expressions are lazy views, not term lists

`4 * x1 + 5 * x2` does not build a list of terms; it builds a *type* that streams terms when the model consumes it. Build expressions where you use them, and don't store them in a member for later unless you `materialize` them. See [why it's fast](../modeling/expressions.md#why-its-fast-expressions-are-views).

### 3. Inner lambdas must capture the loop key by value

The one real trap. In a constraint family, the generator's parameter must be copied into any nested lambda:

```cpp
model.add_constraints(rows, [&](int i) {
    return xsum(cols, [&, i](int j) { return X(i, j); }) == 1;
    //                 ^^^ i by value
});
```

Python closures capture by reference too, and bite in exactly the same way (`for i in ...: lambda: x[i]`) — here the consequence is a dangling reference rather than a wrong value, so the rule is worth internalising. Details in [Constraint families](../modeling/expressions.md#constraint-families).

### 4. Bounds defaults

`add_variable()` gives the classical `x ≥ 0`. But `add_variable({.upper_bound = 3})` gives `-∞ ≤ x ≤ 3` — supplying a parameter list makes every omitted bound *infinite*, unlike gurobipy's `lb=0` default. Write `{.lower_bound = 0, .upper_bound = 3}` when you mean non-negative.

### 5. The backend is a compile-time choice

There is no `solver=` argument at solve time. The backend appears in an include and two type aliases; a runtime `--solver` flag is written by templating the model code and dispatching, which costs a few lines and gives you all backends in one binary. See [Writing solver-generic code](../solvers/generic-code.md).

### 6. Not everything is there yet

MIP++ deliberately covers the modeling and algorithmic core. Currently missing, and on the [roadmap](https://github.com/fhamonic/mippp#roadmap): LP/MPS file I/O, IIS-based infeasibility diagnosis, solution pools, native multi-objective, quadratic *constraints*, and logging control. If your workflow leans on `model.write("m.lp")` for debugging, use [named variables](../modeling/variables.md#names-assigned-lazily) and the [readable-model accessors](../solving/updates.md#constraint-rows) instead.

## What you gain in exchange

- Model building two to three orders of magnitude faster than the CPython layers, and 6–15× faster than JuMP ([Performance](../performance.md)) — decisive for column generation, cutting planes and large instances.
- The same model code on **11 backends**, with capability differences caught at compile time.
- Everything is C++: your data structures, your graph library, your profiler, no marshalling between a modeling layer and the algorithm around it.

## Next

[Variables and index sets](../modeling/variables.md) — the modeling guide proper.
