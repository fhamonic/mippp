# Objectives

The objective is set with the same expression syntax as constraints, but it is the part of a model research code most often *changes* between solves — reweighting, lexicographic passes, Lagrangian penalties, pricing. This page covers setting it, incrementing it, reading it back, and the quadratic case.

## Sense and expression

```cpp
model.set_maximization();          // or model.set_minimization()
model.set_objective(4 * x1 + 5 * x2);
```

`set_objective` **replaces** the objective entirely: every coefficient not mentioned in the expression is reset to `0`, and the objective offset is set to the expression's constant. That makes it safe to call in a loop — you never inherit terms from the previous iteration.

The sense is independent of the expression, and can be flipped at any time without rebuilding it.

Two other ways to reach the same objective:

```cpp
// 1. at variable creation — cheapest, no expression is ever built
auto x = model.add_variable({.obj_coef = 4.0, .lower_bound = 0});

// 2. a constant term on its own
model.set_objective_offset(fixed_cost);
```

The offset is added to `get_solution_value()` by the solver, so it is the right place for the constant part of a decomposition's objective rather than a `+ constant` you subtract by hand later.

## Incremental changes

Backends satisfying `has_modifiable_objective` can change the objective without respecifying it:

```cpp
model.set_objective_coefficient(x, 2.0);      // one column
model.add_objective(xsum(penalised, [&](int i) { return mu[i] * X(i); }));
```

`add_objective(e)` **adds** the expression to the current objective (and adds its constant to the offset) — the natural primitive for Lagrangian relaxation, where each iteration perturbs the objective by a multiplier-weighted term rather than rebuilding it:

```cpp
for(int it = 0; it < max_iters; ++it) {
    model.set_objective(base_objective_expr());          // reset
    model.add_objective(xsum(coupling, [&](int k) {       // perturb
        return mu[k] * slack_expr(k); }));
    model.solve();
    update_multipliers(mu, model.get_solution());
}
```

See [Re-solving and model updates](../solving/updates.md) for what a solver may reuse between two such solves.

## Reading it back

Backends satisfying `has_readable_objective` expose:

```cpp
double c_x    = model.get_objective_coefficient(x);
double offset = model.get_objective_offset();
auto   obj    = model.get_objective();   // itself a linear_expression
```

`get_objective()` returns an expression, so it composes with everything else — it can be evaluated at a solution, summed with another expression, or compared to build a constraint (an objective cut-off row, say):

```cpp
model.add_constraint(model.get_objective() <= incumbent_value - 1);
```

## Quadratic objectives

Model classes satisfying `qp_model` accept a quadratic expression in `set_objective`. Products of expressions and `square(e)` build them:

```cpp
highs_qp model(api);
auto x1 = model.add_variable();
auto x2 = model.add_variable();

model.set_minimization();
model.set_objective(2 * x1 * x1 + 2 * x2 * x2 - 4 * x1 - 6 * x2);
model.add_constraint(x1 + x2 >= 3);
```

Notes specific to the quadratic layer:

- Quadratic objectives are currently available on **HiGHS** (`highs_qp`) only.
- The quadratic term stream is a multiset of `(variable, variable, coefficient)` triples and the pairs are **unordered**: `square(x1 + x2)` emits both `(x1, x2, 1)` and `(x2, x1, 1)`. Backends fold `(i, j)` with `(j, i)` and sum duplicates when building the (triangular) Hessian, so you never do that bookkeeping.
- A product traverses one operand once per term of the other, so both operands must be **multipass** expressions. `square(xsum(...))` is a compile error with the fix spelled out in the message; `square(materialize(e))` is the fix:

    ```cpp
    auto e = xsum(vars);
    // square(e);            // ✗ single-pass
    square(materialize(e));  // ✓
    ```

The rules behind that requirement are detailed in [Inside the expression layer](../reference/expression-layer.md#products-need-a-second-pass).

## Several objectives

Native multi-objective support (solver-side lexicographic or blended objectives) is on the [roadmap](https://github.com/fhamonic/mippp#roadmap). Until it lands, the two classical encodings are written directly:

- **Weighted sum** — one `set_objective` per weight vector, in a loop.
- **Lexicographic / ε-constraint** — solve for the first objective, then add a constraint freezing it (using `get_objective()` above, or the expression you built it from) and set the next objective.

Both are cheap here, because re-setting an objective and re-solving does not rebuild the model.

## Next

[Special constraints](special-constraints.md) — indicator constraints, logical conditions, and what each backend actually provides.
