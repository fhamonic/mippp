# Expressions and constraints

Real models are families of constraints over index sets, not hand-written rows. This page covers the tools MIP++ provides for that: bulk variable creation with lambda id-maps, `xsum`, and `add_constraints` — and explains the lazy-evaluation model that makes them fast, along with its one pitfall.

Everything below assumes:

```cpp
using namespace mippp;
using namespace mippp::operators;
```

## Lambda-indexed variables

`add_variables(count, id_lambda)` creates `count` variables at once and attaches a function mapping *your* coordinates to an offset in `[0, count)`:

```cpp
// One binary variable per cell of an n×n board.
auto X = model.add_binary_variables(
    n * n, [n](int row, int col) { return row * n + col; });

auto v = X(row, col);  // handle, by plain arithmetic — O(1), no hash map
auto w = X[17];        // plain positional access also works
```

The returned object is a random-access range of variable handles. Its `operator()` takes exactly the parameters of your lambda — any number, any types — so multi-dimensional and irregular indexings cost nothing more than the arithmetic you write. Out-of-range ids throw `std::out_of_range`, so an off-by-one in the id-map fails loudly rather than silently touching the wrong column.

The same signature exists for every variable kind:
`add_variables`, `add_integer_variables`, `add_binary_variables`, each optionally followed by `variable_params` (`{.obj_coef = ..., .lower_bound = ..., .upper_bound = ...}`).

!!! tip "Lazily named variables"
    Variable names are pure overhead for the solver, so MIP++ does not assign any by default. If you want meaningful names (e.g. to debug a model), pass a *name lambda* after the id-map to `add_named_variables`:

    ```cpp
    auto X = model.add_named_variables(
        n * n, [n](int i, int j) { return i * n + j; },
        [](int i, int j) { return std::format("x_{}_{}", i, j); });
    ```

    Names are assigned **lazily** — only the first time each variable is accessed through `X(i, j)` — so you pay only for the variables you touch.

## `xsum`: sums over ranges

`xsum` is the workhorse for building linear expressions from data:

```cpp
// sum of x[i] for i in r:
xsum(r, x)                                           // r: range of indices
// sum of f(e) for e in r:
xsum(cities, [&](int j) { return dist[i][j] * X(i, j); })
// a range of expressions can be summed directly too:
xsum(terms_range)
```

Combined with the standard `<ranges>` library, entire objectives are one-liners:

```cpp
model.set_objective(
    xsum(std::views::cartesian_product(cities, cities), [&](auto && p) {
        auto && [i, j] = p;
        return dist[i][j] * X(i, j);
    }));
```

Expressions compose with the usual algebra — `+`, `-`, unary `-`, multiplication/division by a scalar — and compare with `<=`, `>=`, `==` against another expression or a scalar to form constraints:

```cpp
model.add_constraint(xsum(subtour, X) <= int(subtour.size()) - 1);
model.add_constraint(3 * y - xsum(orders, x) == 0);
```

If the same variable appears several times in an expression, its coefficients are summed when the row is registered — `x + 2 * x` is the row `3 x`.

## Constraint families

`add_constraints(keys, generator)` adds one constraint per key and returns a range of the created constraints:

```cpp
auto rows = model.add_constraints(std::views::iota(0, n), [&](int row) {
    return xsum(std::views::iota(0, n),
                [&, row](int col) { return X(row, col); }) == 1;
});
```

The result is iterable like any range, and — like lambda-indexed variables — callable **by key**: `rows(3)` returns the constraint handle built for key `3`. This pairs naturally with dual solutions in decomposition algorithms:

```cpp
auto duals = model.get_dual_solution();
for(int o : orders) price[o] = duals[demand_constraints(o)];
```

!!! warning "Inner lambdas must capture the generator's parameter *by value*"
    `add_constraints` registers each returned expression **lazily**: the terms of the `xsum` are iterated *after* your generator has returned. A nested lambda that captures the generator's parameter by reference (`[&](int col) { return X(row, col); }` captures `row` by reference) therefore dangles by the time the terms are read — producing wrong indices or a `std::out_of_range` throw from the variables range.

    Capture the outer key **by value** in the inner lambda, keeping everything else by reference:

    ```cpp
    model.add_constraints(indices, [&](int row) {
        return xsum(indices, [&, row](int col) { return X(row, col); }) == 1;
        //                    ^^^^^^ row copied into the inner lambda
    });
    ```

## Why it's fast: expressions are views

None of the syntax above allocates or copies terms. An expression in MIP++ is anything satisfying the `linear_expression` concept — it can produce a range of `(variable, coefficient)` pairs plus a constant. A variable handle is itself a one-term expression, and every operator just wraps its operands in a standard-library view:

- `e1 + e2` → `views::concat` of the two term ranges,
- `c * e` → `views::transform` scaling each coefficient,
- `xsum(r, f)` → `views::join` of `views::transform(r, f)`.

So `4 * x1 + 5 * x2` builds a *type*, not a data structure. Only when the expression reaches the model (`set_objective`, `add_constraint`, …) are its terms iterated — once — into a small buffer that the model reuses for every row, and handed to the solver's C API. This is why MIP++'s model-building overhead stays within ~10–20% of hand-written C API code.

Two practical consequences:

1. **Build expressions where you use them.** An expression view references the ranges and lambdas it was built from; the safe (and idiomatic) pattern is to construct it inside the `add_constraint` / `set_objective` / generator call, as in all the examples.
2. **For incremental construction**, when a row is assembled across loops or functions and a view won't do, `runtime_linear_expression` is a materialized expression with `operator+=` that you can pass to the same model functions.

## Evaluating expressions

`evaluate(expr, values)` computes the value of any linear expression under a map from variables to values — typically a solution:

```cpp
auto sol = model.get_solution();
double lhs = evaluate(2 * x1 + x2, sol);
```

Next: [Choosing a solver](solvers.md).
