# Expressions and constraints

Real models are families of constraints over index sets, not hand-written rows. This page covers the tools MIP++ provides for that — `xsum`, `add_constraints`, and the algebra of expressions — and explains the lazy-evaluation model that makes them fast, along with its one pitfall.

It assumes the variables of the [previous page](variables.md), and:

```cpp
using namespace mippp;
using namespace mippp::operators;
```

## `xsum`: sums over ranges

`xsum` is the workhorse for building linear expressions from data — the counterpart of `quicksum` in gurobipy or `sum(...)` in JuMP, except that it builds no term list at all:

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
    xsum(std::views::cartesian_product(cities, cities),
         [&](int i, int j) { return dist[i][j] * X(i, j); }));
```

The elements of a `cartesian_product` (and of `zip`, `enumerate`, or any range of `std::pair`s or `std::tuple`s) are tuples, and a function that does not accept the tuple itself is called with its **elements unpacked**, one parameter per component. A lambda that does accept the tuple — `[&](auto && p)` with a structured binding inside, or one taking `std::pair<int, int>` — is called with it unchanged, so both spellings coexist. The same rule applies to every function MIP++ calls on a key: `xsum`, the generators of `add_constraints`, and the id and name functions of the [`indexed` and `named` wrappers](#constraint-families).

Because the range comes first, every `<ranges>` adaptor is available to describe the index set — `filter` for sparsity, `iota` for intervals, `cartesian_product` for multi-dimensional families, `zip` to walk coefficients and variables together:

```cpp
// only the items that fit
xsum(std::views::filter(items, [&](int i) { return weight[i] <= capacity; }),
     [&](int i) { return value[i] * X(i); });
```

## The algebra

Expressions compose with the usual operations — `+`, `-`, unary `-`, multiplication and division by a scalar — and compare with `<=`, `>=`, `==` against another expression or a scalar to form constraints:

```cpp
model.add_constraint(xsum(subtour, X) <= int(subtour.size()) - 1);
model.add_constraint(3 * y - xsum(orders, x) == 0);
model.add_constraint(2 * x1 + x2 >= 3 * y - 1);   // both sides may be expressions
```

A comparison with expressions on both sides is normalised to a single row (`terms sense rhs`) on the way to the solver — you never build a two-sided object.

If the same variable appears several times in an expression, its coefficients are summed when the row is registered: `x + 2 * x` reaches the solver as the row `3 x`. A term stream is a *multiset*, and its order is unspecified — never write code that depends on it.

!!! note "Scalar types must match exactly"
    Binary operators require both operands to use the same variable *and* scalar types; no implicit conversion is performed. A stray `float` literal in a `double` model is a compile error, not a silent narrowing. See [Inside the expression layer](../reference/expression-layer.md#the-trait-aliases).

## Constraint families

`add_constraints(keys, generator)` adds one constraint per key and returns a range of the created constraints:

```cpp
auto rows = model.add_constraints(std::views::iota(0, n), [&](int row) {
    return xsum(std::views::iota(0, n),
                [&, row](int col) { return X(row, col); }) == 1;
});
```

The result is iterable like any range, indexable by position (`rows[i]` is the constraint built for the *i*-th key) and — like [keyed variables](variables.md#keyed-families) — callable **by key**: `rows(3)` returns the constraint handle built for key `3`. Keeping constraints addressable by your own coordinates is what makes duals usable in decomposition algorithms:

```cpp
auto duals = model.get_dual_solution();
for(int o : orders) price[o] = duals[demand_constraints(o)];
```

How a key is resolved is decided at compile time from the type of the key range, and never costs more than that range requires:

- **`std::views::iota`, and `cartesian_product`s of such ranges** — the position is computed arithmetically and nothing is stored. Tuple keys may be passed unpacked, both to the generator and to the lookup: `cells(i, j)` is `cells(std::tuple{i, j})`.

    ```cpp
    auto cells = model.add_constraints(
        std::views::cartesian_product(rows, cols),
        [&](int i, int j) { return X(i, j) + Y(i, j) <= 1; });
    ```
- **Any other range** — the keys are copied into a hash map when `std::hash` is specialized for them, otherwise into a sorted vector when they are totally ordered (`<` and `==`). Duplicate keys resolve to their first constraint.
- **`indexed(keys, id)`** — you supply a function mapping each key to a dense non-negative integer, and the lookup goes through a table sized to the largest id. It is the counterpart of the id-lambda of [`add_variables`](variables.md#count-and-id-map), for keys that carry their own index (a struct with an `id` field, a filtered subset of an interval):

    ```cpp
    auto demand = model.add_constraints(indexed(orders, &order::id), [&](const order & o) {
        return xsum(patterns_covering(o), [&, o](int p) { return cover[p][o.id] * X(p); }) >= o.demand;
    });
    for(const order & o : orders) price[o.id] = duals[demand(o)];
    ```

Keys that are neither hashable nor ordered still yield an iterable, positionally indexable range; calling it by key is then a compile-time error whose message names `indexed` as the remedy.

A range type can also supply its own lookup: `mippp::key_index` is a customization point object, and a non-template `key_index(const range &)` function found by argument-dependent lookup, returning an object with `position(key)`, takes precedence over the built-in strategies.

On backends with constraint names (concept `has_named_constraints`), a key range wrapped with `named(keys, name)` names each constraint as it is added, from a function of its key; `indexed_named(keys, id, name)` gives both an id and a name (the wrappers do not nest):

```cpp
auto rows = model.add_constraints(
    named(std::views::iota(0, n), [](int i) { return std::format("row_{}", i); }),
    [&](int i) { return xsum(cols, [&, i](int j) { return X(i, j); }) == 1; });
```

Unlike variable names, which are assigned lazily on first access, constraint names are written in the same call: a family is added in bulk, and names are most useful when the whole model is exported.

A single `add_constraint(c)` likewise returns one constraint handle, which you can keep to read its dual or to modify the row later (see [Re-solving and model updates](../solving/updates.md)).

!!! warning "Inner lambdas must capture the generator's parameter *by value*"
    `add_constraints` registers each returned expression **lazily**: the terms of the `xsum` are iterated *after* your generator has returned. A nested lambda that captures the generator's parameter by reference (`[&](int col) { return X(row, col); }` captures `row` by reference) therefore dangles by the time the terms are read — producing wrong indices or a `std::out_of_range` throw from the variables range.

    Capture the outer key **by value** in the inner lambda, keeping everything else by reference:

    ```cpp
    model.add_constraints(indices, [&](int row) {
        return xsum(indices, [&, row](int col) { return X(row, col); }) == 1;
        //                    ^^^^^^ row copied into the inner lambda
    });
    ```

    This is the most common mistake when writing MIP++ models; every constraint family in the [worked examples](../examples.md) shows the correct form.

## Why it's fast: expressions are views

None of the syntax above allocates or copies terms. An expression in MIP++ is anything satisfying the `linear_expression` concept — it can produce a range of `(variable, coefficient)` pairs plus a constant. A variable handle is itself a one-term expression, and every operator just wraps its operands in a standard-library view:

- `e1 + e2` → `views::concat` of the two term ranges,
- `c * e` → `views::transform` scaling each coefficient,
- `xsum(r, f)` → `views::join` of `views::transform(r, f)`.

So `4 * x1 + 5 * x2` builds a *type*, not a data structure. Only when the expression reaches the model (`set_objective`, `add_constraint`, …) are its terms iterated — once — into a small buffer that the model reuses for every row, and handed to the solver's C API. This is why a whole family of constraints costs little more than the C API calls it ultimately makes — see [Performance](../performance.md).

Two practical consequences:

1. **Build expressions where you use them.** An expression view references the ranges and lambdas it was built from; the safe (and idiomatic) pattern is to construct it inside the `add_constraint` / `set_objective` / generator call, as in all the examples.
2. **For incremental construction**, when a row is assembled across loops or functions and no single view can be formed, `runtime_linear_expression` is a materialized expression with `operator+=`, accepted by the same model functions:

    ```cpp
    runtime_linear_expression<decltype(x1), double> row;
    for(auto && part : parts) row += weight(part) * X(part);
    model.add_constraint(row <= capacity);
    ```

    `materialize(e)` produces one from an existing expression, which is also the fix when an expression must be consumed twice.

The full ownership rules — which expressions can be read twice, which are single-pass, and how the diagnostics read — are in [Inside the expression layer](../reference/expression-layer.md).

## Evaluating expressions

`evaluate(expr, values)` computes the value of any linear expression under a map from variables to values — typically a solution:

```cpp
auto sol = model.get_solution();
double lhs = evaluate(2 * x1 + x2, sol);
```

This is the usual way to check a candidate against a constraint that is not in the model — for instance inside a [lazy-constraint callback](../algorithms/branch-and-cut.md).

## Next

[Objectives](objectives.md) — setting, incrementing and reading the objective, including quadratic ones.
