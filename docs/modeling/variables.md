# Variables and index sets

Textbook models are written over meaningful index sets — *x(i,j)* for arcs, *y(p)* for patterns, *z(i,j,v)* for assignments — while solver APIs only know flat column numbers. This page covers how MIP++ bridges the two: variable handles, bulk creation, and the lambda id-maps that let you keep your problem's own coordinates.

Everything below assumes:

```cpp
using namespace mippp;
using namespace mippp::operators;
```

## A variable is a handle

```cpp
auto x = model.add_variable();
```

`add_variable` returns a lightweight, trivially-copyable **handle** — a strongly-typed column index. Copying it costs nothing, comparing it is an integer comparison, and it is also a one-term linear expression, which is why `4 * x1 + 5 * x2` works without any further ceremony.

Handles are typed per model class (`M::variable`), so a variable of one model cannot silently be used in another.

## Bounds and objective coefficient

Options are given by designated initializers on the `variable_params` struct:

```cpp
model.add_variable({.obj_coef = 1.0, .lower_bound = -2.0, .upper_bound = 3.0});
```

Every field is optional, with one subtlety worth internalising early:

!!! warning "A bare `add_variable()` is *not* the same as `add_variable({})`"
    `add_variable()` creates the classic LP default: objective coefficient `0`, lower bound `0`, **no** upper bound. But when a `variable_params` list is given, an **omitted bound is unbounded** — `{.upper_bound = 3}` declares `-∞ ≤ x ≤ 3`, not `0 ≤ x ≤ 3`.

    Write `{.lower_bound = 0, .upper_bound = 3}` when you mean a non-negative bounded variable.

Passing `.obj_coef` at creation is equivalent to — and cheaper than — mentioning the variable in `set_objective` later; see [Objectives](objectives.md).

## Integer and binary variables

MILP model classes (`highs_milp`, `gurobi_milp`, …) add:

```cpp
auto y = model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
auto b = model.add_binary_variable();
```

and let a variable's type be changed afterwards with `set_continuous(v)` / `set_integer(v)` / `set_binary(v)` — useful to solve the LP relaxation and the MILP from the same model object.

An `*_milp` model with only continuous variables is a perfectly good LP; the split exists because some backends (Clp, SoPlex) are LP-only and others (Cbc, SCIP) MILP-only. See [Choosing a solver](../solvers/index.md).

## Bulk creation and lambda id-maps

`add_variables(count, id_lambda)` creates `count` variables at once and attaches a function mapping *your* coordinates to an offset in `[0, count)`:

```cpp
// One binary variable per cell of an n×n board.
auto X = model.add_binary_variables(
    n * n, [n](int row, int col) { return row * n + col; });

auto v = X(row, col);  // handle, by plain arithmetic — O(1), no hash map
auto w = X[17];        // plain positional access also works
```

The returned object is a random-access range of variable handles. Its `operator()` takes exactly the parameters of your lambda — any number, any types — so multi-dimensional and irregular indexings cost nothing more than the arithmetic you write. There is no dictionary, no tuple key, no string lookup anywhere on that path, which is a large part of why model building stays close to C speed.

Out-of-range ids throw `std::out_of_range`, so an off-by-one in the id-map fails loudly rather than silently touching the wrong column.

The same signature exists for every variable kind — `add_variables`, `add_integer_variables`, `add_binary_variables` — each optionally followed by `variable_params`:

```cpp
auto flow = model.add_variables(
    num_arcs, [](arc a) { return a.id(); },
    {.lower_bound = 0, .upper_bound = capacity_max});
```

Without an id-map, `add_variables(count)` returns the same kind of range, indexable positionally.

### Choosing an id-map

The id-map is the place where your problem's structure meets the solver's flat indexing. Two rules of thumb:

- **Make it arithmetic.** Row-major offsets (`i * n + j`), block offsets (`81 * i + 9 * j + (v - 1)` in the Sudoku example), or an id already carried by your data structure (a graph's arc id) are all O(1) and branch-free.
- **Create one batch per family of variables.** A batch is contiguous in the solver, which makes the offsets trivial and keeps the ranges independent — `X` and `Y` can each have their own coordinates.

For irregular index sets (only the arcs present in a sparse graph, only the feasible pairs), build the offset table once in your own data and have the lambda read it:

```cpp
// offset[i] = index of the first variable of row i, built from your data.
auto X = model.add_binary_variables(
    total, [&offset](int i, int k) { return offset[i] + k; });
```

## Names, assigned lazily

Variable names are pure overhead for the solver, so MIP++ assigns none by default. When you want them — to debug a model, or to export it — pass a *name lambda* after the id-map to `add_named_variables`:

```cpp
auto X = model.add_named_variables(
    n * n, [n](int i, int j) { return i * n + j; },
    [](int i, int j) { return std::format("x_{}_{}", i, j); });
```

Names are assigned **lazily** — only the first time each variable is accessed through `X(i, j)` — so you pay only for the variables you touch. Individual variables can also be named on the fly with `add_named_variable(name)` or `set_variable_name(v, name)`, on backends satisfying `has_named_variables`.

## Next

[Expressions and constraints](expressions.md) — how these handles combine into objectives and whole constraint families over your index sets.
