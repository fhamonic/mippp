# Variables and index sets

Textbook models are written over meaningful index sets — *x(i,j)* for arcs, *y(p)* for patterns, *z(i,j,v)* for assignments — while solver APIs only know flat column numbers. This page covers how MIP++ bridges the two: variable handles, bulk creation over a range of keys, and the lambda id-maps for the coordinate spaces you would rather not enumerate.

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

Handles carry their *kind* in the type: a variable handle and a constraint handle never compare equal and never substitute for one another, even though both wrap an integer id. They carry no *model identity*, though — `model_variable_t<M>` is the same type for every backend, and the handle is little more than the column index. Passing a variable of one model to another therefore compiles, and silently addresses whichever column happens to sit at that index. When a program juggles several models — a master and a pricing problem, say — keep each model's handles with it and name them apart.

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

### Infinite bounds

An absent side is stored as whatever the solver uses for "no bound" — `1e20` on CPLEX, SCIP and Xpress, `1e30` on COPT and MOSEK, `1e100` on Gurobi, `DBL_MAX` on the COIN-OR solvers and GLPK, `inf` on HiGHS — and `get_variable_lower_bound` / `get_variable_upper_bound` hand that value back untouched. Two members make this portable without hiding it:

```cpp
double big = model.infinity();                        // the solver's own threshold
bool free_above = model.is_infinite(model.get_variable_upper_bound(x));
model.set_variable_upper_bound(x, model.infinity());  // removes the upper bound
model.set_variable_lower_bound(x, -model.infinity()); // removes the lower bound
```

`is_infinite(v)` is `|v| >= infinity()`, not `v == infinity()`: a solver may store the value it was given and treat anything beyond its threshold as infinite, so equality would miss a bound set to, say, `1e30` on CPLEX. Both members are part of `lp_model`, so generic code can rely on them; `infinity()` is a member rather than a constant because SCIP's threshold is a runtime parameter.

## Integer and binary variables

MILP model classes (`highs_milp`, `gurobi_milp`, …) add:

```cpp
auto y = model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
auto b = model.add_binary_variable();
```

and let a variable's type be changed afterwards with `set_continuous(v)` / `set_integer(v)` / `set_binary(v)` — useful to solve the LP relaxation and the MILP from the same model object.

An `*_milp` model with only continuous variables is a perfectly good LP; the split exists because some backends (Clp, SoPlex) are LP-only and others (Cbc, SCIP) MILP-only. See [Choosing a solver](../solvers/index.md).

## Bulk creation

Variables are created in batches, one batch per family of the model. The preferred form takes a **range of keys**, one variable per key, and returns a range callable by key:

```cpp
// One binary variable per cell of an n×n board.
auto X = model.add_binary_variables(
    std::views::cartesian_product(std::views::iota(0, n), std::views::iota(0, n)));

auto v = X(row, col);  // handle, by plain arithmetic — O(1), no hash map
auto w = X[17];        // plain positional access also works
```

The lookup rules are those of [constraint families](expressions.md#constraint-families), selected at compile time from the key type: arithmetic for `iota` and cartesian products of them, a table for `indexed(keys, id)`, a hash map or a sorted vector for any other key:

```cpp
auto F = model.add_variables(indexed(graph.arcs(), [](arc a) { return a.id(); }));
auto Y = model.add_variables(labels);     // std::vector<std::string>, hashed
auto Z = model.add_integer_variables(std::views::iota(0, m), {.upper_bound = 10});
```

The returned object is a random-access range of variable handles. For the arithmetic and table strategies, `X(i, j)` costs the offset computation and nothing else: no dictionary, no tuple key, no string lookup, which is a large part of why model building stays close to C speed. Unknown keys throw `std::out_of_range`, so an off-by-one fails loudly rather than silently touching the wrong column.

The same signature exists for every variable kind — `add_variables`, `add_integer_variables`, `add_binary_variables` — each optionally followed by `variable_params`. Prefer this form: the offsets are derived from the keys instead of written by hand and the range can be [named](#names) in the same call.

### Count and id-map

Sometimes there is no range of keys to hand over: the coordinate space is too large or too irregular to enumerate, it lives in a data structure you do not want to copy into a view, or the offset arithmetic is simply already written. For those cases `add_variables(count, id_lambda)` creates `count` variables and attaches a function mapping *your* coordinates to an offset in `[0, count)`:

```cpp
auto X = model.add_binary_variables(
    n * n, [n](int row, int col) { return row * n + col; });

auto flow = model.add_variables(
    num_arcs, [](arc a) { return a.id(); },
    {.lower_bound = 0, .upper_bound = capacity_max});
```

`operator()` takes exactly the parameters of the lambda — any number, any types — and out-of-range ids throw `std::out_of_range`. This form is kept for compatibility and for the cases above; it is not deprecated, but it cannot resolve a string or a struct key, does not check that the lambda covers `[0, count)`, and names its variables lazily (see [Names](#names)). When the keys can be materialised, pass them instead.

Without an id-map, `add_variables(count)` returns the same kind of range, indexable positionally.

### Choosing an id-map

When writing an id-map, two rules of thumb:

- **Make it arithmetic.** Row-major offsets (`i * n + j`), block offsets (`81 * i + 9 * j + (v - 1)` in the Sudoku example), or an id already carried by your data structure (a graph's arc id) are all O(1) and branch-free.
- **Create one batch per family of variables.** A batch is contiguous in the solver, which makes the offsets trivial and keeps the ranges independent — `X` and `Y` can each have their own coordinates.

For irregular index sets (only the arcs present in a sparse graph, only the feasible pairs), build the offset table once in your own data and have the lambda read it:

```cpp
// offset[i] = index of the first variable of row i, built from your data.
auto X = model.add_binary_variables(
    total, [&offset](int i, int k) { return offset[i] + k; });
```

## Names

Variable names are pure overhead for the solver, so MIP++ assigns none by default. On backends satisfying `has_named_variables`, a key range wrapped with `named(keys, name)` names each variable as it is created, from a function of its key; `indexed_named(keys, id, name)` gives both an id and a name:

```cpp
auto X = model.add_variables(
    named(std::views::iota(0, n), [](int i) { return std::format("x_{}", i); }));
auto Y = model.add_variables(
    named(std::views::cartesian_product(std::views::iota(0, n), std::views::iota(0, m)),
          [](int i, int j) { return std::format("y_{}_{}", i, j); }));
```

Tuple keys are unpacked into the name function's parameters, as for [every function called on a key](expressions.md#xsum-sums-over-ranges).

With the count and id-map form, pass a *name lambda* taking the same coordinates to `add_named_variables`:

```cpp
auto X = model.add_named_variables(
    n * n, [n](int i, int j) { return i * n + j; },
    [](int i, int j) { return std::format("x_{}_{}", i, j); });
```

Those names are assigned **lazily**, the first time each variable is accessed through `X(i, j)`, since the coordinate space cannot be enumerated ahead of time. Individual variables can also be named on the fly with `add_named_variable(name)` or `set_variable_name(v, name)`.

!!! warning "Reading the name of an unnamed entity is backend-defined"
    `get_variable_name` and `get_constraint_name` are only meaningful for an entity you named — MIP++ assigns no name by default, and the solvers do not agree on what an unnamed entity is called. Observed on a fresh model holding one unnamed variable: Cbc, Clp, COPT, CPLEX, GLPK, MOSEK and SCIP return an empty string; Gurobi and Xpress return a name the solver generated for itself (`C0`, `C1`); HiGHS raises a solver error.

    Keep your own mapping if you need names to round-trip.

## Next

[Expressions and constraints](expressions.md) — how these handles combine into objectives and whole constraint families over your index sets.
