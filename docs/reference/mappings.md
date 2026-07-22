# Mappings

Everywhere MIP++ hands values back — solutions, dual values, reduced costs — or folds an expression against values, it speaks one protocol: **`map[key]` reads a stored value**. This page explains the layer behind that protocol — the concepts generic code should constrain on, the adaptor that lifts arbitrary storage into it, and the entity-keyed layer the solvers use.

The whole layer lives in
[`mapping.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/mapping.hpp)
(solver-agnostic) and the `entity_mapping` part of
[`model_entities.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/model_entities.hpp).
[`test/mapping.cpp`](https://github.com/fhamonic/mippp/blob/main/test/mapping.cpp)
is the authoritative example set.

## The concepts

`mapping<Map, Key>` is duck-typed: `map[key]` must be well-formed, nothing more. On top of it:

- `input_mapping<Map, Key>` — readable through a `const Map`;
- `output_mapping<Map, Key>` — additionally, `map[key] = value` assigns through a real lvalue;
- `contiguous_mapping<Map, Key>` — additionally, integral keys and a `.data()` pointer, for handing the storage to a C API;
- each has an `_of<Map, Key, Value>` variant that also pins the mapped value type.

Constrain generic code on the weakest one that suffices — `evaluate` requires only `input_mapping`.

A subtlety worth relying on: `std::map<K, V>` *is* a `mapping` but is **not** an `input_mapping`, because a const `std::map` has no `operator[]`. Adaptation (below) is what makes it readable const — through `.at()`.

## Adapting storage: `views::mapping_all`

`views::mapping_all(m)` is to mappings what `std::views::all` is to ranges: it lifts anything usable as a mapping into a view with the `[]` protocol, and is the identity on things that already are such views.

- an **lvalue** is *referenced* — the view is a `mapping_ref_view`, and the operand must outlive it, like any `ref_view`;
- an **rvalue** is *moved into* a `mapping_owning_view`;
- a mapping view passes through unchanged.

Inside the view, a lookup dispatches to the first protocol the storage supports, in this order: `m[key]` (vectors, arrays, `std::unique_ptr<T[]>`), then `m(key)` (callables), then `m.at(key)` (associative containers). The order is observable in one place: a *non-const* `std::map` is reached through its `operator[]`, so reading a missing key inserts it — exactly `std::map`'s own semantics — while a *const* `std::map` reads through `.at()` and throws on a missing key.

Reads return references, never copies, and writes go through:

```cpp
std::vector<double> values{1.0, 2.0};
auto view = mippp::views::mapping_all(values);
view[0u] = 3.0;         // writes into `values`
double & v = view[1u];  // a reference, not a copy
```

Constness follows the `std::ranges` precedent: a `mapping_ref_view` is *shallow*-const (constness is carried by the referenced type — adapt a `const` lvalue to get const access), an owning view is *deep*-const.

Owning views stay `std::movable` even when they own a capturing lambda: assignment reconstructs the closure in place, and is only enabled when the move cannot throw. `views::map(f)` is the explicit spelling for lifting a callable; it copies an lvalue callable and moves an rvalue one:

```cpp
auto squares = mippp::views::map([](int i) { return i * i; });
static_assert(mippp::input_mapping_of<decltype(squares), int, int>);
```

`true_map`, `false_map`, `identity_map` and `element_map<I...>` (chained `std::get`) are ready-made mapping views for the common projections.

## Entity-keyed mappings

Model entities (variables, constraints) are strong types, so raw storage doesn't understand them. `entity_mapping<Entity, Map>` is the thin projection layer on top of `mapping_all`: a lookup passes the entity itself when the storage understands it — callables taking the entity, associative maps keyed by it — and otherwise falls back to the entity's `uid()`:

```cpp
using var = mippp::model_variable<int, double>;
mippp::entity_mapping<var, std::vector<double>> values =
    std::vector<double>{1.0, 2.0};
values[var(0)] = 3.0;  // uid() indexes the vector; writes go through
```

This is exactly what every solver's `get_solution()` / `get_dual_solution()` / `get_reduced_costs()` returns (as its `variable_mapping` / `constraint_mapping` aliases), which is why `sol[X(i, j)]` works uniformly whether the backend produced a flat array or a remapping closure.

## Diagnostics

Storage that supports none of the three protocols is rejected with a message, not a template backtrace:

> MIP++: this type cannot be used as a mapping with this key; it must provide operator[](key), operator()(key) or at(key).

`evaluate(expr, values_map)` checks its values map up front the same way:

> MIP++: evaluate needs a values map readable by the expression's variables; adapt raw storage or a callable with views::mapping_all or an entity_mapping.

The remedy it names is real: a bare lambda is not a `mapping` (no `operator[]`), so pass `views::mapping_all(lambda)` — or an `entity_mapping` — instead of the lambda itself.
