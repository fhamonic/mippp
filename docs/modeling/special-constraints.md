# Special constraints

Beyond linear rows, MILP models routinely need logical conditions — "this constraint applies only if that binary is on", "at most one of these is nonzero", "this variable is fixed". This page covers what MIP++ exposes natively, what to encode by hand, and how to keep such a model portable.

## Fixing and freeing variables

The cheapest special constraint is not a constraint at all. Bounds are modifiable on any backend satisfying `has_modifiable_variables_bounds`:

```cpp
model.set_variable_lower_bound(x, 1.0);   // fix x to 1 together with...
model.set_variable_upper_bound(x, 1.0);
```

Fixing through bounds rather than adding an equality row keeps the matrix unchanged, which matters when the same model is re-solved many times — branching by hand, diving heuristics, or scenario loops. See [Re-solving and model updates](../solving/updates.md).

## Indicator constraints

An indicator constraint states that a linear constraint holds whenever a binary variable takes a given value:

```cpp
// if z = 1 then  x + y <= 10
model.add_indicator_constraint(z, true, x + y <= 10);
```

The third argument is an ordinary constraint expression, so everything from the [expression layer](expressions.md) — `xsum`, filters, coefficient arithmetic — is available inside it. The member function is provided by **`gurobi_milp`** and **`cplex_milp`** (see the [caveat](#one-model-both-encodings) about the matching concept below).

Indicators are usually preferable to a big-M encoding when the solver supports them: no M has to be chosen, and the solver's own logic avoids the numerical weakness of a large coefficient.

## Big-M: the portable encoding

When the target backend has no indicator support — or when you deliberately want a single formulation across all 11 backends — write the classical implication by hand:

```cpp
// z = 1  =>  a·x <= b     (M an upper bound on a·x - b)
model.add_constraint(a_x_expr <= b + M * (1 - z));
```

Two practical points, both of which MIP++ makes easy to control:

- **Keep M tight and data-derived.** Since the expression is built where it is used, the bound can be computed from the same data in the same statement, per constraint of a family:

    ```cpp
    model.add_constraints(jobs, [&](int j) {
        const double M = horizon - duration[j];
        return start(j) <= due[j] + M * (1 - late(j));
    });
    ```

- **Watch the feasibility tolerance.** A large M multiplies the effect of the solver's constraint tolerance; if a "logical" constraint appears violated in a solution, compare M against `get_feasibility_tolerance()` before suspecting a modeling bug (see [Status, limits and tolerances](../solving/status-and-limits.md#tolerances)).

## One model, both encodings

You rarely want to choose once and for all: the same experiment may run on Gurobi on your machine and on HiGHS or SCIP on the cluster. Write the implication as a small helper that branches on the capability, and the model builder stays a single piece of code:

```cpp
// z = 1  =>  lhs <= rhs.
// Native indicator where the backend has one, big-M everywhere else.
template <milp_model Model, linear_expression E>
void add_implication(Model & model, model_variable_t<Model> z, E && lhs,
                     model_scalar_t<Model> rhs, model_scalar_t<Model> big_m) {
    if constexpr(has_indicator_constraints<Model>)
        model.add_indicator_constraint(z, true, std::forward<E>(lhs) <= rhs);
    else
        model.add_constraint(std::forward<E>(lhs) <= rhs + big_m * (1 - z));
}
```

The discarded branch is never instantiated, so the helper compiles on every backend, and `big_m` is simply ignored where the native form is used. Callers read the same in both worlds — including inside a constraint family, where M comes from the same data as the row:

```cpp
for(int j : jobs)
    add_implication(model, late(j), start(j), due[j], horizon - due[j]);

add_implication(model, z, xsum(items, [&](int i) { return X(i); }), 2.0,
                double(items.size()));
```

Two details make this work smoothly:

- **Take the expression by forwarding reference** (`E &&`) and `std::forward` it. Expressions are views, some of them single-use; this is the general rule for any function that accepts one (see [Inside the expression layer](../reference/expression-layer.md)).
- **Keep the big-M argument even on indicator backends.** Passing it costs nothing at runtime and keeps one call signature; computing it from data you already have is a one-liner at the call site.

The same shape generalises to every optional capability — `has_sos1_constraints`, `has_mip_start`, `has_time_limit` — and is developed in [Writing solver-generic code](../solvers/generic-code.md).

!!! warning "`has_indicator_constraints` is currently `false` on every backend"
    The concept requires `add_indicator_constraint` to **return** a constraint handle, while `gurobi_milp` and `cplex_milp` declare it returning `void`. Their member function is perfectly usable when called directly — the example at the top of this section compiles and runs on both — but the concept they were meant to satisfy is not, so the `if constexpr` above takes the big-M branch everywhere for now.

    That makes the helper the right thing to write today: it is correct on every backend, and it starts emitting native indicators on Gurobi and CPLEX the moment the signatures line up, with no change at any call site.

## Logical conditions between binaries

Ordinary linear rows cover the usual propositional patterns, and read well with `xsum`:

| Condition | Row |
| :--- | :--- |
| `a ⇒ b` | `a <= b` |
| at most one of a set | `xsum(S, Z) <= 1` |
| exactly one | `xsum(S, Z) == 1` |
| `y = a ∧ b` | `y <= a`, `y <= b`, `y >= a + b - 1` |
| `y = a ∨ b` | `y >= a`, `y >= b`, `y <= a + b` |

## SOS constraints

`has_sos1_constraints` and `has_sos2_constraints` are declared in [`model_concepts.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/model_concepts.hpp) and specify `add_sos1_constraint(variables)` / `add_sos2_constraint(variables)` over a range or initializer list of variables.

!!! warning "Not yet implemented by any backend"
    As of the current release no backend model class provides these functions, so `has_sos1_constraints<M>` and `has_sos2_constraints<M>` are `false` everywhere; they are on the [roadmap](https://github.com/fhamonic/mippp#roadmap). Until they land, encode SOS1 with binaries (`x_i <= u_i z_i`, `xsum(S, Z) <= 1`) and SOS2 with the classical adjacency formulation — or check the concept in your generic code so it starts using the native form as soon as a backend provides it:

    ```cpp
    if constexpr(has_sos1_constraints<decltype(model)>)
        model.add_sos1_constraint(group);
    else
        add_sos1_by_binaries(model, group);
    ```

## Quadratic and conic constraints

Quadratic *objectives* are supported on HiGHS (see [Objectives](objectives.md#quadratic-objectives)). Quadratic **constraints** (QCP) and second-order cones are not part of the modeling interface yet; they are on the [roadmap](https://github.com/fhamonic/mippp#roadmap).

## Next

The model is built — on to [Status, limits and tolerances](../solving/status-and-limits.md).
