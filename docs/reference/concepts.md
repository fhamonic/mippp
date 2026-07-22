# Model concepts

MIP++'s portable interface is specified by C++ concepts, defined in
[`mippp/model_concepts.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/model_concepts.hpp).
The *model* concepts describe what every backend of a given problem class
provides; the *capability* concepts describe optional features that vary per
backend (see the [feature matrices](../solvers/index.md#feature-support)).

Use them to make generic code self-documenting and to fail at compile time
instead of at runtime:

```cpp
template <typename M>
    requires milp_model<M> && has_candidate_solution_callback<M>
void solve_tsp(M & model, const instance & data);

static_assert(has_dual_solution<highs_lp>);
```

[Writing solver-generic code](../solvers/generic-code.md) shows the patterns;
this page is the catalogue.

Every model type also exposes the member types used throughout the interface:
`M::variable` and `M::constraint` (lightweight strongly-typed handles),
`M::scalar` (the coefficient type, `double` on current backends), and
`M::variable_params` (the designated-initializer options struct
`{.obj_coef, .lower_bound, .upper_bound}`). The aliases
`model_variable_t<M>`, `model_constraint_t<M>`, `model_scalar_t<M>` and
`model_variable_params_t<M>` name them without a `typename`.

!!! note "Concept declared ≠ backend provides"
    A few concepts below are specified but satisfied by **no backend yet**;
    they are marked *(no backend yet)* and appear on the
    [roadmap](https://github.com/fhamonic/mippp#roadmap). They are still worth
    using today in `if constexpr` branches and `requires` clauses — generic
    code written against them starts using the native path the day a backend
    provides it.

## Model concepts

| Concept           | Requires |
| :---------------- | :------- |
| `lp_model` | The modeling core: `set_minimization` / `set_maximization`; `add_variable(s)` (with optional `variable_params` and id-lambdas); `set_objective` / `set_objective_offset`; `add_constraint` / `add_constraints`; `num_variables` / `num_constraints`; `solve`; `solve_status`; `get_solution` / `get_solution_value`. |
| `milp_model` | `lp_model`, plus `add_integer_variable(s)`, `add_binary_variable(s)`, and per-variable type changes `set_continuous` / `set_integer` / `set_binary`. |
| `qp_model` | `lp_model`, plus `set_objective` accepting a quadratic expression. |
| `sized_model` | `num_entries()` (number of nonzeros). |

## Solve status

`solve_status()` is required by `lp_model` itself: every model class returns a `std::variant` over the tag hierarchy of namespace `status` (`optimal` and its refinements, `infeasible_or_unbounded` with its refinements `infeasible` and `unbounded`, `interrupted`, `failed`, `numerical_failure`, `out_of_memory`, `limit_reached` and its five refinements, `unknown`). Query it with `is<S>(r)` (exact tag), `is_a<S>(r)` (whole branch) and `status::solution_available(r)`; the variant type is `model_solve_status_t<M>`. Two concepts refine what a given model class can report:

| Concept           | Provides |
| :---------- | --- |
| `has_lp_status` | The status variant can report `infeasible` and `unbounded` as distinct tags, not only the coarse `infeasible_or_unbounded`. |
| `has_refinable_lp_status` | The variant carries the exact `infeasible_or_unbounded` tag and the model provides `refine_lp_status()` to resolve it into `infeasible` or `unbounded` — possibly by re-solving; a no-op on any other status. |

See [Status, limits and tolerances](../solving/status-and-limits.md) for the
hierarchy and how to branch on it.

## Limits

| Concept | Provides |
| :--- | :--- |
| `has_time_limit` | `set_time_limit(std::chrono duration)`, `get_time_limit()`. |
| `has_iteration_limit` | `set_iteration_limit(n)`, `get_iteration_limit()`. |
| `has_node_limit` | `set_node_limit(n)`, `get_node_limit()`. |
| `has_solution_limit` | `set_solution_limit(n)`, `get_solution_limit()`. |
| `has_memory_limit` | `set_memory_limit(size)` for any `memory_size` unit, `get_memory_limit()`. |

Each limit concept additionally requires that the matching `status::*_limit`
tag is among those the backend's `solve_status()` can return — a limit you can
set is a limit you can detect.

## Solution information

|      Concept      | Provides |
| --- | --- |
| `has_dual_solution` | `get_dual_solution()`, indexed by constraint handles. |
| `has_reduced_costs` | `get_reduced_costs()`, indexed by variable handles. |
| `has_lp_basis` | `get_basis()`, whose `is_basic(v/c)` and `get_status(v/c)` report the LP basis (statuses in namespace `basis_status`). *(no backend yet)* |
| `has_lp_basis_warm_start` | `has_lp_basis`, plus `set_basis(b)` and the basis mutators `set_basic` / `set_nonbasic` / `set_status`. *(no backend yet)* |

## Reading and modifying the model

|      Concept      | Provides |
| :--- | :--- |
| `has_readable_objective` | `get_objective()`, `get_objective_coefficient(v)`, `get_objective_offset()`. |
| `has_modifiable_objective` | `set_objective_coefficient(v, s)`, `add_objective(expr)`. |
| `has_readable_variables_bounds` | `get_variable_lower_bound(v)`, `get_variable_upper_bound(v)`. |
| `has_modifiable_variables_bounds` | `set_variable_lower_bound(v, s)`, `set_variable_upper_bound(v, s)`. |
| `has_readable_constraints` | `get_constraint(c)` plus the three finer-grained concepts `has_readable_constraint_lhs` / `_sense` / `_rhs`. |
| `has_modifiable_constraint_lhs` / `_sense` / `_rhs` | `set_constraint_lhs(c, entries)`, `set_constraint_sense(c, s)`, `set_constraint_rhs(c, s)`. |

See [Re-solving and model updates](../solving/updates.md).

## Names

| Concept                                | Provides |
| :--- | :--- |
| `has_named_variables` | `set_variable_name` / `get_variable_name`, `add_named_variable(s)` (including the [lazily-named](../modeling/variables.md#names-assigned-lazily) id-lambda + name-lambda form). |
| `has_named_constraints` | `set_constraint_name` / `get_constraint_name`. |

## Special constraints

| Concept | Provides |
| --- | --- |
| `has_indicator_constraints` | `add_indicator_constraint(v, value, constraint)` returning a constraint handle — the constraint holds whenever binary variable `v` takes `value`. *(no backend yet: `gurobi_milp` and `cplex_milp` provide the function, but declare it `void`, so the concept is `false`; see [Special constraints](../modeling/special-constraints.md#one-model-both-encodings))* |
| `has_sos1_constraints` | `add_sos1_constraint(variables)`. *(no backend yet)* |
| `has_sos2_constraints` | `add_sos2_constraint(variables)`. *(no backend yet)* |

## Algorithmic building blocks

| Concept | Provides |
| --- | --- |
| `has_add_column` | `add_column(entries, params)` from `(constraint, coefficient)` pairs — see [Column generation](../algorithms/column-generation.md). |
| `has_remove_variable` | `remove_variable(v)`, `remove_variables(range)`. |
| `has_mip_start` | `add_mip_start(entries)` from `(variable, value)` pairs. |
| `has_candidate_solution_callback` | `set_candidate_solution_callback(f)` where `f` takes the backend's `candidate_solution_callback_handle` — see [Branch-and-cut](../algorithms/branch-and-cut.md). |
| `has_node_relaxation_callback` | `set_node_relaxation_callback(f)`, for user cuts on fractional solutions. *(no backend yet)* |

## Tolerances

| Concept | Provides |
| --- | --- |
| `has_feasibility_tolerance` | `get`/`set_feasibility_tolerance`. |
| `has_optimality_tolerance` | `get`/`set_optimality_tolerance`. |
| `has_integrality_tolerance` | `get`/`set_integrality_tolerance`. *(no backend yet)* |

## Expression concepts

The expression layer has concepts of its own, defined in
[`linear_expression.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/linear_expression.hpp)
and
[`linear_constraint.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/linear_constraint.hpp):

- `linear_expression` — anything with `linear_terms()` (a range of
  `(variable, coefficient)` pairs) and `constant()`. Variable handles,
  `xsum` results, and operator combinations all satisfy it, and model
  functions accept *any* type that does — you can pass your own expression
  types.
- `linear_constraint` — anything with `linear_terms()`, `sense()`
  (`constraint_sense::less_equal` / `equal` / `greater_equal`), and `rhs()`.

These are the extension points: a function like `add_constraint` is written
against the concept, never against a concrete expression class. Their
sub-concepts — ownership, multipass, and the diagnostics they drive — are
covered in [Inside the expression layer](expression-layer.md).
