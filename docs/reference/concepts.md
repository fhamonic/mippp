# Model concepts

MIP++'s portable interface is specified by C++ concepts, defined in
[`mippp/model_concepts.hpp`](https://github.com/fhamonic/mippp/blob/main/include/mippp/model_concepts.hpp).
The three *model* concepts describe what every backend of a given problem
class provides; the *capability* concepts describe optional features that
vary per backend (see the
[feature matrices](../getting-started/solvers.md#feature-support)).

Use them to make generic code self-documenting and to fail at compile time
instead of at runtime:

```cpp
template <typename M>
    requires milp_model<M> && has_candidate_solution_callback<M>
void solve_tsp(M & model, const instance & data);

static_assert(has_dual_solution<highs_lp>);
```

Every model type also exposes the member types used throughout the interface:
`M::variable` and `M::constraint` (lightweight strongly-typed handles),
`M::scalar` (the coefficient type, `double` on current backends), and
`M::variable_params` (the designated-initializer options struct
`{.obj_coef, .lower_bound, .upper_bound}`).

## Model concepts

| Concept           | Requires |
| :---------------- | :------- |
| `lp_model` | The modeling core: `set_minimization` / `set_maximization`; `add_variable(s)` (with optional `variable_params` and id-lambdas); `set_objective` / `set_objective_offset`; `add_constraint` / `add_constraints`; `num_variables` / `num_constraints`; `solve`; `get_solution` / `get_solution_value`. |
| `milp_model` | `lp_model`, plus `add_integer_variable(s)`, `add_binary_variable(s)`, and per-variable type changes `set_continuous` / `set_integer` / `set_binary`. |
| `qp_model` | `lp_model`, plus `set_objective` accepting a quadratic expression. |
| `sized_model` | `lp_model`, plus `num_entries()` (number of nonzeros). |

## Solve status and limits

| Concept           | Provides |
| :---------- | --- |
| `has_lp_status` | `proven_optimal()`, `proven_infeasible()`, `proven_unbounded()`. |
| `has_termination_reason` | `termination_reason()`, returning one of the tag types in namespace `reason` (`optimal`, `infeasible`, `unbounded`, `infeasible_or_unbounded`, `numerical_failure`, `time_limit`, `node_limit`, `unknown`). |
| `has_time_limit` | `set_time_limit(std::chrono duration)`, `get_time_limit()`. |
| `has_node_limit` | `set_node_limit(n)`, `get_node_limit()`. |

## Solution information

|      Concept      | Provides |
| --- | --- |
| `has_dual_solution` | `get_dual_solution()`, indexed by constraint handles. |
| `has_reduced_costs` | `get_reduced_costs()`, indexed by variable handles. |

## Reading and modifying the model

|      Concept      | Provides |
| :--- | :--- |
| `has_readable_objective` | `get_objective()`, `get_objective_coefficient(v)`, `get_objective_offset()`. |
| `has_modifiable_objective` | `set_objective_coefficient(v, s)`, `add_objective(expr)`. |
| `has_readable_variables_bounds` | `get_variable_lower_bound(v)`, `get_variable_upper_bound(v)`. |
| `has_modifiable_variables_bounds` | `set_variable_lower_bound(v, s)`, `set_variable_upper_bound(v, s)`. |
| `has_readable_constraints` | `get_constraint(c)` plus the three finer-grained concepts `has_readable_constraint_lhs` / `_sense` / `_rhs`. |
| `has_modifiable_constraint_lhs` / `_sense` / `_rhs` | `set_constraint_lhs(c, entries)`, `set_constraint_sense(c, s)`, `set_constraint_rhs(c, s)`. |

## Names

| Concept                                | Provides |
| :--- | :--- |
| `has_named_variables` | `set_variable_name` / `get_variable_name`, `add_named_variable(s)` (including the [lazily-named](../getting-started/expressions.md#lambda-indexed-variables) id-lambda + name-lambda form). |
| `has_named_constraints` | `set_constraint_name` / `get_constraint_name`. |

## Special constraints

| Concept | Provides |
| --- | --- |
| `has_sos1_constraints` | `add_sos1_constraint(variables)`. |
| `has_sos2_constraints` | `add_sos2_constraint(variables)`. |
| `has_indicator_constraints` | `add_indicator_constraint(v, value, constraint)` — the constraint holds whenever binary variable `v` takes `value`. |

## Algorithmic building blocks

| Concept | Provides |
| --- | --- |
| `has_add_column` | `add_column(entries, params)` from `(constraint, coefficient)` pairs — see [Column generation](../advanced/column-generation.md). |
| `has_remove_variable` | `remove_variable(v)`, `remove_variables(range)`. |
| `has_mip_start` | `add_mip_start(entries)` from `(variable, value)` pairs. |
| `has_candidate_solution_callback` | `set_candidate_solution_callback(f)` where `f` takes the backend's `candidate_solution_callback_handle` — see [Branch-and-cut callbacks](../advanced/callbacks.md). |

## Tolerances

| Concept | Provides |
| --- | --- |
| `has_feasibility_tolerance` | `get`/`set_feasibility_tolerance`. |
| `has_optimality_tolerance` | `get`/`set_optimality_tolerance`. |
| `has_integrality_tolerance` | `get`/`set_integrality_tolerance`. |

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
against the concept, never against a concrete expression class.
