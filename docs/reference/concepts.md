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

Four aliases name the types a model works with, and they are the only
portable spelling — model classes declare no public member types:

- `model_variable_t<M>` and `model_constraint_t<M>`, the lightweight
  strongly-typed handles, deduced from what `add_variable()` and
  `add_constraint(...)` return;
- `model_scalar_t<M>`, the coefficient type (`double` on current backends),
  deduced from the variable handle's own linear term;
- `model_variable_params_t<M>`, the designated-initializer options struct
  `{.obj_coef, .lower_bound, .upper_bound}`, deduced from the model's public
  `default_variable_params` constant.

Because they are deduced from the API rather than read from a member, any
type that provides those functions — a wrapper forwarding to a backend, say —
satisfies the concepts without declaring anything else.

## Concepts on callback handles

A callback handle (the `candidate_solution_callback_handle` a backend passes
to your callback) is not a model: it creates no variables, so the four aliases
cannot be deduced from it. The capability concepts that make sense on a handle
therefore take a second, defaulted parameter naming the model whose variables
and constraints the handle works with:

```cpp
template <typename T, typename M = T>
concept has_lazy_constraints = ...;   // T is checked, M supplies the types
```

With one argument the concept applies to a model as usual. With two, the first
is the type being checked and the second is the model it belongs to:

```cpp
static_assert(has_lazy_constraints<
    candidate_solution_callback_handle_t<gurobi_milp>, gurobi_milp>);

// the checked type is inserted first, so the shorthand reads naturally
model.set_candidate_solution_callback(
    [&](has_lazy_constraints<Model> auto & handle) { ... });
```

The concepts declared this way are `has_dual_solution`, `has_reduced_costs`,
`has_readable_variable_bounds`, `has_modifiable_variable_bounds`,
`has_readable_constraints` and its three finer-grained forms,
`has_readable_constraint_bounds`, `has_modifiable_constraint_bounds`,
`has_enumerable_variables`, `has_enumerable_constraints` and
`has_lazy_constraints`. The others describe whole models and stay
single-parameter.

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
| `lp_model` | The modeling core: `set_minimization` / `set_maximization`; `add_variable(s)` (with optional `variable_params`, over a count, a count and an id-lambda, or a key range); `set_objective` / `set_objective_offset`; `add_constraint` / `add_constraints`; `num_variables` / `num_constraints`; `infinity()` / `is_infinite(v)` (the solver's own "no bound" threshold and the portable test for it, see [Bounds](../modeling/variables.md#bounds-and-objective-coefficient)); `solve`; `get_status`; `get_solution` / `get_solution_value`. |
| `milp_model` | `lp_model`, plus `add_integer_variable(s)`, `add_binary_variable(s)`, and per-variable type changes `set_continuous` / `set_integer` / `set_binary`. |
| `qp_model` | `lp_model`, plus `set_quadratic_objective(expr)` (and its `distinct_variables` form) accepting a quadratic expression. `set_objective` stays linear on every model and replaces the whole objective, quadratic part included. |
| `has_num_nonzeros` | `num_nonzeros()`, the number of coefficients in the constraint matrix. |
| `has_enumerable_variables` | `variables()`, every live variable once, in increasing id order, as a random-access range whose `std::ranges::size` is `num_variables()`. The range is a snapshot of the call: it holds no reference to the model, lists nothing added later, and still lists a variable removed since, whose handle is then dead. Only that range interface is portable: backends that never remove variables return the lazy range of a bulk addition, the others a `std::vector`. Satisfied by every model, but not part of `lp_model`, so a model written against `lp_model` need not provide it. See [Enumerating the model](../modeling/variables.md#enumerating-the-model). |
| `has_enumerable_constraints` | `constraints()`, the same for constraints, sized by `num_constraints()`. Satisfied by every model, and not part of `lp_model` either. |

## Solve status

`get_status()` is required by `lp_model` itself: every model class returns a `std::variant` over the tag hierarchy of namespace `status` (`optimal` and its refinements, `infeasible_or_unbounded` with its refinements `infeasible` and `unbounded`, `interrupted`, `failed`, `numerical_failure`, `out_of_memory`, `limit_reached` and its five refinements, `unknown`). Query it with `is<S>(r)` (exact tag), `is_a<S>(r)` (whole branch) and `status::solution_available(r)`; the variant type is `model_status_t<M>`. `is`, `is_a` and the `variant_*` concepts behind them live in `utility/variant.hpp` and serve the basis statuses too. Three concepts refine what a given model class can report or do with it:

| Concept           | Provides |
| :---------- | --- |
| `has_lp_status` | The status variant can report `infeasible` and `unbounded` as distinct tags, not only the coarse `infeasible_or_unbounded`. |
| `has_refinable_lp_status` | The variant carries the exact `infeasible_or_unbounded` tag and the model provides `refine_lp_status()` to resolve it into `infeasible` or `unbounded` — possibly by re-solving; a no-op on any other status. |
| `has_status_reset` | `reset_status()`, `noexcept`, after which `get_status()` reports `unknown`, without a solution, until the next `solve()`. Only the reported status changes: the model's data and the solver's warm start stay. Satisfied by every model class, but not part of `lp_model`. See [Resetting the status](../solving/status-and-limits.md#resetting-the-status). |

See [Status, limits and tolerances](../solving/status-and-limits.md) for the
hierarchy and how to branch on it.

## Limits

| Concept | Provides |
| :--- | :--- |
| `has_time_limit` | `set_time_limit(std::chrono duration)`, `get_time_limit()`, which never returns a negative duration. |
| `has_iteration_limit` | `set_iteration_limit(n)`, `get_iteration_limit()`. |
| `has_node_limit` | `set_node_limit(n)`, `get_node_limit()`. |
| `has_solution_limit` | `set_solution_limit(n)`, `get_solution_limit()`. |
| `has_memory_limit` | `set_memory_limit(size)` for any `memory_size` unit, `get_memory_limit()`. |

Each limit concept additionally requires that the matching `status::*_limit`
tag is among those the backend's `get_status()` can return — a limit you can
set is a limit you can detect.

## Solution information

|      Concept      | Provides |
| --- | --- |
| `has_dual_solution` | `get_dual_solution()`, indexed by constraint handles. |
| `has_reduced_costs` | `get_reduced_costs()`, indexed by variable handles. |
| `has_lp_basis` | `get_basis()`, whose `get_status(v)` / `get_status(c)` report the LP basis as a variant over the tags of namespace `basis_status`. *(no backend yet)* |
| `has_modifiable_lp_basis` | `has_lp_basis`, and the basis returned by `get_basis()` can be edited in place: `set_basic`, `set_nonbasic(v, value)` (snaps to the nearest bound) and `set_status(v, tag)`, each for variables and for constraints. *(no backend yet)* |
| `has_lp_basis_warm_start` | `has_lp_basis`, plus `set_basis(b)` accepting the basis type of `get_basis()`. Warm-starting does not require that type to be modifiable, and a backend may accept other basis sources as well, such as a basis view built from lambdas. *(no backend yet)* |

## Reading and modifying the model

|      Concept      | Provides |
| :--- | :--- |
| `has_readable_objective` | `get_objective()`, `get_objective_coefficient(v)`, `get_objective_offset()`. |
| `has_modifiable_objective` | `set_objective_coefficient(v, s)`, `add_to_objective(expr)` and its `distinct_variables` form. |
| `has_readable_quadratic_objective` | `has_readable_objective`, plus `get_quadratic_objective()` returning the whole objective as a quadratic expression; on such a model `get_objective()` reads the linear part only. Satisfied by `highs_qp`. |
| `has_readable_variable_bounds` | `get_variable_lower_bound(v)`, `get_variable_upper_bound(v)`. |
| `has_modifiable_variable_bounds` | `set_variable_lower_bound(v, s)`, `set_variable_upper_bound(v, s)`. |
| `has_readable_constraints` | `get_constraint(c)` plus the three finer-grained concepts `has_readable_constraint_lhs` / `_sense` / `_rhs`. |
| `has_readable_constraint_bounds` | `get_constraint_lower_bound(c)`, `get_constraint_upper_bound(c)` — defined on every row, including [ranged](../modeling/special-constraints.md#ranged-constraints) ones, where `get_constraint_sense` / `_rhs` are not. A missing side reads as the backend's `infinity()` with its sign, so a `<=` row reads (`-infinity()`, rhs), a `>=` row (rhs, `infinity()`) and an `==` row (rhs, rhs). Satisfied by every model class, but not part of `lp_model`. |
| `has_modifiable_constraint_bounds` | `set_constraint_lower_bound(c, s)`, `set_constraint_upper_bound(c, s)` — either side of any row, ranged ones included; `infinity()` frees a side. A row left with two distinct finite sides, such as a `<=` row given a finite lower side or an `==` row with one side moved, is ranged. Satisfied by `clp_lp`. |
| `has_modifiable_constraint_lhs` / `_sense` / `_rhs` | `set_constraint_lhs(c, entries)`, `set_constraint_sense(c, s)`, `set_constraint_rhs(c, s)`. |

See [Re-solving and model updates](../solving/updates.md).

## Names

| Concept                                | Provides |
| :--- | :--- |
| `has_named_variables` | `set_variable_name` / `get_variable_name`, `add_named_variable(s)` (including the [lazily-named](../modeling/variables.md#names) id-lambda + name-lambda form), and `add_variables` over keys wrapped with `named(keys, name)`. Reading the name of an entity you never named is [backend-defined](../modeling/variables.md#names). |
| `has_named_constraints` | `set_constraint_name` / `get_constraint_name`. |

## Special constraints

| Concept | Provides |
| --- | --- |
| `has_indicator_constraints` | `add_indicator_constraint(v, value, constraint)` and its `distinct_variables` form — the constraint holds whenever binary variable `v` takes `value`. Satisfied by `gurobi_milp` and `cplex_milp`; see [Special constraints](../modeling/special-constraints.md). |
| `has_sos1_constraints` | `add_sos1_constraint(variables)`. *(no backend yet)* |
| `has_sos2_constraints` | `add_sos2_constraint(variables)`. *(no backend yet)* |
| `has_ranged_constraints` | `add_ranged_constraint(expr, lb, ub)` and the `distinct_variables` form: `lb <= expr <= ub` as a single row, returning the usual `constraint` handle. Satisfied by `clp_lp` and `cbc_milp`; see [Ranged constraints](../modeling/special-constraints.md#ranged-constraints). |

Neither the SOS nor the indicator functions require a return type. SOS and indicator constraints live outside the linear-row numbering on most solvers, so the `constraint` handle returned by `add_constraint` could not designate them; a backend may return a handle type of its own, or nothing, and a solver-generic caller must not rely on one.

## Algorithmic building blocks

| Concept | Provides |
| --- | --- |
| `has_column_generation` | `add_column(entries, params)` from `(constraint, coefficient)` pairs — see [Column generation](../algorithms/column-generation.md). |
| `has_remove_variable` | `remove_variable(v)`, `remove_variables(range)`. |
| `has_mip_start` | `add_mip_start(entries)` from `(variable, value)` pairs. |
| `has_candidate_solution_callback` | `set_candidate_solution_callback(f)` where `f` takes the backend's `candidate_solution_callback_handle`, whose `get_solution()` returns the candidate indexed by the model's variable handles and `get_solution_value()` its objective value — see [Branch-and-cut](../algorithms/branch-and-cut.md). |
| `has_lazy_constraints` | On a callback handle: `add_lazy_constraint(constraint)` and the `distinct_variables` form, taking the model as second parameter (see [above](#concepts-on-callback-handles)). Satisfied by the handles of `gurobi_milp`, `cplex_milp` and `copt_milp`. |
| `has_candidate_solution_rejection` | On a callback handle: `reject_solution()` discards the candidate without adding a constraint. Satisfied by the handle of `xpress_milp`. |
| `has_node_relaxation_callback` | `set_node_relaxation_callback(f)`, for user cuts on fractional solutions. *(no backend yet)* |

## Escape hatch

| Concept | Provides |
| --- | --- |
| `has_native_handles` | `native_api()`, the loaded `*_api` object (the solver's raw C functions); `native_model()`, the solver's own model objects; `native_id(v)` and `native_id(c)`, what the solver calls a variable or a constraint. Satisfied by every model class; see [Solver-specific parameters](../solvers/index.md#feature-support). |

## Tolerances

| Concept | Provides |
| --- | --- |
| `has_feasibility_tolerance` | `get`/`set_feasibility_tolerance`. |
| `has_optimality_tolerance` | `get`/`set_optimality_tolerance`. |
| `has_integrality_tolerance` | `get`/`set_integrality_tolerance`. |

## Verbosity

| Concept | Provides |
| --- | --- |
| `has_verbosity` | `set_verbose(bool)`, `is_verbose()`. Models start quiet; a verbose model prints the solver's log on standard output. Satisfied by every model class; see [Solver output](../solving/status-and-limits.md#solver-output). |

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
