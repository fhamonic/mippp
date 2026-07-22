# Writing solver-generic code

Switching backend in a single program is a two-line change ([Choosing a solver](index.md#switching-backends)). Writing a *function* — a model builder, a decomposition, an experiment driver — that works on **any** backend is barely harder, and it is what makes cross-solver benchmarking, and reusable research code, practical. This page shows the patterns.

## Take the model as a template parameter

Model classes share no base class; they share **concepts**. A generic function states which ones it needs:

```cpp
template <milp_model Model>
void build_assignment(Model & model, const instance & data) {
    using namespace mippp::operators;
    auto X = model.add_binary_variables(
        data.n * data.n, [n = data.n](int i, int j) { return i * n + j; });
    model.set_minimization();
    model.set_objective(xsum(data.pairs, [&](auto && p) {
        auto && [i, j] = p; return data.cost[i][j] * X(i, j); }));
    // ...
}
```

Called with `highs_milp`, `gurobi_milp`, `scip_milp`, … it compiles for each; called with an LP-only class, it fails immediately with *"constraint not satisfied: `milp_model`"* rather than with a page of overload-resolution noise.

Member types are reached through the model rather than spelled out:

```cpp
using variable   = model_variable_t<Model>;   // or typename Model::variable
using constraint = model_constraint_t<Model>;
using scalar     = model_scalar_t<Model>;     // double on current backends
```

This matters when you store handles: a `std::vector<typename Model::variable>` is the portable spelling of "the columns I created".

## Require the capabilities you use

Optional features are concepts too, so an algorithm can advertise its requirements in its signature:

```cpp
template <typename Model>
    requires milp_model<Model> && has_candidate_solution_callback<Model>
void solve_tsp(Model & model, const instance & data);

template <typename Model>
    requires lp_model<Model> && has_dual_solution<Model> && has_add_column<Model>
double column_generation(Model & master, const instance & data);
```

Instantiating `solve_tsp` with a backend that has no callback support is a **compile-time** error naming the missing capability — not a surprise at hour three of a run. The same check is available as an assertion:

```cpp
static_assert(has_dual_solution<highs_lp>);
```

## Degrade gracefully with `if constexpr`

When a capability improves a run but is not essential, branch on it. The unsupported branch is never instantiated, so the code still compiles for every backend:

```cpp
if constexpr(has_time_limit<Model>)     model.set_time_limit(budget);
if constexpr(has_mip_start<Model>)      model.add_mip_start(greedy_solution);
if constexpr(has_optimality_tolerance<Model>)
    model.set_optimality_tolerance(1e-6);

// resolve infeasible_or_unbounded where the backend can, before recording
if constexpr(has_refinable_lp_status<Model>) model.refine_lp_status();
record(model.solve_status());
```

This is the idiom to prefer over `#ifdef`s or per-solver overloads: the feature test lives next to the feature use, and the set of backends a function supports is deduced rather than maintained by hand.

## Selecting the backend at run time

Compile the generic function once per backend and dispatch on a string — the shape of every `--solver` command-line flag:

```cpp
template <typename Api, typename Model>
run_record run_instance(const instance & data) {
    Api api;                       // loads that solver's library, here and now
    Model model(api);
    build_assignment(model, data);
    return run(model, 600s);
}

run_record dispatch(std::string_view solver, const instance & data) {
    if(solver == "highs")  return run_instance<highs_api,  highs_milp >(data);
    if(solver == "gurobi") return run_instance<gurobi_api, gurobi_milp>(data);
    if(solver == "scip")   return run_instance<scip_api,   scip_milp  >(data);
    throw std::invalid_argument("unknown solver");
}
```

Because solver libraries are loaded when the `api` object is constructed, a binary built with all three branches runs fine on a machine that only has HiGHS installed — as long as the other branches are not taken. A missing library throws a descriptive `std::runtime_error` from the api constructor, which a benchmark driver can catch and record as "backend unavailable".

Constructing an api is the expensive step: hoist it out of the loop and build many models from the same one.

## A cross-solver benchmark

```cpp
for(auto && inst : instances) {
    for(auto && solver : {"highs", "gurobi", "cplex"}) {
        try {
            auto rec = dispatch(solver, inst);
            std::println("{},{},{},{},{}", inst.name, solver, rec.seconds,
                         rec.objective, rec.optimal);
        } catch(const std::runtime_error & e) {
            std::println("{},{},unavailable", inst.name, solver);
        }
    }
}
```

The model is built by the *same* code in every row of that table — which is the point: differences in the numbers come from the solvers, not from three modeling layers with three sets of defaults.

Model building is measured in tens of milliseconds even at a million variables, so it is negligible next to the solve; and when the *build* is what you are measuring, that is exactly what [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens) reports — see [Performance](../performance.md).

## Writing generic code over expressions

Functions that take expressions rather than models follow the concepts of the expression layer:

```cpp
template <linear_expression E>
void log_expression(E && e);          // take by forwarding reference, and forward
```

Taking `E &&` is not a stylistic choice: some expression types can only be read through a non-`const` lvalue, and some are single-use. The rules — and the diagnostics that fire when they are broken — are in [Inside the expression layer](../reference/expression-layer.md).

## Next

[Worked examples](../examples.md) — complete programs applying all of this.
