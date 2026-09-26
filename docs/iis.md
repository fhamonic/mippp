# Finding infeasible constraints

When a model has no feasible solution, an **irreducible infeasible subsystem**
(IIS) identifies a set of constraints and bounds that conflict with one another.
Removing any one member makes that subsystem feasible. An IIS helps you locate
contradictory requirements or incorrect input data; it does not choose which
requirement to change, and it need not be the smallest or only conflict.

This page documents the explicit linear-system utility. The planned model-level
`compute_iis()` API, handle-based snapshots, row-bound capability, and native IIS
routines are not implemented. See the [implementation audit](iis-implementation-status.md)
for the status of the original checklist.

MIP++ provides `compute_linear_iis` for linear and mixed-integer systems.
Supply the rows and variable bounds, choose a solver through a model factory,
and inspect the returned conflict. The default method repeatedly tries removing
constraints, so it works without a solver-specific IIS feature.

## A first conflict

This example asks for an integer `x` equal to `0.5`. Neither `x >= 0.5` nor
`x <= 0.5` is contradictory alone, but together they conflict with integrality.

```cpp
#include <iostream>
#include <optional>

#include "mippp/solvers/cbc/all.hpp"
#include "mippp/utility/linear_iis.hpp"

int main() {
    using namespace mippp::iis;

    linear_system<> system;
    system.variables = {{std::nullopt, std::nullopt, true}}; // free integer x
    system.rows = {{{{0, 1.}}, 0.5, 0.5}};                  // x == 0.5

    const auto & api = mippp::cbc_api::load();
    auto factory = [&] { return mippp::cbc_milp(api); };
    auto answer = compute_linear_iis(system, factory);

    std::cout << describe_result(answer, {.include_members = true}) << '\n';
}
```

Use the usual [installation and solver-library setup](getting-started/installation.md).
For this example, make the CBC shared library discoverable or set
`MIPPP_CBC_LIBRARY` to its path. To select another backend, change the included
solver header, API loader, and model type; use a MIP-capable model for integer
systems.

On a conclusive run, `answer.members` contains the lower and upper sides of row
0. Indices refer to your input, so you can associate them with application names
or source locations. Integrality stays fixed throughout extraction; it is not
itself a removable member.

## Reading the answer

Always check the proof status before presenting a conflict:

| Result | What you can conclude |
| --- | --- |
| `answer.reduction.initial_status == feasibility::feasible` | The supplied system is feasible in the analyzed domain. |
| `answer.reduction.proven_infeasible()` and `answer.reduction.irreducible` | The returned members form an IIS: each is necessary for this conflict. |
| `proven_infeasible()` without `irreducible` | The returned conflict is valid, but some members may still be removable. |
| Initial status `unknown` | No conflict has been verified. |

`answer.reduction.reason` reports completion, an inconclusive check, a solve
limit, a time limit, or cancellation. A stopped run can still return a verified
conflict. An empty member list alone does not establish feasibility.

Results depend on the solver's numerical tolerances. Unknown results are never
treated as proofs. An IIS is minimal by inclusion, not necessarily by number of
members; different removal orders may find different valid IISs. If you analyze
an LP relaxation, a feasible result says nothing about the original integer
problem's feasibility.

`describe_result` formats these distinctions in plain language. The
[diagnostics and statistics reference](#results-diagnostics-and-statistics)
explains the detailed fields.

## Supplying a linear system

`mippp/utility/linear_iis.hpp` supplies an explicit `linear_system<Scalar>`:
variables with optional lower/upper bounds and an integer flag, and rows with
sparse `(variable_index, coefficient)` terms and optional lower/upper bounds.
Absent bounds are `std::nullopt`; supplied values must be finite. Defaults are
free continuous variables. Row constants must already be moved to their bounds.
Row and variable indices in results refer to this input, not solver row IDs.

The factory returns a fresh empty model on each call, allowing the caller to
choose the backend and configure per-solve limits and logging. The adapter builds
each deletion trial with a zero objective. It neither mutates nor clones an existing
solver model. Both ranged rows and variable bounds are tested one side at a time.
The default strategy rebuilds each trial, paying model construction costs and
not transferring bases or MIP starts. Both deletion and elasticity can instead
use retained models, as described below.

For repeated trials, load a backend's API once and capture it in the factory,
instead of using the default model constructor each time. For example, with
`mippp/solvers/cbc/all.hpp`:

```cpp
const auto & api = mippp::cbc_api::load();
auto factory = [&] { return mippp::cbc_milp(api); };
auto answer = mippp::iis::compute_linear_iis(system, factory);
```

The API must remain alive through extraction. Loading remains caller-controlled;
the adapter does not eagerly load any solver when its budget prevents a trial.

`domain::original` preserves integer types. Irreducibility is relative to those
fixed types; integrality itself is not a removable candidate. Represent binary
variables as integers with explicit [0,1] bounds. An LP-only backend rejects
integer input unless `domain::lp_relaxation` is explicitly selected. The result
records the chosen domain. LP backends can diagnose LPs; only MIP backends can
diagnose integer-only infeasibility.

Supply the complete linear system explicitly; this API does not export an
existing MIP++ model. SOS, indicator, nonlinear, and callback constraints are
outside the linear adapter's scope. Use a custom feasibility checker with the
[generic utility](#using-your-own-feasibility-checker) for those systems.
Extraction uses repeated feasibility checks, not a solver's native IIS routine.

## Model and feature support

The table describes the LP/MILP model classes supported by this IIS adapter,
not every feature offered by the underlying solver. **F** means provided by
MIP++'s generic fallback routines; **N** means a native capability exposed by
the model wrapper and used by the adapter. **F + N** combines generic control
with native model updates or time limits. **—** means unavailable through this
path. Every extraction still uses a solver for feasibility checks.

| Model (`mippp::…`) | Analyzed problems | IIS extraction | LP certificate hint | LP elasticity prefilter | Retained deletion / elasticity | Time budget |
| --- | --- | --- | --- | --- | --- | --- |
| `highs_lp` | LP | F | — | F | F + N | F + N |
| `highs_milp` | LP, MIP | F | — | F | F + N | F + N |
| `clp_lp` | LP | F | N: row weights | F | F + N | F |
| `cbc_milp` | LP, MIP | F | — | F | F + N | F + N |
| `gurobi_lp` | LP | F | — | F | F + N | F + N |
| `gurobi_milp` | LP, MIP | F | — | F | F + N | F + N |
| `cplex_lp` | LP | F | — | F | F + N | F + N |
| `cplex_milp` | LP, MIP | F | — | F | F + N | F + N |
| `copt_lp` | LP | F | — | F | F + N | F |
| `copt_milp` | LP, MIP | F | — | F | F + N | F + N |
| `glpk_lp` | LP | F | — | F | F + N | F |
| `glpk_milp` | LP, MIP | F | — | F | F + N | F |
| `mosek_lp` | LP | F | N: row and variable-bound weights | F | F + N | F + N |
| `mosek_milp` | LP, MIP | F | — | F | F + N | F + N |
| `scip_milp` | LP, MIP | F | — | F | F + N | F |
| `soplex_lp` (stock 8.1.0) | LP | F | — | F | —; rebuilds | F |
| `soplex_lp` (patched 8.1.0) | LP | F | N: row weights | F | —; rebuilds | F |
| `xpress_lp` | LP | F | — | F | F + N | F + N |
| `xpress_milp` | LP, MIP | F | — | F | F + N | F + N |

How to read the feature columns:

- **IIS extraction:** every row uses generic deletion, including optional
  batching and candidate ordering. No native IIS or conflict-refiner routine is
  called, even when the solver itself offers one. Shared solve-count limits,
  cancellation checks, diagnostics, statistics, and optional elapsed timing are
  also provided by MIP++ for every model.
- **LP certificate hint:** native weights only suggest candidates. Generic
  support selection, optional bound screening and weight ordering, and a
  separate feasibility check turn that suggestion into a usable seed. A missing
  or rejected hint falls back to elasticity or ordinary deletion. `mosek_milp`
  does not expose this certificate accessor; use `mosek_lp` for MOSEK LP hints.
- **LP elasticity prefilter:** available for continuous input, including an
  explicitly requested LP relaxation. It and native hints are skipped for
  original-domain integer problems, even on models labeled “LP, MIP.” An
  LP-only model requires `domain::lp_relaxation` to accept integer input.
- **Retained deletion / elasticity:** the generic workspace uses native bound
  updates to keep a model between checks. Both strategies are opt-in; rebuilding
  remains available on every model. Requesting reuse on either SoPlex variant
  selects rebuilding instead. Retention allows the solver to reuse optimization
  state but does not guarantee it; explicit basis export/import is not used.
- **Time budget:** F checks the shared deadline between calls and before native
  solves. F + N also forwards the remaining duration through the model's time-limit
  interface. Neither guarantees immediate interruption; cancellation tokens do
  not interrupt an active native solve.

The [SoPlex patch](#soplex) adds optional certificate functions to the C interface;
it does not add bound updates to the MIP++ wrapper. Stock SoPlex still supports
fallback IIS extraction and rebuilt elasticity.

These entries reflect the current model interfaces and compile-time capability
checks, not a claim that every solver/version/platform combination has been
runtime-tested. See [Running validation](#running-validation) for selecting tests
for installed solvers. The input remains an explicit linear system; this table
does not cover quadratic constraints or objectives.

## Configuring extraction

Structural choices are a `constexpr linear_policy` passed as a template argument.
Per-extraction values live in one aggregate `linear_options`:

```cpp
using namespace mippp::iis;
constexpr linear_policy explain{
    .elasticity = elasticity_strategy::reuse,
    .deletion = deletion_strategy::reuse,
    .native_seed = true,
    .prune_bounds = true
};
auto answer = compute_linear_iis<explain>(system, factory, linear_options{
    .limits = {.max_solves = 100, .time_limit = std::chrono::seconds(2)},
    .elastic = {.max_solves = 8, .violation_tolerance = 1e-7},
    .native = {.relative_tolerance = 1e-9},
    .order = rows_first_order{}
});
```

The default `compute_linear_iis(system, factory)` uses original-domain rebuilding
without prefilters. Set `analyzed_domain = domain::lp_relaxation` in the policy
to relax integer types. Elasticity has `off`, `rebuild`, and `reuse` strategies;
deletion has `rebuild` and `reuse`. Native seeding, bound screening and weight
ordering are policy flags. Screening/weight ordering without native seeding,
or invalid enum policy values, are rejected by a template constraint.

Disabled feature branches and workspace storage are discarded with `if constexpr`;
unsupported optional backend capabilities select their fallback at compile time.
Comparator type is deduced from `.order`; its state may be runtime and move-only
(pass an existing move-only options object with `std::move`). Tolerances are
validated only for enabled policy features. Limits always apply. Generic
`deletion_filter` and `elasticity_filter` use their standalone `options`
type, which is nested under `linear_options::limits` for the linear adapter.

Policies reduce runtime branching and storage for a **fixed specialization**.
Instantiating many policies can increase total binary size and compile time;
keep the set of specializations small. Applications needing
runtime strategy selection should switch once around a small explicit set of
policy-specialized calls, as the benchmark does. Budgets, numerical tolerances
and comparator scores remain runtime values in the options object.

## Solve limits, time limits, and cancellation

Set `linear_options::limits` to budget the entire extraction. Its `options`
object supplies a maximum solve count (including the initial check), an
absolute steady-clock deadline, a relative `time_limit` (`std::chrono::duration<double>`),
and a stop token. The earlier of the absolute and relative limits applies;
the relative allowance is converted once at entry and shared across phases.
It defaults to infinity; zero prevents any calls, and negative/NaN values throw.
Limits are checked between feasibility checks. With a custom checker, you are
responsible for enforcing interruption or a time limit during the check itself.

`max_solves` counts oracle calls, not just native optimizer invocations: initial
verification, bound-only certificates, elastic trials, seed revalidation, fallback,
group deletion and singleton deletion all consume it. No phase gets a fresh
global allowance. On an incomplete result, simultaneous stops are reported as
cancellation first, then time, then solve count. A complete proof obtained on
the final permitted call is still `completed`, even if the clock expired during
that call. Unknown results remain unproven, including when a limit caused them.

The linear adapter also checks cancellation/deadlines after model construction
and immediately before native solves. Where the backend exposes `has_time_limit`
and a fractional-seconds setter, it forwards the remaining overall time before
every solve, including retained-model re-solves. A tighter factory-configured
limit is preserved. Unsupported backends retain cooperative between-call checks.
Native limits are best-effort: solver polling granularity, construction, and
cleanup can overrun the deadline; a stop token does not interrupt an active
native solve. A backend may account for retained-model time cumulatively and
stop earlier. An inconclusive native status before the overall deadline remains
`indeterminate` unless another global limit is exhausted; it is never an IIS
certificate.

For example, pass `{.limits = {.max_solves = 100, .time_limit = std::chrono::seconds(5)}}`
as the options argument to budget the entire extraction, not each phase.

## Optional batched deletion

Set `linear_options::limits.initial_batch_size` (for example, 64) to first try removing groups
of candidates. The filter scans with that group size, halves it, and repeats
until it reaches the final single-candidate pass. Zero and one disable batching;
oversized values are clamped to the candidate count. The default remains one.

A group is removed only after the remaining subsystem is proved infeasible.
Feasible or unknown group trials retain the whole group for later smaller
trials. In particular, an unknown group trial does not prevent a final IIS
certificate if all surviving individual candidates can subsequently be resolved.
Limits include both group and individual calls and preserve the last proven
infeasible subsystem when interrupted.

Batching helps when many assumptions are irrelevant. It can require additional
calls when most assumptions are necessary, and can select a different IIS if
several exist. It uses one extra O(N) scratch vector only when enabled; the
same feasibility checker is used. For example:

```cpp
auto answer = mippp::iis::compute_linear_iis(
    system, [] { return mippp::cbc_milp{}; }, {.limits = {.initial_batch_size = 64}});
```

## Candidate ordering

Choose which candidates to try removing first with `linear_options::order`.
The comparator receives `member` values. MIP++ supplies `rows_first_order{}` and `bounds_first_order{}` convenience policies:

```cpp
constexpr mippp::iis::linear_policy elastic{
    .elasticity = mippp::iis::elasticity_strategy::reuse};
auto answer = mippp::iis::compute_linear_iis<elastic>(system, factory,
    mippp::iis::linear_options{
        .limits = {.initial_batch_size = 64},
        .order = mippp::iis::rows_first_order{}
    });
```

Custom comparators can use application priorities or precomputed scores such as
estimated redundancy or violation magnitude. Keep the comparator stable during
the call and use finite scores (NaNs do not define a strict weak order). Lower
scores should correspond to candidates you want to attempt removing first.
No additional solver calls are made to compute scores automatically.

Ordering is applied before grouping and preserved through successful deletions.
Elastic seeds translate their local IDs back to original candidate identities
before comparing; full-model fallback uses the same comparator. Individual
necessity tests still establish irreducibility, regardless of the heuristic.
Different orderings can select different valid IISs, and neither ordering policy
promises a smaller IIS or faster solve. Their names describe removal priority,
not a preference for which members should remain in the result.

Without a comparator, traversal requires no sorting. Explicit ordering costs O(N log N) comparisons and retains O(1)
single-candidate swap/pop operations; it does not shift the vector on each trial.

## Model reuse for deletion

Set `linear_policy::deletion = deletion_strategy::reuse` to opt into a retained
feasibility model (the default remains rebuilding):

```cpp
constexpr mippp::iis::linear_policy retained{
    .deletion = mippp::iis::deletion_strategy::reuse};
auto answer = mippp::iis::compute_linear_iis<retained>(system, factory);
```

The template workspace needs readable and modifiable variable bounds; otherwise
the adapter uses the rebuild path. Each original candidate gets one continuous,
zero-cost slack: `Ax + s >= lower` or `Ax - s <= upper`. Fixing `s = 0` enforces
the side; releasing its upper bound removes it, without a finite big-M. Original
variables are free, with their finite bounds represented by candidate rows.
Original-domain integrality is retained; LP-relaxation mode still relaxes it.
Rows, columns and candidate IDs remain stable. Only changed slack bounds are
updated, including restoring candidates after feasible or unknown trials and
switching from a rejected seed back to full-set deletion.

Construction is lazy, inside the first budget-approved oracle call. Cancellation
and deadlines are checked again after construction/updates and before every
native solve. Native certificate extraction still uses a separate original-form
model initially, so its ray/bound mappings do not include auxiliary slacks.
Seed verification is never skipped. Elasticity uses a separate positive-cost
model; its objective and state cannot leak into deletion.

`deletion_model_reused` reports whether the retained workspace was constructed;
`deletion_reoptimizations` counts native solves after its first solve. Neither
promises actual basis reuse, fewer simplex iterations, or faster MIP search.
Solver/API exceptions propagate rather than silently trusting stale state.
The extra candidate rows and slack columns cost memory and can hit size-limited
licenses sooner. Reuse is therefore opt-in, and the rebuild path remains useful.
Direct row-bound updates and explicit basis transfer are not implemented here.

## LP elasticity prefilter

For a continuous problem with many irrelevant constraints, elasticity can find a
smaller conflict before deletion. It temporarily permits constraint violations,
then uses the violated sides to propose a seed. Enable it with:

```cpp
constexpr mippp::iis::linear_policy elastic{
    .elasticity = mippp::iis::elasticity_strategy::rebuild};
auto answer = mippp::iis::compute_linear_iis<elastic>(system, factory, {
    .limits = {.initial_batch_size = 64},
    .elastic = {.max_solves = 16, .violation_tolerance = 1e-7}
});
```

The policy's `elasticity` strategy enables an LP elasticity prefilter. After checking
the original model is infeasible, it softens every finite bound and row side with
a nonnegative slack of unit objective cost. A lower side uses `Ax + slack >= L`;
an upper side uses `Ax - slack <= U`. Variable bounds are also represented as
elastic rows, leaving the original variables free. This keeps bounds eligible
for extraction rather than hiding them in the background of the seed.

Each optimal elastic solution identifies positive slacks. Those candidates are
made hard in subsequent elastic solves. The filter accumulates candidates until
the resulting elastic LP is infeasible. The first violated set alone is not a
valid seed: satisfying it might merely move the violation to another row.
Deletion then rechecks the proposed seed in the original formulation and reduces
it to an IIS. Inconclusive elastic solves, no progress, or failed revalidation
fall back to deletion of the full set. Solver/API exceptions still propagate.
Fallback reuses the initial full-set infeasibility proof within this extraction:
the candidates and fixed background have not changed, so it starts directly
with deletion trials rather than spending another call on the same full set.
This internal continuation preserves cumulative solve/time/cancellation limits.
Public `deletion_filter` calls and proposed native or elastic seeds still always
require their own initial verification; there is no public assume-infeasible flag.

`reduction.solve_count` includes initial verification, elastic oracle calls,
seed revalidation and deletion calls. `elasticity_calls` reports the prefilter
portion; `elasticity_seed_used` means the original formulation confirmed the
seed. All calls share `options.max_solves`, deadline and stop token. The elastic
local limit (default 16) bounds its own work even when the global budget is
unlimited. A stopped pipeline retains the last proven infeasible subsystem.
Original-domain integer systems bypass the LP prefilter. An explicitly selected
LP relaxation can use it, but never explains integer-only infeasibility. Slacks
below the configured tolerance are ignored as a heuristic; lack of progress
triggers fallback. Unit penalties depend on row scaling, and large or mostly
irreducible systems can be slower with elasticity. It remains opt-in.

### Reusing the elastic model

```cpp
constexpr mippp::iis::linear_policy warm{
    .elasticity = mippp::iis::elasticity_strategy::reuse};
auto answer = mippp::iis::compute_linear_iis<warm>(system, factory);
```

Warm starts retain a single elastic LP when the model satisfies
`has_modifiable_variable_bounds`. Every candidate initially has a slack column;
hardening a candidate fixes its slack's upper bound to zero. The rows, columns,
coefficients and objective remain unchanged, allowing the solver to reuse its
basis or other optimization state. Fixed slacks are excluded from subsequent
violation reports.

This uses native model retention, not explicit basis export/import: the current
backends do not implement MIP++'s `has_lp_basis_warm_start` concept. Reusing state
is ultimately the solver's choice; an interior-point algorithm or a MIP wrapper
may restart internally. The improvement can include both less model construction
and less solver work. Original-domain MIPs still bypass LP elasticity entirely.

`elasticity_reoptimizations` counts second and subsequent solves on the retained
model, not successful basis imports or saved iterations. Without bound-update
support, or with `elasticity_strategy::rebuild`, elastic models are rebuilt
and this counter stays zero. Elasticity is disabled by default (`off`).
Construction is lazy: a budget or cancellation
that prevents the first elastic call does not invoke its model factory.

The workspace is local to one prefilter invocation and destroyed before seed
revalidation/deletion. No native handles or cached basis are shared between
independent IIS computations. All solve limits and conservative status
handling apply. Deletion rebuilds by default; the separate opt-in deletion
workspace above uses a fixed-size feasibility formulation instead.

## Optional native Farkas seeds (Clp, SoPlex, and MOSEK)

A solver's infeasibility certificate can suggest a smaller set of conflicting
bounds and rows before deletion starts. This proposed set is called a *seed*;
it is always verified with another feasibility check before use. Enable it for
Clp, SoPlex, or MOSEK with:

```cpp
constexpr mippp::iis::linear_policy native{.native_seed = true};
auto answer = mippp::iis::compute_linear_iis<native>(
    system, [] { return mippp::clp_lp{}; }, {.limits = {.max_solves = 100}});
```

Native seeding is selected by `linear_policy::native_seed`. Its runtime
`linear_options::native.relative_tolerance` defaults to `1e-9` (finite and
nonnegative). Native extraction is tried before optional elasticity.
Unsupported models continue with elasticity or ordinary deletion. The compile-time `has_infeasibility_ray` capability requires
`get_infeasibility_ray()` returning an owning optional vector of row multipliers
in original insertion order. It must not perform additional hidden solves.

Clp uses `Clp_infeasibilityRay` and releases its allocation with `Clp_freeRay`,
including if copying throws. It returns no ray unless native status is proven
primal infeasible. The adapter copies the ray before destroying the model.
Clp's primal solve operates directly on the original model; this path
does not introduce a presolve retry or depend on presolved row numbering.

`algorithm/ray_support.hpp` is standard-library-only. It rejects nonfinite and
all-zero rays, divides magnitudes by the maximum magnitude, and selects values
strictly greater than the tolerance. The adapter rejects mismatched dimensions
and maps row sides to their original candidate IDs. By default it conservatively retains
**all finite variable bounds** for row-ray backends, since a row ray alone need not describe bound
contributions. This may give a larger seed than full multiplier reconstruction.

Two further opt-in flags use more of the initial certificate:

```cpp
constexpr mippp::iis::linear_policy policy{
    .native_seed = true, .prune_bounds = true, .order_by_weight = true};
```

`prune_bounds` screens row-ray bound candidates using magnitudes of `A^T y`,
computed from the **full signed ray**, including multipliers below the row-support
threshold and duplicate sparse terms. It normalizes the ray before multiplication
and accumulates in `long double`; nonfinite/malformed projections retain all
bounds. A valid all-zero projection selects no bounds. Selected columns keep
both finite bound sides, avoiding backend-specific ray-sign conventions.
Column support uses the same relative tolerance, relative to the largest column
contribution. MOSEK's full certificate already supplies bound-side multipliers,
so this flag does not further reconstruct them. Every smaller proposal still
requires its own feasibility solve; threshold/cancellation errors cannot authorize
deletion merely from these magnitudes.

`order_by_weight` tries smaller absolute multipliers/contributions first. Explicit
caller priorities remain primary; weights break their equivalences, followed by
original candidate IDs. Scores are captured once and remain fixed during sorting
and reduction. Missing bound contributions sort last. For row rays, row and
column scores use the same ray normalization. Scores are not row-scale invariant
and are not necessity proofs: ordering can help or hurt, and remains off by default.
These options add no solver calls for certificate extraction itself. They use
the initial certificate only, not refreshed rays after subsequent deletion solves.

A smaller seed is always re-solved in the original formulation before deletion.
It is only a hint until verified: thresholding may remove a necessary row.
Feasible/unknown verification results fall back to elasticity (if enabled) or
full deletion. Missing rays and non-smaller support also fall back. Every
verification and reduction call consumes the shared budget; cancellation or
budget exhaustion preserves the last proven infeasible subsystem.

`native_seed_used` means a smaller native proposal was verified and used for
reduction, not necessarily that reduction finished. `native_seed_size` reports
the proposed support size including selected/retained bounds (zero also covers no
proposal). Consult `reduction.irreducible` as usual. Original-domain MIPs skip
this LP-only feature; callers may explicitly analyze `domain::lp_relaxation`,
but it cannot explain integer-only infeasibility.

Clp-specific integration coverage is selected with `--gtest_filter=ClpRay.*`.
Tests exercise owned ray lifetime, bound-dependent/ranged-row conflicts,
redundant rows, deliberately invalid thresholded seeds, missing rays, shared
budgets, and cancellation during seed verification.

### SoPlex

SoPlex uses the same seed interface and verification pipeline. Stock SoPlex
8.1.0 does not expose the C++ certificate methods through its C interface.
The optional C-interface extension in
[`patches/soplex-v8.1.0-farkas.patch`](../patches/soplex-v8.1.0-farkas.patch)
adds `SoPlex_hasDualFarkas` and `SoPlex_getDualFarkasReal` plus native C tests.
See [build instructions](../patches/README.md). This patch is local, not an
upstream release feature.

MIP++ resolves these symbols optionally, including when compiled against stock
SoPlex headers. An unmodified shared library continues to load and solve;
`get_infeasibility_ray()` returns `nullopt` and native seeding falls back.
The getter copies original-row multipliers, performs no optimization, and
returns no certificate for non-infeasible status or failed extraction.

SoPlex simplification may prove infeasibility without producing a Farkas ray.
This is a normal fallback, not an error. To explicitly favor simplex certificate
generation, configure the factory (the adapter never changes this silently):

```cpp
auto factory = [] {
    mippp::soplex_lp model;
    model.native_api().setIntParam(model.native_model(), 10, 0); // SIMPLIFIER_OFF
    return model;
};
constexpr mippp::iis::linear_policy native{.native_seed = true};
auto answer = mippp::iis::compute_linear_iis<native>(system, factory);
```

Disabling simplification may cost performance and does not guarantee a ray on
every numerically difficult LP. All finite variable bounds are retained just
as with Clp. No C++ ABI symbols or external bridge library are loaded.

`SoPlexRay.*` requires the patched library; `SoPlexStock.*` requires an additional
unmodified library at `MIPPP_SOPLEX_STOCK_LIBRARY`. `LinearIis/10.*` exercises the
ordinary SoPlex pipeline and works with either library. Tests cover certificate
ownership, moving a solved model, invalid output buffers, support verification,
threshold fallback, budgets, and default-simplifier behavior. WSL/Linux was
tested; Windows runtime validation remains outstanding.

### MOSEK

`mippp::mosek_lp` exposes an owning `get_infeasibility_certificate()` with four
arrays: `row_lower`, `row_upper`, `variable_lower`, and `variable_upper`.
Their common representation is `mippp::linear_infeasibility_certificate<Scalar>`
in `infeasibility_certificate.hpp`; no MOSEK types leak into the generic adapter.
The `has_infeasibility_certificate` concept selects this richer path at compile
time. Unlike row-ray backends, it can exclude irrelevant variable bounds too.

The getter examines basic and interior-point solution slots, requiring both
primal-infeasible problem status and `MSK_SOL_STA_PRIM_INFEAS_CER`. It copies
the four arrays through `MSK_getsolution` without optimizing. Missing certificate
status returns `nullopt`; solver/API errors still propagate. The adapter checks
array dimensions and finite values, maps each side back to original candidate
IDs, and applies the same support threshold and mandatory revalidation as Clp.
Native seeding remains LP-only; use `mosek_lp` and explicit `lp_relaxation` for
integer input. `mosek_milp` continues to use deletion for original-domain MIPs.

```cpp
constexpr mippp::iis::linear_policy native{.native_seed = true};
auto answer = mippp::iis::compute_linear_iis<native>(
    system, [] { return mippp::mosek_lp{}; },
    {.limits = {.max_solves = 100, .time_limit = std::chrono::seconds(5)}});
```

Interior-point certificates can be dense even for a small underlying conflict.
In that case default support selection may retain the full candidate list and
fall back normally. A larger `relative_tolerance` may produce a smaller proposal
but is a heuristic, not a stronger certificate. No optimizer or presolve setting
is silently changed. Tests explicitly exercise primal/dual simplex and interior
point with basis identification disabled.

MOSEK LP/MIP wrappers call `MSK_optimizetrm` exactly once per `solve()`. Status
inspection and certificate/solution reads do not optimize. LP getters use the
available continuous solution rather than assuming a basic solution exists;
MIP getters also support continuous models. Integer optimality is recognized
with primal-feasible problem status, since MIPs need not have dual solutions.
Native time caps are forwarded, with MOSEK's negative unlimited sentinel mapped
to infinity for generic duration comparisons.

Validation uses MOSEK 11.2.4. Point `MIPPP_MOSEK_LIBRARY` at the shared library,
for example `/opt/mosek/11.2/tools/platform/linux64x86/bin/libmosek64.so.11.2`
or `C:\Program Files\mosek\11.2\tools\platform\win64x86\bin\mosek64_11_2.dll`.
License discovery is left to MOSEK; license files are not parsed, copied, or
committed. WSL tests exercise `MosekCertificate.*`, `LinearIis/8.*` (MIP), and
`LinearIis/12.*` (LP), plus the existing MOSEK model suites. A native Windows
smoke test covers simplex seeding and interior-point fallback; it is not the
full Windows regression suite.

References: [MOSEK certificate extraction](https://docs.mosek.com/latest/capi/tutorial-pinfeas-shared.html)
and [solution/termination status](https://docs.mosek.com/latest/capi/accessing-solution.html).

## Results, diagnostics, and statistics

Results remain structured data. Call `describe_result(answer)` (also available
from `utility/iis_report.hpp`) when you want a plain-language explanation:

```cpp
constexpr mippp::iis::linear_policy policy{
    .native_seed = true,
    .measure_time = true // optional: two additional clock reads per extraction
};
auto answer = mippp::iis::compute_linear_iis<policy>(system, factory,
    {.limits = {.max_solves = 100}});
std::cout << mippp::iis::describe_result(answer) << '\n';
```

Pass `{.include_members = true}` as the formatter's second argument to list the
conflicting sides using original zero-based row/variable indices and lower/upper
bound labels. Listing is opt-in because a large conflict can contain thousands
of sides. Unchecked candidates are never presented as a verified conflict.

The report distinguishes a satisfiable problem, a verified conflict, a conflict
whose every remaining side is necessary, and an inconclusive search. A minimal
conflict is not necessarily the smallest possible one. Fractional-value analysis
always carries the warning that it cannot establish feasibility of the original
integer problem. An empty member list is never used alone to decide the outcome.

`diagnostics` stores value snapshots, including requested policies, limits,
tolerances, supported capabilities, native/elastic seed outcomes, and whether
the pipeline returned to its full-set proof. Seed outcomes distinguish disabled,
unsupported, skipped integer systems, missing or unusable hints, non-smaller
hints, no progress, limits, rejected verification, and verified use. This avoids
guessing the reason for a fallback from a zero count or a false boolean.
Snapshots do not retain the caller's comparator, factory, or cancellation source.

Limits and invalid-input messages identify relevant option names, their values
in this invocation, and allowed ranges/choices. Input errors identify original
zero-based row/variable indices. A stopped report suggests changing the applicable
limit, not weakening the standard of proof. Observed solver time caps are labeled
as the last value read **before** forwarding the remaining global budget. An
unobserved cap is not described as unlimited. No extra native queries are made
just to populate that field, and unsupported caps remain explicitly unsupported.

The counters are deliberately split by scope:

| Data | Meaning |
| --- | --- |
| `reduction.solve_count` | All checks in the extraction, including unsuccessful proposals and elastic passes. |
| `statistics.feasibility_checks`, `elastic_checks` | Satisfiable/conflicting/inconclusive outcomes across the entire extraction; their totals sum to `solve_count`. |
| `reduction.statistics` | Checks and removals in the **final deletion pass**, including its own initial verification or continued proof. It excludes abandoned proposals and elastic passes. |
| `necessary_members`, `unresolved_members` in that pass | Remaining sides individually proved necessary or tested inconclusively. Other remaining sides have not yet been individually tested. Group tests do not establish individual necessity. |
| `statistics.rebuild`, `deletion`, `elastic` | Models built, actual solver runs, changed bounds, bound-only proofs, skipped runs after stopping, largest constructed dimensions, and last observed solver issue for each kind of workspace. |
| `native_verification_calls`, `elastic_verification_calls` | Separate initial checks of proposed conflicts; zero when the remaining budget prevents verification. |
| `certificate_queries`, seed sizes | Requests for solver hints and proposed/accumulated sizes, not extra optimization calls. |
| `statistics.elapsed` | Optional overall duration, including solver construction and cleanup, not pure optimization time. Absent unless `measure_time` is enabled. |

`statistics.models_built()` and `solver_runs()` sum the three workspaces. A model
run counts a call to `Model::solve()`, not any hidden internal optimizer passes.
These counters cannot inspect private work inside an arbitrary user-supplied model.
A model
can be created without running the solver: contradictory bounds can prove a
conflict directly, or cancellation can stop a constructed model. Peak dimensions
include extra slack variables, split row sides, and any dummy variable. Existing
elastic/deletion reoptimization counters retain their meaning; they do not count
saved iterations or guarantee that a solver reused its internal state.

Counters use scalar updates and dimensions already known while building models.
There are no added model scans, solver calls, environment reads, strings, or
per-check clock reads in the successful extraction path. Detailed timing,
memory/allocation tracing, solver-specific iteration/node counts, and per-trial
history are not collected. The current common model API does not expose those
iteration/node counts uniformly; missing statistics are not reported as zero.

### Loading and license errors

Library-loading errors show the explicit path, `MIPPP_<SOLVER>_LIBRARY`, and the
platform's relevant search variables (`PATH`, `LD_LIBRARY_PATH`, or the macOS
`DYLD_*` paths), with current values and accepted formats. Explicit paths override
environment selection, which overrides name search. A failed explicit selection
does not silently load a different file. These messages report the environment
of the running process: changing a desktop setting does not update an already
running application. Version warnings explain that **any set value**, including
`0`, suppresses `MIPPP_NO_VERSION_WARNING`; hiding a warning does not fix a mismatch.

Recognized Gurobi, CPLEX, and MOSEK license failures also list the corresponding
license settings. Ordinary path values are quoted; subscription keys, endpoint
values that might include credentials, and MOSEK's potentially embedded license
text are shown only as set/empty/unset. No license file is read or parsed.
Sources: [Gurobi license path](https://support.gurobi.com/hc/en-us/articles/15064259352209-How-do-I-resolve-an-Error-10009-Unable-to-open-Gurobi-license-file),
[IBM subscription settings](https://www.ibm.com/support/pages/node/297247),
[MOSEK license locations](https://docs.mosek.com/latest/licensing/licensefaq.html),
and [MOSEK inline license support](https://docs.mosek.com/latest/faq/faq.html).

The linear adapter enriches `solver_error` and `license_error` with input size
and policy settings, retains their catchable base types, and nests the original
exception. Caller exceptions and other exception types propagate unchanged.
Solver-provided detail remains after the plain-language explanation; review it
before sharing logs, since third-party messages can contain their own private
data. Switching to rebuilding with elasticity off can avoid extra model expansion
under size-limited licenses, but cannot guarantee fitting a limit: two-sided
rows are still split. Changing environment variables does not grant a license.

## Using your own feasibility checker

`mippp/algorithm/deletion_filter.hpp` is a standalone utility of the MIP++
library. Use it with your own feasibility checker when the linear adapter does
not fit your application. It needs only C++20 and the standard library; using
MIP++ models and solver wrappers requires the library's usual C++23 setup.

Its template takes a candidate count and
a callable accepting `std::span<const std::size_t>` and returning
`mippp::iis::feasibility` (`feasible`, `infeasible`, or `unknown`). Candidate
IDs identify assumptions; the oracle checks their conjunction with any fixed
background assumptions. Feasibility must be monotone under removing candidates.
The span is valid only for the duration of the callback.

The deletion filter checks the complete system first, then tries removing each
candidate. It permanently removes a candidate only on proven infeasibility.
With the default single-candidate strategy, it makes at most N+1 oracle calls,
and returns original candidate IDs. The
deterministic swap/pop traversal need not preserve input order. A feasible
witness obtained while testing a retained candidate remains feasible when other
candidates are subsequently removed, so one pass suffices.

`deletion_filter` accepts an optional fourth argument: a strict-weak-order
comparator over original candidate IDs. Candidates for which the comparator
returns true are tried for removal first. Move-only comparators are supported.
Equivalent priorities are resolved by original ID for deterministic traversal.

```cpp
auto answer = mippp::iis::deletion_filter(count, oracle, options,
    [&](std::size_t a, std::size_t b) { return scores[a] < scores[b]; });
```

Check `proven_infeasible()` and `irreducible` on the returned `result` before
using its candidate IDs as an IIS. Until the initial check proves infeasibility,
`result.members` is only a list of unchecked candidates. Exceptions from your
checker propagate unchanged.

`algorithm/elasticity_filter.hpp` is another standalone C++20 utility. It
coordinates an elastic checker independently of model construction; the MIP++
linear implementation is in `utility/elastic_lp.hpp`.

## Local timing benchmark

```bash
cmake -S . -B build-iis-bench -DCMAKE_BUILD_TYPE=Release -DENABLE_IIS_BENCHMARK=ON
cmake --build build-iis-bench --target mippp_iis_benchmark
# Locate HiGHS through its normal search path or MIPPP_HIGHS_LIBRARY.
./build-iis-bench/mippp_iis_benchmark 128 3 iis-benchmark.local.csv
# Optional open-source certificate benchmark (Clp normal path or MIPPP_CLP_LIBRARY):
./build-iis-bench/mippp_iis_benchmark 128 5 iis-clp.local.csv clp
```

This target needs no GoogleTest or MELON dependencies and loads only the chosen
open-source backend: HiGHS by default, or Clp via the last argument.
It compares single deletion, batching, cold elasticity, cold elasticity plus
batching, warm elasticity, warm elasticity plus batching, batched rows-first,
batched bounds-first, retained deletion, retained batched deletion, and combined
warm elasticity/retained batched deletion
on six deterministic families: redundant rows, feasible systems, scaled rows,
disjoint conflicts, redundant variable bounds, and a chain with every row needed.
The size argument controls each family, not an identical model dimension.
Clp adds five native-certificate variants: ordinary row support, bound screening,
weight ordering, both, and both with deletion reuse. Compare these within Clp,
not against the HiGHS run as a solver ranking. The benchmark resolves Clp's
logging/version functions from the same library outside timing; it adds no
required symbols to the production Clp wrapper.

Each strategy gets an untimed warm-up and measured execution order rotates
across repetitions. The solver uses one thread with logging disabled. CSV rows
record elapsed time including model construction, total calls, elastic calls,
elastic/deletion reoptimizations, deletion reuse, IIS size, verification status,
native-seed size/use, solver version, and work counters for model construction,
solver runs, changed bounds, proof checks, and deletion progress.
Outside timing, fresh original
models verify the returned subsystem is infeasible and every single-member
deletion is feasible. Any failed or inconclusive verification fails the run.
For feasible inputs, the expected feasible status is checked instead.

Output is local, existing files are not overwritten, and `*.local.csv` is ignored
by Git. Nothing is uploaded or automatically included in a PR. The driver does
not run commercial-solver benchmarks. It is a fallback-strategy comparison,
not a native-IIS comparison, a solver ranking or a representative industrial
corpus. Record compiler, build mode, machine and library path alongside private
results when making performance decisions.

## Headers and ownership

The public entry point is `compute_linear_iis<Policy>(system, factory, options)`
in `mippp/utility/linear_iis.hpp`. The generic utilities can also be used on their
own. Templates and concepts allow direct calls to your checker and selected
model type, while compile-time policies omit disabled workspaces and branches.

The linear adapter uses MIP++ models and solver capabilities. Clp and SoPlex
wrappers expose row certificates; MOSEK exposes bound-side certificates as well
as solution/status handling used by extraction. Shared loading and license
messages also provide diagnostics outside IIS extraction. These integrations
are part of MIP++, not dependencies of the standalone algorithms.

For custom integrations, the supporting headers are:

- `linear_iis_types.hpp`: public input, options, results and concepts.
- `prepared_linear_system.hpp`: input validation, stable candidate IDs and the
  shared one-sided inequality representation. It borrows the input only during
  the synchronous extraction; callers must not modify it during that call.
- `linear_iis_model.hpp`: common inequality data and conservative status mapping,
  independent of the elastic or deletion workspace implementation.
- `cold_model_workspace.hpp`: rebuilding feasibility models and reusable scratch;
  a templated observer reads an infeasible model before it is destroyed.
- `native_seed.hpp`: owning certificate-to-seed conversion, dimension validation
  and fixed ordering scores. It neither solves models nor certifies a proposal.
- `deletion_workspace.hpp` and `elastic_lp.hpp`: separate retained feasibility
  and elasticity state, respectively. Neither owns the pipeline's proof state.
- `phase_budget` in `algorithm/iis_limits.hpp`: shared phase accounting and one
  normalized deadline; impossible solve counts throw instead of underflowing.

The coordinator keeps proof state and shared budgets together: a proposed seed
cannot replace a known conflict before verification. Separate feasibility and
elasticity workspaces prevent an elastic objective from affecting deletion.
Scratch capacity for parameters, handles, row terms, and candidate translation
is reused within an extraction to reduce allocations. Fresh-model trials reset
bounds and rebuild handles; no borrowed expression or solver state is shared
between independent extractions.

MOSEK binds a scoped cleanup guard before its first native allocation.
Partial constructor failures release acquired handles; moving a model transfers
ownership without duplicating cleanup. Destruction attempts task cleanup before
environment cleanup and does not throw on native cleanup error codes. Cleanup
failures cannot mask an original exception; this does not claim a native resource
was successfully freed if MOSEK itself reports a failure. Solver-free tests
inject partial allocation and failed cleanup, malformed certificate dimensions,
nonfinite arrays, and exhausted/overreported phase budgets.

## Running validation

Build `mippp_iis_test`. Run
`--gtest_filter=DeletionFilter.*:ElasticityFilter.*:IisVectors.*` without solvers.
The typed integration tests compile for all eleven backend families. Suite
indices are 0: HiGHS LP, 1: HiGHS MIP, 2: Clp, 3: CBC, 4: Gurobi, 5: CPLEX,
6: COPT, 7: GLPK, 8: MOSEK MIP, 9: SCIP, 10: SoPlex, 11: Xpress,
12: MOSEK LP.
Select their numbered `LinearIis/N.*` suites when only some are installed.
These tests fail on unavailable libraries or license errors rather than silently
skipping validation.

CTest runs the solver-free core tests by default. On a fresh configure,
`MIPPP_REQUIRED_SOLVERS` adds the matching HiGHS, Clp, CBC, and GLPK integration
suites, including the published-vector tests where available. Set the CMake cache value
`MIPPP_IIS_TEST_FILTER` to include installed backends, for example
`DeletionFilter.*:ElasticityFilter.*:LinearIis/2.*:LinearIis/3.*` for Clp and CBC. The executable
also accepts the usual GoogleTest `--gtest_filter` option directly.

The core tests exhaust every family of conflicts over four candidates and check
both returned infeasibility and feasibility after removing each reported member.
Integration tests include bounds, ranged rows, redundant constraints and
integer-only infeasibility with a feasible LP relaxation.

[Published HiGHS vectors](https://github.com/ERGO-Code/HiGHS/blob/755a8e027a99a8d4ecf153a8dde4b2a767cdf384/check/TestIis.cpp)
also exercise competing conflicts, coupled rows and both signs of empty rows.
`PublishedIis/0.*` through `PublishedIis/3.*` select HiGHS LP, HiGHS MIP, Clp,
and CBC respectively. An independent Fourier-Motzkin checker verifies each
returned conflict and every single-member removal. See `test/data/iis/README.md`
for provenance, transformations, licensing and the checker's limited scope.

## Further reading

J. W. Chinneck, *Feasibility and Infeasibility in Optimization* (2008),
[DOI: 10.1007/978-0-387-74932-7](https://doi.org/10.1007/978-0-387-74932-7).
