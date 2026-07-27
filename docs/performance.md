# Performance

The cost of an algebraic modeling layer is invisible for a one-shot solve where solver time dominates — a two-second build in front of a two-hour solve is noise, whatever layer produced it. It becomes real in two regimes: very large models, where construction is a nontrivial fraction of wall-clock time, and iterative methods — column generation, row generation, repeated re-solves in a benchmark loop — that touch the model thousands of times. The numbers below quantify what MIP++ buys you in those regimes, and only those; this page collects what is actually measured, on which the "no modeling tax" claim rests.

All numbers come from [**mippp_nqueens**](https://github.com/fhamonic/mippp_nqueens), which times the *filling* of an N-Queens MILP — `N²` binary variables, `6N−6` constraints, ≈ 4·10⁶ nonzeros at N = 1000 — through eight modeling libraries spanning C++, Julia and Python, on up to eight solver backends.

!!! note "What is timed"
    **Only model construction, never the resolution.** The timer starts once the solver API object exists and stops once the model holds all `N²` variables and all `6N−6` constraints, after forcing any pending update to be flushed. No presolve, no optimization, no solution retrieval.

!!! note "What the N-Queens shape does and does not stress"
    N-Queens is variable-heavy and constraint-light: a million variables against ~6000 rows, each row a plain sum of existing variables. The benchmark therefore chiefly measures **variable creation and short-row streaming** — a shape that favours a zero-copy expression system. It says nothing about models dominated by dense, structured constraints, nor about solve time, memory footprint or model quality. Read the multiples as evidence for the build-bound regimes described above, not as a universal speedup.

Every model in the comparison is written in its best-case form. The natural way to express a diagonal constraint — filtering the whole board once per diagonal — makes filling a Θ(N²) model cost Θ(N³), which at N = 1000 costs the PuLP model 115 s where indexing each diagonal directly builds the same model in 6.9 s. None of that factor happens inside PuLP. Every model here touches only its own nonzeros, addressed by index, and where an interface offers several ways to hand over a row, the fastest one is used — for both OR-Tools APIs that is writing coefficients straight into an opened row rather than going through an expression object ([see below](#or-tools-is-shown-in-its-fastest-row-filling-form)). The tables therefore report a *floor*: the cost of the interface layer itself.

## MIP++ vs the Gurobi C API

The most direct measure of MIP++'s overhead. The pure Gurobi C API in absolute milliseconds, MIP++ as a *percentage* of it, and the other Gurobi-capable interfaces as multiples:

| N | Gurobi C API<br>per constr. | Gurobi C API<br>bulk | MIP++ | gurobipy | JuMP<br>warm | JuMP<br>cold | Python-MIP<br>CPython | Python-MIP<br>PyPy |
|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 3.2 ms | 99.1 % | 102.3 % | 6.5 × | 3.5 × | 69.4 × | 19.4 × | 36.5 × |
| 200 | 9.0 ms | 98.1 % | 102.5 % | 8.2 × | 5.0 × | 36.2 × | 18.3 × | 16.4 × |
| 300 | 17.5 ms | 103.9 % | 104.3 % | 9.4 × | 6.4 × | 23.9 × | 18.9 × | 11.2 × |
| 400 | 29.8 ms | 101.1 % | 105.4 % | 9.7 × | 8.3 × | 18.0 × | 18.8 × | 9.2 × |
| 500 | 48.2 ms | 95.6 % | 103.5 % | 9.6 × | 7.5 × | 13.2 × | 17.8 × | 7.5 × |
| 600 | 68.1 ms | 100.6 % | 103.3 % | 10.4 × | 7.5 × | 12.8 × | 18.2 × | 7.1 × |
| 700 | 93.1 ms | 99.5 % | 106.8 % | 11.3 × | 7.4 × | 11.3 × | 18.1 × | 6.9 × |
| 800 | 122.2 ms | 100.3 % | 107.0 % | 11.7 × | 7.5 × | 10.0 × | 18.1 × | 6.9 × |
| 900 | 154.1 ms | 101.3 % | 108.4 % | 12.0 × | 7.5 × | 9.2 × | 18.4 × | 6.7 × |
| 1000 | 190.0 ms | 101.7 % | 107.3 % | 12.3 × | 7.4 × | 9.3 × | 18.5 × | 6.9 × |

MIP++ stays within 2–8 % of the raw C API across the whole sweep (102–108 %): **the modeling layer is thin.** Handing Gurobi the whole matrix in a single `GRBaddconstrs` call instead of one `GRBaddconstr` per constraint is worth nothing (96–104 %), so matching the per-constraint path is the meaningful comparison rather than a handicap.

The JuMP columns are `direct_model`; the `cold` one includes Julia's JIT compilation, paid once per process, which is why it improves with N — as does the Python-MIP PyPy column, for the same reason.

## MIP++ vs the other C++ and Julia interfaces

Time in milliseconds for MIP++, and how much longer each other interface takes **on the same backend**. HiGHS is the one backend all of them share: MathOpt has no Cbc, and JuMP's direct mode needs an incrementally modifiable backend, which the Cbc wrapper is not.

| N | MIP++ | OR-tools<br>MPSolver | OR-tools<br>MathOpt | JuMP<br>cached | JuMP<br>direct |
|:---:|---:|---:|---:|---:|---:|
| 100 | 1.6 ms | 1.3 × | 2.6 × | 3.7 × | 12.0 × |
| 200 | 5.8 ms | 1.3 × | 2.8 × | 4.6 × | 13.2 × |
| 300 | 14.4 ms | 1.3 × | 2.6 × | 4.4 × | 12.8 × |
| 400 | 23.7 ms | 1.3 × | 3.1 × | 7.3 × | 15.0 × |
| 500 | 37.4 ms | 1.3 × | 3.7 × | 6.1 × | 16.1 × |
| 600 | 59.1 ms | 1.2 × | 3.1 × | 5.5 × | 16.3 × |
| 700 | 75.0 ms | 1.3 × | 4.7 × | 5.8 × | 17.2 × |
| 800 | 104.1 ms | 1.2 × | 4.2 × | 5.3 × | 16.6 × |
| 900 | 127.0 ms | 1.3 × | 4.0 × | 5.4 × | 17.8 × |
| 1000 | 151.5 ms | 1.3 × | 5.6 × | 5.7 × | 18.3 × |

Reading it: a 1000-Queens model — one million binary variables and ~6000 constraints — is filled into HiGHS in **152 ms**, and into Cbc in **67 ms**. OR-Tools' `MPSolver` takes 1.2–1.3× longer, MathOpt 2.6–5.6×, JuMP 3.7–7.3× in cached mode and 12.0–18.3× in direct mode.

### Deferred vs. direct model construction

One structural difference explains most of the spread between interfaces, and it is not the language.

!!! important "The comparison is not work-for-work, and the bias is against MIP++"
    Both OR-Tools APIs and JuMP's default `Model` accumulate the model in their own data structures and translate it for the solver later, whereas MIP++, the Gurobi C API and JuMP's `direct_model` write into the solver's own model as each constraint is added.

    Two independent tells confirm the deferral: JuMP's cached mode fills at the same speed whichever backend is named (867 ms for HiGHS, 873 ms for Gurobi at N = 1000), and `MPSolver` needs ~200–220 ms at N = 1000 for Cbc, HiGHS and SCIP alike. Wherever a deferred interface looks cheap, it has simply not handed the solver anything yet.

The practical consequence for a research code is the one the ratios understate: with MIP++ the model *is* in the solver when construction returns, so re-solves, in-place [model updates](solving/updates.md) and [column generation](algorithms/column-generation.md) rounds do not re-extract anything.

## MIP++ vs OR-Tools, and where MIP++ *loses*

OR-Tools ships two C++ modeling APIs and both are benchmarked: `MPSolver` and the newer MathOpt. MIP++ in absolute milliseconds, each OR-Tools API as a percentage of it:

| N | MIP++<br>Cbc | MIP++<br>HiGHS | MIP++<br>SCIP | MPSolver<br>Cbc | MPSolver<br>HiGHS | MPSolver<br>SCIP | MathOpt<br>HiGHS | MathOpt<br>SCIP |
|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.8 ms | 1.6 ms | 7.5 ms | 244 % | 127 % | 28 % | 260 % | 56 % |
| 200 | 2.9 ms | 5.8 ms | 24.1 ms | 269 % | 134 % | 34 % | 281 % | 69 % |
| 300 | 6.3 ms | 14.4 ms | 51.7 ms | 294 % | 127 % | 36 % | 262 % | 71 % |
| 400 | 11.0 ms | 23.7 ms | 85.1 ms | 279 % | 129 % | 37 % | 314 % | 89 % |
| 500 | 17.2 ms | 37.4 ms | 124.3 ms | 291 % | 132 % | 43 % | 371 % | 114 % |
| 600 | 24.8 ms | 59.1 ms | 181.9 ms | 293 % | 121 % | 42 % | 312 % | 103 % |
| 700 | 34.4 ms | 75.0 ms | 244.0 ms | 280 % | 127 % | 43 % | 467 % | 143 % |
| 800 | 44.6 ms | 104.1 ms | 315.8 ms | 277 % | 118 % | 42 % | 419 % | 135 % |
| 900 | 55.7 ms | 127.0 ms | 397.6 ms | 301 % | 134 % | 46 % | 404 % | 132 % |
| 1000 | 66.7 ms | 151.5 ms | 482.2 ms | 301 % | 134 % | 45 % | 560 % | 175 % |

MIP++ fills the model 2.4–3.0× faster than `MPSolver` for Cbc and 1.2–1.3× faster for HiGHS; MathOpt is slower still, up to 5.6× MIP++ for HiGHS at N = 1000, and it degrades as the model grows where `MPSolver` stays flat.

**SCIP is the exception in both columns**, and it is the deferral effect above in its clearest form: the OR-Tools fill time excludes the SCIP load entirely, which is why it is nearly identical across backends, while MIP++ builds directly in SCIP's native representation, whose incremental build API is slow. MIP++ appears "slower" for SCIP while doing strictly more work up front.

### OR-Tools is shown in its fastest row-filling form

The idiomatic way to fill a row in either OR-Tools API is an expression object — `LinearExpr` for `MPSolver`, `LinearExpression` for MathOpt — which accumulates the terms in a hash map before the finished row is handed over. Both APIs also let the row be opened first and its coefficients written straight into it (`MakeRowConstraint` + `SetCoefficient`, `AddLinearConstraint` + `set_coefficient`), and that is the form every OR-Tools figure on this page uses.

Going through an expression object instead costs `MPSolver` 1.6–1.9× (widening with N) and MathOpt a flat ~1.2×. So **OR-Tools is always shown at its best** here — and at its most verbose: a loop opening a row and writing N coefficients into it, where MIP++ and the expression form both state the constraint as a sum.

## MIP++ vs JuMP

`Model(optimizer)` puts a `CachingOptimizer` and the bridge layer between the model and the solver; `direct_model(optimizer())` removes both, so every `@variable` and `@constraint` goes straight into the solver's own model. Warm build times (a rebuild inside an already-compiled process):

| N | JuMP · HiGHS<br>cached | JuMP · HiGHS<br>direct | JuMP · Gurobi<br>cached | JuMP · Gurobi<br>direct |
|:---:|---:|---:|---:|---:|
| 100 | 5.8 ms | 18.7 ms | 7.4 ms | 11.2 ms |
| 200 | 26.6 ms | 77.3 ms | 26.1 ms | 45.2 ms |
| 300 | 63.4 ms | 183.4 ms | 58.6 ms | 112.1 ms |
| 400 | 174.1 ms | 355.7 ms | 176.0 ms | 246.7 ms |
| 500 | 229.9 ms | 602.0 ms | 227.2 ms | 360.4 ms |
| 600 | 327.0 ms | 963.8 ms | 324.1 ms | 508.7 ms |
| 700 | 433.2 ms | 1289.1 ms | 429.5 ms | 692.2 ms |
| 800 | 549.7 ms | 1723.7 ms | 544.1 ms | 921.0 ms |
| 900 | 691.4 ms | 2264.1 ms | 695.6 ms | 1157.0 ms |
| 1000 | 866.6 ms | 2777.5 ms | 872.9 ms | 1405.3 ms |

Cutting out the caching layer makes filling the model *slower*, by 3.2× for HiGHS and 1.6× for Gurobi — the price of actually depositing the model in the solver as you go, which is the same effect that separates MIP++ from `MPSolver` in the C++ tables, though it costs far less there (1.3× on HiGHS at N = 1000). The two solvers' incremental APIs differ enough to make direct mode twice as expensive on HiGHS as on Gurobi, a difference the cached columns hide completely.

Direct mode is the one to compare against MIP++, since it is the mode that has the model in the solver when the timer stops. Against MIP++ on HiGHS (151.5 ms at N = 1000), that is 18× for JuMP direct and 5.7× for JuMP cached.

## Python interfaces

Self-contained builders timing a single build per model:

| N | gurobipy<br>Gurobi | highspy<br>HiGHS | PuLP<br>Cbc · CPython | PuLP<br>Cbc · PyPy | Python-MIP<br>Cbc · CPython | Python-MIP<br>Cbc · PyPy |
|:---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.02 s | 0.20 s | 0.06 s | 0.06 s | 0.08 s | 0.15 s |
| 200 | 0.07 s | 0.80 s | 0.22 s | 0.15 s | 0.18 s | 0.18 s |
| 300 | 0.16 s | 1.76 s | 0.48 s | 0.31 s | 0.34 s | 0.23 s |
| 400 | 0.29 s | 3.20 s | 0.93 s | 0.53 s | 0.58 s | 0.32 s |
| 500 | 0.46 s | 4.84 s | 1.46 s | 0.84 s | 0.89 s | 0.41 s |
| 600 | 0.71 s | 6.98 s | 2.21 s | 1.26 s | 1.28 s | 0.54 s |
| 700 | 1.05 s | 9.49 s | 3.11 s | 1.78 s | 1.73 s | 0.71 s |
| 800 | 1.43 s | 12.77 s | 4.18 s | 2.38 s | 2.29 s | 0.91 s |
| 900 | 1.84 s | 15.75 s | 5.50 s | 3.09 s | 2.94 s | 1.15 s |
| 1000 | 2.33 s | 19.37 s | 6.91 s | 3.72 s | 3.61 s | 1.44 s |

Against MIP++ on the same backend at N = 1000, the whole Python field costs **one to two orders of magnitude** more: 11× for gurobipy, 54× for CPython Python-MIP, 103× for CPython PuLP and 128× for highspy.

Two secondary observations, reported for completeness rather than as claims about MIP++: `highspy` is the slowest of the four despite being a thin binding over a C++ solver, because a `highspy` integer variable takes two calls (`addVar` then `changeColIntegrality`) and each `addRow` hands over freshly built Python lists; and PyPy's JIT is worth 1.9–2.5× at N = 1000 but nothing at N = 100, where it never warms up inside a process that builds a single model.

## MIP++ across solver backends

The same MIP++ executable against every backend whose shared library was found at runtime. This is the cost of the solver's own model-building API plus MIP++'s (near-zero) overhead, so the spread is the solvers':

| N | Cbc | MOSEK | HiGHS | CPLEX | Gurobi | GLPK | Xpress | SCIP |
|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.9 ms | 2.2 ms | 1.6 ms | 1.5 ms | 3.3 ms | 1.8 ms | 2.4 ms | 7.6 ms |
| 200 | 3.1 ms | 4.2 ms | 6.2 ms | 6.9 ms | 9.2 ms | 7.1 ms | 7.5 ms | 24.1 ms |
| 300 | 7.2 ms | 8.1 ms | 15.4 ms | 15.1 ms | 18.5 ms | 16.9 ms | 17.3 ms | 53.3 ms |
| 400 | 12.3 ms | 12.7 ms | 25.4 ms | 25.3 ms | 31.8 ms | 29.4 ms | 29.3 ms | 86.6 ms |
| 500 | 19.1 ms | 19.1 ms | 38.3 ms | 47.0 ms | 49.6 ms | 53.0 ms | 48.1 ms | 126.0 ms |
| 600 | 27.0 ms | 29.0 ms | 59.6 ms | 63.8 ms | 72.9 ms | 81.1 ms | 104.6 ms | 184.5 ms |
| 700 | 37.8 ms | 37.9 ms | 79.3 ms | 85.6 ms | 101.6 ms | 128.3 ms | 99.7 ms | 251.7 ms |
| 800 | 50.1 ms | 48.2 ms | 107.4 ms | 117.4 ms | 132.8 ms | 162.0 ms | 144.5 ms | 325.7 ms |
| 900 | 61.4 ms | 61.7 ms | 133.0 ms | 171.4 ms | 168.1 ms | 237.2 ms | 219.7 ms | 409.4 ms |
| 1000 | 71.7 ms | 74.4 ms | 152.6 ms | 207.1 ms | 209.6 ms | 261.2 ms | 255.2 ms | 499.6 ms |

Cbc and MOSEK accept the model fastest and SCIP is an order of magnitude behind the rest (7.0× Cbc at N = 1000). Normalised by model size, most backends are flat across the sweep; GLPK (×1.5 from N = 200 to N = 1000), Xpress (×1.4) and CPLEX (×1.2) cost progressively more per nonzero as the model grows.

Cbc leads this table only because its `devel` branch caches `addRow` calls; on the 2.10.13 release, which rebuilds the matrix at every call, the same code is far slower. That is a property of the solver's build API, not of MIP++, and it is the clearest illustration of what the column actually measures (see the limitations below).

This is what [writing solver-generic code](solvers/generic-code.md) looks like in practice: the same source, recompiled per backend, and the spread you see is the solvers' own build APIs — not the abstraction.

## One constraint at a time, in bulk, and the `distinct_variables` hint

Four MIP++ executables are built from the same model, differing only in *how* the constraints are handed to the library: one at a time with `add_constraint(...)` or in bulk with the [`add_constraints(range, generator)` overload](modeling/expressions.md#constraint-families), each with and without the `distinct_variables` tag. That tag tells MIP++ that the terms of each linear expression reference pairwise-distinct variables, letting it skip the coefficient-merging step it would otherwise perform. In N-Queens every constraint genuinely has distinct variables, so the hint is an idiomatic use rather than a shortcut.

Both axes are backend-dependent. At N = 1000, as a percentage of that backend's one-at-a-time time:

| backend | one-at-a-time | + distinct | bulk | bulk + distinct |
|:---:|---:|---:|---:|---:|
| Cbc | 71.7 ms | 93 % | 105 % | 94 % |
| MOSEK | 74.4 ms | 90 % | 113 % | 106 % |
| HiGHS | 152.6 ms | 99 % | 93 % | 91 % |
| CPLEX | 207.1 ms | 96 % | 65 % | 66 % |
| Gurobi | 209.6 ms | 97 % | 104 % | 103 % |
| GLPK | 261.2 ms | 95 % | 100 % | 93 % |
| Xpress | 255.2 ms | 94 % | 83 % | 78 % |
| SCIP | 499.6 ms | 97 % | 100 % | 98 % |

The `distinct_variables` hint never hurts: worth 1–10 %, most on MOSEK (10 %) and Cbc (7 %), least on HiGHS (1 %) whose own build API swallows the difference. **Bulk cuts both ways** — worth 35 % for CPLEX, 17 % for Xpress and 7 % for HiGHS, nothing for GLPK and SCIP, and *counter*-productive for MOSEK (+13 %), Cbc (+5 %) and Gurobi (+4 %), whose per-constraint entry points are already the fast path. The same one-at-a-time / bulk split exists for the Gurobi C API and buys nothing there either. Prefer whichever form reads better, and measure before assuming bulk is faster.

The comparison tables on this page use the per-constraint variant with the hint, which mirrors how the OR-Tools, JuMP and Python models are written; the backend table above uses the plain variant, so that it measures the backends rather than the hint.

## Setup and methodology

- **Machine**: AMD Ryzen 7 7800X3D, Ubuntu 22.04.
- **C++**: GCC 14, `-std=c++23`, `Release`, `-flto` — the same compiler and flags for MIP++ and OR-Tools (`or-tools/9.15`, with statically linked Cbc, SCIP and HiGHS).
- **Sweep**: N from 100 to 1000 in steps of 100.
- **Repetitions**: each point is the **median** of a number of repetitions the runner picks itself — it repeats until the standard error of the median drops under 1.5 %, capped at 25 repetitions and about five seconds per point. Every repetition is a fresh process, so there is nothing to warm up and nothing to discard — except for JuMP, where starting Julia and loading the solver package costs seconds: there the build is repeated five times *inside* the process and the median reported (the "warm" figure).
- **Traceability**: each CSV records the repetitions spent on a point and the resulting error percentage, so every number can be taken with the right amount of salt. In the committed results 37 of the 570 points miss the 1.5 % target, half of them JuMP — where a single repetition costs seconds, so the time budget stops the sampling at two — and most of the rest Xpress, whose own build API stays erratic even at the 25-repetition cap. Ratios drawn from those two should be read with the recorded error next to them.

## Limitations

- **The comparison is not work-for-work.** The deferred interfaces (`MPSolver`, MathOpt, JuMP cached) have less of the model in the solver when their timer stops. This is the main caveat on every cross-interface ratio.
- **Single machine, single compiler.** Absolute times are machine-dependent; the ratios are the transferable quantity.
- **The Cbc columns need an unreleased Cbc.** They were measured against the `devel` branch, the only version that caches `addRow` calls; on release 2.10.13 — which is what `apt install coinor-libcbc-dev` provides — every direct-building interface is substantially slower on Cbc. The Cbc figures are therefore not reproducible from a distribution package today. This is a property of the solver's build API, not of MIP++, and it is the clearest illustration of what these columns actually measure.
- **Backends without a license are reported as skipped, not measured.** COPT is wired up and its library loads, but no usable license was available on the benchmark machine, so it has no column.
- **OR-Tools' Gurobi paths could not be measured** and do not fail gracefully: `MPSolver` segfaults and MathOpt throws an uncaught `std::bad_function_call` when no Gurobi license is available. MathOpt has no Cbc backend at all, and Conan's or-tools recipe does not build the GLPK one.
- **Only model construction is characterised.** Nothing here says anything about solve time, memory footprint, or the quality of the models produced.

## Reproducing

The benchmark repository builds everything through Conan, writes one CSV per interface and solver, and regenerates every table above from those CSVs with a script per section. Instructions, Conan profiles, the Cbc `devel` requirement and the known or-tools build issue are documented in [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens). Solvers, packages or licenses missing at runtime are reported as skipped rather than being fatal, so a partial reproduction on a machine with only the free solvers works out of the box.

## Why it is fast

Nothing in the numbers above comes from micro-optimisation. It follows from two design decisions, described in [Why MIP++](getting-started/index.md):

- **No intermediate model representation.** `add_constraint` calls into the solver's C API; there is no model object to fill and later translate.
- **Zero-copy expressions.** `xsum`, `+` and `*` compose standard-library views; term ranges are streamed once into a reused buffer, and nothing is allocated per row. The mechanics are in [Inside the expression layer](reference/expression-layer.md).

Together with lambda id-maps — `X(i, j)` is arithmetic, not a hash lookup — this is why a modeling layer that reads like JuMP costs a few tens of milliseconds on a million-variable model, and why that cost stays within a few percent of the solver's own C API.
