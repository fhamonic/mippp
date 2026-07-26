# Performance

The cost of an algebraic modeling layer is invisible for a one-shot solve where solver time dominates — a two-second build in front of a two-hour solve is noise, whatever layer produced it. It becomes real in two regimes: very large models, where construction is a nontrivial fraction of wall-clock time, and iterative methods — column generation, row generation, repeated re-solves in a benchmark loop — that touch the model thousands of times. The numbers below quantify what MIP++ buys you in those regimes, and only those; this page collects what is actually measured, on which the "no modeling tax" claim rests.

All numbers come from [**mippp_nqueens**](https://github.com/fhamonic/mippp_nqueens), which times the *filling* of an N-Queens MILP — `N²` binary variables, `6N−6` constraints, ≈ 4·10⁶ nonzeros at N = 1000 — through eight modeling libraries spanning C++, Julia and Python, on up to seven solver backends.

!!! note "What is timed"
    **Only model construction, never the resolution.** The timer starts once the solver API object exists and stops once the model holds all `N²` variables and all `6N−6` constraints, after forcing any pending update to be flushed. No presolve, no optimization, no solution retrieval.

!!! note "What the N-Queens shape does and does not stress"
    N-Queens is variable-heavy and constraint-light: a million variables against ~6000 rows, each row a plain sum of existing variables. The benchmark therefore chiefly measures **variable creation and short-row streaming** — a shape that favours a zero-copy expression system. It says nothing about models dominated by dense, structured constraints, nor about solve time, memory footprint or model quality. Read the multiples as evidence for the build-bound regimes described above, not as a universal speedup.

Every model in the comparison is written in its best-case form. The natural way to express a diagonal constraint — filtering the whole board once per diagonal — makes filling a Θ(N²) model cost Θ(N³), which at N = 1000 costs the PuLP model 115 s where indexing each diagonal directly builds the same model in 6.9 s. None of that factor happens inside PuLP. Every model here touches only its own nonzeros, addressed by index, so the tables report a *floor*: the cost of the interface layer itself.

## MIP++ vs the Gurobi C API

The most direct measure of MIP++'s overhead. The pure Gurobi C API in absolute milliseconds, MIP++ as a *percentage* of it, and the other Gurobi-capable interfaces as multiples:

| N | Gurobi C API<br>per constr. | Gurobi C API<br>bulk | MIP++ | gurobipy | JuMP<br>warm | JuMP<br>cold | Python-MIP<br>CPython | Python-MIP<br>PyPy |
|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 3.2 ms | 103.0 % | 99.3 % | 6.5 × | 3.7 × | 73.2 × | 19.7 × | 36.6 × |
| 200 | 8.3 ms | 105.8 % | 102.1 % | 9.0 × | 5.5 × | 40.2 × | 19.8 × | 17.6 × |
| 300 | 16.7 ms | 102.3 % | 102.8 % | 9.7 × | 7.3 × | 28.5 × | 19.3 × | 11.7 × |
| 400 | 29.3 ms | 102.6 % | 100.5 % | 9.7 × | 8.9 × | 19.1 × | 19.1 × | 9.4 × |
| 500 | 46.1 ms | 100.8 % | 99.3 % | 9.9 × | 8.0 × | 14.2 × | 18.7 × | 7.8 × |
| 600 | 66.0 ms | 101.4 % | 101.7 % | 10.6 × | 8.0 × | 13.5 × | 18.5 × | 7.2 × |
| 700 | 92.4 ms | 98.5 % | 102.0 % | 10.9 × | 7.7 × | 11.6 × | 18.1 × | 6.9 × |
| 800 | 120.6 ms | 99.9 % | 101.0 % | 11.5 × | 7.8 × | 10.4 × | 18.3 × | 6.9 × |
| 900 | 152.3 ms | 102.7 % | 103.7 % | 11.9 × | 7.7 × | 9.6 × | 18.7 × | 6.8 × |
| 1000 | 190.2 ms | 100.5 % | 103.1 % | 12.1 × | 7.5 × | 9.6 × | 18.4 × | 6.9 × |

MIP++ tracks the raw C API to within a few percent across the whole sweep (≈ 99–104 %): **the modeling layer is essentially free.** Handing Gurobi the whole matrix in a single `GRBaddconstrs` call instead of one `GRBaddconstr` per constraint is worth nothing (98–106 %), so matching the per-constraint path is the meaningful comparison rather than a handicap.

The JuMP columns are `direct_model`; the `cold` one includes Julia's JIT compilation, paid once per process, and is the only figure on this page that improves with N.

## MIP++ vs the other C++ and Julia interfaces

Time in milliseconds for MIP++, and how much longer each other interface takes **on the same backend**. HiGHS is the one backend all of them share: MathOpt has no Cbc, and JuMP's direct mode needs an incrementally modifiable backend, which the Cbc wrapper is not.

| N | MIP++ | OR-tools<br>MPSolver | OR-tools<br>MathOpt | JuMP<br>cached | JuMP<br>direct |
|:---:|---:|---:|---:|---:|---:|
| 100 | 1.5 ms | 2.4 × | 3.3 × | 4.1 × | 13.1 × |
| 200 | 5.6 ms | 2.4 × | 3.6 × | 4.8 × | 13.9 × |
| 300 | 13.5 ms | 2.3 × | 3.5 × | 4.3 × | 14.8 × |
| 400 | 22.2 ms | 2.4 × | 4.0 × | 8.0 × | 17.1 × |
| 500 | 34.1 ms | 2.6 × | 5.2 × | 6.8 × | 16.6 × |
| 600 | 54.8 ms | 2.4 × | 4.3 × | 6.1 × | 18.0 × |
| 700 | 68.2 ms | 2.5 × | 6.4 × | 6.3 × | 19.6 × |
| 800 | 95.9 ms | 2.3 × | 5.8 × | 5.8 × | 18.2 × |
| 900 | 116.3 ms | 2.6 × | 6.0 × | 6.0 × | 20.2 × |
| 1000 | 137.6 ms | 2.8 × | 7.7 × | 6.5 × | 21.1 × |

Reading it: a 1000-Queens model — one million binary variables and ~6000 constraints — is filled into HiGHS in **138 ms**, and into Cbc in **67 ms**. OR-Tools' `MPSolver` takes 2.3–2.8× longer, MathOpt 3.3–7.7×, JuMP 4.1–8.0× in cached mode and 13.1–21.1× in direct mode.

### Deferred vs. direct model construction

One structural difference explains most of the spread between interfaces, and it is not the language.

!!! important "The comparison is not work-for-work, and the bias is against MIP++"
    Both OR-Tools APIs and JuMP's default `Model` accumulate the model in their own data structures and translate it for the solver later, whereas MIP++, the Gurobi C API and JuMP's `direct_model` write into the solver's own model as each constraint is added.

    Two independent tells confirm the deferral: JuMP's cached mode fills at the same speed whichever backend is named (888 ms for HiGHS, 887 ms for Gurobi at N = 1000), and `MPSolver` needs ~385 ms at N = 1000 for Cbc, HiGHS and SCIP alike. Wherever a deferred interface looks cheap, it has simply not handed the solver anything yet.

The practical consequence for a research code is the one the ratios understate: with MIP++ the model *is* in the solver when construction returns, so re-solves, in-place [model updates](solving/updates.md) and [column generation](algorithms/column-generation.md) rounds do not re-extract anything.

## MIP++ vs OR-Tools, and where MIP++ *loses*

OR-Tools ships two C++ modeling APIs and both are benchmarked: `MPSolver` and the newer MathOpt. MIP++ in absolute milliseconds, each OR-Tools API as a percentage of it:

| N | MIP++<br>Cbc | MIP++<br>HiGHS | MIP++<br>SCIP | MPSolver<br>Cbc | MPSolver<br>HiGHS | MPSolver<br>SCIP | MathOpt<br>HiGHS | MathOpt<br>SCIP |
|:---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.8 ms | 1.5 ms | 8.9 ms | 422 % | 241 % | 40 % | 327 % | 55 % |
| 200 | 2.9 ms | 5.6 ms | 26.7 ms | 465 % | 245 % | 53 % | 361 % | 77 % |
| 300 | 6.3 ms | 13.5 ms | 60.9 ms | 507 % | 228 % | 52 % | 348 % | 74 % |
| 400 | 11.3 ms | 22.2 ms | 95.7 ms | 477 % | 241 % | 57 % | 401 % | 91 % |
| 500 | 17.3 ms | 34.1 ms | 147.2 ms | 524 % | 264 % | 62 % | 520 % | 111 % |
| 600 | 25.0 ms | 54.8 ms | 219.3 ms | 525 % | 236 % | 60 % | 432 % | 101 % |
| 700 | 33.2 ms | 68.2 ms | 310.5 ms | 537 % | 253 % | 58 % | 642 % | 131 % |
| 800 | 42.7 ms | 95.9 ms | 421.8 ms | 535 % | 234 % | 54 % | 577 % | 130 % |
| 900 | 55.0 ms | 116.3 ms | 550.6 ms | 572 % | 265 % | 58 % | 597 % | 124 % |
| 1000 | 67.0 ms | 137.6 ms | 680.4 ms | 573 % | 279 % | 58 % | 770 % | 156 % |

MIP++ fills the model 5–6× faster than `MPSolver` for Cbc and ~2.5× faster for HiGHS; MathOpt is slower still, up to 7.7× MIP++ for HiGHS at N = 1000, and it degrades as the model grows where `MPSolver` stays flat.

**SCIP is the exception in both columns**, and it is the deferral effect above in its clearest form: the OR-Tools fill time excludes the SCIP load entirely, which is why it is nearly identical across backends, while MIP++ builds directly in SCIP's native representation, whose incremental build API is slow. MIP++ appears "slower" for SCIP while doing strictly more work up front.

## MIP++ vs JuMP

`Model(optimizer)` puts a `CachingOptimizer` and the bridge layer between the model and the solver; `direct_model(optimizer())` removes both, so every `@variable` and `@constraint` goes straight into the solver's own model. Warm build times (a rebuild inside an already-compiled process):

| N | JuMP · HiGHS<br>cached | JuMP · HiGHS<br>direct | JuMP · Gurobi<br>cached | JuMP · Gurobi<br>direct |
|:---:|---:|---:|---:|---:|
| 100 | 6.0 ms | 19.2 ms | 7.7 ms | 11.8 ms |
| 200 | 26.7 ms | 77.3 ms | 25.9 ms | 45.8 ms |
| 300 | 57.9 ms | 199.8 ms | 59.6 ms | 121.1 ms |
| 400 | 176.7 ms | 378.7 ms | 177.8 ms | 260.1 ms |
| 500 | 231.7 ms | 567.8 ms | 236.6 ms | 370.9 ms |
| 600 | 332.6 ms | 988.7 ms | 342.0 ms | 527.3 ms |
| 700 | 430.0 ms | 1339.8 ms | 441.0 ms | 707.3 ms |
| 800 | 556.4 ms | 1750.7 ms | 547.7 ms | 940.0 ms |
| 900 | 699.9 ms | 2351.1 ms | 712.9 ms | 1175.7 ms |
| 1000 | 888.3 ms | 2908.8 ms | 886.6 ms | 1424.9 ms |

Cutting out the caching layer makes filling the model *slower*, by 3.3× for HiGHS and 1.6× for Gurobi — the price of actually depositing the model in the solver as you go, and the same gap the C++ interfaces show between MIP++ and `MPSolver`. The two solvers' incremental APIs differ enough to make direct mode twice as expensive on HiGHS as on Gurobi, a difference the cached columns hide completely.

Direct mode is the one to compare against MIP++, since it is the mode that has the model in the solver when the timer stops. Against MIP++ on HiGHS (137.6 ms at N = 1000), that is 21× for JuMP direct and 6.5× for JuMP cached.

## Python interfaces

Self-contained builders timing a single build per model:

| N | gurobipy<br>Gurobi | highspy<br>HiGHS | PuLP<br>Cbc · CPython | PuLP<br>Cbc · PyPy | Python-MIP<br>Cbc · CPython | Python-MIP<br>Cbc · PyPy |
|:---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.02 s | 0.21 s | 0.06 s | 0.06 s | 0.08 s | 0.15 s |
| 200 | 0.07 s | 0.79 s | 0.22 s | 0.16 s | 0.18 s | 0.18 s |
| 300 | 0.16 s | 1.76 s | 0.50 s | 0.31 s | 0.34 s | 0.23 s |
| 400 | 0.29 s | 3.09 s | 0.92 s | 0.53 s | 0.57 s | 0.31 s |
| 500 | 0.46 s | 4.92 s | 1.50 s | 0.85 s | 0.87 s | 0.40 s |
| 600 | 0.70 s | 7.01 s | 2.34 s | 1.28 s | 1.25 s | 0.54 s |
| 700 | 1.01 s | 9.47 s | 3.20 s | 1.81 s | 1.71 s | 0.71 s |
| 800 | 1.39 s | 12.32 s | 4.38 s | 2.38 s | 2.23 s | 0.91 s |
| 900 | 1.81 s | 15.70 s | 5.69 s | 3.13 s | 2.90 s | 1.13 s |
| 1000 | 2.30 s | 19.50 s | 6.89 s | 3.77 s | 3.58 s | 1.42 s |

Against MIP++ on the same backend, the whole Python field costs **one to two orders of magnitude** more: 12× for gurobipy, ~50× for CPython Python-MIP, ~100× for CPython PuLP and ~140× for highspy.

Two secondary observations, reported for completeness rather than as claims about MIP++: `highspy` is the slowest of the five despite being a thin binding over a C++ solver, because a `highspy` integer variable takes two calls (`addVar` then `changeColIntegrality`) and each `addRow` hands over freshly built Python lists; and PyPy's JIT is worth 1.8–2.5× at N = 1000 but nothing at N = 100, where it never warms up inside a process that builds a single model.

## MIP++ across solver backends

The same MIP++ executable against every backend whose shared library was found at runtime. This is the cost of the solver's own model-building API plus MIP++'s (near-zero) overhead, so the spread is the solvers':

| N | Cbc | MOSEK | HiGHS | CPLEX | Gurobi | GLPK | SCIP |
|:---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.9 ms | 2.2 ms | 1.5 ms | 1.4 ms | 3.3 ms | 1.6 ms | 8.4 ms |
| 200 | 3.1 ms | 4.2 ms | 5.9 ms | 5.8 ms | 9.0 ms | 6.9 ms | 27.8 ms |
| 300 | 6.8 ms | 8.3 ms | 14.4 ms | 14.3 ms | 17.5 ms | 15.6 ms | 59.9 ms |
| 400 | 12.5 ms | 12.7 ms | 23.0 ms | 24.1 ms | 30.1 ms | 28.0 ms | 94.0 ms |
| 500 | 19.4 ms | 18.8 ms | 35.7 ms | 44.1 ms | 47.5 ms | 52.3 ms | 146.8 ms |
| 600 | 28.6 ms | 29.9 ms | 57.5 ms | 60.6 ms | 71.4 ms | 78.1 ms | 220.4 ms |
| 700 | 37.6 ms | 37.6 ms | 73.8 ms | 78.2 ms | 95.8 ms | 124.7 ms | 306.0 ms |
| 800 | 49.1 ms | 48.7 ms | 103.6 ms | 108.4 ms | 128.3 ms | 161.6 ms | 410.9 ms |
| 900 | 61.5 ms | 61.9 ms | 122.6 ms | 161.3 ms | 160.5 ms | 235.9 ms | 537.8 ms |
| 1000 | 74.1 ms | 74.2 ms | 144.3 ms | 195.7 ms | 197.5 ms | 256.1 ms | 677.9 ms |

Cbc and MOSEK accept the model fastest and SCIP is an order of magnitude behind the rest. Normalised by model size, most backends are flat across the sweep; GLPK (×1.5 from N = 200 to N = 1000) and CPLEX (×1.3) cost progressively more per nonzero as the model grows.

This is what [writing solver-generic code](solvers/generic-code.md) looks like in practice: the same source, recompiled per backend, and the spread you see is the solvers' own build APIs — not the abstraction.

## One constraint at a time, in bulk, and the `distinct_variables` hint

Four MIP++ executables are built from the same model, differing only in *how* the constraints are handed to the library: one at a time with `add_constraint(...)` or in bulk with the [`add_constraints(range, generator)` overload](modeling/expressions.md#constraint-families), each with and without the `distinct_variables` tag. That tag tells MIP++ that the terms of each linear expression reference pairwise-distinct variables, letting it skip the coefficient-merging step it would otherwise perform. In N-Queens every constraint genuinely has distinct variables, so the hint is an idiomatic use rather than a shortcut.

Both axes are backend-dependent. At N = 1000, as a percentage of that backend's one-at-a-time time:

| backend | one-at-a-time | + distinct | bulk | bulk + distinct |
|:---:|---:|---:|---:|---:|
| Cbc | 74.1 ms | 90 % | 99 % | 91 % |
| MOSEK | 74.2 ms | 91 % | 113 % | 106 % |
| HiGHS | 144.3 ms | 95 % | 100 % | 95 % |
| CPLEX | 195.7 ms | 95 % | 69 % | 64 % |
| Gurobi | 197.5 ms | 99 % | 106 % | 104 % |
| GLPK | 256.1 ms | — | 104 % | — |
| SCIP | 677.9 ms | 100 % | 99 % | 99 % |

The `distinct_variables` hint never hurts: 5–10 % for Cbc, MOSEK and HiGHS, and nothing at all for SCIP, whose own build API dominates. **Bulk cuts both ways** — worth 31 % for CPLEX, worth nothing for Cbc, HiGHS and SCIP, and *counter*-productive for MOSEK (+13 %) and Gurobi (+6 %), whose per-constraint entry points are already the fast path. The same one-at-a-time / bulk split exists for the Gurobi C API and buys nothing there either. Prefer whichever form reads better, and measure before assuming bulk is faster.

The comparison tables on this page use the per-constraint variant with the hint, which mirrors how the OR-Tools, JuMP and Python models are written; the backend table above uses the plain variant so that GLPK can appear in it (see the limitations below).

## Setup and methodology

- **Machine**: AMD Ryzen 7 7800X3D, Ubuntu 22.04.
- **C++**: GCC 14, `-std=c++23`, `Release`, `-flto` — the same compiler and flags for MIP++ and OR-Tools (`or-tools/9.15`, with statically linked Cbc, SCIP and HiGHS).
- **Sweep**: N from 100 to 1000 in steps of 100.
- **Repetitions**: each point is the **median** of a number of repetitions the runner picks itself — it repeats until the standard error of the median drops under 1.5 %, capped at 25 repetitions and about five seconds per point. Every repetition is a fresh process, so there is nothing to warm up and nothing to discard — except for JuMP, where starting Julia and loading the solver package costs seconds: there the build is repeated five times *inside* the process and the median reported (the "warm" figure).
- **Traceability**: each CSV records the repetitions spent on a point and the resulting error percentage, so every number can be taken with the right amount of salt.

## Limitations

- **The comparison is not work-for-work.** The deferred interfaces (`MPSolver`, MathOpt, JuMP cached) have less of the model in the solver when their timer stops. This is the main caveat on every cross-interface ratio.
- **Single machine, single compiler.** Absolute times are machine-dependent; the ratios are the transferable quantity.
- **The Cbc columns need an unreleased Cbc.** They were measured against the `devel` branch, the only version that caches `addRow` calls; on release 2.10.13 — which is what `apt install coinor-libcbc-dev` provides — every direct-building interface is substantially slower on Cbc. The Cbc figures are therefore not reproducible from a distribution package today. This is a property of the solver's build API, not of MIP++, and it is the clearest illustration of what these columns actually measure.
- **The `distinct_variables` path aborts on GLPK** (`glp_set_mat_row: ind[1] = 0; column index out of range`): the hinted path hands GLPK 0-based column indices where its API is 1-based. GLPK therefore has no `distinct` figure.
- **Backends without a license are reported as skipped, not measured.** COPT and Xpress are wired up and their libraries load, but neither had a usable license on the benchmark machine.
- **OR-Tools' Gurobi paths could not be measured** and do not fail gracefully: `MPSolver` segfaults and MathOpt throws an uncaught `std::bad_function_call` when no Gurobi license is available. MathOpt has no Cbc backend at all, and Conan's or-tools recipe does not build the GLPK one.
- **Only model construction is characterised.** Nothing here says anything about solve time, memory footprint, or the quality of the models produced.

## Reproducing

The benchmark repository builds everything through Conan, writes one CSV per interface and solver, and regenerates every table above from those CSVs with a script per section. Instructions, Conan profiles, the Cbc `devel` requirement and the known or-tools build issue are documented in [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens). Solvers, packages or licenses missing at runtime are reported as skipped rather than being fatal, so a partial reproduction on a machine with only the free solvers works out of the box.

## Why it is fast

Nothing in the numbers above comes from micro-optimisation. It follows from two design decisions, described in [Why MIP++](getting-started/index.md):

- **No intermediate model representation.** `add_constraint` calls into the solver's C API; there is no model object to fill and later translate.
- **Zero-copy expressions.** `xsum`, `+` and `*` compose standard-library views; term ranges are streamed once into a reused buffer, and nothing is allocated per row. The mechanics are in [Inside the expression layer](reference/expression-layer.md).

Together with lambda id-maps — `X(i, j)` is arithmetic, not a hash lookup — this is why a modeling layer that reads like JuMP costs a few tens of milliseconds on a million-variable model, and why that cost is indistinguishable from the solver's own C API.
