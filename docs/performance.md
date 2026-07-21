# Performance

The cost of an algebraic modeling layer is invisible for a one-shot solve of a small model, and dominant in exactly the situations research code cares about: large instances, column generation, row generation, and repeated re-solves in a benchmark loop. This page collects what is actually measured, on which the "no modeling tax" claim rests.

All numbers below come from [**mippp_nqueens**](https://github.com/fhamonic/mippp_nqueens), which times the *filling* of an N-Queens MILP — `N²` binary variables, `6N−6` constraints — through MIP++, the OR-Tools `MPSolver` C++ API, JuMP, Python-MIP and PuLP. **Only model construction is timed, never the resolution.**

## MIP++ vs the other C++ and Julia interfaces

Time in milliseconds for MIP++, and how much longer each other interface takes **with the same backend** (higher = slower than MIP++):

| N | MIP++<br>Cbc | MIP++<br>HiGHS | OR-tools<br>Cbc | OR-tools<br>HiGHS | JuMP<br>Cbc | JuMP<br>HiGHS |
|:---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.9 ms | 1.5 ms | 4.0 × | 2.3 × | 14.3 × | 43.9 × |
| 200 | 3.2 ms | 5.9 ms | 4.2 × | 2.3 × | 30.8 × | 4.5 × |
| 300 | 6.8 ms | 14.2 ms | 4.6 × | 2.2 × | 10.8 × | 7.8 × |
| 400 | 12.2 ms | 23.4 ms | 4.4 × | 2.3 × | 15.4 × | 7.4 × |
| 500 | 18.9 ms | 36.6 ms | 4.8 × | 2.4 × | 13.6 × | 7.4 × |
| 600 | 27.6 ms | 57.7 ms | 4.7 × | 2.3 × | 12.7 × | 6.3 × |
| 700 | 37.2 ms | 75.9 ms | 4.7 × | 2.3 × | 15.0 × | 5.9 × |
| 800 | 48.7 ms | 102.1 ms | 4.7 × | 2.3 × | 12.4 × | 6.8 × |
| 900 | 61.8 ms | 124.6 ms | 5.0 × | 2.5 × | 13.0 × | 6.1 × |
| 1000 | 72.8 ms | 146.2 ms | 5.2 × | 2.6 × | 13.7 × | 6.5 × |

Reading it: a 1000-Queens model — one million binary variables and ~6000 constraints — is built in **73 ms** through Cbc and **146 ms** through HiGHS. OR-Tools takes 2.2–2.6× (HiGHS) or 4.0–5.2× (Cbc) longer. JuMP takes 6–15× longer from N = 400 on; its two outlying points (43.9× at N = 100 with HiGHS, 30.8× at N = 200 with Cbc) sit where the absolute times are smallest and Julia's warm-up noise largest.

Cbc is the one backend common to every interface compared, and HiGHS is shared by MIP++, OR-Tools and JuMP, which is why the tables use them.

## Where MIP++ is *not* fastest: SCIP

| N | MIP++<br>Cbc | MIP++<br>HiGHS | MIP++<br>SCIP | OR-tools<br>Cbc | OR-tools<br>HiGHS | OR-tools<br>SCIP |
|:---:|---:|---:|---:|---:|---:|---:|
| 100 | 0.9 ms | 1.5 ms | 8.3 ms | 4.0 × | 2.3 × | 0.4 × |
| 500 | 18.9 ms | 36.6 ms | 146.7 ms | 4.8 × | 2.4 × | 0.6 × |
| 1000 | 72.8 ms | 146.2 ms | 661.3 ms | 5.2 × | 2.6 × | 0.6 × |

With SCIP, OR-Tools takes **0.4–0.6×** the MIP++ time — it is the faster of the two. The reason is structural and worth understanding before reading any of these tables as a verdict:

!!! note "What each layer's timing includes"
    `MPSolver` (and JuMP in its default *cached* mode) stores the model in its own backend-independent data structures and only extracts it to the underlying solver when `Solve()` is called. Their fill times therefore **exclude the actual solver load**, which is also why they barely change between backends.

    MIP++ has no intermediate representation at all: its timings are the cost of building the model in the solver's own native in-memory form. Against a backend whose C API is itself slow to populate — SCIP here — that shows up as a loss on this metric, while the deferred layers simply pay it later, inside `Solve()`.

The practical consequence for a research code is the opposite of what the SCIP row suggests: with MIP++ the model *is* in the solver when construction returns, so re-solves, in-place [model updates](solving/updates.md) and [column generation](algorithms/column-generation.md) rounds do not re-extract anything.

## Python interfaces

Time in milliseconds for JuMP, and the Python layers as multiples of **JuMP with Cbc** (all Python entries use Cbc):

| N | JuMP<br>Cbc | JuMP<br>HiGHS | Python-MIP<br>Cbc (CPython) | Python-MIP<br>Cbc (PyPy) | PuLP<br>Cbc (CPython) | PuLP<br>Cbc (PyPy) |
|:---:|---:|---:|---:|---:|---:|---:|
| 100 | 13.0 ms | 66.6 ms | 12.3 × | 12.1 × | 10.6 × | 5.4 × |
| 200 | 99.8 ms | 26.7 ms | 8.1 × | 0.9 × | 9.2 × | 1.7 × |
| 300 | 74.1 ms | 111.0 ms | 36.9 × | 2.7 × | 40.0 × | 4.9 × |
| 400 | 187.0 ms | 174.4 ms | 36.2 × | 2.1 × | 39.3 × | 3.6 × |
| 500 | 257.9 ms | 270.7 ms | 50.7 × | 2.3 × | 54.5 × | 4.4 × |
| 600 | 348.6 ms | 365.0 ms | 64.1 × | 2.6 × | 70.1 × | 5.1 × |
| 700 | 558.7 ms | 450.9 ms | 64.8 × | 2.3 × | 70.3 × | 4.5 × |
| 800 | 606.3 ms | 689.3 ms | 88.1 × | 3.1 × | 96.9 × | 5.7 × |
| 900 | 802.8 ms | 759.6 ms | 94.9 × | 3.3 × | 104.7 × | 5.8 × |
| 1000 | 994.5 ms | 956.7 ms | 107.1 × | 3.3 × | 116.0 × | 6.1 × |

Chaining the two baselines at N = 1000 — JuMP-Cbc is 13.7× MIP++-Cbc, and Python-MIP under CPython is 107.1× JuMP-Cbc — puts the CPython layers around **10³×** the MIP++ model-building time (≈ 1.5 × 10³ for Python-MIP, ≈ 1.6 × 10³ for PuLP), and the PyPy runs around **50×**. Treat those composite figures as orders of magnitude rather than measurements: unlike the C++ and Julia benchmarks, the Python scripts time a *single* build with no warm-up run, and the JuMP-Cbc column they are normalised against is visibly noisy (the N = 200 row).

## Setup and methodology

- **Machine**: AMD Ryzen 7 7800X3D, Ubuntu 22.04.
- **C++**: GCC 14.1, `-std=c++23`, `Release`, `-flto` — the same compiler and flags for MIP++ and OR-Tools (`or-tools/9.15`, with statically linked Cbc, SCIP and HiGHS).
- **Sweep**: N from 100 to 1000 in steps of 100.
- **Repetitions**: each point is run 5–20 times (fewer for the larger models); the first run is dropped as warm-up and the rest averaged. JuMP reports both a cold time, including Julia's JIT compilation, and the warm rebuild — **the tables use the warm one**, which is two orders of magnitude smaller.
- **Python**: `mip==1.15.0`, PuLP, run through the benchmark scripts of the [python-mip repository](https://github.com/coin-or/python-mip) unmodified, which time a single build without warm-up.
- Only construction is timed; nothing is solved.

### One constraint at a time, or in bulk

The MIP++ numbers above come from the executable that adds constraints **one at a time** with `model.add_constraint(...)`, mirroring how the OR-Tools, JuMP, Python-MIP and PuLP models are written, so that the comparison stays apples-to-apples. A second executable (`mippp_bulk`) uses the [bulk `add_constraints(range, generator)` overload](modeling/expressions.md#constraint-families) instead, and the same split exists on the Gurobi C API side (`GRBaddconstr` per row vs. one `GRBaddconstrs` call).

In other words, the tables measure MIP++ in its *least* favourable idiom; the bulk form is the one used throughout this documentation and in [`examples/nqueens.cpp`](https://github.com/fhamonic/mippp/blob/main/examples/nqueens.cpp).

## Reproducing

The benchmark repository builds everything through Conan and writes one CSV per interface and solver; the tables in its README are regenerated from those CSVs. Instructions, profiles and the known or-tools build issue are documented in [mippp_nqueens](https://github.com/fhamonic/mippp_nqueens). The runners also emit CSVs for every other solver they find at runtime — GLPK, SCIP, CPLEX, MOSEK, COPT, Gurobi and Xpress for MIP++ — which are simply not tabulated upstream.

## Why it is fast

Nothing in the numbers above comes from micro-optimisation. It follows from two design decisions, described in [Why MIP++](getting-started/index.md):

- **No intermediate model representation.** `add_constraint` calls into the solver's C API; there is no model object to fill and later translate.
- **Zero-copy expressions.** `xsum`, `+` and `*` compose standard-library views; term ranges are streamed once into a reused buffer, and nothing is allocated per row. The mechanics are in [Inside the expression layer](reference/expression-layer.md).

Together with lambda id-maps — `X(i, j)` is arithmetic, not a hash lookup — this is why a modeling layer that reads like JuMP costs a few tens of milliseconds on a million-variable model.
