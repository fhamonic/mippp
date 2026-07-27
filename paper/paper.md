---
title: 'MIP++: solver-agnostic algebraic modeling for mathematical programming in C++23'
tags:
  - C++
  - operations research
  - mathematical programming
  - linear programming
  - mixed-integer programming
  - algebraic modeling
authors:
  - name: François Hamonic
    orcid: 0000-0002-3383-3100
    affiliation: 1
affiliations:
  - name: Aix-Marseille Univ, CNRS, Univ Avignon, IRD, IMBE, Marseille, France
    index: 1
date: 25 July 2026
bibliography: paper.bib
---

# Summary

MIP++ is a header-only C++23 library for modeling and solving linear programs
(LP), mixed-integer linear programs (MILP), and — on backends that support
them, currently HiGHS — quadratic objectives. It provides an algebraic
modeling syntax comparable in readability to JuMP [@jump2023] or Pyomo
[@pyomo2011] — variables, expressions built with overloaded operators, sums
over index ranges, and constraint families — while compiling down to direct
calls into each solver's native C API. The same model code can target any of
eleven solver backends (Gurobi [@gurobi], CPLEX [@cplex], Xpress [@xpress],
COPT [@copt], MOSEK [@mosek], HiGHS [@highs2018], SCIP [@scip8], Cbc [@cbc],
Clp [@clp], SoPlex [@soplex], and GLPK [@glpk]); the backend is selected at
compile time and its shared library is discovered and loaded at runtime, so no
solver SDK needs to be present at link time and a single compiled binary runs
on whatever solver the target machine has installed. Beyond a small
dynamic-loading helper the library is dependency-free; GoogleTest and MELON
[@melon] are needed only to build the test suite.

Constraint families are written over ranges, close to their mathematical
statement. The row constraints of an N-Queens model, for instance, read:

```cpp
highs_api api;          // loads the HiGHS shared library at runtime
highs_milp model(api);  // or gurobi_milp, cplex_milp, …
auto indices = std::views::iota(0, n);
auto X = model.add_binary_variables(
    n * n, [n](int row, int col) { return row * n + col; });
model.add_constraints(indices, [&](int row) {
    return xsum(indices, [&, row](int col) { return X(row, col); }) == 1;
});
```

The expression layer is functional and allocation-free: objectives and
constraint families are composed from C++ ranges as lazy views, and `xsum`
expresses sums over index sets. When a constraint is added, its term range is
iterated directly into pre-allocated scratch buffers passed to the solver's C
entry points (`Highs_addRow`, `GRBaddconstr`, …); no intermediate model
representation is built, extracted, or garbage-collected. A MIP++ model *is*
the solver's model, so re-solves after in-place modifications — added rows or
columns, changed bounds or coefficients, removed variables — pay only the
solver's incremental update cost.

Beyond model construction, MIP++ exposes the facilities that decomposition and
cutting-plane methods need: branch-and-cut callbacks with lazy constraints,
column generation with `add_column` and a column-pool manager, dual values,
reduced costs, MIP starts, LP basis access, and SOS and indicator constraints.
Solve statuses are not flattened into a lowest-common-denominator enum: each
backend returns a `std::variant` whose alternatives are exactly the outcomes
that solver reports, arranged in a type hierarchy so that generic queries
(`is_a<status::infeasible_or_unbounded>`) work everywhere while exact ones
(`is<status::primal_and_dual_infeasible>`) compile only on backends that can
report them — a distinction resolved entirely at compile time.

# Statement of need

Researchers in operations research and combinatorial optimization face an
uncomfortable trade-off. High-level modeling languages such as JuMP
[@jump2023], Pyomo [@pyomo2011], PuLP [@pulp2011], and Python-MIP
[@pythonmip2020] make models easy to write and solver-independent, but their
model-construction overhead becomes a real cost in workflows that build many
models — column generation, Benders decomposition, cutting planes, iterated
reoptimization, or large-scale experiments — where models are built, modified,
and re-solved constantly rather than solved once. Conversely, coding directly
against a solver's C API is maximally fast but verbose, error-prone, and
locked to a single vendor, which undermines both reproducibility and fair
computational comparisons across solvers.

Existing C++ alternatives only partially resolve this tension. Google OR-Tools
[@ortools] is the closest competitor, offering solver-agnostic linear and
mixed-integer modeling over several of the same backends through `MPSolver`
and, more recently, MathOpt. Both are part of a large compiled library that
must be linked against the chosen solvers, both route models through a
backend-independent representation before they reach the solver, and neither
exposes the full range of algorithmic hooks MIP++ targets, such as column
generation with reduced-cost access across backends. A cache-free path is not
unique to MIP++ — JuMP's `direct_model` writes straight to the solver too —
but in MIP++ it is the only mode, and it is combined with runtime backend
loading, so the choice of solver never reaches the build system. Elsewhere,
Gravity [@gravity2018] provides algebraic modeling in C++ but links its
solvers statically, the COIN-OR Open Solver Interface [@osi] abstracts solvers
at the matrix level without algebraic modeling, and FlopC++ [@flopcpp2007]
predates modern C++ facilities and is no longer actively developed.
Solver-vendor C++ APIs are expressive but proprietary to one solver each. The
same overhead concern has recently driven work in Python, notably
PyOptInterface [@pyoptinterface2024].

MIP++ removes the trade-off by using C++23 ranges, concepts, and lazy views to
keep the modeling layer thin. On a model-construction benchmark (N-Queens,
$N^2$ binary variables and $6N-6$ constraints; only construction is timed,
never the solve), MIP++ builds models within 2–8 % of hand-written C against
the Gurobi C API, 1.2–1.3$\times$ faster than OR-Tools' `MPSolver` on HiGHS
and 2.4–3.0$\times$ faster on Cbc (2.6–5.6$\times$ faster than OR-Tools'
MathOpt on HiGHS), and 3.7–7.3$\times$ faster than JuMP in its default cached
mode after warm-up; Python layers are one to two orders of magnitude slower,
though those scripts time a single build without warm-up and should be read as
orders of magnitude. Both OR-Tools APIs are measured in their fastest
row-filling form, but the comparison is still not like-for-like: `MPSolver`
fills its own backend-independent structures and defers the native model build
to `Solve()`, which the MIP++ timings include — on SCIP this makes the
OR-Tools fill phase measure 0.3–0.5$\times$ of a full MIP++ build. The model
is also variable-heavy and constraint-light. Full tables, hardware and library
versions, and reproduction instructions are in a companion repository
[@mippp_nqueens].

Solver independence, in turn, makes computational studies portable:
benchmarking Gurobi against HiGHS or SCIP is a two-line change. The
per-backend feature matrices (duals, callbacks, MIP starts, basis access) are
verified by a shared, backend-instantiated test suite; continuous integration
runs it on the four open-source backends installable there (Clp, Cbc, GLPK,
HiGHS) across GCC 14, GCC 15, Clang 18 and MinGW, and the same suites are run
manually against the commercial backends. Because backends are loaded rather
than linked, a generated compatibility matrix additionally records which
released versions of each solver library the wrapper still drives correctly: 101
published libraries across all eleven solvers, each downloaded and run through
the full backend suite rather than assumed compatible from its version number.
It documents real breakage — Cbc 2.10.8 and earlier abort inside the MILP
suite — that version numbers alone would not reveal.

MIP++ grew out of the author's doctoral work on optimizing the ecological
connectivity of landscapes [@hamonic2023], where a flow-based MILP formulation
is coupled with graph algorithms — from the companion MELON library [@melon] —
that contract the instance graphs during model construction, and where columns
and cuts come from shortest-path and flow computations. The modeling layer sits
inside the algorithmic loop, so per-call overhead is paid thousands of times:
Python layers made this prohibitive, and raw solver C APIs made it
non-portable.

The library targets a deliberate niche: optimization embedded in a larger C++
system that must run against whatever solver is installed, cross-solver
computational studies, and build-bound iterative methods. It requires GCC 14
or Clang 18 in C++23 mode (GCC 15 in C++26 mode remains the primary target)
and assumes comfort with modern C++ — ranges, concepts, and template
diagnostics. Quadratic objectives are currently supported on HiGHS only, and
several features useful to re-solve-heavy research code — explicit LP basis
warm-starts, user-cut callbacks, heuristic-solution injection, and access to
the underlying native solver handle — are on the roadmap rather than in the
current release. For everyday one-shot modeling in Python or Julia, or for
constraint programming and scheduling, the mature ecosystems around gurobipy,
JuMP, Pyomo, and OR-Tools CP-SAT remain the better choice.

# Acknowledgements

This work is grounded in the author's PhD thesis and postdoctoral positions,
funded by Région Sud and Natural Solutions (PhD grant), the ERC
project SCALED (grant n°949812), the PEPR VDBI project RESILIENCE, and the
OASIS project of Aix-Marseille University's ITEM institute (postdoctoral
positions).

# References