---
title: 'MIP++: solver-agnostic mathematical programming in C++23 at raw C API speed'
tags:
  - C++
  - operations research
  - mathematical programming
  - linear programming
  - mixed-integer programming
  - algebraic modeling
authors:
  - name: François Hamonic
    orcid: 0000-0000-0000-0000 # TODO: replace with your ORCID
    affiliation: 1
affiliations:
  - name: Aix-Marseille University, France # TODO: adjust to your current affiliation/lab (e.g. LIS, ITEM institute)
    index: 1
date: 19 July 2026
bibliography: paper.bib
---

# Summary

MIP++ is a header-only C++23 library for modeling and solving linear programs
(LP), mixed-integer linear programs (MILP), and quadratic programs (QP). It
provides an algebraic modeling syntax comparable in readability to JuMP
[@jump2023] or Pyomo [@pyomo2011] — variables, expressions built with
overloaded operators, sums over index ranges, and constraint families — while
compiling down to direct calls into each solver's native C API. A single model
written against the MIP++ interface can target any of eleven solver backends
(Gurobi [@gurobi], CPLEX, Xpress, COPT, MOSEK, HiGHS [@highs2018], SCIP
[@scip8], Cbc, Clp, SoPlex, and GLPK); the backend is selected at compile time
and its shared library is discovered and loaded at runtime, so no solver SDK
needs to be present at link time.

The expression system is functional and allocation-free: objectives and
constraint families are composed from C++ ranges as lazy views, and the
`xsum` combinator expresses sums over index sets in a form close to
mathematical notation. Beyond model construction, MIP++ exposes the
algorithmic facilities needed for decomposition and cutting-plane methods:
branch-and-cut callbacks with lazy constraints, column generation, dual
values, reduced costs, MIP starts, and SOS and indicator constraints. On a
model-construction benchmark (N-Queens, $N^2$ binary variables), MIP++ builds
models within 10–20% of hand-written C against the Gurobi C API, whereas
Python-based layers are two to three orders of magnitude slower and JuMP,
after warm-up, remains an order of magnitude slower; the benchmark is
reproducible from a companion repository [@mippp_nqueens].

# Statement of need

Researchers in operations research and combinatorial optimization face an
uncomfortable trade-off. High-level modeling languages such as JuMP
[@jump2023], Pyomo [@pyomo2011], PuLP [@pulp2011], and Python-MIP
[@pythonmip2020] make models easy to write and solver-independent, but their
model-construction overhead becomes a real cost in workflows that build many
models — column generation, decomposition schemes, iterated reoptimization,
or large-scale experiments. Conversely, coding directly against a solver's C
API achieves maximal performance but produces verbose, error-prone code that
is locked to a single vendor, which undermines both reproducibility and fair
computational comparisons across solvers.

Existing C++ alternatives only partially resolve this tension. Google
OR-Tools [@ortools] is the closest competitor: it offers solver-agnostic
linear and mixed-integer modeling in C++ over several of the same backends.
It is, however, a large compiled library that must be linked together with
the chosen solvers, and models pass through an intermediate representation
before reaching the solver, whereas MIP++ is header-only, loads solver shared
libraries at runtime, and streams expressions directly into the buffers of
each solver's C API. OR-Tools also does not expose the full range of
algorithmic hooks MIP++ targets, such as column generation with reduced-cost
access across backends. Elsewhere in the C++ ecosystem, the COIN-OR Open
Solver Interface (OSI) offers solver abstraction at the matrix level without
algebraic modeling, and FlopC++ [@flopcpp2007] provides algebraic modeling in
C++ but predates modern C++ facilities and is no longer actively developed.
Solver-vendor C++ APIs (e.g., Gurobi's or CPLEX's) are expressive but
proprietary to one solver each.

MIP++ removes the trade-off by using C++23 ranges, concepts, and lazy views
to make the modeling layer essentially free: expression templates are
consumed directly into the buffers passed to solver C functions, with no
intermediate model representation. Solver independence, in turn, makes
computational studies portable — benchmarking Gurobi against HiGHS or SCIP is
a two-line change — and the runtime loading of solver shared libraries means
a published research code compiles on any machine and merely requires the
chosen solver to be installed at run time. The per-backend feature matrices
(duals, callbacks, MIP starts) are verified by a shared, backend-instantiated
test suite run in continuous integration.

MIP++ grew out of the author's doctoral work on optimizing the ecological
connectivity of landscapes [@hamonic2023]. There, a flow-based MILP
formulation is coupled with graph preprocessing algorithms that contract the
instance graphs before and during model construction, and columns and cuts
are generated from shortest-path and flow computations. This workflow
requires performant, fine-grained interleaving of graph algorithms — provided
by the companion MELON graph library — with model building: the modeling
layer is invoked inside the algorithmic loop, so any per-call overhead is
paid thousands of times. Python modeling layers made this prohibitive, and
raw solver C APIs made it non-portable; MIP++ was built to be both. It is
intended for operations-research practitioners and researchers who need
C-level performance, solver portability, and access to advanced solver
features from modern C++.

# Acknowledgements

This work is grounded in the PhD thesis and postdoctoral positions of
François Hamonic, funded by Région Sud and Natural Solutions (PhD grant), the
ERC project SCALED (grant n°949812), the PEPR VDBI project RESILIENCE, and
Aix-Marseille University's ITEM institute.

# References
