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
date: 8 October 2026
bibliography: paper.bib
---

# Summary

Many questions in science, engineering and planning come down to choosing the
best option under constraints: which habitat patches to protect, how to route
vehicles, or how to schedule staff. Mathematical programming turns such a
question into a model, a set of decision variables, linear constraints and an
objective, where some variables may be restricted to whole numbers. A program
called a solver then finds an optimal solution, and several commercial and
open-source solvers compete on that task.

MIP++ is a C++23 library that lets researchers write these models in a notation
close to the mathematics, as JuMP [@jump2023] or Pyomo [@pyomo2011] do in Julia
and Python, and run them unchanged on any of eleven solvers: Gurobi [@gurobi],
CPLEX [@cplex], Xpress [@xpress], COPT [@copt], MOSEK [@mosek], HiGHS
[@highs2018], SCIP [@scip8], Clp [@clp], SoPlex [@soplex], GLPK [@glpk] and,
experimentally, Cbc [@cbc]. Variables, sums over index ranges and whole
families of constraints are written as ordinary C++ expressions, and every such
expression is translated, when the program is compiled, into direct calls to
the chosen solver's own programming interface, so the convenience costs almost
nothing at run time. The solver is picked at compile time and its shared
library is located and loaded when the program runs: nothing solver-specific is
needed on the machine that builds the program, and one compiled binary runs on
whatever solver the target machine has installed. The library is a set of
header files with no dependency of its own; GoogleTest and MELON [@melon] are
needed only to build the test suite.

Beyond writing models, MIP++ provides what advanced algorithms need:
constraints added while the solver searches (branch-and-cut callbacks with
lazy constraints), columns generated on demand with a column-pool manager,
dual values and reduced costs, starting solutions, indicator constraints, and
in-place modification of a model between solves. Quadratic objectives are
available on the solvers that support them, currently HiGHS alone.

# Statement of need

Researchers in operations research and combinatorial optimization face an
uncomfortable trade-off. High-level modeling languages such as JuMP
[@jump2023], Pyomo [@pyomo2011], PuLP [@pulp2011], and Python-MIP
[@pythonmip2020] make models easy to write and solver-independent, but the
time they spend constructing a model becomes a significant share of the
running time in workflows where models are built, modified, and re-solved
constantly rather than solved once, such as column generation, Benders
decomposition, cutting-plane methods, iterated reoptimization, or large-scale
computational experiments. Conversely, coding directly against a solver's C
API is maximally fast but verbose, error-prone, and locked to a single vendor,
which undermines both reproducibility and fair computational comparisons
across solvers.

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
same concern about model-construction overhead has recently driven work in
Python, notably PyOptInterface [@pyoptinterface2024].

MIP++ removes the trade-off by using C++23 ranges, concepts, and lazy views to
keep the modeling layer thin. On a model-construction benchmark (N-Queens,
$N^2$ binary variables and $6N-6$ constraints, for $N$ from 100 to 1000; only
construction is timed, never the solve), MIP++ builds models within 2–14 % of
hand-written C against the Gurobi C API, 1.2–1.5$\times$ faster than OR-Tools'
`MPSolver` on HiGHS, 1.6–2.4$\times$ faster on Xpress and 2.2–2.9$\times$
faster on Cbc (2.7–6.2$\times$ faster than OR-Tools' MathOpt on HiGHS), and
4.2–7.7$\times$ faster than JuMP in its default cached mode after warm-up
(13–21$\times$ in its direct mode); the Python layers are one to two orders of
magnitude slower, though those scripts time a single build without warm-up and
should be read as orders of magnitude. Both OR-Tools APIs are measured in
their fastest row-filling form, but the comparison is still not like-for-like:
`MPSolver` fills its own backend-independent structures and defers the native
model build to `Solve()`, which the MIP++ timings include — on SCIP this makes
the OR-Tools fill phase measure 0.3–0.5$\times$ of a full MIP++ build. The
model is also variable-heavy and constraint-light, and the Cbc figures rely on
its development branch, the only one that caches row insertions. Full tables,
per-backend timings for nine solvers, hardware and library versions, and
reproduction instructions are in a companion repository [@mippp_nqueens].

Solver independence, in turn, makes computational studies portable:
benchmarking Gurobi against HiGHS or SCIP is a two-line change. The
per-backend feature matrices (duals, reduced costs, callbacks, MIP starts,
column generation) are verified by a shared, backend-instantiated test suite;
continuous integration runs it on the four open-source backends installable
there (Clp, Cbc, GLPK, HiGHS) with GCC 14, GCC 15, Clang 18 and AppleClang 21
on Linux and macOS, on HiGHS alone with MinGW and MSVC on Windows, and the
same suites are run manually against the commercial backends. Because
backends are loaded rather than linked, a generated compatibility matrix
additionally records how far back each wrapper drives the solver's released
libraries: 83 published libraries across ten of the eleven solvers (COPT's
Python wheels ship no loadable C library), each downloaded and run through
the backend's test suites rather than assumed compatible from its version
number. It documents real breakage — SoPlex 6.0 lacks six of the C entry
points the wrapper needs, and HiGHS 1.7.0 and earlier fail the
iteration-limit tests of the quadratic suite — that version numbers alone
would not reveal.

MIP++ grew out of earlier work on optimizing the ecological connectivity of
landscapes [@hamonic2023], where a flow-based MILP formulation is coupled with
graph algorithms — from the companion MELON library [@melon] — that contract
the instance graphs during model construction, and where columns and cuts come
from shortest-path and flow computations. The modeling layer sits inside the
algorithmic loop, so per-call overhead is paid thousands of times: Python
layers made this prohibitive, and raw solver C APIs made it non-portable.

For everyday one-shot modeling in Python or Julia, or for constraint
programming and scheduling, the mature ecosystems around gurobipy, JuMP,
Pyomo, and OR-Tools CP-SAT remain the better choice. MIP++ requires GCC 14 or Clang 18 in
C++23 mode (GCC 15 in C++26 mode remains the primary target) and assumes
comfort with modern C++ — ranges, concepts, and template diagnostics.
Quadratic objectives are currently supported on the HiGHS backend only, and
several features useful to re-solve-heavy research code — explicit LP basis
warm-starts, SOS constraints, user-cut callbacks, and heuristic-solution
injection — are on the roadmap rather than in the current release; the native
solver handle stays reachable through `native_model()` for solver-specific
parameters. The library instead targets a deliberate niche: optimization
embedded in a larger C++ system that must run against whatever solver is
installed, cross-solver computational studies, and build-bound iterative
methods that rebuild or modify a model thousands of times.

# Software design

Three decisions shape the library. The first is that a MIP++ model *is* the
solver's model. The expression layer is functional and allocation-free:
objectives and constraint families are composed from C++ ranges as lazy views,
and `xsum` expresses sums over index sets. When a constraint is added, its term
range is iterated directly into pre-allocated scratch buffers passed to the
solver's C entry points (`Highs_addRow`, `GRBaddconstr`, …); no intermediate
model representation is built, extracted, or garbage-collected. The row
constraints of an N-Queens model, for instance, read:

```cpp
highs_milp model;  // loads HiGHS at runtime; or gurobi_milp, cplex_milp, …
auto rows = std::views::iota(0, n), cols = std::views::iota(0, n);
auto X = model.add_binary_variables(std::views::cartesian_product(rows, cols));
model.add_constraints(rows, [&](int row) {
    return xsum(cols, [&, row](int col) { return X(row, col); }) == 1;
}); // exactly on queen per row
```

The price of this choice is that nothing stands between the model and the
solver: there is no stage at which the library could reformulate a constraint
the solver does not accept natively, as JuMP's bridges do, and the solver of
an existing model cannot be changed. The gain is that re-solves after in-place
modifications — added rows or columns, changed bounds or coefficients, removed
variables — pay only the solver's incremental update cost, which is what
build-bound iterative methods need.

The second decision is to select the backend at compile time and load its
shared library at run time. The model type (`highs_milp`, `gurobi_milp`, …)
fixes the solver, so every call resolves statically; the library itself is
located through an explicit path, a per-solver environment variable or the
platform's search path when the program first creates a model. Nothing
solver-specific reaches the build system, at the cost of discovering a missing
or incompatible library only when the program runs — which is why the
compatibility matrix described above exists.

The third is that solve statuses are not flattened into a
lowest-common-denominator enum: each backend returns a `std::variant` whose
alternatives are exactly the outcomes that solver reports, arranged in a type
hierarchy so that generic queries (`is_a<status::infeasible_or_unbounded>`)
work everywhere while exact ones (`is<status::primal_and_dual_infeasible>`)
compile only on backends that can report them — a distinction resolved
entirely at compile time.

# Acknowledgements

Michael Heyman contributed the runtime discovery of the Gurobi 13 and CPLEX
22.2 libraries and the initial implementation of the infeasibility-diagnosis
(IIS) support.

MIP++ is grounded in the PhD thesis and postdoctoral positions of François Hamonic, funded by Région Sud - Provence-Alpes-Côte d'Azur, Natural Solutions, the European Research Council grant [SCALED](https://www.scaled-erc.eu/) to Cécile ALBERT (ERC-STG no. 949812), the ANR project [RESILIENCE](https://www.pepr-resilience.eu/index.php) (no. ANR-24-PEVD-0002) and the project OASIS of [ITEM](https://institut-item.univ-amu.fr), an A\*Midex Initiative d'Excellence institute funded under France 2030 (AMX-19-IET-012).

# References
