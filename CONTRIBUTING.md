# Contributing to MIP++

Thanks for your interest in contributing to MIP++! This document explains how to
build the project, run the tests, follow the coding style, and submit changes.
Contributions of all kinds are welcome: bug reports, documentation, new solver
backends, additional test suites, and feature work.

## Getting help & reporting issues

- **Questions and support:** open a [GitHub Discussion](https://github.com/fhamonic/mippp/discussions)
  or an issue with the `question` label.
- **Bugs:** open a [GitHub issue](https://github.com/fhamonic/mippp/issues). Please include:
  - the solver backend and its version (e.g. `highs 1.10`),
  - your compiler and version (MIP++ currently targets **GCC 15 / C++26**),
  - a minimal reproducing snippet and the actual vs. expected behavior.
- **Feature requests:** open an issue describing the use case. Items already on the
  radar are listed in the *Roadmap* section of the [README](README.md).

## Development setup

MIP++ is a header-only C++ library. Building is only required to run the test
suite. You will need:

- **GCC 15** (`g++-15`) — the codebase relies on C++26 features currently only
  available in GCC 15.
- **CMake ≥ 3.12**
- **Conan 2.0** for dependency management
- Open-source solvers for local testing (at minimum HiGHS, Clp/Cbc, GLPK, or
  SCIP). The CI installs `coinor-clp coinor-libclp-dev libglpk-dev`.

The build depends on [dylib](https://github.com/martin-olivier/dylib),
[GoogleTest](https://github.com/google/googletest), and the
[MELON](https://github.com/fhamonic/melon) library (used by the graph-based
tests). MELON is not on Conan Center yet, so build it locally first:

```bash
git clone https://github.com/fhamonic/melon.git
cd melon && conan create . -u -b=missing -pr=<your_conan_profile> -c tools.build:skip_test=true
```

Ready-to-use Conan profiles are provided:

- [.github/workflows/gcc15_c++26](.github/workflows/gcc15_c++26) — Linux, GCC 15,
  matching the CI.
- [.github/workflows/mingw15_c++26](.github/workflows/mingw15_c++26) — Windows,
  MinGW (GCC 15). There is no Windows CI yet (installing the solver libraries on
  Windows is cumbersome), so please build and test locally with this profile
  before submitting Windows-related changes.

## Making solver libraries discoverable at runtime

MIP++ loads each solver's C API **at runtime** through `dylib`, so the solver's
shared library must be on your dynamic loader path (`LD_LIBRARY_PATH` on Linux)
when you run the tests — it is not needed at compile time. Commercial and
source-built solvers usually live outside the system library directories, so
export their locations from your shell profile (`~/.bashrc`). Adjust the base
paths to wherever you installed each solver:

```bash
# Replace /path/to/solvers with your own installation directory.

# Gurobi
export GUROBI_HOME="/path/to/solvers/gurobi1201/linux64"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$GUROBI_HOME/lib"

# COIN-OR (Clp / Cbc, e.g. built with coinbrew)
export COIN_HOME="/path/to/solvers/coinbrew/dist"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$COIN_HOME/lib"

# HiGHS
export HIGHS_HOME="/path/to/solvers/HiGHS"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HIGHS_HOME/lib"

# MOSEK
export MOSEK_HOME="/path/to/solvers/mosek/11.0/tools/platform/linux64x86"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$MOSEK_HOME/bin"

# CPLEX
export CPLEX_HOME="/path/to/solvers/cplex-community/cplex"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$CPLEX_HOME/bin/x86-64_linux"

# COPT
export COPT_HOME="/path/to/solvers/copt72"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$COPT_HOME/lib"

# FICO Xpress
export XPRESS_HOME="/path/to/solvers/xpressmp"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$XPRESS_HOME/lib"
export XPAUTH_PATH="$XPRESS_HOME/bin"   # Xpress license file location
```

Only export the solvers you actually have installed. Backends whose library
cannot be loaded at runtime are skipped automatically, so you can develop and
test against a single solver.

### Pointing at a specific library file

Relying on `LD_LIBRARY_PATH` loads whichever matching library the loader finds
first, using the conventional decorated name (e.g. `libhighs.so`). When you have
several versions installed, or the file carries a version-suffixed soname such
as `libhighs.so.1.10.0`, set the per-solver `MIPPP_<SOLVER>_LIBRARY` environment
variable to the **full path** of the exact library to load:

```bash
export MIPPP_HIGHS_LIBRARY="/path/to/solvers/HiGHS/build/lib/libhighs.so.1.10.0"
```

The recognized keys are `GUROBI`, `CPLEX`, `XPRESS`, `MOSEK`, `COPT`, `SCIP`,
`HIGHS`, `SOPLEX`, `CLP`, `CBC`, and `GLPK`. This variable points at the **main**
library only; its sibling dependencies are still resolved through the normal
loader search, so their folder must remain on `LD_LIBRARY_PATH`.

## Building and running tests

The [Makefile](Makefile) wraps the Conan/CMake workflow:

```bash
# Build and run the full test suite
make CONAN_PROFILE=<your_conan_profile>

# Run the tests for a single solver backend (case-insensitive)
make test highs CONAN_PROFILE=<your_conan_profile>

# Create the Conan package without running tests
make package CONAN_PROFILE=<your_conan_profile>

# Remove the build directory and generated presets
make clean
```

`make test <solver>` sets the `TEST_FILTER` variable, which restricts
compilation and execution to `test/solvers/<solver>.cpp` — handy when you only
have one solver installed locally. Without a filter, every backend in
[test/CMakeLists.txt](test/CMakeLists.txt) is built; backends whose runtime
library is missing are skipped automatically.

Before opening a pull request, make sure the suite passes for at least one
open-source backend. The GitHub Actions workflow
([.github/workflows/c-cpp.yml](.github/workflows/c-cpp.yml)) runs the tests on
Ubuntu with GCC 15 and the open-source solvers.

## How the tests are organized

Test logic is written once as reusable, solver-agnostic suites in
[test/test_suites/](test/test_suites/) (e.g. `lp_model.hpp`, `milp_model.hpp`,
`travelling_salesman.hpp`). Each backend then instantiates the relevant suites in
its own `test/solvers/<solver>.cpp` file, for example:

```cpp
#include "mippp/solvers/highs/all.hpp"
using namespace fhamonic::mippp;
#include "test_suites/all.hpp"

struct highs_lp_test : public model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api(); }
};
INSTANTIATE_TEST(HiGHS_lp, LpModelTest, highs_lp_test);
// ...
```

When you add a capability, add its test to the shared suite so that **every**
backend supporting it gets coverage, rather than duplicating logic per solver.

## Coding style

- Format all C++ with **clang-format** using the repository's
  [.clang-format](.clang-format) (Google base style, 4-space indent, no tabs).
  Run `clang-format -i` on changed files before committing.
- Match the surrounding code: naming, header layout, and idioms already in the
  file take precedence over personal preference.
- Keep the library **header-only**. Solver libraries are loaded at runtime via
  `dylib`; do not add link-time dependencies on solver SDKs.
- Target C++26 as used elsewhere in the codebase.

## Adding a new solver backend

Solver backends live under
[include/mippp/solvers/](include/mippp/solvers/)`<name>/<version>/`. Follow the
layout of an existing backend such as
[glpk](include/mippp/solvers/glpk/v5/) or
[highs](include/mippp/solvers/highs/v1_10/):

- `<name>_api.hpp` — thin binding that loads the solver's C API through `dylib`.
- `<name>_base.hpp` — shared model machinery.
- `<name>_lp.hpp`, `<name>_milp.hpp`, and (where supported) `<name>_qp.hpp` —
  the model classes exposing the MIP++ interface.
- an `all.hpp` aggregating the headers for convenience.

Then add a `test/solvers/<name>.cpp` file that instantiates the shared test
suites (see above) and register it in [test/CMakeLists.txt](test/CMakeLists.txt).
Update the feature tables in [misc/features_tables/](misc/features_tables/) and
the solver list in the README.

## Submitting changes

1. Fork the repository and create a topic branch off `main`.
2. Make your change, keeping commits focused and messages descriptive.
3. Ensure `clang-format` is applied and the tests pass for at least one backend.
4. Open a pull request against `main`, describing what changed and why. Link any
   related issue.
5. The CI must pass before a review can be merged.

## License

MIP++ is distributed under the **Boost Software License 1.0** (see
[LICENSE](LICENSE)). By contributing, you agree that your contributions will be
licensed under the same terms.
