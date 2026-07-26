# Contributing to MIP++

Thanks for your interest in contributing to MIP++! This document explains how to
build the project, run the tests, follow the coding style, and submit changes.
Contributions of all kinds are welcome: bug reports, documentation, new solver
backends, additional test suites, and feature work.

Participation in this project is governed by its
[Code of Conduct](CODE_OF_CONDUCT.md).

## Getting help & reporting issues

- **Questions and support:** open a [GitHub Discussion](https://github.com/fhamonic/mippp/discussions)
  or an issue with the `question` label.
- **Bugs:** open a [GitHub issue](https://github.com/fhamonic/mippp/issues). Please include:
  - the solver backend and its version (e.g. `highs 1.10`) — please check it against
    the [compatibility table](docs/solvers/compatibility.md) first, a version known
    to fail there is not a new bug,
  - your compiler and version (MIP++ requires **GCC 14** or **Clang 18** at the very
    least; **GCC 15 / C++26** is the primary target),
  - a minimal reproducing snippet and the actual vs. expected behavior.
- **Feature requests:** open an issue describing the use case. Items already on the
  radar are listed in the *Roadmap* section of the [README](README.md).

## Development setup

MIP++ is a header-only C++ library. Building is only required to run the test
suite. You will need:

- **GCC 15**, **GCC 14** or **Clang 18** — the codebase relies on C++23 with a few
  C++26 features for which fallbacks are provided, so it also builds under C++23
  with GCC 14 or Clang 18. GCC 15 / C++26 remains the primary target if you have it.
- **CMake ≥ 3.12**
- **Conan 2.0** for dependency management
- Open-source solvers for local testing (at minimum HiGHS, Clp/Cbc, GLPK, or
  SCIP). The CI installs `coinor-clp coinor-libclp-dev coinor-cbc
  coinor-libcbc-dev highs libhighs1 libglpk-dev`.

The build depends on [dylib](https://github.com/martin-olivier/dylib),
[GoogleTest](https://github.com/google/googletest), and the
[MELON](https://github.com/fhamonic/melon) library (used by the graph-based
tests). MELON is not on Conan Center yet, so build it locally first:

```bash
git clone https://github.com/fhamonic/melon.git
cd melon && conan create . -u -b=missing -pr=<your_conan_profile> -c tools.build:skip_test=true
```

Ready-to-use Conan profiles are provided, one per CI job, so a local run
reproduces exactly what the workflow does:

- [.github/workflows/gcc15_c++26](.github/workflows/gcc15_c++26) — Linux, GCC 15,
  C++26.
- [.github/workflows/gcc14_c++23](.github/workflows/gcc14_c++23) — Linux, GCC 14,
  C++23. This is the [Makefile](Makefile)'s default profile.
- [.github/workflows/clang18_c++23](.github/workflows/clang18_c++23) — Linux,
  Clang 18, C++23 (with libstdc++).
- [.github/workflows/mingw15_c++26](.github/workflows/mingw15_c++26) — Windows,
  MinGW (GCC 15). The Windows job only exercises HiGHS, whose library it downloads
  from the upstream release; installing the other solvers on Windows is cumbersome,
  so test those locally with this profile before submitting Windows-related changes.

## Making solver libraries discoverable at runtime

MIP++ loads each solver's C API **at runtime** through `dylib`, so the solver's
shared library must be reachable when you run the tests — it is not needed at
compile time. [include/mippp/detail/solver_library.hpp](include/mippp/detail/solver_library.hpp)
implements that lookup once for every backend, with the following precedence
(first match wins):

1. **The constructor argument.** Every `<solver>_api` takes an optional path:
   `highs_api api("/path/to/libhighs.so.1.10.0")`. Used verbatim.
2. **`MIPPP_<KEY>_LIBRARY`.** An environment variable holding the **full path** of
   one library file — also used verbatim, and it wins over anything on
   `LD_LIBRARY_PATH`.
3. **A search by name**, over the directories the dynamic loader would itself
   search. Since dylib 3.0 opens paths only and no longer looks libraries up by
   name, MIP++ reproduces that list itself: `LD_LIBRARY_PATH`, then `/etc/ld.so.conf` and
   `/etc/ld.so.conf.d/*.conf`, then `/usr/local/lib`, `/usr/lib`, `/lib` (on
   Windows: `PATH` then `System32`; on macOS the `DYLD_*` variables then the
   Homebrew/MacPorts prefixes).

In each directory the search accepts the decorated name (`libhighs.so`) or, when
the unversioned symlink is absent — usual in runtime-only packages — a versioned
variant (`libhighs.so.1.10.0`, `libhighs.1.10.0.dylib`), taking the
lexicographically greatest, which approximates the highest version. Where a
backend declares probe symbols, a candidate is kept only if it exports them, which
is how a same-named library without the C API gets rejected rather than
half-loaded (Ubuntu's `libCbc.so` versus the `libCbcSolver.so` MIP++ needs) — and
backends are tried under several names when a solver has been renamed across
releases.

Once loaded, the backend compares the library's reported version against the one
its wrapper was written against and warns on `stderr` when they differ — mostly
harmless, since these C APIs are stable, but it is the first thing to look at when
a solver misbehaves. Set `MIPPP_NO_VERSION_WARNING` to silence it.

### Which of the two you should use

They are complementary, and the split follows how the solver ships:

- **`MIPPP_<KEY>_LIBRARY` for self-contained solvers** — HiGHS, GLPK, Gurobi,
  CPLEX, Mosek, COPT, Xpress. One file carries the whole API, so pinning it is
  exact: it selects a precise version among several installs and needs no
  `LD_LIBRARY_PATH` at all. This is what the compatibility matrix uses to swap
  one released library for another under a single test binary.
- **`LD_LIBRARY_PATH` for solvers split across several shared objects** — Cbc
  (`libCbcSolver` needs `libCgl`, `libClp`, `libCoinUtils`), SCIP (`libscip` needs
  `libipopt`, the MUMPS/SCOTCH/METIS stack), and COIN-OR builds in general. The
  variable pins the **main** library only; its siblings are still resolved by the
  ordinary loader search, so their directory has to be on `LD_LIBRARY_PATH`
  regardless. For those, exporting the directory is the reliable route and pinning
  the main file buys little — unless you also need to select among several
  installed versions, in which case set both.

Commercial and source-built solvers usually live outside the system library
directories, so export their locations from your shell profile (`~/.bashrc`).
Adjust the base paths to wherever you installed each solver:

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

To pin an exact file, give `MIPPP_<KEY>_LIBRARY` its full path:

```bash
export MIPPP_HIGHS_LIBRARY="/path/to/solvers/HiGHS/build/lib/libhighs.so.1.10.0"
```

The recognized keys are `GUROBI`, `CPLEX`, `XPRESS`, `MOSEK`, `COPT`, `SCIP`,
`HIGHS`, `SOPLEX`, `CLP`, `CBC`, and `GLPK`. Unlike the name search, this path is
never second-guessed: if the file is missing or does not export the expected
symbols, the backend throws with the loader's own message instead of quietly
falling back to another install.

## Building and running tests

The [Makefile](Makefile) wraps the Conan/CMake workflow. It defaults to the
`gcc14_c++23` profile; pass `CONAN_PROFILE=<profile>` to pick another one from the
list above:

```bash
# Build and run the full test suite
make

# Run the tests for a single solver backend (case-insensitive)
make test highs

# Narrow further to the tests whose name matches a regex (ctest -R)
make test highs LpModelTest

# Same, with another profile
make test highs CONAN_PROFILE=gcc15_c++26

# Create the Conan package without running tests
make package

# Remove the build directory, generated presets and the compat-matrix cache
make clean
```

`make test <solver>` sets the `TEST_SOURCE` variable, which restricts
compilation and execution to `test/solvers/<solver>.cpp` — handy when you only
have one solver installed locally. An optional third word sets `TEST_FILTER`,
passed to `ctest -R`. Without a source, every backend in
[test/CMakeLists.txt](test/CMakeLists.txt) is built; backends whose runtime
library is missing are skipped automatically.

That automatic skipping is convenient locally but hides packaging mistakes in
CI, where the solvers are installed on purpose. Set `MIPPP_REQUIRED_SOLVERS` to
a `;`-separated list of solver keys — the same keys as the
`MIPPP_<key>_LIBRARY` variables above — to turn "this backend could not be
loaded" from a skip into a test failure:

```bash
MIPPP_REQUIRED_SOLVERS="CLP;CBC;GLPK;HIGHS" make
```

Only the loading of the backend is asserted; tests skipped because a solver
lacks a capability, or because its license is unavailable, are unaffected.

Note that `TEST_SOURCE` is sticky in the CMake cache: after `make test highs`, the
build directory keeps producing a HiGHS-only binary until you run `make test` (or
`make clean`) again — the compatibility matrix below needs an all-backends one.

Before opening a pull request, make sure the suite passes for at least one
open-source backend. The rest is covered by
[.github/workflows/c-cpp.yml](.github/workflows/c-cpp.yml), which on every push and
pull request to `main` builds and runs the suite under GCC 15 / C++26, GCC 14 /
C++23, Clang 18 / C++23 and MinGW 15 on Windows — each job declaring its installed
backends through `MIPPP_REQUIRED_SOLVERS` — plus a job that installs the library
with plain CMake and builds an out-of-tree `find_package(mippp)` consumer against
it. Changes to the CMake install/export rules should be checked against that last
job.

## The version compatibility matrix

MIP++ wraps one version of each solver API but loads the library at runtime, so
a given wrapper usually drives a range of releases.
[tools/compat_matrix.py](tools/compat_matrix.py) measures that range: it
downloads published libraries, points each one at the test binary through
`MIPPP_<key>_LIBRARY`, and renders
[docs/solvers/compatibility.md](docs/solvers/compatibility.md).

```bash
make test                    # once: an all-backends binary
make compat_table            # download and test the 5 newest of each
make compat_table LIMIT=8    # ... or the 8 newest
```

The `compat_table` target passes `--commercial`, so rows for solvers you have no
license for will simply report that the library could not be obtained. Call
[tools/compat_matrix.py](tools/compat_matrix.py) directly for finer control:
`list` shows the published versions a source exposes, `run` downloads and tests
them, `render` rebuilds the table from the cached results in `.compat-cache/`, and
`--solvers` restricts the run to a comma-separated subset.

Sources are declared in
[tools/compat_manifest.json](tools/compat_manifest.json) — conda-forge
`linux-64` packages and manylinux wheels, both of which enumerate every
published version over a JSON API without an account. Adding a solver or
changing where its libraries come from is an edit to that file.

Two things to know when extending it. A wheel is only usable when it carries a
real library in an auditwheel `.libs/` directory: `highspy` links HiGHS
statically into a pybind11 extension that exports the whole C API but cannot be
`dlopen`ed on its own, since it needs libpython. And conda-forge splits runtime
dependencies across packages — `libCbcSolver` arrives without `libCgl`,
`libscip` without `libipopt` — so each source has a `depends` list of extra
packages to put on `LD_LIBRARY_PATH`. When one is missing the tool reports the
unresolved soname rather than blaming the solver version, so the fix is to name
the providing package (note that conda-forge splits tools from libraries:
`scotch` ships binaries, `libscotch` ships the shared objects).

The run is Linux/x86-64 only. It is deliberately kept out of the PR workflow — it
is slow and depends on the network — and lives in its own workflow instead
([.github/workflows/compat.yml](.github/workflows/compat.yml)), which runs monthly
and on manual dispatch, and opens a pull request when the regenerated table
differs. So you do not need to run it yourself for an ordinary change: do it when
you bump a wrapper to a new solver API version, or when you edit the manifest, and
include the regenerated table in your pull request.

## How the tests are organized

Test logic is written once as reusable, solver-agnostic suites in
[test/test_suites/](test/test_suites/) (e.g. `lp_model.hpp`, `milp_model.hpp`,
`travelling_salesman.hpp`). Each backend then instantiates the relevant suites in
its own `test/solvers/<solver>.cpp` file, for example:

```cpp
#include "mippp/solvers/highs/all.hpp"
using namespace mippp;
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
- Target C++23 as used elsewhere in the codebase. GCC 14 and Clang 18 are built in
  CI, so a C++26 feature may only be used behind a fallback that keeps those
  compilers working (as [include/mippp/detail/concat_view.hpp](include/mippp/detail/concat_view.hpp)
  does for `views::concat`).

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
Update the feature tables in [docs/assets/features_tables/](docs/assets/features_tables/) and
the solver list in the README. If published builds of the solver are downloadable
without an account, declare a source for it in
[tools/compat_manifest.json](tools/compat_manifest.json) so the new backend gets a
row in the compatibility table.

## Submitting changes

1. Fork the repository and create a topic branch off `main`.
2. Make your change, keeping commits focused and messages descriptive.
3. Ensure `clang-format` is applied and the tests pass for at least one backend.
4. Open a pull request against `main`, describing what changed and why. Link any
   related issue.
5. The CI must pass before a review can be merged.

## License

MIP++ is distributed under the **Boost Software License 1.0** (see
[LICENSE.md](LICENSE.md)). By contributing, you agree that your contributions will be
licensed under the same terms.
