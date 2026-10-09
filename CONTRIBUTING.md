# Contributing to MIP++

Thanks for your interest in contributing to MIP++! This document explains how to
build the project, run the tests, follow the coding style, and submit changes.
Contributions of all kinds are welcome: bug reports, documentation, new solver
backends, additional test suites, and feature work.

Participation in this project is governed by its
[Code of Conduct](CODE_OF_CONDUCT.md).

## Getting help & reporting issues

- **Questions and support:** open a [GitHub issue](https://github.com/fhamonic/mippp/issues)
  with the `question` label.
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
- **Conan 2.0** to fetch the test dependencies (the library itself has none)
- Open-source solvers for local testing (at minimum HiGHS, Clp/Cbc, GLPK, or
  SCIP). The CI installs `coinor-clp coinor-libclp-dev coinor-cbc
  coinor-libcbc-dev highs libhighs1 libglpk-dev`.

The library itself has no dependency; the tests need
[GoogleTest](https://github.com/google/googletest) and the
[MELON](https://github.com/fhamonic/melon) library (used by the graph-based
tests). Both come from Conan Center, so `conan build` fetches them and nothing
has to be built by hand.

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

MIP++ loads each solver's C API **at runtime** through the platform loader
(`dlopen` / `LoadLibrary`, wrapped by
[include/mippp/detail/dynamic_library.hpp](include/mippp/detail/dynamic_library.hpp)),
so the solver's shared library must be reachable when you run the tests — it is
not needed at compile time. `dynamic_library` is the only place with
platform-specific loading code (`dlopen` on POSIX, `LoadLibraryExW` on Windows).
It never unmaps a library once loaded (`RTLD_NODELETE`, a pinned module on
Windows): solvers keep worker threads and thread-local state alive past the
destruction of their models, and unloading their code underneath those threads
crashed the HiGHS tests at process exit;
its behavior and the resolution rules below are pinned by
[test/dynamic_library.cpp](test/dynamic_library.cpp), which opens a small fixture
library built alongside the test binary. [include/mippp/detail/solver_library.hpp](include/mippp/detail/solver_library.hpp)
implements that lookup once for every backend, with the following precedence
(first match wins):

1. **The argument of `load`.** Every `<solver>_api::load` takes an optional path:
   `highs_api::load("/path/to/libhighs.so.1.10.0")`; a model's default
   constructor calls it with none. Used verbatim.
2. **`MIPPP_<KEY>_LIBRARY`.** An environment variable holding the **full path** of
   one library file — also used verbatim, and it wins over anything on
   `LD_LIBRARY_PATH`.
3. **A search by name**, over the directories the dynamic loader would itself
   search. `dynamic_library` opens exact paths only, so MIP++ reproduces that
   list itself: `LD_LIBRARY_PATH`, then `/etc/ld.so.conf` and
   `/etc/ld.so.conf.d/*.conf`, then `/usr/local/lib`, `/usr/lib`, `/lib` (on
   Windows: `PATH` then `System32`; on macOS the `DYLD_*` variables then the
   Homebrew/MacPorts prefixes).

The result of that search is memoized per solver key and name list for the life
of the process. Only successes are cached; a cached file that no longer loads
triggers a fresh search. The one consequence is that a change the process makes
to `LD_LIBRARY_PATH` after its first search is not seen. Steps 1 and 2 bypass the
cache, which is what lets two versions of one solver be loaded side by side.

`load` then interns the loaded file (`detail::solver_api`, the base of every
`<solver>_api`): one instance per loaded file, keyed by the loader's handle,
returned to every later call resolving to that file and never destroyed. An api
object is thus one library file, `api.library_path()` says which, a model holds
a pointer to the api it was built from that cannot dangle, and whatever the api
constructor does once per library (version check, licence initialisation) runs
once per process.

The names searched for are the backend's `library_names`, newest first: one per
release where the solver ships its library so (`libgurobi130.so`, `libgurobi120.so`,
…), two where it was renamed (Cbc's `libCbcSolver` and `libCbc`), one otherwise.
The first directory holding any of them wins, as it would for the loader, and the
list order only ranks candidates inside one directory — so `LD_LIBRARY_PATH`
order chooses between two installed Gurobi releases, and the newest is taken when
both sit in one directory. In each directory the search accepts the decorated name
(`libhighs.so`) or, when the unversioned symlink is absent — usual in runtime-only
packages — a versioned variant (`libhighs.so.1.10.0`, `libhighs.1.10.0.dylib`),
taking the lexicographically greatest, which approximates the highest version.
Where a backend declares `probe_symbols`, a candidate is kept only if it exports
them, which is how a same-named library without the C API gets rejected rather
than half-loaded (Ubuntu's `libCbc.so` versus the `libCbcSolver.so` MIP++ needs).

Once loaded, the backend asks the library its release — `api.library_version()`
returns it, empty when the C API has no version call (SoPlex) or reports something
that is not a number (a Cbc or Clp `devel` build) — and classifies it against two
lists (see the [compatibility matrix](#the-version-compatibility-matrix)):
`validated_versions`, the release ranges its implementation has been driven
through the full test suite on, and the optional, wider `supported_versions`,
which adds the releases it supports partially, where models build and solve and
the members the release lacks throw `mippp::feature_unavailable_error`.
`Api::support_of(api.library_version())` returns the result, a
`mippp::release_support`: `validated`, `partial` or `untested`. A validated or a
partial release loads silently. Any other one, and a release that is not a
number, loads with a warning on `stderr`, which names both lists where the
backend declares two; SoPlex, which reports nothing, never warns. The warning is
mostly harmless, since these C APIs are stable, but it is the first thing to look
at when a solver misbehaves. Set `MIPPP_NO_VERSION_WARNING` to silence it; it
changes nothing else. No release is refused for its number: only a library the
api cannot be built from, one missing a required entry point or a probe symbol,
is rejected.

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
export GUROBI_HOME="/path/to/solvers/gurobi1303/linux64"
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

# Narrow further to the test suites whose name matches a regex (ctest -R)
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
have one solver installed locally. A `;`-separated list selects several
(`make test "clp;cbc"`). An optional third word sets `TEST_FILTER`, passed to
`ctest -R`. Each ctest test is a GoogleTest test suite run in a process of its
own, `HiGHS_lp/LpModelTest/0` say, so the filter matches suite names; a single
test runs directly, as in
`build/test/mippp_test --gtest_filter='HiGHS_lp/LpModelTest/0.*status*'`.
Without a source, every backend in [test/CMakeLists.txt](test/CMakeLists.txt)
is built; a backend whose runtime library is missing, or whose solver refuses
its license when the first model is created, is skipped automatically, one
whole suite at a time. Each solver file instantiates
every shared suite for every model type of its backend and takes about as long
to compile as all the core tests together, so a targeted build is several
times faster than a full one. Whatever the selection, every public header is
also compiled on its own in a generated translation unit, which catches a
header that only works when something else was included before it.

That automatic skipping is convenient locally but hides packaging mistakes in
CI, where the solvers are installed on purpose. Set `MIPPP_REQUIRED_SOLVERS` to
a `;`-separated list of solver keys — the same keys as the
`MIPPP_<key>_LIBRARY` variables above — to turn "this backend could not be
loaded" from a skip into a test failure:

```bash
MIPPP_REQUIRED_SOLVERS="CLP;CBC;GLPK;HIGHS" make
```

Only the loading of the backend is asserted; tests skipped because a solver
lacks a capability, because the loaded release lacks a member (see
[supporting a release partially](#supporting-a-release-partially)), or because
its license is unavailable, are unaffected. A partially supported release, such
as the HiGHS 1.9.0 the Linux jobs install from `apt`, therefore passes such a
run, with the tests of the members it lacks skipped.

Note that `TEST_SOURCE` is sticky in the CMake cache: after `make test highs`, the
build directory keeps producing a HiGHS-only binary until you remove it
(`rm -rf build`; `make clean` does too, but also deletes the matrix's
`.compat-cache`), since a `make test` without a source leaves the cached list
alone — the compatibility matrix below needs an all-backends one.

The tests can be built with sanitizers, in which case a Debug build is what
makes their reports precise:

```bash
TEST_SANITIZE=address,undefined conan build . -of=build_sanitize -b=missing -pr=gcc15_c++26 -s build_type=Debug
```

Two source-level checks run in CI and can be run locally: `make check-format`
verifies the tree against `.clang-format` (CI pins clang-format 18.1.8, another
major version flags different lines) and `make check-includes` runs
[misc/tools/check_std_includes.py](misc/tools/check_std_includes.py), which
fails if a header names a `std::` symbol that none of its own includes is
guaranteed to provide. The second one matters because every Linux job compiles
against libstdc++, whose `<ranges>` drags in most of `<utility>` and
`<type_traits>`, so a missing include compiles across the whole matrix and
breaks only in a consumer's translation unit.

Before opening a pull request, make sure the suite passes for at least one
open-source backend. The rest is covered by
[.github/workflows/c-cpp.yml](.github/workflows/c-cpp.yml), which runs on every
push and pull request to `main` and on every `v*` tag:

- `source-hygiene`: the two checks above, and the checks of the
  [code in the documentation](#code-in-the-documentation).
- `linux-gcc15-sanitize` and `macos-appleclang21-build` build and run the whole
  suite, every backend compiled, with all four open-source solvers installed and
  required — the first one under ASan and UBSan in Debug, the second one under
  Apple clang 21 / libc++. The sanitized job also defines
  `MIPPP_PORTABLE_RANGE_SHAPES`, which makes MIP++ use its own `concat` and
  `cartesian_product` views even where the standard library has them, so the
  fallbacks that C++23 and libc++ users get are sanitized too.
- `linux-gcc15-build`, `linux-gcc14-build`, `linux-clang18-build`,
  `windows-mingw15-build` and `windows-msvc1711-build` build the tests of one or
  two backends each (`TEST_SOURCE`) and require them (`MIPPP_REQUIRED_SOLVERS`),
  which keeps every compiler of the matrix under a few minutes; every backend
  is still compiled there through the per-header check.
- `linux-cmake-install`, `linux-cmake-submodule` and `linux-conan-create` build
  and run [examples/simple_lp](examples/simple_lp) against, respectively, a
  `cmake --install` prefix, an `add_subdirectory` of the source tree, and a
  package created by `conan create`; the program's output is diffed against
  [.github/simple_lp_expected_output.txt](.github/simple_lp_expected_output.txt),
  kept out of the example folder so that folder stays what the README says
  can be copied as-is.
  Changes to the CMake install/export rules or to `conanfile.py` should be
  checked against these.

## The version compatibility matrix

Each backend is one implementation, `include/mippp/solvers/<name>/impl/v1/`,
that adapts at runtime to a range of solver releases (probing for entry points
that appeared or disappeared along the way) and states that range in its api
class: `library_names`, the library names it opens, `validated_versions`, the
half-open release ranges it has been driven through the full suite on —
`{{10}, {14}}` reads "every 10.x.y up to 13.x.y" — and, optionally,
`supported_versions`, ranges of the same shape that also hold the releases it
supports partially (see
[supporting a release partially](#supporting-a-release-partially)). A backend
without `supported_versions` supports exactly its validated releases; only HiGHS
declares one today, `{{1, 7, 2}, {1, 16}}` around a validated `{{1, 14}, {1, 16}}`.
[misc/tools/compat_matrix.py](misc/tools/compat_matrix.py) is the evidence behind those
ranges: it downloads published libraries, points each one at the test binary
through `MIPPP_<key>_LIBRARY`, and renders
[docs/solvers/compatibility.md](docs/solvers/compatibility.md), whose rows read:

- ✅ full: no test failed, and none skipped for a member the release lacks. Such
  a row earns its release a place in `validated_versions`.
- ☑️ partial: no test failed, and some skipped with a reason opening with
  `release lacks: `, none of them in a core suite (`LpModelTest`,
  `MilpModelTest`, `QpModelTest`). Such a row earns its release a place in
  `supported_versions`, and only there.
- ❌ a test failed, or the row contradicts the claim, see below.
- ⚠️ nothing failed, but an unusual number of tests skipped, or nothing ran.

The table and the claim are tied by the
`<Solver>_api.loaded_release_is_a_supported_one` test each backend instantiates.
It records the backend's `validated_versions` and `supported_versions`, and the
`release_support` of the loaded release (`validated`, `partial` or `untested`),
as properties of those names in the GoogleTest report, the first two even on a
skipped row, and fails when the loaded release lies outside both lists. So a row
whose release passes but is missing from the claim fails on that test alone,
noted "unclaimed", and the claim cannot silently lag behind the table: add the
release to `validated_versions` when the row lacks nothing, and to
`supported_versions` alone when it does. Reading those properties beside the
skips, the matrix also renders ❌ for:

- a `release lacks: ` skip inside a core suite: a partially supported release
  still builds and solves models through the whole `lp_model`, `milp_model` and
  `qp_model` interface;
- a release in `supported_versions` alone whose row lacks nothing: move it into
  `validated_versions`;
- a release in `validated_versions` whose row lacks a member, which the shared
  suites already fail, see below;
- on a library that reports no version (SoPlex), a package version the recorded
  lists do not place where the row's result does: outside both lists on a clean
  row, or inside `validated_versions` on a row with lacking skips.

Skips for a lacking member are counted apart, so they never make a row ⚠️ nor
raise the number of skips a fully supported row is compared against. For the
solvers the matrix cannot obtain or license (COPT, MOSEK, Xpress) the ranges rest
on a maintainer's local full run instead, recorded with its date in the manifest
`note` shown above the solver's table.

```bash
make test                    # once: an all-backends binary (see TEST_SOURCE)
make compat_table            # download and test the 10 newest of each
make compat_table LIMIT=8    # ... or the 8 newest
```

The `compat_table` target passes `--commercial`. Those rows run under whatever
license your environment provides and read "not tested (licence)" without one,
with three exceptions: CPLEX runs under the community edition its wheels carry,
MOSEK only checks its license when it solves, so its rows still count the tests
that merely build models, and COPT's rows always read "no library in archive".
Commit commercial rows only from a run under the licenses the manifest notes
name. Call
[misc/tools/compat_matrix.py](misc/tools/compat_matrix.py) directly for finer control:
`list` shows the published versions a source exposes, `run` downloads and tests
them, `render` rebuilds the table from the cached results in `.compat-cache/`, and
`--solvers` restricts the run to a comma-separated subset.

Sources are declared in
[misc/tools/compat_manifest.json](misc/tools/compat_manifest.json) — conda-forge
`linux-64` packages and manylinux wheels, both of which enumerate every
published version over a JSON API without an account. Adding a solver or
changing where its libraries come from is an edit to that file.

Two things to know when extending it. A wheel is only usable when it carries a
real library in an auditwheel `.libs/` directory: `highspy` links HiGHS
statically into a pybind11 extension that exports the whole C API but cannot be
`dlopen`ed on its own, since it needs libpython. And conda-forge splits runtime
dependencies across packages — `libCbcSolver` arrives without `libCgl`,
`libscip` without `libipopt` — so each source has a `depends` list of packages
the tool lays out beside the library, as one conda environment would hold them.
Each comes at the exact build the row's own recipe was compiled against: the
published ranges, such as `coin-or-cgl >=0.60,<0.61`, are not enough, since Cgl
0.60.7 changed the layout of a class that Cbc inlines, and a Cbc built against
Cgl 0.60.6 aborts on 0.60.10. BLAS and the other system libraries come from the
host, and each row records in the results which ones it took. When a package is
missing the tool reports the unresolved soname rather than blaming the solver
version, so the fix is to name the providing package (note that conda-forge
splits tools from libraries: `scotch` ships binaries, `libscotch` ships the
shared objects).

The run is Linux/x86-64 only, and it is not part of CI: it is slow, it depends
on the network, and the commercial rows need licences the runners do not have,
so a maintainer regenerates the table locally with `make compat_table`. You do
not need to run it yourself for an ordinary change: do it when you extend a
backend's `validated_versions`, `supported_versions` or
`library_names` to a new solver release, or when you edit the manifest, and
include the regenerated table in your pull request; never extend either list
without a recorded ✅ or ☑️ row, or a maintainer's run, behind it. A release the
implementation can neither drive at runtime nor support partially is the one case
that calls for a new implementation, `impl/v2`, with its own lists; the `mippp`
aliases in `all.hpp` then move to it and `impl/v1` stays available unchanged.

### Supporting a release partially

A release is supported partially when the implementation builds and solves
models on it, but some member needs what the release lacks: an entry point it
does not export, or a behavior that changed later. HiGHS 1.7.2 to 1.13 is the
case today, whose `compute_iis()` needs 1.14, and whose 1.7 also lacks the
entry point `add_mip_start()` needs. Such a member:

- throws `mippp::feature_unavailable_error`, from
  [include/mippp/utility/solver_exceptions.hpp](include/mippp/utility/solver_exceptions.hpp),
  never a plain `solver_error`, and throws it before it changes the model's
  data, so the model stays usable. Its message names what is missing, the
  library's path and the release it reports. The type derives from
  `solver_error`, as `license_error` does, and never from `license_error`, on
  which the tests skip whatever the release.
- keys on the entry point when the entry point is what is missing
  (`find_function` returned `nullptr`), as `add_mip_start()` on `highs_milp`
  does, and on a release floor otherwise. A floor is a public
  `static constexpr solver_version` of the api class, as
  `highs_api::native_iis_release` is, never a literal in the model code, and a
  `static_assert` beside it checks that it is at or below the `from` of every
  range of `validated_versions`: a validated release never lacks a member.
- is never called from a core suite (`LpModelTest`, `MilpModelTest`,
  `QpModelTest`), so that building and solving models is what every supported
  release does.

The release then goes in `supported_versions`, beside `validated_versions`, on
the evidence of a matrix row, or a maintainer's run, whose only failure is the
version test and whose lacking skips all lie outside the core suites; once
claimed, the row reads ☑️. `solver_api::support_of`, which classifies a release,
checks with a `static_assert` that `supported_versions` contains
`validated_versions`.

The shared suites stay solver-agnostic: none of them skips on a solver or on a
release. `model_test::SkipOnLicenseError`, which wraps the shared test bodies,
catches `feature_unavailable_error` and skips the test with a reason opening with
`release lacks: `, the `release_lacks` constant of
[test/test_suites/all.hpp](test/test_suites/all.hpp) that the matrix matches
verbatim. On a validated release it fails the test instead, so a gate that
misfires there cannot pass for a skip, as long as the library reports its
version: one that reports none (SoPlex, a `devel` build) skips, and only the
matrix holds it to its claim, by package version. A backend's own tests, in
`test/solvers/<name>.cpp`, may key a skip on the release, through the api's floor
constant and with the same prefix, as the HiGHS tests of the native routine do.

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

Tests that time solves against the wall clock, `TimeLimitTest.interrupts_long_solve`
and `TimeLimitIncumbentTest.keeps_the_incumbent` for now, are registered apart from
their suites with ctest's `RUN_SERIAL`: a parallel `ctest` runs them one at a time,
since a loaded machine stretches a solve past any fixed tolerance, or stops it before
its first incumbent. A new timing-dependent test belongs in the same `SERIAL_TESTS`
filter of [test/CMakeLists.txt](test/CMakeLists.txt).

### Code in the documentation

C++ on a new documentation page is not written inline, where it could fall
behind the API, but compiled and tested like the rest. Each such page has one
source file under [test/doc_snippets/](test/doc_snippets/), named after it, and
each block the page shows is a section of that file, between the
comments `// --8<-- [start:<name>]` and `// --8<-- [end:<name>]`. The page
includes the section with the line
`--8<-- "test/doc_snippets/<page>.cpp:<name>"` inside a code fence, through the
`pymdownx.snippets` extension configured in [zensical.toml](zensical.toml).
Paths are relative to the repository root, where `zensical build --clean` and
`make doc` run, and the build fails on a missing file or section. A start
marker without its end does not fail it, since the page then shows the rest
of the file, so `make check-snippets` runs
[misc/tools/check_doc_snippets.py](misc/tools/check_doc_snippets.py), which
requires one start and one end per section. CI runs that check and the
documentation build on every pull request, and fails on any issue the build
reports, broken links and anchors included.

A section is plain user code, without GoogleTest. Test cases below it in the
same file call that code on a real backend, through a `model_test` fixture that
skips where the solver is missing, and check what the page says of it; output
the page shows lives in a text file next to the source, which the page includes
and the test compares against. The files are core sources of `mippp_test`, so
every CI job compiles them whatever `TEST_SOURCE` says, and `make check-format`
covers them. A section must compile, and its tests must pass.

## Coding style

- Format all C++ with **clang-format** using the repository's
  [.clang-format](.clang-format) (Google base style, 4-space indent, no tabs).
  Run `clang-format -i` on changed files before committing.
- Match the surrounding code: naming, header layout, and idioms already in the
  file take precedence over personal preference.
- Keep the library **header-only** and dependency-free. Solver libraries are
  loaded at runtime via `detail::dynamic_library`; do not add link-time
  dependencies on solver SDKs.
- Target C++23 as used elsewhere in the codebase. GCC 14 and Clang 18 are built in
  CI, so a C++26 feature may only be used behind a fallback that keeps those
  compilers working (as [include/mippp/detail/concat_view.hpp](include/mippp/detail/concat_view.hpp)
  does for `views::concat`).

## Adding a new solver backend

Solver backends live under
[include/mippp/solvers/](include/mippp/solvers/)`<name>/impl/v1/` — one
implementation per solver, in the namespace `mippp::<name>::impl::v1`, driving
every release it can adapt to at runtime. Follow the layout of an existing backend
such as [glpk](include/mippp/solvers/glpk/impl/v1/) or
[highs](include/mippp/solvers/highs/impl/v1/):

- `<name>_api.hpp` — thin binding that loads the solver's C API. It redeclares the
  C prototypes it needs, so the solver's SDK headers are not required to build
  (defining `MIPPP_INCLUDE_<SOLVER>_HEADER` includes the real header instead, to check
  them against a release), lists them in an `X`-macro, and resolves each one in its
  private constructor: `lib.get_function<F>("name")` fetches a required entry point
  (throwing `detail::symbol_not_found`) and `lib.find_function<F>("name")` an
  optional one, returning `nullptr` for entry points absent from older releases
  (see the `*_OPTIONAL_FUNCTIONS` lists of the HiGHS and Gurobi bindings). The
  class derives from `detail::solver_api<<name>_api>`, which provides `load()`,
  `library_path()`, `library_version()` and the static `support_of()` from
  static data members: `key` (the `MIPPP_<KEY>_LIBRARY` stem), `library_names`
  (newest first), `validated_versions` (half-open `solver_version_range`s, see
  the [compatibility matrix](#the-version-compatibility-matrix)) and, optionally,
  `supported_versions`, the wider ranges of a backend that supports some
  releases partially (see
  [supporting a release partially](#supporting-a-release-partially)), and
  `probe_symbols`. A new backend usually starts with `validated_versions`
  alone. The constructor ends by handing the base what the library
  reports, `check_library_version(...)`, as components (`{major, minor, patch}`)
  or as the string the solver returns; that call stores it and warns when the
  release is in neither list. A solver whose C API reports no version (SoPlex)
  simply does not call it.
- `<name>_base.hpp` — shared model machinery. Derive it from `model_base`
  (`remapping_model_base` when the solver renumbers columns on deletion) with
  protected inheritance and re-expose `default_variable_params` and
  `is_infinite` with public using-declarations; define `infinity()` to return
  the solver's own "no bound" threshold (`GRB_INFINITY`, `CPX_INFBOUND`,
  `SCIPinfinity(scip)`, ...). Bounds are handed back to the user exactly as the
  solver stores them — never normalised — which is why the threshold, not a
  library constant, is what `is_infinite` compares against. A model is quiet
  from construction: switch the solver's log off in the constructor through its
  own parameter, never by redirecting the standard output, and expose that
  parameter as `set_verbose(bool)`/`is_verbose()`; `VerbosityTest` checks both.
- `<name>_lp.hpp`, `<name>_milp.hpp`, and (where supported) `<name>_qp.hpp` —
  the model classes exposing the MIP++ interface.
- an `all.hpp` aggregating the headers for convenience.

Then add a `test/solvers/<name>.cpp` file that instantiates the shared test
suites (see above) plus `MIPPP_API_VERSION_TEST(<Name>_api, <name>_api, "<KEY>")`,
which defines the `<Name>_api.loaded_release_is_a_supported_one` test of the
[compatibility matrix](#the-version-compatibility-matrix), and register it in
[test/CMakeLists.txt](test/CMakeLists.txt).
Update the feature tables in [docs/assets/features_tables/](docs/assets/features_tables/) and
the solver list in the README. If published builds of the solver are downloadable
without an account, declare a source for it in
[misc/tools/compat_manifest.json](misc/tools/compat_manifest.json) so the new backend gets a
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
