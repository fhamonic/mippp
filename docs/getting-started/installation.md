# Installation

## Requirements

- **GCC 14 / C++23.** MIP++ targets **GCC 14 / C++23**, though its original target — **GCC 15 / C++26** — remains the recommended toolchain. A handful of C++26 features it relies on (such as `std::views::concat` and `std::flat_map`) are provided by built-in fallbacks, so GCC 14 in C++23 mode is enough to build it.
- [**dylib**](https://github.com/martin-olivier/dylib) **3.0** — the only library dependency, used to load solver shared libraries at runtime. It is pulled in automatically when using Conan.
- At least one solver installed on the machine that *runs* your program (see [below](#making-solver-libraries-discoverable)). Nothing is needed at compile time.

MIP++ is header-only: there is nothing to build, and your binary never links against a solver SDK.

## Getting the headers

=== "Conan (from source)"

    ```bash
    git clone https://github.com/fhamonic/mippp && cd mippp
    conan create . -u -b=missing -pr=<your_conan_profile>
    ```

    then add `mippp/1.0.0` to the requirements of your `conanfile`.

=== "CMake subdirectory"

    Vendor the repository (e.g. as a git submodule) and add:

    ```cmake
    add_subdirectory(dependencies/mippp)
    target_link_libraries(<target> INTERFACE mippp)
    ```

    The build calls `find_package(dylib REQUIRED)`, so dylib 3.0 must be discoverable by CMake (installed system-wide, vendored with its own `add_subdirectory`, or provided through Conan's `CMakeDeps`).

## Making solver libraries discoverable

Each `<solver>_api` object locates and loads the solver's shared library when it is constructed. Resolution proceeds in this order — first match wins:

1. **An explicit path** given to the api constructor: `highs_api api("/path/to/libhighs.so");` loads exactly that file.
2. **The `MIPPP_<SOLVER>_LIBRARY` environment variable**, used verbatim as the full path of the library file. This is the way to pin an exact file when several versions are installed, or when only a version-suffixed soname exists:

    ```bash
    export MIPPP_HIGHS_LIBRARY="/path/to/HiGHS/build/lib/libhighs.so.1.10.0"
    ```

    Recognized keys: `GUROBI`, `CPLEX`, `XPRESS`, `MOSEK`, `COPT`, `SCIP`, `HIGHS`, `SOPLEX`, `CLP`, `CBC`, `GLPK`.

3. **The dynamic loader's search directories** — `LD_LIBRARY_PATH` and the system library directories on Linux (with `/etc/ld.so.conf` honored), `DYLD_LIBRARY_PATH` and the usual locations on macOS, `PATH` on Windows. The conventional decorated name (`libhighs.so`) is preferred; if only version-suffixed variants exist (`libhighs.so.1.10.0`), the lexicographically greatest filename — usually the highest version — is picked.

Solvers installed through the system package manager (e.g. `apt install coinor-clp coinor-libclp-dev libglpk-dev`) are found without any configuration. Commercial and source-built solvers usually live outside the system directories, so export their locations from your shell profile, adjusting the base paths to your installation:

```bash
# Gurobi
export GUROBI_HOME="/path/to/solvers/gurobi1201/linux64"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$GUROBI_HOME/lib"

# HiGHS
export HIGHS_HOME="/path/to/solvers/HiGHS"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$HIGHS_HOME/lib"

# CPLEX
export CPLEX_STUDIO_BINARIES="/path/to/solvers/cplex-community/cplex/bin/x86-64_linux"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$CPLEX_STUDIO_BINARIES"

# FICO Xpress (also needs its license variable)
export XPRESS_HOME="/path/to/solvers/xpressmp"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$XPRESS_HOME/lib"
export XPAUTH_PATH="$XPRESS_HOME/bin"
```

(The same pattern applies to COPT, MOSEK, and COIN-OR builds — see [CONTRIBUTING.md](https://github.com/fhamonic/mippp/blob/main/CONTRIBUTING.md) for the full list.)

Only export the solvers you actually have. If a library cannot be located, the api constructor throws a `std::runtime_error` naming the files it tried and the environment variable to set.

## Checking that everything works

Build and run any of the [examples](../examples.md) — the smallest is `simple_lp.cpp`, which uses HiGHS. If you cloned the repository, the test suite can also be run with `make CONAN_PROFILE=<your_profile>`, and restricted to one backend with `make test highs CONAN_PROFILE=<your_profile>`; backends whose runtime library is missing are skipped automatically.

## Next

[A first model](first-model.md) — the two-variable LP, line by line.
