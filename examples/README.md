# MIP++ examples

Small, self-contained programs demonstrating the MIP++ modeling interface. They
depend only on `mippp` (no GoogleTest; only the TSP example also uses the
[MELON](https://github.com/fhamonic/melon) graph library) and are adapted from the
library's test suites and the
[mippp_nqueens](https://github.com/fhamonic/mippp_nqueens) benchmark.

| Example | Demonstrates | Default backend |
| --- | --- | --- |
| [`simple_lp/`](simple_lp) | The minimal build/solve/read loop | HiGHS |
| [`nqueens/`](nqueens) | Lambda-indexed variables, constraint families over ranges | HiGHS |
| [`sudoku/`](sudoku) | Multi-dimensional lambda indexing, cartesian-product constraints | HiGHS |
| [`travelling_salesman_dfj/`](travelling_salesman_dfj) | Branch-and-cut via the candidate-solution callback (lazy DFJ subtour elimination) | Gurobi |
| [`cutting_stock/`](cutting_stock) | Column generation with dual values and `add_column` | HiGHS |

Each example is an independent project folder:

```text
nqueens/
├── main.cpp        # the program
├── CMakeLists.txt  # find_package(mippp) + one executable
├── conanfile.py    # requires mippp/1.0.0, builds with CMake
└── .gitignore
```

Every example selects its backend through an alias at the top of `main.cpp`,
e.g.

```cpp
using milp_type = highs_milp;
```

Change it to target another solver (`gurobi_milp`, `scip_milp`, `cplex_lp`,
...). Note that
`travelling_salesman_dfj` needs the candidate-solution callback, which is currently
validated on **Gurobi, CPLEX and COPT** only.

## Using an example as a template

Copy any example folder to start a new project: it already contains everything
needed to build a MIP++ program.

```bash
cp -r examples/nqueens ~/my_model && cd ~/my_model
```

Then rename the project and executable in `CMakeLists.txt` (`project(...)`,
`add_executable(...)`, `target_link_libraries(...)`) and edit `main.cpp`.

## Building

### Standalone, with Conan

Export MIP++ into your Conan cache once (from a clone of this repository):

```bash
conan create <path/to/mippp> -pr=<your_profile> -b=missing -c tools.build:skip_test=true
```

then build any example from its own folder:

```bash
cd examples/nqueens
conan build . -of=build -b=missing -pr=<your_profile>
./build/nqueens 8
```

The profile must select C++23 or later (`compiler.cppstd=23`); the ones under
[`.github/conan-profiles/`](../.github/conan-profiles) are working starting
points. `travelling_salesman_dfj` additionally requires `melon/1.0.0`, exported
the same way from a clone of [MELON](https://github.com/fhamonic/melon).

### Standalone, with plain CMake

If MIP++ is installed where CMake can find it, no Conan is needed:

```bash
cd examples/nqueens
cmake -S . -B build -DCMAKE_PREFIX_PATH=<install_prefix>
cmake --build build
./build/nqueens 8
```

### All examples, from the repository root

```bash
ENABLE_EXAMPLES=ON conan build . -of=build -b=missing -pr=<your_profile> -c tools.build:skip_test=true
./build/examples/nqueens/nqueens 8
```

(`make examples` runs this with the profile set in the `Makefile`.) The
executables are named after their folder (`simple_lp`, `nqueens`, ...); the TSP
example is skipped when MELON is not found.

## Running

MIP++ loads each solver's C API at runtime, so the corresponding shared library
must be on your loader path when you run an example (e.g. `LD_LIBRARY_PATH` on
Linux). See the repository [CONTRIBUTING.md](../CONTRIBUTING.md) for details, or
set `MIPPP_<SOLVER>_LIBRARY` to point at a specific library file.
