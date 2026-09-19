# Optional SoPlex Farkas C API extension

`soplex-v8.1.0-farkas.patch` targets upstream SoPlex tag `v8.1.0`, commit
`a00610534e044f35aa298059eb963312336c4c9c`. It changes only the C header,
implementation, and existing C-interface test program. It has not been
submitted upstream. SoPlex's Apache-2.0 license remains applicable to the
patched source; this directory does not vendor the solver.

The two functions return C `int` values (1 for success/available, 0 otherwise).
The getter accepts caller-owned storage of at least the original row count,
checks null/undersized arguments, and catches extraction exceptions at the C
boundary. Neither entry point invokes optimization. A presolve-only
infeasibility result need not supply a certificate.

## Reproduce the WSL/Linux build

Run in a development directory; replace `/path/to/mippp` with this repository.
The minimal floating-point build needs CMake, a C/C++ compiler, and Git. It
disables optional rational arithmetic and external presolve dependencies.

```bash
git clone --depth 1 --branch v8.1.0 https://github.com/scipopt/soplex.git soplex-farkas
git -C soplex-farkas apply /path/to/mippp/patches/soplex-v8.1.0-farkas.patch
cmake -S soplex-farkas -B soplex-build -DCMAKE_BUILD_TYPE=Release \
  -DBOOST=OFF -DGMP=OFF -DMPFR=OFF -DPAPILO=OFF -DZLIB=OFF
cmake --build soplex-build --target libsoplexshared soplex_c_testing -j2
./soplex-build/bin/soplex_c_testing
export MIPPP_SOPLEX_LIBRARY="$PWD/soplex-build/lib/libsoplexshared.so"
/path/to/mippp-build/test/mippp_iis_test --gtest_filter='SoPlexRay.*:LinearIis/10.*'
```

To test stock compatibility, build the same tag **before applying the patch**,
copy its shared library to a separate file, and set
`MIPPP_SOPLEX_STOCK_LIBRARY` to that file's absolute path. Include `SoPlexStock.*`
in the filter. The test requires both new symbols to be absent and verifies
that IIS extraction still succeeds through the fallback path.

Local validation used:

- Patched library: `/tmp/mippp-soplex-build/lib/libsoplexshared.so`
- Stock snapshot: `/tmp/mippp-soplex-stock.so`
- SoPlex checkout: `../soplex-farkas` relative to the MIP++ worktree.

These temporary build products are not required at runtime on other machines;
rebuild from the pinned source and patch. MIP++ itself remains header-only and
does not require SoPlex headers or libraries at build time. Windows builds can
use SoPlex's CMake workflow, but have not been runtime-tested for this extension.
