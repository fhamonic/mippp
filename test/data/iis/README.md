# Published IIS regression vectors

`test/iis_vectors.cpp` adapts numerical vectors from
[HiGHS 1.12.0 TestIis.cpp](https://github.com/ERGO-Code/HiGHS/blob/755a8e027a99a8d4ecf153a8dde4b2a767cdf384/check/TestIis.cpp),
under the accompanying [MIT license](HIGHS-LICENSE.txt).
Only the small numerical models are adapted; the test harness and independent
checker are local code. No external downloads are required to run the tests.

| Upstream case | Regression covered |
| --- | --- |
| `lp-incompatible-bounds` | Multiple valid conflicts: crossed variable bounds, crossed row bounds, and a three-side row/variable conflict. The test permits different IISs and sizes. |
| `lp-empty-infeasible-row` | Both signs of an empty infeasible row, surrounded by nonempty rows. |
| `lp-get-iis` | A conflict involving two nonnegative variable bounds and `x + y <= -2`, with redundant coupled constraints. |

An additional integer-bound vector comes from the same release's
[TestMipSolver.cpp](https://github.com/ERGO-Code/HiGHS/blob/755a8e027a99a8d4ecf153a8dde4b2a767cdf384/check/TestMipSolver.cpp):
`x + 2y >= 1`, `x >= 0`, `1/4 <= y <= 3/4`, integer `y`.
Its two bound sides form an integer-only IIS. Explicit integer witnesses prove
minimality after either bound is removed; the LP relaxation and the repair
`y <= 1` are feasible. This test checks both rebuilding and retained deletion,
and verifies that elasticity is bypassed for the original integer domain.

The independent checker uses Fourier-Motzkin elimination on these small
continuous systems. An exhaustive subset test checks the fixtures, and each
solver result is checked for valid identities, duplicate-free membership,
infeasibility, and feasibility after every individual member removal.
The checker uses neither the production model-preparation code nor the solver
under test. It is restricted to these small integer/dyadic data, not offered as
a general floating-point or integer feasibility oracle.

Integration tests cover rebuilt deletion, retained deletion, rebuilt elasticity,
and combined retained elasticity/deletion. Each uses batching and row-first
ordering, reverses row identities, and tests positive row scaling by 1, 1/4,
and 4. These transformations preserve the mathematical feasible subsets.

Run `IisVectors.*` without a solver. Run `PublishedIis/0.*` for HiGHS LP,
`PublishedIis/1.*` for HiGHS MIP, `PublishedIis/2.*` for Clp, or
`PublishedIis/3.*` for CBC. Explicitly selected integration tests fail if the
library is unavailable. Existing CMake build directories must update their
cached `MIPPP_IIS_TEST_FILTER` to include the new suites.

Local validation on Windows/MSVC with a source-built HiGHS 1.12.0 passed all
98 selected tests: the solver-free suites, new published vectors for HiGHS LP
and MIP, and the existing HiGHS LP/MIP integrations. This includes 96 LP vector
extractions (four vectors, three scalings, four strategies, two wrappers).
Clp and CBC instantiations compiled but were not executed locally.
Fresh CI configurations select the integrations corresponding to their
`MIPPP_REQUIRED_SOLVERS` environment setting.

Remaining gaps include commercial-backend runtime validation, large research
benchmarks, combinatorial integer conflicts involving many variables,
and broad numerical-stability testing. These LP vectors do not establish those
properties. The unimplemented model-level/native IIS API cannot yet be tested.
