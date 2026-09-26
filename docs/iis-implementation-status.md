# IIS implementation status

The original IIS implementation checklist is **not complete**. The existing
`compute_linear_iis` utility accepts an explicit linear system and creates
temporary models. It is useful independently, but does not implement the
requested model-level API. There is no recorded approval of this substitution
in the supplied checklist. Its companion design document and six open questions
were not supplied; the existing `docs/iis.md` is the earlier utility's guide.

| Checklist step | Verified source state |
| --- | --- |
| 0: design decisions and PR response | Decisions on the six original questions and the requested PR response are not established by this checkout. |
| 1: MOSEK fixes | Single `optimizetrm` call, corrected signature, scoped cleanup, solution-slot selection, time limits, and optimizer-count regression tests exist. The missing MILP `TimeLimitTest` registration was added during review. Runtime MOSEK validation is still required. |
| 2: generic model API | `iis_status`, `has_iis`, handle-based snapshots, and `test/test_suites/iis.hpp` are absent. The separate utility has index-based results and its own tests. |
| 3: row relaxation | `has_modifiable_constraint_bounds`, row-side setters, and the shared row-bound modification suite are absent. |
| 4: Clp fallback | Generic deletion, budgets, cancellation, conservative status classification, and crossing-variable-bound checks exist in the explicit-system utility. `clp_lp::compute_iis()`, original-model restoration, quadratic-objective preservation, and handle snapshots are absent. Retained trials use auxiliary slack variables. |
| 5: more fallback backends | Explicit-system tests cover eleven backend families. This does not establish the requested row-bound capability or in-place fallback. Local and CI passes for that requested API remain outstanding. |
| 6: native routines | Gurobi, CPLEX, Xpress, COPT, and HiGHS native IIS paths are absent. Native dual-certificate hints are a different feature. Minimum-version validation and native/fallback comparisons remain outstanding. |
| 7: documentation | The explicit-system guide is in navigation. The requested concepts, shared IIS feature-table column, and replacement model-level guide remain outstanding. The README distinguishes available utility support from planned model/native support. |

## Review changes and validation

The review adds MOSEK's existing shared time-limit suite, keeps assertions enabled
in the separate IIS test executable in release builds, and applies the main test
target's MSVC environment-access setting to that executable. Solution-slot
ranking uses a switch instead of a nested conditional, matching the adjacent
MOSEK status dispatch style without changing selection priorities.
The elasticity loop now exits directly on an exhausted budget, removing one
level of nesting while preserving proof and termination semantics.

Solver-independent tests exercise the deletion oracle, unknown results,
cancellation, budgets, diagnostics, and cleanup. They cannot validate native
solver behavior, licenses, or the missing model-level interface. An independently
compiled header check verifies self-containment; the standard-include audit
alone is not a compilation check. See [Running validation](iis.md#running-validation)
for selecting installed solver integrations.

Local verification used MSVC 19.51 and GoogleTest 1.17.0 in `out/review`.
The release build passed all 39 solver-independent tests; the final readability
edits also passed all 39 tests in a debug rebuild. All 93 public headers
compiled independently, and the standard-include audit passed. The compiler
reported an existing C4267 index-narrowing warning in `model_base.hpp`.
Commercial solver execution and the complete shared backend suites were not
run. Feature-table regeneration requires the missing Pillow/LaTeX rendering
environment; generated images have not been updated for the MOSEK time-limit
registration. No new claim of runtime-validated MOSEK support is made here.
