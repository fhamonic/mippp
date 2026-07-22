---
name: solver-backend-conformance
description: Add or upgrade a MIP++ solver backend, or add a new capability test suite, and prove conformance through the shared typed test suites. Use when wiring a new solver, bumping a solver API version, or when a backend fails/skips shared suites.
---

# Purpose

Every solver backend in MIP++ earns its feature claims by passing the same
shared, typed test suites (`test/test_suites/*.hpp`). The list of
`INSTANTIATE_TEST(...)` lines per backend **is** the documented feature
matrix — `docs/assets/features_tables/tested_features_table.py` regex-parses
those lines to generate the LP/MILP tables. This skill is the procedure for
extending that system without breaking its invariants.

# When to use

- Adding a new solver backend or a new model class (lp/milp/qp) to one.
- Bumping a backend to a new vendor API version.
- Adding a new capability (a new typed suite) across backends.
- Diagnosing a backend that fails or unexpectedly skips shared suites.

# Architecture invariants

- One header tree per backend: `include/mippp/solvers/<name>/v<version>/`
  with `<name>_base.hpp`, `<name>_lp.hpp` (+ `_milp`, `_qp` as supported)
  and an umbrella `all.hpp`. New vendor versions get a **new** versioned
  directory, not edits to the old one.
- One test TU per backend: `test/solvers/<name>.cpp`, registered in
  `test/CMakeLists.txt` (`MIPPP_TEST_ALL_SOLVER_SOURCES`).
- One fixture per model class:
  `struct <name>_lp_test : model_test<<name>_api, <name>_lp>` with
  `SetUpTestSuite() { construct_api(); }`. `construct_api()` turns a missing
  library into `GTEST_SKIP`, never a failure; license-gated calls go through
  `SkipOnLicenseError`.
- One suite per capability, mirroring the model concepts: a backend claims a
  capability by instantiating the suite, and only then.
- `dumb.cpp` / `dumb_lp` is the harness-reference backend: it materializes
  model modifications without a real solver and is excluded from the feature
  tables. New suites should run against it first.
- Numeric comparisons use `TEST_EPSILON` / `TEST_INFINITY`, never raw
  equality on solver output.

# Workflow: new backend (or new model class)

1. **Pick the nearest existing backend** (same vendor API style) and read
   its header tree and test TU side by side; mirror the structure.
2. Implement `<name>_api` (library loading) and the LP model first. MILP/QP
   only after LP conforms.
3. Create `test/solvers/<name>.cpp`:
   - [ ] fixture per model class,
   - [ ] start by instantiating `LpModelTest` only; build with
         `make test <name>` and iterate,
   - [ ] then add suites one at a time, in the order a full backend lists
         them (diff against `highs.cpp` or `gurobi.cpp` as the reference
         list so no suite is silently forgotten).
4. Register the TU in `test/CMakeLists.txt`.
5. **Unsupported features stay visible**: instantiate every candidate suite;
   comment out the unsupported ones in place (`// INSTANTIATE_TEST(...)`),
   optionally with a reason. Never omit the line entirely — the commented
   line is the capability record and the diff target for future versions.
6. Verify the skip path: the suite must pass on a machine **without** the
   vendor library installed (everything skips) — a hard failure there is a
   fixture bug.
7. Regenerate the feature tables (`make featuers_tables`) and update
   `docs/solvers/` (see the `api-doc-sync` skill).

# Workflow: new capability suite

1. Write `test/test_suites/<capability>.hpp` as a
   `TYPED_TEST_SUITE_P`, and include it from `test_suites/all.hpp`.
2. The suite may use **only** the generic model interface (`new_model()`,
   the model concepts, `TEST_EPSILON`) — no vendor headers, no
   backend-specific calls. If a test needs something the concepts don't
   expose, the concept is missing, not the test.
3. Prove the suite on `dumb.cpp` first (harness correctness), then on one
   real full-featured backend, then instantiate for every backend that
   supports it and add commented-out lines for those that do not.
4. Prefer three layers of tests inside a capability suite:
   - unit-level: one API effect per test,
   - one integration problem with a known optimum (Sudoku / CuttingStock
     pattern),
   - a fuzzy/randomized test when the capability has numeric surface
     (`LpFuzzyTest` pattern).
5. Regenerate feature tables; the new column appears automatically from the
   instantiation lines.

# Conformance checklist (before claiming a backend done)

- [ ] `make test <name>` passes with the vendor library installed.
- [ ] Full suite still passes; on library-less machines everything skips.
- [ ] Instantiation list diffed against a reference backend — every shared
      suite is either instantiated or present-but-commented with a reason.
- [ ] No test bypasses the model concepts to poke the vendor API.
- [ ] Feature tables regenerated; generated images/tex committed together.
- [ ] Solver docs page updated (installation, version range, quirks).

# Common mistakes to avoid

- Deleting (instead of commenting) an unsupported suite instantiation —
  it silently drops the feature from the generated tables *and* removes the
  reminder to retry on the next vendor version.
- Letting a missing library or license produce a test failure instead of a
  skip; CI machines differ in which solvers they have.
- Writing a capability test against one vendor's behavior (term ordering,
  duplicate Q-entries, default bounds) instead of the concept's contract —
  expression terms are an unordered multiset; backends must fold duplicates.
- Comparing solver floating-point output with `ASSERT_EQ`.
- Editing an old versioned solver directory to accommodate a new vendor
  version instead of adding a new `v<version>/` tree.
