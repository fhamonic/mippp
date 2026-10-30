# Column manager: feedback from porting gecot's tree_rr

**Status (2026-10-30).** Addressed on `feat/lp-basis` by a breaking redesign
(see the CHANGELOG, 1.1.0, "Column manager"):
- L1: `activate_columns` and `evict_columns` replace `manage_columns`.
- L3: no shipped predicate assumes a sense. `negative`/`positive` already
  covered both, the minimization-only aliases are gone, and `window_below`
  was added.
- L4: a `Data` template parameter, passed in every entry.
- L5: `seed_hash`, the default `Hash`.
- L6: predicates are plain arguments, so a capturing lambda passes straight
  through.
- L8: `age_above_with_backoff`.
- L9: `emplace_master_column(model, seed, add)`.
- L10: `stable_erase_if` keeps the order in `purge_pool`.
- L11: documented. The example below is off: `update_pool_columns` then
  `update_master_columns` sends each column one event; overlapping broadcasts
  age twice.
- X1: on every model, not only the column manager. The result getters throw
  `solve_required_error` from a change of the model to the next `solve()`,
  where `get_status()` reports `unknown`. Snapshots taken before a removal
  read the right variables on CPLEX, Gurobi and HiGHS.

Not addressed:
- L2: deferred by ruling, with no park mode.
- L7: deferred. `<execution>` needs TBB on libstdc++ and is incomplete on
  libc++, for 0.2% of a run.

Design note on `mippp::colgen::column_manager`. It records what porting a real
column generation, gecot's `tree_rr`, onto the column manager showed: what
worked, and what limits the helper's genericity or its use there, ranked by
impact, each with a proposal. It is not user documentation.

**Code basis.** The column manager headers (`utility/column_manager.hpp` and
`utility/column_generation.hpp`) are identical in the `mippp/1.0.0` Conan
package gecot builds against and on this repository's `feat/lp-basis` branch
as of 2026-10-27. Line numbers below refer to this repository.

**Files next to this note.**
- [add_after_solve.cpp](add_after_solve.cpp) and
  [remove_after_solve.cpp](remove_after_solve.cpp) are the two backend probes
  of §3.

**The port** is gecot's `tree_rr_cm` algorithm (gecot commits 30a19eb and
7ea437c), in `include/gecot/solvers/tree_formulation_rr_cm.hpp`.
It reuses `tree_rr`'s master and pricing models unchanged and replaces only
how columns reach the master.

## Summary

- **The port works.**
  - `tree_rr_cm` reaches the same LP relaxation values as `tree_rr`.
  - With gecot's preprocessing now deterministic, one sort of the priced
    columns before emplacing makes its runs repeatable. `tree_rr`'s runs
    cannot be made repeatable that way. gecot does not keep that sort.
  - Optional eviction keeps the master 2.5–3× smaller on Aude. On Quebec,
    where the master LP takes 79% of the time, it gives 54–62% more rounds
    within the same time limit.
  - The compile-time property lists, the plain function-object predicates
    and seed deduplication all held up.
- **Two problems are serious and should be fixed before the helper is
  advertised for general use.**
  - **L1.** `manage_columns` evicts before it activates. At convergence it
    therefore changes the master after its last solve. Reading the master's
    solution then throws on Gurobi and CPLEX, and silently returns an
    objective value of 0 on HiGHS. This happened in the port.
  - **L2.** Eviction needs `has_remove_variable`, which only 4 LP backends
    model: Gurobi, CPLEX, HiGHS and CLP. Every LP backend can change bounds,
    but a remove lambda that sets the upper bound to 0 leaks a variable at
    every reactivation.
- **Two problems limit genericity.**
  - **L3.** The shipped predicates and eviction aliases assume a minimizing
    master. tree_rr maximizes.
  - **L4.** A column cannot carry data other than its key. tree_rr's
    contribution had to go into the seed and be excluded from the seed's
    hash and equality.
- **The rest is ergonomics and documentation** (L5–L11).
- **One finding concerns all of mippp (X1).** Reading a solution after
  modifying the model is unspecified, and the 8 LP backends behave in 4
  different ways. It caused a crash and a silent zero value in gecot's
  `tree_rr` time-limit path, now fixed in gecot (33ccf0d). Column generation
  always mixes modifications and solution reads, so it is the most exposed
  user.

| # | Limitation | Impact | Proposal | Effort |
|---|---|---|---|---|
| L1 | Eviction runs before activation in `manage_columns` | crash or wrong value at convergence | split `activate_columns` / `evict_columns` | S |
| L2 | Eviction requires removal | 5 of 9 LP backends cannot evict | a "park" mode: upper bound 0, variable kept | M |
| L3 | Strategies assume minimization | wrong silently on a max master | sense as a comparator, or a documented convention | S |
| L4 | No per-column data | data forced into the key | `Data` template parameter, like `unordered_map`'s `T` | M |
| L5 | Range seeds need a hand-written hash | boilerplate, duplicated in tests | `colgen::range_hash` / `hash_combine` | S |
| L6 | Thresholds are absolute | relative tolerances need a capturing predicate | document the pattern and its pitfall | S |
| L7 | `update_columns` is serial over all columns | low: 0.2% of a Quebec run | execution-policy overload | S |
| L8 | Eviction thrashes | many evictions per pool column | document a `times_activated` hysteresis | S |
| L9 | Initial columns bypass the add lambda | duplicated add path | `emplace_master_column(model, seed, add)` | S |
| L10 | Activation order is undocumented | reproducibility | document insertion order and the purge caveat | S |
| L11 | `age` counts events, not rounds | surprise with two events per round | document | S |
| X1 | Solution reads after a modification are unspecified | crashes and silent wrong values | define the contract, cheap debug check | M |

## 1. The port

### 1.1 tree_rr in brief

- **Master LP (maximizes).**
  - `X` variables, one per option, and a budget row.
  - Per case: a contribution variable `C` and a row `C − Σ contribution·λ ≤ 0`.
  - One uniqueness row `Σ λ ≤ 1` per target.
  - One purchase row `Σ λ ≤ X_o` per target and option.
- **Columns** are trees, one per target and option set, with a contribution
  value.
- **Pricing** solves one MILP per target and case, in parallel with TBB. It
  returns at most one tree per target and round.
- **Acceptance** is relative to the target's dual: a tree enters when its
  reduced cost exceeds `1e-7·π_t + 1e-6`.

### 1.2 Mapping onto the column manager

- **Seed.** `tree_seed{case_id, target, options (sorted), contribution}`.
  Equality and hash ignore `contribution`: the same tree priced at two
  rounds must deduplicate (L4).
- **Properties.**
  - pool: `reduced_cost`;
  - master: `reduced_cost` and `age`, the latter only for eviction.
- **One round:**
  1. Solve the master and read the duals.
  2. Price in parallel into a `tbb::concurrent_vector<tree_seed>`, since the
     manager is not thread-safe. The trees are not sorted (gecot's owner
     chose this), so the pool's insertion order, which is the activation
     order, follows thread timing (L10).
  3. `emplace_pool_columns(trees | std::views::as_rvalue)`.
  4. `update_columns`: `priced{rc}` for every column, with the reduced cost
     computed from the seed at the master's duals.
  5. Activate with `colgen::all{lambda}`. The lambda reads the state's
     `reduced_cost` and compares it with `1e-7·π_t + 1e-6`, looking `π_t` up
     through the seed (L6).
  6. Converged if nothing was activated and the round priced every target.
  7. Otherwise, with `--evict-age k`, evict in a **second** `manage_columns`
     call with activation `none{}` and eviction
     `all<conjunction<above<age>, negative<reduced_cost>>>{k, 1e-6}`: columns
     in the master for more than `k` rounds whose reduced cost is now below
     −1e-6, so nonbasic.
     - Two calls work around L1.
     - `negative` is used here because the master maximizes (L3).
- **MIP-start trees** are added before the first solve with
  `emplace_master_column(seed, add_tree(...))` (L9).

The new gecot file is 290 lines. About 50 of them copy `tree_rr`'s master
construction and logging; the rest is the seed, its hash, pricing into a
seed, the reduced cost, and the loop above.

### 1.3 Results on Aude (Gurobi)

**Parity.** Same LP relaxation value as `tree_rr`, within 3e-16 relative:

| Budget | tree_rr | tree_rr_cm |
|---|---|---|
| 3 | 1123539.1724554603 | 1123539.1724554603 |
| 5 | 1256023.9772037698 | 1256023.9772037698 |
| 8 | 1439058.5617959066 | 1439058.5617959064 |

**Repeatability.**
- Both algorithms vary in iteration count from run to run: 27–38 for
  `tree_rr` and 30–39 for `tree_rr_cm` on B=5, over 4 runs.
- With one thread, both always give 29.
- The cause is in gecot: its preprocessing collects arcs into
  `tbb::concurrent_vector`s whose order depends on thread timing.
- gecot now sorts those vectors (748d20a). The order in which pricing
  threads deliver trees is then the only source of variation left.
- Sorting the priced trees by `(case, target)` before
  `emplace_pool_columns` (tested, not kept in gecot):
  - `tree_rr_cm` gave 29 on every multi-threaded run;
  - `tree_rr` still varied (28–39), because its pricing threads insert
    columns into the master under a mutex.
- Without that sort, which is the committed state (7ea437c), both vary:
  31–38 for `tree_rr` and 33–38 for `tree_rr_cm`, with the same value.
- So, with the manager, repeatable runs cost one sort before emplacing.
  Without it, the activation order is whatever the pool received (L10).

**Eviction** (Aude, with both sorts, so runs are deterministic):

| Budget | evict-age | LP solves | master columns at the end | pool | evictions | wall time |
|---|---|---|---|---|---|---|
| 5 | 0 (never) | 29 | 415 | 0 | 0 | 0.29 s |
| 5 | 1 | 55 | 210 | 260 | 679 | 0.39 s |
| 5 | 3 | 37 | 205 | 226 | 420 | 0.36 s |
| 8 | 0 (never) | 45 | 766 | 0 | 0 | 0.60 s |
| 8 | 1 | 75 | 270 | 776 | 2320 | 0.94 s |
| 8 | 3 | 60 | 288 | 716 | 1509 | 0.75 s |

- Same relaxation value in every run.
- The master is 2.5–3× smaller.
- Iterations go up, and the evictions outnumber the pool several times over:
  columns bounce in and out (L8).
- Aude's master is too small for a time gain. See §1.4 for Quebec.

### 1.4 Results on Quebec, B=10000, 140 s (Gurobi)

Setup: 438 targets, a master of 111,618 rows. Single runs, from one binary
built with both sorts of §1.3 and timers around the three phases. Column generation is far from converged at 140 s (the optimum is
about 1.21e11), so these compare progress, not end results.

| Run | LP solves | Relaxation value | Master columns at the end | Pool | Evictions | Master LP | Pricing | Column bookkeeping | Peak memory |
|---|---|---|---|---|---|---|---|---|---|
| tree_rr | 41 | 5.658e10 | – | – | – | – | – | – | 1.31 GB |
| tree_rr_cm | 39 | 5.622e10 | 15,689 | 3 | 0 | 110.4 s | 28.6 s | 0.29 s | 1.29 GB |
| tree_rr_cm, evict-age 3 | 60 | 5.895e10 | 14,856 | 5,656 | 72,016 | 98.1 s | 40.6 s | 0.71 s | 1.75 GB |
| tree_rr_cm, evict-age 1 | 63 | 5.952e10 | 12,940 | 5,898 | 95,953 | 96.7 s | 42.2 s | 0.71 s | 1.73 GB |

- **Without eviction, `tree_rr_cm` tracks `tree_rr`.** On Quebec the master
  LP dominates: 79% of the time limit.
- **Eviction pays here, unlike on Aude.**
  - A master solve drops from 2.8 s to 1.5–1.6 s.
  - The same 140 s gives 54–62% more rounds and a 5–6% better relaxation
    value.
  - Peak memory rises by about 0.45 GB, for the pool and Gurobi's work on a
    changing master.
- **Thrashing (L8) is large:** 72,000–96,000 evictions for a pool of about
  5,700 columns.
- **The column manager's own work** (emplacing, updating every column,
  activating, evicting) stays under 1 s per run (L7).

### 1.5 Backends (Aude B=5)

`tree_rr_cm` with each `--solver` it accepts (every one but GLPK, which is not
thread-safe), without and with `--evict-age 3`:

| Backend | Relaxation value | Eviction |
|---|---|---|
| gurobi, copt, xpress, highs, scip, cbc | 1256023.977204–.977205 | evicts on gurobi, highs, scip (HiGHS master), cbc (CLP master) |
| cplex | 1256023.977234 | evicts: 39 LP solves and 397 evictions in 690 s; slow because of CPLEX pricing |
| mosek | 1256023.977168 | – |

- **Agreement.** All values agree within 3e-11 relative.
- **Eviction ignored where removal is missing.** On copt, xpress and mosek,
  `--evict-age` is ignored with a warning, because `has_remove_variable` is
  false (L2).
- **MIP start.** With the same `--mip-start`, `tree_rr` and `tree_rr_cm` end
  on the same value.

## 2. Limitations, by impact

### L1. `manage_columns` evicts before it activates

`manage_columns` selects and removes the columns to evict
(`column_manager.hpp:259–267`), then activates (`:281–286`). Whether anything
will be activated is unknown until the call has already changed the master.
At convergence nothing is activated, but evicted columns have still been
removed after the master's last solve.

- **Evidence.**
  - The first version of `tree_rr_cm` used one `manage_columns` call per round.
    On Aude B=5 with `--evict-age 1`, it converged, then failed reading the
    master's solution: `Unable to retrieve attribute 'X'` (Gurobi).
  - [remove_after_solve.cpp](remove_after_solve.cpp) shows the backends
    disagree. Removing a nonbasic column after an optimal solve, then
    reading:

    | Backend | Result |
    |---|---|
    | gurobi | throws `Unable to retrieve attribute 'X'` |
    | cplex | throws `CPLEX Error 1217: No solution exists` |
    | highs | objective value 0, variable values unchanged |
    | clp | solution unchanged |

    On HiGHS, the port would have reported a relaxation value of 0 without
    any error.
- **Why the docs do not cover it.** The column generation page already says
  to take the basis "after `solve()` and before `manage_columns`". The same
  rule silently applies to the solution, and a convergence test needs the
  activation count before deciding whether to modify the master.
- **Proposal: split the operation.**

  ```cpp
  // add the pool columns the strategy selects; returns how many
  std::size_t activate_columns(Model &, AS && activation, AF && add);
  // remove (or park, L2) the master columns the strategy selects
  std::size_t evict_columns(Model &, ES && eviction, RF && remove);
  ```

  - These are single-purpose algorithms, so the user places the convergence
    test between them:

    ```cpp
    if(!mgr.activate_columns(model, improving, add) && complete) break;
    mgr.evict_columns(model, unattractive);
    ```

  - Keep `manage_columns` as a thin wrapper, documented for loops that always
    solve again. Its evict-first order is the one the basis paragraph of the
    docs assumes.
- **Cost.** Moving code. No new state.

### L2. Eviction requires `has_remove_variable`

- **Which backends.** Only `gurobi`, `cplex`, `highs` and `clp` model
  `remove_variable(s)`. `copt`, `xpress`, `mosek`, `glpk` and `soplex` cannot
  evict through `manage_columns`. gecot's `--evict-age` therefore warns and
  is ignored on copt, xpress and mosek.
- **Why a user workaround leaks.** Every LP backend models
  `has_modifiable_variable_bounds`. But a remove lambda that sets the upper
  bound to 0 would leak: the pool state holds no variable
  (`in_pool_state{}`, `column_manager.hpp:112, 134`), so reactivating the
  column calls the add lambda and creates a second variable.
- **Proposal: a "park" deactivation mode.** An evicted column keeps its
  variable with upper bound 0, and reactivation restores the bound instead of
  adding a column.
  - In a pay-for-what-you-use design, this is a policy parameter of the
    manager, for example `colgen::remove_columns` (default when
    `has_remove_variable`) or `colgen::park_columns`.
  - Only in park mode does the pool state hold `std::optional<variable>`:
    empty for a column never activated, set for a parked one.
- **Benefits.**
  - Eviction on every LP backend.
  - No id remapping and no basis discarded on Gurobi (the docs' warning about
    evicting basic columns).
  - Reactivation costs a bound change rather than a column insertion.
- **Costs.** Parked columns stay in the solver's matrix. Simplex pricing
  skips fixed columns, but they use memory. That is a fair trade for a pool
  bounded by `purge_pool`.

### L3. Strategies assume a minimizing master

These predicates and aliases encode minimization:
- `negative<reduced_cost>`, the natural activation predicate
  (`column_generation.hpp:334`);
- `evict_window_above<K>` (`:483`);
- `evict_unattractive_aged`, which is
  `conjunction<above<age>, above<reduced_cost>>` (`:487`).

Used on a maximizing master, they activate unattractive columns and evict
attractive ones, with no error. The `column_manager` test suite and the docs
only show minimization. The port had to spell out `positive`/`negative` the
other way round.

- **Proposal.** Either:
  - (a) document a convention: events carry reduced costs in the minimizing
    sense, negated by the user on a maximizing master; or
  - (b) name predicates by meaning, with a sense parameter, like the
    `Better` comparator proposed for `dual_smoother`:
    `improving<P, Better>(tol)` and `unattractive<P, Better>(tol)`, plus the
    eviction aliases on top.
- **Recommendation:** (b). (a) moves a sign flip into every user's
  reduced-cost formula, which is exactly where sign errors happen.

### L4. A column carries no data besides its key

At emplacement the state is `in_pool_state{}`: every property is
value-initialized or takes its static `initial_value()`. Data known when the
column is priced cannot be attached, such as:
- tree_rr's contribution, needed by the add lambda and the reduced cost;
- a cost;
- the round it was found;
- precomputed coefficients.

It must go into the seed, which then needs a custom `KeyEqual` and `Hash` to
keep it out of the identity, as the port does. The alternative is a side
table keyed by seed.

- **Proposal: a `Data` template parameter**, like `std::unordered_map`'s
  mapped type.
  - `emplace_pool_column(seed, data...)` constructs it.
  - It stays with the column across transitions.
  - Entries expose it to the add lambda and to predicates.
  - Its default is an empty type under `[[no_unique_address]]`, so users who
    don't need it pay nothing.
  - Properties keep their role: values driven by events.

### L5. Range seeds need a hand-written hash

Both the `column_manager` test suite (`pattern_hash`) and the port wrote the
same `hash_combine` loop, because the standard has no hash for vectors or
tuples.

- **Proposal.** Ship `colgen::hash_combine` and a `colgen::range_hash`, or a
  default `Hash` that falls back to them for ranges and tuple-like seeds.

### L6. Thresholds are absolute constants

`above<P>{rhs}` compares against a fixed value. tree_rr accepts a tree when
`rc > 1e-7·π_t + 1e-6`, which depends on the target's dual.

- **What works.** A lambda predicate with `colgen::all{lambda}` (CTAD), built
  in each round so it captures that round's duals. Predicates being plain
  function objects pays off here.
- **Pitfall to document.** Folding the tolerance into the event's reduced cost
  (`priced{rc − 1e-7·π_t}`) looks simpler but breaks eviction: basic master
  columns, with `rc = 0`, then look unattractive.
- **Proposal.** Document the capturing-predicate pattern and this pitfall in
  `docs/algorithms/column-generation.md`.

### L7. `update_columns` is serial over every column

Each round the port recomputes the reduced cost of every pool and master
column from its seed. On Quebec, with about 16,000 master columns, the
whole bookkeeping took 0.29 s of a 140 s run: emplacing, updating every
column, selecting and activating. Not a bottleneck here.

- **Proposal, low priority.** Each column's event is independent of the
  others, so an overload taking an execution policy is the STL-compliant way
  to parallelize, for pricers whose events are expensive:
  `update_columns(std::execution::par, make_event)`.
- **Already possible.** The two-argument `update_columns` can take master
  columns' reduced costs from `get_reduced_costs()` and recompute only the
  pool's.

### L8. Eviction thrashes

On Aude B=8, `--evict-age 1` made 2320 evictions for 776 pool columns. Most
evicted columns come back within a few rounds.

- `times_activated` exists, but no shipped strategy uses it.
- **Proposal.** A documented alias that never evicts a column already
  activated more than `n` times, or an age threshold that grows with
  `times_activated`.

### L9. Initial columns bypass the add lambda

`emplace_master_column(seed, var)` requires adding the column outside the
manager. The port therefore calls its add lambda by hand for the MIP-start
trees.

- **Proposal.** An overload `emplace_master_column(model, seed, add)` that
  calls `add` and registers the result.

### L10. Activation order is undocumented

Candidates are visited in map order. `unordered_dense_map` iterates in
insertion order, except that `erase` uses swap-and-pop, so `purge_pool`
reorders the pool. Activation order sets the column order in the master, so
it affects LP paths, iteration counts and reproducibility (§1.3).

- **Proposal.** Document "activation follows insertion order, until a purge".
  Consider an order-preserving compaction in `purge_pool`: purges are rare,
  so O(n) is fine there.

### L11. `age` counts events, not rounds

`age` increments on every event carrying `reduced_cost`
(`column_generation.hpp:206–218`). A loop that sends two such events per
round, for instance `update_pool_columns` then `update_master_columns`, ages
columns twice as fast as the name suggests.

- **Proposal.** Document it.

## 3. Related finding outside the column manager

### X1. Reading a solution after modifying the model is unspecified

`docs/solving/updates.md` says solution objects are snapshots. It does not
say what `get_solution()` and `get_solution_value()` return when called
after a modification and before the next `solve()`. The backends disagree.
After adding one column to an optimal LP, then reading
([add_after_solve.cpp](add_after_solve.cpp)):

| Backend | Result |
|---|---|
| gurobi | throws `Index out of range for attribute 'X'`: mippp asks for one more variable than Gurobi has before its lazy update |
| cplex | throws `CPLEX Error 1217: No solution exists` |
| copt | throws `API call failed: invalid data` |
| xpress | returns `y = 0` instead of `1` (wrong values) |
| highs | returns objective value `0` (wrong value) |
| mosek, clp, glpk | return the previous solution |

After a removal, see L1.

- **Impact on gecot.**
  - `tree_rr` stops on its time limit after a pricing round that has already
    added columns, then reads the master's solution.
  - On main (bbc3410), Quebec B=10000 with a 140 s limit failed on Gurobi
    with that exact error and wrote no result.
  - Aude B=8 with a 0.3 s limit crashed on one of two runs on Gurobi, and
    reported `relaxation_value` 0 on HiGHS every time.
  - The fix, 33ccf0d, solves the master again before reading.
- **Proposal.**
  - Define the contract: solution, dual, reduced-cost and basis getters are
    valid only between a `solve()` and the next modification.
  - Enforce it uniformly. Every mutator sets a `_modified_since_solve` flag
    and every getter checks it, throwing a mippp error that names the
    contract. That is one bool per model, and could be debug-only if the
    branch matters.
  - `has_status_reset` and `status::unknown` already exist; a modification
    could reset the status too.
  - Snapshots fetched before the modification stay valid, as documented.
    "Fetch right after `solve()`" is the pattern to recommend to column
    generation users.

## 4. What worked well, to keep

- **Compile-time property lists.** The master state carries `age` only
  because eviction reads it, and dispatch costs nothing for unused events.
- **Plain function-object predicates.** A capturing lambda with CTAD
  (`colgen::all{lambda}`) covered tree_rr's relative tolerance without any
  library change.
- **Seed deduplication and the pool.** A column evicted and needed again comes
  back by activation, without pricing it again.
- **No thread-safety inside the manager.** Collecting in parallel and
  emplacing serially is the right split. It also leaves the user one place
  to sort what is emplaced, the cheap route to reproducible runs that
  `tree_rr` lacks.
- **`emplace_pool_columns_result`.** Its counts are useful diagnostics, even
  though the port only checks activation.

## 5. Suggested order

1. **X1 and L1.** Both silently produce wrong values or crashes.
   - L1 is a small API split.
   - X1 is a documented contract plus a cheap check.
2. **L3.** Sense-aware predicates, sharing the `Better` comparator of the
   `dual_smoother` proposal.
3. **L2 and L4.** The two template-parameter extensions, park mode and
   `Data`. Both are opt-in, so users who don't need them pay nothing.
4. **Documentation items:** L6, L8, L10 and L11, in the column generation
   page.
5. **Small conveniences:** L5, L7 and L9.
