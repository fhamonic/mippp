# Dual smoothing for column generation: design note

Design note for a dual stabilization helper in `mippp::colgen`. Its first
user would be gecot's `tree_rr`. It proposes an interface and gives the
background, the evidence collected so far and the open questions. It is not
user documentation.

Two files sit next to this note:
- [dual_smoother.hpp](dual_smoother.hpp) is a working sketch of the class.
- [cutting_stock_smoothing.cpp](cutting_stock_smoothing.cpp) checks it. It
  runs unit checks without a solver, then cutting stock on HiGHS, minimizing
  and maximizing, and asserts the bound at every round.

Both build against this repository's headers as of 2026-10-27, with gcc 15 and
`-std=c++26`. The build command is at the top of the `.cpp`.

## Summary

- **What.** `colgen::dual_smoother<Model, Better>` implements Wentges smoothing
  with the mispricing schedule of Pessoa et al.
  - Pricing reads the duals `α·π̂ + (1−α)·π_out` instead of the master's
    duals `π_out`.
  - The stability center `π̂` is the separation point with the best
    Lagrangian value so far.
  - When smoothing hides every improving column, the weight `α` decreases on
    a fixed schedule until pricing runs at `π_out` itself. Only that round can
    end column generation.
- **Why in mippp.**
  - Smoothing never modifies the master, so it works on every LP backend with
    `has_dual_solution`.
  - The part worth factoring is the control logic, which is easy to get
    subtly wrong:
    - the interpolated bound;
    - the center update in the right sense;
    - the mispricing schedule;
    - rows added after the center was taken;
    - ending only at `α = 0`.
  - It complements `column_manager`, which handles which columns are in the
    master; the smoother handles which duals the pricing reads.
- **What it leaves to the user.**
  - The pricing part of the Lagrangian value, which depends on the problem.
  - The improving-column test at `π_out`.
  - The subgradient that adaptive `α` needs.
  - Out of scope for now: penalty and box-step stabilization (they modify the
    master), and directional smoothing.
- **Cost.**
  - Memory: two vectors of `num_constraints` scalars.
  - Each master solve: one copy of the duals of the rows you pass.
  - Each dual read: two loads and a few flops (one FMA if written as in
    §3.3), since the separation point is computed on access.
  - Each center move: one pass over the rows.
  - Mispricing: O(1).
  - A user who does not smooth does not instantiate the class and pays
    nothing.
- **Is it worth it?**
  - Fixed `α` gives mixed results, both on gecot and in the literature (§2).
  - The adaptive `α` of Pessoa et al. matches the best fixed `α` without
    tuning in the authors' results.
  - The helper is mostly worth it as the base that adaptive `α` needs, plus a
    bound you get for free when pricing is exact.
  - In gecot's `path_rr`, fixed `α` lost everywhere and the bound was the part
    worth having (§7).

## 1. Background

The notation below is for a **minimizing** master, as in the literature. For a
maximizing master, swap min and max, lower and upper bound, and use
`std::less<>` for `Better`.

### 1.1 Master and pricing

```
min  c·x + Σ_k Σ_{q∈Q_k} c_q λ_q
s.t. A x + Σ_k Σ_{q∈Q_k} a_q λ_q  (≥ | ≤ | =)  b
     l ≤ x ≤ u,  λ ≥ 0
```

- **Static columns** `x` are never produced by pricing and never removed.
- **Generated columns** `λ_q` come from pricing problem `k`. They may be
  evicted (`column_manager`), since pricing covers them anyway.
- **`κ_k`** bounds `Σ_{q∈Q_k} λ_q` at some optimal solution. It is the
  right-hand side of a convexity row when there is one. Otherwise it is any
  valid bound. For cutting stock, every column costs 1, so `κ = z_RMP` works:
  `Σλ* = z_LP ≤ z_RMP`.
- **Convexity rows are ordinary rows.** Their duals enter the reduced costs
  `rc_q(π) = c_q − π·a_q`.

### 1.2 Lagrangian value

For sign-feasible duals `π`:

```
L(π) = D(π) + P(π)
D(π) = π·b + Σ_j min_{l_j ≤ x_j ≤ u_j} (c_j − π·A_j) x_j      master part, concave
P(π) = Σ_k κ_k · min(0, min_{q∈Q_k} rc_q(π))                pricing part
```

- `L(π) ≤ z_LP`, where `z_LP` is the master LP's optimum over all columns.
  With exact pricing, `L` is a valid lower bound.
- The user computes `P` from its pricing results. Under inexact pricing, use
  each subproblem's dual bound (for a MIP pricer, its best bound) in place of
  its best column.
- The helper computes the master part without reading `b`, `A` or the bounds:

```
D(π_sep) ≥ α·D̂ + (1−α)·z_RMP          (D̂: stored master part of the center)
```

This holds for two reasons:
- **`z_RMP ≤ D(π_out)`.** By LP duality, `z_RMP` equals `D(π_out)` plus the
  bound terms of the generated columns. Those terms are 0 without upper bounds
  on `λ`, and ≤ 0 with them.
- **Concavity.** `D` is concave, and `D̂ ≤ D(π̂)` by induction.

So the value the helper computes never exceeds `L(π_sep)` and stays a valid
bound. It is exact under two separate conditions:
- **`z_RMP = D(π_out)`** holds when the *generated* columns have no upper
  bound.
- **`D` is linear on the segment** holds when every *static* column is either
  unbounded on its improving side (its reduced cost keeps its sign along the
  segment, so it contributes 0) or fixed (it contributes a term linear in
  `π`).

Cutting stock meets both trivially, since it has no static columns. gecot's
master also meets both: its `X`, `C` and criterion columns are `[0, ∞)` or
fixed. Static columns with two finite bounds make `D` only piecewise linear,
and the value is then a strictly weaker but still valid bound. The check
program covers that case (§2.3).

**Rows added to the master** (cuts) enter the center with a **zero dual**.
This leaves `D(π̂)`, and so the center's value, unchanged, and the value stays
a bound for the tightened master. Any other change to the master's static part
invalidates `D̂`, and the user calls `reset()`:
- a new static column;
- a changed bound, right-hand side or objective coefficient;
- branching.

### 1.3 Smoothing, mispricing, convergence

- **Separation point.** `π_sep = α·π̂ + (1−α)·π_out`, with `α ∈ [0, 1)`
  (Wentges 1997).
- **Pricing.**
  - Pricing runs at `π_sep`.
  - A column is added only if it improves at `π_out`: `rc_q(π_out) < −ε`.
  - If no column improves at `π_out`, the round is a **mispricing**.
- **Center update.** After a complete pricing round, `L(π_sep)` is known. The
  center moves to `π_sep` when that value is better.
- **Mispricing improves the center.** Under a mispricing with exact pricing,
  `L(π_sep) ≥ α·L(π̂) + (1−α)·z_RMP ≥ L(π̂)`, so the round moves the center
  or ties.
  - Derived here, in this note's notation: `rc` is affine in `π`, the best
    column at `π_sep` has `rc ≥ 0` at `π_out`, `min(0, ·)` and `D` are
    concave, and `L(π̂) ≤ z_LP ≤ z_RMP`.
  - It is the property behind Wentges' convergence argument; check the
    paper's statement and numbering (Pessoa et al. 2018).
- **Mispricing schedule.** Re-price on the same `π_out` with
  `α_k = max(0, 1 − k(1−α))`.
  - The weight reaches 0 after at most `⌈1/(1−α)⌉` attempts.
  - At `α = 0`, the round is plain column generation at `π_out`, so its
    "nothing improves" is the usual optimality proof.
  - Convergence needs `α < 1` and a center no worse than the best bound so
    far. Wang & Khir (2026) summarize this as Pessoa et al.'s Proposition 2;
    I have not checked it against the paper.
- **Indexing to check (O5).** The sketch indexes `k` by pricing attempt: the
  first re-price uses `1 − 2(1−α)`, which goes straight to `π_out` for
  `α = 0.5`. Wang & Khir describe the `k`-th mispricing event as setting
  `max(0, 1 − k(1−α0))`. Read literally, that keeps `α` for the first
  re-price, from the moved center. Both orders terminate. I could not read the
  paper's full text to settle it.

### 1.4 Adaptive α

Pessoa et al. adjust the base `α` once per master iteration:
- Take a subgradient `g` of `L` at `π_sep`.
- If `g·(π_out − π_sep) > 0`, the out point lies in an ascent direction, so
  decrease `α`. Otherwise increase it.
- In the helper's terms, the out point improves when
  `Better{}(g·(π_out − π_sep), 0)`.

The rule's figure is on slide 17 of Sadykov's ROADEF 2013 talk. The step
sizes and the initial `α` are in the paper, which I could not read; check them
before shipping a rule (O6).

The subgradient is `g = b − A x̃ − Σ_k κ_k a_{q̃_k}·[rc_{q̃_k}(π_sep) < 0]`:
- `q̃_k` is subproblem `k`'s best column at `π_sep`.
- `x̃` minimizes the static part at `π_sep`. It is not unique where static
  reduced costs vanish.
- A master without static columns, like cutting stock, avoids that question.
  gecot's master, with static `X`, `C` and criterion columns, does not.
- The helper cannot compute `g`, because it never reads `b`, `A` or the
  bounds. It only needs the sign of `g·(π_out − π_sep)` from the user, who
  reads `π_sep` through `separation_point()`.

## 2. Evidence

### 2.1 The authors' results, adaptive α

Source: Sadykov, ROADEF 2013, slide 21. Values are geometric means of the
ratio for `α = 0` over the smoothed run.

| Problem | Iterations, vs best fixed α | Iterations, vs adaptive α | Time, vs best fixed α | Time, vs adaptive α |
|---|---|---|---|---|
| Generalized assignment | 3.45 | 3.45 | 5.19 | 5.32 |
| Lot-sizing | 2.16 | 2.40 | 3.08 | 4.26 |
| Machine scheduling | 2.30 | 2.29 | 3.04 | 2.98 |
| Bin packing | 1.54 | 1.49 | 1.79 | 1.65 |
| Vehicle routing | 1.32 | 1.15 | 1.37 | 1.28 |

Adaptive `α` matches the best fixed `α`, which has to be tuned per problem.
The gain is smallest on bin packing, the closest problem to cutting stock.

### 2.2 Fixed α can lose

- Wang & Khir (2026) found that rule-based smoothing underperforms plain column
  generation on their cutting-stock instances. Their rules include the
  mispricing schedule above, with `α0 = 0.5`.
- In gecot's `tree_rr` (one column per target per round, CBC pricing), I
  compared α = 0.5 against no smoothing, both with the pricing budget row.
  Counts are measured at the first round reaching a certified gap of 1e-4, over
  15 instance and budget configurations:
  - iterations: fewer on 5, more on 6, about the same on 4;
  - wall time: faster on 5, slower on 8, and 11% slower in total (241 s
    against 217 s);
  - bound: unchanged, as expected;
  - mispricings: 0–3 per run;
  - about 99% of the time is in pricing (Aude B=5: 0.03 s master, 4.98 s
    pricing).

  With so few mispricings, the losses cannot come from wasted re-pricing
  rounds. A center lagging behind `π_out` is the likely cause, and it is what
  adaptive `α` targets.

### 2.3 The sketch on cutting stock

Run with [cutting_stock_smoothing.cpp](cutting_stock_smoothing.cpp): HiGHS
master, one pattern per round, a single run each. Cells show master solves /
pricing rounds.

| Instance | α = 0 | α = 0.5 | α = 0.8 | α = 0.9 |
|---|---|---|---|---|
| test-suite instance (452.25) | 3 / 3 | 3 / 4 | 3 / 8 | 3 / 14 |
| 40 random piece types | 78 / 78 | 63 / 66 | 58 / 64 | 57 / 74 |
| 80 random piece types | 113 / 113 | 114 / 115 | 104 / 108 | 99 / 117 |
| 40 types, buy at 1.2 | 72 / 72 | 56 / 57 | 64 / 70 | 60 / 78 |
| 40 types, buy at 0.9 | 65 / 65 | 53 / 54 | 55 / 59 | 55 / 70 |
| 80 types, buy at 1.2 | 108 / 108 | 101 / 102 | 96 / 102 | 107 / 123 |
| 80 types, buy at 0.9 | 100 / 100 | 96 / 97 | 88 / 94 | 101 / 116 |

The "buy" rows add one **static column per piece type with two finite
bounds**: buy the piece outside at `factor ×` its share of a roll, up to a
third of its demand. This makes the master part `D` only piecewise linear,
the general case of §1.2.
- At 1.2, buying is dearer than cutting and leaves the optimum unchanged; it
  only changes the early rounds.
- At 0.9, the static columns sit at their upper bound in the optimum (651.13
  falls to 629.43 on 40 types).

- **Correctness:** every run converged to the same optimum.
  - At every round, the center's value stayed at or below the optimum
    (minimizing), or at or above it (maximizing, objective negated).
  - The final center value equals the optimum.
  - The minimizing and maximizing runs give identical counts, which checks the
    `Better` orientation.
- **Speed:** fewer master solves at high `α`. Pricing rounds, which is what
  counts when pricing dominates, barely move, then rise with mispricings.

## 3. Design

### 3.1 How it follows mippp's principles

- **Generic like the STL.**
  - The optimization sense is a comparator, `Better`, like
    `std::priority_queue`'s `Compare`. It is named for what it decides, since
    its orientation is not "less": use `std::greater<>` for a minimizing
    master and `std::less<>` for a maximizing one.
  - Duals go in as any mapping keyed by the model's constraints, so
    `get_dual_solution()` passes straight through.
  - The separation point comes out as a mapping view (`maps::function`). Pricing
    code written against `get_dual_solution()`, `duals[c]`, takes it
    unchanged.
  - Rows go in as an input range: `model.constraints()`, or only the rows the
    pricing reads.
- **Simple.**
  - The smoother is a state object, not a driver. The user's loop stays the
    user's, as with `column_manager`.
  - It holds no model reference, no callbacks and no `std::function`.
  - It has five operations: `set_out_point`, `separation_point`, `update`,
    `reduce_alpha` and `reset`.
- **Pay for what you use.**
  - Separation-point values are computed on access, so rows the pricing never
    reads cost nothing.
  - Mispricing is O(1).
  - The O(rows) pass happens only when the center moves.
  - Passing a subset of rows to `set_out_point` stores and smooths only those
    rows. The bound is unaffected: pricing never reads the other rows, so
    leaving them unsmoothed changes nothing it computes.
  - Adaptive `α` is a separate rule calling `set_alpha()`, not a policy built
    into the class.

### 3.2 Interface

```cpp
namespace mippp::colgen {

template <typename Model, typename Better>
class dual_smoother {
public:
    using constraint = model_constraint_t<Model>;
    using scalar = model_scalar_t<Model>;

    explicit dual_smoother(scalar alpha, Better better = {});

    // after an optimal solve of the master: copies duals[c] for every c of
    // the range and starts a new pricing round (no center yet: alpha 0)
    template <std::ranges::input_range R, typename Duals>
    void set_out_point(R && constraints, const Duals & duals,
                       scalar master_value);

    // mapping views keyed by constraint, computed on access
    auto separation_point() const;   // a * pi_center + (1 - a) * pi_out
    auto center() const;

    // after a complete pricing round at the separation point: moves the
    // center there if (interpolated master part + pricing_part) is Better
    bool update(scalar pricing_part);

    // after a mispricing: next weight of the schedule; false if the round
    // already priced at pi_out, i.e. column generation has converged
    bool reduce_alpha();

    scalar alpha() const noexcept;           // base weight
    void set_alpha(scalar) noexcept;         // for adaptive rules
    scalar separation_alpha() const noexcept;
    std::size_t num_mispricings() const noexcept;  // on this pi_out
    const std::optional<scalar> & center_value() const noexcept;

    // after any change of the master other than new columns or new rows
    void reset() noexcept;
};

}  // namespace mippp::colgen
```

Preconditions and semantics to document:
- **`set_out_point`** follows an *optimal* LP solve. Pass the same rows every
  round, plus new ones. A row the range has not covered before enters the
  center with a zero dual (§1.2).
- **Views from `separation_point()` and `center()`** read the smoother's current
  state, like `std::ranges::ref_view`.
  - Concurrent reads are safe; do not call a mutator while pricing reads
    them.
  - Do not move the smoother while a view is alive: the views capture
    `this`.
  - Reading a row never passed to `set_out_point` is a precondition violation.
- **`update`** needs a pricing part valid for the whole round (§1.2).
  - After a partial round (time limit, pricing stopped at the first columns),
    do not call it.
  - Calling `reduce_alpha()` stays correct after a partial round.
- **`center_value()`** is a bound on the master LP over all columns only if
  every pricing part was. Do not call it a bound in the docs without that
  condition.

### 3.3 Implementation notes

See [dual_smoother.hpp](dual_smoother.hpp); about 130 lines.

- **Storage.** `_center` and `_out` are indexed by `uid()` and always have
  the same size. mippp never removes rows, so uids stay dense. The rule that a
  new row enters the center at 0 is `_center.resize(i + 1)`.
- **`separation_point()`** is
  `maps::function([this](const constraint & c) { ... })`.
  - It could be written `out + a·(center − out)` for a single FMA. At
    `a = 0` that still gives exactly `out`.
  - It uses `_sep_alpha` through `this`, so a view taken before
    `reduce_alpha()` sees the new weight.
- **`update()`**
  - Computes `master_part = a·D̂ + (1−a)·z_RMP` and
    `value = master_part + pricing_part`.
  - When `value` is `Better` than the center's value, or there is no center
    yet, it materializes the separation point into `_center` and stores
    `D̂ = master_part` and the value.
  - A tie keeps the old center.
- **`reduce_alpha()`**
  - Returns `false` when `_sep_alpha == 0`.
  - Otherwise it applies the schedule (§1.3, O5) and snaps a positive residue
    below a few ulps to 0. A weight of 1e-17 would otherwise cost one more
    full pricing round at practically `π_out`.
- **`reset()`** drops the center value. The next `set_out_point` then prices
  at `π_out`, and the next `update` takes it as the center.

### 3.4 Usage

A plain loop, minimizing (from the check program):

```cpp
colgen::dual_smoother<highs_lp, std::greater<>> smoother(0.8);
for(bool improving = true; improving;) {
    model.solve();
    const auto duals = model.get_dual_solution();
    const double z = model.get_solution_value();
    smoother.set_out_point(model.constraints(), duals, z);
    do {
        const auto sep = smoother.separation_point();
        const auto pattern = price(sep);  // best column at pi_sep
        smoother.update(z * std::min(0.0, reduced_cost(pattern, sep)));  // kappa = z
        improving = reduced_cost(pattern, duals) < -1e-9;  // tested at pi_out
        if(improving) add_column(pattern);
    } while(!improving && smoother.reduce_alpha());
}
```

With `column_manager`, the improving test becomes "the activation strategy
activated something". Reduced costs sent to the columns must be taken at
`π_out`, never at `π_sep`: the activation and eviction predicates compare
against the master.

```cpp
do {
    const auto sep = smoother.separation_point();
    columns.emplace_pool_columns(price(sep));          // seeds found at pi_sep
    smoother.update(pricing_part);
    columns.update_columns([&](const auto & seed) {    // at pi_out
        return colgen::priced{reduced_cost(seed, duals)};
    });
    improving = columns.manage_columns(model,
                    colgen::all<colgen::negative<colgen::reduced_cost>>{},
                    add_column).num_activated > 0;
} while(!improving && smoother.reduce_alpha());
```

`emplace_pool_columns_result::num_already_in_master` counts columns found at
`π_sep` that are already in the master. They cannot improve at an optimal
`π_out`, so the counter signals mispricing. It is a diagnostic, not the test:
a new pool column can also fail to improve.

- **Parallel pricing:** price from one `separation_point()` view in all
  threads, then call `update()` once, after the join.
- **Heuristic pricing:**
  - Give `update()` a pricing part built from subproblem bounds, or skip it.
  - A round at `α = 0` that finds nothing proves convergence only if the
    pricing was exact. Switch to exact pricing before stopping.

### 3.5 Pitfalls for the user docs

1. Testing columns at `π_sep` instead of `π_out`. The master then receives
   columns that do not improve it, and the loop can stop early.
2. Sending `column_manager` events with reduced costs at `π_sep`.
3. Calling `set_out_point` after a non-optimal solve (time limit, iteration
   limit).
4. Changing the master's static part without `reset()`.
5. Reporting `center_value()` as a bound when pricing is heuristic or works on
   a restricted model.
6. Choosing the wrong orientation for `Better`. Convergence survives, because
   the schedule still reaches `π_out`. But the center then chases the worst
   points, and `center_value()` reports the worst value.

## 4. Tests

### 4.1 Unit tests, no solver

Write them in `test/dual_smoother.cpp`, with a fake model as in
`test/column_manager.cpp`. The check program has most of these.
- **Before the first center:** the separation point is `π_out`, and
  `center_value()` is empty.
- **After a center:** the separation point is `α·π̂ + (1−α)·π_out`, row by
  row.
- **Schedule:** `α = 0.8` gives 0.6, 0.4, 0.2, 0, then `false`; `α = 0.5`
  gives 0, then `false`.
- **Center moves only on `Better`,** for both orientations; a tie keeps it.
- **New rows:** a row past the old size reads `(1−α)·π_out` in the separation
  point.
- **`reset()`:** the next separation point is `π_out`.
- **Concepts:**
  - `separation_point()` satisfies
    `input_mapping_of<constraint, scalar>`;
  - the class compiles with only `model_constraint_t` and `model_scalar_t`
    available.

### 4.2 Shared suite, per LP backend

Write it as `test/test_suites/dual_smoother.hpp`, reusing the cutting-stock
instance of the `column_manager` suite (452.25).
- Run `α ∈ {0, 0.5, 0.8}`, minimizing.
- Run the same with the objective negated and `std::less<>`.
- Run a variant with bounded static columns, as the check program's "buy at
  0.9", so the piecewise-linear master part is covered.
- Assert:
  - the optimum at the end;
  - the center's value on the safe side of the optimum at every round;
  - the final center value equals the optimum.
- Do not assert fewer iterations: §2 shows that does not hold in general.

### 4.3 Docs

- Add a "Stabilization" section to `docs/algorithms/column-generation.md`:
  the loop, the user's two duties (the pricing part and the test at `π_out`)
  and §3.5.
- Possibly give the cutting-stock example an `alpha` argument.

## 5. Open questions, with a recommendation

- **O1. Name.** Options: `dual_smoother`, an agent noun like `column_manager`,
  or `wentges_smoothing`. *Rec.:* `dual_smoother`, in
  `mippp/utility/dual_smoother.hpp`, not included by `column_manager.hpp`.
- **O2. Template parameters.**
  - `<Model, Better>` matches `column_manager`; only `model_constraint_t` and
    `model_scalar_t` are used.
  - `<Constraint, Scalar, Better>` needs no fake model in tests.
  - *Rec.:* `Model`.
- **O3. A default for `Better`.**
  - `std::greater<>` would fit the textbook (minimizing) case.
  - A wrong orientation fails silently (§3.5, item 6).
  - *Rec.:* no default.
- **O4. Lazy or materialized separation point.**
  - Lazy costs two loads and an FMA per read.
  - Materialized costs O(rows) at every `α` change and center move.
  - A pricer that reads each dual many times can copy what it reads.
  - *Rec.:* lazy.
- **O5. Schedule indexing.** Does the first re-price use `1 − (1−α)` (the same
  `α`, from the moved center) or `1 − 2(1−α)`? *Rec.:* follow the paper.
  gecot's prototype and this sketch use the latter.
- **O6. Adaptive α.**
  - Ship `set_alpha()` now.
  - Add a free function `colgen::adapt_alpha(alpha, bool out_point_improves)`
    once the paper's step sizes are checked.
  - The subgradient stays with the user (§1.4).
  - *Rec.:* both steps, in that order. Adaptive `α` is what makes the helper
    worth using (§2).
- **O7. Row selection.**
  - Keep the range argument.
  - An overload `set_out_point(model, duals)` would only save typing
    `model.constraints()` and `get_solution_value()`.
  - *Rec.:* range only.
- **O8. Penalty and box-step stabilization** (du Merle et al. 1999; Ben Amor,
  Desrosiers & Frangioni 2009).
  - These are expressible with `add_column` and modifiable bounds: four
    artificial columns per row for the five-piece function.
  - But they change the master, its LP value and its basis.
  - Pessoa et al. find smoothing plus a penalty function cumulative.
  - *Rec.:* out of scope for the first version.

## 6. First user: gecot `tree_rr`

- **Instantiation:** `dual_smoother<LP, std::less<>>`, since the master
  maximizes.
- **Rows:** pass `model.constraints()`. Only the contribution, uniqueness and
  purchase rows are read by pricing; the subset would also do.
- **Pricing part:** `Σ_t max(0, v_t − π_sep[uniqueness_t])`.
  - `κ_t = 1`, from the uniqueness rows.
  - `v_t` is the sub-MIP's optimum, or its best bound under a MIP gap.
- **Acceptance at `π_out`:**
  `π_out[contribution]·value(tree) − Σ_o π_out[purchase(t,o)] − π_out[uniqueness_t] > tol`.
  Every term is already at hand where the column is built, so no second solve
  is needed.
- **Static columns:** the `X`, `C` and criterion variables. MIP-start trees
  are generated columns and need no special handling.
- **Time limit:** a round cut short by `--time-limit` calls neither `update()`
  nor `reduce_alpha()`.
- **Caveat:** gecot's pricing runs on a preprocessed, resolution-pruned graph
  and can miss trees. On Quebec B=20, the converged LP fell below the empty
  solution's score. Until that is explained, `center_value()` only steers the
  center and is not reported as a bound.
- **Release:** gecot pins `mippp/1.0.0`, so it can use the helper only after a
  release that contains it.

## 7. Field test: gecot `path_rr`

gecot's `path_rr` (path formulation) ran a copy of the sketch,
`gecot/utils/dual_smoother.hpp`, on 2026-10-28. The copy only drops the
accessors it did not use. It is on gecot's branch `path-rr-smoothing`
(a99eb98), not merged.

- **Shape.**
  - One subproblem per (source, target) pair that options can change:
    132k–182k pairs on Quebec, depending on the budget and the pricing mode,
    and 268k on the "green" instance.
  - Exact pricing, a biobjective label-setting search per pair. So
    `center_value()` is a valid bound, and gecot reports it.
  - A pair's rows are created with its first column in the master, so new
    rows arrive at every round, up to 1.2M.
- **What worked as designed.**
  - At `α = 0` the helper computes the plain Lagrangian bound, so gecot has a
    single code path. Single-threaded, it reproduces the unsmoothed code
    exactly: the same master value and the same bound at every round.
  - New rows enter the center at dual 0 at every round. The bound stayed above
    the converged optimum at every round, and met it at convergence within the
    pricing tolerances.
  - The interpolated master part is exact here, since gecot's static columns
    are `[0, ∞)`.
- **Friction.**
  - **F1. No row range.** mippp 1.0.0 models have no `constraints()`. The
    client passes `iota(num_constraints()) | transform(constraint(i))`, which
    relies on dense uids. O7's "range only" needs a range to exist.
  - **F2. `α = 1` never ends.** `reduce_alpha()` computes `1 − k·0 = 1`
    forever. The constructor and `set_alpha()` should require `α < 1`, and
    §3.5 should say so. gecot checks `--smoothing` against `[0, 1)`.
  - **F3. Smoothing defeats pricing caches keyed on the duals.** `path_rr`
    skips a pair whose duals are unchanged since its last search. `π_sep`
    moves at every round even when `π_out` does not, so at `α = 0.5` the
    searches rose 13× on Quebec (90k to 1.18M). This belongs in §3.5.
  - **F4.** `update()` returns whether the center moved, but not the
    Lagrangian value at `π_sep`, which a per-round log wants.
- **Results, fixed `α`, Quebec** (518 vertices, 260 options). "Bound rounds" is
  the number of pricing rounds until the bound is within 1e-4 of the converged
  value.

| Budget, pricing | α | Master solves | Mispricings | Searches | Time | Bound rounds |
|---|---|---|---|---|---|---|
| 1000, search | 0 | 15 | 0 | 90k | 5.4 s | 7 |
| | 0.5 | 32 | 3 | 1.18M | 9.7 s | 10 |
| | 0.8 | 77 | 18 | 3.26M | 19.7 s | 24 |
| | 0.9 | 144 | 72 | 7.46M | 36.7 s | 49 |
| 10000, search | 0 | 15 | 0 | 210k | 28.8 s | 7 |
| | 0.5 | 34 | 5 | 2.49M | 43.9 s | 13 |
| | 0.8 | 92 | 37 | 8.74M | 96.3 s | 35 |
| | 0.9 | 182 | 111 | 20.2M | 189 s | 72 |
| 10000, enumerate | 0 | 15 | 0 | – | 33.4 s | 9 |
| | 0.5 | 41 | 1 | – | 39.6 s | 13 |
| | 0.8 | 116 | 6 | – | 64.8 s | 34 |
| | 0.9 | 237 | 14 | – | 107 s | 71 |

- **Results, green** (3,062 vertices, 762 options), B=5, from a greedy
  solution, at most 5,000 activations per round, 600 s:
  - `α = 0`: 45 master solves, bound 3.1186e13.
  - `α = 0.5`: 43 master solves, 0 mispricings, bound 3.1108e13 (0.25% lower).
    The early rounds are worse: 3.59e13 against 3.41e13 at round 2.
- **Reading.**
  - Here many subproblems each add a column per round. Plain column
    generation converges in about 15 rounds on Quebec, and smoothing slows both
    the master and the bound, more as `α` grows.
  - On green, what limits the bound is coverage, not oscillation. A pair's
    purchase rows exist only for the options its columns use, so its pricing
    sees every other option at dual 0, as if free. Only 5,000 of 268k pairs
    gain a column per round.
  - Adaptive `α` could at best steer toward the best fixed `α`, which is 0
    here, so it was not tried.
  - The helper's payoff depends on the problem's shape: a few subproblems
    with oscillating duals, not many weakly coupled ones. §2.2's `tree_rr`
    data points the same way.

## References

- P. Wentges. Weighted Dantzig–Wolfe decomposition for linear mixed-integer
  programming. *International Transactions in Operational Research*
  4(2):151–162, 1997.
- A. Pessoa, R. Sadykov, E. Uchoa, F. Vanderbeck. Automation and combination of
  linear-programming based stabilization techniques in column generation.
  *INFORMS Journal on Computing* 30(2):339–360, 2018.
  doi:10.1287/ijoc.2017.0784.
- R. Sadykov et al. Slides, ROADEF 2013:
  <https://www.math.u-bordeaux.fr/~rsadykov/slides/Sadykov_ROADEF13slides.pdf>.
  Slide 17 (adaptive `α`) and slides 21–22 (results).
- O. Wang, R. Khir. Learning to control stabilization in column generation.
  arXiv:2604.23889, 2026. Includes a survey of smoothing rules and the
  convergence conditions.
- O. du Merle, D. Villeneuve, J. Desrosiers, P. Hansen. Stabilized column
  generation. *Discrete Mathematics* 194(1–3):229–237, 1999.
- H. Ben Amor, J. Desrosiers, A. Frangioni. On the choice of explicit
  stabilizing terms in column generation. *Discrete Applied Mathematics*
  157(6):1167–1184, 2009.
