# IIS API review of 2026-10-05

The maintainer asked on 2026-10-05 whether the IIS feature is coherent with
the design patterns of the library's other features. A second question was
whether simplifications and factorizations would give a simpler API, closer
to the mathematical formulation, with cleaner code. The constraint was that
the result stay flexible, stay the thinnest layer possible, and keep every
relevant datum a backend provides, as the solve status does with a
`std::variant` over a tag hierarchy. This note records the answer.

The review ran on main at `b9d2834`, and every line number below refers to
that commit. It ran in two rounds:

- **Mapping and review.** Four agents mapped the library's non-IIS patterns,
  the IIS core, the five native wrappers, and the tests, docs and rulings.
  Five reviewers then judged the design through one lens each: coherence,
  mathematical formulation, thin layer and backend data, factorization, and
  extensibility.
- **Verification.** Each proposal was checked in refute mode against the code
  and prototyped against the real headers. A three-judge panel and a
  synthesizer decided the completion report. A completeness critic looked for
  what both rounds had missed.

All prototypes passed a gcc-15 `-std=c++26 -fsyntax-only` check, some also a
clang-18 `-std=c++23` check, and a few were built and run. No solver ran during
the review. *Measured* below means a compiled or executed probe, and
*inferred* means a reading of code or manuals.

**Status.** The completion report was ruled on 2026-10-05: the maintainer
asked for the tag hierarchy of [How a run ends](#how-a-run-ends), and it is
implemented on branch `feat/iis-outcome-tags`. Every other recommendation
awaits a ruling; [Pending decisions](#pending-decisions) lists them.

## Verdict

The IIS feature is mostly coherent with the library. Its core shape follows
the library's patterns and should stay:

- the capability concept;
- the per-entity answer;
- the per-routine status variants;
- the snapshot by value;
- the two explicit paths;
- the monotone-oracle engine.

One part departed from the library's pattern: how a run reports its end, as an
enum and an optional enum where the library uses a variant over a tag
hierarchy. Beside it, the review found:

- plumbing written several times over in the native wrappers;
- two places where a backend's data is lost or misreported;
- one native decode that overclaims;
- several false sentences in the docs.

## What follows the library's patterns

- **Capability surface.** `has_iis`, `model_iis_t` and `lp_iis<I, T>`
  (model_concepts.hpp:692-709) copy the basis trio one for one: `lp_basis`,
  `model_basis_t` and `has_lp_basis` (623-634). `has_iis` takes one parameter,
  stays outside `lp_model`, and its type is deduced from the call.
- **Per-entity answers.** The library returns numeric per-entity results as
  mappings (`solution[x]`) and categorical ones as an object with
  `get_status(v)` and `get_status(c)` over tag variants (the basis). An IIS is
  categorical, so `iis_snapshot` (iis_snapshot.hpp:50-55) takes the basis shape.

  Mathematically, the per-entity tag is the restriction to one entity of the
  set S of finite half-spaces. The five tags are exactly the cases a
  restriction can take:
  - none;
  - the lower side;
  - the upper side;
  - both sides as two units;
  - both sides as one unit, for a routine whose unit is the row.

  Overloading on the handle type is also how later handle kinds, SOS and
  indicators, would attach. A mapping shape would need a new accessor per
  kind, and would touch about 130 call sites for no gain in data.
- **Per-routine variants.** Each routine's status variant lists only the tags
  it can produce (iis_snapshot.hpp:75-93), as each backend's solve-status
  variant does (P5 of the patterns map). That `is_a<member_both>` does not
  compile on the Gurobi and CPLEX row status is the library-wide rule of
  `is_a`, documented for statuses at status-and-limits.md:51.
- **Snapshot by value.** The snapshot is keyed by handle id, holds no model
  reference and translates native indices when it is taken. This matches the
  enumeration's snapshot contract (model_concepts.hpp:255-268) and N8.
- **Two explicit paths.** Native `compute_iis()` is a leaf member that resets
  the status and calls a protected base helper, the `solve()`/`_run` split.
  `compute_iis_by_deletion` is a free algorithm in `utility/`, like the column
  generation algorithm. The two paths differ in substance:
  - a native routine picks its own unit (rows whole on Gurobi and CPLEX);
  - the filter keeps a registered callback as background, which the native
    wrappers detach.
- **The engine.** `deletion_filter` over a monotone oracle and candidate
  indices (deletion_filter.hpp:21-30, 150-171) is Chinneck and Dravnieks'
  filter with nothing model-specific in it. `classify_deletion_trial` reads
  the shared status hierarchy, including the flag the tag carries
  (iis_by_deletion.hpp:135-153).
- **The sense/rhs fallback (N41, N43).** It is chosen on capabilities, never
  on a backend type (iis_by_deletion.hpp:42-65).
- **Error policy.** A missing entry point throws `solver_error`. Bad limits
  throw `invalid_argument`. Misuse is documented undefined behavior (N40). A
  feasible model is an outcome.
- **Placement and names.** `utility/` plus a page under `docs/algorithms/`
  follows the column-generation precedent. The `lp_` prefix of `lp_iis`
  parallels `lp_model`, which MILP classes also satisfy. `compute_` matches
  `GRBcomputeIIS` and `COPT_ComputeIIS`.

## How a run ends

The completion report was `enum class iis_outcome {irreducible,
not_proven_minimal, feasible, undetermined}` plus
`std::optional<iis_reason>`, where `enum class iis_reason {solve_limit,
time_limit, cancelled, inconclusive_trial}` (iis_outcome.hpp:20-35). The rest
of the library reports how a call ended as one variant over a tag hierarchy
whose tags carry data: `status::any::solution_available`, read by
`status::solution_available` (status.hpp:23-58). The pair broke that pattern,
with measured consequences:

- **Invalid states.** 8 of the pair's 20 states are never produced, `irreducible`
  with a reason among them. Only the test fixture checks that they never occur
  (iis_cases.hpp:541-545).
- **An empty reason had four meanings** (infeasibility.md:180):
  - a complete answer;
  - a native answer whose routine proved no minimality (N27);
  - a stop by a limit other than time;
  - an interrupt.
- **Native stop causes were lost** while the same backends' solve status names
  them:
  - CPLEX conflict statuses 34 to 39 (iteration, node, objective, memory, user
    and deterministic time) all became an empty reason (cplex_base.hpp:757-764).
  - `cplex_lp` names the same causes for a solve (cplex_lp.hpp:90-98).
  - COPT's pre-solve `NODELIMIT` and `INTERRUPTED` became an empty reason
    (copt_base.hpp:627-630).
  - On an LP, Gurobi's Status after a stopped `GRBcomputeIIS` reads 7, 9 or
    16 (iis.md:180-182, 204), and the wrapper ignored it.
  - Xpress computed a `stopped` bit and discarded it unless the clock blamed
    the time limit (xpress_base.hpp:671-676).
- **A library rule was broken:** a limit you can set is a limit the status can
  report (model_concepts.hpp:306-342, status-and-limits.md:75).
  `set_iteration_limit` on `cplex_lp` or `gurobi_lp`, and `set_node_limit` on
  `cplex_milp`, stopped `compute_iis()` with no reason, and the tests pinned
  the loss (cplex.cpp:328, 370; gurobi.cpp:145).

**Options weighed by the panel**, each scored by three judges with different
lenses:

| Option | Scores |
| :--- | :--- |
| (A) keep the outcome enum, reason as a variant over existing `status::` tags | 4, 4, 5 |
| (B) one variant whose IIS tags derive from `status::` tags | 3, 4, 6 |
| (C) keep the outcome enum, reason as a tag hierarchy of its own | 6, 6, 6.5 |
| (D) one hierarchy of its own, the flag on the root, no reason getter | 6, 8, 7.5 |
| (E) the status quo with more reason enumerators | 4, 4, 4 |
| (F) the status quo | 2, 3, 3 |
| (G) refinements: judge 1 refined C (scored 8), judges 2 and 3 refined D (8.5 each) | |

**Why not reuse the `status::` tags (A, B).** That is a type hazard, not just a
pun, measured with a probe against the real headers. A reused tag satisfies
`variant_of<status::any>`, so every API built for solve statuses accepts an
IIS outcome:

- `classify_deletion_trial` reads an `irreducible` outcome as a feasible trial;
- `status::solution_available` reads a proven conflict as a primal point, the
  opposite meaning.

**Why one hierarchy (D) rather than enum plus reason hierarchy (C).** The two
fields are not independent:

- `not_proven_minimal` against `undetermined` is exactly a flag on the
  undecided branch, equal to the engine's `state.proven` at every return.
- Projecting the new outcome back to the old pair reproduces every
  expectation of test/deletion_filter.cpp: 24/24, measured.
- Keeping both fields keeps the invalid pairs and a second accessor.

The cost of D is churn (about 375 test lines against 109 for C), with no
release carrying IIS yet (v1.0.0 predates it). Option C remained the fallback
if that churn was refused.

**The design chosen:**

```text
iis_outcome::any { bool conflict_available; }
├── completed
│   ├── irreducible        conflict_available
│   └── feasible
└── incomplete             no decision, for a cause the routine does not name
    ├── inconclusive_trial a filter trial proved neither way
    └── stopped            cut short from outside the routine
        ├── interrupted    was iis_reason::cancelled
        └── limit_reached
            ├── time_limit
            ├── solve_limit      iis_limits::max_solves
            ├── iteration_limit
            ├── node_limit
            └── memory_limit
```

- **Lists and folding.** Each path's variant lists only the tags it can tell
  apart. A cause without a tag folds into its nearest listed ancestor,
  explicitly at the producer, as P5 does for statuses.
- **Reading it.** `iis_outcome::conflict_available(o)` reads the flag, as
  `status::solution_available` reads a status. It replaces the old
  `irreducible || not_proven_minimal`.
- **What goes.** `get_reason()`, `iis_reason` and
  `deletion_filter_result::reason` are removed.
- **The incomplete branch.** `incomplete` names N27's case: a routine that
  ended without proving minimality, for no named cause. `inconclusive_trial`
  sits under `incomplete`, not under `stopped`, because the filter's single
  pass still runs to its end. CPLEX status 32, a contradiction that follows an
  iteration-limit stop on the unchanged problem (iis.md:258-265), is also
  `incomplete`, since nothing outside the call cut it short.

**Per-backend lists** (measured or documented, as the mapping tables of the
synthesis record):

| Path | Tags it lists |
| :--- | :--- |
| Deletion filter | incomplete (default value only), irreducible, feasible, inconclusive_trial, interrupted, time_limit, solve_limit |
| HiGHS | incomplete, irreducible, feasible, time_limit. No stop tag: a warning cannot be told from a failed post-check (highs_base.hpp:858-863) |
| Gurobi | incomplete, irreducible, feasible, stopped, interrupted, limit_reached, time_limit, iteration_limit, memory_limit. Status is read only after a stop that left no subsystem; on a MIP it reads 1, and the clock decides |
| CPLEX | incomplete, irreducible, feasible, interrupted, limit_reached (objective and deterministic time), time_limit, iteration_limit, node_limit, memory_limit |
| Xpress | incomplete, irreducible, feasible, stopped, time_limit |
| COPT | incomplete, irreducible, feasible, interrupted, time_limit, node_limit |

**Rulings.**
- N19 (e), "enum class for outcome and reason", is overturned for both enums.
  Its recorded premise, closed flat sets without payload
  (iis_pr_plan.md:105-110), failed on three counts:
  - N2 already reopened the set;
  - the causes refine one another, as `time_limit` refines `limit_reached`;
  - the flag is a payload.
- The "outcome and reason split" item of N37 (g) is overturned. The rest of
  N37 (g) stands.
- N27 (a) is amended in spelling only: its empty reason becomes the
  `incomplete` tag, which is not a limit, so its point that "a missing proof
  does not imply a limit" now holds in the type.
- N2 (b) is kept as the `inconclusive_trial` tag.
- `cancelled` becomes `interrupted`, in line with `status::interrupted` and
  N31 (b).

The prototype measured +252/−146 lines across 10 headers, mostly the native
cause mappings. In test/, 370 distinct lines name the old types, 76 of which
need a hand rewrite.

## Factorizations that overturn no ruling

| Change | Measured effect |
| :--- | :--- |
| One `detail::restore_guard` instead of the seven per-backend guard classes | −152 lines |
| A `detail::iis_answer` builder for the 36 native exit points | −21 to −35 lines |
| `iis_status::sides_of(s)` returning `iis_sides {lower, upper, whole}` | +28 library, −52 in docs code and the example |
| `compute_iis_by_deletion(model, within, limits)` | +42 library, −57 docs code |
| `handle_status_table` over `std::vector<Status>` | 68 → 38 code lines |
| Small cleanups and visibility | about −20 |

**`detail::restore_guard`.** Seven classes repeat one protocol, 277 lines in
all:

- `iis_option_guard` in HiGHS and in Xpress;
- Gurobi's `iis_force_guard`;
- COPT's `iis_time_limit_guard`;
- the callback guards of `cplex_milp`, `xpress_milp` and `copt_milp`.

The protocol is: save, write, a checked `restore()`, and a destructor that
writes back only after an exception. One template in its own header
(37 lines) replaces them, for −152 lines measured:

```cpp
template <std::invocable WriteBack>
class restore_guard {
    WriteBack _write_back;
    bool _armed = true;
public:
    // a discarded guard would write back at once, before the writes it undoes
    [[nodiscard]] explicit restore_guard(WriteBack write_back);
    ~restore_guard();   // writes back if still armed, swallowing an error
    void restore();     // writes back once, and reports the error
};
```

Rules from the verification:

- **Arming.** Arm the guard before the writes when there are several, so that
  the constructors' rollbacks disappear. Arm it after the write when there is
  only one.
- **Callbacks.** A callback guard is constructed after a successful detach,
  since Xpress `addcbpreintsol` would register the callback a second time.
- **Gurobi.** Gurobi records an array as saved only after its force
  succeeded: an attribute write discards the held solution
  (gurobi_base.hpp:679-683), so writing back everything saved is not neutral.
- **Write-backs are not free.** The comment "values read back moments ago" is
  wrong on two backends. A HiGHS option write runs `optionChangeAction`, and
  a Gurobi attribute write discards the solution.

The same guard fixes a non-IIS bug: `refine_lp_status` on `gurobi_lp`
(gurobi_lp.hpp:101-105) and `cplex_lp` (cplex_lp.hpp:121-128) leaves its
parameters changed when `optimize` or `primopt` throws.

**`detail::iis_answer`.** It holds the two tables, has `flag_variable` and
`flag_constraint` (which skip an entity flagged on neither side), and three
verbs: `feasible()`, `undetermined(stop)` and `found(minimal, stop)`. It
replaces 36 exit points across the five native bases, for −21 net as its own
header and about −35 placed in iis_snapshot.hpp's detail section. It fixes no
present bug: all 36 exits were correct. Writing the stop explicitly at every
exit surfaced one gap, on the HiGHS no-member path (below).

**`iis_status::sides_of`.** The docs, the example and the tests carry several
hand-written decoders that read plain `member` differently:

| Decoder | Location | Reading of plain `member` |
| :--- | :--- | :--- |
| `print_member` | infeasibility.cpp:49-50 | prints it like `member_both` |
| `side_of` | infeasibility.cpp:169 | "side not named" |
| `sides_named` | infeasibility.cpp:201 | both sides |
| `needed_sides` | main.cpp:47 | no side, which would relax nothing if reused for repair |
| `membership_of` | iis_oracle.hpp:144-165 | the entity as one unit |

The contract pinned on every path (iis_oracle.hpp:187-190,
iis_cases.hpp:546-558) is the last one: the entity's finite sides as one unit.
One library reader, shaped like `status::solution_available`, compiles on every
variant:

```cpp
struct iis_sides { bool lower = false; bool upper = false; bool whole = false; };
namespace iis_status {
template <lp_iis_status S>
[[nodiscard]] constexpr iis_sides sides_of(const S & s) noexcept;
// member_both {1,1,0}; member_lower {1,0,0}; member_upper {0,1,0};
// member {1,1,1}; absent {0,0,0}
}
```

N1 refinement tags inherit the right answer, since the reader decides by
`derived_from`. The oracle's `membership_of` must stay independent of it.
Optionally, the two second-side merges (iis_by_deletion.hpp:431-441,
cplex_base.hpp:788-797) fold through it for −8 lines.

**Narrowing overload.** `compute_iis_by_deletion(model, within, limits)` takes
as candidates the sides that an earlier answer, from the filter or a native
routine, names; every other side stays relaxed. It is sound: an IIS of an
infeasible subsystem is an IIS of the system relative to the same background,
because the guard computes the same feasibility function for any active set.
What it gives:

- **It replaces the docs' 57-line `rerun_on_members`**
  (infeasibility.cpp:271-327), which has three defects:
  - its relax loops run outside the `try`;
  - its restore stops at the first throwing setter;
  - it returns `feasible` when the kept sides hold together, a false claim
    about the user's model.
- **It refines native whole rows.** It turns the plain-member `==` rows of
  Gurobi and CPLEX into sides, in one solve per named side plus one.

One detail runner serves both overloads, with three rules:

- the N36 crossed-pair start restricted to the seed;
- the N6 column-less precheck over the selected sides;
- a seeded run that finds its sides feasible answers undetermined, never
  feasible.

The first trial cannot be skipped for a seed that comes from the filter.
`model_iis_t<highs_lp>` is the same type as `iis_by_deletion_t<highs_lp>`, so
neither the type nor the outcome tells a proven subset from HiGHS's "maybe".

**`handle_status_table` over `std::vector<Status>`.** It removes the uint8
codec: `is_tag_variant_v`, `_make_status` and `_make_derived_mask`. The
measured costs:

- one byte per id more, since `sizeof` is 2 for every IIS status variant
  (+10 MB per 10 million ids);
- construction about 1.5 times slower;
- lookups faster or slower depending on the tag, under 0.5 ns either way.

It is a readability change only, and not a door for tags that carry data (see
[Rejected](#rejected)). Requirements:

- a separate non-explicit defaulted default constructor, because clang-18
  rejects `{}` against an explicit default constructor and the build uses
  `-Werror`;
- `count_a` constrained on `variant_containing_a`;
- `get()` noexcept only when copying is noexcept.

**Small cleanups.**
- Answer the column-less case through the enumeration and the shared fold
  (iis_by_deletion.hpp:471-490), for −6. An equivalence probe found 0
  differences in 1352 cases. Add a test pinning that the lower side wins when
  0 violates both sides.
- Delete the unused `iis_deletion_guard::solved()` (iis_by_deletion.hpp:320).
- Store the saved time limit as a plain `std::optional<seconds>` instead of
  `iis_deletion_time_limit_slot`, for −11.
- Drop the unused `has_enumerable_constraints` requirement of
  `iis_column_less_precheck` (iis_arithmetic.hpp:32).
- Share the Xpress and COPT self-infeasible-column scan in iis_arithmetic.hpp,
  for −5 and a solver-free test. The two backends already use the same type
  codes.
- **Visibility.**
  - Move `iis_sided_status` into `detail` beside its two siblings: all 19
    solve-status variants are private, and none of the three IIS aliases is
    documented.
  - Move `iis_limits` into deletion_filter.hpp. Every model header then stops
    including `<stop_token>`, 3,518 preprocessed lines fewer for
    model_concepts.hpp.
  - Document `iis_by_deletion_t<M>` next to `model_iis_t<M>`.

## Backend data and correctness

- **Xpress clears its IIS data.** `XPRSiisclear` runs after a successful search
  (xpress_base.hpp:733-734), although `XPRSiisfirst` already clears the
  previous one (optimizerC.md:13324). After the clear, `XPRSgetiisdata`
  (duals and djs, the Farkas multipliers of an LP IIS), `XPRSiiswrite`,
  `XPRSiisisolations` and `XPRSiisnext` are unavailable through
  `native_model()`. Recommended:
  - delete the call and its binding (−4);
  - document that IISOPS is restored before the call returns, so a native
    `XPRSiisnext` searches under the caller's IISOPS, not under the background
    of the first IIS;
  - pin the result with one test.
- **COPT minimality relative to special constraints.** COPT's routine treats
  native SOS and indicator constraints as candidates. The wrapper reads their
  IIS counts only, and returns `irreducible` on `IsMinIIS`
  (copt_base.hpp:704-710, 790). When the IIS names only some of the special
  constraints, the answer may not be minimal relative to the background. A
  counterexample, with z fixed to 1:
  - I1: z = 1 → x ≥ 5;
  - I2: z = 1 → x ≤ 2;
  - r: x ≤ 3.

  COPT may return {r, I1, z.lb}, which the wrapper reports as `r` irreducible
  while the background alone is infeasible. Recommended (+7): report
  `irreducible` only when every special constraint is named or no linear
  member is flagged. Add one sentence to "Native IIS on COPT", and a COPT line
  to iis.md:141-146, which lists how every other wrapper keeps special
  constraints out. It is reachable only through native handles (N23).
- **HiGHS decode.** `iis_flagged_status(bound != Upper, bound != Lower)`
  (highs_base.hpp:871-879) decodes the Dropped (−1) and Null (0) bound codes,
  and any unknown code, as `member_both`: measured by running the decode.
  Make Lower, Upper and Boxed explicit (+4), so that the other codes make no
  member.
- **HiGHS no-member path.** A kOk answer with no member and a model status
  other than optimal or unbounded returned undetermined with no reason, even
  when the clock had reached the limit (highs_base.hpp:893-900). The new
  completion report applies the same time attribution there.
- **Kept on purpose.** The wrappers drop no side that a routine names:
  - HiGHS bound codes, Gurobi IISLB/IISUB, Xpress L/G/E/F entries and COPT's
    four flag arrays survive;
  - plain `member` keeps exactly what a row-unit routine says;
  - integrality and special constraints are background by design;
  - Gurobi's IIS attributes on SOS, quadratic and general constraints would
    read the forced 1s and carry nothing;
  - Xpress isolations are never computed, an expensive separate call.

## Documentation errors

- **"Each path returns a snapshot of a type of its own"** (infeasibility.md:49,
  58, 74; main.cpp:145) is false. Across all 29 (model, path) pairs there are
  only 4 distinct answer types, measured: the deletion type equals the native
  one on `highs_lp`, `highs_qp`, `xpress_lp` and `copt_lp`. The real reason
  `print_member` must be a template is another: inside one answer,
  `get_status(v)` and `get_status(c)` return different variants on `gurobi_*`,
  `cplex_*` and `xpress_milp`.
- infeasibility.md:58 should name `iis_by_deletion_t<M>` instead of "taken
  through `auto`".
- infeasibility.md:74, "generic code picks one at compile time": `diagnose`
  also falls back at run time when `compute_iis()` throws.
- infeasibility.md:100 defines plain `member` as "a member whose side the answer
  does not name". That is weaker than the pinned contract, and read as "some
  side, unknown which" it would describe `copt_milp`'s continuous x in
  [0.5, 1.5] wrongly, since both bounds are needed there (iis.md:360-364).
- infeasibility.md:286 says what a routine offers beyond an IIS "stays
  reachable through the solver's C API". That is not true for Xpress (the
  clear above), nor for Gurobi models with special constraints (the
  force-restore update).
- main.cpp:39, "the tags its path reports", should read "its routine reports".
- **Exception safety.** deletion-filter.md:134 and infeasibility.md:246 claim
  restoration on every exit. A write-back that fails while a trial's exception
  propagates is swallowed, and that item stays as the trial left it.
- **Callbacks (N42).** The filter's guarantee needs a callback whose accept or
  reject decision depends on the candidate point alone. One with memory makes
  the trial oracle non-monotone, and `irreducible` is then unproven.
- **On `highs_qp`.** deletion-filter.md:136's advice to save an objective
  through `materialize(model.get_objective())` loses the Hessian. Copy the
  terms of `get_quadratic_objective()` too, and restore them through
  `set_quadratic_objective`.
- **Feature tables.** README.md:133, docs/index.md:83 and
  deletion-filter.md:217 point to the feature tables for IIS support, but the
  generated tables have no IIS, enumeration or row-bound rows. Regenerate them
  from a full CI run before the release that carries IIS.
- **Native settings.** Each "Native IIS on X" note should say which native
  routine settings apply. Gurobi IISMethod, CPLEX Conflict.Algorithm and COPT
  IISMethod set through `native_api()` reach `compute_iis()`. HiGHS
  `iis_strategy` and Xpress IISOPS are overwritten for each call.

## Rejected

| Proposal | Why it was rejected |
| :--- | :--- |
| Mappings (`iis.variables[x]`) instead of `get_status` | Loses the overload on the handle type that later kinds attach through, and touches about 130 call sites for no gain in data |
| One `compute_iis` dispatching native-or-filter | Against Q1 (b), and the paths differ: background callbacks, the unit of a row |
| The filter taking the model's time limit as its whole budget | Conflates the run budget with the per-trial cap it forwards, and leaves `max_solves` and `stop_token` without a home |
| Member lists `variable_members()` / `constraint_members()` | Same length as looping over `model.variables()`, +33 library lines; the snapshot would make handles the user never held, which a later removal can recycle (N8). The basis has no member list either |
| `member_both` deriving from `member_lower` and `member_upper` | Virtual bases make the tags non-empty and not `constexpr`; a non-virtual diamond makes `derived_from<member_both, member>` false, so `is_a<member>` silently misses it. Plain `member` would still need a decoder |
| `possible_*` refinement tags | No routine is known to report per-member possible flags on a complete answer; catch-all visitors, which status-and-limits.md:78 recommends, would route the new tags away from their base overload |
| Tags carrying data (Xpress multipliers per member) | The library keeps numbers apart from categories, as the basis keeps duals out of `basis_status`; payloads are reachable only by `std::get` on a backend type, and every entry grows to 16 bytes. If the multipliers are wanted, they belong in an accessor |
| Refreshing the status after a native call instead of resetting it | A Gurobi stop turns a proven `infeasible` into `time_limit`; COPT's `COPT_Reset` drops the status anyway; HiGHS 1.10 sets kNotset where 1.15.1 reports optimal. Keep the reset, and close N15's "leave untouched" follow-up |
| A per-side role callback (candidate, background, relaxed) | More general than the one need on record, which the `within` overload covers with no new name; it would ship N37 (f)'s protected sides early |
| An order-preserving single pass | Changes the answer on 4 of 23 simulated systems, a published page output among them, and 4 stub tests; the order semantics belong to N37 (f) |
| Dropping COPT's unused `GetSOSIIS` and `GetIndicatorIIS` bindings | Both exist in every COPT that has `ComputeIIS`, and they are the user's only way to read special-constraint membership through `native_api()` |
| The answer builder's shared stopwatch | −3 to +4 net: each backend's attribution differs (Xpress's `stopped` precondition, COPT's clock spanning its pre-solve) |

## Rulings the review would keep

- **Dormant batching (N3 b, N37 e).** The fact used to argue for its removal,
  that no public entry reaches it, was known when both rulings were made, and
  N37 (e) confirmed it in a simplification review. If removal is still wanted,
  the gain is −27 header lines and −96 test lines, and the code stays at
  `b9d2834` and `archive/pr3-a1a9f11`.
- **The guard's `Clock` parameter.** Removing it saves no line, and it is the
  only seam for testing the expired-deadline branch (iis_by_deletion.hpp:336-341),
  which has no test. Add the test, with a fake clock.
- **`num_variable_members()` and `num_constraint_members()`.** They are cheap,
  immutable and required by `lp_iis`.
- **The snapshot's requires clause.** Each conjunct is load-bearing: distinct
  handle types, `lp_iis_status` on both variants, and `absent` first.
- **`iis_by_deletion_model` with the sense/rhs arm (N41, N43).** No
  simplification beats the reasons recorded for them.

## Findings outside IIS

- **A throwing `solve()` keeps the previous status.** 17 of the 19 `solve()`
  implementations assign `_status` only after the native call succeeds, so
  `get_status()` can report an earlier `optimal` while the solver holds a
  failed run. Only `mosek_lp` and `mosek_milp` reset first.
  `compute_iis()` and the filter's guard already follow the safer rule.
- **`has_time_limit` asks for too little.** It checks
  `set_time_limit(std::chrono::seconds)` (model_concepts.hpp:306-310), while the
  filter writes a `duration<double>` (iis_by_deletion.hpp:344-345). A user model
  whose setter takes only seconds satisfies the concept and then fails inside
  the template.
- **Gurobi error messages.** Gurobi's `_check` takes its message from the env's
  last error (gurobi_api.hpp:342-346), so an "attempt every item" write-back can
  raise the first failure's code with a later failure's message.
- **`highs_lp` lists `status::solution_limit`** but never returns it
  (highs_lp.hpp:52, 58-89).
- **`candidate_solution_callback_handle_t` reads a public nested type**
  (model_concepts.hpp:729-731), contrary to concepts.md:25-26, which says
  model classes declare no public member types.
- **Cancellation is per call on the filter only** (`iis_limits::stop_token`),
  while configuration is otherwise persistent model state. When N31 is taken
  up, a model capability built like the time limit would serve `solve()`,
  `compute_iis()` and the filter's trials alike.

## Pending decisions

1. `detail::restore_guard`, with the `refine_lp_status` fix as its own commit.
2. The `detail::iis_answer` builder.
3. `iis_status::sides_of` and `iis_sides`.
4. The narrowing overload `compute_iis_by_deletion(model, within, limits)`.
   It ships an N37 (f) mechanism in the first version, so it needs a ruling on
   timing.
5. `handle_status_table` over `std::vector<Status>`.
6. The small cleanups and the visibility moves.
7. The Xpress `iisclear` removal and the COPT special-constraint downgrade.
8. The HiGHS explicit decode.
9. The documentation corrections.
10. Library-wide: reset `_status` first in every `solve()`, or document that a
    throwing solve keeps the previous status.
11. Optional, and low priority: rename `solve_limit` / `max_solves` to
    `trial_limit` / `max_trials`, since the engine's unit is a trial and the
    engine is solver-agnostic.
