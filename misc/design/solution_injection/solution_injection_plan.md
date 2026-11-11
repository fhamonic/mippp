# Repairing a rejected candidate (Benders): assessment and plan

On 2026-11-11 the maintainer reported a limit of the candidate-solution callback. In a
Benders decomposition, a lazy constraint added in the callback rejects the candidate, although
its integer part may be feasible: only the value of the epigraph variable was too optimistic.
Solution injection was suggested as a cure, with a cleaner design welcome, breaking changes
included.

**Status: ruled and implemented.** On 2026-11-11 the maintainer accepted the recommended
option of every decision below, and the feature is on the branch `feat/solution-injection`:
`add_solution` on the candidate handles of Gurobi, CPLEX and COPT, and on their
node-relaxation handles, with the suite `SolutionInjectionTest`. Run through the library, the
probes gave the native results again: injected points come back to the callback on Gurobi and
CPLEX, with a removed variable too, and COPT takes them unchecked.

The probes are in [survey/](survey/), described in [survey/RECORD.md](survey/RECORD.md). They
ran against `include/` of `main` (`080045d`), unpatched, with the native injection call made
from inside MIP++'s own candidate callback.

## Short answer

- **The limit is real, and it is mostly Gurobi's.** On a Benders master with one epigraph
  variable, Gurobi rejected 18 to 45 candidates per run whose integer part was better than
  its incumbent, and later accepted 9 to 15 integer parts it had rejected. CPLEX offers the
  rejected integer part again in the very next call, by itself. On COPT nearly every accepted
  candidate had been rejected before.
- **Injection cures it on Gurobi.** Handing back the candidate with its epigraph value
  corrected brought an incumbent within 1% of the best known 2 to 12 times sooner in 4 of 5
  seeds, and a better incumbent at the time limit in 2 of 5.
- **No solver offers anything cleaner than injection.** None can accept a candidate "with
  this value changed", and letting the solver complete the integer part fails (see
  [Alternatives](#alternatives-that-fail)). The clean part is the interface: read the
  candidate, correct it, hand it back.
- **Proposal:** `handle.add_solution(point)` on the candidate handle of Gurobi, CPLEX and
  COPT, with `get_solution()` returning a point that owns its values and can be written.
  That is three lines in a Benders callback, and the only breaking change is the return type
  of `get_solution()`.
- **The contract has one sharp edge.** COPT makes an injected point the incumbent without
  showing it to the candidate callback, and so does CPLEX under one of its strategies. A
  point the callback would have rejected then comes back as the optimum. So the point must be
  feasible for the whole problem, and the caller answers for it.

## The limit, measured

The instance is a facility location master: binaries `y` under a budget, one epigraph
variable for the assignment cost, and an optimality cut added as a lazy constraint whenever
the candidate's epigraph value is below the true cost of its `y`. The cut is tight at that
`y`, so the candidate with the true cost is always feasible. 60 facilities, 200 customers, 5
seeds, one thread, a 30 s limit. Gurobi 12.0.1, CPLEX 22.1.2, COPT 8.0.5.

Lazy constraints alone, per run:

| Backend | Candidates rejected | Of those, truly better than the incumbent | Accepted candidates | Accepted ones that had been rejected earlier | Accepted in the call right after their rejection |
|---|---|---|---|---|---|
| Gurobi | 173 to 880 | 18 to 45 | 10 to 19 | 9 to 15 | 3 to 6 |
| CPLEX | 286 to 677 | 16 to 21 | 9 to 14 | 9 to 14 | 9 to 14 |
| COPT | 297 to 494 | 21 to 42 | 23 to 33 | 22 to 32 | 8 to 18 |

- **CPLEX:** every accepted candidate was the integer part rejected in the call before, with
  its epigraph value corrected, in 4 of 5 runs, and 9 of 10 were in the fifth. The mechanism
  is not established; the effect is that CPLEX needs no help here.
- **Gurobi:** most of the truly better candidates are lost. On seed 3 an integer part as good
  as the final incumbent was first rejected at 10.8 s and accepted at 22.5 s.

With the corrected candidate handed back (`GRBcbsolution` from the `MIPSOL` callback), on
Gurobi 12.0.1:

| Seed | Seconds to an incumbent within 1% of the best known: lazy only | With injection | Incumbent at 30 s: lazy only | With injection |
|---|---|---|---|---|
| 1 | 0.26 | 0.47 | 2682, optimal in 10.9 s | 2682, optimal in 9.4 s |
| 2 | 13.12 | 1.05 | 2786 | 2786 |
| 3 | 22.45 | 10.21 | 2732 | 2732 |
| 4 | 0.42 | 0.07 | 2719 | 2710 |
| 5 | 0.99 | 0.13 | 2788 | 2772 |

2710 and 2772 are the optima CPLEX proves. One-thread Gurobi runs repeat exactly, so each
cell is one run. On the same seed 2, 11.0.3 went from 12.93 s to 0.66 s, and 13.0.2 from 0.57 s
to 0.55 s.

- **CPLEX:** no measurable effect. Times to optimality without and with: 8.66 and 8.68,
  24.40 and 25.15, 25.44 and 25.48, 28.24 and 27.86, 20.35 and 21.72 s.
- **COPT:** mixed at one thread. The incumbent at 30 s was the same in 3 runs, better in 1
  (2772 for 2800) and worse in 1 (2799 for 2732).
- **Four threads,** 3 seeds, one run per cell, not repeatable:
  - Gurobi reached 1% in 1.01, 0.25 and 0.04 s, against 0.74, 2.16 and 13.43 s;
  - CPLEX showed no clear change (0.23, 0.32 and 1.48 s against 0.37, 0.05 and 1.65 s);
  - COPT reached it in 0.58, 1.15 and 1.57 s, against 8.32, 7.26 and 3.78 s, and ended one
    run on 2732 for 2773.

A master with one epigraph variable per customer closed in 0.1 to 1.1 s on Gurobi, too fast
to show anything. It did not run on the other two.

## What injection does on each solver

| | Gurobi 11.0.3, 12.0.1, 13.0.2 | CPLEX 22.1.2 | COPT 8.0.5 |
|---|---|---|---|
| Call, from the candidate context | `GRBcbsolution`, return code 0 | `CPXcallbackpostheursoln`, return code 0 | `COPT_AddCallbackSolution`, return code 0 |
| When the point is taken | later: stored, per the manual | later: queued until heuristics next run, per the manual | at once: the call returns its objective, or 1e30 if it does not improve |
| Shown to the candidate callback before it becomes the incumbent | yes | `CHECKFEAS`: yes. `NOCHECK`: no | no |
| A point the callback would reject | rejected, right optimum | `CHECKFEAS`: rejected, no wrong value. `NOCHECK`: wrong optimum | wrong optimum |
| A partial point (integer part only) | completed without the lazy constraint just added, offered, rejected again | no gain | ignored |

The evidence for the two middle rows is a run where every rejected candidate is answered
with an injected neighbour given half its true value:

- **Gurobi:** 373 of 465 and 629 of 786 such points came back to the callback on 12.0.1, 313
  of 397 on 11.0.3, 446 of 555 on 13.0.2. All were rejected there, and the optimum was right.
- **CPLEX, `CHECKFEAS`:** 1029 of 1305 and 2203 of 2759 came back and were rejected. The
  first run proved the right optimum, and the second held the right incumbent at its 15 s
  limit.
- **CPLEX, `NOCHECK`:** `status::optimal` at 2001.12 and 1759.12, for a true 2682 and 2786.
- **COPT:** `status::optimal` at 1815.12 and 1872.12, for the same two. On a model with one
  integer, an injected `x = 8` came back as the optimum 7.75, for a true 6. COPT's guide says
  it "will check any provided solution for feasibility": that check knows the rows COPT
  holds, not the ones the callback has yet to add.

The partial row:

- **Gurobi:** in 5 of 5 runs, every partial point was followed by one more rejection of the
  same integer part, its epigraph value still too low (33 of 33, 34 of 34, 26 of 26, 22 of
  22, 23 of 23). With complete points this never happened (0 in 5 runs).
- **COPT:** the 5 runs had the same counts of calls, rejections and acceptances as the runs
  without injection. Its guide says it "only supports complete feasible solutions within
  callbacks".

Three more facts:

- **The callback runs twice on a repaired candidate** on Gurobi and CPLEX: once on the
  candidate, once on the point handed back. A costly subproblem is solved twice unless the
  caller remembers its last answer.
- **A truthful point is often not used,** and that is fine. Of the 19 to 30 integer parts
  handed back per run, Gurobi later accepted 11 to 16. A point that no longer improves the
  incumbent when its turn comes is dropped.
- **Gurobi can drop a feasible point silently.** A point that set a free binary column with a
  positive cost to 1 had no effect in 20 of 20 injections: the search was the same, node for
  node, as without injection. Presolve fixing that column is the suspect, not verified.

## Proposed interface

The candidate is read, corrected and handed back:

```cpp
model.set_candidate_solution_callback([&](auto & handle) {
    auto x = handle.get_solution();          // owns its values, writable
    bool repaired = false;
    for(const auto & [var, data] : subproblems) {
        auto [opt, duals] = solve_subproblem(data, x);
        if(x[var] - opt <= tolerance * opt) continue;
        handle.add_lazy_constraint(var <= cut(data, duals));
        x[var] = opt;                        // what this integer part is worth
        repaired = true;
    }
    if(repaired) handle.add_solution(x);
});
```

Against gecot's `target_benders.hpp:245-263` that is three added lines.

```cpp
// On a callback handle. The point gives a value to every variable and must satisfy
// the whole problem: bounds, integrality, rows, and every constraint the
// candidate-solution callback would add. The solver may make it the incumbent
// without showing it to that callback, so a point the callback would reject is
// undefined behavior: it can come back as the optimum at status::optimal. The
// solver uses the point at once, later or never, and drops one that does not
// improve the incumbent. Where it does show the point to the callback, the
// callback runs once more, on that point.
template <typename T, typename M = T>
concept has_solution_injection = requires(T & handle) {
    { handle.add_solution(/* an input mapping from model_variable_t<M>
                             to model_scalar_t<M> */) };
};
```

Notes on the shape:

- **A complete point, as a mapping.** A mapping is total by construction, which is what COPT
  requires and what the Gurobi result calls for. A range of pairs, as `add_mip_start` takes,
  would invite partial points.
- **`get_solution()` returns a point that owns its values and can be written.** COPT and
  Xpress already return one: `entity_mapping` over an owned array hands out references
  (`model_entities.hpp:125-131`). Gurobi and CPLEX return a lambda that captures the handle
  (`gurobi_milp.hpp:167-170`, `cplex_milp.hpp:187-190`), which is defect D6 of the user-cut
  plan. One owning type with the native-id step inside closes D6 and makes the repair an
  assignment. This is the only breaking change: code that names the return type breaks, code
  that reads through `auto` does not.
- **`add_solution` takes any input mapping,** so a heuristic can pass a lambda. The handle's
  own point is the fast path, with no copy.
- **Handle first, model second,** per the ruling of 2026-09-16. The node-relaxation handle of
  the user-cut plan takes the same member when it lands, for rounding heuristics.
- **CPLEX always posts with `CHECKFEAS`.** That call wants the objective value of the point,
  so `solve()` reads the objective once before `CPXmipopt`, as the user-cut plan already has
  it do for the column count. *What CPLEX does with a wrong value there is not measured.*
- **No return value.** Only COPT knows at once whether the point was taken.
- **Xpress, SCIP and HiGHS have no lazy constraints in MIP++,** so there is no Benders
  callback to repair there yet. `XPRSaddmipsol` is already in `xpress_api.hpp:200`.

## Alternatives that fail

- **Let the wrapper keep the integer part alive,** by handing it back as a partial point
  whenever a lazy constraint is added. Gurobi completes it without the new constraint and
  the same wrong candidate returns (5 of 5 runs, every point). COPT ignores it. It would also
  fire on subtour cuts, where the integer part is the infeasible one.
- **Accept the candidate with a corrected value.** No solver has such a call: Xpress's
  `preintsol` can change the cutoff only, and SCIP's own Benders code builds a new solution
  with the auxiliary variables raised and submits it (`cons_benders.c`,
  `constructValidSolution`). The wrapper would emulate it with the same injection, under a name that
  promises more than it does.
- **Do nothing and lean on the solver.** Enough on CPLEX. Not on Gurobi.
- **A Benders driver in `mippp/utility/`.** It would sit on this member, so it can come
  later without breaking anything, if a second consumer appears.

## What it takes

About 300 lines:

| Part | Lines |
|---|---|
| Owning, writable candidate point for Gurobi and CPLEX | 40 |
| `add_solution` on Gurobi (`cbsolution` is already in `gurobi_api.hpp:302`) | 25 |
| `add_solution` on CPLEX: `CPXcallbackpostheursoln` added to the table, objective read in `solve()` | 50 |
| `add_solution` on COPT (`AddCallbackSolution` is already in `copt_api.hpp:321`) | 20 |
| Concept and shared typed suite | 100 |
| Docs: a Benders section in `branch-and-cut.md`, `concepts.md`, the README roadmap row | 60 |

This is an estimate. What a test can assert:

- the optimum is right with the repair on, checked against the compact model;
- the callback saw the repaired value accepted, or the final value is the truth;
- a stored point read after the call (D6);
- nothing about whether or when the solver used the point: the contract leaves it open.

## Decisions for the maintainer

Recommended option first.

1. **Mechanism.** `add_solution(point)` on the candidate handle, behind
   `has_solution_injection<Handle, Model>`? The name could also be `submit_solution`, JuMP's
   verb.
2. **Contract.** The point must be feasible for the whole problem, lazy constraints
   included, and a point the callback would reject is undefined behavior? The alternative is
   a guard: on COPT the wrapper runs the user's candidate callback on the point before
   handing it over. That is a callback entered from inside itself.
3. **Complete points only?** The alternative, partial points, is measured to fail on Gurobi
   and COPT.
4. **`get_solution()` returns an owning, writable point on every backend?** It closes D6. The
   alternative keeps the type and adds an overload that takes the candidate plus a list of
   corrections.
5. **Backends.** Gurobi, CPLEX and COPT together, so generic code needs no branch? COPT's
   gain is unclear and its point is unchecked; the contract covers that. The alternative is
   Gurobi alone.
6. **Order.** Before the user cuts: it is small, it needs none of their rulings, and it
   removes a limit met in practice. It shares D6 with them, and D7 matters to both: on CPLEX
   the candidate callback runs concurrently today, with or without injection.

## Not verified

- **gecot's own masters.** The effect is measured on a synthetic master with one epigraph
  variable, not on gecot's, which have one per case and target.
- **Releases and platforms.** Gurobi 11.0.3 and 13.0.2 ran one seed each. Gurobi 10, COPT 7.2,
  MSVC, Windows and macOS did not run.
- **Mechanisms.** Why CPLEX offers a rejected integer part again. Why Gurobi dropped the 20
  points with the free column. What CPLEX does with a wrong objective value in the posting
  call.
- **Xpress.** `XPRSaddmipsol` from `preintsol` was not run: there is no lazy path to repair.
- **More than one thread.** One run per cell, not repeatable. The probe holds its own mutex
  around the callback, so no injection ran concurrently with another.
- **Vendor manuals.** Gurobi's and CPLEX's were read through a summarizing fetch, not
  verbatim. COPT's guide was read from the local install.
- **A real wrapper.** Every call ran natively from inside MIP++'s callback, reaching the
  handle's private members, not as library code.
