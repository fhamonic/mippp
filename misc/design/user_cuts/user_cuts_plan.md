# User cuts: assessment and plan

On 2026-11-07 the maintainer asked how user-cut callbacks should be implemented,
noting that backends seem to give "user cut" different meanings: globally valid, or
valid only in the current branch of the search tree.

**Status: ruled, partly implemented.** On 2026-11-11 the maintainer accepted the recommended
option of each of the nine decisions. On the branch `feat/solution-injection`:

- **Done:** the shared preparation (`detail/row_preparation.hpp`); D3, D4, D5, D6, D7 and D8;
  D1 and D2 in the docs; the node-relaxation callback and `add_user_cut` on Gurobi, CPLEX and
  COPT, with the suites `NodeRelaxationCallbackTest` and `UserCutsTest`; the deletion filter
  takes the callback away for its trials.
- **Not done:** Xpress with its validated floor, SCIP, GLPK, and D9 to D14 apart from the
  candidate handle's share of D9.
- **One departure from the letter of D5's fix.** On the lazy path a row is dropped when the
  candidate is within the solver's feasibility tolerance of it, measured on the row as
  written, not on the scaled row: scaled, a Benders cut with a large coefficient is dropped
  at a violation that matters. Gurobi's lazy path is left as it is, since Gurobi accepts a
  satisfied row by itself.

The text below is the proposal as it was ruled on. The
evidence is in [survey/](survey/), described in [survey/RECORD.md](survey/RECORD.md),
and was gathered in four rounds:

1. One survey agent per backend.
2. One independent skeptic per backend, writing fresh probes to refute the first.
3. A run of the single cut preparation proposed here, on all six backends that would
   ship.
4. Review rounds on this text: a fact check, an API attack and a gap search.

What remains unverified is listed at the end.

## Short answer

- **The worry is right about the solvers.** The plain "add a cut" call means something
  different on nearly every solver, and the same cut can give a right or a wrong optimum
  depending on which one runs it.
- **It is a difference of enforcement, not of what can be promised.** Two questions got
  mixed. What the *user promises* about a cut is the contract, and it is ours to define.
  Where the *solver applies* it is the solver's business. A cut valid for every integer
  feasible point can be applied everywhere, in one subtree, or nowhere, and the answer
  stays right. So one contract fits all of them.
- **The reverse is not portable.** A cut valid only in a subtree, given to Gurobi or COPT,
  can silently lose the optimum at `status::optimal` (COPT: 21 of 24 runs at true
  defaults). They have no flag for it, and their callbacks cannot even read the node's
  bounds to justify one.
- **Proposal:** `handle.add_user_cut(c)` promises global validity, on Gurobi, CPLEX,
  Xpress, COPT and SCIP, then GLPK. Cbc stays out, and HiGHS and MOSEK have no such call.
  Local cuts are a later, separate concept if someone needs them.
- **The contract alone is not safe.** Truly valid cuts with large coefficients lose the
  optimum on COPT, SCIP and GLPK, a valid equality segfaults Xpress, and duplicate columns
  corrupt the heap on Gurobi. So the handle prepares every cut in shared code, which ran as
  one step on all six backends with no failure (see [Preparing a cut](#preparing-a-cut)).
- **The probes also found defects in shipped code**, unrelated to any new feature. Five of
  them give wrong answers today: see [Defects found in shipped
  code](#defects-found-in-shipped-code).

## What "add a cut" means on each solver

The decisive experiment uses a tree that splits on one binary, with the optimum on one
side. Once, in the worse subtree, add a cut valid *there* and invalid elsewhere (S1), then
separately a globally valid hull cut (S2). Global semantics lose the optimum (value = opt -
1). Local semantics keep it. Each later node is checked against the cut.

It is the same design on every backend with a hook, adapted where the harness instance had
no power:
- Cbc ran its own S1i.
- COPT ran S1b with root cuts off.
- SCIP needed breadth-first selection.
- Xpress ran two front modes.

HiGHS and MOSEK have no hook to run it on. The instance is in `cut_common.hpp`. A first
version let Gurobi close the tree at the root by probing, so the hook never ran.

| Backend | Native call | Meaning of the plain call | S1: subtree-only cut | Local cut | Node's bounds readable |
|---|---|---|---|---|---|
| Gurobi | `GRBcbcut` | global, no flag | lost (106.4 of 107.4), on 11, 12, 13 | no | no |
| CPLEX | `CPXcallbackaddusercuts` | a flag per cut, mandatory | `local=0` lost, `local=1` kept | `local=1` | yes |
| Xpress | `XPRSaddcuts` | local to the node's subtree | kept | yes | yes, presolved space |
| Xpress | `XPRSaddmanagedcuts(g)` | `g=1`: "might be used on any node" | `g=1` lost once a round runs at the other node, `g=0` kept | `g=0` | yes, presolved space |
| COPT | `COPT_AddCallbackUserCut` | no flag and no vendor wording; global validity assumed | lost (21 of 24 at true defaults); no setting protects it | no | no |
| SCIP | `SCIPcreateEmptyRowSepa(local)` | a flag per row; `FALSE` is a claim of validity, not global enforcement | `TRUE` kept; `FALSE` depends on the row's age at the node jump, and lost at default selection on larger trees | `local=TRUE` | yes |
| GLPK | `glp_ios_add_row` | always local to the subproblem, no global call | kept | every cut | yes |
| Cbc | `OsiCuts_addRowCut` | may cut only in the subtree, not enforced there (violated in up to 21% of its subtree's calls); at the root that is everything | below the root kept, at the root lost | every cut | bounds only |
| HiGHS, MOSEK | none | | | | |

Three refinements matter:

- **"Global" does not mean enforced everywhere.**
  - SCIP: a `local=FALSE` row reaches another subtree only while its age is 0, unless it is
    also in the pool.
  - Xpress: a global managed cut loads only where a cut round runs.
  - Gurobi and CPLEX: the cut is a pool entry the solver may use, purge or filter. COPT
    behaves alike, but its pool is inferred from traces.
- **"Local" at the root is global, by definition.** SCIP rewrites a root row to global,
  Cbc's "two meanings" are this one meaning, and GLPK's root cuts reach every node.
- **A user cut is not guaranteed to bind incumbents.** On CPLEX, Gurobi, COPT and Cbc the
  returned point can violate an accepted cut, with `status::optimal` (on COPT in 1 of 9
  runs; in the other 8 the cut bound). A user cut must never carry feasibility. Lazy
  constraints do that.

## Proposed interface

`has_node_relaxation_callback` and its handle alias already exist in
`model_concepts.hpp:826-834`, with no backend behind them. `has_user_cuts` and
`add_user_cut` are new.
- **The node-relaxation concept** grows by the same reads as the candidate handle.
- **`has_user_cuts`** takes the `has_lazy_constraints<T, M>` shape: handle first, model
  second, per the ruling of 2026-09-16.

An API attack compiled real generic code against these definitions at `-fsyntax-only`:
- a DFJ branch-and-cut with lazy and user cuts sharing one separator;
- a Benders cut;
- a cover separator with shared state.

All of it compiled, and the expected negatives were rejected.

```cpp
template <typename T>
using node_relaxation_callback_handle_t =
    typename T::node_relaxation_callback_handle;

// Called during solve() on a node's LP relaxation, possibly on a solver's worker
// thread, never at the same time as another call of any callback of this model.
// Whether it runs at a given node is the solver's: it may never run, an integer
// point may never reach it, and on a model without integer variables it may never
// run, which is fine. get_solution_value() is the objective at the point
// get_solution() returns, offset included; that point stays readable after the
// call, until the next change of the model. An exception leaving the callback is
// undefined behavior.
template <typename T>
concept has_node_relaxation_callback =
    requires(T & model, node_relaxation_callback_handle_t<T> & handle) {
        { model.set_node_relaxation_callback(
                [](node_relaxation_callback_handle_t<T> &) {}) };
        { handle.get_solution_value() } -> std::same_as<model_scalar_t<T>>;
        { handle.get_solution() }
                -> input_mapping_of<model_variable_t<T>, model_scalar_t<T>>;
    };

// c must hold at every point solve() may return as feasible: integer, within the
// model's rows and bounds, and accepted by the candidate-solution callback. The
// solver may apply it anywhere or nowhere, and may drop it later: a later point, at
// this node or another, can violate it, so offer a cut whenever it is violated
// rather than remembering which cuts were added. add_user_cut does not throw: a cut
// the solver refuses is dropped. A cut that removes such a point is undefined
// behavior, even one that keeps an optimum: it can give a wrong optimum at
// status::optimal, or status::infeasible on a feasible model. So is a NaN or an
// infinite coefficient or right-hand side.
template <typename T, typename M = T>
concept has_user_cuts = requires(T & handle) {
    { handle.add_user_cut(archetype::linear_constraint<M>()) };
    { handle.add_user_cut(distinct_variables,
                          archetype::linear_constraint<M>()) };
};
```

Lazy constraints for feasibility, user cuts for strength. One separator can serve both:

```cpp
auto separate = [&](auto & handle) {
    auto x = handle.get_solution();
    for(const auto & S : violated_subtours(x)) {
        auto cut = xsum(leaving(S), arc_var) >= 1;
        if constexpr(has_user_cuts<std::remove_cvref_t<decltype(handle)>, Model>)
            handle.add_user_cut(cut);
        else
            handle.add_lazy_constraint(cut);
    }
};
model.set_candidate_solution_callback(separate);  // feasibility
model.set_node_relaxation_callback(separate);     // strength
model.solve();
```

With user cuts alone, on the degree-only model of `examples/travelling_salesman_dfj`, the
solvers return subtours at `status::optimal`: 418 with 11 cycles, for a true 549.

Notes on the shape:

- **`add_lazy_constraint` stays off this handle.** No backend takes one from a relaxation
  without further ado:
  - Gurobi needs `LazyConstraints=1` (error 10011 without it), which costs presolve
    reductions as COPT's `MIPSOL` bit does.
  - CPLEX refuses with 1811.
  - On Xpress, a row added from a relaxation hook does not bind heuristic incumbents: lazy
    rows from `optnode` alone gave 418 and 510, both wrong.
- **"Accepted by the candidate-solution callback" is measured, not promised by every
  vendor.** DFJ cuts on a degree-only model, with lazy subtours, reach the right optimum on:
  - Gurobi, for n up to 100;
  - CPLEX, in 42 of 42 runs;
  - COPT;
  - Xpress, through a preintsol recipe the wrapper does not have yet;
  - SCIP, with a constraint handler the wrapper does not have yet.

  Gurobi's manual only names "the original constraints"; CPLEX says "the rest of the model".
- **Dominance cuts are outside the contract on purpose.** On Gurobi with `PreCrush=1`, and
  on SCIP, a cut that removes feasible points but keeps an optimum gave a wrong optimum or a
  false infeasible. This happened on gadgets where presolve fixes the dominated column (20
  of 20 on Gurobi 12.0.1); another gadget stayed right in 24 of 24. Vendors disagree: SCIP's
  FAQ allows such cuts, and CPLEX says to use lazy constraints instead.
- **So are cuts valid only for points better than an incumbent:** an objective cutoff,
  reduced-cost or Lagrangian fixing, no-goods, local branching. They also remove integer
  feasible points. The handle reads no incumbent and MIP++ has no cutoff setter, so these
  go through `native_api()` before the solve, or through a lazy constraint. Whether a
  narrower contract for them is safe is unmeasured.
- **Rejected alternatives,** each with a concrete failure:
  - *One callback object for candidates and relaxations.* It imposes the lazy machinery on
    users who only add cuts: `LazyConstraints=1` on Gurobi, and the `MIPSOL` bit's lost
    reductions on COPT, which took one knapsack from 28860 nodes to 4806.
  - *A scope tag now.* It has no portable meaning on Gurobi or COPT, and
    `add_local_user_cut` can be added later without breaking anything.
- **Out of scope:** `*_lp`, by ruling. `*_qp`: only `highs_qp` exists, and HiGHS takes no
  cut.

## Preparing a cut

The survey's first recipe passed the shared conformance set. The skeptics then broke it
with truly valid cuts, checked against enumeration, a baseline solve, or a second solver:
- at large or tiny coefficients;
- at ordinary scale too: a valid equality on merged columns segfaults Xpress 46+, and a
  valid cut violated by at most `EpRHS * max|a|` spins CPLEX's root (18 of 40).

Each fault below is silent or kills the process, and a control run proved every step
necessary on at least one backend. So `add_user_cut` prepares every cut in shared code, in
this order, before the native call.

| Step | Without it (truly valid cut unless noted) |
|---|---|
| Fold duplicate columns | **Gurobi:** last coefficient wins; past about `NumVars` entries `GRBcbcut` overflows the heap with return code 0. **CPLEX:** last entry wins, wrong optimum 1718 for 1733. **COPT:** SIGSEGV, heap-corruption abort, a hang past the time limit, or a wrong optimum (87.05 for 107.4). **GLPK:** process abort. **Cbc:** `CoinError` through a C entry point. |
| Strip exact zeros | **COPT:** wrong optimum or false infeasible (`+0.0` in `<=`, `-0.0` in `>=`; either sign in `==`, measured on lazy rows). **GLPK 4.59 to 4.63:** abort. `x - x` folds to a zero, so this comes after folding. |
| Skip what is left empty | **COPT:** returns 3, and `check()` would throw through its frames. |
| Scale to `max\|a\| = 1`, then strip zeros again | Optimum lost without it: **COPT** at 1e12 on covers (5 of 5) and at 1e10 on its skeptic's network model. **SCIP** from 1e9 (10 to 13 of 40). **GLPK** from 1e9 on covers and DFJ (13 of 20, with a SIGABRT), from about 2e6 coefficients on its skeptic's families, and below 1e-7. **Gurobi** ignores 1e12 cuts unscaled (5510 nodes against 1 on ATSP n=16). With scaling every backend's search is nearly independent of the cut's scale, and CPLEX, Xpress and COPT explore the same nodes as unscaled. The second zero strip only matters past a 1e308 span. |
| Split `==` into `<=` and `>=`, then drop each part unless violated by more than 1e-6 | **Xpress:** a valid equality with unequal coefficients on columns presolve merged segfaults inside `XPRSaddmanagedcuts` on 46.01.03, 47.01.01, 47.01.02 and 48.01.01. **GLPK pool:** abort. **The drop:** a conditional big-M cut kept at 1e-9 stalls GLPK to the time limit even with its stall guard. Write the test as `!(violation > 1e-6)`, so that a cut made non-finite by a NaN is dropped too. |

What the drop costs: a conditional big-M cut `w'x - K x_b <= r`, with `x_b = 0` at the point,
is dropped once K exceeds its violation times 1e6, on every backend, from K = 1e6 for covers.
"May apply nowhere" allows that, and on GLPK both alternatives are unsafe:
- unscaled at K = 1e10: a value above the true optimum at `status::optimal`;
- a 1e-9 drop: stalls to the time limit.

No natural separator offered a cut violated by between 1e-9 and 1e-6. A deliberately weak one
lost every cut to the drop; Gurobi would have used them (1.28 times the nodes in geometric
mean).

Three solver-specific steps stay in their backends:

- **SCIP, tiny terms, mandatory after scaling.** `var.c:14312` drops a term with
  `|a| <= 1e-9` without moving the side, and the shared scaling produces such terms from
  correct cuts. Without this step, the large form of TINY2 at M >= 1e10 gives 103 for 106 in
  12 of 12 runs, and 0 of 12 unscaled. So:
  - move each such term to the side at its worst global bound;
  - skip the cut if that bound is infinite or the cut is an equality.

  A cut whose coefficients span 1e9 or more then becomes useless on SCIP, though still valid.
  Geometric scaling kept it intact in 18 of 18 runs, but was wrong once on a big-M model: a
  lead, not a fix.
- **CPLEX, same point.** `EpRHS` applies to the crushed cut, so a presolve aggregation can
  raise the threshold past any test on the written cut (`x = 1000 y` spins at 1e-5).
  - The step: skip the separator when a thread sees the same node and an identical point as
    its last call. That took 40 of 40 cases to completion, where the plain recipe managed 22
    of 40. It also skips CPLEX's post-loop call on the same LP.
  - It assumes the separator is a function of the point.
  - It does not stop a slowly moving loop of fresh weak cuts: violated by 1e-5, they held the
    root for 60 s, against 0.12 s with `CUTPASS=50` (decision 5c).
- **GLPK, stall.** Offer nothing after 5 rounds at one node with relative objective change
  below 1e-4. Without it 17 of 40 hit the limit; with it 40 of 40 finish.

As one step (`final_prepare.hpp`, with these three backend steps), the preparation passed the
conformance set at the skeptics' sizes on six backends:

| Backend | Release | Conformance cases passed |
|---|---|---|
| Gurobi | 12.0.1 | 72 of 72 |
| CPLEX | 22.1.2 | 64 of 64 |
| Xpress | 47.01.01 | 49 of 49 |
| COPT | 8.0.5 | 49 of 49, both with and without the separator delay that reaches the tree |
| SCIP | 9.2.1 | 49 of 49, plain and deep |
| GLPK | 5.0 | 49 of 49 |

It was also right in 360 of 360 scale-sweep runs against Gurobi's `MIPGap=0` optimum. Other
releases ran the skeptics' variants, not this one.

## Recipe per backend

Each skeptic ran the shared conformance set `cut_conformance.hpp` with 0 recipe failures: 49
cases and more, but 39 on Xpress releases other than 47.01.01. The failures seen were
baselines over the time limit.

| Backend | Hook | Point | Cut | Must also |
|---|---|---|---|---|
| Gurobi | `GRB_CB_MIPNODE`, with status `GRB_OPTIMAL` (statuses 1, 3 and 6 fire too, and their point is garbage) | `MIPNODE_REL` | `GRBcbcut` | `PreCrush=1`, written by `solve()` while the callback is attached and restored afterwards, never by the setter; value `Obj'x + ObjCon`, with `Obj` read once per solve, since Gurobi has no native read |
| CPLEX | `CPX_CALLBACKCONTEXT_RELAXATION` | `CPXcallbackgetrelaxationpoint` | `CPXcallbackaddusercuts`, `CPX_USECUT_PURGE`, `local=0` | the mutex; the same-point guard; error 1121 is a dropped cut, counted; explicit dispatch on three contexts; column count captured before `mipopt` |
| Xpress | `XPRSaddcbcutround` | `XPRSgetcallbacksolution` on `cbprob`, sized by `XPRS_INPUTCOLS` | `XPRSaddmanagedcuts(cbprob, 1, ...)`, never touching `p_action` | the mutex; managed cuts do nothing below 45.01.04, with return code 0, which sits inside the validated range (decision 3) |
| COPT | `COPT_CBCONTEXT_MIPRELAX`, with the `MIPNODE` bit kept | `COPT_CBINFO_RELAXSOLUTION` | `COPT_AddCallbackUserCut` | the mutex; dispatch `MIPRELAX` before the bound path; always return 0, since nonzero stops the solve |
| SCIP | dormant separator from the constructor; priority 1000000, `freq=1` written in `solve()`, `expbackoff=1`, `maxbounddist=1` | `SCIPgetSolVals` on original pointers | row `local=FALSE`, `removable=TRUE`, `SCIPaddRow(force=FALSE)`, then `SCIPaddPoolCut` | scoped `thread_local` plugin, since the callback has no user-data argument and `SCIPsepaGetData` needs the runtime-loaded API; a valid `*result` on every path; `SCIPgetSolOrigObj` for the value |
| GLPK | `GLP_ICUTGEN` | `glp_get_col_prim` on the tree's problem | `glp_ios_add_row` (the pool) | presolve off with the root LP run by the wrapper, written by `solve()` and restored; column ids checked against the column count, since a foreign variable is a process abort; the stall guard |

Why these and not their neighbours:

- **Xpress: `cutround` and managed cuts, not `optnode` and `XPRSaddcuts`.**
  - Managed cuts take original columns, and they were the only recipe that finished every
    sweep. `cutround` is also 3 to 5 times faster on ATSP at the root.
  - The other recipe needs `XPRSpresolverow`, whose omission gives time limits and a wrong
    optimum (299 for 297). It also needs folding, a relative tolerance and handling of
    status 1, and it still loops on valid cuts.
  - `cutround`'s one cost: `CUTSTRATEGY=0` silences the hook with no error.
- **SCIP: row and pool.** No consistent node or time difference from the row alone, and the
  pool is what survives a restart (769 nodes against 1282). The pool alone does little, for a
  cut added below the root, until a depth multiple of 10.
- **CPLEX: `PURGE`.**
  - `FORCE` helps DFJ, slows covers, and `RepeatPresolve=1` drops it anyway.
  - `FILTER` refuses some valid root cuts.
  - Clearing a `Reformulations` bit instead of swallowing 1121 cost about 200 times the nodes
    on two duplicate-integer knapsacks (54 against 10611; 905 against more than 113453). It
    cost nothing on a duplicate-column facility model (7 against 4).
- **GLPK: the pool.** Rows added directly at `GLP_ICUTGEN` abort the process under the
  wrapper's default generators, and they need a violation check or the node never ends.

## Interactions with the rest of MIP++

- **The deletion filter.** Its trials go through `solve()`, so a node callback attached per
  solve runs in every trial.
  - **The failure.** A cut valid for the model is invalid in a trial that relaxed the row it
    came from. On an infeasible model, "valid at every feasible point" is vacuous, so the
    contract cannot protect the trials. On Gurobi a cover separator made
    `compute_iis_by_deletion` report `irreducible` for a feasible set: 20 seeds, one
    deterministic path, with a native hook standing in for the proposed one.
  - **The cure.** Lazy constraints can be background because they define the model; user cuts
    by design do not. So the filter should run its trials without the node callback (decision
    7).
- **A callback set after a finished solve (D4).** For user cuts alone this is harmless: the
  old optimum stays optimal, and "it may never run" covers the silent non-call. So
  `set_node_relaxation_callback` keeps results, as the other setters do. The late-attach test
  is solve, attach, solve with no change: a correct optimum, call count unasserted.
- **Parameters a callback needs** (`PreCrush`, GLPK's presolve, COPT's context bits) are
  written by `solve()` for its duration and restored, as the callbacks are attached. A setter
  that writes them, as `set_candidate_solution_callback` writes `LazyConstraints`, leaves every
  later solve paying for them. A parameter write did not reset the search on Gurobi or CPLEX,
  so restoring should keep resumption: inferred, not run with `PreCrush`.
- **The point's lifetime.** The handle's `get_solution()` must own its values. The shipped
  candidate handles on Gurobi and CPLEX capture the handle, which dies with the call (D6).
- **Errors.** `add_user_cut` never throws: a refused cut is dropped. Any other native error
  needs one rule for every backend (decision 6). CPLEX's shipped `cbcheck` aborts on any
  nonzero, which would abort the search over an optional cut. The Gurobi, Xpress and COPT
  handles throw through the C frames.
- **Model changes between solves.** No backend kept a hook's cuts across a change: `P` found
  the re-solve right on all of them. On several backends `P` used `set_constraint_upper_bound`
  in place of `set_constraint_rhs`. *Across `remove_variable` and `add_column`: unverified.*
  `COPT_AddUserCut`, called before a solve, does persist, even through `COPT_Reset`, and there
  is no call to delete it. That is why this proposal does not use it.
- **Cuts outlive a stop on Gurobi.** An unchanged model resumes with its cuts, even after the
  callback is detached or replaced. Valid cuts stay valid, so this is harmless, and any
  attribute write or `GRBreset` clears them.

## GLPK and Cbc: the honest cost

**GLPK can ship, at a price.** Its presolver gives the hook another column space: the result
is an abort, or a valid cover separator losing the optimum at `status::optimal` in 6 of 10
seeds. So a set callback turns presolve off, except on a model without columns, which keeps
the presolve path (presolve off aborts the process there on 4.65 and 5.0). The wrapper then:
- runs the root LP itself, since `glp_intopt` refuses without one;
- maps a failed root to a status, ignoring the stale `glp_mip_status`;
- subtracts the root's time from the limit.

The costs:

- **Without a callback, nothing changes:** same path, and 175 pinned tests pass (4 skipped) on
  seven releases through the recipe path.
- **Presolve off costs nodes:** 1.60 times in geometric mean over 46 instance pairs, 1 to 189
  times on loose big-M, 957 at worst, and 4 of 56 instances stopped closing.
- **Cuts usually repay it:** 0.65 times the presolve-on nodes over 47 pairs, and ATSP from 6997
  nodes to 1. They do not on fixed-charge networks: 50.5 times, still not closed.
- **Cuts apply only in the subtree** where they were added, and the pool drops some silently,
  so a separator must re-offer them.
- **A malformed cut is an unrecoverable process abort,** with no error code. The preparation
  and the column-id check stand between users and that.
- **`glp_npp_*` is no way out:** it runs only part of presolve, only on 4.65+, and recovers none
  of the big-M benefit (1.08 times the presolve-off nodes).

**Cbc should stay out.** The contract does hold at sane scale: 58 of 58 cases on four releases,
0 failures in 4365 attack runs, and root cuts work (70150 nodes to 0). But:
- below the root, the hook misses 82 to 96% of nodes on 2.10 on long searches of models under
  500 rows plus columns (9 to 24% above that), and 13 to 99.7% on master;
- a restart hands the hook a smaller model and drops all cuts;
- every safety piece fails silently (657 for 549 without the column guard and
  `preprocess off`);
- two ABIs share one symbol;
- the maintainer has ruled Cbc experimental.

**HiGHS and MOSEK have no call.**
- **HiGHS:** `kCallbackMipDefineLazyConstraints` has no call site, and `#1837 Add user cuts` is
  still open upstream. `Highs_addRow` from a callback returns 0 yet never reaches the
  relaxation, and it filters candidates unsoundly (false infeasible 8 of 8).
- **MOSEK:** its manual forbids calling the solver from a callback, and the row is ignored for
  that solve.

The fallback is the docs' cutting-plane loop (`branch-and-cut.md:110`), done as LP re-solves and
then the MIP with the rows in. On ATSP n=26:

| Solver | LP rounds | MIP with the rows | MIP alone |
|---|---|---|---|
| HiGHS | 7 rounds, 0.008 s | 0.28 to 0.52 s | 9 to 17 s |
| MOSEK, told to use dual simplex | 0.021 s | 0.14 s | 40 s |

At n=60 the MIP stage dominates (7.9 s and 52 nodes on HiGHS). *That instance is friendly: DFJ
alone closes the root to 0.4 to 1.7%.*

## Local cuts: not now

Available on CPLEX, Xpress, SCIP, GLPK and Cbc; absent from Gurobi and COPT. A later additive
pair would be `has_local_user_cuts<Handle, Model>` with `add_local_user_cut(c)`, plus a read of
the node's bounds, without which a valid local cut cannot be written:
- CPLEX, SCIP, GLPK and Cbc read them directly, though CPLEX does not tighten them on columns
  presolve eliminated;
- Xpress reads them in presolved space, which needs a reverse map per call and cannot serve
  eliminated columns.

No consumer asks, and Gurobi and COPT could not offer it. A subtree-valid cut has a global form
only when the subtree is a set of binary fixings: `a'x <= b + sum M_j (1 - x_j) + sum M_j x_j`,
with a finite `M_j` for every variable. That form has three problems:
- it needs the node's bounds, which Gurobi and COPT do not expose;
- it grows by one term per fixing and is nearly redundant outside the subtree;
- its big M runs into the scaling and the 1e-6 drop.

Until then, a user with local cuts has:

| Backend | Route |
|---|---|
| Gurobi, COPT, GLPK | none |
| CPLEX | a native generic callback, only if no MIP++ callback is set, since `solve()` takes the slot |
| Xpress | a native `cutround` with `XPRSaddmanagedcuts(g=0)` beside MIP++'s |
| SCIP | a native separator included before the first solve |

A native-context accessor on the handle would open these routes without a new concept
(decision 2).

## When it runs

"At every node" holds nowhere, so the concept's comment refuses to say it. COPT's guide promises
a call on every solved LP, and measurement contradicts it. A separator that needs a call to be
correct is wrong by construction.

| Backend | Tree nodes | Repeats after a cut | Integer points | Silent if |
|---|---|---|---|---|
| Gurobi | at most one call per explored node, and only where its LP is optimal (13156 of 20203 hook fires in one run) | root only, never at a tree node | sometimes | solved in presolve, or integral root LP |
| CPLEX | not pruned or integer nodes; 2 calls per node on the same LP | yes, no limit | not normally | `CUTPASS=-1`, or every cut family off, removes the root call |
| Xpress | nodes with an optimal LP | yes | yes | `CUTSTRATEGY=0` |
| COPT | from COPT's own cut loop, not every LP | yes | only with lazy machinery on | `CutLevel=0` gives 0 calls (7823 nodes on ATSP n=26) |
| SCIP | 711 of 982 ATSP nodes at `expbackoff=1`, 35 at the default of 4 (gadgets: 642 of 1281 against 19) | up to 7 per node | sometimes | `separating/maxcuts=0` with `maxcutsroot=0`, `separating/maxruns=0`, or `SCIPsetSeparating(OFF)` if `freq` is not rewritten; `separating/maxrounds=0` leaves the root calls only |
| GLPK | processed nodes | yes, no limit | never at `GLP_ICUTGEN` | an empty model (it keeps the presolve path), a root LP that fails, or an integral LP |

An integral root LP means no call at all on most of them: hence lazy constraints for
feasibility.

## Threads

How each backend runs the hook:
- **Gurobi** runs it on one thread, as promised (`c_logging.txt:237-239`).
- **COPT and Xpress** serialize it, measured and not promised. COPT's guide says "COPT does not
  guarrantee thread safe", and Xpress's `MUTEXCALLBACKS` entry does not list `cutround`.
- **SCIP and GLPK** have one search thread.
- **CPLEX** is the only proposed backend that calls it concurrently, by design: up to 15 calls
  in flight at 16 threads. Threaded Cbc 2.10 builds overlap too.

**Recommendation: one mutex per solve, held around every MIP++ callback that solve attaches
(candidate, node relaxation, progress), on CPLEX, Xpress and COPT.**
- **Why the scope.** CPLEX runs CANDIDATE and RELAXATION at the same time, so a separator shared
  by the lazy and the user-cut lambdas races under a per-lambda mutex. A lambda with a plain
  shared `std::set` corrupted the heap on CPLEX with no mutex.
- **Where it lives.** In the solve's callback context, never in a static, so that a nested solve
  inside a separator cannot deadlock on it.
- **Deadlock risk.** One mutex held across RELAXATION, CANDIDATE and GLOBAL_PROGRESS on CPLEX ran
  18 runs of 15 s without deadlock. CPLEX already holds its own lock across GLOBAL_PROGRESS, so
  that combination is the one to watch.
- **It also fixes D7.**

The mutex costs speed on CPLEX. Nodes per second with the mutex against without, on
mkp30x100s7 with covers, deterministic mode:

| Separator | 4 threads | 8 threads | 16 threads |
|---|---|---|---|
| covers alone | 0.99 | 0.995 | 0.98 |
| covers + 200 us | 0.57 | 0.43 | 0.31 |
| covers + 1 ms | 0.33 | 0.20 | 0.13 |

Opportunistic mode loses more: 0.90, 0.28 and 0.098 at 16 threads. The timings were taken under
load from other runs, so read them as ratios. A slow separator stalls the tree on Gurobi and COPT
anyway: with 2 ms in the hook, Gurobi at 8 threads explores about as many nodes as at 1. An
opt-out for thread-safe lambdas can come later without breaking anything.

## Side effects

With no cut added, the search path is unchanged at one thread on Gurobi, CPLEX, SCIP, GLPK, Xpress
and COPT at default cut settings. The exceptions:

- **Gurobi, `PreCrush=1`.** It is needed: without it, a cut on merged general integers is ignored
  with return code 0 (60 of 60 later points violate it). It costs nothing measurable without
  parallel columns, and with them:
  - 100 to 300 times the nodes on parallel general integers;
  - at least 12 to 41 times on a doubled knapsack, where all three runs timed out.

  The manual calls its impact on the size of the presolved problem "usually small"; here it was
  40 presolved columns against 20.
- **COPT, root cuts off.** A no-op hook changes the search (ATSP from 7823 nodes to 5847; a
  knapsack unchanged), apparently by switching tree cuts on (inferred).
- **GLPK.** Presolve off, described above.

## Defects found in shipped code

Each reproduced through the public API unless noted; D1, D3, D4, D5 and D6 again in a second,
separate run. None is fixed. D1 to D5 give wrong answers and do not depend on this
feature.

| # | Where | What | Reproduction |
|---|---|---|---|
| D1 | `xpress_milp.hpp:100`, `docs/reference/concepts.md:190`, `docs/algorithms/branch-and-cut.md:35,38` | `reject_solution()` on a node's relaxation drops the whole node (vendor note 4), while the docs say it "discards the candidate". The fix changes `test/test_suites/iis.hpp:104`, which pins the behaviour, and needs a value check in `progress_callback.hpp:445-472`. | max x, x integer in [0,5], reject x=5: `status::optimal`, value 1, truth 4. With the row 2x<=10: `status::infeasible`. [own_xpress_reject.cpp](survey/own_xpress_reject.cpp). 14 of 33 rejecting knapsacks ended with a worse value at `status::optimal`. |
| D2 | `docs/algorithms/deletion-filter.md:185` | Follows from D1: `compute_iis_by_deletion` on `xpress_milp` claims a proof the rule does not give. | x, y in [0,5], x+y<=8, reject value 8 only, (5,2) accepted: reports an irreducible set. |
| D3 | `copt_milp.hpp:95-97`, `model_base.hpp:340-352` | `add_lazy_constraint` passes exact zeros, and COPT mishandles them. | `x - x` or `0.0 * x` in a lazy row: false infeasible on a feasible model; link rows give 1.5 to 4.7 times the optimum at `status::optimal`. [verify_copt_lazyzero_min.cpp](survey/verify_copt_lazyzero_min.cpp). |
| D4 | `gurobi_milp.hpp:229-233`, `cplex_milp.hpp:264-266` | A candidate callback set after a finished solve has no effect on the next `solve()` of an unchanged model, and a parameter write is not a change. | A lazy row forbidding the optimum: the second solve still returns 5, at a point violating it. Gurobi calls the callback once and ignores the row; CPLEX never calls it. After an objective write, both give 4. COPT is not affected. On Xpress the callback runs, but D1 masks the value. [own_gurobi_late2.cpp](survey/own_gurobi_late2.cpp), `final_late_attach.cpp`. Fix in `solve()`, keyed on the candidate callback having changed since the last finished solve. |
| D5 | `cplex_milp.hpp:159`, `copt_milp.hpp:95-97` | `add_lazy_constraint` rejects the candidate even when the candidate satisfies the constraint; Gurobi accepts it. D3's empty case is one instance. | max x+y, x and y integer in [0,5], x+y<=7, lazy x+y<=100 at the value-7 candidate only: `status::optimal`, value 6, on CPLEX 22.1.2 and COPT 8.0.5. At every candidate: `status::infeasible`. Knapsacks 5x30: no solution at the time limit, where Gurobi finishes in 0.06 s. [final_satlazy.cpp](survey/final_satlazy.cpp). Fix: the drop-satisfied step of the shared preparation, on the lazy path too. |
| D6 | `gurobi_milp.hpp:168-169`, `cplex_milp.hpp:188-189` | The candidate point captures the handle, which dies with the call: a separator that keeps its last point reads a dead frame. | ASan stack-use-after-return at `gurobi_milp.hpp:169` ([final_stored_point.cpp](survey/final_stored_point.cpp)); CPLEX by reading the code. COPT and Xpress own their buffer. |
| D7 | `cplex_milp.hpp:236-241`, `docs/algorithms/branch-and-cut.md:69` | The candidate callback runs concurrently on CPLEX with no lock: 10 overlaps in 40 calls with a 3 ms hook. The docs mention a solver thread but not concurrency. | `verify_cplex_threads c3`. |
| D8 | `cplex_milp.hpp:236-241`, latent | `callback_fun` sends every non-CANDIDATE context to the progress callback: `std::bad_function_call` once RELAXATION is registered. | `verify_cplex_c9.cpp` with a patched header. |
| D9 | `cplex_milp.hpp:131,147,182` | `CPXgetnumcols` inside a callback, documented undefined. Never seen to misbehave in 314842200 reads. | Vendor `callbacksetfunc.html`. |
| D10 | `glpk_milp.hpp:103-110,348` | `set_time_limit` can be exceeded about 2 times: the root LP and the tree each get the whole limit. | An 8 s limit, 16.9 s total, on ATSP n=150. |
| D11 | `cbc_milp.hpp:726,737` | Null read on Cbc master after a stop with no incumbent. Arguably misuse under the UB ruling. | `verify_cbc_nullsol.cpp`. |
| D12 | `scip_milp.hpp:100-110` | `~scip_milp` nulls `_scip_vars` before `SCIPfree`, so a plugin reading it crashes. Latent: no shipped plugin does. It matters for lazy constraints on SCIP. | `verify_scip s11`. |
| D13 | `xpress_milp.hpp:62-68` | `check()` throws from inside the candidate callback, across the C frames. | Read, not executed. |
| D14 | `docs/solving/progress.md:60`, `cbc_api.hpp:99-105`, `model_base.hpp:299-301`, `copt_milp.hpp:124` | Text and declarations: "reaches no callback" is false for Cbc's cut callback. The declarations are dead, and wrong on master. "COPT keeps the last" holds for rows only. "`NodeCnt` reads 0" holds for `GetIntAttr`; an undocumented `GetCallbackInfo(cb, "NodeCnt", int*)` is exact in MIPRELAX, MIPNODE and MIPSOL, and lags in INCUMBENT. | |

Upstream:

- **COPT 8.0.5 proves false optima with no MIP++ at all.**
  - `copt_cmd` with `Threads=1` on
    [copt_false_optimum_mkp_15.mps](survey/copt_false_optimum_mkp_15.mps) gives 11997 at 0% gap;
    default threads and Gurobi give 12135.
  - On that 5x30 family: 5 of 25 at `Threads=1`, 4 of 25 at default threads, 0 of 25 with
    `Presolve=0`.
  - With presolve off, valid cuts whose coefficients span 1e9 or more also gave false optima,
    whatever the scaling (7 of 10 at 1e10). At default presolve: 0 of 60.
  - So COPT tests need an oracle other than COPT.
- **SCIP 8.0.4 and 9.2.1 in one process crash** on plain solves, in both orders, against the "two
  versions may coexist" ruling. The cause is unknown, so it may be ours rather than the vendor's.
  9.2.1 then 10.1.0 in sequence works.
- **HiGHS `Highs_setCallbackSolution` has no effect** through the C callback on 1.14.0 and 1.15.1.

## Tests

A shared typed suite, `NodeRelaxationCallbackTest` and `UserCutsTest`, using only the concepts. It
is proven on a real backend, since `dumb` has no search. The conformance runs taught what a test
must do:

- **Assert the hook ran.** A dead hook passes every optimum comparison (seen on Cbc).
- **Force a tree** with families that bite there (covers, C5, C10). DFJ closes the root on most
  solvers, so C1 to C4 prove only the root mapping, and a depth read would be a later concept.
- **Check the point, not only the optimum.** Garbage points still yield valid covers (Gurobi with
  no status filter).
- **Anchor COPT's baselines to another solver.**
- **Do not pin scope.** S1 is an experiment, not a test: the contract leaves scope undefined.

Cases with power, each shown by removing one recipe piece:
- fixed columns first with a cover separator (column shift);
- split-term covers (folding);
- `x - x` in a `<=` form on COPT (zeros);
- a scale sweep from 1e-8 to 1e12, checked against another solver's `MIPGap=0` optimum (scaling);
- a valid cut whose scaled form has a tiny term (SCIP);
- a conditional big-M cover (GLPK, the drop);
- a satisfied or tiny cut re-offered at the root, and a moving loop of fresh weak cuts (CPLEX
  termination);
- a racy separator shared by the candidate and the relaxation callbacks (the mutex);
- a value check in both senses, with an offset.

Seams to cover:
- all three callbacks in one solve;
- a moved model keeps its callback;
- a second callback replaces the first;
- solve, attach, solve with no change (correct optimum, calls unasserted);
- stop, then re-solve;
- the deletion filter with a cover separator;
- an integer-free model (status optimal, calls unasserted);
- an empty model;
- a non-finite coefficient;
- a satisfied lazy constraint (D5);
- a stored point read after the call (D6).

Harness fixes: C4 needs Euclidean costs, C7 needs a real `==`, and C8 must check that its baseline
reached optimal. Covers have unit coefficients and cannot test the scaling. Cbc, HiGHS and MOSEK
get commented `INSTANTIATE_TEST` lines with reasons.

## Docs

Lines that become false the day this lands:
- `branch-and-cut.md`: the title, and lines 28, 35, 38, 69 and 108-110;
- `concepts.md:190-191`;
- `solvers/index.md`: lines 103, 119 and 125 (the single-slot note names COPT only, but CPLEX and
  GLPK also clobber native hooks);
- `getting-started/index.md:92`;
- `README.md`: lines 176, 182 and 194;
- `progress.md:60` (D14), and `progress.md:32-33` if the mutex is shared;
- `deletion-filter.md:181-198`.

Tooling: two entries in `tested_features_table.py`'s `formated_test_names` ("Node relaxation
callback", "User cuts"), regenerated tables, and a paragraph in the conformance skill carrying the
test lessons above.

## Size

About 1300 lines with tests and docs:

| Part | Lines |
|---|---|
| Shared preparation | 80 |
| Gurobi | 70 |
| CPLEX | 120 |
| Xpress | 90 |
| COPT | 70 |
| SCIP | 150 |
| GLPK, if ruled in | 190 |
| Tests | 380 |
| Docs | 150 |

This is an estimate.

## Decisions for the maintainer

Recommended option first.

1. **Contract.** `add_user_cut` promises validity at every point `solve()` may return as feasible,
   and the solver may apply it anywhere or nowhere? The alternative, a per-solver meaning, cannot
   be written generically.
2. **Local cuts.** Leave them out for now, additive later? The alternatives:
   - ship `has_local_user_cuts` on CPLEX, Xpress, SCIP and GLPK, with a local-bounds read;
   - expose a native-context accessor on the handle, so the native routes above open without a
     concept.
3. **Backends.** Gurobi, CPLEX, Xpress, COPT and SCIP first; GLPK as its own commit, accepting
   presolve off, its guards and aborts on malformed input; Cbc left out?
   - **SCIP and Xpress have no correct lazy path today.** SCIP has no candidate callback, and
     Xpress has only D1. User cuts still serve complete formulations there, and the concepts
     already stop generic code that needs both. The alternative: wait for their lazy paths.
   - **Xpress floor.** Managed cuts do nothing below 45.01.04, which sits inside the validated
     range [45.1, 47.2), so the shared suite would fail on 45.01.01 to 45.01.03. The options:
     - **(a)** raise Xpress's validated floor to 45.1.4 after a compat-matrix run, which demotes
       45.01.01 to .03 to the partial tier for every feature;
     - **(b)** use the `optnode` and `XPRSpresolverow` recipe below 45.01.04, which the plan
       rejects elsewhere;
     - **(c)** leave Xpress out until then.

     Recommended: (a). Whichever is chosen, the setter throws `feature_unavailable_error`, never
     `solve()` or the hook.
   - **Untested validated releases.** COPT 7.2 and Gurobi 10 are validated but untested here:
     instantiate there only after a run, or narrow those claims.
4. **Threads.** One mutex per solve, around every MIP++ callback, on CPLEX, Xpress and COPT, so that
   "never at the same time" holds everywhere? This also closes D7.
5. **Preparing a cut.** Three parts:
   - **(a)** The structural steps in shared code: fold, strip zeros, skip empty, split `==`?
     Recommended; uncontroversial.
   - **(b)** Scaling and the 1e-6 drop everywhere? Recommended: they were needed on COPT, SCIP and
     GLPK, cost nothing measured on CPLEX, Xpress and COPT, and help Gurobi. The cost is that
     conditional big-M cuts with K above about 1e6 are dropped on every backend. The alternative is
     to scale and drop only where measured.
   - **(c)** CPLEX's moving root loop: document it, or have `solve()` write `CUTPASS=50` when the
     user left it at the default? Recommended: document. `CUTPASS=50` cost nothing on 5 instances,
     but it changes CPLEX's own cut loop behind the user's back.
6. **Native errors in the handle.** A refused cut is dropped. Any other nonzero native return
   should either drop that cut too, or end the solve as `status::failed`, with one rule for every
   backend. Recommended: drop it, since a user cut is optional by contract.
7. **Deletion filter.** Run its trials without the node-relaxation callback? The alternative is to
   document that an `irreducible` answer is unproven while one is set.
8. **D1.** Two parts:
   - **(a)** Correct the docs now.
   - **(b)** Wire `add_lazy_constraint` on Xpress as its own proposal: preintsol, `XPRSpresolverow`
     and `XPRSaddcuts`, `MIPDUALREDUCTIONS=0` written in `solve()` after user parameters (a later
     `PRECOMPONENTS=-1` undoes it), measured right to n=40. That way users no longer need
     `reject_solution`, which cannot be repaired in place, because the node drop is the solver's.
9. **Order.**
   1. The shared preparation lands first, and D3 and D5 reuse it on the lazy path.
   2. Then D1 (docs), D4, D6, and D7 with D8, each in its own commit.
   3. Then the feature.

## Not verified

- **Releases.** Gurobi 10.x (not installed). COPT 7.2.5 (licence refused). MSVC, Windows and macOS.
  The single preparation ran on one release per backend; other releases ran the skeptics'
  variants.
- **Instances.** Mostly ATSP, covers, gadgets and random binary models, plus 56 real and synthetic
  instances on GLPK. The thresholds (1e-6, 5 rounds) rest on those.
- **Equality cuts** through the preparation: no conformance case has one. **Non-finite input:** no
  run offered one.
- **Mechanisms.** Unknown or inferred:
  - the root causes of COPT's signed-zero and false-optimum faults;
  - CPLEX's same-node behaviour at 2 or more threads, and its D4 cause;
  - Gurobi's cut filter;
  - the SCIP coexistence crash.
- **Seams.** Unverified:
  - the deletion-filter failure on backends other than Gurobi;
  - cuts across `remove_variable` and `add_column`;
  - CPLEX indicator constraints with cuts;
  - Gurobi `ConcurrentMIP` with real cuts.
- **A real wrapper.** Every recipe ran in probes against native hooks, not as library code.
