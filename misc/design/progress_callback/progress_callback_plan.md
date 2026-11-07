# Progress callback: assessment and plan

On 2026-11-06 the maintainer asked how progress callbacks could get a common
interface like `candidate_solution_callback`, which backends support them, and
what to use otherwise. This note answered, and the maintainer accepted all seven
recommendations the same day.

**Status: implemented.** The API is the one proposed below, documented in
[docs/solving/progress.md](../../../docs/solving/progress.md) and pinned by
`test/test_suites/progress_callback.hpp`. Where the build differs from this note,
which was written first, [What the implementation changed](#what-the-implementation-changed)
says so; the rest of the note is the assessment as it stood.

**Method.** One probe per backend, run against the real libraries, registers the
native hook and records what it reads during a solve. Probes, outputs and
releases are in [survey/RECORD.md](survey/RECORD.md). *Measured* means a probe
ran it, *documented* that a vendor header or manual says it, *inferred* a reading
of either. Line numbers are on `feat/lp-basis` with the uncommitted 1.1.0 changes.

## What the implementation changed

- **Callbacks are attached by `solve()`, not by the setters, and detached after
  it.** This note kept registration in the setters and asked for "any callback
  attached" detaches around `compute_iis()`. Native user data is `this`, so a
  setter's registration dangles once the model moves: the existing candidate
  callback segfaulted that way on Gurobi, CPLEX, Xpress and COPT, which nothing
  here had measured. Attaching per solve fixes that, removes the `compute_iis()`
  detaches, and fixes Xpress calling a callback once per registration (4 calls
  with one, 8 with two, on the same search).
- **SCIP catches events once per transformed problem, in `solve()`.** The trap
  section asks for a catch in `initsol` and a drop in `exitsol`. Those callbacks
  receive no way to reach the runtime-loaded API, so the handler is included by the
  constructor, because a callback setter must keep results and SCIP takes plugins
  only before a solve, and `solve()` transforms and catches. A restart keeps the
  subscription, so no event doubles.
- **HiGHS needs no reset.** Attaching per solve calls `Highs_setCallback`, which
  clears the interrupt flag (mutation checked on 1.7.2, 1.14.0 and 1.15.1).
- **One native context per backend where one suffices:** Gurobi reads `GRB_CB_MIP`
  alone and HiGHS `MipInterrupt` alone. The incumbent shows at the next call.
- **The Xpress wrapper holds a mutex** around the callback, for one solve, as the
  review of this note asked: `MUTEXCALLBACKS` does not list `miplog`.
- **MOSEK callback codes are resolved by name too**, through
  `MSK_callbackcodetostr`, since they are numbered alphabetically like the items.
- **"Gap arithmetic just works" was too strong.** An infinite incumbent over an
  infinite bound gives NaN. Comparisons are what work: `value <= target` is simply
  false until there is an incumbent. The docs show that form.
- **The side finding on `HighsCallbackDataOut` is fixed:** the struct is now opaque
  and `HighsCallbackDataIn` holds only the field every release keeps first. The two
  others below stay open.

### How it was verified

- **Full suite:** `conan build` with `gcc14_c++23`: 608 of 608 ctest suites pass, 0
  warnings. Four skip, none from this change: `Cbc_api`, `Clp_api` and `SoPlex_api`
  on local libraries outside the validated range, and `HiGHS_lp` for a test that
  needs a HiGHS below its IIS floor. One test assertion was removed after that run,
  and the progress and candidate suites were rerun on all 8 backends.
- **Releases:** the progress and candidate suites pass on Gurobi 10.0.3, 11.0.3,
  12.0.1, 13.0.2, 13.0.3; CPLEX 22.1.0, 22.1.1, 22.1.2, 22.2.0; Xpress 45, 46, 47;
  COPT 8.0; MOSEK 11.0.9, 11.0 (local), 11.1.11, 11.2.8; HiGHS 1.7.2, 1.10.0, 1.14.0,
  1.15.1; SCIP 8.0.4, 9.2.1, 10.1.0; GLPK 4.59, 4.65, 5.0. Loaded libraries were
  confirmed with `LD_DEBUG=files`. COPT 7.2 refuses the license here and skips.
- **Stability:** 25 repeats per backend alone, 10 with all 8 running at once, no
  failure.
- **Mutations** ([survey/mutate.py](survey/mutate.py)), each caught by a test:
  sentinels left raw; registration by the setters on Gurobi and Xpress; Xpress and
  GLPK offsets dropped; MOSEK validity flags ignored; CPLEX attached on the LP path;
  CPLEX on `LOCAL_PROGRESS`; COPT on `MIPSOL`; SCIP's subscription kept across a
  free; GLPK's `cb_info` left stale. Removing the HiGHS flag reset was not caught
  because it changes nothing, which is how `Highs_setCallback`'s reset was found.
- **Others:** ASan and UBSan on SCIP, Xpress and GLPK, no report. Clang 18 compiles
  the 8 backends and the docs snippet. With a callback set, slices still resume on
  Gurobi, CPLEX and SCIP with a growing limit
  ([survey/probe_resume.cpp](survey/probe_resume.cpp)).
- **Not run:** MSVC, and COPT 7.2.

## Recommendation in short

- **Yes, a common interface works on 8 of the 9 `*_milp` backends**: Gurobi,
  CPLEX, Xpress, COPT, MOSEK, HiGHS, SCIP and GLPK. All 8 can read the incumbent
  and the dual bound and stop the solve from the hook, under the quiet default,
  and on 7 of them the search path is unchanged at one thread (GLPK has no node
  count to compare). Released Cbc has no usable hook.
- **Shape.** `set_progress_callback(f)` and a `progress_callback_handle` with
  `get_solution_value()`, `get_dual_bound()` and `interrupt()`. `get_node_count()`
  is optional, because COPT cannot report it, and is tested with the existing
  `has_node_count` on the handle.
- **Cadence stays unspecified.** The same instance gets about 27 calls/s on Xpress
  and about 100 000/s on Gurobi, so the lambda throttles itself. A hook may run on
  a solver thread, but no probe ever saw two calls overlap.
- **MILP only for now.** LP hooks are poorer and absent on COPT, GLPK and SoPlex.
- **Do not use `candidate_solution_callback` as a tracker.** A no-op callback moves
  the search on COPT and CPLEX, in either direction, and it never fires when only
  the bound moves.
- **Otherwise.** No single fallback covers everything:
  - A log sink, `set_log_callback`, is the widest (9 of 11 backends) but text only.
  - Time slices resume only on Gurobi and CPLEX, and on SCIP with a growing limit.
- **Size.** About 600 lines with the tests and docs.

## Proposed interface

```cpp
template <typename T>
using progress_callback_handle_t = typename T::progress_callback_handle;

// Best incumbent and best proven bound so far. While there is none, the incumbent
// reads as the worst infinity of the objective's sense and the bound as the
// opposite one.
template <typename T>
concept has_progress_callback =
    requires(T & model, progress_callback_handle_t<T> & handle) {
        { model.set_progress_callback(
                [](progress_callback_handle_t<T> &) {}) };
        { handle.get_solution_value() } -> std::same_as<model_scalar_t<T>>;
        { handle.get_dual_bound() } -> std::same_as<model_scalar_t<T>>;
        { handle.interrupt() };
    };
```

The names are the model's own (`has_dual_bound`, `has_node_count`,
`model_concepts.hpp:304-313`), and
[concepts apply to handles](../../../docs/reference/concepts.md#concepts-on-callback-handles)
already, so `has_node_count<progress_callback_handle_t<M>>` needs no new concept.

```cpp
auto last = std::chrono::steady_clock::now();
model.set_progress_callback([&](auto & handle) {
    const auto now = std::chrono::steady_clock::now();
    if(now - last < 1s) return;
    last = now;
    std::println("{} {}", handle.get_solution_value(), handle.get_dual_bound());
    if(stop_requested) handle.interrupt();
});
```

Elapsed time stays out of the handle. `steady_clock` in the lambda costs nothing
and means the same everywhere, while the solvers' clocks do not: COPT has none in
its callback data, and SCIP's limit counts from the start of the transformed run.

**The contract to document:**

- Called from inside `solve()`, possibly on a solver thread, never concurrently.
  No `thread_local` state, no calls on the model. 0 overlaps were measured on all
  8. CPLEX documents the guarantee, six backends stayed on one thread, and on
  Xpress the wrapper has to provide it (see the traps).
- Keep it short. CPLEX holds a global lock on this context (documented), so a slow
  lambda blocks its search.
- The cadence is unspecified, and a call does not mean anything changed. Presolve
  or a long root can be silent: COPT went 1.9 s between calls here.
- The last call is not the final state. Read the model after `solve()`.
- It may never fire on a model without integer variables.
- `interrupt()` ends the solve as `status::interrupted`, and the next `solve()`
  is unaffected.

## What each backend does

Measured on the `large` instance, default threads, quiet. Rates are orders of
magnitude (see RECORD). "Path" compares node counts at one thread, with and
without the hook.

| Backend | Native hook | Calls/s | Threads seen | Path | No incumbent | No bound yet | Offset included | Fires without integers |
|---|---|---|---|---|---|---|---|---|
| Gurobi | `GRB_CB_MIP`, `MIPSOL` | 100 000 | 1 | same | `1e100` | `-1e100` | yes | no (`SIMPLEX` only) |
| CPLEX | `GLOBAL_PROGRESS` | 200 | 16 | same | `1e75` | `-1e20` (documented, `-CPX_INFBOUND`) | yes | **`CPXlpopt` fails, 1811** |
| Xpress | `miplog`, `intsol` | 27 | 32 | same | `1e40` | `-1e40` | **no** | yes |
| COPT | `MIPNODE`, `INCUMBENT` | 540 | 1 | same | `1e30` | `-1e30` | yes | no |
| MOSEK | `IM_MIO`, `NEW_INT_MIO` | 100-200 | 1 | same | **`0`** | **`0`** | yes | yes |
| HiGHS | `MipInterrupt`, `MipImprovingSolution` | 440 | 1 | same | `inf` | `-inf` | yes | no |
| SCIP | `NODESOLVED`, `BESTSOLFOUND` | 1000 | 1 | same | `1e20` | `-1e20` (inferred) | yes | yes |
| GLPK | `mip.cb_func`, `ISELECT`, `IBINGO` | 2000 | 1 | no count | `glp_mip_status` | `-DBL_MAX` | **no** | yes |
| Cbc 2.10.x | none | | | | | | | |

The table shows the signs under minimization. Under maximization each sentinel
flips (measured, runs D and E).

Gurobi's rate counts `GRB_CB_MIP` alone. **Overhead** stayed within run-to-run
spread at one thread on all 8, with a recording hook taking a mutex on every call
(no hook, then hook): Gurobi 0.88 s, 0.90 s; CPLEX 1.20 s, 1.18 s; Xpress 1.48 s,
1.47 s; COPT 25.51 s, 25.48 s; MOSEK 1.09 s, 1.09 s; HiGHS 4.96 s, 4.74 s; SCIP
4.50 s, 4.48 s; GLPK 8.38 s, 8.98 s, whose own spread over two reruns is the same
size.

Every hook read here is monotone once its sentinels are normalized, with no primal
or dual move in the wrong direction. The one exception is MOSEK's undefined dual
`0` before the root.

**Stopping.** Every path below returned within 10 ms of the request and ended as
`interrupted{solution_available = true}`.

| Backend | Stop |
|---|---|
| Gurobi | `GRBterminate(model)`. A non-zero return is no stop: `solve()` throws "Error number 1". |
| CPLEX | `CPXcallbackabort(context)` |
| Xpress | `XPRSinterrupt(prob, XPRS_STOP_USER)` or a non-zero `miplog` return |
| COPT | `COPT_Interrupt(prob)` |
| MOSEK | non-zero return, `MSK_RES_TRM_USER_CALLBACK` (`mosek_milp.hpp:160`) |
| HiGHS | `data_in->user_interrupt = 1`, **reset by the wrapper on every other call** |
| SCIP | `SCIPinterruptSolve(scip)` |
| GLPK | `glp_ios_terminate(tree)` |

## The traps a wrapper has to absorb

All measured unless marked.

- **HiGHS, the obvious hook is silent.** `kCallbackMipLogging` never fires under
  `output_flag = false`: `printDisplayLine` returns first
  (`HighsMipSolverData.cpp:1726`). `MipInterrupt` fires regardless. `user_interrupt`
  is honoured only from the `*Interrupt` types (`HighsCallback.cpp:86-93`,
  source), so `interrupt()` from an `ImprovingSolution` call has to wait for
  the next `MipInterrupt`.
- **HiGHS, the struct is stale.** `HighsCallbackDataOut` at `highs_api.hpp:227`
  has the 1.10 layout. From 1.11 the struct starts with `void * cbdata`, so on
  1.15.1 it reads `nodes=-4550735141097439218`, where the item says 1001. Read
  through `Highs_getCallbackDataOutItem(out, "mip_dual_bound")`, which worked on
  1.7.2, 1.10.0, 1.14.0 and 1.15.1. `Highs_setCallback` and `Highs_startCallback`
  are declared in `highs_api.hpp:257-259` but not loaded.
- **HiGHS, the stop is sticky.** Without a reset, the next `solve()` returns
  `interrupted` at once, on all four releases. Writing
  `user_interrupt = stop_requested` on every call fixes it.
- **SCIP, the bits moved.** `BESTSOLFOUND` went from `0x4000000` to `0x10000000` in
  SCIP 10, and `0x4000000` is now `LPSOLVED`. A mask from one header silently
  catches other events on the other, so key it on `SCIPmajorVersion()`, as
  `scip_milp.hpp:531` does for statuses. SCIP 10 also adds `DUALBOUNDIMPROVED`.
- **SCIP, a restart doubles the subscription.** Catch in `initsol` and drop in
  `exitsol`, or a restart makes each event fire twice (`NODESOLVED` 19135 against
  10089 nodes). Read `SCIPgetNTotalNodes`, as the model does; `SCIPgetNNodes` counts
  the current run only.
- **Xpress, serialization is not documented for `miplog`.** The `MUTEXCALLBACKS`
  entry of the manual lists `optnode`, `infnode`, `intsol`, `nodecutoff` and
  `prenode`, not `miplog`, which ran on 32 threads with 0 overlaps in 3301 calls at
  `MIPLOG = -1`. The wrapper should hold its own mutex around the lambda on Xpress:
  at this rate it costs nothing, and it turns a measurement into a guarantee.
- **Xpress, thread-local attributes.** `optnode` and `nodelpsolved` read a worker's
  view, with thousands of bound regressions in 5 s, so they cannot be used.
  `miplog` is clean. `checktime` read fine on 47.01, but the manual forbids it.
  `MIPLOG` defaults to every 100 nodes, so slow nodes make `miplog` sparse;
  `MIPLOG = -1` gives every node but floods a verbose log (inferred). The offset
  is the wrapper's (`xpress_milp.hpp:59`).
- **COPT, no node count.** `NodeCnt` returns success and reads 0 inside the callback
  throughout. Counting `MIPNODE` calls is exact at one thread (28860 of 28860) and
  close otherwise (5373 of 5405).
- **MOSEK, no sentinels.** "None yet" reads 0. Test
  `MSK_IINF_MIO_NUM_INT_SOLUTIONS` and `MSK_IINF_MIO_OBJ_BOUND_DEFINED`, resolved
  with `MSK_getinfindex` since item numbers are not stable across releases
  (inferred). `MSK_putcallbackfunc` is declared in `mosek_api.hpp:385` but not
  loaded.
- **GLPK, private and presolved.** `cb_func` sits in `glpk_milp::model_params`. The
  hook sees the presolved problem, without the offset. It has more than the model
  does: `glpk_milp` has neither `has_dual_bound` nor `has_node_count`, yet the hook
  reads both.
- **Gurobi, one registration does two things.** `_enable_callbacks()` sets
  `LazyConstraints = 1` (`gurobi_milp.hpp:188`). A progress-only registration must
  not.

### One callback slot

Gurobi, CPLEX, COPT, MOSEK and HiGHS take one function per model, so progress and
candidate callbacks have to share it (documented for COPT, code for the others).
Today the candidate registration simply takes the slot:

- CPLEX and COPT: register once with the union of the masks, and dispatch on
  `contextid` or `cbctx`. Both candidate and progress calls arrived on a
  `GLOBAL_PROGRESS | CANDIDATE` mask (run X2). `copt_milp.hpp:129` returns early on
  anything but `MIPSOL` and has to dispatch instead.
- CPLEX: with any generic callback attached, `CPXlpopt` fails with 1811 (run I) and
  the refiner too (run X3). The detaches at `cplex_milp.hpp:420` and the throw at
  `:398` key on the candidate callback alone, and have to become "any callback
  attached". That throw should also become a detach for a progress-only model.
- Xpress and COPT: `compute_iis()` keys its detach the same way
  (`xpress_milp.hpp:291`, `copt_milp.hpp:277`). Not measured with a progress hook;
  treat it like CPLEX, since documented IIS behaviour must not change (X1 ruling).

## Why not the candidate callback

It fires on candidates only, so a bound that moves for an hour says nothing, and
only 4 backends have it. It also moves the search (no-op lambda, one thread):

| Backend | MKP 10 x 60, nodes | Market split m = 5, 40 s |
|---|---|---|
| COPT | 28860 → **4806** (25.5 s → 0.7 s) | proven optimal 1 in 16.8 s → **time limit, value 4** |
| CPLEX | 16133 → 17120 | value 1 → **value 4** |
| Gurobi | 11035 → 11035 | 1 120 097 → 1 113 187 nodes, value 2 both |
| Xpress | 5896 → 5896 | 337 130 → 335 871 nodes, value 4 both |

The COPT mechanism is measured: `LazyConstraints = 1` without any callback gives
the same 4806 nodes, and `MIPSOL` with `LazyConstraints = 0` returns to 28860.
COPT documents this and points to `CBCONTEXT_INCUMBENT` for monitoring. The progress
hook uses `INCUMBENT | MIPNODE`, which leaves the path at 28860. The speedup is
this instance's luck, as the other column shows. CPLEX's `GLOBAL_PROGRESS`, by
contrast, keeps the path (16133), dynamic search and deterministic parallel mode.

## Otherwise: what to use without the hook

**Time slices** need no backend code: `set_time_limit` plus a re-solve, reading the
model between solves under the X1 ruling. They only work where a re-solve resumes.

| Backend | MIP slices, 8 x 1 s | Reading |
|---|---|---|
| Gurobi, CPLEX | node count keeps growing; dual -41354 and -41360 against -41364 and -41355 in one piece | **resumes** |
| SCIP | second solve returns in 0.00 s: `limits/time` counts from the transformed start | **resumes with a growing limit** (1 s, 2 s, ... 8 s reaches 6860 nodes, dual -41525.7, against 6837 and -41520.1 in one piece) |
| Xpress | ~10 000 nodes per slice, bound stuck | restarts: the wrapper's postsolve "drops the search tree" (`xpress_milp.hpp:269-272`) |
| COPT | 1 node per slice, never leaves the root | restarts |
| MOSEK, HiGHS, GLPK, Cbc | results stuck, node counts per slice where the model reads them | restart |

So slices are not a portable tool. Use them on Gurobi and CPLEX when no callback is
wanted at all, and nowhere else.

**LP slices** behave differently: GLPK (6 slices, 1.12 s against 0.89 s) and Clp
(0.85 s against 0.88 s) resume. SoPlex finishes, but in 3.6x the time (9.75 s against
2.71 s) with `solution_available = false` on the intermediate stops. COPT never
finishes: 30 slices of 0.5 s on an LP that takes 0.71 s in one piece.

**A log sink** is the widest fallback, and the only path on COPT and GLPK LPs. It
would be a separate concept, say `set_log_callback(std::string_view)`, and it fits
the never-redirect-stdout ruling better than parsing stdout.

| Backend | Hook | Lines, quiet | Lines, verbose | Print-free capture |
|---|---|---|---|---|
| Gurobi | `GRB_CB_MESSAGE` | 59 | 58 | yes, quiet |
| CPLEX | `CPXaddfuncdest` | 84 | 84 | yes, quiet |
| HiGHS | `kCallbackLogging` | 0 | 44 | yes, `output_flag` on, `log_to_console` off |
| Xpress | `XPRSaddcbmessage` | 0 | 563 | yes, `OUTPUTLOG` on, the wrapper's printing hook removed |
| MOSEK | task stream | 0 | 469 | yes, the hook replaces the wrapper's on the stream |
| SCIP | message handler | 0 | 1955 | yes, the handler takes over the output |
| COPT | `COPT_SetLogCallback` | 0 | 95 | yes, `LogToConsole = 0`; the one stray line echoes setting `Logging` before it |
| GLPK | `glp_term_hook` | 0 | 47 | yes, the hook returns 1; **not per model** (inferred from the signature) |
| Clp | `Clp_registerCallBack` | 0 | 62 lines in 0.9 s on 1.17.11, 1.3/s on the development branch | **no**: it still prints |
| Cbc 2.10.x | `Cbc_registerCallBack` | 0 | 0 from the search | no |
| SoPlex | none in the C API | | | |

"Print-free capture" is measured with stdout to a file ([out_log_capture.txt](survey/out_log_capture.txt)):
log on, hook registered, 0 lines on stdout. Every backend above Clp has a way.

Solver logs have no common format, so a sink serves display and archiving, not
arithmetic. Bounds and incumbents belong to the structured hook.

**Cbc** has no option but `set_verbose(true)`. On 2.10.9 and 2.10.13 the message
callback receives nothing from the search, verbose or not, and `Cbc_addCutCallback`
was not called in the one run tried. The development branch adds
`Cbc_addProgrCallback`, which works (quiet, offset excluded) but sparsely: 6 calls
in 10 s, with its return value ignored, so no stop. Cbc is experimental under the
IIS rulings; document it and move on.

**LP hooks**, should they ever be wanted, are a separate, weaker concept: an
iteration count and an objective with no bound meaning.

| Backend | Hook | Reads |
|---|---|---|
| Gurobi | `GRB_CB_SIMPLEX`, `GRB_CB_BARRIER` | iterations, objective (barrier's is not monotone) |
| CPLEX | legacy `CPXsetlpcallbackfunc` | iterations, objective. Refuses to coexist with a generic callback on a MIP (1060) |
| Xpress | `lplog`, `barlog` | iterations, objective |
| MOSEK | `INTPNT`, `UPDATE_*_SIMPLEX` | objective (simplex codes not exercised: the default ran the interior point) |
| HiGHS | `SimplexInterrupt` | iterations only: the objective stays `-inf`. `IpmInterrupt` not exercised |
| COPT, GLPK, SoPlex | none | |
| Clp | message callback | verbose only, and it prints |

## Questions for the maintainer

All seven recommendations were accepted on 2026-11-06.

1. **Scope.** Ship on the 8 backends and leave Cbc out? Recommended.
2. **Node count.** Optional through `has_node_count` on the handle, false on COPT?
   Recommended. The alternative is a counted `MIPNODE` on COPT, exact at one thread
   only, against the "as the solver counts them" comment at `model_concepts.hpp:308`.
3. **"None yet".** Normalized infinities? Recommended: gap arithmetic then just works,
   and each native sentinel goes away. The alternatives are `std::optional` or a
   `has_incumbent()`. The model getters stay as they are (raw sentinels, CPLEX
   throws), so the handle would be stricter than the model.
4. **Throttling.** Left to the lambda, with the snippet above in the docs? Recommended:
   no new surface, and it is one comparison. The alternative is a
   `set_progress_callback(f, min_interval)`.
5. **`*_lp`.** Defer until someone asks? Recommended: no demand yet, and the pure MILP
   ruling keeps `*_milp` clean either way.
6. **Log sink.** Separate proposal? Recommended: it is useful on its own and needs its
   own look at print-free capture per backend.
7. **Exceptions from the lambda.** Documented UB, like the candidate callback, under
   the misuse ruling? Recommended. The stronger option exists now that all 8 have a
   stop: catch, interrupt, rethrow after `solve()`.

## Side findings

Both of these were fixed after the implementation, on the maintainer's request.

- `docs/solving/status-and-limits.md:178` says a limit set before a later solve
  "applies to that solve". On SCIP the second `solve()` under the same 1 s limit
  returned in 0.00 s with `time_limit`. `solve()` now raises `limits/time` by the
  solving time already spent and restores it afterwards.
- `copt_milp::get_dual_bound()` returned `1e30` after a time limit with no
  incumbent, where the hook read `BestBnd` 26.0 just before. Both `BestBnd` and
  `ObjBound` read `+1e30` then, in either sense ([survey/probe_copt_bound.cpp](survey/probe_copt_bound.cpp)),
  so `solve()` keeps the bound from a `MIPNODE` callback. On a do-nothing callback,
  COPT's node count at one thread stayed 28860 in 6 runs and its 8 s runs at default
  threads stayed within their spread ([survey/probe_copt_overhead.cpp](survey/probe_copt_overhead.cpp)).
  The cost is COPT's only callback slot: a native callback does not run in a solve.
- `HighsCallbackDataOut` and `HighsCallbackDataIn` in `highs_api.hpp:227-253` should
  be fixed or removed whichever way this goes: they are wrong on every validated
  release, and only unused code keeps that harmless.

## Not verified

As it stood before the implementation; [How it was verified](#how-it-was-verified)
has what remains.

- **Releases:** Xpress 45.01 (license expired), COPT 7.2 (license refused), Gurobi
  10.x (not installed), CPLEX 22.1.0 and 22.1.1, MOSEK 11.1 and 11.2, GLPK below 5.0.
  The validated floors for those rest on the newer run.
- **Instances:** one family, fast nodes. Cadence on slow nodes, on long roots and in
  presolve is not measured, and neither is hook overhead on them.
- **Platforms:** Linux only. Nothing here touched MSVC.
- **Coexistence:** progress and candidate callbacks were run together on CPLEX only
  (X2), with a no-op candidate. Lazy constraints under a shared slot are untested.
