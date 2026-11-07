# User-cut survey: record (2026-11-07)

Probes behind [../user_cuts_plan.md](../user_cuts_plan.md), run from the session scratchpad
against `include/` of `feat/lp-basis` (`36878b2` plus the uncommitted 1.1.0 changes), unpatched.
No library file changed. Hooks are native, through `native_api()`, `native_model()`, `dlsym`.

```
/opt/gcc-15/bin/g++ -std=c++26 -O1 -I<repo>/include -I. probe_<backend>.cpp -o probe_<backend> -ldl
MIPPP_NO_VERSION_WARNING=1 timeout -k 5 300 ./probe_<backend> 2>out_<backend>.txt >log_<backend>.txt
```

GLPK and Cbc probes also need `-fno-access-control` or `#define private public` around the wrapper
include, to reach `mip.cb_func` and `mip_copy`. COPT's reach around `solve()`, which always takes its
one callback slot.

## Phases

| Phase | Files | What |
|---|---|---|
| Harness | `cut_common.hpp` | instances (gadgets, MTZ ATSP, knapsack), `scope_tracker`, `run_matrix`: E1-E5, S0-S2, P, W |
| Survey | `probe_*`, `survey_results.json` | one agent per backend, Q1-Q14 |
| Conformance | `cut_conformance.hpp`, `conf_gurobi.cpp` | `cf::sanitize`, cases C1-C10 |
| Skeptics | `verify_*`, `verify_results.json` | one agent per backend: refute the claims, run the recipe, remove each piece |
| Review | `final_*`, `review_results.json` | the single preparation (`final_prepare.hpp`) run on six backends; a fact check, an API attack, a gap search |
| Own runs | `own_*.cpp` | shipped-code defects reproduced again in a separate run, through the public API |

`survey_results.json` and `verify_results.json` are the agents' structured results and the record of
every claim, since report files were refused by the agents' Write tool. `raw_outputs.tar.xz` holds the
outputs of the first phases (`out_*`, `vout_*`), and `raw_outputs_review.tar.xz` those of the review
(`final_*`). Downloaded releases and vendor documents are not kept here.

## The scope instance

Independent gadgets, each a binary whose two continuous variables peak at opposite ends of y, so that
probing only tightens bounds and the LP keeps y strictly inside (0,1): y integral means branching fixed it.
A first version with one tent per binary closed at the root on Gurobi, by probing, and the hook never ran.
Three further gadgets, free only in the worse subtree, make its LP bound better, so that best-first visits
it first. Without them the optimum is already the incumbent, and a global cut cannot lose it. Read the
tracker's counts along with the final value: a filtered cut also keeps the optimum, and "HELD IN PART" on S2
is usually the hull cut holding by itself once y3 is fixed.

## Harness limits found by the agents

- `atsp::separate` packs sets in 64 bits: n <= 64, now an abort.
- The `P` block needs `set_constraint_rhs`, absent on several backends, so it was silently skipped there;
  those agents ran their own.
- E4 has little power: complements are an invertible aggregation. Parallel general integers and fixed
  columns first are what bite.
- C4 needs Euclidean costs, C7 has no `==`, C8 does not check its baseline reached optimal.
- Most of C1-C4, C6, C7, C9 close at the root.

## Status of the pipeline run

The plan's single preparation ran as one function (`final_prepare.hpp`) on Gurobi 12.0.1, CPLEX 22.1.2,
Xpress 47.01.01, COPT 8.0.5, SCIP 9.2.1 and GLPK 5.0, at the skeptics' sizes, with no recipe failure.
Other releases ran the skeptics' own variants only.
