# Progress callback survey: record (2026-11-06)

Probes behind [../progress_callback_plan.md](../progress_callback_plan.md). Run
from the session scratchpad against `include/` of `feat/lp-basis` (`2dc8083`
plus the uncommitted 1.1.0 changes), unpatched. Each backend's native hook is
registered through `native_api()`, `native_model()` and, for entry points the
wrappers do not load, `dlsym`. Everything else goes through the MIP++ API.

```
/opt/gcc-15/bin/g++ -std=c++26 -O1 -I<repo>/include -I. probe_<backend>.cpp -o probe_<backend> -ldl
MIPPP_NO_VERSION_WARNING=1 timeout -k 5 300 ./probe_<backend> 2>out_<backend>.txt >log_<backend>.txt
```

`probe_glpk.cpp` also needs `-fno-access-control`: `mip.cb_func` lives in the
private `glpk_milp::model_params`. `probe_candidate_cost.cpp` takes
`-DSOLVER_HEADER='"mippp/solvers/<s>/all.hpp"' -DSOLVER_MODEL=mippp::<s>_milp`.

## What the matrix runs

[probe_common.hpp](probe_common.hpp), on Chu-Beasley multidimensional knapsacks
(`small` = 10 x 60, solved to optimality; `large` = 30 x 100, stopped by the
limit), objective offset 100:

| Run | What | Answers |
|---|---|---|
| A, B | `small`, one thread, without and with the hook | is the search path unchanged? (node counts) |
| C | `small`, one thread, no-op candidate callback | does the existing callback change the path? |
| D, E | `large`, 10 s, default threads, quiet; minimization and maximization | cadence, threads, overlaps, monotonicity, sign and offset |
| F | D with `set_verbose(true)` | does verbosity gate the hook? |
| G | market split without slacks, 5 s | what "no incumbent yet" reads as |
| H, H2 | stop asked at the 3rd sample, then a 2 s re-solve | status, latency, does the next solve work? |
| I | integer-free 400 x 600 model on the `*_milp` type | does the hook fire without a tree? |
| S0-S8 | 8 s in one piece, then 8 solves under a 1 s limit, MIP++ API only | does a re-solve resume? |
| L | an LP on the `*_lp` type | does an LP hook exist and fire? |

The recorder flags a hook that re-enters (`overlaps`), a primal or dual move in
the wrong direction (`worse-moves`) and samples at `|v| >= 1e19`. A dual "DIFFER"
in "last sample vs final" is expected when a solver rounds the final bound on an
integer objective (HiGHS -41585.39 against -41585), and on the sparse hooks,
whose last call comes before the end. Rates are one machine, one instance
family; two MOSEK runs of D differ by a factor 2 to 3 (`IM_MIO` 916 against 1844
calls, 122 k against 375 k nodes). Read them as orders of magnitude.

## Releases

| Backend | Full matrix | Quick run or other (`q_*.txt`, `out_*_<version>.txt`) | Not run |
|---|---|---|---|
| Gurobi | 12.0.1 | 11.0.3, 13.0.2 | 10.x, not installed |
| CPLEX | 22.1.2 Community | | 22.1.0, 22.1.1 |
| Xpress | 47.01 Community | | 45.01: license expired 2026-04-30 |
| COPT | 8.0 | | 7.2: refused the license on 2026-11-02 |
| MOSEK | 11.0 | | 11.1, 11.2 |
| HiGHS | 1.15.1 | 1.14.0, 1.10.0, 1.7.2 | |
| SCIP | 8.0.4 | 9.2.1, 10.1.0 | |
| GLPK | 5.0 | | 4.59 floor |
| Cbc | | 2.10.9, 2.10.13, development branch (`out_cbc_devel.txt`) | |
| Clp | | 1.17.11, development branch (`out_clp.txt`) | |
| SoPlex | 7.1.3 | | |

`PROBE_QUICK=1` runs D with a 3 s limit, H, H2 and I. Conda releases were fetched
into the scratchpad with [fetch_release.py](fetch_release.py), which reuses
`misc/tools/compat_matrix.py` with its cache redirected, so `.compat-cache` is
untouched. The conda SCIP 8.0.4 does not load here (no `libscotch-6`); the SCIP
8.0.4 above is `/usr/local/lib/libscip.so`.

## Other probes

- [probe_copt_mech.cpp](probe_copt_mech.cpp): `MIPSOL` with and without
  `LazyConstraints`, against `LazyConstraints=1` alone.
- [probe_highs_sticky.cpp](probe_highs_sticky.cpp): a re-solve after a stop, with
  and without resetting `user_interrupt`. `q_highs_sticky*.txt` hold 1.7.2 and
  1.14.0; 1.10.0 and 1.15.1 agree, output not kept as a file:

  ```
  1.15.1 rearm=false second solve, hook calls in it=1: status=interrupted{sol=true} solve=0.01s
  1.15.1 rearm=true second solve, hook calls in it=1835: status=time_limit{sol=true} solve=2.00s
  1.10.0 rearm=false second solve, hook calls in it=1: status=interrupted{sol=true} solve=0.00s
  1.10.0 rearm=true second solve, hook calls in it=1998: status=time_limit{sol=true} solve=2.00s
  ```

- [probe_log_capture.cpp](probe_log_capture.cpp): log on and hook registered,
  stdout to a file, on Xpress, MOSEK, SCIP, COPT and GLPK (`-DB_<BACKEND>`,
  [out_log_capture.txt](out_log_capture.txt)).
- CPLEX with a generic callback attached, verbose run F, `log_cplex.txt` not kept:

  ```
  Generic callback                                 0x10
  MIP search method: dynamic search.
  Parallel mode: deterministic, using up to 16 threads.
  ```

- [probe_candidate_cost.cpp](probe_candidate_cost.cpp): no-op candidate callback
  on market split m = 5 and a capacitated facility location, one thread, 40 s
  (`out_candidate_<backend>.txt`). The facility location solves at or near the
  root on all four, so only market split says anything.
- [probe_copt_lp.cpp](probe_copt_lp.cpp), [probe_soplex.cpp](probe_soplex.cpp):
  LP time slices, on 3000 x 4500.

## Pitfalls met

- The first SCIP probe caught its events in `initsol` and never dropped them: a
  restart ran `initsol` again and every event then fired twice (`NODESOLVED`
  19135 against 10089 nodes). Kept as run X3.
- Bit masks copied from the SCIP 8 header silently caught other events on 10.1
  (`BESTSOLFOUND=2136`). The record uses masks chosen by `SCIPmajorVersion()`.
- CPLEX Community stops at 1000 rows or columns, Xpress Community at 5000 in
  all: their LP runs are smaller, so their LP rates do not compare.
- The first `probe_cbc` run loaded the development library through
  `LD_LIBRARY_PATH`, not a validated release. Cbc results are by named library.
- Rerunning after a harness change: `out_cplex.txt`, `out_xpress.txt`,
  `out_copt.txt`, `out_highs.txt` and `out_glpk.txt` predate `PROBE_QUICK`, which
  does not change what a full run does.
