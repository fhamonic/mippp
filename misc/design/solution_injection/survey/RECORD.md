# Solution-injection survey: record (2026-11-11)

Probes behind [../solution_injection_plan.md](../solution_injection_plan.md), run from the
session scratchpad against `include/` of `main` (`080045d`), unpatched. No library file
changed. The model, the candidate callback and its handle are MIP++'s. The injection is the
native call, made from inside that callback through the handle's private members, hence
`-fno-access-control`.

```
/opt/gcc-15/bin/g++ -std=c++26 -O1 -fno-access-control -I<repo>/include -I. probe_<backend>.cpp -o probe_<backend> -ldl
MIPPP_NO_VERSION_WARNING=1 timeout -k 5 900 ./probe_<backend> variant=2 n=60 m=200 budget=0.1 seeds=5 thr=1 limit=30 modes=none,full,partial,lie
```

The shell needs the solver exports of `~/.bashrc` (library paths and licences), which a
session shell lacks. Another Gurobi release is chosen with `MIPPP_GUROBI_LIBRARY`.

## Files

| File | What |
|---|---|
| `benders_common.hpp` | The instance, the Benders callback, the counters and the report line |
| `probe_gurobi.cpp`, `probe_cplex.cpp`, `probe_copt.cpp` | The native injection call of each backend |
| `recheck_common.hpp`, `recheck_*.cpp` | Two small models: an injected point that breaks a rule only the callback knows, and a lazy constraint the candidate satisfies |
| `satlazy_gurobi.cpp` | The candidates Gurobi offers when every call adds a satisfied lazy constraint |
| `writable_check.cpp` | Which candidate handles already return a point that can be written |
| `summarize.py` | Report lines to a table |
| `out3_*_thr1.txt` | 5 seeds, one thread, 30 s, four modes |
| `out4_*_thr4.txt` | 3 seeds, four threads, 30 s, two modes |
| `out_biglie.txt` | Every rejected candidate answered with a neighbour injected at half its true value |
| `out_recheck.txt` | The small models, on every backend and on Gurobi 11.0.3 and 13.0.2 |

## The instance

Facility location: binaries `y`, opening costs under a budget (`variant=1`, `2`) or in the
objective (`variant=0`), assignment cost through epigraph variables. `variant=2` has one
epigraph variable for the whole cost, and is the only variant with a search long enough to
measure. Costs are integers.

The cut for customer `j` at a candidate with nearest open facility at distance `t` is
`eta_j >= t - sum_i max(t - c_ij, 0) y_i`. It is valid for every `y` with a facility open and
tight at the candidate.

## Modes

| Mode | What the callback does after adding its lazy constraints |
|---|---|
| `none` | nothing |
| `full` | injects the candidate with the true epigraph value, when that improves on the best the callback has accepted |
| `partial` | the same, with the epigraph entries left undefined |
| `lie` | injects a neighbour (one facility closed) one unit below its true value; with `BIG_LIE=1`, at half its true value |

CPLEX takes its strategy from `STRAT` (`-1` nocheck, `0` checkfeas, the default) and, for
partial points, from `PSTRAT` (default `2`, solve).

## Counters

- `lost_improving`: rejected candidates whose integer part, at its true value, beats the best
  the callback has accepted.
- `refound`: accepted candidates whose integer part had been rejected earlier and never
  injected. `refound_next`: accepted in the call right after their rejection.
- `inj_rej_again`: rejected candidates whose integer part had been injected.
- `marked_*`: injected points carry the fraction 0.123456789 in the epigraph value, which no
  candidate of the solver's own has, since costs are integers. `marked_rej` counts injected
  points the callback saw and rejected.
- `unchecked_inc`: incumbent values reported by the progress callback that carry the mark
  although the candidate callback never accepted a point with that value.
- `ti_1pct`, `ti_opt`: seconds until the incumbent the progress callback reports is within 1%
  of the reference, and equal to it. The reference is the final value of the first mode run on
  that seed. CPLEX's progress callback is rare, so its times are coarse.

## Harness faults found on the way

- A first marker was a free binary column with a small cost, set to 1 by injected points only.
  On Gurobi the 20 points injected that way had no effect at all: the run was the same, node
  for node, as without injection. The marker moved to the epigraph value.
- A one-unit lie rarely improves the incumbent, so few reach the callback and none can corrupt
  the final value. `BIG_LIE` is the mode with power.
- `out3_*` came from the harness just before `BIG_LIE` was added. The other modes did not
  change.
- Timings were taken with up to three one-thread probes running side by side on 16 cores.
