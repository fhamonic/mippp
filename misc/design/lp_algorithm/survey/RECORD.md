# LP algorithm survey: record (2026-11-02)

Probes behind [../lp_algorithm_plan.md](../lp_algorithm_plan.md). Run from the
session scratchpad against a copy of `include/` (tree of `feat/lp-basis`,
`2dc8083` plus the uncommitted 1.1.0 changes) patched by
[hooks.patch](hooks.patch): `clp_lp`, `cplex_lp` and `xpress_lp` get a global
hook choosing the routine or the `flags` argument that `solve()` calls, and
`glpk_lp::model_params` is made public. The other five backends are driven
through `native_api()` only.

Build and run (one binary per backend, `-ldl` for the extra symbols resolved
with `dlopen(library_path(), RTLD_NOLOAD)`):

```
/opt/gcc-15/bin/g++ -std=c++26 -O1 -Iinc -I. probe_<backend>.cpp -o probe_<backend> -ldl
MIPPP_NO_VERSION_WARNING=1 timeout -k 5 600 ./probe_<backend>
```

## What each probe does

[probe_common.hpp](probe_common.hpp) runs, for every configuration of a
backend, each on a fresh model with a 10 s time limit where the model has one:

1. a 120 x 80 feasible LP (max, `A >= 0`, `0 <= x <= 10`): status, solve
   time, the solver's own iteration counters (the witness of which algorithm
   ran), whether the objective, solution, duals, reduced costs and basis can
   be read;
2. the same rows plus `a'x >= 1.2 max a'x`, infeasible in a way presolve did
   not detect on any backend;
3. a 120 x 80 unbounded LP whose ray is all ones (the last column cancels the
   others in every row), which no presolve detected either;
   `refine_lp_status()` follows 2 and 3 where the model has it;
4. `set_iteration_limit(1)` on LP 1, where the model has it;
5. `set_basis(get_basis())` after an optimal default solve, then the
   configuration, then `solve()`;
6. the configuration written after a default solve, then `get_solution()`;
7. the five LPs of `test/test_suites/lp_status.hpp` (`max_bounded`,
   `min_bounded`, `max_unbounded`, `min_unbounded`, `max_infeasible`), with
   the suite's refine step and a 5 s limit, printed pass/FAIL.

## Releases

| Backend | Library | Notes |
|---|---|---|
| Clp | coinbrew `dist/lib/libClp.so` (devel) | no 1.17.x release installed |
| GLPK | system `libglpk.so`, 5.0 | |
| SoPlex | 7.1.3 (scipoptsuite 9.2.1) | |
| HiGHS | 1.15.1 local build | knobs at 1.7.2 and 1.10.0 read from git tags |
| Gurobi | 12.0.1 (full), 11.0.3 and 13.0.2 (`out_gurobi_<ver>.txt`) | 10.x not installed |
| CPLEX | 22.1.2 Community | |
| Xpress | 47.01 and 45.01 | |
| COPT | 8.0 | 7.2.5 refuses the license here (`out_copt_72.txt`) |
| MOSEK | 11.0 | |

## Output files

- `out_<backend>.txt`: the final run, with the `lp_status` instances.
- `out_gurobi_12.txt`, `out_xpress_47.txt`, `out_copt_80.txt`: earlier runs of
  the same probes before the suite instances were added; the Xpress one prints
  `LPSTATUS`, `SOLSTATUS`, `STOPSTATUS` and `SOLVESTATUS`.
- `out_copt.txt` stops at `LpMethod 6 PDLP`: the process died of SIGFPE.
  [probe_copt_pdlp.cpp](probe_copt_pdlp.cpp) runs one suite instance per
  process: instances 0 to 3 pass, instance 4 (`max_infeasible`) dumps core,
  every time.
- [probe_highs_tiny.cpp](probe_highs_tiny.cpp): `max_bounded` under
  `solver=ipm|ipx|pdlp`, `run_crossover=off|on`. Output, not kept as a file:

  ```
  ipm off: solve ok status=unknown model_status=15 primal_sol=2 dual_sol=1 ipm=0
  ipm on:  solve ok status=optimal model_status=7 primal_sol=2 dual_sol=2 ipm=0
  ipx off: solve ok status=unknown model_status=15 primal_sol=2 dual_sol=1 ipm=0
  pdlp off: solve ok status=unknown model_status=15 primal_sol=2 dual_sol=1 ipm=0
  ```

- [probe_defaults.cpp](probe_defaults.cpp): HiGHS `solver=simplex` with
  `simplex_strategy` 0, 1 and 4, and the native defaults of fresh models.
  Output, not kept as a file:

  ```
  highs fresh: simplex_strategy=1
  highs simplex strategy 0 LP 0: status=optimal simplex=92      (= primal)
  highs simplex strategy 0 LP 1: status=infeasible simplex=105  (= dual)
  highs simplex strategy 0 LP 2: status=unbounded simplex=106   (= primal)
  highs simplex strategy 1: 109 / 105 / 251, strategy 4: 92 / 72 / 106
  lp_status suite: pass under all three strategies
  xpress fresh: LPFLAGS=0 CROSSOVER=-1   (47.01 and 45.01)
  ```

- The first Gurobi run without a time limit on the suite instances never
  returned under `Method 6`: killed after about 4 minutes at 100 % CPU.

## Pitfalls met

- `pkill -f run_all.sh` matched the shell running it: kill probes by PID.
- SoPlex's counts are equal for `ALGORITHM` 0 and 1 under every
  representation; the parameter reads back as written. SoPlex maps it to its
  entering or leaving type (`soplex.hpp:8404-8418` in 9.2.1); no probe here
  showed an effect.

# Fuzz survey and review probes: record (2026-11-11)

Run on the implementation, on `feat/lp-algorithm`, with the libraries of the
table above, to answer what the probes of 2026-11-02 could not: they ran one
cold solve per configuration on seven fixed LPs.

## Fuzz survey

[fuzz_survey.sh](fuzz_survey.sh) runs
`LpAlgorithmTest.random_operations_under_each_algorithm` of every backend
under one seed per process, `MIPPP_FUZZ_SEED` 1 to 1500, 40 rounds each. The
test is the state machine of `test/test_suites/lp_fuzzy_tests.hpp`: random
variables, rows, objectives, bounds, removals and solves, each solve compared
with `dumb_lp`, a reference model solved by Clp's primal simplex, on
optimality, objective value and feasibility of the point. It runs once per
tag the model lists.

```
./fuzz_survey.sh <mippp_test> <output directory> 1 1500
```

[out_fuzz_survey.txt](out_fuzz_survey.txt) lists the failing seeds of two
runs. The first ran while `clp_lp` had a selector and `copt_lp` listed
`interior_point_without_crossover`:

| Model | Tag | Failing seeds of 1500 | What the solver said |
|---|---|---|---|
| `copt_lp` | `interior_point_without_crossover` | 26 | `optimal`, with a value, on an LP the reference calls infeasible or unbounded |
| `clp_lp` | `dual_simplex` | 2 | `optimal` at -1.1e11 on an unbounded LP (409); `infeasible` on an LP of optimum 8 (1124) |
| `gurobi_lp` | `primal_simplex` | 1 | `unbounded` on an LP of optimum -34.5 (98) |
| `highs_lp` | `primal_simplex` | 1 | `unknown` on an LP of optimum -31 (1090) |

Every other tag of every model passed the 1500 seeds, `automatic` included.
The second run, on the final tree, fails on seeds 98 and 1090 alone.

## Probes behind the rulings

Built as the probes above, against the final `include/`:

- [probe_fuzz_copt.cpp](probe_fuzz_copt.cpp),
  [out_fuzz_copt.txt](out_fuzz_copt.txt): the LP of a failing COPT run,
  built directly. It is feasible and unbounded. COPT's default, its dual
  simplex and its barrier with crossover say `unbounded`; its barrier
  without crossover says `LpStatus` 1, `HasLpSol` 1, `LpObjval` -35.63.
  Clp's primal simplex says `infeasible` on the same LP.
- [probe_fuzz_gurobi.cpp](probe_fuzz_gurobi.cpp),
  `out_fuzz_gurobi_<release>.txt`: the operations of seed 98. Ten columns,
  no row, every column with a cost bounded in the improving direction. After
  a first solve, the primal simplex of 11.0.3 and 12.0.1 ends `Status` 5 in
  0 iterations; without the first solve it ends optimal, and so does 13.0.2
  in both cases, in 5 iterations.
- [probe_fuzz_clp_highs.cpp](probe_fuzz_clp_highs.cpp),
  [out_fuzz_clp_highs.txt](out_fuzz_clp_highs.txt): `Clp_dual` called on
  the native model of seed 1124's LP, fresh, ends on status 1 after one
  iteration where `Clp_primal` ends on 0 with the objective 2; HiGHS's
  primal simplex ends on model status 15 after 7 iterations on seed 1090's
  LP when a first solve came before, and optimal without it.
- [probe_both_infeasible.cpp](probe_both_infeasible.cpp),
  [out_both_infeasible.txt](out_both_infeasible.txt): what every model says,
  under each tag, on an LP that is primal and dual infeasible. `gurobi_lp`
  ends `unbounded` after `refine_lp_status()` and `mosek_lp` reports
  `unbounded`, except under its crossover-less interior point.
- [probe_cplex_threads.cpp](probe_cplex_threads.cpp),
  [out_cplex_threads.txt](out_cplex_threads.txt): cores `cplex_lp` keeps
  busy on the 900-column dense LP of `ThreadLimitTest`, under each tag, at
  the default thread limit and at 1.

## Pitfalls met

- The fuzz seed is not portable: the distributions of the standard library
  draw other sequences on another implementation.
- On `lp_basis_cases::build_dense`, whose slack basis is primal feasible,
  `Clp_dual` took the 20 iterations of `Clp_primal` and `Clp_algorithm` read
  primal after both: the witness LP of the backend tests is
  `lp_algorithm_cases::build_dense_covering`, whose slack basis is dual
  feasible.
- A tag that only `detail::overloaded` rows encode falls back to its base
  tag's row; a requires-expression outside a template is a hard error on an
  ill-formed call, so the negative setter checks go through a concept.
