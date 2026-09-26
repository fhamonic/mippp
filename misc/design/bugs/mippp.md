# mippp 1.0.0: issues found by rest-or

A work list for a 1.0.x release. The issues were found while building rest-or on
mippp v1.0.0 (tag `v1.0.0`, which is `main` at `2da7ebe`; line numbers refer to
it). Each one is reproduced in isolation by a mode of
[`mippp_repro.cpp`](mippp_repro.cpp):

```sh
g++ -std=c++23 -O2 -I<mippp>/include mippp_repro.cpp -o mippp_repro -ldl
MIPPP_NO_VERSION_WARNING=1 ./mippp_repro <mode>   # no mode: all of them, about 10 s
```

Machine: Linux x86-64, GCC 15.1. Solvers: HiGHS 1.10.0, CBC (coinbrew master),
CLP 1.17.5, GLPK 5.0, SCIP 8.0.4, SoPlex 6.0, Gurobi 12.0.1, CPLEX 22.1.2, Xpress
(libxprs 47.1.1), COPT 8.0.5, MOSEK 11.0.14. The outputs below come from that
run. The repro discards solver logs everywhere except in the `stdout` and
`gurobi-banner` modes.

"Fix tested" means three things:

- the diff shown was applied to a copy of v1.0.0;
- the repro mode then printed the expected line;
- mippp's own tests still passed for the touched backends: `main`, `status`
  and the HiGHS, COPT, SCIP, MOSEK, Xpress and Gurobi solver files, 1153 tests,
  with identical results before and after.

## Summary

| # | Issue | Backends | Effect | Release | State |
|---|---|---|---|---|---|
| 1 | `solve()` throws when the solver stops at a limit | HiGHS (lp, milp, qp), MOSEK (lp, milp) | the caller loses the answer and its incumbent | patch | fix tested |
| 2 | `set_optimality_tolerance` sets the integrality tolerance | Xpress | fractional values accepted as integers | patch | fix tested |
| 3 | Stopped statuses say no solution while the solver has one | COPT, SCIP, MOSEK | the incumbent is reported absent | patch | fix tested |
| 4 | `solve()` optimizes twice | MOSEK (lp, milp) | every solve takes twice as long | patch | fix tested |
| 5 | Stopping at the requested gap is `limit_reached` | SCIP | the same call gives `optimal` on the other backends | patch | proposal |
| 6 | Output on stdout outside solves | Gurobi (construction, setters), COPT (setters) | corrupts a stdout protocol or JSON output | patch | fix tested (Gurobi) |
| 7 | No `set_time_limit` | SCIP, GLPK, MOSEK, CLP, SoPlex | solves cannot be bounded | minor | SCIP and MOSEK paths checked |
| 8 | No `set_optimality_tolerance` | HiGHS, GLPK, MOSEK | `optimal` on HiGHS means within 1e-4 | minor | HiGHS path checked |
| 9 | No verbosity control | all; GLPK cannot be silenced at all | 8 of 11 backends log on stdout | minor | proposal |
| 10 | No interrupt | all | only the time limit stops a solve | minor | proposal |

Issues 1 to 6 change no API. Issues 7 and 8 add members to existing wrappers, so
the corresponding `has_*` concepts turn true. That is a minor release by semver,
or a patch if you count parity between backends as a fix.

## 1. `solve()` throws when HiGHS or MOSEK stops at a limit

HiGHS returns `kHighsStatusWarning` from `Highs_run` when it stops at a limit
(verified here for the time limit). `highs_api::_check` throws on that value
(`highs_api.hpp:358-364`), and `solve()` passes it `Highs_run`'s result directly
(`highs_milp.hpp:128`, `highs_lp.hpp:90`, `highs_qp.hpp:239`).

MOSEK behaves the same way through a different path. `MSK_optimize` returns the
termination code (`MSK_RES_TRM_MAX_TIME`, ...) as its result, and `check()` turns
it into a `solver_error` (`mosek_milp.hpp:156`, `mosek_lp.hpp:94`). As a result,
the limit cases in `_get_status()` (`mosek_milp.hpp:131-138`) are never reached.

In both cases `get_status()` keeps the status of the previous solve, so
`solution_available()` is false while the solver holds an incumbent.

```
$ ./mippp_repro highs-warning
== highs-warning: HiGHS reaching its time limit throws
solve() threw solver_error("HiGHS kHighsStatusWarning")
  HiGHS model status           13 (kHighsModelStatusTimeLimit = 13)
  HiGHS primal_solution_status 2 (kHighsSolutionStatusFeasible = 2)
  HiGHS incumbent objective    7351
  mippp solution_available()   0 (status stale after the throw)
$ ./mippp_repro mosek-limit
== mosek-limit: MSK_DPAR_OPTIMIZER_MAX_TIME = 0.5 s, set natively
  model.solve() threw solver_error("The optimizer terminated at the maximum amount of time.")
    MOSEK integer solution defined=1, objective 148024
  MSK_optimizetrm() on the same task returns 0 (MSK_RES_OK = 0), trmcode 100001 (MSK_RES_TRM_MAX_TIME = 100001)
    MOSEK integer solution defined=1, objective 148024
```

**Expected:** `solve()` returns and `get_status()` is `time_limit{true}`. With the
fix, both modes print `time_limit=1 solution_available=1`.

**Fix (tested).** For HiGHS, make the same change in `highs_lp.hpp` and
`highs_qp.hpp`:

```diff
--- a/include/mippp/solvers/highs/impl/v1/highs_milp.hpp
+++ b/include/mippp/solvers/highs/impl/v1/highs_milp.hpp
@@ -125,7 +125,10 @@
             _status = status::unknown{};
             return;
         }
-        check(Highs->run(model));
+        // HiGHS returns kHighsStatusWarning when it stops at a limit; the
+        // model status tells which, so only an error is exceptional.
+        if(Highs->run(model) == kHighsStatusError)
+            throw solver_error("HiGHS kHighsStatusError");
         _status = _get_status();
     }
     double get_solution_value() { return Highs->getObjectiveValue(model); }
```

For MOSEK, call `MSK_optimizetrm` once and map its termination code. That diff is
under issue 4, which it also fixes.

**Test.** `TimeLimitTest` is not instantiated for HiGHS (`test/solvers/highs.cpp`),
which is how this shipped. Adding
`INSTANTIATE_TEST(HiGHS_milp, TimeLimitTest, highs_milp_test);` fails on v1.0.0
with `C++ exception with description "HiGHS kHighsStatusWarning" thrown in the
test body`, and passes with the fix. MOSEK can join the suite once it has
`set_time_limit` (issue 7).

**In rest-or:** `mippp_support::solve()` reads the outcome from HiGHS after the
throw (`src/problems/common/mippp_support.hpp:91-134`).

## 2. Xpress: `set_optimality_tolerance` sets the integrality tolerance

`xpress_milp::set_optimality_tolerance` writes `XPRS_MIPTOL` (7009;
`xpress_milp.hpp:157-164`, `xpress_api.hpp:129`). Xpress defines that control as
the integer feasibility tolerance. The relative gap, which the other backends
set, is `XPRS_MIPRELSTOP` (7020). So a caller who asks for a 45% gap gets
fractional values accepted as integral:

```
$ ./mippp_repro xpress-miptol
== xpress-miptol: what set_optimality_tolerance changes on Xpress
  max x, x integer, 2x <= 2.8; set_optimality_tolerance not set -> objective 1
  max x, x integer, 2x <= 2.8; set_optimality_tolerance 0.45    -> objective 1.4
```

The repro switches presolve off through `XPRSsetintcontrol`, since presolve would
first round `2x <= 2.8` to `x <= 1`. With the fix, both lines print 1.

**Fix (tested):**

```diff
--- a/include/mippp/solvers/xpress/impl/v1/xpress_api.hpp
+++ b/include/mippp/solvers/xpress/impl/v1/xpress_api.hpp
@@ -127,6 +127,7 @@
 enum DblCtrlPar : int {
     XPRS_FEASTOL = 7003,
     XPRS_MIPTOL = 7009,
+    XPRS_MIPRELSTOP = 7020,
     XPRS_TIMELIMIT = 7158
 };
 int XPRSsetdblcontrol(XPRSprob prob, int control, double value);
--- a/include/mippp/solvers/xpress/impl/v1/xpress_milp.hpp
+++ b/include/mippp/solvers/xpress/impl/v1/xpress_milp.hpp
@@ -155,11 +155,11 @@
     ////////////////////////// Tolerance parameters ///////////////////////////
     ///////////////////////////////////////////////////////////////////////////
     void set_optimality_tolerance(double tol) {
-        check(XPRS->setdblcontrol(prob, XPRS_MIPTOL, tol));
+        check(XPRS->setdblcontrol(prob, XPRS_MIPRELSTOP, tol));
     }
     double get_optimality_tolerance() {
         double tol;
-        check(XPRS->getdblcontrol(prob, XPRS_MIPTOL, &tol));
+        check(XPRS->getdblcontrol(prob, XPRS_MIPRELSTOP, &tol));
         return tol;
     }
     ///////////////////////////////////////////////////////////////////////////
```

**Test.** A get/set round trip cannot catch a wrong control id. The repro's case,
which turns presolve off natively, can become a test in `test/solvers/xpress.cpp`.

On the other backends the call sets the relative MIP gap: `cbc_milp.hpp:451`,
`cplex_milp.hpp:263`, `gurobi_milp.hpp:212`, `copt_milp.hpp:175` and
`scip_milp.hpp:424`. The exception is `cplex_lp` (`cplex_lp.hpp:37`), where it
sets the simplex reduced-cost tolerance. That is the LP meaning of the word, so
it is probably intended.

## 3. Stopped statuses say no solution while the solver has one

These wrappers build their stopped statuses without the solution flag:

- COPT: `time_limit{}`, `node_limit{}` and `interrupted{}` (`copt_milp.hpp:236-239`);
- SCIP: every stopped status (`scip_milp.hpp:461-473`);
- MOSEK: every stopped status (`mosek_milp.hpp:131-138`), reachable once issue 1
  is fixed.

`highs_milp` computes the flag (`highs_milp.hpp:96-98`), and CBC, Gurobi, CPLEX
and Xpress report it.

```
$ ./mippp_repro limit-solution      # 400 items, 10 rows, 0.2 s
  cbc_milp     time_limit=1  solution_available=1  get_solution_value()=147915
  gurobi_milp  time_limit=1  solution_available=1  get_solution_value()=147981
  cplex_milp   time_limit=1  solution_available=1  get_solution_value()=148024
  xpress_milp  time_limit=1  solution_available=1  get_solution_value()=147959
  copt_milp    time_limit=1  solution_available=0  get_solution_value()=147968
$ ./mippp_repro native-time-limit   # SCIP has no set_time_limit: limits/time set natively
  stopped after 0.50 s: time_limit=1 solution_available=0 incumbent objective 7352
$ ./mippp_repro scip-gap-limit      # first line: SCIP stopped by mippp's own set_optimality_tolerance
  scip_milp    limit_reached  solution_available=0  get_solution_value()=7349.999999999999
               (SCIP status 7, SCIP_STATUS_GAPLIMIT = 7; SCIPgetBestSol() non-null)
```

For MOSEK, the pending IIS-support branch already stops `solve()` from throwing
(see issue 4). On that branch, the same 0.5 s run returns `time_limit=1
solution_available=0`.

**Fix (tested).** With it, all three backends print `solution_available=1` in the
modes above. The flag comes from COPT's `HasMipSol` attribute, from
`SCIPgetBestSol` and from MOSEK's integer solution status. All three functions are
already in mippp's function tables.

```diff
--- a/include/mippp/solvers/copt/impl/v1/copt_milp.hpp
+++ b/include/mippp/solvers/copt/impl/v1/copt_milp.hpp
@@ -228,15 +228,18 @@
         using namespace status;
         int status_;
         check(COPT->GetIntAttr(prob, COPT_INTATTR_MIPSTATUS, &status_));
+        int has_mip_sol = 0;
+        check(COPT->GetIntAttr(prob, "HasMipSol", &has_mip_sol));
+        const bool has_sol = has_mip_sol != 0;
         switch(status_) {
             case COPT_MIPSTATUS_OPTIMAL:     return optimal{};
             case COPT_MIPSTATUS_INF_OR_UNB:  return infeasible_or_unbounded{};
             case COPT_MIPSTATUS_INFEASIBLE:  return infeasible{};
             case COPT_MIPSTATUS_UNBOUNDED:   return unbounded{};
-            case COPT_MIPSTATUS_TIMEOUT:     return time_limit{};
-            case COPT_MIPSTATUS_NODELIMIT:   return node_limit{};
+            case COPT_MIPSTATUS_TIMEOUT:     return time_limit{has_sol};
+            case COPT_MIPSTATUS_NODELIMIT:   return node_limit{has_sol};
             case COPT_MIPSTATUS_UNFINISHED:  return numerical_failure{};
-            case COPT_MIPSTATUS_INTERRUPTED: return interrupted{};
+            case COPT_MIPSTATUS_INTERRUPTED: return interrupted{has_sol};
             default:
                 return unknown{};
         }
--- a/include/mippp/solvers/scip/impl/v1/scip_milp.hpp
+++ b/include/mippp/solvers/scip/impl/v1/scip_milp.hpp
@@ -453,24 +453,25 @@
 
     status_variant _get_status() {
         using namespace status;
+        const bool has_sol = SCIP->getBestSol(model) != nullptr;
         switch(SCIP->getStatus(model)) {
             case SCIP_STATUS_OPTIMAL:       return optimal{};
             case SCIP_STATUS_INFORUNBD:     return infeasible_or_unbounded{};
             case SCIP_STATUS_INFEASIBLE:    return infeasible{};
             case SCIP_STATUS_UNBOUNDED:     return unbounded{};
-            case SCIP_STATUS_TIMELIMIT:     return time_limit{};
-            case SCIP_STATUS_MEMLIMIT:      return memory_limit{};
-            case SCIP_STATUS_NODELIMIT:     return node_limit{};
-            case SCIP_STATUS_SOLLIMIT:      return solution_limit{};
+            case SCIP_STATUS_TIMELIMIT:     return time_limit{has_sol};
+            case SCIP_STATUS_MEMLIMIT:      return memory_limit{has_sol};
+            case SCIP_STATUS_NODELIMIT:     return node_limit{has_sol};
+            case SCIP_STATUS_SOLLIMIT:      return solution_limit{has_sol};
             case SCIP_STATUS_TOTALNODELIMIT:
             case SCIP_STATUS_GAPLIMIT:
             case SCIP_STATUS_PRIMALLIMIT:
             case SCIP_STATUS_DUALLIMIT:
             case SCIP_STATUS_BESTSOLLIMIT:
-            case SCIP_STATUS_RESTARTLIMIT:   return limit_reached{};
-            case SCIP_STATUS_STALLNODELIMIT: return numerical_failure{};
+            case SCIP_STATUS_RESTARTLIMIT:   return limit_reached{has_sol};
+            case SCIP_STATUS_STALLNODELIMIT: return numerical_failure{has_sol};
             case SCIP_STATUS_TERMINATE:
-            case SCIP_STATUS_USERINTERRUPT:  return interrupted{};
+            case SCIP_STATUS_USERINTERRUPT:  return interrupted{has_sol};
             case SCIP_STATUS_UNKNOWN:
             default:
                 return unknown{};
```

The MOSEK part (`_has_integer_solution()` on every stopped status) is in the
issue 4 diff.

**Test.** In `TimeLimitTest::interrupts_long_solve`, assert
`status::solution_available(result.second)` next to the `time_limit` check
(`test/test_suites/time_limit.hpp:240`). The instance's all-zero point is
feasible, so every solver holds an incumbent after 1 s. On v1.0.0 the assertion
fails for COPT. With the fix, CBC, COPT and HiGHS pass. CPLEX, Gurobi and Xpress
skip on this machine because they close the largest instance within the limit.

**Not probed:** `highs_lp` and `highs_qp` (`highs_lp.hpp:70`, `highs_qp.hpp:219`),
`copt_lp` and `mosek_lp` also map their limits without the flag. I did not test
whether an LP solver can hold a primal feasible point at that moment.

## 4. MOSEK: `solve()` optimizes twice

`solve()` calls `MSK_optimize` (`mosek_milp.hpp:156`, `mosek_lp.hpp:94`). Then
`_get_status()` calls `MSK_optimizetrm` (`mosek_milp.hpp:98`, `mosek_lp.hpp:43`),
which optimizes again rather than reading the last termination code.

```
$ ./mippp_repro mosek-double-solve   # 300 items, 10 rows, no limit
  model.solve() 0.753 s   one MSK_optimizetrm() 0.396 s   ratio 1.90
  model.solve() 0.810 s   one MSK_optimizetrm() 0.385 s   ratio 2.10
  model.solve() 0.851 s   one MSK_optimizetrm() 0.423 s   ratio 2.01
```

With the fix, the ratios are 1.07, 0.96 and 0.99. The pending
`mheyman-astra/iis-support` branch already makes the single-`optimizetrm` change
in both files. Measured there, the ratios are 0.97, 1.05 and 0.94, so that part
can be cherry-picked. That branch still returns `time_limit{}` without the flag
(issue 3).

**Fix (tested).** Make the same `solve()`/`_get_status(trm)` change in
`mosek_lp.hpp`, without the solution helper:

```diff
--- a/include/mippp/solvers/mosek/impl/v1/mosek_milp.hpp
+++ b/include/mippp/solvers/mosek/impl/v1/mosek_milp.hpp
@@ -92,10 +92,18 @@
         return MSK_SOL_ITR;
     }
 
-    status_variant _get_status() {
+    bool _has_integer_solution() {
+        MSKbooleant defined = 0;
+        check(MSK->solutiondef(task, MSK_SOL_ITG, &defined));
+        if(!defined) return false;
+        MSKsolstae solsta;
+        check(MSK->getsolsta(task, MSK_SOL_ITG, &solsta));
+        return solsta == MSK_SOL_STA_PRIM_FEAS ||
+               solsta == MSK_SOL_STA_INTEGER_OPTIMAL;
+    }
+
+    status_variant _get_status(MSKrestrmcode trm) {
         using namespace status;
-        MSKrestrmcode trm;
-        check(MSK->optimizetrm(task, &trm));
         switch(trm) {
             case MSK_RES_OK: {
                 MSKsoltypee soltype = _pick_sol();
@@ -128,14 +136,14 @@
                         return unknown{};
                 }
             }
-            case MSK_RES_TRM_MAX_TIME:          return time_limit{};
-            case MSK_RES_TRM_MAX_ITERATIONS:    return iteration_limit{};
+            case MSK_RES_TRM_MAX_TIME:          return time_limit{_has_integer_solution()};
+            case MSK_RES_TRM_MAX_ITERATIONS:    return iteration_limit{_has_integer_solution()};
             case MSK_RES_TRM_MIO_NUM_BRANCHES:
-            case MSK_RES_TRM_MIO_NUM_RELAXS:    return node_limit{};
+            case MSK_RES_TRM_MIO_NUM_RELAXS:    return node_limit{_has_integer_solution()};
             case MSK_RES_TRM_NUM_MAX_NUM_INT_SOLUTIONS:    
-                                                return solution_limit{};
-            case MSK_RES_TRM_OBJECTIVE_RANGE:   return limit_reached{};
-            case MSK_RES_TRM_USER_CALLBACK:     return interrupted{};
+                                                return solution_limit{_has_integer_solution()};
+            case MSK_RES_TRM_OBJECTIVE_RANGE:   return limit_reached{_has_integer_solution()};
+            case MSK_RES_TRM_USER_CALLBACK:     return interrupted{_has_integer_solution()};
             case MSK_RES_TRM_NUMERICAL_PROBLEM:
             case MSK_RES_TRM_MAX_NUM_SETBACKS:
             case MSK_RES_TRM_STALL:             return numerical_failure{};
@@ -153,8 +161,10 @@
     ////////////////////////////////// Solve //////////////////////////////////
     ///////////////////////////////////////////////////////////////////////////
     void solve() {
-        check(MSK->optimize(task));
-        _status = (num_variables() > 0) ? _get_status() : status::optimal{};
+        // MSK_optimizetrm optimizes: it is the solve, not a status query.
+        MSKrestrmcode trm;
+        check(MSK->optimizetrm(task, &trm));
+        _status = (num_variables() > 0) ? _get_status(trm) : status::optimal{};
     }
     double get_solution_value() {
         double val = 0.0;
```

`MSK_optimizetrm` returns `MSK_RES_OK` when it stops at a limit (see the
`mosek-limit` output under issue 1), so this `check()` no longer throws there.

**In rest-or:** not worked around. rest-or's MOSEK solves take twice as long.

## 5. SCIP: stopping at the requested gap is `limit_reached`, not `optimal`

`set_optimality_tolerance` sets `limits/gap` (`scip_milp.hpp:424-426`). When SCIP
stops there, it reports `SCIP_STATUS_GAPLIMIT`, which mippp maps to
`limit_reached{}` (`scip_milp.hpp:466,470`). The four other backends that
implement the call answer the same request with `optimal`:

```
$ ./mippp_repro scip-gap-limit      # set_optimality_tolerance(0.01), then solve()
  scip_milp    limit_reached  solution_available=0  get_solution_value()=7349.999999999999
               (SCIP status 7, SCIP_STATUS_GAPLIMIT = 7; SCIPgetBestSol() non-null)
  gurobi_milp  optimal        solution_available=1  get_solution_value()=7301
  cplex_milp   optimal        solution_available=1  get_solution_value()=7327
  copt_milp    optimal        solution_available=1  get_solution_value()=7324
  cbc_milp     optimal        solution_available=1  get_solution_value()=7352
```

**Proposal (not tested):** `case SCIP_STATUS_GAPLIMIT: return optimal{};`. Then
`optimal` means "optimal within the requested tolerance" on every backend, which
it already means under the default gaps (issue 8). This changes the result for
code that tests `limit_reached`. If you would rather not change the category,
issue 3's fix alone (`limit_reached{true}`) keeps the answer reachable.

## 6. Output on stdout outside solves

The `stdout` mode captures file descriptor 1 around three steps: constructing a
model, calling `set_time_limit(10 s)` (`-` where there is none) and solving a
2-variable LP.

```
$ ./mippp_repro stdout
  highs   construction   0 B                           set_time_limit 0 B                                          solve  819 B "Running HiGHS 1.10.0 (git hash: fd"
  cbc     construction   0 B                           set_time_limit 0 B                                          solve  515 B "Starting solution of the Linear pr"
  clp     construction   0 B                           set_time_limit -                                            solve   79 B "Clp0006I 0  Obj 0 Dual inf 1.99999"
  glpk    construction   0 B                           set_time_limit -                                            solve  194 B "GLPK Simplex Optimizer 5.0"
  scip    construction   0 B                           set_time_limit -                                            solve 1542 B "feasible solution found by trivial"
  soplex  construction   0 B                           set_time_limit -                                            solve 1010 B "Equilibrium scaling LP (persistent"
  gurobi  construction 132 B "Set parameter Username"  set_time_limit 36 B "Set parameter TimeLimit to value 10"   solve  883 B "Gurobi Optimizer version 12.0.1 bu"
  cplex   construction   0 B                           set_time_limit 0 B                                          solve    0 B
  xpress  construction   0 B                           set_time_limit 0 B                                          solve    0 B
  copt    construction   0 B                           set_time_limit 36 B "Setting parameter 'TimeLimit' to 10"   solve 1004 B "Model fingerprint: 8078bcf9"
  mosek   construction   0 B                           set_time_limit -                                            solve    0 B
```

The byte counts depend on what ran earlier in the process. CBC, for example,
prints its welcome banner only once per process. Solve logs belong to issue 9.

Two backends write outside a solve, where no caller expects output. Gurobi
prints its licence banner on every model construction (`GRBstartenv`,
`gurobi_base.hpp:68`) and echoes every parameter change. COPT echoes parameter
changes. In a program whose stdout is a protocol, such as an MCP stdio server,
or whose output is JSON, constructing one model corrupts the stream.

**Gurobi fix (tested):** set `OutputFlag` to 0 on the empty environment, before
`GRBstartenv`. With it, the `stdout` mode prints 0 B for Gurobi in all three
columns.

```diff
--- a/include/mippp/solvers/gurobi/impl/v1/gurobi_base.hpp
+++ b/include/mippp/solvers/gurobi/impl/v1/gurobi_base.hpp
@@ -65,6 +65,7 @@
         , env(GRB->_empty_env())
         , _num_var_native_ids(0)
         , _lazy_num_constraints(0) {
+        check(GRB->setintparam(env, "OutputFlag", 0));
         check(GRB->startenv(env));
         check(GRB->newmodel(env, &model, "GUROBI", 0, nullptr, nullptr, nullptr,
                             nullptr, nullptr));
```

The model inherits the flag, so Gurobi also stops logging solves. The
`gurobi-banner` mode replays the constructor to measure the alternatives:

```
$ ./mippp_repro gurobi-banner
  as gurobi_base does                    construction  132 B "Set parameter Username"   optimize  777 B "Gurobi Optimizer version 12.0.1 buil"
  OutputFlag 0 before GRBstartenv        construction    0 B                            optimize    0 B
  same, OutputFlag 1 before GRBoptimize  construction    0 B                            optimize  813 B "Set parameter OutputFlag to value 1"
```

You could restore `OutputFlag` on the model's environment right after
`GRBnewmodel`, but that prints `Set parameter OutputFlag to value 1` (36 B)
during construction again. Restoring it just before `GRBoptimize` keeps the log
and moves that line into it. Staying quiet matches CPLEX, Xpress and MOSEK. The
choice is yours; issue 9 is the real fix.

**COPT (not tested):** setting `Logging` to 0 (`COPT_INTPARAM_LOGGING` in
`copt.h`) should silence it, but `COPT_SetIntParam` is not in mippp's COPT
function table.

**In rest-or:** file descriptor 1 is redirected to stderr before any backend is
touched (`reserve_stdout()`).

## 7. No `set_time_limit` on SCIP, GLPK, MOSEK, CLP and SoPlex

```
$ ./mippp_repro controls
  highs_milp   time_limit=1 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  cbc_milp     time_limit=1 iteration_limit=0 node_limit=1 solution_limit=1 memory_limit=0 optimality_tolerance=1
  scip_milp    time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=1
  glpk_milp    time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  gurobi_milp  time_limit=1 iteration_limit=0 node_limit=1 solution_limit=1 memory_limit=1 optimality_tolerance=1
  cplex_milp   time_limit=1 iteration_limit=0 node_limit=1 solution_limit=1 memory_limit=1 optimality_tolerance=1
  xpress_milp  time_limit=1 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=1
  copt_milp    time_limit=1 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=1
  mosek_milp   time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  highs_lp     time_limit=1 iteration_limit=1 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  clp_lp       time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  glpk_lp      time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  soplex_lp    time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
  mosek_lp     time_limit=0 iteration_limit=0 node_limit=0 solution_limit=0 memory_limit=0 optimality_tolerance=0
```

Each of these solvers has a native limit:

| Backend | Native control | In mippp's function table | Checked |
|---|---|---|---|
| SCIP | `SCIPsetRealParam(scip, "limits/time", s)` | yes, `setRealParam`, already used at `scip_milp.hpp:417,425` | yes: `native-time-limit` stops at 0.50 s |
| MOSEK | `MSK_putdouparam(task, MSK_DPAR_OPTIMIZER_MAX_TIME, s)`, with the constant equal to 50 in mosek.h 11.0 | yes, `putdouparam`; the constant is not declared (`mosek_api.hpp:153` has `MSKdparame = int`) | yes: in `mosek-limit` MOSEK stops with `MSK_RES_TRM_MAX_TIME`; usable once issue 1 is fixed |
| GLPK | `tm_lim` in milliseconds, in the private `glp_iocp`/`glp_smcp`; hard-coded to `INT_MAX` at `glpk_milp.hpp:35` and `glpk_lp.hpp:39` | n/a | no |
| CLP | `Clp_setMaximumSeconds` | no; exported by libClp 1.17.5 | no |
| SoPlex | `SoPlex_setRealParam` (time limit) | no; SoPlex 6.0 exports only `SoPlex_setIntParam`, SoPlex master's `soplex_interface.h` has it | no |

The SoPlex symbol would have to be optional, as `HIGHS_OPTIONAL_FUNCTIONS` is
(`highs_api.hpp:325`), and `set_time_limit` would then throw on an older SoPlex.
Whether a `has_time_limit` that holds at compile time may fail at run time is
your decision. Once these limits exist, instantiate `TimeLimitTest` for these
backends; its `static_assert(has_time_limit<...>)` requires the member.

## 8. No `set_optimality_tolerance` on HiGHS, GLPK and MOSEK

On HiGHS, `status::optimal` means optimal within `mip_rel_gap`, which is 1e-4 by
default, and mippp has no call to change it:

```
$ ./mippp_repro gap                 # 150 items, profits 1e5..1e6, 5 rows
  mip_rel_gap 1e-04  (default   ) optimal=1 objective 57061066
  mip_rel_gap 0      (set to 0  ) optimal=1 objective 57062866
```

Both answers are `optimal`, and they differ by 1800. The native controls are:

- HiGHS `mip_rel_gap`, through the loaded `setDoubleOptionValue`; the repro sets
  it;
- GLPK `mip_gap`, in the private `glp_iocp`, hard-coded to 1e-4 at
  `glpk_milp.hpp:37`;
- MOSEK `MSK_DPAR_MIO_TOL_REL_GAP`, 48 in mosek.h 11.0.

Only the HiGHS one was tested. A sentence in the documentation saying that
`optimal` means "within the backend's relative gap tolerance" would help callers
who need exact optima.

**In rest-or:** algorithms that claim exactness set `mip_rel_gap` to 0 natively
(`mippp_support::require_zero_gap()`, `src/problems/common/mippp_support.hpp:151-164`).

## 9. Verbosity control (feature)

The `stdout` table under issue 6 shows that 8 of 11 backends log their solves on
stdout by default, while CPLEX, Xpress and MOSEK stay silent. mippp has no
portable switch.

GLPK cannot be silenced even through its native handle.
`msg_lev = GLP_MSG_ALL` is written into a private parameter struct, which is
passed to every solve (`glpk_milp.hpp:21,30,107`; `glpk_lp.hpp:19,29,76`).

A `set_verbosity`/`has_verbosity` pair, or a log sink, would map to controls
that exist:

- HiGHS `output_flag` (rest-or sets it);
- Gurobi `OutputFlag` (measured above);
- GLPK `msg_lev`;
- COPT `Logging`;
- MOSEK `MSK_IPAR_LOG` (34 in mosek.h 11.0);
- CBC and CLP `Cbc_setLogLevel`/`Clp_setLogLevel` (exported, not loaded).

A thread-count control belongs in the same batch: rest-or runs several solves
at once, and each backend picks its own default.

## 10. Interrupt (feature)

Nothing stops a running solve except its time limit, so a caller that cancels
work, for example a client that disconnects, has to wait for it. The libraries
on this machine export these entry points:

- HiGHS `Highs_setCallback`;
- SCIP `SCIPinterruptSolve`;
- GLPK `glp_ios_terminate`, from its MIP callback;
- Gurobi `GRBterminate`;
- CPLEX `CPXsetterminate`;
- Xpress `XPRSinterrupt`;
- COPT `COPT_Interrupt`;
- MOSEK `MSK_putcallbackfunc`, whose callback returns nonzero to stop.

I did not check CBC, CLP or SoPlex. An `interrupt()` callable from another
thread, or `solve(std::stop_token)`, behind a `has_interrupt` concept, would fit
the existing style.

## Not verified

- The LP-side limit statuses listed under issue 3.
- The GLPK, CLP and SoPlex time limits (issue 7), and the GLPK and MOSEK gap
  controls (issue 8). The MOSEK constants come from mosek.h 11.0 and are not
  declared by mippp.
- COPT `Logging` (issue 6) and the verbosity and interrupt entry points
  (issues 9 and 10). For those I only checked that the symbols are exported or
  the constants are defined.
- `TimeLimitTest` ran on CPLEX, Gurobi and Xpress, but skipped on this machine,
  so the suite does not exercise their limits here. The repro's
  `limit-solution` mode does.
