#!/bin/bash
cd "$(dirname "$0")"
export MIPPP_NO_VERSION_WARNING=1
export MIPPP_SOPLEX_LIBRARY=/home/plaiseek/Softwares/solvers/scipoptsuite-9.2.1/build/lib/libsoplexshared.so.7.1.3.0
for b in cplex highs xpress copt mosek clp glpk soplex gurobi; do
  timeout -k 5 600 ./probe_$b > out_$b.txt 2>&1; echo "$b exit $?"
done
