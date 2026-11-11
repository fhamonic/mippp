#!/bin/bash
# One process per seed, every backend's LpAlgorithmTest fuzz under that seed.
BIN=$1; OUT=$2; FROM=$3; TO=$4
for seed in $(seq $FROM $TO); do
    MIPPP_NO_VERSION_WARNING=1 MIPPP_FUZZ_SEED=$seed timeout -k 5 120 $BIN \
        --gtest_filter='*LpAlgorithmTest*random_operations*' > $OUT/seed_$seed.log 2>&1
    echo "seed $seed exit $?" >> $OUT/exits.txt
done
