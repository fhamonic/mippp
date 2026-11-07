#pragma once
// The plan's "Preparing a cut" as one shared step, run by every backend's recipe
// before its native call. Steps, in this order:
//   1. reject a non-finite coefficient or side (the GLPK skeptic's item 1; not in the plan's table)
//   2. fold duplicate columns (skipped by the distinct_variables overload)
//   3. strip exact zeros, +0.0 and -0.0 (after folding: x - x folds to a zero)
//   4. skip what is left empty
//   5. scale to max|a| = 1
//   6. split == into <= and >= where the solver needs it (Xpress, GLPK)
//   7. drop each resulting cut violated by at most drop_tol at the point
// Split comes before the drop so that the satisfied half of a violated equality is
// not passed on: the plan's table lists the split last, after the violation test.
// Returns 0, 1 or 2 cuts. Backend-only steps (SCIP tiny terms, CPLEX same point,
// GLPK stall) stay in the backend recipes.
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <format>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "cut_common.hpp"

namespace fp {
struct options {
    bool fold = true;
    bool scale = true;
    double drop_tol = 1e-6;
    bool split_eq = false;
    bool reject_nonfinite = true;
    bool geometric = false;  // experiment only: divide by sqrt(max|a| min|a|), the Cbc skeptic's choice
};
struct counters {
    std::atomic<long> offered{0}, nonfinite{0}, folded{0}, zeros{0}, empty{0}, rescaled{0}, split{0}, dropped{0}, passed{0};
    void reset() { for(auto * a : {&offered, &nonfinite, &folded, &zeros, &empty, &rescaled, &split, &dropped, &passed}) a->store(0); }
    std::string line() const {
        return std::format("prep(offered={} passed={} dropped={} rescaled={} folded={} zeros={} empty={} split={} nonfinite={})",
            offered.load(), passed.load(), dropped.load(), rescaled.load(), folded.load(), zeros.load(), empty.load(), split.load(), nonfinite.load());
    }
};
// What every recipe reads: experiments change it between solves, never during one.
inline options & opts() {
    static options o = [] {
        options r;
        if(const char * e = std::getenv("FP_NO_SCALE")) r.scale = std::atoi(e) == 0;
        if(const char * e = std::getenv("FP_DROP")) r.drop_tol = std::atof(e);
        return r;
    }();
    return o;
}
inline counters & stats() { static counters c; return c; }

inline std::vector<cp::raw_cut> prepare(const cp::raw_cut & in, const std::vector<double> & x, const options & o = opts(), counters & c = stats()) {
    c.offered.fetch_add(1, std::memory_order_relaxed);
    std::vector<cp::raw_cut> out;
    if(o.reject_nonfinite) {
        bool bad = !std::isfinite(in.rhs);
        for(double v : in.val) bad = bad || !std::isfinite(v);
        if(bad) { c.nonfinite.fetch_add(1, std::memory_order_relaxed); return out; }
    }
    cp::raw_cut cut{{}, {}, in.sense, in.rhs};
    if(o.fold) {
        std::map<int, double> folded;
        for(std::size_t k = 0; k < in.ind.size(); ++k) folded[in.ind[k]] += in.val[k];
        if(folded.size() != in.ind.size()) c.folded.fetch_add(1, std::memory_order_relaxed);
        for(auto [j, v] : folded) {
            if(v != 0.0) { cut.ind.push_back(j); cut.val.push_back(v); }
            else c.zeros.fetch_add(1, std::memory_order_relaxed);
        }
    } else {
        for(std::size_t k = 0; k < in.ind.size(); ++k) {
            if(in.val[k] != 0.0) { cut.ind.push_back(in.ind[k]); cut.val.push_back(in.val[k]); }
            else c.zeros.fetch_add(1, std::memory_order_relaxed);
        }
    }
    if(cut.ind.empty()) { c.empty.fetch_add(1, std::memory_order_relaxed); return out; }
    if(o.scale) {
        double big = 0, small = std::numeric_limits<double>::infinity();
        for(double v : cut.val) { big = std::max(big, std::fabs(v)); small = std::min(small, std::fabs(v)); }
        if(o.geometric) big = std::sqrt(big) * std::sqrt(small);
        if(big != 1.0) {
            for(double & v : cut.val) v /= big;
            cut.rhs /= big;
            c.rescaled.fetch_add(1, std::memory_order_relaxed);
            if(!std::isfinite(cut.rhs)) { c.nonfinite.fetch_add(1, std::memory_order_relaxed); return out; }
        }
    }
    if(cut.sense == 'E' && o.split_eq) {
        c.split.fetch_add(1, std::memory_order_relaxed);
        cp::raw_cut lo = cut, hi = cut;
        lo.sense = 'L'; hi.sense = 'G';
        out.push_back(std::move(lo));
        out.push_back(std::move(hi));
    } else out.push_back(std::move(cut));
    std::erase_if(out, [&](const cp::raw_cut & r) {
        const bool drop = !(r.violation(x) > o.drop_tol);
        if(drop) c.dropped.fetch_add(1, std::memory_order_relaxed);
        return drop;
    });
    c.passed.fetch_add(static_cast<long>(out.size()), std::memory_order_relaxed);
    return out;
}
}  // namespace fp
