// Follow-ups to probe_basis.cpp: stored statuses after bound edits, Reset /
// compute_iis on an infeasible model, SetBasis on a solved model, time limit
// 0 on a warm basis, free rows, BarIterLimit stop.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/solvers/copt/all.hpp"

using namespace mippp;
using namespace mippp::copt::impl::v1;
using params_t = model_variable_params_t<copt_lp>;
using variable = model_variable_t<copt_lp>;
using constraint = model_constraint_t<copt_lp>;

constexpr int LOWER = 0, BASIC = 1, UPPER = 2, SUPERBASIC = 3, FIXED = 4;
constexpr int UNTOUCHED = -77;
static const char * code_name(int c) {
    switch(c) {
        case LOWER: return "LOWER";
        case BASIC: return "BASIC";
        case UPPER: return "UPPER";
        case SUPERBASIC: return "SUPERBASIC";
        case FIXED: return "FIXED";
        case UNTOUCHED: return "untouched";
        default: return "?";
    }
}
struct nat {
    const copt_api & api;
    copt_prob * prob;
    explicit nat(copt_lp & m)
        : api(m.native_api()), prob(m.native_model().second) {}
    int iattr(const char * n) {
        int v = -999;
        int r = api.GetIntAttr(prob, n, &v);
        return r ? -1000 - r : v;
    }
    double dattr(const char * n) {
        double v = NAN;
        int r = api.GetDblAttr(prob, n, &v);
        return r ? -1000.0 - r : v;
    }
    int setint(const char * n, int v) { return api.SetIntParam(prob, n, v); }
    int get_basis(std::vector<int> & c, std::vector<int> & r) {
        c.assign(static_cast<std::size_t>(iattr("Cols")), UNTOUCHED);
        r.assign(static_cast<std::size_t>(iattr("Rows")), UNTOUCHED);
        return api.GetBasis(prob, c.data(), r.data());
    }
    int set_basis(const std::vector<int> & c, const std::vector<int> & r) {
        return api.SetBasis(prob, c.data(), r.data());
    }
    std::string state() {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "LpStatus=%d HasBasis=%d HasLpSol=%d SimplexIter=%d "
                      "BarrierIter=%d",
                      iattr("LpStatus"), iattr("HasBasis"), iattr("HasLpSol"),
                      iattr("SimplexIter"), iattr("BarrierIter"));
        return buf;
    }
};
static std::string codes(const std::vector<int> & v) {
    std::string s;
    for(int c : v) s += std::string(code_name(c)) + " ";
    return s;
}
static int count_basic(const std::vector<int> & v) {
    return static_cast<int>(std::ranges::count(v, BASIC));
}

struct conv_handles {
    variable x1, x2, x3, x4, y, z, f, l;
    constraint r1, r2, r3, r4, r5;
};
static conv_handles build_conv(copt_lp & m) {
    using namespace operators;
    m.set_time_limit(std::chrono::seconds(10));
    conv_handles h;
    h.x1 = m.add_variable();
    h.x2 = m.add_variable();
    h.x3 = m.add_variable();
    h.x4 = m.add_variable();
    h.y = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
    h.z = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 2, .upper_bound = 2});
    h.f = m.add_variable(params_t{.obj_coef = 0, .lower_bound = std::nullopt,
                                  .upper_bound = std::nullopt});
    h.l = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 0,
                                  .upper_bound = std::nullopt});
    h.r1 = m.add_constraint(h.x1 + h.x2 <= 4);
    h.r2 = m.add_constraint(h.x3 + h.x4 >= 2);
    h.r3 = m.add_constraint(h.x1 - h.x3 == 1);
    h.r4 = m.add_constraint(h.x2 + h.x4 <= 3);
    m.set_constraint_lower_bound(h.r4, 1);
    h.r5 = m.add_constraint(h.x1 + h.x2 + h.x3 + h.x4 + h.y + h.z + h.l <= 100);
    m.set_minimization();
    m.set_objective(-0.5 * h.x1 - 2.0 * h.x2 + 0.5 * h.x3 + 0.0 * h.x4 -
                    1.0 * h.y + 1.0 * h.z + 1.0 * h.l);
    return h;
}

static void build_dense(copt_lp & m, std::size_t size = 200) {
    using namespace operators;
    m.set_time_limit(std::chrono::seconds(20));
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(1, 1000);
    auto columns = std::views::iota(std::size_t{0}, size);
    auto x = m.add_variables(size);
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    m.set_maximization();
    m.set_objective(xsum(columns, [&](auto j) { return row[j] * x(j); }));
    for(std::size_t i = 0; i < size; ++i) {
        for(int & a : row) a = coef(rng);
        m.add_constraint(xsum(columns, [&](auto j) { return row[j] * x(j); }) <=
                         1000.0 * static_cast<double>(size));
    }
}

int main() {
    std::printf("library: %s\n", copt_api::load().library_path().c_str());
    // 1. stored status after bound edits on the solved convention LP
    std::printf("\n=== 1. stored status after bound edits (solved conv LP) ===\n");
    const std::vector<std::pair<const char *, std::function<void(copt_lp &, conv_handles &)>>>
        edits = {
            {"y (UPPER) ub 3 -> 2", [](copt_lp & m, conv_handles & h) { m.set_variable_upper_bound(h.y, 2); }},
            {"y (UPPER) ub 3 -> 5", [](copt_lp & m, conv_handles & h) { m.set_variable_upper_bound(h.y, 5); }},
            {"y (UPPER) ub 3 -> 3 (same value)", [](copt_lp & m, conv_handles & h) { m.set_variable_upper_bound(h.y, 3); }},
            {"y (UPPER) lb 0 -> 1", [](copt_lp & m, conv_handles & h) { m.set_variable_lower_bound(h.y, 1); }},
            {"y (UPPER) ub 3 -> inf", [](copt_lp & m, conv_handles & h) { m.set_variable_upper_bound(h.y, m.infinity()); }},
            {"l (LOWER) lb 0 -> 1", [](copt_lp & m, conv_handles & h) { m.set_variable_lower_bound(h.l, 1); }},
            {"l (LOWER) lb 0 -> -inf (becomes free)", [](copt_lp & m, conv_handles & h) { m.set_variable_lower_bound(h.l, -m.infinity()); }},
            {"z (FIXED) ub 2 -> 4", [](copt_lp & m, conv_handles & h) { m.set_variable_upper_bound(h.z, 4); }},
            {"x1 (BASIC) ub inf -> 1", [](copt_lp & m, conv_handles & h) { m.set_variable_upper_bound(h.x1, 1); }},
            {"f (SUPERBASIC) lb -inf -> 0", [](copt_lp & m, conv_handles & h) { m.set_variable_lower_bound(h.f, 0); }},
            {"R1 (UPPER) ub 4 -> 5", [](copt_lp & m, conv_handles & h) { m.set_constraint_upper_bound(h.r1, 5); }},
            {"R2 (LOWER) lb 2 -> 1", [](copt_lp & m, conv_handles & h) { m.set_constraint_lower_bound(h.r2, 1); }},
            {"R3 (FIXED) ub 1 -> 2 (becomes ranged)", [](copt_lp & m, conv_handles & h) { m.set_constraint_upper_bound(h.r3, 2); }},
            {"R4 (UPPER) ub 3 -> inf (becomes >=)", [](copt_lp & m, conv_handles & h) { m.set_constraint_upper_bound(h.r4, m.infinity()); }},
            {"R5 (BASIC) ub 100 -> inf (becomes free row)", [](copt_lp & m, conv_handles & h) { m.set_constraint_upper_bound(h.r5, m.infinity()); }},
        };
    for(auto & [name, edit] : edits) {
        try {
            copt_lp m;
            auto h = build_conv(m);
            nat n(m);
            m.solve();
            std::vector<int> c0, r0;
            n.get_basis(c0, r0);
            edit(m, h);
            std::vector<int> c1, r1;
            int ret = n.get_basis(c1, r1);
            std::printf("  [%s] GetBasis ret=%d %s\n      before: %s| %s\n      after:  %s| %s\n",
                        name, ret, n.state().c_str(), codes(c0).c_str(),
                        codes(r0).c_str(), codes(c1).c_str(), codes(r1).c_str());
            m.solve();
            std::vector<int> c2, r2;
            n.get_basis(c2, r2);
            std::printf("      re-solve: %s obj=%g\n      basis:  %s| %s\n",
                        n.state().c_str(), n.dattr("LpObjval"),
                        codes(c2).c_str(), codes(r2).c_str());
        } catch(const std::exception & e) {
            std::printf("  [%s] exception %s\n", name, e.what());
        }
    }

    // 2. compute_iis on an infeasible LP, Reset on an infeasible LP
    std::printf("\n=== 2. basis after compute_iis / Reset ===\n");
    for(int which : {0, 1, 2}) {
        try {
            using namespace operators;
            copt_lp m;
            m.set_time_limit(std::chrono::seconds(10));
            nat n(m);
            auto x = m.add_variable();
            auto y = m.add_variable();
            auto r1 = m.add_constraint(x + y >= 5);
            m.add_constraint(x + 2 * y <= 3);
            (void)r1;
            if(which == 0) {
                std::vector<int> c, r;
                int g0 = n.get_basis(c, r);
                std::printf("  infeasible, before anything: %s GetBasis ret=%d %s| %s\n",
                            n.state().c_str(), g0, codes(c).c_str(), codes(r).c_str());
                auto snap = m.compute_iis();
                (void)snap;
                int g = n.get_basis(c, r);
                std::printf("  infeasible, after compute_iis(): %s GetBasis ret=%d %s| %s\n",
                            n.state().c_str(), g, codes(c).c_str(), codes(r).c_str());
            } else if(which == 1) {
                m.solve();
                n.api.Reset(n.prob, 0);
                std::vector<int> c, r;
                int g = n.get_basis(c, r);
                std::printf("  infeasible, solve then Reset(0): %s GetBasis ret=%d %s| %s\n",
                            n.state().c_str(), g, codes(c).c_str(), codes(r).c_str());
            } else {
                // feasible after compute_iis on the infeasible one? relax row
                auto snap = m.compute_iis();
                (void)snap;
                m.set_constraint_upper_bound(constraint(1), 100);
                std::vector<int> c, r;
                int g = n.get_basis(c, r);
                std::printf("  infeasible, compute_iis then relax a row: %s GetBasis ret=%d %s| %s\n",
                            n.state().c_str(), g, codes(c).c_str(), codes(r).c_str());
                m.solve();
                g = n.get_basis(c, r);
                std::printf("     re-solve: %s GetBasis ret=%d %s| %s\n",
                            n.state().c_str(), g, codes(c).c_str(), codes(r).c_str());
            }
        } catch(const std::exception & e) {
            std::printf("  2.%d exception %s\n", which, e.what());
        }
    }
    // Reset on a feasible solved model, then SetBasis restores?
    try {
        copt_lp m;
        build_dense(m);
        nat n(m);
        m.solve();
        std::vector<int> c, r;
        n.get_basis(c, r);
        n.api.Reset(n.prob, 0);
        std::vector<int> c1, r1;
        n.get_basis(c1, r1);
        std::printf("  dense: after Reset(0) %s; #basic read=%d (rows %zu)\n",
                    n.state().c_str(), count_basic(c1) + count_basic(r1), r1.size());
        m.solve();
        std::printf("  dense: solve after Reset(0): %s\n", n.state().c_str());
    } catch(const std::exception & e) {
        std::printf("  2.dense exception %s\n", e.what());
    }

    // 3. SetBasis on a solved, unchanged model: is the solution dropped?
    std::printf("\n=== 3. SetBasis on a solved model ===\n");
    try {
        copt_lp m;
        build_dense(m);
        nat n(m);
        m.solve();
        std::vector<int> c, r;
        n.get_basis(c, r);
        std::printf("  solved: %s\n", n.state().c_str());
        int s = n.set_basis(c, r);
        std::printf("  SetBasis(own optimal basis) ret=%d: %s\n", s, n.state().c_str());
        m.solve();
        std::printf("  re-solve: %s obj=%.10g\n", n.state().c_str(), n.dattr("LpObjval"));
        // write the slack basis on the solved model, re-solve: cold?
        std::vector<int> cs(c.size(), LOWER), rs(r.size(), BASIC);
        s = n.set_basis(cs, rs);
        std::printf("  SetBasis(slack basis) ret=%d: %s\n", s, n.state().c_str());
        m.solve();
        std::printf("  re-solve: %s obj=%.10g\n", n.state().c_str(), n.dattr("LpObjval"));
    } catch(const std::exception & e) {
        std::printf("  3 exception %s\n", e.what());
    }

    // 4. warm optimal basis + time limit 0
    std::printf("\n=== 4. warm optimal basis under TimeLimit=0 ===\n");
    try {
        std::vector<int> c, r;
        {
            copt_lp a;
            build_dense(a);
            nat n(a);
            a.solve();
            n.get_basis(c, r);
        }
        for(int presolve : {-1, 0}) {
            copt_lp b;
            build_dense(b);
            nat n(b);
            n.setint("Presolve", presolve);
            n.set_basis(c, r);
            b.set_time_limit(std::chrono::duration<double>(0.0));
            b.solve();
            std::vector<int> c2, r2;
            int g = n.get_basis(c2, r2);
            std::printf("  Presolve=%d: %s obj=%.10g GetBasis ret=%d same-as-written=%d\n",
                        presolve, n.state().c_str(), n.dattr("LpObjval"), g,
                        c2 == c && r2 == r);
        }
    } catch(const std::exception & e) {
        std::printf("  4 exception %s\n", e.what());
    }

    // 5. BarIterLimit stop (no crossover / with crossover)
    std::printf("\n=== 5. BarIterLimit stop ===\n");
    for(int crossover : {0, 1}) {
        try {
            copt_lp b;
            build_dense(b);
            nat n(b);
            n.setint("LpMethod", 2);
            n.setint("BarIterLimit", 2);
            n.setint("Crossover", crossover);
            b.solve();
            std::vector<int> c2, r2;
            int g = n.get_basis(c2, r2);
            std::printf("  Crossover=%d: %s Status=%d GetBasis ret=%d #basic=%d wrapper status index=%zu\n",
                        crossover, n.state().c_str(), n.iattr("Status"), g,
                        count_basic(c2) + count_basic(r2), b.get_status().index());
        } catch(const std::exception & e) {
            std::printf("  5 exception %s\n", e.what());
        }
    }

    // 6. superbasic on a boxed column: does the solver ever report it?
    //    PDLP + crossover on the dense LP, report code histogram
    std::printf("\n=== 6. code histograms after crossover paths (dense) ===\n");
    for(int method : {2, 6}) {
        try {
            copt_lp b;
            build_dense(b);
            nat n(b);
            n.setint("LpMethod", method);
            b.solve();
            std::vector<int> c2, r2;
            n.get_basis(c2, r2);
            int hc[5] = {}, hr[5] = {};
            for(int x : c2) if(x >= 0 && x < 5) ++hc[x];
            for(int x : r2) if(x >= 0 && x < 5) ++hr[x];
            std::printf("  LpMethod=%d: %s cols L/B/U/S/F=%d/%d/%d/%d/%d rows=%d/%d/%d/%d/%d\n",
                        method, n.state().c_str(), hc[0], hc[1], hc[2], hc[3], hc[4],
                        hr[0], hr[1], hr[2], hr[3], hr[4]);
        } catch(const std::exception & e) {
            std::printf("  6 exception %s\n", e.what());
        }
    }

    // 7. free row and a nonbasic code on it
    std::printf("\n=== 7. free row ===\n");
    try {
        using namespace operators;
        copt_lp m;
        auto h = build_conv(m);
        nat n(m);
        auto fr = m.add_constraint(h.x1 + h.l <= 5);
        m.set_constraint_upper_bound(fr, m.infinity());
        m.solve();
        std::vector<int> c, r;
        n.get_basis(c, r);
        std::printf("  free row code: %s (%s)\n", code_name(r.back()), n.state().c_str());
    } catch(const std::exception & e) {
        std::printf("  7 exception %s\n", e.what());
    }

    // 8. the IIS-free path: after a wrapper solve on a model with zero rows
    std::printf("\n=== 8. columns only, no rows ===\n");
    try {
        copt_lp m;
        m.set_time_limit(std::chrono::seconds(10));
        nat n(m);
        auto x = m.add_variable(params_t{.obj_coef = 0, .lower_bound = 0, .upper_bound = 4});
        m.set_maximization();
        using namespace operators;
        m.set_objective(1.0 * x);
        std::vector<int> c, r;
        int g0 = n.get_basis(c, r);
        std::printf("  before solve: %s GetBasis ret=%d %s\n", n.state().c_str(), g0, codes(c).c_str());
        m.solve();
        int g = n.get_basis(c, r);
        std::printf("  after solve: %s GetBasis ret=%d %s\n", n.state().c_str(), g, codes(c).c_str());
    } catch(const std::exception & e) {
        std::printf("  8 exception %s\n", e.what());
    }
    std::printf("\nDONE\n");
}
