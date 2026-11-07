// Does a user-cut separator change the deletion filter's answer? The trials go
// through solve(); MIP++ attaches no callback of its own here, so a native
// MIPNODE callback registered on native_model() stays in place in every trial
// and stands for the proposed relaxation callback.
//
// Model: x in {0,1}^6, y in [0.5, 2] continuous,
//   M: 2 sum x + y == 6.5   (integer points: sum x = 3, y = 0.5)
//   R: 2 sum x <= 5         (integer points: sum x <= 2)
// M and R conflict at integer points only; the LP of M and R is feasible.
// The separator is R's cover cut, sum x <= 2, offered when violated: valid at
// every integer point of R, hence of the model. It is what a cover separator
// derives from R's data, and it cannot know that a trial relaxed R.
#include <cstdio>
#include <print>
#include <string>
#include <vector>

#include "mippp/solvers/gurobi/all.hpp"
#include "mippp/utility/iis_by_deletion.hpp"

using namespace mippp;
using namespace mippp::operators;
using namespace mippp::gurobi::impl::v1;
// not declared by the wrapper; values from gurobi_c.h 12.0.1:647-648
constexpr int MIPNODE_STATUS = 5001, MIPNODE_REL = 5002;

struct sep_state {
    const gurobi_api * GRB = nullptr;
    std::vector<int> cols;
    int num_cols = 0;
    double k = 2;
    long calls = 0, cuts = 0;
};

static int separator(GRBmodel *, void * cbdata, int where, void * usr) {
    if(where != GRB_CB_MIPNODE) return 0;
    auto & s = *static_cast<sep_state *>(usr);
    int st = 0;
    if(s.GRB->cbget(cbdata, GRB_CB_MIPNODE, MIPNODE_STATUS, &st)) return 0;
    if(st != GRB_OPTIMAL) return 0;
    ++s.calls;
    std::vector<double> x(static_cast<std::size_t>(s.num_cols));
    if(s.GRB->cbget(cbdata, GRB_CB_MIPNODE, MIPNODE_REL, x.data())) return 0;
    double sum = 0;
    for(int j : s.cols) sum += x[static_cast<std::size_t>(j)];
    if(sum > s.k + 1e-6) {
        std::vector<double> ones(s.cols.size(), 1.0);
        if(s.GRB->cbcut(cbdata, static_cast<int>(s.cols.size()), s.cols.data(),
                        ones.data(), GRB_LESS_EQUAL, s.k) == 0)
            ++s.cuts;
    }
    return 0;
}

struct probe : gurobi_milp {
    using gurobi_milp::gurobi_milp;
    sep_state * s = nullptr;
    std::string log;
    void solve() {
        const long cuts = s ? s->cuts : 0;
        gurobi_milp::solve();
        const auto & st = get_status();
        log += is<status::optimal>(st)           ? "F"
               : is_a<status::infeasible>(st) ? "I"
                                              : "?";
        if(s && s->cuts > cuts) log += "*";
    }
};

struct answer {
    bool irreducible, M, R, y_up, y_low;
    std::size_t x_bounds;
    long calls, cuts;
    std::string trials;
};

static answer run(const bool with_separator, const int seed) {
    probe m;
    const auto [env, grb_model] = m.native_model();
    const gurobi_api & GRB = m.native_api();
    // keep the node LPs fractional so that the node callback sees them
    GRB.setintparam(env, "Presolve", 0);
    GRB.setintparam(env, "Cuts", 0);
    GRB.setdblparam(env, "Heuristics", 0.0);
    GRB.setintparam(env, "PreCrush", 1);
    GRB.setintparam(env, "Threads", 1);
    GRB.setintparam(env, "Seed", seed);
    m.set_time_limit(std::chrono::seconds(10));
    auto x = m.add_binary_variables(6);
    auto y = m.add_variable({.lower_bound = 0.5, .upper_bound = 2.0});
    const auto ids = std::views::iota(std::size_t{0}, std::size_t{6});
    auto M = m.add_constraint(
        xsum(ids, [&](auto i) { return 2.0 * x(i); }) + y == 6.5);
    auto R = m.add_constraint(xsum(ids, [&](auto i) { return 2.0 * x(i); }) <=
                              5);
    sep_state s;
    s.GRB = &GRB;
    s.num_cols = 7;
    for(auto i : ids) s.cols.push_back(m.native_id(x(i)));
    if(with_separator) {
        m.s = &s;
        GRB.setcallbackfunc(grb_model, separator, &s);
    }
    const auto iis = compute_iis_by_deletion(m);
    answer a{};
    a.irreducible = is<iis_outcome::irreducible>(iis.get_outcome());
    a.M = is_a<iis_status::member>(iis.get_status(M));
    a.R = is_a<iis_status::member>(iis.get_status(R));
    const auto ys = iis_status::sides_of(iis.get_status(y));
    a.y_up = ys.upper;
    a.y_low = ys.lower;
    for(auto i : ids)
        a.x_bounds += is_a<iis_status::member>(iis.get_status(x(i)));
    a.calls = s.calls;
    a.cuts = s.cuts;
    a.trials = m.log;
    return a;
}

int main() {
    int wrong = 0, runs = 0;
    for(int seed = 0; seed < 20; ++seed) {
        for(const bool sep : {false, true}) {
            const answer a = run(sep, seed);
            std::println(
                "seed {:2} {}: {} M={} R={} y<=2:{} y>=0.5:{} x-bounds={} | "
                "separator calls={} cuts={} | trials {}",
                seed, sep ? "separator" : "control  ",
                a.irreducible ? "irreducible" : "OTHER", a.M, a.R, a.y_up,
                a.y_low, a.x_bounds, a.calls, a.cuts, a.trials);
            if(sep) {
                ++runs;
                if(!a.R) ++wrong;
            }
            std::fflush(stdout);
        }
    }
    std::println("with the separator: {} of {} answers leave R out", wrong,
                 runs);
}
