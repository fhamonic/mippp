// A tolerance-free guard against every same-LP spin, and whether the max|a| tolerance survives aggregation.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static std::string spin(M & m, double t, double limit) { return t > limit * 0.9 && m.get_node_count() == 0 ? "SPIN" : "ok"; }
// x_j = 1000 y_j: presolve aggregates one of the two; the cut is written on x, coefficient c_j / 1000, so max|a| of the cut as written is ~1.
static void aggregated(const double limit) {
    for(const bool guard : {false, true}) for(const double inf : {0.0, 1.1e-6}) for(const double v : {1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1}) {
        M m; m.set_thread_limit(1);
        const std::size_t C = 60, R = 5;
        std::mt19937 rng(3); std::uniform_int_distribution<int> A(1, 1000), Q(0, 500);
        std::vector<std::vector<double>> a(R, std::vector<double>(C)); for(auto & r : a) for(auto & w : r) w = A(rng);
        std::vector<double> c(C); for(std::size_t j = 0; j < C; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / static_cast<double>(R) + Q(rng)); }
        auto y = m.add_binary_variables(C);
        auto x = m.add_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1000});
        const auto ids = std::views::iota(std::size_t{0}, C);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * y(j); }));
        for(std::size_t i = 0; i < R; ++i) { double s = 0; for(double w : a[i]) s += w; m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * y(j); }) <= std::floor(s / 2)); }
        for(std::size_t j : ids) m.add_constraint(x(j) - 1000.0 * y(j) == 0);
        std::vector<int> cx; for(std::size_t j : ids) cx.push_back(m.native_id(x(j)));
        all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
        m.set_time_limit(duration<double>(limit));
        auto init = std::make_shared<std::optional<double>>();
        recipe Rr; Rr.inf_tol = inf; Rr.same_point_guard = guard;
        auto & h = vattach(m, [=](const std::vector<double> & p, const add_fn & add) {
            raw_cut cut{{}, {}, 'L', 0}; double act = 0;
            for(std::size_t j = 0; j < C; ++j) { const double w = c[j] / 1000.0; cut.ind.push_back(cx[j]); cut.val.push_back(w); act += w * p[static_cast<std::size_t>(cx[j])]; }
            if(!*init) *init = act - v;
            cut.rhs = **init;
            add(cut);
        }, Rr);
        const double t = timed_solve(m);
        std::println(stderr, "AGG guard={} inf_tol={:g} v={:g} (written max|a|~1.1, crushed ~1100): nodes={} | {} {}", guard, inf, v, m.get_node_count(), h.line(), spin(m, t, limit));
    }
}
int main() {
    { M m; print_version(m); }
    const double limit = 2.0;
    aggregated(limit);
    // the guard alone: no drop tolerance beyond what the recipe has
    for(const auto & [name, kind] : std::initializer_list<std::pair<const char *, int>>{{"constant satisfied cut, no drop", 0}, {"frozen, constant cut violated by 5e-7", 1}, {"defaults, fresh cut violated by 5e-7", 2}, {"defaults, fresh cut violated by 1e-5", 3}}) for(const bool guard : {false, true}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); if(kind == 1) { all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0); }
        m.set_time_limit(duration<double>(limit));
        recipe R; R.same_point_guard = guard; if(kind == 0) R.drop_satisfied = false;
        auto cutp = std::make_shared<std::optional<raw_cut>>();
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            if(kind == 0) { add(raw_cut{{a.col[0][1]}, {1.0}, 'L', 1.0}); return; }
            if(kind == 1) { if(!*cutp) { auto cs = a.separate(x, 1); if(cs.empty()) return; raw_cut c = cs.front(); c.rhs = c.activity(x) + 5e-7; *cutp = c; } add(**cutp); return; }
            auto cs = a.separate(x, 1); if(cs.empty()) return; raw_cut c = cs.front(); const double rhs = c.activity(x) + (kind == 2 ? 5e-7 : 1e-5); if(rhs > 1) return; c.rhs = rhs; add(c);
        }, R);
        const double t = timed_solve(m);
        std::println(stderr, "GUARD {} guard={}: {} | {} {}", name, guard, summary(m, t), h.line(), spin(m, t, limit));
    }
    // does the guard change anything for a normal separator?
    for(const bool guard : {false, true}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 31, false, 11); m.set_time_limit(duration<double>(20));
        recipe R; R.same_point_guard = guard;
        auto & h = vattach(m, cf::dfj<M>(a), R);
        const double t = timed_solve(m);
        std::println(stderr, "GUARD normal DFJ atsp31s11 guard={}: {} | {}", guard, summary(m, t), h.line());
    }
}
