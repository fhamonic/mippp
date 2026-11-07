// Where does a valid, re-offered, slightly violated cut stop making the root loop spin?
// The root LP is frozen (one family on, EACHCUTLIM=0) or left at defaults. Report for each violation v and scale S.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static std::vector<double> objective_of(M & m) {
    auto [env, lp] = m.native_model();
    const int n = static_cast<int>(m.num_variables());
    std::vector<double> c(static_cast<std::size_t>(n));
    m.native_api()._check(env, m.native_api().getobj(env, lp, c.data(), 0, n - 1));
    return c;
}
struct out { bool spin; long nodes; long calls; std::string st; double viol_seen; };
int main() {
    { M m; print_version(m); }
    const std::vector<double> vs = {1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1};
    const double limit = 2.0;
    // 1) objective floor: norm of c is large, the cut is parallel to the objective
    for(const bool frozen : {true, false}) for(const double S : {1e-3, 1.0, 1e3}) for(const double v : vs) {
        M m; m.set_thread_limit(1); build_mkp(m, 5, 60, 3); if(frozen) { all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0); }
        m.set_time_limit(duration<double>(limit));
        const auto c = objective_of(m);
        double ninf = 0, n2 = 0; for(double w : c) { ninf = std::max(ninf, std::fabs(w)); n2 += w * w; } n2 = std::sqrt(n2);
        auto init = std::make_shared<std::optional<double>>();
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            raw_cut cut{{}, {}, 'L', 0}; double act = 0;
            for(std::size_t j = 0; j < c.size(); ++j) if(c[j] != 0) { cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(S * c[j]); act += S * c[j] * x[j]; }
            if(!*init) *init = act - v * S;
            cut.rhs = **init;
            add(cut);
        });
        const double t = timed_solve(m);
        std::println(stderr, "THR objective-floor frozen={} S={:g} v(unscaled)={:g} v/|c|inf={:.2e} v/|c|2={:.2e}: nodes={} added={} dropped={} {} {}", frozen, S, v, v / ninf, v / n2, m.get_node_count(), h.added.load(), h.dropped_noop.load(), status_name(m.get_status()), t > limit * 0.9 && m.get_node_count() == 0 ? "SPIN" : "");
    }
    // 2) a DFJ cut relaxed until its violation at the first point is exactly v: valid because weaker
    for(const bool frozen : {true, false}) for(const double S : {1e-3, 1.0, 1e3}) for(const double v : vs) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); if(frozen) { all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0); }
        m.set_time_limit(duration<double>(limit));
        auto cutp = std::make_shared<std::optional<raw_cut>>();
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            if(!*cutp) {
                auto cs = a.separate(x, 1);
                if(cs.empty()) return;
                raw_cut c = cs.front();
                c.rhs = c.activity(x) + v;
                for(auto & w : c.val) w *= S;
                c.rhs *= S;
                *cutp = c;
            }
            add(**cutp);
        });
        const double t = timed_solve(m);
        std::println(stderr, "THR relaxed-DFJ frozen={} S={:g} v(unscaled)={:g} nz={} v/|a|2={:.2e}: nodes={} added={} dropped={} {} {}", frozen, S, v, *cutp ? (*cutp)->ind.size() : 0, *cutp ? v / std::sqrt(static_cast<double>((*cutp)->ind.size())) : 0.0, m.get_node_count(), h.added.load(), h.dropped_noop.load(), status_name(m.get_status()), t > limit * 0.9 && m.get_node_count() == 0 ? "SPIN" : "");
    }
}
