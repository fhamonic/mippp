// The proposed sanitize: drop when violation <= 1.1e-6 max|a|. Does it remove every sub-threshold spin? Does the threshold follow EpRHS?
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static bool looks_spin(M & m, double t, double limit) { return t > limit * 0.9 && m.get_node_count() == 0; }
int main() {
    { M m; print_version(m); }
    const double limit = 2.0;
    // 1) does EpRHS move the threshold?
    for(const double ep : {1e-5, 1e-6, 1e-7, 1e-9}) {
        std::string line = std::format("EPRHS={:g}:", ep);
        for(const double r : {1e-9, 1e-8, 1e-7, 3e-7, 9.5e-7, 1.05e-6, 3e-6, 9.5e-6, 1.05e-5, 3e-5}) {
            M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0); vparamd(m, CPX_PARAM_EPRHS, ep);
            m.set_time_limit(duration<double>(limit));
            auto cutp = std::make_shared<std::optional<raw_cut>>();
            recipe R; R.satisfied_tol = 1e-12;
            vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
                if(!*cutp) { auto cs = a.separate(x, 1); if(cs.empty()) return; raw_cut c = cs.front(); c.rhs = c.activity(x) + r; *cutp = c; }
                add(**cutp);
            }, R);
            const double t = timed_solve(m);
            line += std::format(" {:g}{}", r, looks_spin(m, t, limit) ? "=SPIN" : "=ok");
        }
        std::println(stderr, "{}", line);
    }
    // 2) the proposed drop on the frozen constant cuts and on fresh tiny cuts at defaults
    for(const double inf : {0.0, 1.1e-6}) {
        for(const double r : {3e-9, 1e-8, 1e-7, 5e-7, 9e-7}) {
            M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
            m.set_time_limit(duration<double>(limit));
            auto cutp = std::make_shared<std::optional<raw_cut>>();
            recipe R; R.inf_tol = inf;
            auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
                if(!*cutp) { auto cs = a.separate(x, 1); if(cs.empty()) return; raw_cut c = cs.front(); c.rhs = c.activity(x) + r; *cutp = c; }
                add(**cutp);
            }, R);
            const double t = timed_solve(m);
            std::println(stderr, "FROZEN constant v/a_inf={:g} inf_tol={:g}: nodes={} added={} dropped={} {}", r, inf, m.get_node_count(), h.added.load(), h.dropped_noop.load(), looks_spin(m, t, limit) ? "SPIN" : "ok");
        }
        for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{26, 5}, {31, 5}, {22, 1}}) for(const double r : {1e-8, 1e-7, 5e-7, 9e-7, 1.2e-6, 1e-5, 1e-4}) {
            M m; m.set_thread_limit(1); auto a = build_atsp(m, n, false, seed);
            m.set_time_limit(duration<double>(limit));
            recipe R; R.inf_tol = inf;
            auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
                auto cs = a.separate(x, 1); if(cs.empty()) return;
                raw_cut c = cs.front();
                const double rhs = c.activity(x) + r;
                if(rhs > 1) return;
                c.rhs = rhs; add(c);
            }, R);
            const double t = timed_solve(m);
            std::println(stderr, "DEFAULTS fresh cut per point atsp{}s{} v/a_inf={:g} inf_tol={:g}: {} added={} dropped={} {}", n, seed, r, inf, summary(m, t), h.added.load(), h.dropped_noop.load(), looks_spin(m, t, limit) ? "SPIN" : "ok");
        }
    }
}
