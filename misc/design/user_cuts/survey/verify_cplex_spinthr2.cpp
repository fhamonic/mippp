// Bisect the relative threshold; a fresh per-point tiny-violation cut at default settings; the proposed relative sanitize.
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
int main() {
    { M m; print_version(m); }
    const double limit = 3.0;
    // A: constant relaxed DFJ, frozen root, relative violation r
    for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{26, 5}, {31, 5}}) for(const double r : {1e-7, 2e-7, 3e-7, 5e-7, 7e-7, 9e-7, 1.1e-6, 1.5e-6, 2e-6, 5e-6}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, n, false, seed); all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
        m.set_time_limit(duration<double>(limit));
        auto cutp = std::make_shared<std::optional<raw_cut>>();
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            if(!*cutp) {
                auto cs = a.separate(x, 1);
                if(cs.empty()) return;
                raw_cut c = cs.front();
                c.rhs = c.activity(x) + r * std::sqrt(static_cast<double>(c.ind.size()));
                *cutp = c;
            }
            add(**cutp);
        });
        const double t = timed_solve(m);
        std::println(stderr, "A atsp{}s{} frozen, constant cut, v/|a|2={:.2e} nz={}: nodes={} added={} dropped={} {} {}", n, seed, r, *cutp ? (*cutp)->ind.size() : 0, m.get_node_count(), h.added.load(), h.dropped_noop.load(), status_name(m.get_status()), t > limit * 0.9 && m.get_node_count() == 0 ? "SPIN" : "");
    }
    // B: default settings, a separator whose cut at every point is DFJ relaxed down to a tiny violation (valid: weaker than DFJ while rhs <= 1)
    for(const double rel : {0.0, 1e-6, 2e-6}) for(const auto & [n, seed] : std::initializer_list<std::pair<int, unsigned>>{{26, 5}, {31, 5}, {22, 1}}) for(const double r : {1e-8, 1e-7, 3e-7, 1e-6, 1e-5, 1e-3}) {
        double base = NAN; { M m; m.set_thread_limit(1); build_atsp(m, n, false, seed); m.set_time_limit(duration<double>(20)); timed_solve(m); base = cf::value_of(m); }
        M m; m.set_thread_limit(1); auto a = build_atsp(m, n, false, seed);
        m.set_time_limit(duration<double>(limit));
        recipe R; R.rel_tol = rel;
        auto d0 = std::make_shared<std::atomic<long>>(0);
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            ++*d0;
            auto cs = a.separate(x, 1);
            if(cs.empty()) return;
            raw_cut c = cs.front();
            const double rhs = c.activity(x) + r * std::sqrt(static_cast<double>(c.ind.size()));
            if(rhs > 1) return;
            c.rhs = rhs;
            add(c);
        }, R);
        const double t = timed_solve(m);
        std::println(stderr, "B atsp{}s{} defaults, fresh tiny-violation cut per point, v/|a|2={:.2e} rel_tol={:g}: base {} | {} added={} dropped={} {}", n, seed, r, rel, base, summary(m, t), h.added.load(), h.dropped_noop.load(), t > limit * 0.9 && m.get_node_count() == 0 ? "SPIN" : "");
    }
    // C: the frozen spin with the relative sanitize
    for(const double rel : {1e-6, 2e-6}) for(const double r : {1e-7, 5e-7, 9e-7}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
        m.set_time_limit(duration<double>(limit));
        recipe R; R.rel_tol = rel;
        auto cutp = std::make_shared<std::optional<raw_cut>>();
        auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            if(!*cutp) { auto cs = a.separate(x, 1); if(cs.empty()) return; raw_cut c = cs.front(); c.rhs = c.activity(x) + r * std::sqrt(static_cast<double>(c.ind.size())); *cutp = c; }
            add(**cutp);
        }, R);
        const double t = timed_solve(m);
        std::println(stderr, "C atsp26s5 frozen, constant cut v/|a|2={:.2e} with rel_tol={:g}: {} added={} dropped={} {}", r, rel, summary(m, t), h.added.load(), h.dropped_noop.load(), t > limit * 0.9 && m.get_node_count() == 0 ? "SPIN" : "");
    }
}
