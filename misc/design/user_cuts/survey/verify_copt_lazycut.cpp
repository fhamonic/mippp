// Interplay in the one slot: link rows hidden, added by the shipped add_lazy_constraint in MIPSOL, and user cuts from
// MIPRELAX through the recipe. The cut x^k_e <= M y_e is valid only given the hidden rows (M >= d_k). M sweeps from d_k
// to very large: big-M coefficients in a valid cut. usage: verify_copt_lazycut <M: 0 for d_k, or a value> seed
// Also threads: verify_copt_lazycut threads <nd|atsp> <reps>
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    if(std::string(argv[1]) == "threads") {
        const std::string inst = argv[2];
        const int reps = std::atoi(argv[3]);
        setenv("CAPF", "0.2", 1);
        for(int rep = 0; rep < reps; ++rep) for(const unsigned seed : {1u, 2u, 3u}) {
            double base; std::string bs;
            {
                V b; b.set_time_limit(duration<double>(60.0));
                if(inst == "nd") build_mcnd(b, 25, 60, 30, seed); else cp::build_atsp(b, 26, false, seed == 1 ? 5 : seed + 5);
                const double t = cp::timed_solve(b); base = b.get_solution_value(); bs = cp::summary(b, t);
            }
            V m; m.set_time_limit(duration<double>(60.0));
            std::atomic<long> calls{0}, cuts{0};
            std::mutex mu; std::set<std::thread::id> tids;
            cp::separator sep;
            std::optional<mcnd> g; std::optional<cp::atsp> a;
            if(inst == "nd") { auto v = build_mcnd(m, 25, 60, 30, seed); g = v.g; sep = [&](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : g->links(x, 10)) add(c); }; }
            else { a = cp::build_atsp(m, 26, false, seed == 1 ? 5 : seed + 5); sep = [&](const std::vector<double> & x, const cp::add_fn & add) { if(calls.load() > 40) for(const auto & c : a->separate(x)) add(c); }; }
            recipe_opts o; o.fold = o.strip_zero = o.skip_empty = o.drop_satisfied = false;
            m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
                ++calls; { std::lock_guard l(mu); tids.insert(std::this_thread::get_id()); }
                sep(x, [&](const cp::raw_cut & raw) -> int { const auto r = cf::sanitize(raw, x); if(!r) return 0; ++cuts; return add(*r); });
            }, o);
            const double t = cp::timed_solve(m);
            const double v = cf::value_of(m);
            std::println(stderr, "threads {} rep={} seed={} {} {} | base {} | calls={} cuts={} os_threads={} err={}", inst, rep, seed, is_a<status::optimal>(m.get_status()) && cf::same(v, base) ? "OK   " : "WRONG", cp::summary(m, t), bs, calls.load(), cuts.load(), tids.size(), m.last->native_err.load());
        }
        return 0;
    }
    const double M0 = std::atof(argv[1]);
    const unsigned seed = std::atoi(argv[2]);
    setenv("CAPF", "0.2", 1);
    double base; std::string bs;
    { V b; b.iparam("Threads", 1); build_mcnd(b, 14, 30, 16, seed); b.set_time_limit(duration<double>(30.0)); const double t = cp::timed_solve(b); base = b.get_solution_value(); bs = cp::summary(b, t); }
    V m; m.iparam("Threads", 1); m.set_time_limit(duration<double>(30.0));
    auto v = build_mcnd(m, 14, 30, 16, seed, true);
    auto & g = v.g;
    const auto Ks = std::views::iota(std::size_t{0}, g.K);
    long lazy_calls = 0, lazies = 0;
    m.set_candidate_solution_callback([&](auto & h) {
        ++lazy_calls;
        auto sol = h.get_solution();
        std::vector<double> p(m.num_variables());
        for(std::size_t e = 0; e < g.A; ++e) { p[(std::size_t)g.y[e]] = sol[v.yv(e)]; for(std::size_t k : Ks) p[(std::size_t)g.x[k][e]] = sol[v.xv(v.idx(k, e))]; }
        for(std::size_t e : g.violated_caps(p)) { ++lazies; h.add_lazy_constraint(xsum(Ks, [&, e](std::size_t k) { return 1.0 * v.xv(v.idx(k, e)); }) - g.cap[e] * v.yv(e) <= 0.0); }
    });
    recipe_opts o; o.fold = o.strip_zero = o.skip_empty = o.drop_satisfied = false;
    std::atomic<long> calls{0}, cuts{0};
    m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
        ++calls;
        int given = 0;
        for(std::size_t k = 0; k < g.K; ++k) for(std::size_t e = 0; e < g.A && given < 10; ++e) {
            const double M = M0 > 0 ? M0 : g.d[k];
            cp::raw_cut c{{g.x[k][e], g.y[e]}, {1.0, -M}, 'L', 0.0};
            const auto r = cf::sanitize(c, x, true, 1e-6);
            if(!r) continue;
            add(*r); ++cuts; ++given;
        }
    }, o);
    const double t = cp::timed_solve(m);
    const double val = cf::value_of(m);
    std::println(stderr, "lazy+cut M={} seed={} {} {} | base {} | relax={} cuts={} lazy_calls={} lazies={} native_err={}", M0, seed, is_a<status::optimal>(m.get_status()) && cf::same(val, base) ? "OK   " : "WRONG", cp::summary(m, t), bs, calls.load(), cuts.load(), lazy_calls, lazies, m.last->native_err.load());
}
