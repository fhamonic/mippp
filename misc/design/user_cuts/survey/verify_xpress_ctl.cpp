// C: targeted negative controls, for traps the shared conformance set has no power on. One process per case.
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
using cut = cp::raw_cut;
template <class F>
static void isolated(const std::string & name, F f, unsigned seconds = 80) {
    std::fflush(stderr);
    const pid_t p = fork();
    if(p == 0) { alarm(seconds); vx::quiet_flush = true; f(); std::fflush(stderr); _exit(0); }
    int st = 0; waitpid(p, &st, 0);
    if(WIFSIGNALED(st)) std::println(stderr, "CRASH {}: signal {}", name, WTERMSIG(st));
    else if(WEXITSTATUS(st)) std::println(stderr, "CHILD {}: exit {}", name, WEXITSTATUS(st));
}
static std::string line(vx::ctx & c, cp::stats & s) {
    std::string ps; for(auto [k, n] : c.presolve_status) ps += std::format("{}:{} ", k, n);
    std::string er; for(auto [k, n] : c.errors) er += std::format("{}:{} ", k, n);
    return std::format("calls={} native={} dropped={} add_errors={} bad_points={} nodes_seen={} depth={} rows {}->{} CUTS_max={} main/worker={}/{} threads={} presolve{{{}}} errors{{{}}}{}", s.calls.load(), c.added_native.load(), c.dropped.load(), s.add_errors.load(), s.bad_points.load(), c.nodes.size(), c.max_depth, c.rows_first, c.rows_max, c.cuts_max, c.cb_is_main.load(), c.cb_differs.load(), s.threads.size(), ps, er, c.first_error.empty() ? "" : " first=[" + c.first_error + "]");
}
static void reoffer() {
    for(const char * spec : {"R1", "R1,keep_satisfied", "R1,add2", "R1,add2,keep_satisfied", "R2", "R2,keep_satisfied", "R2cr", "R2cr,keep_satisfied"}) for(const unsigned seed : {5u, 1u}) {
        isolated(std::format("reoffer [{}]", spec), [&] {
            M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(10));
            auto a = cp::build_atsp(m, 26, false, seed);
            auto all = std::make_shared<std::vector<cut>>();
            auto s = std::make_shared<cp::stats>();
            auto & c = vx::attach(m, cp::watched([a, all](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) all->push_back(k); for(const auto & k : *all) add(k); }, s), vx::parse(spec));
            const double t = cp::timed_solve(m);
            std::println(stderr, "T1 re-offers all, DFJ n=26 seed={} [{}]: {} | {}", seed, spec, cp::summary(m, t), line(c, *s));
        }, 30);
    }
}
static void points() {
    for(const char * spec : {"R1", "R1,point_lpsol", "R1,point_cols", "R1,point_presolved", "R2", "R2,point_cols", "R2,point_presolved"}) {
        isolated(std::format("points [{}]", spec), [&] {
            M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
            auto f = m.add_binary_variables(40);
            for(std::size_t i = 0; i < 40; ++i) m.add_constraint(1.0 * f(i) == 1.0);
            auto a = cp::build_atsp(m, 26, true, 5);
            auto s = std::make_shared<cp::stats>();
            auto & c = vx::attach(m, cp::watched([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) add(k); }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }), vx::parse(spec));
            const double t = cp::timed_solve(m);
            std::println(stderr, "T2 point, shifted + complement ATSP n=26 truth 549 [{}]: {} | {}", spec, cp::summary(m, t), line(c, *s));
        }, 40);
    }
}
template <class Mo> static auto knap(Mo & m, unsigned seed, std::size_t cols, std::size_t rows) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), R(0, 50);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(cols), cap(rows);
    for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / static_cast<double>(rows) + R(rng)); }
    for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
    std::vector<int> col; for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    return std::tuple{col, a, cap};
}
static cp::separator covers(std::vector<std::vector<double>> a, std::vector<double> cap, std::vector<int> col) {
    return [=](const std::vector<double> & p, const cp::add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; cut k{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; k.ind.push_back(col[j]); k.val.push_back(1);
                if(w > cap[i]) { k.rhs = static_cast<double>(k.ind.size()) - 1; if(k.violation(p) > 1e-4) add(k); break; }
            }
        }
    };
}
static void threads_case() {
    double truth = NAN;
    { M m; m.set_time_limit(duration<double>(120)); auto [col, a, cap] = knap(m, 1, 80, 10); const double t = cp::timed_solve(m); truth = cf::value_of(m); std::println(stderr, "T3 baseline covers 80x10 default threads: {}", cp::summary(m, t)); }
    for(const char * spec : {"R1", "R1,main_prob", "R2", "R2,main_prob", "R1,no_mutex", "R2,no_mutex"}) for(int rep = 0; rep < 2; ++rep) {
        isolated(std::format("threads [{}]", spec), [&] {
            M m; m.set_time_limit(duration<double>(30));
            auto [col, a, cap] = knap(m, 1, 80, 10);
            auto s = std::make_shared<cp::stats>();
            auto & c = vx::attach(m, cp::watched(covers(a, cap, col), s), vx::parse(spec));
            const double t = cp::timed_solve(m);
            std::println(stderr, "T3 covers 80x10 default threads truth {} [{}] rep={}: {} | {} overlaps={} peak={}", truth, spec, rep, cp::summary(m, t), line(c, *s), s->overlaps.load(), s->max_inflight.load());
        }, 60);
    }
}
static void managed_optnode() {
    isolated("managed optnode", [&] {
        M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
        auto a = cp::build_atsp(m, 26, false, 5);
        auto s = std::make_shared<cp::stats>();
        auto & c = vx::attach(m, cp::watched([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) add(k); }, s), vx::parse("managed_optnode"));
        const double t = cp::timed_solve(m);
        std::println(stderr, "T4 XPRSaddmanagedcuts in optnode, ATSP n=26 truth 549: {} | {}", cp::summary(m, t), line(c, *s));
    }, 30);
}
static void folding() {
    struct K { std::string name; std::function<void(cut &)> mess; };
    const std::vector<K> ks = {
        {"column doubled, 0.5 + 0.5 on the first term", [](cut & c) { c.val[0] = 0.5; c.ind.push_back(c.ind[0]); c.val.push_back(0.5); }},
        {"zero coefficient appended", [](cut & c) { c.ind.push_back(c.ind[0]); c.val.push_back(0.0); }},
        {"cancelling pair on a column outside the cut", [](cut & c) { c.ind.push_back(0); c.val.push_back(1); c.ind.push_back(0); c.val.push_back(-1); }},
    };
    for(const char * spec : {"R1", "R1,no_folding", "R2", "R2,no_folding", "R2cr,no_folding"}) for(const auto & k : ks) {
        isolated(std::format("folding {} [{}]", k.name, spec), [&] {
            M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
            auto a = cp::build_atsp(m, 26, false, 5);
            auto s = std::make_shared<cp::stats>();
            auto & c = vx::attach(m, cp::watched([a, k](const std::vector<double> & x, const cp::add_fn & add) { for(cut q : a.separate(x)) { k.mess(q); add(q); } }, s), vx::parse(spec));
            const double t = cp::timed_solve(m);
            std::println(stderr, "T6 {} truth 549 [{}]: {} | {}", k.name, spec, cp::summary(m, t), line(c, *s));
        }, 30);
    }
    // an empty row and an empty row with a false rhs, unsanitized
    for(const char * spec : {"R1", "R1,no_folding", "R2", "R2,no_folding"}) for(double rhs : {1.0, -1.0}) {
        isolated(std::format("empty rhs {} [{}]", rhs, spec), [&] {
            M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
            auto a = cp::build_atsp(m, 26, false, 5);
            auto s = std::make_shared<cp::stats>();
            auto done = std::make_shared<bool>(false);
            auto & c = vx::attach(m, cp::watched([a, done, rhs](const std::vector<double> & x, const cp::add_fn & add) { if(!*done) { *done = true; add(cut{{}, {}, 'L', rhs}); } for(const auto & q : a.separate(x)) add(q); }, s), vx::parse(spec));
            const double t = cp::timed_solve(m);
            std::println(stderr, "T7 empty row 0 <= {} (rhs<0 is not valid unless infeasible) truth 549 [{}]: {} | {}", rhs, spec, cp::summary(m, t), line(c, *s));
        }, 30);
    }
}
static void parallel_power() {
    for(const char * spec : {"R1,first2", "R1"}) for(const int mode : {0, 1, 2}) for(unsigned seed : {5u, 1u}) {
        // mode 0 cuts on a + b; 1 a + 2 b (weaker, valid); 2 a only + b only on half: a + b on some arcs, 2 a + b on others
        isolated(std::format("parallel mode {} [{}]", mode, spec), [&] {
            double base = NAN;
            for(int phase : {0, 1}) {
                M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(phase == 0 ? 60 : 20));
                const int n = 24; const auto N = static_cast<std::size_t>(n);
                std::mt19937 rng(seed);
                std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
                std::vector<std::pair<double, double>> pts(N); for(auto & p : pts) p = {P(rng), P(rng)};
                std::vector<std::vector<double>> d(N, std::vector<double>(N, 0));
                for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
                auto a = m.add_binary_variables(N * N); auto b = m.add_binary_variables(N * N);
                auto u = m.add_variables(N, {.obj_coef = 0, .lower_bound = 0, .upper_bound = static_cast<double>(n - 1)});
                const auto nodes = std::views::iota(std::size_t{0}, N);
                const auto id = [N](std::size_t i, std::size_t j) { return i * N + j; };
                m.set_minimization();
                m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes, [&, i](std::size_t j) { return d[i][j] * (a(id(i, j)) + b(id(i, j))); }); }));
                for(std::size_t i : nodes) {
                    m.set_variable_upper_bound(a(id(i, i)), 0); m.set_variable_upper_bound(b(id(i, i)), 0);
                    m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return a(id(i, j)) + b(id(i, j)); }) == 1);
                    m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return a(id(j, i)) + b(id(j, i)); }) == 1);
                }
                for(std::size_t i = 1; i < N; ++i) for(std::size_t j = 1; j < N; ++j) if(i != j)
                    m.add_constraint(u(i) - u(j) + static_cast<double>(n - 1) * (a(id(i, j)) + b(id(i, j))) <= static_cast<double>(n - 2));
                std::vector<std::vector<int>> ca(N, std::vector<int>(N)), cb(N, std::vector<int>(N));
                for(std::size_t i : nodes) for(std::size_t j : nodes) { ca[i][j] = m.native_id(a(id(i, j))); cb[i][j] = m.native_id(b(id(i, j))); }
                if(phase == 0) { const double t = cp::timed_solve(m); base = cf::value_of(m); std::println(stderr, "T8 baseline Euclid parallel n=24 seed={}: {}", seed, cp::summary(m, t)); if(!is_a<status::optimal>(m.get_status())) return; continue; }
                auto s = std::make_shared<cp::stats>();
                auto sep = [n, N, ca, cb, mode](const std::vector<double> & x, const cp::add_fn & add) {
                    cp::atsp sc; sc.n = n; sc.col.assign(N, std::vector<int>(N, -1));
                    std::vector<double> sum; int next = 0;
                    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) { sc.col[i][j] = next++; sum.push_back(x[static_cast<std::size_t>(ca[i][j])] + x[static_cast<std::size_t>(cb[i][j])]); }
                    for(const auto & c : sc.separate(sum)) {
                        cut out; out.sense = c.sense; out.rhs = c.rhs;
                        std::set<int> used(c.ind.begin(), c.ind.end());
                        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && used.contains(sc.col[i][j])) {
                            const double ka = 1, kb = mode == 0 ? 1 : mode == 1 ? 2 : ((i + j) % 2 ? 2 : 1);
                            out.ind.push_back(ca[i][j]); out.val.push_back(ka); out.ind.push_back(cb[i][j]); out.val.push_back(kb);
                        }
                        add(out);
                    }
                };
                auto & c = vx::attach(m, cp::watched(sep, s), vx::parse(spec));
                const double t = cp::timed_solve(m);
                const double v = cf::value_of(m);
                std::println(stderr, "T8 Euclid parallel n=24 seed={} mode={} {} [{}] base {}: {} | {}", seed, mode, (is_a<status::optimal>(m.get_status()) && cf::same(v, base)) ? "OK" : "BAD", spec, base, cp::summary(m, t), line(c, *s));
            }
        }, 100);
    }
}
static void stop_power() {
    for(const char * spec : {"R2"}) for(const double first : {1.0, 1.5, 2.0, 2.5, 3.0}) for(unsigned seed : {9u, 5u}) {
        isolated(std::format("stop [{}]", spec), [&] {
            const int n = 40;
            double want = NAN;
            { M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(60)); auto a = cp::build_atsp(m, n, false, seed); auto s = std::make_shared<cp::stats>(); vx::attach(m, cp::watched([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) add(k); }, s), vx::parse("R2")); cp::timed_solve(m); want = cf::value_of(m); }
            M m; m.set_thread_limit(1);
            auto a = cp::build_atsp(m, n, false, seed);
            auto s = std::make_shared<cp::stats>();
            const auto sep = [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & k : a.separate(x)) add(k); };
            auto & c1 = vx::attach(m, cp::watched(sep, s), vx::parse(spec));
            m.set_time_limit(duration<double>(first));
            const double t1 = cp::timed_solve(m);
            const std::string f = cp::summary(m, t1);
            const long calls1 = s->calls.load(), native1 = c1.added_native.load();
            auto & c2 = vx::attach(m, cp::watched(sep, s), vx::parse(spec));
            m.set_time_limit(duration<double>(60));
            const double t2 = cp::timed_solve(m);
            const double v = cf::value_of(m);
            std::println(stderr, "T9 stop at {} n=40 seed={} [{}]: first {} calls={} native={} | then {} calls={} native={} | want {} => {}", first, seed, spec, f, calls1, native1, cp::summary(m, t2), s->calls.load() - calls1, c2.added_native.load(), want, cf::same(v, want) && is_a<status::optimal>(m.get_status()) ? "OK" : "BAD");
        }, 200);
    }
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    vx::quiet_flush = true;
    if(w == "reoffer") reoffer();
    if(w == "points") points();
    if(w == "threads") threads_case();
    if(w == "optnode") managed_optnode();
    if(w == "folding") folding();
    if(w == "parallel") parallel_power();
    if(w == "stop") stop_power();
}
