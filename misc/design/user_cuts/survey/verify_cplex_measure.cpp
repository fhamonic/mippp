// E: the open design questions. mutex | reform | purge | postloop
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static void spin_us(int us) { if(us <= 0) return; const auto u = clk::now() + std::chrono::microseconds(us); while(clk::now() < u) {} }
struct rows { std::vector<std::vector<double>> a; std::vector<double> b; };
static rows read_rows(M & m) {
    rows r; auto [env, lp] = m.native_model();
    const int n = static_cast<int>(m.num_variables()), nr = static_cast<int>(m.num_constraints());
    r.a.assign(static_cast<std::size_t>(nr), std::vector<double>(static_cast<std::size_t>(n), 0.0)); r.b.assign(static_cast<std::size_t>(nr), 0.0);
    std::vector<int> ind(static_cast<std::size_t>(n)); std::vector<double> val(static_cast<std::size_t>(n)); int nz, beg, surplus;
    for(int i = 0; i < nr; ++i) {
        m.native_api()._check(env, m.native_api().getrows(env, lp, &nz, &beg, ind.data(), val.data(), n, &surplus, i, i));
        m.native_api()._check(env, m.native_api().getrhs(env, lp, &r.b[static_cast<std::size_t>(i)], i, i));
        for(int k = 0; k < nz; ++k) r.a[static_cast<std::size_t>(i)][static_cast<std::size_t>(ind[static_cast<std::size_t>(k)])] = val[static_cast<std::size_t>(k)];
    }
    return r;
}
static separator covers(rows r, const int extra_us = 0) {
    return [r, extra_us](const std::vector<double> & x, const add_fn & add) {
        spin_us(extra_us);
        std::vector<std::size_t> order(x.size()); for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
        std::ranges::sort(order, [&](std::size_t p, std::size_t q) { return x[p] > x[q]; });
        for(std::size_t i = 0; i < r.a.size(); ++i) {
            raw_cut cut{{}, {}, 'L', 0}; double w = 0;
            for(std::size_t j : order) { w += r.a[i][j]; cut.ind.push_back(static_cast<int>(j)); cut.val.push_back(1.0); if(w > r.b[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; break; } }
            if(w > r.b[i] && cut.violation(x) > 1e-3) add(cut);
        }
    };
}
static void mutex_q() {
    const double limit = 6;
    for(const int extra : {0, 200, 1000}) for(const int th : {4, 8, 16}) for(const int par : {1, -1}) for(int rep = 0; rep < 2; ++rep) for(const bool mx : {true, false}) {
        M m; build_mkp(m, 30, 100, 7); set_threads(m, th); vparam(m, CPXPARAM_Parallel, par); m.set_time_limit(duration<double>(limit));
        recipe R; R.use_mutex = mx;
        auto & h = vattach(m, covers(read_rows(m), extra), R);
        const double t = timed_solve(m);
        std::println(stderr, "MUTEX extra={}us threads={} parallel={} rep={} mutex={}: nodes={} nodes/s={:.0f} calls/s={:.0f} added={} dual={:.1f}", extra, th, par, rep, mx, m.get_node_count(), static_cast<double>(m.get_node_count()) / t, static_cast<double>(h.hook_calls.load()) / t, h.added.load(), m.get_dual_bound());
    }
}
// Instances for the reform, purge and post-loop questions. Each returns its separator.
struct inst { const char * name; std::function<separator(M &)> build; };
template <class MM>
static separator dup_int(MM & m, const int cols, const int nrows, const unsigned seed) {
    const auto C = static_cast<std::size_t>(cols), R = static_cast<std::size_t>(nrows);
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 60), P(5, 80);
    std::vector<std::vector<double>> a(R, std::vector<double>(C));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(C), b(R);
    for(auto & v : c) v = P(rng);
    for(std::size_t i = 0; i < R; ++i) { double t = 0; for(double v : a[i]) t += v; b[i] = std::floor(t * 1.3); }
    auto x1 = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7});
    auto x2 = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7});
    const auto ids = std::views::iota(std::size_t{0}, C);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * (x1(j) + x2(j)); }));
    for(std::size_t i = 0; i < R; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * (x1(j) + x2(j)); }) <= b[i]);
    std::vector<int> c1, c2;
    for(std::size_t j = 0; j < C; ++j) { c1.push_back(m.native_id(x1(j))); c2.push_back(m.native_id(x2(j))); }
    auto gen = std::make_shared<std::mt19937>(seed * 31u);
    auto mx = std::make_shared<std::mutex>();
    return separator([a, b, c1, c2, gen, mx](const std::vector<double> & p, const add_fn & add) {
        std::lock_guard l(*mx);
        const int ds[] = {2, 3, 4, 5, 7, 11, 13, 17, 19, 23};
        for(int attempt = 0, added = 0; attempt < 200 && added < 3; ++attempt) {
            const std::size_t i = (*gen)() % a.size();
            const double d = ds[(*gen)() % std::size(ds)];
            raw_cut cut{{}, {}, 'L', std::floor(b[i] / d)};
            for(std::size_t j = 0; j < c1.size(); ++j) { const double f = std::floor(a[i][j] / d); if(f == 0) continue; cut.ind.push_back(c1[j]); cut.val.push_back(f); cut.ind.push_back(c2[j]); cut.val.push_back(f); }
            if(cut.violation(p) > 1e-4) { add(cut); ++added; }
        }
    });
}
// Capacitated facility location where every flow column is duplicated: x + x' in all rows at the same cost.
// Cuts: strong linking x + x' <= d y (valid, touches the merged columns), and Sum y >= k (touches binaries only).
template <class MM>
static separator cfl_dup(MM & m, const std::size_t F, const std::size_t K, const unsigned seed, const bool dup, const int families) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> Dm(5, 35), Cap(40, 90), Open(100, 250), Ship(1, 30);
    std::vector<double> d(K), cap(F), open(F);
    for(auto & v : d) v = Dm(rng);
    for(auto & v : cap) v = Cap(rng);
    for(auto & v : open) v = Open(rng);
    std::vector<std::vector<double>> c(F, std::vector<double>(K));
    for(auto & r : c) for(auto & v : r) v = Ship(rng);
    double tot = 0; for(double v : d) tot += v;
    auto y = m.add_binary_variables(F);
    auto x = m.add_variables(F * K, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1e20});
    auto xb = m.add_variables(F * K, {.obj_coef = 0, .lower_bound = 0, .upper_bound = dup ? 1e20 : 0.0});
    const auto FF = std::views::iota(std::size_t{0}, F); const auto KK = std::views::iota(std::size_t{0}, K);
    const auto id = [K](std::size_t f, std::size_t k) { return f * K + k; };
    m.set_minimization();
    m.set_objective(xsum(FF, [&](std::size_t f) { return open[f] * y(f); }) + xsum(FF, [&](std::size_t f) { return xsum(KK, [&, f](std::size_t k) { return c[f][k] * (x(id(f, k)) + xb(id(f, k))); }); }));
    for(std::size_t k : KK) m.add_constraint(xsum(FF, [&, k](std::size_t f) { return x(id(f, k)) + xb(id(f, k)); }) == d[k]);
    for(std::size_t f : FF) m.add_constraint(xsum(KK, [&, f](std::size_t k) { return x(id(f, k)) + xb(id(f, k)); }) - cap[f] * y(f) <= 0);
    std::vector<int> cy; for(std::size_t f : FF) cy.push_back(m.native_id(y(f)));
    std::vector<std::vector<int>> cx(F, std::vector<int>(K)), cxb = cx;
    for(std::size_t f : FF) for(std::size_t k : KK) { cx[f][k] = m.native_id(x(id(f, k))); cxb[f][k] = m.native_id(xb(id(f, k))); }
    std::vector<double> sorted = cap; std::ranges::sort(sorted, std::greater<>{});
    double acc = 0; int need = 0; for(double v : sorted) { acc += v; ++need; if(acc >= tot) break; }
    return separator([=](const std::vector<double> & p, const add_fn & add) {
        if(families & 1) { raw_cut c{cy, std::vector<double>(F, 1.0), 'G', static_cast<double>(need)}; if(c.violation(p) > 1e-4) add(c); }
        if(families & 2) {
            int n = 0;
            for(std::size_t f = 0; f < F && n < 20; ++f) for(std::size_t k = 0; k < K && n < 20; ++k) {
                raw_cut c{{cx[f][k], cxb[f][k], cy[f]}, {1.0, 1.0, -d[k]}, 'L', 0.0};
                if(c.violation(p) > 1e-4) { add(c); ++n; }
            }
        }
    });
}
static std::vector<inst> instances() {
    std::vector<inst> v;
    v.push_back({"dupint25x6s1", [](M & m) { return dup_int(m, 25, 6, 1); }});
    v.push_back({"dupint25x6s2", [](M & m) { return dup_int(m, 25, 6, 2); }});
    v.push_back({"cfl10x20dup,linking", [](M & m) { return cfl_dup(m, 10, 20, 1, true, 3); }});
    v.push_back({"cfl10x20dup,y only", [](M & m) { return cfl_dup(m, 10, 20, 1, true, 1); }});
    v.push_back({"cfl10x20nodup,linking", [](M & m) { return cfl_dup(m, 10, 20, 1, false, 3); }});
    v.push_back({"atsp26s5 DFJ", [](M & m) { auto a = build_atsp(m, 26, false, 5); return cf::dfj<M>(a); }});
    v.push_back({"atsp31s11 DFJ", [](M & m) { auto a = build_atsp(m, 31, false, 11); return cf::dfj<M>(a); }});
    v.push_back({"atsp22s1 complements DFJ", [](M & m) { auto a = build_atsp(m, 22, true, 1); return cf::dfj<M>(a); }});
    v.push_back({"covers mkp5x60s3", [](M & m) { build_mkp(m, 5, 60, 3); return covers(read_rows(m)); }});
    v.push_back({"covers mkp5x100s3", [](M & m) { build_mkp(m, 5, 100, 3); return covers(read_rows(m)); }});
    v.push_back({"covers mkp30x100s7 (limit)", [](M & m) { build_mkp(m, 30, 100, 7); return covers(read_rows(m)); }});
    return v;
}
static void run_q(const char * tag, const std::vector<std::pair<std::string, recipe>> & arms, const int th, const double limit, const int reps) {
    for(const auto & in : instances()) {
        { M m; if(th > 1) set_threads(m, th); else m.set_thread_limit(1); in.build(m); m.set_time_limit(duration<double>(limit)); const double t = timed_solve(m); std::println(stderr, "{} {} threads={} no hook: {}", tag, in.name, th, summary(m, t)); }
        for(int rep = 0; rep < reps; ++rep) for(const auto & [name, R] : arms) {
            M m; if(th > 1) set_threads(m, th); else m.set_thread_limit(1); auto sep = in.build(m); m.set_time_limit(duration<double>(limit));
            auto & h = vattach(m, sep, R);
            const double t = timed_solve(m);
            std::println(stderr, "{} {} threads={} rep={} {}: {} | {} numcuts_user={}", tag, in.name, th, rep, name, summary(m, t), h.line(), vnumcuts(m));
        }
    }
}
int main(int, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argv[1] ? argv[1] : "";
    if(what == "mutex") mutex_q();
    if(what == "reform") {
        recipe a; recipe b; b.reform = 2; recipe c; c.reform = 1;
        run_q("REFORM", {{"Reform=3 swallow 1121", a}, {"Reform=2", b}, {"Reform=1", c}}, 1, 15, 1);
    }
    if(what == "purge") {
        recipe a; recipe b; b.purgeable = CPX_USECUT_FORCE; recipe c; c.purgeable = CPX_USECUT_FILTER;
        run_q("PURGE", {{"PURGE", a}, {"FORCE", b}, {"FILTER", c}}, 1, 15, 1);
    }
    if(what == "cutpass50") {
        for(const auto & in : instances()) {
            const std::string nm = in.name;
            if(nm != "covers mkp5x100s3" && nm != "covers mkp5x60s3" && nm != "atsp31s11 DFJ" && nm != "atsp26s5 DFJ" && nm != "atsp22s1 complements DFJ") continue;
            for(const int cp : {0, 50, 200, 0, 50, 200}) {
                M m; m.set_thread_limit(1); auto sep = in.build(m); if(cp) vparam(m, CPX_PARAM_CUTPASS, cp); m.set_time_limit(duration<double>(15));
                recipe R; R.same_point_guard = true;
                auto & h = vattach(m, sep, R);
                const double t = timed_solve(m);
                std::println(stderr, "CUTPASS={} {}: {} | {}", cp, in.name, summary(m, t), h.line());
            }
        }
    }
    if(what == "post2") {
        recipe a; recipe b; b.skip_post_loop = true; recipe c; c.same_point_guard = true;
        for(const int th : {1, 4}) for(const auto & in : instances()) {
            const std::string nm = in.name;
            if(nm != "covers mkp5x100s3" && nm != "covers mkp30x100s7 (limit)" && nm != "atsp31s11 DFJ" && nm != "covers mkp5x60s3") continue;
            for(int rep = 0; rep < 2; ++rep) for(const auto & [name, R] : std::initializer_list<std::pair<const char *, recipe>>{{"keep", a}, {"skip-post", b}, {"same-point-guard", c}}) {
                M m; if(th > 1) set_threads(m, th); else m.set_thread_limit(1); auto sep = in.build(m); m.set_time_limit(duration<double>(10));
                auto & h = vattach(m, sep, R);
                const double t = timed_solve(m);
                std::println(stderr, "POST2 {} threads={} rep={} {}: {} | {}", in.name, th, rep, name, summary(m, t), h.line());
            }
        }
    }
    if(what == "post") {
        recipe a; recipe b; b.skip_post_loop = true;
        run_q("POST", {{"keep post-loop", a}, {"skip post-loop", b}}, 1, 15, 1);
        run_q("POST", {{"keep post-loop", a}, {"skip post-loop", b}}, 4, 15, 2);
    }
}
