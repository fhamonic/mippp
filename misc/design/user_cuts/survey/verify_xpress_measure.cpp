// R1 against R2 at default settings: ATSP n=40, 60 and knapsack covers. One thread. Also the p_action variants.
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
using cut = cp::raw_cut;
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
static const char * specs[] = {"", "R1", "R1,add2", "R1,first2", "R1,add2,first2", "R2", "R2cr", "R2cr,add2"};
// Degree-only ATSP for n=60: the complete MTZ model is over the licence limit. Lazy DFJ in preintsol, user cuts through the recipe.
namespace lazy60 {
struct L { const xn::xpress_api * api; std::shared_ptr<vx::fns> fn; const cp::atsp * a; long cand0 = 0, cand1 = 0, cuts0 = 0; };
inline std::vector<cut> subtours(const cp::atsp & a, const std::vector<double> & x) {
    const auto N = static_cast<std::size_t>(a.n);
    std::vector<int> next(N, -1);
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && x[static_cast<std::size_t>(a.col[i][j])] > 0.5) next[i] = static_cast<int>(j);
    std::vector<int> label(N, -1); int k = 0;
    for(std::size_t s0 = 0; s0 < N; ++s0) if(label[s0] < 0) { for(std::size_t v = s0; label[v] < 0; v = static_cast<std::size_t>(next[v])) label[v] = k; ++k; }
    std::vector<cut> out;
    if(k == 1) return out;
    for(int q = 0; q < k; ++q) {
        cut c; c.sense = 'G'; c.rhs = 1;
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && label[i] == q && label[j] != q) { c.ind.push_back(a.col[i][j]); c.val.push_back(1); }
        out.push_back(std::move(c));
    }
    return out;
}
inline void preint(prob_t cb, void * d, int soltype, int * p_reject, double *) {
    auto & c = *static_cast<L *>(d);
    (soltype == 0 ? c.cand0 : c.cand1)++;
    int input = 0; c.api->getintattrib(cb, vx::A_INPUTCOLS, &input);
    std::vector<double> x(static_cast<std::size_t>(input)); int avail = 0;
    if(c.api->getcallbacksolution(cb, &avail, x.data(), 0, input - 1)) return;
    const auto cuts = subtours(*c.a, x);
    if(cuts.empty()) return;
    if(soltype != 0) { *p_reject = 1; return; }
    for(const auto & k : cuts) {
        int cols = 0, n = 0, st = -99; c.api->getintattrib(cb, vx::A_COLS, &cols);
        std::vector<int> pi(static_cast<std::size_t>(cols) + 1); std::vector<double> pv(static_cast<std::size_t>(cols) + 1); double prhs = 0;
        if(c.fn->presolverow(cb, k.sense, static_cast<int>(k.ind.size()), k.ind.data(), k.val.data(), k.rhs, cols, &n, pi.data(), pv.data(), &prhs, &st) || st < 0) { *p_reject = 1; return; }
        const int s2[2] = {0, n}; const int type = 1;
        if(!c.api->addcuts(cb, 1, &type, &k.sense, &prhs, s2, pi.data(), pv.data())) ++c.cuts0;
    }
}
template <class Mo> cp::atsp build(Mo & m, int n, unsigned seed) {
    cp::atsp a; a.n = n;
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(N * (N - 1));
    a.col.assign(N, std::vector<int>(N, -1));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N);
    m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); }); }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
}
template <class Build>
static void row_inner(const std::string & name, const std::string & spec, double limit, Build build) {
    M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(limit));
    auto sep = build(m);
    auto s = std::make_shared<cp::stats>();
    vx::ctx * c = nullptr;
    if(!spec.empty()) c = &vx::attach(m, cp::watched(sep, s), vx::parse(spec));
    const double t = cp::timed_solve(m);
    std::println(stderr, "{} [{}]: {} | calls={} cuts_native={} dropped={} nodes_seen={} depth={} rows {}->{} CUTS_max={} restarts={} errs={}", name, spec.empty() ? "no hook" : spec, cp::summary(m, t), s->calls.load(), c ? c->added_native.load() : 0, c ? c->dropped.load() : 0, c ? c->nodes.size() : 0, c ? c->max_depth : 0, c ? c->rows_first : 0, c ? c->rows_max : 0, c ? c->cuts_max : 0, c ? c->restarts : 0, s->add_errors.load());
}
template <class Build>
static void row(const std::string & name, const std::string & spec, double limit, Build build) {
    std::fflush(stderr);
    const pid_t p = fork();
    if(p == 0) { alarm(static_cast<unsigned>(limit) + 15); row_inner(name, spec, limit, build); std::fflush(stderr); _exit(0); }
    int st = 0; waitpid(p, &st, 0);
    if(WIFSIGNALED(st)) std::println(stderr, "{} [{}]: IGNORED THE TIME LIMIT {} s, killed by signal {} at limit + 15 s", name, spec, limit, WTERMSIG(st));
    else if(WEXITSTATUS(st)) std::println(stderr, "{} [{}]: exit {}", name, spec, WEXITSTATUS(st));
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    vx::quiet_flush = true;
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    if(w == "atsp") for(int n : {40, 50}) for(unsigned seed : {5u, 1u, 2u}) for(const char * sp : specs) {
        if(std::string(sp).empty() && n == 50) continue;
        row(std::format("ATSP n={} seed={}", n, seed), sp, 100, [&](M & m) { auto a = cp::build_atsp(m, n, false, seed); return cp::separator([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }); });
    }
    if(w == "lazy60") for(int n : {60, 80}) for(unsigned seed : {5u, 1u, 2u}) for(const char * sp : {"", "R1", "R1,add2", "R2", "R2cr"}) {
        if(n == 80 && std::string(sp) == "R1,add2") continue;
        std::fflush(stderr);
        const pid_t p = fork();
        if(p == 0) {
            alarm(95);
            M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(60));
            vx::setint(m, vx::C_MIPDUALREDUCTIONS, 0);
            auto a = lazy60::build(m, n, seed);
            lazy60::L l{&m.native_api(), std::make_shared<vx::fns>(m.native_api().library_path()), &a};
            m.native_api().addcbpreintsol(m.native_model(), lazy60::preint, &l, 0);
            auto s = std::make_shared<cp::stats>();
            vx::ctx * c = nullptr;
            if(sp[0]) c = &vx::attach(m, cp::watched([a, n](const std::vector<double> & x, const cp::add_fn & add) { if(n > 64) return; for(const auto & k : a.separate(x)) add(k); }, s), vx::parse(sp));
            const double t = cp::timed_solve(m);
            std::println(stderr, "degree-only lazy n={} seed={} [{}]: {} | candidates soltype0={} other={} lazy_cuts={} | user calls={} cuts={} dropped={} seen={} depth={} rows {}->{} CUTS={}", n, seed, sp[0] ? sp : "lazy only", cp::summary(m, t), l.cand0, l.cand1, l.cuts0, s->calls.load(), c ? c->added_native.load() : 0, c ? c->dropped.load() : 0, c ? c->nodes.size() : 0, c ? c->max_depth : 0, c ? c->rows_first : 0, c ? c->rows_max : 0, c ? c->cuts_max : 0);
            std::fflush(stderr);
            _exit(0);
        }
        int st = 0; waitpid(p, &st, 0);
        if(WIFSIGNALED(st)) std::println(stderr, "degree-only lazy n={} seed={} [{}]: killed by signal {} at 95 s (limit 60 s)", n, seed, sp, WTERMSIG(st));
    }
    if(w == "hang") {
        const std::string sp = argc > 2 ? argv[2] : "R1,add2,first2";
        const double lim = argc > 3 ? std::atof(argv[3]) : 5;
        const unsigned seed = argc > 4 ? static_cast<unsigned>(std::atoi(argv[4])) : 1u;
        M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(lim));
        const std::size_t ncols = argc > 5 ? static_cast<std::size_t>(std::atoi(argv[5])) : 40;
        auto [col, a, cap] = knap(m, seed, ncols, 5);
        auto s = std::make_shared<cp::stats>();
        static std::atomic<double> last_viol{0}, last_obj{0}, sum{0};
        static std::atomic<long> last_terms{0};
        auto inner = covers(a, cap, col);
        const auto probe = [inner](const std::vector<double> & x, const cp::add_fn & add) {
            double h = 0; for(double v : x) h = h * 0.9999 + v;
            sum = h;
            inner(x, [&](const cut & k) { last_viol = k.violation(x); last_terms = static_cast<long>(k.ind.size()); return add(k); });
        };
        auto & c = vx::attach(m, cp::watched(probe, s), vx::parse(sp));
        std::atomic<bool> stop{false};
        const auto t0 = clk::now();
        std::thread dog([&] { while(!stop) { std::this_thread::sleep_for(std::chrono::seconds(5)); std::println(stderr, "  t={:.0f}s fired={} native={} dropped={} nodes={} rounds={} depth={} rows {}->{} CUTS_max={} last_viol={:.3g} terms={} point_hash={:.12g}", std::chrono::duration<double>(clk::now() - t0).count(), c.fired.load(), c.added_native.load(), c.dropped.load(), c.nodes.size(), c.max_rounds, c.max_depth, c.rows_first, c.rows_max, c.cuts_max, last_viol.load(), last_terms.load(), sum.load()); } });
        const double t = cp::timed_solve(m);
        stop = true; dog.join();
        std::println(stderr, "HANG [{}] limit {}: {} | fired={} native={} dropped={} nodes={} rounds={}", sp, lim, cp::summary(m, t), c.fired.load(), c.added_native.load(), c.dropped.load(), c.nodes.size(), c.max_rounds);
    }
    if(w == "knap") for(auto [cols, rows] : std::vector<std::pair<std::size_t, std::size_t>>{{40, 5}, {60, 5}, {80, 10}}) for(unsigned seed : {1u, 2u, 3u}) for(const char * sp : specs) {
        row(std::format("covers {}x{} seed={}", cols, rows, seed), sp, 60, [&](M & m) { auto [col, a, cap] = knap(m, seed, cols, rows); return covers(a, cap, col); });
    }
}
