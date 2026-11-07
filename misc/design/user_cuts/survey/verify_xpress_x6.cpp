// X5 smallest models + X6 remedy. Public API part first, then the native remedy on the same tiny model
// and degree-only ATSP. Also why the shipped test misses: soltype counts on build_small.
#include "verify_xpress_hook.hpp"
using namespace mippp; using namespace mippp::operators;
using preint_t = void(prob_t, void *, int, int *, double *);
namespace {
struct lz {
    const xn::xpress_api * api; std::shared_ptr<vx::fns> fn; prob_t main;
    std::function<std::vector<cp::raw_cut>(const std::vector<double> &)> sep;
    bool set_reject_on_0 = false;  // the reject-only recipe
    bool use_presolverow = true;
    long sol0 = 0, sol1 = 0, sol2 = 0, rej0 = 0, rej1 = 0, cuts0 = 0, errs = 0, pst_neg = 0;
    std::map<int, long> status;
};
void preint(prob_t cb, void * d, int soltype, int * p_reject, double *) {
    auto & c = *static_cast<lz *>(d);
    try {
        (soltype == 0 ? c.sol0 : soltype == 1 ? c.sol1 : c.sol2)++;
        int input = 0; c.api->getintattrib(cb, vx::A_INPUTCOLS, &input);
        std::vector<double> x(static_cast<std::size_t>(input)); int avail = 0;
        if(c.api->getcallbacksolution(cb, &avail, x.data(), 0, input - 1)) { ++c.errs; return; }
        const auto cuts = c.sep(x);
        if(cuts.empty()) return;
        if(soltype != 0 || c.set_reject_on_0) { *p_reject = 1; (soltype == 0 ? c.rej0 : c.rej1)++; return; }
        for(const auto & cut : cuts) {
            int cols = 0, n = 0, st = -99; c.api->getintattrib(cb, vx::A_COLS, &cols);
            std::vector<int> pi(static_cast<std::size_t>(cols) + 1); std::vector<double> pv(static_cast<std::size_t>(cols) + 1); double prhs = 0;
            int e = c.fn->presolverow(cb, cut.sense, static_cast<int>(cut.ind.size()), cut.ind.data(), cut.val.data(), cut.rhs, cols, &n, pi.data(), pv.data(), &prhs, &st);
            c.status[st]++;
            if(e || st < 0) { ++c.pst_neg; *p_reject = 1; ++c.rej0; return; }
            const int s2[2] = {0, n}; const int type = 1;
            e = c.api->addcuts(cb, 1, &type, &cut.sense, &prhs, s2, pi.data(), pv.data());
            if(e) ++c.errs; else ++c.cuts0;
        }
    } catch(...) { c.api->interrupt(cb, vx::STOP_USER); }
}
std::vector<std::unique_ptr<lz>> keep;
lz & attach_lz(M & m, std::function<std::vector<cp::raw_cut>(const std::vector<double> &)> sep, bool reject_only) {
    keep.push_back(std::make_unique<lz>());
    auto & c = *keep.back();
    c.api = &m.native_api(); c.fn = std::make_shared<vx::fns>(m.native_api().library_path()); c.main = m.native_model(); c.sep = std::move(sep); c.set_reject_on_0 = reject_only;
    const int e = m.native_api().addcbpreintsol(m.native_model(), preint, &c, 0);
    if(e) std::exit(2);
    return c;
}
std::string line(lz & c) {
    std::string s = std::format("soltype0={} soltype1={} soltype2={} rejected0={} rejected1={} cuts_in_preintsol={} errors={} presolverow_neg={} status{{", c.sol0, c.sol1, c.sol2, c.rej0, c.rej1, c.cuts0, c.errs, c.pst_neg);
    for(auto [k, v] : c.status) s += std::format("{}:{} ", k, v);
    return s + "}";
}
void tiny(bool dualred0, bool reject_only) {
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(10));
    auto x = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
    auto y = m.add_integer_variable({.lower_bound = 0., .upper_bound = 5.});
    m.add_constraint(x + y <= 8.);
    m.set_maximization(); m.set_objective(x + y);
    if(dualred0) vx::setint(m, vx::C_MIPDUALREDUCTIONS, 0);
    const int ix = m.native_id(x), iy = m.native_id(y);
    auto & c = attach_lz(m, [=](const std::vector<double> & p) -> std::vector<cp::raw_cut> {
        if(p[static_cast<std::size_t>(ix)] + p[static_cast<std::size_t>(iy)] > 7.5) return {cp::raw_cut{{ix, iy}, {1.0, 1.0}, 'L', 7.0}};
        return {};
    }, reject_only);
    const double t = cp::timed_solve(m);
    std::println(stderr, "X6 tiny int (truth 7) {} dualreductions={}: {} | {}", reject_only ? "reject only" : "preintsol lazy", dualred0 ? 0 : 1, cp::summary(m, t), line(c));
}
// Degree-only ATSP: only the two assignment rows, no MTZ.
template <class Mo> cp::atsp degree_only(Mo & m, int n, unsigned seed) {
    cp::atsp a; a.n = n;
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
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
// one DFJ cut per cycle of an integral point; empty for a tour
std::vector<cp::raw_cut> subtours(const cp::atsp & a, const std::vector<double> & x) {
    const auto N = static_cast<std::size_t>(a.n);
    std::vector<int> next(N, -1);
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && x[static_cast<std::size_t>(a.col[i][j])] > 0.5) next[i] = static_cast<int>(j);
    std::vector<int> label(N, -1); int k = 0;
    for(std::size_t s0 = 0; s0 < N; ++s0) if(label[s0] < 0) { for(std::size_t v = s0; label[v] < 0; v = static_cast<std::size_t>(next[v])) label[v] = k; ++k; }
    std::vector<cp::raw_cut> out;
    if(k == 1) return out;
    for(int q = 0; q < k; ++q) {
        cp::raw_cut c; c.sense = 'G'; c.rhs = 1;
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && label[i] == q && label[j] != q) { c.ind.push_back(a.col[i][j]); c.val.push_back(1); }
        out.push_back(std::move(c));
    }
    return out;
}
void degree(int n, unsigned seed, int kind, int dualred, double truth) {
    // kind 0: preintsol + addcuts; 1: reject only; 2: optnode cuts only (separate on integral points too), no preintsol
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(120));
    auto a = degree_only(m, n, seed);
    if(dualred >= 0) vx::setint(m, vx::C_MIPDUALREDUCTIONS, dualred);
    lz * pc = nullptr;
    if(kind <= 1) pc = &attach_lz(m, [&a](const std::vector<double> & p) { return subtours(a, p); }, kind == 1);
    if(kind == 2) {
        vx::recipe r = vx::parse("R2");
        vx::attach(m, [&a](const std::vector<double> & p, const cp::add_fn & add) { bool integer = true; for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) { double v = p[static_cast<std::size_t>(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)])]; if(std::fabs(v - std::round(v)) > 1e-6) integer = false; } if(integer) for(const auto & c : subtours(a, p)) add(c); }, r);
    }
    const double t = cp::timed_solve(m);
    double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
    std::string tour = "n/a";
    if(status::solution_available(m.get_status())) { auto sol = m.get_solution(); std::vector<double> xv(static_cast<std::size_t>(m.num_variables())); for(int k = 0; k < static_cast<int>(xv.size()); ++k) { /* native ids are in column order */ } tour = "readable"; }
    std::println(stderr, "X6 degree-only n={} seed={} {} MIPDUALREDUCTIONS={}: {} {} | {}", n, seed, kind == 0 ? "preintsol+presolverow+addcuts" : kind == 1 ? "reject only" : "optnode cuts only", dualred, cp::summary(m, t), truth > 0 ? (std::fabs(v - truth) < 1e-6 ? "RIGHT" : std::format("WRONG (truth {})", truth)) : "", pc ? line(*pc) : "");
}
void shipped_build_small() {
    // Same arrays as progress_callback.hpp build_knapsack(5, 25, false), with soltype counts: why does the test pass?
    M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(30));
    std::mt19937 rng(7); std::uniform_int_distribution<int> coef(1, 1000), noise(0, 500);
    const std::size_t rows = 5, cols = 25;
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & row : a) for(auto & v : row) v = coef(rng);
    std::vector<double> c(cols);
    for(std::size_t j = 0; j < cols; ++j) { double sum = 0; for(const auto & row : a) sum += row[j]; c[j] = std::floor(sum / static_cast<double>(rows) + noise(rng)); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    m.set_minimization(); m.set_objective(xsum(ids, [&](auto j) { return c[j] * x(j); }));
    for(const auto & row : a) { double sum = 0; for(double v : row) sum += v; m.add_constraint(xsum(ids, [&](auto j) { return row[j] * x(j); }) >= std::floor(sum / 2)); }
    const int i0 = m.native_id(x(std::size_t{0})), i1 = m.native_id(x(std::size_t{1}));
    auto & l = attach_lz(m, [=](const std::vector<double> & p) -> std::vector<cp::raw_cut> { if(p[static_cast<std::size_t>(i0)] + p[static_cast<std::size_t>(i1)] > 1.5) return {cp::raw_cut{{i0, i1}, {1.0, 1.0}, 'L', 1.0}}; return {}; }, true);
    const double t = cp::timed_solve(m);
    std::println(stderr, "build_small rule x0+x1>1.5, reject: {} | {}", cp::summary(m, t), line(l));
}
}
int main() {
    tiny(false, true); tiny(true, true); tiny(false, false); tiny(true, false);
    shipped_build_small();
    double truth14 = 0, truth26 = 0, truth40 = 0;
    for(auto [n, tr] : std::vector<std::pair<int, double *>>{{14, &truth14}, {26, &truth26}, {40, &truth40}}) {
        // Truth: lazy with dual reductions off is one candidate; cross-check on the complete MTZ model where it finishes.
        M m; m.set_thread_limit(1); m.set_time_limit(std::chrono::duration<double>(n <= 26 ? 60 : 100));
        auto a = cp::build_atsp(m, n, false, 5);
        auto s = std::make_shared<cp::stats>();
        vx::attach(m, cp::watched([a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s), vx::parse("R2"));
        const double t = cp::timed_solve(m);
        std::println(stderr, "truth MTZ n={}: {}", n, cp::summary(m, t));
        if(is_a<status::optimal>(m.get_status())) *tr = m.get_solution_value();
    }
    for(auto [n, tr] : std::vector<std::pair<int, double>>{{14, truth14}, {26, truth26}, {40, truth40}}) {
        for(const int dr : {0, 1, 2, -1}) degree(n, 5, 0, dr, tr);
        for(const int dr : {0, -1}) { degree(n, 5, 1, dr, tr); degree(n, 5, 2, dr, tr); }
    }
}
