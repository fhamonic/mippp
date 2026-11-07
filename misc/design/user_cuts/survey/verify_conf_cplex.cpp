// The CPLEX recipe under the shared conformance set, plus X cases for what the set has no power on here.
#include "verify_cplex_common.hpp"
using namespace cp;
// Duplicated general-integer columns: presolve merges them at Reformulations=3 and any cut touching them is 1121.
template <class MM>
static separator build_dup_int(MM & m, const int cols, const int rows, const unsigned seed) {
    const auto C = static_cast<std::size_t>(cols), R = static_cast<std::size_t>(rows);
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
            for(std::size_t j = 0; j < c1.size(); ++j) {
                const double f = std::floor(a[i][j] / d);
                if(f == 0) continue;
                cut.ind.push_back(c1[j]); cut.val.push_back(f);
                cut.ind.push_back(c2[j]); cut.val.push_back(f);
            }
            if(cut.violation(p) > 1e-4) { add(cut); ++added; }
        }
    });
}
template <class MM>
static separator build_covers(MM & m, const std::size_t cols, const std::size_t rows, const unsigned seed) {
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
    std::vector<int> col;
    for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
    return separator([a, cap, col](const std::vector<double> & p, const add_fn & add) {
        for(std::size_t i = 0; i < a.size(); ++i) {
            std::vector<std::size_t> order(col.size());
            for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
            std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
            double w = 0; raw_cut cut{{}, {}, 'L', 0};
            for(std::size_t j : order) {
                w += a[i][j]; cut.ind.push_back(col[j]); cut.val.push_back(1);
                if(w > cap[i]) { cut.rhs = static_cast<double>(cut.ind.size()) - 1; if(cut.violation(p) > 1e-4) add(cut); break; }
            }
        }
    });
}
int main() {
    cf::setup<M> B;
    recipe R;
    if(std::getenv("NC_NO_DROP")) R.drop_satisfied = false;
    if(std::getenv("NC_NO_FOLD")) R.fold = false;
    if(std::getenv("NC_NO_MUTEX")) R.use_mutex = false;
    if(std::getenv("NC_ABORT_1121")) R.abort_on_1121 = true;
    if(std::getenv("NC_FORCE")) R.purgeable = CPX_USECUT_FORCE;
    if(std::getenv("NC_FILTER")) R.purgeable = CPX_USECUT_FILTER;
    if(std::getenv("NC_LOCAL")) R.local = 1;
    if(std::getenv("NC_SKIP_POST")) R.skip_post_loop = true;
    if(const char * e = std::getenv("NC_REFORM")) R.reform = std::atoi(e);
    if(const char * e = std::getenv("RECIPE_INF_TOL")) R.inf_tol = std::atof(e);
    if(std::getenv("RECIPE_GUARD")) R.same_point_guard = true;
    B.attach = [R](M & m, separator sep) { vattach(m, std::move(sep), R); };
    B.one_thread = [](M & m) { m.set_thread_limit(1); };
    B.atsp_sizes = {22, 24, 31};
    B.stop_size = 31;
    B.first_limit = 0.02;
    B.fuzz_seeds = 20;
    B.limit = 60;
    if(const char * e = std::getenv("CONF_LIMIT")) B.limit = std::atof(e);
    if(const char * e = std::getenv("CONF_SIZES")) {
        B.atsp_sizes.clear();
        std::string s = e; std::size_t at = 0;
        while(at < s.size()) { auto n = s.find(',', at); if(n == std::string::npos) n = s.size(); B.atsp_sizes.push_back(std::atoi(s.substr(at, n - at).c_str())); at = n + 1; }
    }
    if(const char * e = std::getenv("CONF_FUZZ")) B.fuzz_seeds = std::atoi(e);
    { M m; print_version(m); }
    if(!std::getenv("CONF_ONLY_X")) cf::run(B);
    // X6: re-offers on a model that needs a tree: C6 closes at the root here, so a satisfied re-add has no time to spin
    for(const unsigned seed : {1u, 2u}) cf::compare(B, std::format("X6a covers 40x5, every cut ever found re-offered at every call, seed={}", seed), [=](M & m) {
        auto base = build_covers(m, 40, 5, seed);
        auto all = std::make_shared<std::vector<raw_cut>>();
        auto mx = std::make_shared<std::mutex>();
        return separator([base, all, mx](const std::vector<double> & x, const add_fn & add) {
            std::vector<raw_cut> fresh;
            base(x, [&](const raw_cut & c) { fresh.push_back(c); return 0; });
            std::lock_guard l(*mx);
            for(auto & c : fresh) all->push_back(std::move(c));
            for(const auto & c : *all) add(c);
        });
    });
    cf::compare(B, "X6b constant satisfied one-term cut at every call, atsp22 seed=1", [=](M & m) {
        auto a = build_atsp(m, 22, false, 1);
        const int c0 = a.col[0][1];
        return separator([c0](const std::vector<double> &, const add_fn & add) { add(raw_cut{{c0}, {1.0}, 'L', 1.0}); });
    });
    // X7: duplicate entries in a violated, valid cut: -1 x_j + 2 x_j means x_j. Last-entry-wins makes it 2 x_j, invalid.
    for(const unsigned seed : {1u, 2u, 3u}) cf::compare(B, std::format("X7 covers with one term written as -1 + 2, seed={}", seed), [=](M & m) {
        auto base = build_covers(m, 40, 5, seed);
        return separator([base](const std::vector<double> & x, const add_fn & add) {
            base(x, [&](const raw_cut & c) {
                raw_cut d = c;
                const std::size_t t = static_cast<std::size_t>(std::ranges::max_element(c.ind, {}, [&](int j) { return x[static_cast<std::size_t>(j)]; }) - c.ind.begin());
                d.val[t] = -1.0; d.ind.push_back(c.ind[t]); d.val.push_back(2.0);
                return add(d);
            });
        });
    });
    // X4: the 1121 path, which C4's binary parallel columns never reach here
    for(const unsigned seed : {1u, 2u, 3u}) cf::compare(B, std::format("X4 duplicated general integers, cuts on them, seed={}", seed), [=](M & m) { return build_dup_int(m, 25, 6, seed); });
    // X9: default threads with a real tree, which C9's seed never gives on CPLEX
    for(int rep = 0; rep < 3; ++rep) cf::compare(B, std::format("X9a default threads, atsp31 seed=11, rep={}", rep), [=](M & m) { auto a = build_atsp(m, 31, false, 11); return cf::dfj<M>(a); }, true);
    for(const unsigned seed : {1u, 2u, 3u}) cf::compare(B, std::format("X9b default threads, covers 40x5 seed={}", seed), [=](M & m) { return build_covers(m, 40, 5, seed); }, true);
    return cf::finish();
}
