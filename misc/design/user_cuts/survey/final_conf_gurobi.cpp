// final: the Gurobi recipe with cf::sanitize replaced by fp::prepare (final_prepare.hpp).
// argv[1]: conf (default), sweep, sweep_ns, bigm, tiny, harm, oracle.
// Skeptic's run of the Gurobi recipe under the shared conformance set.
// Env knobs remove one piece of the recipe at a time (negative controls):
//   VG_NO_PRECRUSH, VG_NO_FILTER, VG_NO_FOLD, VG_NO_STRIP, VG_NO_EMPTY, VG_KEEP_SATISFIED
// Sizes: VG_ATSP="26,30", VG_SEEDS="1,2,3", VG_STOP, VG_FIRST, VG_FUZZ, VG_LIMIT.
// VG_POWER=1 runs C1 only, to pick sizes that exercise the hook.
#include "mippp/solvers/gurobi/all.hpp"
#include "cut_conformance.hpp"
#include "final_prepare.hpp"
#include "final_tests.hpp"
using M = mippp::gurobi_milp;
namespace g = mippp::gurobi::impl::v1;
enum { W_MIPNODE = 5, NODE_STATUS = 5001, NODE_REL = 5002, NODE_NODCNT = 5005, ST_OPTIMAL = 2 };

static bool env_on(const char * n) { return std::getenv(n) != nullptr; }
static double env_dbl(const char * n, double d) { const char * v = std::getenv(n); return v ? std::atof(v) : d; }
template <class T>
static std::vector<T> env_list(const char * n, std::vector<T> d) {
    const char * v = std::getenv(n);
    if(!v) return d;
    std::vector<T> out; std::string s(v), cur;
    for(char c : s + ",") { if(c == ',') { if(!cur.empty()) out.push_back(static_cast<T>(std::atoi(cur.c_str()))); cur.clear(); } else cur += c; }
    return out;
}
static const bool NO_PRECRUSH = env_on("VG_NO_PRECRUSH"), NO_FILTER = env_on("VG_NO_FILTER"), NO_FOLD = env_on("VG_NO_FOLD"),
                  NO_STRIP = env_on("VG_NO_STRIP"), NO_EMPTY = env_on("VG_NO_EMPTY"), KEEP_SAT = env_on("VG_KEEP_SATISFIED");

struct totals {
    std::map<int, long> status;
    long rel_err = 0, garbage_points = 0, tree_calls = 0, root_calls = 0, tree_cuts = 0, root_cuts = 0, native_err = 0, swallowed = 0, dropped_satisfied = 0, dropped_empty = 0;
} T;

// cf::sanitize, with one piece removable at a time.
static std::optional<cp::raw_cut> clean(const cp::raw_cut & in, const std::vector<double> & x) {
    if(!NO_FOLD && !NO_STRIP && !NO_EMPTY) {
        const auto r = cf::sanitize(in, x, !KEEP_SAT);
        if(!r) { if(in.ind.empty()) ++T.dropped_empty; else ++T.dropped_satisfied; }
        return r;
    }
    cp::raw_cut out{{}, {}, in.sense, in.rhs};
    if(NO_FOLD) {
        for(std::size_t k = 0; k < in.ind.size(); ++k) if(NO_STRIP || in.val[k] != 0.0) { out.ind.push_back(in.ind[k]); out.val.push_back(in.val[k]); }
    } else {
        std::map<int, double> folded;
        for(std::size_t k = 0; k < in.ind.size(); ++k) folded[in.ind[k]] += in.val[k];
        for(auto [j, v] : folded) if(NO_STRIP || v != 0.0) { out.ind.push_back(j); out.val.push_back(v); }
    }
    if(out.ind.empty() && !NO_EMPTY) { ++T.dropped_empty; return std::nullopt; }
    if(!KEEP_SAT && !out.ind.empty() && out.violation(x) <= 1e-9) { ++T.dropped_satisfied; return std::nullopt; }
    return out;
}

struct ctx { cp::separator sep; const g::gurobi_api * api; int cols; };
static std::vector<std::unique_ptr<ctx>> keep;
static int hook(g::GRBmodel *, void * cbdata, int where, void * usr) {
    if(where != W_MIPNODE) return 0;
    auto & c = *static_cast<ctx *>(usr);
    try {
        int status = 0; c.api->cbget(cbdata, where, NODE_STATUS, &status);
        T.status[status]++;
        if(status != 2 && status != 3 && status != 6) {
            double nod0 = -1; c.api->cbget(cbdata, where, NODE_NODCNT, &nod0);
            std::vector<double> x0(static_cast<std::size_t>(c.cols), -7.0);
            const int rel0 = c.api->cbget(cbdata, where, NODE_REL, x0.data());
            std::println(stderr, "   odd MIPNODE status {} at nodcnt {}: REL returns {}, x[0]={:g}", status, nod0, rel0, x0[0]);
        }
        if(status != ST_OPTIMAL && !NO_FILTER) return 0;
        double nod = 0; c.api->cbget(cbdata, where, NODE_NODCNT, &nod);
        std::vector<double> x(static_cast<std::size_t>(c.cols), -7.0);
        const int rel = c.api->cbget(cbdata, where, NODE_REL, x.data());
        if(rel) ++T.rel_err;
        if(std::ranges::any_of(x, [](double v) { return v > 1e20 || v == -7.0; })) ++T.garbage_points;
        ++(nod == 0 ? T.root_calls : T.tree_calls);
        c.sep(x, [&](const cp::raw_cut & raw) {
            int e = 0;
            for(const auto & r : fp::prepare(raw, x)) {
                e = c.api->cbcut(cbdata, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(), r.sense == 'E' ? '=' : r.sense == 'L' ? '<' : '>', r.rhs);
                if(e) { ++T.native_err; break; } else ++(nod == 0 ? T.root_cuts : T.tree_cuts);
            }
            return e;
        });
    } catch(...) { ++T.swallowed; }
    return 0;
}

static void write_oracle() {
    std::vector<std::pair<std::string, std::function<cp::separator(M &)>>> keys;
    for(unsigned s = 1; s <= 5; ++s) keys.push_back({std::format("cov40x5s{}", s), [s](M & m) { return ft::covers(m, 40, 5, s); }});
    for(unsigned s = 1; s <= 5; ++s) keys.push_back({std::format("atsp16s{}", s), [s](M & m) { return ft::atsp_dfj(m, 16, s); }});
    for(unsigned s : {9u, 10u}) keys.push_back({std::format("cov40x5s{}", s), [s](M & m) { return ft::covers(m, 40, 5, s); }});
    for(int s : {7, 23, 33}) keys.push_back({std::format("cg{}", s), [s](M & m) { return ft::cgfuzz(m, s); }});
    for(unsigned s = 4; s <= 10; ++s) keys.push_back({std::format("fctp_u1_s{}", s), [s](M & m) { return ft::fctp(m, s, 1.0, 0, 0, true); }});
    for(unsigned s = 1; s <= 3; ++s) {
        keys.push_back({std::format("fctp_u1_s{}", s), [s](M & m) { return ft::fctp(m, s, 1.0, 0, 0, true); }});
        for(double Mc : {1e3, 1e6, 1e8, 1e10}) { const double u = Mc / 30.0; keys.push_back({std::format("fctp_u{}_s{}", ft::sci(u), s), [s, u](M & m) { return ft::fctp(m, s, u, 0, 0, true); }}); }
        for(double Mb : {1e3, 1e6, 1e8, 1e10, 1e12}) keys.push_back({std::format("tiny2_s{}_M{}", s, ft::sci(Mb)), [s, Mb](M & m) { return ft::tiny2(m, s, Mb); }});
    }
    for(unsigned s : {1u, 2u, 3u}) keys.push_back({std::format("mcnd12s{}", s), [s](M & m) { return ft::mcnd(m, s, 12, 24, 15); }});
    for(unsigned s : {1u, 2u}) keys.push_back({std::format("gi25x6s{}", s), [s](M & m) { return ft::genint_cg(m, s); }});
    std::ofstream f(ft::oracle_path());
    for(auto & [k, b] : keys) {
        M m; auto sep = b(m);
        m.set_optimality_tolerance(0.0);
        m.set_time_limit(std::chrono::duration<double>(300));
        const double t = cp::timed_solve(m);
        const bool opt = is_a<mippp::status::optimal>(m.get_status());
        std::println(stderr, "ORACLE {} optimal={} {}", k, opt, cp::summary(m, t));
        f << k << ' ' << std::format("{:.17g}", opt ? m.get_solution_value() : NAN) << '\n';
    }
}

int main(int argc, char ** argv) {
    using std::chrono::duration;
    const std::string mode = argc > 1 ? argv[1] : "conf";
    if(mode == "oracle") { write_oracle(); return 0; }
    if(mode != "conf") {
        ft::bed<M> E;
        E.name = "gurobi";
        E.attach = [](M & m, cp::separator sep) {
            auto [env, model] = m.native_model();
            auto & api = m.native_api();
            api._check(env, api.setintparam(env, "PreCrush", 1));
            keep.push_back(std::make_unique<ctx>(ctx{std::move(sep), &api, static_cast<int>(m.num_variables())}));
            api._check(env, api.setcallbackfunc(model, hook, keep.back().get()));
        };
        E.one_thread = [](M & m) { m.set_thread_limit(1); };
        E.presolve_off = [](M & m) { auto [env, model] = m.native_model(); m.native_api()._check(env, m.native_api().setintparam(env, "Presolve", 0)); };
        E.recipe_line = [] { return std::format("native_err_total={}", T.native_err); };
        if(!ft::run_mode(E, mode)) { std::println(stderr, "unknown mode {}", mode); return 2; }
        return 0;
    }
    cf::setup<M> B;
    B.attach = [](M & m, cp::separator sep) {
        auto [env, model] = m.native_model();
        auto & api = m.native_api();
        if(!NO_PRECRUSH) api._check(env, api.setintparam(env, "PreCrush", 1));
        keep.push_back(std::make_unique<ctx>(ctx{std::move(sep), &api, static_cast<int>(m.num_variables())}));
        api._check(env, api.setcallbackfunc(model, hook, keep.back().get()));
    };
    B.one_thread = [](M & m) { m.set_thread_limit(1); };
    B.atsp_sizes = env_list<int>("VG_ATSP", B.atsp_sizes);
    B.seeds = env_list<unsigned>("VG_SEEDS", B.seeds);
    B.stop_size = static_cast<int>(env_dbl("VG_STOP", B.stop_size));
    B.first_limit = env_dbl("VG_FIRST", B.first_limit);
    B.fuzz_seeds = static_cast<int>(env_dbl("VG_FUZZ", B.fuzz_seeds));
    B.limit = env_dbl("VG_LIMIT", B.limit);
    {
        M probe;
        int a = 0, b = 0, c = 0;
        cp::symf<void (*)(int *, int *, int *)>(probe.native_api().library_path(), "GRBversion")(&a, &b, &c);
        std::println(stderr, "== library {} version {}.{}.{}; knobs no_precrush={} no_filter={} no_fold={} no_strip={} no_empty={} keep_satisfied={}; atsp {} seeds {} stop {} first {} fuzz {} limit {}",
            probe.native_api().library_path().string(), a, b, c, NO_PRECRUSH, NO_FILTER, NO_FOLD, NO_STRIP, NO_EMPTY, KEEP_SAT, B.atsp_sizes, B.seeds, B.stop_size, B.first_limit, B.fuzz_seeds, B.limit);
    }
    const auto dump = [] {
        std::string st; for(auto [k, v] : T.status) st += std::format(" {}:{}", k, v);
        std::println(stderr, "== hook totals: status{{{} }} rel_err={} garbage_points={} root_calls={} tree_calls={} root_cuts={} tree_cuts={} native_err={} swallowed={} dropped_satisfied={} dropped_empty={}",
            st, T.rel_err, T.garbage_points, T.root_calls, T.tree_calls, T.root_cuts, T.tree_cuts, T.native_err, T.swallowed, T.dropped_satisfied, T.dropped_empty);
    };
    if(env_on("VG_POWER")) {
        for(int n : B.atsp_sizes) for(unsigned seed : B.seeds)
            cf::compare(B, std::format("C1 atsp n={} seed={}", n, seed), [=](M & m) { auto a = cp::build_atsp(m, n, false, seed); return cf::dfj<M>(a); });
        dump();
        return cf::finish();
    }
    cf::run(B);
    dump();
    // X1 covers whose terms are split in two: folded it is the cover, last-wins it is a stronger, invalid cut.
    for(unsigned seed : B.seeds) cf::compare(B, std::format("X1 split-term covers seed={}", seed), [=](M & m) {
        const std::size_t cols = 40, rows = 5;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> A(1, 100), R(0, 50);
        std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
        for(auto & r : a) for(auto & v : r) v = A(rng);
        std::vector<double> c(cols), cap(rows);
        for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / rows + R(rng)); }
        for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
        auto x = m.add_binary_variables(cols);
        const auto ids = std::views::iota(std::size_t{0}, cols);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }) + 100.0);
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
        std::vector<int> col;
        for(std::size_t j = 0; j < cols; ++j) col.push_back(m.native_id(x(j)));
        return cp::separator([a, cap, col](const std::vector<double> & p, const cp::add_fn & add) {
            for(std::size_t i = 0; i < a.size(); ++i) {
                std::vector<std::size_t> order(col.size());
                for(std::size_t j = 0; j < order.size(); ++j) order[j] = j;
                std::ranges::sort(order, [&](std::size_t l, std::size_t r) { return p[static_cast<std::size_t>(col[l])] > p[static_cast<std::size_t>(col[r])]; });
                double w = 0; cp::raw_cut cut{{}, {}, 'L', 0}, split{{}, {}, 'L', 0};
                for(std::size_t j : order) {
                    w += a[i][j]; cut.ind.push_back(col[j]); cut.val.push_back(1);
                    split.ind.push_back(col[j]); split.val.push_back(-1); split.ind.push_back(col[j]); split.val.push_back(2);
                    if(w > cap[i]) { cut.rhs = split.rhs = static_cast<double>(cut.ind.size()) - 1; if(cut.violation(p) > 1e-4) add(split); break; }
                }
            }
        });
    });
    dump();
    return cf::finish();
}
