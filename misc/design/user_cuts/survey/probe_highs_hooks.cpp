// HiGHS: does any native callback let a user reach the LP relaxation or the cut pool?
// usage: probe_highs_hooks fire|fire_comp|mutate|usersol|addrow <type> <call> <cut> <flip>|addrow_atsp <type> <call> <presolve> <cut>|lazyhack|mps <file>
#include "mippp/solvers/highs/all.hpp"
#include "cut_common.hpp"
using M = mippp::highs_milp;
namespace v1 = mippp::highs::impl::v1;
using Out = v1::HighsCallbackDataOut;
using In = v1::HighsCallbackDataIn;
using HI = v1::HighsInt;

static const char * const TN[] = {"Logging", "SimplexInterrupt", "IpmInterrupt", "MipSolution", "MipImprovingSolution",
                                  "MipLogging", "MipInterrupt", "MipGetCutPool", "MipDefineLazyConstraints", "MipUserSolution"};
enum { T_SOL = 3, T_IMPROVE = 4, T_LOG = 5, T_INT = 6, T_POOL = 7, T_LAZY = 8, T_USER = 9 };

using item_fn = const void * (*)(const Out *, const char *);
static item_fn g_item = nullptr;
template <class T>
static const T * item(const Out * o, const char * n) { return static_cast<const T *>(g_item(o, n)); }

static void one_thread(M & m) { (void)m.native_api().setIntOptionValue(m.native_model(), "threads", 1); }
static void scope(M & m) {
    auto & a = m.native_api();
    auto h = m.native_model();
    one_thread(m);
    (void)a.setStringOptionValue(h, "presolve", "off");
    (void)a.setDoubleOptionValue(h, "mip_heuristic_effort", 0.0);
    (void)a.setBoolOptionValue(h, "mip_detect_symmetry", 0);
    (void)a.setBoolOptionValue(h, "mip_allow_restart", 0);                  // later releases only
    (void)a.setBoolOptionValue(h, "mip_allow_cut_separation_at_nodes", 0);  // later releases only
}
static void * soft_sym(M & m, const char * name) {
    void * h = dlopen(m.native_api().library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    return h ? dlsym(h, name) : nullptr;
}
static void hdr(M & m) {
    std::println(stderr, "library {} version {}", m.native_api().library_path().string(), m.native_api().version());
}
static std::vector<int> start_types(M & m, void (*cb)(int, const char *, const Out *, In *, void *), void * usr, std::vector<int> types) {
    auto & a = m.native_api();
    g_item = a.getCallbackDataOutItem;
    (void)a.setCallback(m.native_model(), cb, usr);
    std::vector<int> rc;
    for(int t : types) rc.push_back(static_cast<int>(a.startCallback(m.native_model(), t)));
    return rc;
}

// ------------------------------------------------------------- fire ------------
struct fire_rec {
    long calls[12] = {};
    long with_sol[12] = {};
    long frac_sol[12] = {};
    long long first_node[12], last_node[12];
    double first_dual[12], last_dual[12];
    const cp::atsp * a = nullptr;
    int orig_cols = 0;
    int pool_col = -1, pool_cut = -1, pool_nz = -1;
    bool pool_items_ok = true;
    long pool_calls_with_cut = 0;
    std::string pool_first;
    bool mutate = false;
    fire_rec() { for(int i = 0; i < 12; ++i) { first_node[i] = last_node[i] = -7; first_dual[i] = last_dual[i] = NAN; } }
};
static void fire_cb(int t, const char *, const Out * o, In *, void * u) {
    auto & r = *static_cast<fire_rec *>(u);
    if(t < 0 || t >= 12) return;
    ++r.calls[t];
    if(t >= T_SOL) {
        const auto * n = item<std::int64_t>(o, "mip_node_count");
        const auto * d = item<double>(o, "mip_dual_bound");
        if(n) { if(r.first_node[t] == -7) r.first_node[t] = *n; r.last_node[t] = *n; }
        if(d) { if(std::isnan(r.first_dual[t])) r.first_dual[t] = *d; r.last_dual[t] = *d; }
        const auto * s = item<double>(o, "mip_solution");
        if(s && (t == T_SOL || t == T_IMPROVE || t == T_INT || t == T_POOL || t == T_USER)) {
            // HiGHS fills it for types 3 and 4; the C view gives null for an empty vector.
            // Some releases leave the previous candidate there at later calls, so check
            // integrality wherever it shows up: is any of these a relaxation point?
            ++r.with_sol[t];
            if(r.a) {
                std::vector<double> x(s, s + r.orig_cols);
                if(!r.a->integral(x)) ++r.frac_sol[t];
            }
        }
    }
    if(t == T_POOL) {
        const auto * nc = item<HI>(o, "cutpool_num_col");
        const auto * nr = item<HI>(o, "cutpool_num_cut");
        const auto * nz = item<HI>(o, "cutpool_num_nz");
        const auto * st = item<HI>(o, "cutpool_start");
        const auto * ix = item<HI>(o, "cutpool_index");
        const auto * va = item<double>(o, "cutpool_value");
        const auto * lo = item<double>(o, "cutpool_lower");
        const auto * up = item<double>(o, "cutpool_upper");
        if(!nc || !nr || !nz) { r.pool_items_ok = false; return; }
        r.pool_col = *nc; r.pool_cut = *nr; r.pool_nz = *nz;
        if(*nr > 0) ++r.pool_calls_with_cut;
        if(*nr > 0 && st && ix && va && lo && up && r.pool_first.empty()) {
            std::string s = std::format("{} <= ", lo[0]);
            for(HI k = st[0]; k < st[1] && k < st[0] + 6; ++k) s += std::format("{:+.4g} x{} ", va[k], ix[k]);
            r.pool_first = s + std::format("... <= {}", up[0]);
        }
        if(r.mutate && *nr > 0 && st && va && lo && up) {
            // the C view hands out non-const arrays: overwrite everything
            auto * L = const_cast<double *>(lo);
            auto * U = const_cast<double *>(up);
            auto * V = const_cast<double *>(va);
            for(HI i = 0; i < *nr; ++i) { L[i] = 1e6; U[i] = 1e6; }
            for(HI k = 0; k < *nz; ++k) V[k] = 1e30;
        }
    }
}
static int run_fire(bool complements, bool mutate, int n) {
    M m; hdr(m); one_thread(m);
    auto a = cp::build_atsp(m, n, complements);
    m.set_time_limit(std::chrono::duration<double>(60));
    fire_rec r; r.a = &a; r.orig_cols = static_cast<int>(m.num_variables()); r.mutate = mutate;
    auto rc = start_types(m, fire_cb, &r, {1, 2, 3, 4, 5, 6, 7, 8, 9});
    for(std::size_t i = 0; i < rc.size(); ++i) std::println(stderr, "startCallback({}={}) returned {}", i + 1, TN[i + 1], rc[i]);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== fire atsp n={} complements={} mutate={}: {} (cols in the model: {})", n, complements, mutate, cp::summary(m, t), r.orig_cols);
    for(int k = 1; k < 10; ++k)
        std::println(stderr, "   {:>26}: calls={} first_node={} last_node={} first_dual={:.6g} last_dual={:.6g} calls_with_mip_solution={} of which fractional={}",
                     TN[k], r.calls[k], r.first_node[k], r.last_node[k], r.first_dual[k], r.last_dual[k], r.with_sol[k], r.frac_sol[k]);
    std::println(stderr, "   cut pool: items_ok={} num_col={} num_cut={} num_nz={} (model num_col {}) calls_with_cuts={} first cut: {}", r.pool_items_ok, r.pool_col, r.pool_cut, r.pool_nz, r.orig_cols, r.pool_calls_with_cut, r.pool_first);
    return 0;
}

// ---------------------------------------------------------- user solution ------
struct user_rec {
    std::vector<double> sol;
    long calls = 0;
    int ret = -99;
    using set_fn = HI (*)(In *, HI, const double *);
    set_fn set = nullptr;
    bool once = false;
    bool raise_flag = false;
};
// the layout of HighsCallbackStruct.h in 1.14 and 1.15: the wrapper's struct only has the first field
struct in_layout { int user_interrupt; double * user_solution; void * cbdata; int user_has_solution; HI user_solution_size; };
static void user_cb(int t, const char *, const Out *, In * in, void * u) {
    auto & r = *static_cast<user_rec *>(u);
    if(t != T_USER) return;
    ++r.calls;
    if(!r.once && r.set) {
        r.once = true;
        r.ret = r.set(in, static_cast<HI>(r.sol.size()), r.sol.data());
        // HighsCallback.cpp:159: on return the C view's flag overwrites the one setSolution raised
        if(r.raise_flag) reinterpret_cast<in_layout *>(in)->user_has_solution = 1;
    }
}
static int run_usersol() {
    std::vector<double> opt;
    int cols = 0;
    {
        M m; hdr(m); one_thread(m);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(60));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== usersol baseline, no hook: {}", cp::summary(m, t));
        cols = static_cast<int>(m.num_variables());
        opt.resize(static_cast<std::size_t>(cols));
        (void)m.native_api().getSolution(m.native_model(), opt.data(), nullptr, nullptr, nullptr);
    }
    for(const std::string kind : {"optimum", "infeasible", "optimum+flag", "infeasible+flag"}) {
        M m; one_thread(m);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(60));
        user_rec r;
        r.sol = opt;
        if(kind.starts_with("infeasible")) std::ranges::fill(r.sol, 0.0);
        r.raise_flag = kind.ends_with("+flag");
        r.set = reinterpret_cast<user_rec::set_fn>(soft_sym(m, "Highs_setCallbackSolution"));
        auto rc = start_types(m, user_cb, &r, {T_USER});
        std::println(stderr, "   startCallback(MipUserSolution)={} Highs_setCallbackSolution={}", rc[0], r.set ? "present" : "ABSENT");
        const double t = cp::timed_solve(m);
        std::println(stderr, "== usersol {}: {} calls={} setCallbackSolution returned {}", kind, cp::summary(m, t), r.calls, r.ret);
    }
    return 0;
}

// -------------------------------------------------- addRow inside a callback ---
struct addrow_rec {
    M * m = nullptr;
    int type = 0;
    long at = 1, calls = 0;
    cp::raw_cut cut;
    int ret = -99, rows_before = -1, rows_after = -1;
    double dual_at = NAN, last_dual = NAN, last_primal = NAN;
    long long node_at = -1, last_node = -1;
    long calls_after = 0;
};
static std::pair<double, double> bounds_of(const cp::raw_cut & c) {
    constexpr double inf = 1e30;
    return c.sense == 'L' ? std::pair{-inf, c.rhs} : c.sense == 'G' ? std::pair{c.rhs, inf} : std::pair{c.rhs, c.rhs};
}
static void addrow_cb(int t, const char *, const Out * o, In *, void * u) {
    auto & r = *static_cast<addrow_rec *>(u);
    const auto * d = item<double>(o, "mip_dual_bound");
    const auto * p = item<double>(o, "mip_primal_bound");
    const auto * n = item<std::int64_t>(o, "mip_node_count");
    if(d) r.last_dual = *d;
    if(p) r.last_primal = *p;
    if(n) r.last_node = *n;
    if(t != r.type) return;
    ++r.calls;
    if(r.ret != -99) { ++r.calls_after; return; }
    if(r.calls == r.at) {
        auto & a = r.m->native_api();
        auto h = r.m->native_model();
        r.rows_before = a.getNumRow(h);
        const auto [lo, up] = bounds_of(r.cut);
        r.ret = a.addRow(h, lo, up, static_cast<HI>(r.cut.ind.size()), r.cut.ind.data(), r.cut.val.data());
        r.rows_after = a.getNumRow(h);
        r.dual_at = d ? *d : NAN;
        r.node_at = n ? *n : -1;
        std::println(stderr, "   inside {} call {}: Highs_addRow returned {} rows {} -> {} node={} dual={:.6g}", TN[t], r.calls, r.ret, r.rows_before, r.rows_after, r.node_at, r.dual_at);
    }
}
static int run_addrow(int type, long at, const std::string & kind, bool flip) {
    {
        M m; hdr(m); scope(m);
        auto g = cp::build_gadgets(m, 10, flip);
        m.set_time_limit(std::chrono::duration<double>(30));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== addrow baseline flip={}: {} known opt {}", flip, cp::summary(m, t), g.opt);
    }
    M m; scope(m);
    auto g = cp::build_gadgets(m, 10, flip);
    m.set_time_limit(std::chrono::duration<double>(30));
    if(std::getenv("PROBE_LOG")) (void)m.native_api().setBoolOptionValue(m.native_model(), "output_flag", 1);
    addrow_rec r;
    r.m = &m; r.type = type; r.at = at;
    r.cut = kind == "invalid" ? g.invalid_global_cut() : g.hull_cut(3);
    auto rc = start_types(m, addrow_cb, &r, {T_INT, T_POOL, T_USER, T_SOL, T_IMPROVE});
    std::println(stderr, "   start returned {},{},{},{},{}", rc[0], rc[1], rc[2], rc[3], rc[4]);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== addrow in {} at call {} cut={} flip={}: {} known opt {} rows after solve={} hook calls of the type={} after_add={} ret={} last hook dual={:.6g} primal={:.6g} node={}",
                 TN[type], at, kind, flip, cp::summary(m, t), g.opt, m.native_api().getNumRow(m.native_model()), r.calls, r.calls_after, r.ret, r.last_dual, r.last_primal, r.last_node);
    if(r.ret != -99) {
        std::vector<double> x(m.num_variables());
        (void)m.native_api().getSolution(m.native_model(), x.data(), nullptr, nullptr, nullptr);
        std::println(stderr, "   final point: cut activity {:.6g} rhs {:.6g} violation {:.6g}, sum z {:.6g}", r.cut.activity(x), r.cut.rhs, r.cut.violation(x), g.objective(x));
    }
    // and the next solve: is the row a plain model row now?
    m.set_time_limit(std::chrono::duration<double>(30));
    (void)m.native_api().stopCallback(m.native_model(), T_INT);
    (void)m.native_api().stopCallback(m.native_model(), T_POOL);
    (void)m.native_api().stopCallback(m.native_model(), T_USER);
    (void)m.native_api().stopCallback(m.native_model(), T_SOL);
    (void)m.native_api().stopCallback(m.native_model(), T_IMPROVE);
    r.type = -1;
    const double t2 = cp::timed_solve(m);
    std::println(stderr, "== re-solve, hook types stopped: {}", cp::summary(m, t2));
    return 0;
}

// ---------------------------------------- addRow inside a callback, on a real tree
// A tree this time: ATSP n=26. "invalid" is objective >= opt+1, which cuts off the optimum
// everywhere: in the LP it must raise the bound and lose 549. "dfj" is a batch of subtour
// rows taken from an LP loop done beforehand: in the LP it closes the gap and the tree.
static std::vector<cp::raw_cut> collect_dfj(const int n, const int max_rounds = 40) {
    std::vector<cp::raw_cut> all;
    M m; one_thread(m);
    auto a = cp::build_atsp(m, n);
    auto & api = m.native_api();
    auto h = m.native_model();
    for(int j = 0; j < static_cast<int>(m.num_variables()); ++j) (void)api.changeColIntegrality(h, j, 0);
    const auto cols = m.num_variables();
    for(int r = 0; r < max_rounds; ++r) {
        if(api.run(h) != 0) break;
        std::vector<double> x(cols);
        (void)api.getSolution(h, x.data(), nullptr, nullptr, nullptr);
        auto cuts = a.separate(x);
        if(cuts.empty()) break;
        for(auto & c : cuts) {
            const auto [lo, up] = bounds_of(c);
            (void)api.addRow(h, lo, up, static_cast<HI>(c.ind.size()), c.ind.data(), c.val.data());
            all.push_back(std::move(c));
        }
    }
    return all;
}
struct addrow_batch {
    M * m = nullptr;
    int type = 0;
    long at = 1, calls = 0;
    std::vector<cp::raw_cut> cuts;
    long added = 0;
    int ret = 0, rows_before = -1, rows_after = -1;
    long long node_at = -1, last_node = -1;
    double dual_at = NAN, last_dual = NAN;
    long dual_prints = 0;
};
static void batch_cb(int t, const char *, const Out * o, In *, void * u) {
    auto & r = *static_cast<addrow_batch *>(u);
    const auto * d = item<double>(o, "mip_dual_bound");
    const auto * n = item<std::int64_t>(o, "mip_node_count");
    const auto * q = item<double>(o, "mip_primal_bound");
    // with TRACE=1, each change of the dual bound after the add: does the LP see the row?
    if(std::getenv("TRACE") && r.rows_after >= 0 && d && *d != r.last_dual && r.dual_prints < 400) {
        ++r.dual_prints;
        std::println(stderr, "     trace node={} dual={:.8g} primal={:.8g} type={}", n ? *n : -1, *d, q ? *q : NAN, TN[t]);
    }
    if(d) r.last_dual = *d;
    if(n) r.last_node = *n;
    if(t != r.type) return;
    ++r.calls;
    if(r.calls != r.at) return;
    auto & a = r.m->native_api();
    auto h = r.m->native_model();
    r.rows_before = a.getNumRow(h);
    for(const auto & c : r.cuts) {
        const auto [lo, up] = bounds_of(c);
        const int e = a.addRow(h, lo, up, static_cast<HI>(c.ind.size()), c.ind.data(), c.val.data());
        if(e) r.ret = e; else ++r.added;
    }
    r.rows_after = a.getNumRow(h);
    r.dual_at = d ? *d : NAN;
    r.node_at = n ? *n : -1;
}
static int run_addrow_atsp(int type, long at, bool presolve, const std::string & kind) {
    const int n = 26;
    {
        M m; hdr(m); one_thread(m);
        if(!presolve) (void)m.native_api().setStringOptionValue(m.native_model(), "presolve", "off");
        auto a = cp::build_atsp(m, n);
        m.set_time_limit(std::chrono::duration<double>(60));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== addrow_atsp baseline presolve={}: {}", presolve, cp::summary(m, t));
    }
    M m; one_thread(m);
    if(!presolve) (void)m.native_api().setStringOptionValue(m.native_model(), "presolve", "off");
    auto a = cp::build_atsp(m, n);
    m.set_time_limit(std::chrono::duration<double>(60));
    if(std::getenv("PROBE_LOG")) (void)m.native_api().setBoolOptionValue(m.native_model(), "output_flag", 1);
    addrow_batch r; r.m = &m; r.type = type; r.at = at;
    if(kind == "invalid") {
        cp::raw_cut c; c.sense = 'G'; c.rhs = 550;
        for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j) {
            c.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
            c.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        }
        r.cuts.push_back(std::move(c));
    } else r.cuts = collect_dfj(n);
    if(type == 0) {  // the reference: plain model rows from the start, no hook
        for(const auto & c : r.cuts) {
            const auto [lo, up] = bounds_of(c);
            if(m.native_api().addRow(m.native_model(), lo, up, static_cast<HI>(c.ind.size()), c.ind.data(), c.val.data()) == 0) ++r.added;
        }
    } else (void)start_types(m, batch_cb, &r, {T_INT, T_POOL, T_USER, T_SOL, T_IMPROVE});
    const double t = cp::timed_solve(m);
    std::println(stderr, "== addrow_atsp in {} at call {} presolve={} cut={} ({} rows, {} added, last error {}): {} rows {} -> {} (now {}), at node={} dual={:.6g}; last hook node={} dual={:.6g}",
                 TN[type], at, presolve, kind, r.cuts.size(), r.added, r.ret, cp::summary(m, t), r.rows_before, r.rows_after, m.native_api().getNumRow(m.native_model()), r.node_at, r.dual_at, r.last_node, r.last_dual);
    std::vector<double> x(m.num_variables());
    (void)m.native_api().getSolution(m.native_model(), x.data(), nullptr, nullptr, nullptr);
    long viol = 0;
    for(const auto & c : r.cuts) viol += c.violation(x) > 1e-6;
    std::println(stderr, "   final point: objective {:.6g}, added rows violated {}", a.objective(x), viol);
    return 0;
}

// ------------------------------- subtour rows from MipSolution, degree-only ATSP
struct dist_data { std::vector<std::vector<double>> d; };
static dist_data make_dist(const int n, const unsigned seed = 5) {
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    dist_data D; D.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        D.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    return D;
}
struct lazy_rec {
    M * m = nullptr;
    int n = 0;
    std::vector<std::vector<int>> col;
    long sol_calls = 0, subtour_candidates = 0, rows_added = 0, tour_candidates = 0;
    double best_tour = 1e30;
    std::vector<std::vector<double>> d;
};
static std::vector<std::vector<int>> cycles_of(const lazy_rec & r, const double * x) {
    const auto N = static_cast<std::size_t>(r.n);
    std::vector<int> next(N, -1);
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j && x[r.col[i][j]] > 0.5) next[i] = static_cast<int>(j);
    std::vector<char> seen(N, 0);
    std::vector<std::vector<int>> out;
    for(std::size_t s = 0; s < N; ++s) if(!seen[s]) {
        std::vector<int> c;
        for(int v = static_cast<int>(s); v >= 0 && !seen[static_cast<std::size_t>(v)]; v = next[static_cast<std::size_t>(v)]) { seen[static_cast<std::size_t>(v)] = 1; c.push_back(v); }
        out.push_back(std::move(c));
    }
    return out;
}
static void lazy_cb(int t, const char *, const Out * o, In *, void * u) {
    auto & r = *static_cast<lazy_rec *>(u);
    if(t != T_SOL) return;
    ++r.sol_calls;
    const auto * x = item<double>(o, "mip_solution");
    if(!x) return;
    auto cs = cycles_of(r, x);
    if(cs.size() == 1) {
        ++r.tour_candidates;
        double len = 0;
        for(std::size_t i = 0; i < r.col.size(); ++i) for(std::size_t j = 0; j < r.col.size(); ++j) if(i != j && x[r.col[i][j]] > 0.5) len += r.d[i][j];
        r.best_tour = std::min(r.best_tour, len);
        return;
    }
    ++r.subtour_candidates;
    auto & a = r.m->native_api();
    auto h = r.m->native_model();
    for(const auto & c : cs) {
        std::vector<HI> ind; std::vector<double> val;
        for(int i : c) for(int j : c) if(i != j) { ind.push_back(r.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); val.push_back(1.0); }
        if(ind.empty()) continue;
        const auto ret = a.addRow(h, -1e30, static_cast<double>(c.size() - 1), static_cast<HI>(ind.size()), ind.data(), val.data());
        if(ret == 0) ++r.rows_added;
    }
}
static int run_lazyhack(int n, bool presolve) {
    M m; hdr(m); one_thread(m);
    if(!presolve) (void)m.native_api().setStringOptionValue(m.native_model(), "presolve", "off");
    const auto D = make_dist(n);
    const auto N = static_cast<std::size_t>(n);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(N * (N - 1));
    lazy_rec r; r.m = &m; r.n = n; r.d = D.d;
    r.col.assign(N, std::vector<int>(N, -1));
    const auto nodes = std::views::iota(std::size_t{0}, N);
    for(std::size_t i : nodes) for(std::size_t j : nodes) if(i != j) r.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return D.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    m.set_time_limit(std::chrono::duration<double>(60));
    auto rc = start_types(m, lazy_cb, &r, {T_SOL});
    const double t = cp::timed_solve(m);
    std::vector<double> sol(m.num_variables());
    (void)m.native_api().getSolution(m.native_model(), sol.data(), nullptr, nullptr, nullptr);
    const auto cs = cycles_of(r, sol.data());
    std::println(stderr, "== lazyhack presolve={}: rows from MipSolution, degree-only ATSP n={}: {} rows now={} solution_calls={} subtour_candidates={} tour_candidates={} rows_added={} final point has {} cycle(s) best tour seen in hook={}",
                 presolve, n, cp::summary(m, t), m.native_api().getNumRow(m.native_model()), r.sol_calls, r.subtour_candidates, r.tour_candidates, r.rows_added, cs.size(), r.best_tour);
    return 0;
}

// ------------- Is a row added from a callback a sound lazy constraint? Every subtour row
// of a small degree-only ATSP, added at the first MipInterrupt: the correct answer is known
// from the same rows present from the start.
struct allrows_rec {
    M * m = nullptr;
    std::vector<cp::raw_cut> rows;
    bool done = false;
    int err = 0;
};
static void allrows_cb(int t, const char *, const Out *, In *, void * u) {
    auto & r = *static_cast<allrows_rec *>(u);
    if(t != T_INT || r.done) return;
    r.done = true;
    for(const auto & c : r.rows) {
        const auto [lo, up] = bounds_of(c);
        const int e = r.m->native_api().addRow(r.m->native_model(), lo, up, static_cast<HI>(c.ind.size()), c.ind.data(), c.val.data());
        if(e) r.err = e;
    }
}
static int run_allrows(int n, unsigned seed, bool from_callback, bool presolve) {
    M m; one_thread(m);
    if(!presolve) (void)m.native_api().setStringOptionValue(m.native_model(), "presolve", "off");
    const auto D = make_dist(n, seed);
    const auto N = static_cast<std::size_t>(n);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(N * (N - 1));
    lazy_rec lr; lr.n = n; lr.d = D.d;
    lr.col.assign(N, std::vector<int>(N, -1));
    const auto nodes = std::views::iota(std::size_t{0}, N);
    for(std::size_t i : nodes) for(std::size_t j : nodes) if(i != j) lr.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return D.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    m.set_time_limit(std::chrono::duration<double>(30));
    allrows_rec r; r.m = &m;
    for(std::uint32_t mask = 1; mask < (1u << (n - 1)); ++mask) {
        std::vector<int> S;
        for(int b = 0; b < n - 1; ++b) if(mask >> b & 1) S.push_back(b + 1);
        if(S.size() < 2) continue;
        cp::raw_cut c; c.sense = 'L'; c.rhs = static_cast<double>(S.size() - 1);
        for(int i : S) for(int j : S) if(i != j) { c.ind.push_back(lr.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); c.val.push_back(1.0); }
        r.rows.push_back(std::move(c));
    }
    if(from_callback) (void)start_types(m, allrows_cb, &r, {T_INT});
    else for(const auto & c : r.rows) { const auto [lo, up] = bounds_of(c); (void)m.native_api().addRow(m.native_model(), lo, up, static_cast<HI>(c.ind.size()), c.ind.data(), c.val.data()); }
    const double t = cp::timed_solve(m);
    std::vector<double> sol(m.num_variables());
    (void)m.native_api().getSolution(m.native_model(), sol.data(), nullptr, nullptr, nullptr);
    const auto cs = cycles_of(lr, sol.data());
    std::println(stderr, "== allrows n={} seed={} presolve={} {}: {} rows={} final point cycles={}", n, seed, presolve, from_callback ? "CALLBACK" : "from start", cp::summary(m, t), r.rows.size(), cs.size());
    return 0;
}

// --------------------------------------------------------- USERCUTS in an MPS ---
static int run_mps(const char * file) {
    M m; hdr(m);
    auto & a = m.native_api();
    auto h = m.native_model();
    (void)a.setBoolOptionValue(h, "output_flag", 1);
    auto read = cp::symf<HI (*)(void *, const char *)>(a.library_path(), "Highs_readModel");
    const HI ret = read(h, file);
    std::println(stderr, "== Highs_readModel({}) returned {} rows={} cols={}", file, ret, a.getNumRow(h), a.getNumCol(h));
    return 0;
}

int main(int argc, char ** argv) {
    const std::string what = argc > 1 ? argv[1] : "fire";
    if(what == "fire") return run_fire(false, false, argc > 2 ? std::atoi(argv[2]) : 26);
    if(what == "fire_comp") return run_fire(true, false, 26);
    if(what == "mutate") return run_fire(false, true, argc > 2 ? std::atoi(argv[2]) : 26);
    if(what == "usersol") return run_usersol();
    if(what == "addrow") return run_addrow(std::atoi(argv[2]), std::atol(argv[3]), argv[4], std::atoi(argv[5]) != 0);
    if(what == "addrow_atsp") return run_addrow_atsp(std::atoi(argv[2]), std::atol(argv[3]), std::atoi(argv[4]) != 0, argv[5]);
    if(what == "lazyhack") return run_lazyhack(argc > 2 ? std::atoi(argv[2]) : 26, argc > 3 ? std::atoi(argv[3]) != 0 : true);
    if(what == "allrows") return run_allrows(std::atoi(argv[2]), static_cast<unsigned>(std::atoi(argv[3])), std::atoi(argv[4]) != 0, argc > 5 ? std::atoi(argv[5]) != 0 : true);
    if(what == "mps") return run_mps(argv[2]);
    return 2;
}
