// C5 (root spin on a satisfied cut), C6 (1121 and Reformulations), C7 (duplicate columns), C10 (FILTER, RepeatPresolve).
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
struct spinctx {
    const vsyms * s; const cx::cplex_api * api; int cols; int purgeable; bool sanitize; raw_cut cut;
    std::atomic<long> calls{0}, depth0{0}, added{0}, skipped{0};
};
static std::vector<std::unique_ptr<spinctx>> keep_spin;
static int spinhook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<spinctx *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        cxlong d = 0; h.s->infolong(context, CPXCALLBACKINFO_NODEDEPTH, &d);
        ++h.calls; if(d == 0) ++h.depth0;
        if(h.sanitize && h.cut.violation(x) <= 1e-9) { ++h.skipped; return 0; }
        int beg = 0, lo = 0;
        const int e = h.api->callbackaddusercuts(context, 1, static_cast<int>(h.cut.ind.size()), &h.cut.rhs, &h.cut.sense, &beg, h.cut.ind.data(), h.cut.val.data(), &h.purgeable, &lo);
        if(!e) ++h.added;
    } catch(...) { return 1; }
    return 0;
}
static void c5() {
    const std::pair<const char *, int> modes[] = {{"PURGE", CPX_USECUT_PURGE}, {"FORCE", CPX_USECUT_FORCE}, {"FILTER", CPX_USECUT_FILTER}};
    for(int kind = 0; kind < 3; ++kind) for(const auto & [mn, mp] : modes) for(const bool san : {false, true}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); m.set_time_limit(duration<double>(5.0));
        keep_spin.push_back(std::make_unique<spinctx>());
        auto & h = *keep_spin.back();
        h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = mp; h.sanitize = san;
        const char * kname;
        if(kind == 0) { kname = "one-term x<=1 (slack at most points)"; h.cut = raw_cut{{a.col[0][1]}, {1.0}, 'L', 1.0}; }
        else if(kind == 1) {
            kname = "degree row, tight at every point";
            raw_cut c; c.sense = 'L'; c.rhs = 1.0;
            for(int j = 1; j < 26; ++j) { c.ind.push_back(a.col[0][static_cast<std::size_t>(j)]); c.val.push_back(1.0); }
            h.cut = c;
        } else {
            kname = "dense, far from tight";
            raw_cut c; c.sense = 'L'; c.rhs = 1e6;
            for(int i = 0; i < 26; ++i) for(int j = 0; j < 26; ++j) if(i != j) { c.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]); c.val.push_back(1.0); }
            h.cut = c;
        }
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, spinhook, &h));
        const double t = timed_solve(m);
        std::println(stderr, "C5 [{}] {} sanitize={}: {} | calls={} depth0={} added={} skipped={}", kname, mn, san, summary(m, t), h.calls.load(), h.depth0.load(), h.added.load(), h.skipped.load());
    }
    // same at tree nodes only: every family off, so no root call
    for(const auto & [mn, mp] : modes) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 22, false, 1); all_families_off(m); m.set_time_limit(duration<double>(5.0));
        keep_spin.push_back(std::make_unique<spinctx>());
        auto & h = *keep_spin.back();
        h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = mp; h.sanitize = false;
        h.cut = raw_cut{{a.col[0][1]}, {1.0}, 'L', 1.0};
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, spinhook, &h));
        const double t = timed_solve(m);
        std::println(stderr, "C5 tree only {} no sanitize: {} | calls={} depth0={} added={}", mn, summary(m, t), h.calls.load(), h.depth0.load(), h.added.load());
    }
    // CUTPASS caps the spin
    for(const int cp : {1, 5}) {
        M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); vparam(m, CPX_PARAM_CUTPASS, cp); m.set_time_limit(duration<double>(5.0));
        keep_spin.push_back(std::make_unique<spinctx>());
        auto & h = *keep_spin.back();
        h.s = &vsyms::get(m); h.api = &m.native_api(); h.cols = static_cast<int>(m.num_variables()); h.purgeable = CPX_USECUT_PURGE; h.sanitize = false;
        h.cut = raw_cut{{a.col[0][1]}, {1.0}, 'L', 1.0};
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, spinhook, &h));
        const double t = timed_solve(m);
        std::println(stderr, "C5 CUTPASS={} PURGE no sanitize: {} | calls={} depth0={}", cp, summary(m, t), h.calls.load(), h.depth0.load());
    }
}
// a x + b y: parallel columns, each arc is a + b. kind 0 = cut on a+b, kind 1 = 2a + b (valid, weaker), kind 2 = a only on single arcs
struct par { separator sep; };
static separator build_parallel(M & m, const int n, const unsigned seed, const int kind) {
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100);
    std::vector<std::vector<double>> d(N, std::vector<double>(N));
    for(auto & r : d) for(auto & v : r) v = std::round(P(rng));
    auto a = m.add_binary_variables(N * N);
    auto b = m.add_binary_variables(N * N);
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
    return separator([n, N, ca, cb, kind](const std::vector<double> & x, const add_fn & add) {
        atsp s; s.n = n; s.col.assign(N, std::vector<int>(N, -1));
        std::vector<double> sum; int next = 0;
        for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) { s.col[i][j] = next++; sum.push_back(x[static_cast<std::size_t>(ca[i][j])] + x[static_cast<std::size_t>(cb[i][j])]); }
        for(const auto & c : s.separate(sum)) {
            raw_cut out; out.sense = c.sense; out.rhs = c.rhs;
            for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
                if(std::ranges::find(c.ind, s.col[i][j]) != c.ind.end()) {
                    out.ind.push_back(ca[i][j]); out.val.push_back(kind == 1 ? 2.0 : 1.0);
                    out.ind.push_back(cb[i][j]); out.val.push_back(1.0);
                }
            add(out);
        }
    });
}
static void c6() {
    for(const int n : {16, 22}) for(const unsigned seed : {1u, 2u, 3u}) for(const int kind : {0, 1}) {
        double base = NAN; long bnodes = 0;
        { M m; m.set_thread_limit(1); build_parallel(m, n, seed, kind); m.set_time_limit(duration<double>(30)); timed_solve(m); base = cf::value_of(m); bnodes = static_cast<long>(m.get_node_count()); }
        for(const auto & [name, setter, mask] : std::initializer_list<std::tuple<const char *, std::function<void(M &)>, long>>{
                {"Reform=3(default)", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 3); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=2", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 2); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=1", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 1); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=0", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 0); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"PRELINEAR=0", [](M & m) { vparam(m, CPX_PARAM_PRELINEAR, 0); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=3 mask C|R", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 3); }, CPX_CALLBACKCONTEXT_RELAXATION | CPX_CALLBACKCONTEXT_CANDIDATE}}) {
            M m; m.set_thread_limit(1); auto sep = build_parallel(m, n, seed, kind); setter(m); m.set_time_limit(duration<double>(30));
            recipe r; auto & h = vattach(m, sep, r, mask);
            const double t = timed_solve(m);
            std::println(stderr, "C6 n={} seed={} cut={} {}: base {} ({} nodes) | {} | {}", n, seed, kind == 0 ? "a+b" : "2a+b", name, base, bnodes, summary(m, t), h.line());
        }
    }
}

// Duplicated general-integer columns: x1[j], x2[j] same everywhere. CG cut on the sum, then one of the two
// coefficients lowered by one (x >= 0, so still valid): the two columns no longer share a coefficient.
static separator build_dup_int(M & m, const int cols, const int rows, const unsigned seed, const int kind) {
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
    return separator([a, b, c1, c2, kind, gen, mx](const std::vector<double> & p, const add_fn & add) {
        std::lock_guard l(*mx);
        const int ds[] = {2, 3, 4, 5, 7, 11, 13, 17, 19, 23};
        for(int attempt = 0, added = 0; attempt < 200 && added < 3; ++attempt) {
            const std::size_t i = (*gen)() % a.size();
            const double d = ds[(*gen)() % std::size(ds)];
            raw_cut cut{{}, {}, 'L', std::floor(b[i] / d)};
            for(std::size_t j = 0; j < c1.size(); ++j) {
                const double f = std::floor(a[i][j] / d);
                if(f == 0) continue;
                const double v1 = p[static_cast<std::size_t>(c1[j])], v2 = p[static_cast<std::size_t>(c2[j])];
                double f1 = f, f2 = f;
                if(kind == 1) { if(v1 <= v2) f1 = std::max(0.0, f - 1); else f2 = std::max(0.0, f - 1); }
                if(f1 != 0) { cut.ind.push_back(c1[j]); cut.val.push_back(f1); }
                if(f2 != 0) { cut.ind.push_back(c2[j]); cut.val.push_back(f2); }
            }
            if(cut.violation(p) > 1e-4) { add(cut); ++added; }
        }
    });
}
static void c6b() {
    for(const unsigned seed : {1u, 2u, 3u}) for(const int kind : {0, 1}) {
        double base = NAN; long bnodes = 0;
        { M m; m.set_thread_limit(1); build_dup_int(m, 25, 6, seed, kind); m.set_time_limit(duration<double>(20)); const double t = timed_solve(m); base = cf::value_of(m); bnodes = static_cast<long>(m.get_node_count()); std::println(stderr, "base {}", summary(m, t)); }
        for(const auto & [name, setter, mask] : std::initializer_list<std::tuple<const char *, std::function<void(M &)>, long>>{
                {"Reform=3(default)", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 3); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=2", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 2); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=1", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 1); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=0", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 0); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"PRELINEAR=0", [](M & m) { vparam(m, CPX_PARAM_PRELINEAR, 0); }, CPX_CALLBACKCONTEXT_RELAXATION},
                {"Reform=3 mask C|R", [](M & m) { vparam(m, CPXPARAM_Preprocessing_Reformulations, 3); }, CPX_CALLBACKCONTEXT_RELAXATION | CPX_CALLBACKCONTEXT_CANDIDATE}}) {
            M m; m.set_thread_limit(1); auto sep = build_dup_int(m, 25, 6, seed, kind); setter(m); m.set_time_limit(duration<double>(20));
            recipe r; auto & h = vattach(m, sep, r, mask);
            std::println("#### seed={} kind={} {}", seed, kind, name); std::fflush(stdout);
            vparam(m, CPX_PARAM_SCRIND, 1);
            const double t = timed_solve(m);
            std::fflush(stdout);
            std::println(stderr, "C6b seed={} cut={} {}: base {} ({} nodes) | {} | {}", seed, kind == 0 ? "same coef" : "coef differ", name, base, bnodes, summary(m, t), h.line());
        }
    }
}
static int reduced_columns_log(M & m) { (void)m; return 0; }
// C7: gadget 1's za is the probe. natural bound of za1 is the point: cut written so that it means za1 <= B.
static void c7() {
    for(const int dc : {0, 1, 2}) {
        M m; scope_settings(m); set_threads(m, 1);
        auto g = build_gadgets(m, 10, false);
        m.set_time_limit(duration<double>(10));
        std::println(stderr, "C7 baseline opt {} DATACHECK {}", g.opt, dc);
        (void)dc;
    }
    struct cs { const char * name; std::vector<double> vals; double rhs; };
    const std::vector<cs> cases = {
        {"0.8,0.2 <=1 (written 1.0 z <=1)", {0.8, 0.2}, 1.0},
        {"0.2,0.8 <=1 (written 1.0 z <=1)", {0.2, 0.8}, 1.0},
        {"0.5,0.25,0.25 <=1 (written 1.0 z <=1)", {0.5, 0.25, 0.25}, 1.0},
        {"-0.5,1.0 <=1 (written 0.5 z <=1, z<=2)", {-0.5, 1.0}, 1.0},
        {"1.0,-0.5 <=1 (written 0.5 z <=1, z<=2)", {1.0, -0.5}, 1.0},
        {"1,-1 <=1 (written nothing)", {1.0, -1.0}, 1.0},
        {"-1,1 <=1 (written nothing)", {-1.0, 1.0}, 1.0},
    };
    for(const int dc : {0, 2}) for(const auto & cc : cases) {
        M m; scope_settings(m); set_threads(m, 1); vparam(m, CPXPARAM_Read_DataCheck, dc);
        auto g = build_gadgets(m, 10, false);
        m.set_time_limit(duration<double>(10));
        const int z = g.za[1];
        auto once = std::make_shared<std::atomic<bool>>(false);
        auto zmax = std::make_shared<std::atomic<double>>(-1e30);
        auto zmaxpre = std::make_shared<std::atomic<double>>(-1e30);
        recipe r; r.fold = false; r.drop_satisfied = false;
        const auto & hh = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
            const double v = x[static_cast<std::size_t>(z)];
            if(!once->exchange(true)) {
                zmaxpre->store(v);
                raw_cut c; c.sense = 'L'; c.rhs = cc.rhs;
                for(double w : cc.vals) { c.ind.push_back(z); c.val.push_back(w); }
                add(c);
            } else { double cur = zmax->load(); while(v > cur && !zmax->compare_exchange_weak(cur, v)) {} }
        }, r);
        const double t = timed_solve(m);
        std::println(stderr, "C7 DATACHECK={} {}: za1 before={:.4f} max later za1={:.4f} | {} known_opt {} | {}", dc, cc.name, zmaxpre->load(), zmax->load(), summary(m, t), g.opt, hh.line());
    }
}
int main(int, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argv[1] ? argv[1] : "";
    if(what == "c5") c5();
    if(what == "c6") c6();
    if(what == "c7") c7();
    if(what == "c6b") c6b();
    return 0;
}
