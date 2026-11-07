// Experiments cut_common does not run on Xpress. argv[1] selects one, see main.
#include "probe_xpress_hook.hpp"
using namespace std::chrono_literals;
static constexpr auto secs = [](double s) { return std::chrono::duration<double>(s); };

// ---- P: harness P is skipped here (no set_constraint_rhs on xpress_milp); same experiment.
static void persist() {
    for(const std::string mode : {"addcuts", "managed_global", "managed_local"}) for(int presolve_default : {0, 1}) {
        M m;
        scope_settings(m);
        if(presolve_default) { set_int(m, C_PRESOLVE, 1); set_int(m, C_MIPRESTART, -1); }
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(secs(30));
        auto s = std::make_shared<cp::stats>();
        const cp::raw_cut cut = g.hull_cut(3);
        bool once = false;
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(!once && cut.violation(x) > 1e-4) { once = true; add(cut); } }, s), mode);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== P mode {} presolve_on={} first solve: {} (opt {})\n   {}", mode, presolve_default, cp::summary(m, t), g.opt, s->line());
        auto & api = m.native_api();
        for(std::size_t r = 12; r < 16; ++r) {
            int id = m.native_id(g.rows[r]);
            double rhs = 0;
            api._check(m.native_model(), api.getrhs(m.native_model(), &rhs, id, id));
            m.set_constraint_upper_bound(g.rows[r], rhs + 50);
        }
        const double t2 = cp::timed_solve(m);
        const double v = m.get_solution_value();
        std::println(stderr, "== P re-solve, rhs raised so that the cut is invalid: {} (new opt {}) => {}", cp::summary(m, t2), g.opt + 100,
                     std::fabs(v - (g.opt + 100)) < 1e-4 ? "cut did NOT persist" : "cut PERSISTED (wrong optimum)");
        flush_last();
    }
}

// ---- Q3/Q5: what presolverow says, and what a managed cut silently does, on reduced columns.
// An ATSP block keeps the search alive; a small block adds columns presolve removes:
//   dup: t,s binary, identical columns, row t + s <= 1, both worth 10: duplicate columns.
//   dual: w continuous in [0,1], cost +3 and in no row: dual reduction fixes it at 0.
//   slack: v continuous in [0,5], in the single row v + t' == 1 with t' binary: singleton column.
struct block { int t, s, w, v, tp; };
static block add_block(M & m) {
    auto b = m.add_binary_variables(3);
    auto c = m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
    block k{m.native_id(b(std::size_t{0})), m.native_id(b(std::size_t{1})), m.native_id(c(std::size_t{0})), m.native_id(c(std::size_t{1})), m.native_id(b(std::size_t{2}))};
    m.add_constraint(1.0 * b(std::size_t{0}) + 1.0 * b(std::size_t{1}) <= 1);
    m.add_constraint(1.0 * c(std::size_t{1}) + 1.0 * b(std::size_t{2}) == 1);
    m.set_objective_coefficient(b(std::size_t{0}), -10);
    m.set_objective_coefficient(b(std::size_t{1}), -10);
    m.set_objective_coefficient(c(std::size_t{0}), 3);
    m.set_objective_coefficient(c(std::size_t{1}), 0);
    m.set_objective_coefficient(b(std::size_t{2}), 1);
    return k;
}
static void presolverow_cases() {
    const int n = 14;
    struct kase { std::string name; std::function<cp::raw_cut(const block &)> make; double dv; };
    const std::vector<kase> cases = {
        {"dominance: t <= 0 on a duplicate column (s=1 is as good, so no optimum is lost)", [](const block & k) { return cp::raw_cut{{k.t}, {1.0}, 'L', 0.0}; }, 0.0},
        {"dual: w >= 0.5 on a column dual reductions fix at 0 (value goes up by 1.5 if enforced)", [](const block & k) { return cp::raw_cut{{k.w}, {1.0}, 'G', 0.5}; }, 1.5},
        {"equality t == 0 on a duplicate column (presolverow with E segfaults here at default dual reductions)", [](const block & k) { return cp::raw_cut{{k.t}, {1.0}, 'E', 0.0}; }, 0.0},
        {"slack: v <= 0.5 on a singleton column (forces t'=1, value up by 1 if enforced)", [](const block & k) { return cp::raw_cut{{k.v}, {1.0}, 'L', 0.5}; }, 1.0},
    };
    double base;
    { M m; one_thread(m); cp::build_atsp(m, n); add_block(m); m.set_time_limit(secs(30)); cp::timed_solve(m); base = m.get_solution_value(); }
    std::println(stderr, "baseline with the block: value={}", base);
    const char * only_case = std::getenv("XP_ONLY_CASE");
    const char * only_mode = std::getenv("XP_ONLY_MODE");
    for(const auto & c : cases) for(const std::string mode : {"addcuts", "managed_global", "managed_local"}) for(int dr : {1, 0}) for(int rs : {-1, 0}) {
        if(only_case && !c.name.starts_with(only_case)) continue;
        if(only_mode && mode != only_mode) continue;
        M m; one_thread(m);
        set_int(m, C_MIPDUALREDUCTIONS, dr);
        set_int(m, C_MIPRESTART, rs);
        auto a = cp::build_atsp(m, n);
        const block k = add_block(m);
        m.set_time_limit(secs(30));
        auto s = std::make_shared<cp::stats>();
        const cp::raw_cut cut = c.make(k);
        bool once = false; int err = 0; double viol_at_add = 0;
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(!once) { once = true; viol_at_add = cut.violation(x); err = add(cut); } }, s), mode);
        const double t = cp::timed_solve(m);
        std::string finals = "no solution";
        try { auto sol = m.get_solution(); const auto V = [&](int j) { return sol[mippp::model_variable<int, double>(j)]; }; finals = std::format("t={} s={} w={} v={} t'={}", V(k.t), V(k.s), V(k.w), V(k.v), V(k.tp)); } catch(const std::exception &) {}
        std::println(stderr, "== {}\n   mode {} MIPDUALREDUCTIONS={} MIPRESTART={}: {} base {} expected base+{} viol_at_add={} add_return={} finals {}", c.name, mode, dr, rs, cp::summary(m, t), base, c.dv, viol_at_add, err, finals);
        flush_last();
    }
}

// ---- Q5: naive XPRSaddcuts with a small shift, so that every index is in range and nothing errors.
static void naive_shift() {
    for(const int n : {12, 16, 20, 26}) for(const int K : {1, 2, 3}) {
        double base;
        { M m; one_thread(m); cp::build_atsp(m, n); m.set_time_limit(secs(30)); cp::timed_solve(m); base = m.get_solution_value(); }
        for(const std::string mode : {"naive", "addcuts", "managed_global"}) {
            M m; one_thread(m);
            m.add_variables(static_cast<std::size_t>(K), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 0});
            auto a = cp::build_atsp(m, n);
            m.set_time_limit(secs(10));
            auto s = std::make_shared<cp::stats>();
            attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
                [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), mode);
            const double t = cp::timed_solve(m);
            double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
            std::println(stderr, "== n={} K={} mode {}: {} base {} => {}\n   {}", n, K, mode, cp::summary(m, t), base, std::fabs(v - base) < 1e-6 ? "value right" : "VALUE WRONG", s->line());
            flush_last();
        }
    }
}

// ---- Q5: what XPRSpresolverow returns for each sense on each kind of reduced column.
struct psctx { const x::xpress_api * api; std::shared_ptr<xp_fns> fn; block k; bool done = false; int dr; int only_col = -1, only_sense = -1; };
static void ps_opt(prob_t cb, void * d, int *) {
    auto & c = *static_cast<psctx *>(d);
    if(c.done) return;
    c.done = true;
    int cols = 0; c.api->getintattrib(cb, A_COLS, &cols);
    std::vector<int> pi(static_cast<std::size_t>(cols)); std::vector<double> pv(static_cast<std::size_t>(cols));
    const std::vector<std::pair<std::string, int>> columns = {{"t(duplicate)", c.k.t}, {"s(duplicate)", c.k.s}, {"w(dual fix)", c.k.w}, {"v(singleton)", c.k.v}, {"t'(in the singleton's row)", c.k.tp}};
    int ci = -1;
    for(const auto & [name, col] : columns) { ++ci; if(c.only_col >= 0 && ci != c.only_col) continue; for(const char sense : {'L', 'G', 'E'}) {
        if(c.only_sense >= 0 && "LGE"[c.only_sense] != sense) continue;
        const double val = 1, rhs = 0.5;
        int n = -1, status = -99; double prhs = 0;
        const int e = c.fn->presolverow(cb, sense, 1, &col, &val, rhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
        std::string row; for(int q = 0; q < std::min(n, 4); ++q) row += std::format(" {}*c{}", pv[static_cast<std::size_t>(q)], pi[static_cast<std::size_t>(q)]);
        std::println(stderr, "   MIPDUALREDUCTIONS={} {} {} 0.5: err={} status={} -> {} terms{} {} {}", c.dr, name, sense, e, status, n, row, sense, prhs);
    } }
}
static void pstatus(int argc, char ** argv) {
    // a crash in the native call takes the process, so the driver runs one combination per process
    const int only_dr = argc > 2 ? std::atoi(argv[2]) : -1, only_col = argc > 3 ? std::atoi(argv[3]) : -1, only_sense = argc > 4 ? std::atoi(argv[4]) : -1;
    for(int dr : {1, 0}) {
        if(only_dr >= 0 && dr != only_dr) continue;
        M m; one_thread(m);
        set_int(m, C_MIPDUALREDUCTIONS, dr);
        cp::build_atsp(m, 14);
        psctx c; c.k = add_block(m); c.api = &m.native_api(); c.fn = fns_of(m); c.dr = dr; c.only_col = only_col; c.only_sense = only_sense;
        m.set_time_limit(secs(20));
        auto & api = m.native_api();
        api._check(m.native_model(), api.addcboptnode(m.native_model(), ps_opt, &c, 0));
        cp::timed_solve(m);
    }
}
// ---- Q5: can naive XPRSaddcuts end in status optimal at a wrong value, with nothing reported?
static void naive_wrong() {
    int wrong = 0, runs = 0;
    for(unsigned seed = 1; seed <= 12; ++seed) for(const int n : {12, 14, 16, 18, 20}) for(const int K : {1, 3, 7}) {
        double base;
        { M m; one_thread(m); cp::build_atsp(m, n, false, seed); m.set_time_limit(secs(8)); cp::timed_solve(m); base = m.get_solution_value(); }
        M m; one_thread(m);
        m.add_variables(static_cast<std::size_t>(K), {.obj_coef = 0, .lower_bound = 0, .upper_bound = 0});
        auto a = cp::build_atsp(m, n, false, seed);
        m.set_time_limit(secs(3));
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s), "naive");
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        ++runs;
        const bool bad = !(std::fabs(v - base) < 1e-6);
        if(bad) { ++wrong; std::println(stderr, "== seed={} n={} K={} naive: {} base {} => VALUE WRONG cuts={} add_errors={}", seed, n, K, cp::summary(m, t), base, s->cuts.load(), s->add_errors.load()); }
        keep.clear();
    }
    std::println(stderr, "naive_wrong: {} wrong of {} runs", wrong, runs);
}
// ---- CUTSTRATEGY=0 and managed note 3.
static void strategy0() {
    for(const std::string mode : {"managed_global", "managed_local", "addcuts", "cutround_addcuts"}) for(const int cs : {-1, 0}) {
        M m; one_thread(m);
        set_int(m, C_CUTSTRATEGY, cs);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(secs(30));
        auto s = std::make_shared<cp::stats>();
        attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }, s,
            [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), mode);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== CUTSTRATEGY={} mode {}: {} \n   {}", cs, mode, cp::summary(m, t), s->line());
        flush_last();
    }
}

// ---- Q3: removing non-optimal integer points, and a cut that removes every optimum.
static void objective_cuts() {
    const int n = 26;
    double base;
    std::vector<int> cols;
    { M m; one_thread(m); cp::build_atsp(m, n); m.set_time_limit(secs(30)); cp::timed_solve(m); base = m.get_solution_value(); }
    for(const std::string mode : {"addcuts", "managed_global"}) for(const double rhs : {base + 5, base, base - 1}) {
        M m; one_thread(m);
        auto a = cp::build_atsp(m, n);
        m.set_time_limit(secs(30));
        cp::raw_cut cut; cut.sense = 'L'; cut.rhs = rhs;
        for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j) { cut.ind.push_back(a.col[i][j]); cut.val.push_back(a.d[i][j]); }
        auto s = std::make_shared<cp::stats>();
        bool once = false;
        attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s), mode);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== objective cut d.x <= {} (optimum {}) mode {}: {} \n   {}", rhs, base, mode, cp::summary(m, t), s->line());
        flush_last();
    }
}

// ---- Q8: can two calls overlap? On a search that lasts, per hook, per mutex control.
struct octx { std::atomic<int> in{0}, peak{0}; std::atomic<long> calls{0}, over{0}; std::mutex m; std::set<std::thread::id> th; std::set<prob_t> probs; std::set<int> ids; long spin_us; const x::xpress_api * api; };
static void o_common(octx & c, prob_t cb) {
    const int now = c.in.fetch_add(1) + 1;
    if(now > 1) c.over.fetch_add(1);
    int pk = c.peak.load(); while(now > pk && !c.peak.compare_exchange_weak(pk, now)) {}
    c.calls.fetch_add(1);
    int id = 0; c.api->getintattrib(cb, A_MIPTHREADID, &id);
    { std::lock_guard l(c.m); c.th.insert(std::this_thread::get_id()); c.probs.insert(cb); c.ids.insert(id); }
    const auto until = clk::now() + std::chrono::microseconds(c.spin_us);
    while(clk::now() < until) {}
    c.in.fetch_sub(1);
}
static void o_optnode(prob_t cb, void * d, int *) { o_common(*static_cast<octx *>(d), cb); }
static void o_cutround(prob_t cb, void * d, int, int *) { o_common(*static_cast<octx *>(d), cb); }
static void o_preintsol(prob_t cb, void * d, int, int *, double *) { o_common(*static_cast<octx *>(d), cb); }
static void overlap() {
    struct cfg { std::string hook; int mutex; int main_thread; };
    for(const auto & f : std::vector<cfg>{{"optnode", 1, 0}, {"optnode", 0, 0}, {"optnode", 1, 1}, {"cutround", 1, 0}, {"cutround", 0, 0}, {"cutround", 1, 1}, {"preintsol", 1, 0}, {"preintsol", 0, 0}}) {
        M m;

        cp::build_mkp(m, 30, 100);
        m.set_time_limit(secs(8));
        set_int(m, C_MUTEXCALLBACKS, f.mutex);
        set_int(m, C_CALLBACKFROMMAINTHREAD, f.main_thread);
        octx c; c.spin_us = 200; c.api = &m.native_api();
        auto fns = fns_of(m);
        auto & api = m.native_api();
        if(f.hook == "optnode") api._check(m.native_model(), api.addcboptnode(m.native_model(), o_optnode, &c, 0));
        else if(f.hook == "cutround") api._check(m.native_model(), fns->addcbcutround(m.native_model(), o_cutround, &c, 0));
        else api._check(m.native_model(), api.addcbpreintsol(m.native_model(), o_preintsol, &c, 0));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== {} MUTEXCALLBACKS={} CALLBACKFROMMAINTHREAD={}: {} calls={} overlaps={} peak_inflight={} os_threads={} cbprobs={} worker_ids={}", f.hook, f.mutex, f.main_thread, cp::summary(m, t), c.calls.load(), c.over.load(), c.peak.load(), c.th.size(), c.probs.size(), c.ids.size());
    }
}

static void overlap_preintsol();
// ---- Q6: does a no-op hook switch the component solves off? (release note 37.01.01 names optnode)
static void c_opt(prob_t, void *, int *) {}
static void c_cut(prob_t, void *, int, int *) {}
static void c_pre(prob_t, void *, int, int *, double *) {}
static void components() {
    // four independent 5x40 knapsacks: components presolve can split, each one needing a search
    for(const std::string hook : {"none", "optnode", "cutround", "preintsol"}) for(const int prec : {-1, 1}) {
        M m; one_thread(m);
        set_int(m, C_PRECOMPONENTS, prec);
        std::mt19937 rng(11);
        std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
        m.set_maximization();
        for(int block = 0; block < 4; ++block) {
            auto x = m.add_binary_variables(40);
            const auto ids = std::views::iota(std::size_t{0}, std::size_t{40});
            std::vector<std::vector<double>> a(5, std::vector<double>(40));
            for(auto & row : a) for(auto & v : row) v = A(rng);
            for(std::size_t j : ids) {
                double sum = 0; for(const auto & row : a) sum += row[j];
                m.set_objective_coefficient(x(j), std::floor(sum / 5 + R(rng)));
            }
            for(const auto & row : a) {
                double sum = 0; for(double v : row) sum += v;
                m.add_constraint(xsum(ids, [&row, &x](std::size_t j) { return row[j] * x(j); }) <= std::floor(sum / 2));
            }
        }
        m.set_time_limit(secs(20));
        auto & api = m.native_api();
        if(hook == "optnode") api._check(m.native_model(), api.addcboptnode(m.native_model(), c_opt, nullptr, 0));
        if(hook == "cutround") api._check(m.native_model(), fns_of(m)->addcbcutround(m.native_model(), c_cut, nullptr, 0));
        if(hook == "preintsol") api._check(m.native_model(), api.addcbpreintsol(m.native_model(), c_pre, nullptr, 0));
        m.set_verbose(true);
        std::println("@@@ hook={} PRECOMPONENTS={}", hook, prec);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== components hook={} PRECOMPONENTS={}: {}", hook, prec, cp::summary(m, t));
    }
}
// ---- Q11: a lazy row on a component the presolve solves alone. It forbids that component's best answer.
struct clctx { const x::xpress_api * api; std::shared_ptr<xp_fns> fn; int cols; cp::raw_cut row; long calls = 0, viol = 0, added = 0, rejected = 0; std::map<int, long> status; };
static void cl_pre(prob_t cb, void * d, int soltype, int * p_reject, double *) {
    auto & c = *static_cast<clctx *>(d);
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    ++c.calls;
    if(c.row.violation(xv) <= 1e-6) return;
    ++c.viol;
    if(soltype != 0) { *p_reject = 1; ++c.rejected; return; }
    int cols = 0, n = 0, status = -1; double prhs = 0;
    c.api->getintattrib(cb, A_COLS, &cols);
    std::vector<int> pi(static_cast<std::size_t>(cols)); std::vector<double> pv(static_cast<std::size_t>(cols));
    c.fn->presolverow(cb, c.row.sense, static_cast<int>(c.row.ind.size()), c.row.ind.data(), c.row.val.data(), c.row.rhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
    c.status[status]++;
    const int s2[2] = {0, n}, type = 1;
    if(status < 0 || c.api->addcuts(cb, 1, &type, &c.row.sense, &prhs, s2, pi.data(), pv.data())) { *p_reject = 1; ++c.rejected; } else ++c.added;
}
template <class Fn>
static auto build_blocks(M & m, Fn && after_first_block) {
    std::vector<int> first;
    std::mt19937 rng(11);
    std::uniform_int_distribution<int> A(1, 1000), R(0, 500);
    m.set_maximization();
    constexpr std::size_t C = 22;
    for(int block = 0; block < 3; ++block) {
        auto x = m.add_binary_variables(C);
        const auto ids = std::views::iota(std::size_t{0}, C);
        std::vector<std::vector<double>> a(5, std::vector<double>(C));
        for(auto & row : a) for(auto & v : row) v = A(rng);
        for(std::size_t j : ids) {
            double sum = 0; for(const auto & row : a) sum += row[j];
            m.set_objective_coefficient(x(j), std::floor(sum / 5 + R(rng)));
            if(block == 0) first.push_back(m.native_id(x(j)));
        }
        for(const auto & row : a) {
            double sum = 0; for(double v : row) sum += v;
            m.add_constraint(xsum(ids, [&row, &x](std::size_t j) { return row[j] * x(j); }) <= std::floor(sum / 2));
        }
    }
    after_first_block(first);
    return first;
}
static void component_lazy() {
    std::vector<int> first; std::vector<int> chosen;
    { M m; one_thread(m); first = build_blocks(m, [](const auto &) {}); m.set_time_limit(secs(60)); const double t = cp::timed_solve(m); auto sol = m.get_solution();
      for(int j : first) if(sol[mippp::model_variable<int, double>(j)] > 0.5) chosen.push_back(j); std::println(stderr, "plain, default controls: {} chosen in block 0: {}", cp::summary(m, t), chosen.size()); }
    const cp::raw_cut row{chosen, std::vector<double>(chosen.size(), 1.0), 'L', static_cast<double>(chosen.size()) - 1};
    double truth;
    { M m; one_thread(m); build_blocks(m, [](const auto &) {});
      auto & api = m.native_api();
      const char sense = 'L'; const double rhs = row.rhs, rng = 0; const int start[2] = {0, static_cast<int>(row.ind.size())};
      api._check(m.native_model(), api.addrows(m.native_model(), 1, start[1], &sense, &rhs, &rng, start, row.ind.data(), row.val.data()));
      m.set_time_limit(secs(60)); const double t = cp::timed_solve(m); truth = m.get_solution_value(); std::println(stderr, "explicit row, default controls: {}", cp::summary(m, t)); }
    // setting: "default" leaves PRECOMPONENTS alone; "-1 after" sets it again AFTER MIPDUALREDUCTIONS, whose doc says it adjusts other controls
    struct cfg { int dr; int prec; const char * name; };
    for(const cfg f : {cfg{1, -100, "MIPDUALREDUCTIONS=1"}, cfg{0, -100, "MIPDUALREDUCTIONS=0"}, cfg{0, -1, "MIPDUALREDUCTIONS=0 then PRECOMPONENTS=-1"}, cfg{1, 0, "MIPDUALREDUCTIONS=1 PRECOMPONENTS=0"}, cfg{0, 0, "MIPDUALREDUCTIONS=0 PRECOMPONENTS=0"}}) {
        M m; one_thread(m); build_blocks(m, [](const auto &) {});
        set_int(m, C_MIPDUALREDUCTIONS, f.dr);
        if(f.prec != -100) set_int(m, C_PRECOMPONENTS, f.prec);
        m.set_time_limit(secs(60));
        clctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.cols = static_cast<int>(m.num_variables()); c.row = row;
        m.native_api()._check(m.native_model(), m.native_api().addcbpreintsol(m.native_model(), cl_pre, &c, 0));
        const double t = cp::timed_solve(m);
        double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
        std::string st; for(auto [k, q] : c.status) st += std::format("{}:{} ", k, q);
        int prec_now = 0, dr_now = 0; m.native_api().getintcontrol(m.native_model(), C_PRECOMPONENTS, &prec_now); m.native_api().getintcontrol(m.native_model(), C_MIPDUALREDUCTIONS, &dr_now);
        std::println(stderr, "== lazy row on a component, {} (read back PRECOMPONENTS={} MIPDUALREDUCTIONS={}): {} truth {} => {} calls={} violated={} added={} rejected={} presolverow{{{}}}", f.name, prec_now, dr_now, cp::summary(m, t), truth, std::fabs(v - truth) < 0.5 ? "value right" : "VALUE WRONG", c.calls, c.viol, c.added, c.rejected, st);
    }
}
// ---- Q14: does a wrapper that sets MIPDUALREDUCTIONS=0 for a solve and restores it get the default search back?
static void restore_dual() {
    const auto plain = [](const char * name, auto && prepare) {
        M m; one_thread(m); build_blocks(m, [](const auto &) {}); m.set_time_limit(secs(60));
        prepare(m);
        const double t = cp::timed_solve(m);
        std::println(stderr, "== {}: {}", name, cp::summary(m, t));
    };
    plain("default", [](M &) {});
    plain("MIPDUALREDUCTIONS=0", [](M & m) { set_int(m, C_MIPDUALREDUCTIONS, 0); });
    plain("set 0 then 1 before the solve", [](M & m) { set_int(m, C_MIPDUALREDUCTIONS, 0); set_int(m, C_MIPDUALREDUCTIONS, 1); });
    M m; one_thread(m); build_blocks(m, [](const auto &) {}); m.set_time_limit(secs(60));
    set_int(m, C_MIPDUALREDUCTIONS, 0);
    double t = cp::timed_solve(m); std::println(stderr, "== same model, first solve at 0: {}", cp::summary(m, t));
    set_int(m, C_MIPDUALREDUCTIONS, 1);
    m.set_objective_offset(0.0);
    t = cp::timed_solve(m); std::println(stderr, "== same model, second solve after restoring 1: {}", cp::summary(m, t));
}
// ---- Q14: does XPRS_LPOBJVAL in a hook equal the objective of the input-space point, on a model where presolve removes costed columns?
struct octr { const x::xpress_api * api; cp::atsp * a; int cols; double fixed; long n = 0; double maxdiff = 0; };
static void ob_body(octr & c, prob_t cb) {
    int lp = 0; c.api->getintattrib(cb, A_LPSTATUS, &lp);
    if(lp != LP_OPTIMAL) return;
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    double obj = 0; c.api->getdblattrib(cb, D_LPOBJVAL, &obj);
    ++c.n; c.maxdiff = std::max(c.maxdiff, std::fabs(obj - (c.a->objective(xv) + c.fixed)));
}
static void ob_opt(prob_t cb, void * d, int *) { ob_body(*static_cast<octr *>(d), cb); }
static void ob_cut(prob_t cb, void * d, int, int *) { ob_body(*static_cast<octr *>(d), cb); }
static void objread() {
    for(const std::string hook : {"optnode", "cutround"}) {
        M m; one_thread(m);
        auto f = m.add_variables(10, {.obj_coef = 0, .lower_bound = 1, .upper_bound = 1});
        auto a = cp::build_atsp(m, 26);
        for(std::size_t j = 0; j < 10; ++j) m.set_objective_coefficient(f(j), 7);
        m.set_time_limit(secs(20));
        octr c; c.api = &m.native_api(); c.a = &a; c.cols = static_cast<int>(m.num_variables()); c.fixed = 70;
        if(hook == "optnode") m.native_api()._check(m.native_model(), m.native_api().addcboptnode(m.native_model(), ob_opt, &c, 0));
        else m.native_api()._check(m.native_model(), fns_of(m)->addcbcutround(m.native_model(), ob_cut, &c, 0));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== LPOBJVAL against c.x, hook {}, 10 fixed columns costing 70 removed by presolve: {} firings={} max |LPOBJVAL - c.x|={}", hook, cp::summary(m, t), c.n, c.maxdiff);
    }
}
// ---- Q9: does a cut survive an in-tree restart? A witness column d the objective pushes to 1
// and the cut d <= 0.5, added once at the root. XPRSloadsecurevecs keeps d through presolve.
struct rctx {
    const x::xpress_api * api; std::shared_ptr<xp_fns> fn; int cols, d; std::string mode;
    bool added = false; std::map<int, std::array<long, 3>> per_restart; std::set<std::pair<int, int>> cols_seen;
    long depth1_after = 0;
    int add_err = 0;
    long trigger_at = -1, seen = 0; bool triggered = false; int trigger_ret = 0, trigger_status = -9, tree = 0;
    int (*trigger)(prob_t, int *) = nullptr;
};
using securevecs_t = int(prob_t, int, int, const int *, const int *);
static void r_body(rctx & c, prob_t cb, int * p_action) {
    int restarts = 0, lp = 0, cols = 0, depth = 0;
    c.api->getintattrib(cb, A_RESTARTS, &restarts); c.api->getintattrib(cb, A_LPSTATUS, &lp); c.api->getintattrib(cb, A_COLS, &cols); c.api->getintattrib(cb, A_NODEDEPTH, &depth);
    c.cols_seen.insert({restarts, cols});
    c.api->getintattrib(cb, 1335, &c.tree);  // TREERESTARTS
    if(lp != LP_OPTIMAL) return;
    if(c.trigger && !c.triggered && ++c.seen > c.trigger_at) { c.triggered = true; c.trigger_ret = c.trigger(cb, &c.trigger_status); }
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    auto & st = c.per_restart[restarts];
    ++st[0];
    if(c.added && xv[static_cast<std::size_t>(c.d)] > 0.5 + 1e-6) ++st[1];
    if(c.added && depth == 1) ++st[2];
    if(!c.added && xv[static_cast<std::size_t>(c.d)] > 0.5 + 1e-6) {
        const int ind = c.d, start[2] = {0, 1}; const double val = 1, rhs = 0.5; const char sense = 'L'; const int type = 1;
        if(c.mode == "addcuts") {
            int n = 0, status = 0; double prhs = 0;
            std::vector<int> pi(static_cast<std::size_t>(cols)); std::vector<double> pv(static_cast<std::size_t>(cols));
            c.fn->presolverow(cb, sense, 1, &ind, &val, rhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
            const int s2[2] = {0, n};
            c.add_err = c.api->addcuts(cb, 1, &type, &sense, &prhs, s2, pi.data(), pv.data());
        } else {
            c.add_err = c.fn->addmanaged(cb, c.mode == "managed_global", 1, &sense, &rhs, start, &ind, &val);
            if(p_action) *p_action = 2;
        }
        c.added = true;
    }
}
static void r_optnode(prob_t cb, void * d, int *) { r_body(*static_cast<rctx *>(d), cb, nullptr); }
static void r_cutround(prob_t cb, void * d, int, int * a) { r_body(*static_cast<rctx *>(d), cb, a); }
static void restart() {
    for(const std::string mode : {"addcuts", "managed_global", "managed_local"}) for(const int rootpre : {-100, 0}) for(const bool force : {false, true}) {
        M m; one_thread(m);
        cp::build_mkp(m, 30, 100);
        auto dv = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
        m.set_objective_coefficient(dv(std::size_t{0}), 0.001);
        m.set_time_limit(secs(25));
        rctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.cols = static_cast<int>(m.num_variables()); c.d = m.native_id(dv(std::size_t{0})); c.mode = mode;
        auto & api = m.native_api();
        auto sec = cp::symf<securevecs_t *>(api.library_path(), "XPRSloadsecurevecs");
        api._check(m.native_model(), sec(m.native_model(), 0, 1, nullptr, &c.d));
        set_int(m, C_MIPRESTART, 2);
        // with dual reductions on, presolve fixes d at 1: the witness cut then empties the root and the
        // search ends "optimal" at the incumbent, see the report. XP_RESTART_DUALRED=1 keeps them on to show it.
        if(!std::getenv("XP_RESTART_DUALRED")) set_int(m, C_MIPDUALREDUCTIONS, 0);
        if(rootpre != -100) set_int(m, 8224, rootpre);  // ROOTPRESOLVE
        if(force) {
            void * h = dlopen(api.library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
            c.trigger = reinterpret_cast<int (*)(prob_t, int *)>(dlsym(h, "XPRStriggerrestart"));
            c.trigger_at = 300;
        }
        if(mode == "addcuts") api._check(m.native_model(), api.addcboptnode(m.native_model(), r_optnode, &c, 0));
        else api._check(m.native_model(), c.fn->addcbcutround(m.native_model(), r_cutround, &c, 0));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== restart witness mode {} ROOTPRESOLVE={} trigger={}: {} add_err={} added={} TREERESTARTS={} trigger ret={} status={}", mode, rootpre, c.trigger != nullptr, cp::summary(m, t), c.add_err, c.added, c.tree, c.trigger_ret, c.trigger_status);
        for(auto [r, st] : c.per_restart) std::println(stderr, "   RESTARTS={}: firings={} with d>0.5 (cut not in force)={} root firings after the add={}", r, st[0], st[1], st[2]);
        std::string cs; for(auto [r, cc] : c.cols_seen) cs += std::format(" r{}:{}", r, cc);
        std::println(stderr, "   COLS by restart:{}", cs);
    }
}

// ---- Q12: is the node readable in the hook?
struct nctx { const x::xpress_api * api; std::shared_ptr<xp_fns> fn; int cols, y0, printed = 0, maxp; bool presolved; long agree = 0, disagree = 0, fired = 0; int err_over = 0; };
static void n_body(nctx & c, prob_t cb, const char * who) {
    int node = 0, parent = 0, depth = 0, cols = 0, rows = 0, lp = 0;
    c.api->getintattrib(cb, A_CURRENTNODE, &node); c.api->getintattrib(cb, A_PARENTNODE, &parent); c.api->getintattrib(cb, A_NODEDEPTH, &depth);
    c.api->getintattrib(cb, A_COLS, &cols); c.api->getintattrib(cb, A_ROWS, &rows); c.api->getintattrib(cb, A_LPSTATUS, &lp);
    ++c.fired;
    if(lp != LP_OPTIMAL) return;
    std::vector<double> lb(static_cast<std::size_t>(cols)), ub(static_cast<std::size_t>(cols)), xp(static_cast<std::size_t>(cols));
    const int e1 = c.api->getlb(cb, lb.data(), 0, cols - 1), e2 = c.api->getub(cb, ub.data(), 0, cols - 1);
    c.fn->getpresolved(cb, nullptr, xp.data(), 0, cols - 1);
    std::vector<int> rowmap(static_cast<std::size_t>(rows)), colmap(static_cast<std::size_t>(cols));
    const int e3 = c.fn->getmap(cb, rowmap.data(), colmap.data());
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    int fixed = 0; double ylb = NAN, yub = NAN;
    for(int j = 0; j < cols; ++j) {
        const auto J = static_cast<std::size_t>(j);
        if(ub[J] - lb[J] < 1e-9) ++fixed;
        // the mapping sends this presolved column to an input column: its value there must match
        if(!e3 && colmap[J] >= 0 && colmap[J] < c.cols) { if(std::fabs(xp[J] - xv[static_cast<std::size_t>(colmap[J])]) < 1e-6) ++c.agree; else ++c.disagree; }
        if(!e3 && colmap[J] == c.y0) { ylb = lb[J]; yub = ub[J]; }
    }
    if(!c.err_over) { std::vector<double> big(static_cast<std::size_t>(c.cols) + 8); c.err_over = c.api->getlb(cb, big.data(), 0, c.cols - 1) ? 1 : -1; }
    if(c.printed++ < c.maxp)
        std::println(stderr, "   {} node={} parent={} depth={} COLS={} input={} fixed_cols={} y0: x={:.4f} presolved bounds=[{},{}] err(lb,ub,map)={},{},{}", who, node, parent, depth, cols, c.cols, fixed, xv[static_cast<std::size_t>(c.y0)], ylb, yub, e1, e2, e3);
}
static void n_opt(prob_t cb, void * d, int *) { n_body(*static_cast<nctx *>(d), cb, "optnode "); }
static void n_cut(prob_t cb, void * d, int, int *) { n_body(*static_cast<nctx *>(d), cb, "cutround"); }
static void reads() {
    for(const bool presolve_on : {false, true}) for(const std::string hook : {"optnode", "cutround"}) {
        M m;
        if(presolve_on) { one_thread(m); set_int(m, C_COVERCUTS, 0); set_int(m, C_GOMCUTS, 0); set_int(m, C_TREECOVERCUTS, 0); set_int(m, C_TREEGOMCUTS, 0); }
        else scope_settings(m);
        // k gadgets, K=30 fixed columns first when presolve is on: the presolved index differs from the input index
        if(presolve_on) m.add_variables(30, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 0});
        int y0 = -1;
        if(presolve_on) { auto a = cp::build_atsp(m, 26); y0 = a.col[0][1]; }
        else { auto g = cp::build_gadgets(m, 8, false); y0 = g.y[0]; }
        m.set_time_limit(secs(20));
        nctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.cols = static_cast<int>(m.num_variables()); c.y0 = y0; c.maxp = 12; c.presolved = presolve_on;
        auto & api = m.native_api();
        if(hook == "optnode") api._check(m.native_model(), api.addcboptnode(m.native_model(), n_opt, &c, 0));
        else api._check(m.native_model(), c.fn->addcbcutround(m.native_model(), n_cut, &c, 0));
        std::println(stderr, "== reads hook={} presolve_on={} (watched column: input {})", hook, presolve_on, c.y0);
        const double t = cp::timed_solve(m);
        std::println(stderr, "   {} fired={} presolved value == value at mapped input column: {} of {}; getlb over the input range err={}", cp::summary(m, t), c.fired, c.agree, c.agree + c.disagree, c.err_over);
    }
}

// ---- Q11: lazy constraints.
struct lctx {
    const x::xpress_api * api; std::shared_ptr<xp_fns> fn; cp::atsp * a; int cols; std::string lazy, user;
    std::mutex m;
    long pre[3] = {0, 0, 0}, pre_bad[3] = {0, 0, 0}, rejected = 0, lazy_cuts = 0, user_cuts = 0, user_calls = 0, integral_at_user = 0;
    std::map<int, long> status; std::map<int, long> errs; std::string first_err;
    int extra_lazy_var = -1;  // Y: a lazy row y >= x(0, next), see lazy_dual
    void err(prob_t cb, int e) { errs[e]++; if(first_err.empty()) { char msg[512] = {}; api->getlasterror(cb, msg); first_err = msg; } }
    bool is_tour(const std::vector<double> & x) { return a->separate(x, 1).empty(); }
};
static int l_add(lctx & c, prob_t cb, const cp::raw_cut & r, const bool managed) {
    const int start[2] = {0, static_cast<int>(r.ind.size())};
    if(managed) return c.fn->addmanaged(cb, 1, 1, &r.sense, &r.rhs, start, r.ind.data(), r.val.data());
    int cols = 0, n = 0, status = -1; double prhs = 0;
    c.api->getintattrib(cb, A_COLS, &cols);
    std::vector<int> pi(static_cast<std::size_t>(cols)); std::vector<double> pv(static_cast<std::size_t>(cols));
    int e = c.fn->presolverow(cb, r.sense, start[1], r.ind.data(), r.val.data(), r.rhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
    { std::lock_guard l(c.m); c.status[status]++; }
    if(e) return e;
    if(status < 0) return 9000 - status;
    const int s2[2] = {0, n}; const int type = 1;
    return c.api->addcuts(cb, 1, &type, &r.sense, &prhs, s2, pi.data(), pv.data());
}
static void l_preintsol(prob_t cb, void * d, int soltype, int * p_reject, double *) {
    auto & c = *static_cast<lctx *>(d);
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    auto cuts = c.a->separate(xv);
    { std::lock_guard l(c.m); c.pre[std::clamp(soltype, 0, 2)]++; if(!cuts.empty()) c.pre_bad[std::clamp(soltype, 0, 2)]++; }
    if(cuts.empty()) return;
    if(soltype != 0 || c.lazy == "reject_only") { *p_reject = 1; std::lock_guard l(c.m); ++c.rejected; return; }
    long ok = 0;
    for(const auto & r : cuts) {
        const int e = l_add(c, cb, r, c.lazy == "managed_preintsol");
        std::lock_guard l(c.m);
        if(e) c.err(cb, e); else { ++ok; ++c.lazy_cuts; }
    }
    if(!ok) { *p_reject = 1; std::lock_guard l(c.m); ++c.rejected; }
}
static void l_user_common(lctx & c, prob_t cb, int * p_action, const bool managed, const bool also_lazy) {
    int lp = 0; c.api->getintattrib(cb, A_LPSTATUS, &lp);
    if(lp != LP_OPTIMAL) return;
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    const bool integral = c.a->integral(xv);
    { std::lock_guard l(c.m); ++c.user_calls; c.integral_at_user += integral; }
    if(integral && !also_lazy) return;
    bool added = false;
    for(const auto & r : c.a->separate(xv)) {
        const int e = l_add(c, cb, r, managed);
        std::lock_guard l(c.m);
        if(e) c.err(cb, e); else { ++c.user_cuts; added = true; }
    }
    if(added && p_action) *p_action = 2;
}
static void l_optnode(prob_t cb, void * d, int *) { auto & c = *static_cast<lctx *>(d); l_user_common(c, cb, nullptr, false, c.lazy == "optnode_only"); }
static void l_cutround(prob_t cb, void * d, int, int * a) { auto & c = *static_cast<lctx *>(d); l_user_common(c, cb, a, c.user == "managed", false); }
template <class M_>
static cp::atsp build_dfj(M_ & m, const int n, const unsigned seed = 5) {
    // degree constraints only: the model is a relaxation until subtours are cut
    cp::atsp a; a.n = n;
    const auto N = static_cast<std::size_t>(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> P(0, 100), noise(1.0, 1.3);
    std::vector<std::pair<double, double>> pts(N);
    for(auto & p : pts) p = {P(rng), P(rng)};
    a.d.assign(N, std::vector<double>(N, 0));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j)
        a.d[i][j] = std::round(std::hypot(pts[i].first - pts[j].first, pts[i].second - pts[j].second) * noise(rng));
    const auto arcs = N * (N - 1);
    const auto id = [N](std::size_t i, std::size_t j) { return i * (N - 1) + (j < i ? j : j - 1); };
    auto x = m.add_binary_variables(arcs);
    a.col.assign(N, std::vector<int>(N, -1)); a.comp.assign(N, std::vector<int>(N, -1));
    for(std::size_t i = 0; i < N; ++i) for(std::size_t j = 0; j < N; ++j) if(i != j) a.col[i][j] = m.native_id(x(id(i, j)));
    m.set_minimization();
    const auto nodes = std::views::iota(std::size_t{0}, N);
    m.set_objective(xsum(nodes, [&](std::size_t i) {
        return xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return a.d[i][j] * x(id(i, j)); });
    }));
    for(std::size_t i : nodes) {
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(i, j)); }) == 1);
        m.add_constraint(xsum(nodes | std::views::filter([i](std::size_t j) { return j != i; }), [&, i](std::size_t j) { return 1.0 * x(id(j, i)); }) == 1);
    }
    return a;
}
static void lazy() {
    struct cfg { std::string name, lazy, user; int dualred; int threads; };
    for(const int n : {26, 40}) {
        double base = NAN;
        { M m; one_thread(m); cp::build_atsp(m, n); m.set_time_limit(secs(60)); cp::timed_solve(m); try { base = m.get_solution_value(); std::println(stderr, "n={} compact MTZ value: {}", n, cp::summary(m, 0)); } catch(...) {} }
        for(const auto & f : std::vector<cfg>{
                {"lazy preintsol addcuts, MIPDUALREDUCTIONS=0", "addcuts", "none", 0, 1},
                {"lazy preintsol addcuts, user addcuts on fractional (optnode)", "addcuts", "addcuts", 0, 1},
                {"lazy preintsol addcuts, user managed global (cutround)", "addcuts", "managed", 0, 1},
                {"same, default threads", "addcuts", "managed", 0, -1},
                {"lazy preintsol addcuts, user managed, default MIPDUALREDUCTIONS=1", "addcuts", "managed", 1, 1},
                {"p_reject only (what reject_solution does), MIPDUALREDUCTIONS=0", "reject_only", "none", 0, 1},
                {"lazy from optnode only (no preintsol), integral points cut there", "optnode_only", "none", 0, 1},
                {"XPRSaddmanagedcuts from preintsol", "managed_preintsol", "none", 0, 1}}) {
            M m;
            if(f.threads == 1) one_thread(m);
            set_int(m, C_MIPDUALREDUCTIONS, f.dualred);
            auto a = build_dfj(m, n);
            m.set_time_limit(secs(60));
            lctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.a = &a; c.cols = static_cast<int>(m.num_variables()); c.lazy = f.lazy; c.user = f.user;
            auto & api = m.native_api();
            if(f.lazy != "optnode_only") api._check(m.native_model(), api.addcbpreintsol(m.native_model(), l_preintsol, &c, 0));
            if(f.user == "addcuts" || f.lazy == "optnode_only") api._check(m.native_model(), api.addcboptnode(m.native_model(), l_optnode, &c, 0));
            if(f.user == "managed") api._check(m.native_model(), c.fn->addcbcutround(m.native_model(), l_cutround, &c, 0));
            const double t = cp::timed_solve(m);
            double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
            std::string st; for(auto [k, q] : c.status) st += std::format("{}:{} ", k, q);
            std::string er; for(auto [k, q] : c.errs) er += std::format("{}:{} ", k, q);
            bool tour = false; try { auto sol = m.get_solution(); std::vector<double> xs(static_cast<std::size_t>(c.cols)); for(int j = 0; j < c.cols; ++j) xs[static_cast<std::size_t>(j)] = sol[mippp::model_variable<int, double>(j)]; tour = a.integral(xs) && a.separate(xs, 1).empty(); } catch(const std::exception &) {}
            std::println(stderr, "== n={} [{}]: {} true optimum {} => {} final_is_a_tour={}\n   preintsol soltype0/1/2 calls {}/{}/{} with subtours {}/{}/{} rejected={} lazy_cuts={} user_calls={} user_cuts={} integral_at_user={} presolverow_status{{{}}} errors{{{}}} {}", n, f.name, cp::summary(m, t), base,
                std::fabs(v - base) < 1e-6 ? "value right" : "VALUE WRONG", tour, c.pre[0], c.pre[1], c.pre[2], c.pre_bad[0], c.pre_bad[1], c.pre_bad[2], c.rejected, c.lazy_cuts, c.user_calls, c.user_cuts, c.integral_at_user, st, er, c.first_err);
        }
    }
}
// A lazy row on a column dual reductions remove: y costs 1 and is in no row, the lazy row is y >= x(0,k)
// with x(0,k) an arc of the best tour. Truth comes from the same row given up front.
struct ldctx { lctx * base; cp::raw_cut row; long added = 0; int bad = 0; };
static void ld_preintsol(prob_t cb, void * d, int soltype, int * p_reject, double *) {
    auto & q = *static_cast<ldctx *>(d);
    auto & c = *q.base;
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    auto cuts = c.a->separate(xv);
    if(q.row.violation(xv) > 1e-6) cuts.push_back(q.row);
    if(cuts.empty()) return;
    if(soltype != 0) { *p_reject = 1; return; }
    long ok = 0;
    for(const auto & r : cuts) {
        const int e = l_add(c, cb, r, false);
        if(e) { c.err(cb, e); ++q.bad; } else ++ok;
    }
    if(!ok) *p_reject = 1;
    q.added += ok;
}
static void lazy_dual() {
    for(const int n : {26, 40}) {
        // best tour of the plain model, then an arc of it
        int k = -1;
        { M m; one_thread(m); auto a = cp::build_atsp(m, n); m.set_time_limit(secs(60)); cp::timed_solve(m); auto sol = m.get_solution();
          for(int j = 1; j < n; ++j) if(sol[mippp::model_variable<int, double>(a.col[0][static_cast<std::size_t>(j)])] > 0.5) k = j; }
        double truth = NAN;
        {
            M m; one_thread(m); set_int(m, C_MIPDUALREDUCTIONS, 1);
            auto a = cp::build_atsp(m, n);
            auto y = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
            m.set_objective_coefficient(y(std::size_t{0}), 2);
            auto xcols = m.num_variables();
            (void)xcols;
            const int yc = m.native_id(y(std::size_t{0})), xc = a.col[0][static_cast<std::size_t>(k)];
            const char sense = 'G'; const double rhs = 0, rng = 0; const int start[2] = {0, 2}; const int ind[2] = {yc, xc}; const double val[2] = {1, -1};
            auto & api = m.native_api();
            api._check(m.native_model(), api.addrows(m.native_model(), 1, 2, &sense, &rhs, &rng, start, ind, val));
            m.set_time_limit(secs(60));
            const double t = cp::timed_solve(m);
            truth = m.get_solution_value();
            std::println(stderr, "n={} arc 0->{} explicit row y >= x, y costs 2: {}", n, k, cp::summary(m, t));
        }
        for(const int dr : {1, 0, 2}) {
            M m; one_thread(m);
            set_int(m, C_MIPDUALREDUCTIONS, dr);
            auto a = build_dfj(m, n);
            auto y = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
            m.set_objective_coefficient(y(std::size_t{0}), 2);
            m.set_time_limit(secs(60));
            lctx c; c.api = &m.native_api(); c.fn = fns_of(m); c.a = &a; c.cols = static_cast<int>(m.num_variables()); c.lazy = "addcuts"; c.user = "none";
            ldctx q; q.base = &c; q.row = cp::raw_cut{{m.native_id(y(std::size_t{0})), a.col[0][static_cast<std::size_t>(k)]}, {1.0, -1.0}, 'G', 0.0};
            auto & api = m.native_api();
            api._check(m.native_model(), api.addcbpreintsol(m.native_model(), ld_preintsol, &q, 0));
            const double t = cp::timed_solve(m);
            double v = NAN; try { v = m.get_solution_value(); } catch(const std::exception &) {}
            std::string st; for(auto [s2, cnt] : c.status) st += std::format("{}:{} ", s2, cnt);
            std::string er; for(auto [s2, cnt] : c.errs) er += std::format("{}:{} ", s2, cnt);
            std::println(stderr, "== n={} lazy row on a dual-reduced column, MIPDUALREDUCTIONS={}: {} truth {} => {} presolverow_status{{{}}} errors{{{}}} lazy_added={} unadded={}", n, dr, cp::summary(m, t), truth, std::fabs(v - truth) < 1e-6 ? "value right" : "VALUE WRONG", st, er, q.added, q.bad);
        }
    }
}
// preintsol is outside the list in the MUTEXCALLBACKS entry. A long spin on a model with many candidates.
struct opctx { octx o; lctx * l; };
static void op_preintsol(prob_t cb, void * d, int soltype, int * p_reject, double * p_cutoff) {
    auto & c = *static_cast<opctx *>(d);
    o_common(c.o, cb);
    l_preintsol(cb, c.l, soltype, p_reject, p_cutoff);
}
static void overlap_preintsol() {
    for(const int mutex : {1, 0}) {
        M m;
        set_int(m, C_MIPDUALREDUCTIONS, 0);
        set_int(m, C_MUTEXCALLBACKS, mutex);
        auto a = build_dfj(m, 60);
        m.set_time_limit(secs(20));
        lctx l; l.api = &m.native_api(); l.fn = fns_of(m); l.a = &a; l.cols = static_cast<int>(m.num_variables()); l.lazy = "addcuts"; l.user = "none";
        opctx c; c.l = &l; c.o.spin_us = 20000; c.o.api = &m.native_api();
        auto & api = m.native_api();
        api._check(m.native_model(), api.addcbpreintsol(m.native_model(), op_preintsol, &c, 0));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== preintsol 20 ms spin MUTEXCALLBACKS={}: {} calls={} overlaps={} peak_inflight={} os_threads={} cbprobs={} worker_ids={} soltype0/1/2={}/{}/{}", mutex, cp::summary(m, t), c.o.calls.load(), c.o.over.load(), c.o.peak.load(), c.o.th.size(), c.o.probs.size(), c.o.ids.size(), l.pre[0], l.pre[1], l.pre[2]);
    }
}
// The public wrapper API alone: set_candidate_solution_callback and reject_solution.
static void wrapper_reject() {
    for(const int n : {26, 40}) {
        M m; one_thread(m);
        auto a = build_dfj(m, n);
        m.set_time_limit(secs(60));
        long calls = 0, rejected = 0;
        const int cols = static_cast<int>(m.num_variables());
        m.set_candidate_solution_callback([&](auto & h) {
            ++calls;
            auto sol = h.get_solution();
            std::vector<double> xs(static_cast<std::size_t>(cols));
            for(int j = 0; j < cols; ++j) xs[static_cast<std::size_t>(j)] = sol[mippp::model_variable<int, double>(j)];
            if(!a.separate(xs, 1).empty()) { h.reject_solution(); ++rejected; }
        });
        const double t = cp::timed_solve(m);
        std::println(stderr, "== wrapper reject_solution only, degree-only model, n={}: {} candidate calls={} rejected={}", n, cp::summary(m, t), calls, rejected);
    }
}
int main(int argc, char ** argv) {
    const std::string what = argc > 1 ? argv[1] : "";
    if(what == "persist") persist();
    else if(what == "presolverow") presolverow_cases();
    else if(what == "naive_shift") naive_shift();
    else if(what == "strategy0") strategy0();
    else if(what == "objective") objective_cuts();
    else if(what == "overlap") overlap();
    else if(what == "restart") restart();
    else if(what == "reads") reads();
    else if(what == "lazy") lazy();
    else if(what == "wrapper_reject") wrapper_reject();
    else if(what == "lazy_dual") lazy_dual();
    else if(what == "pstatus") pstatus(argc, argv);
    else if(what == "components") components();
    else if(what == "objread") objread();
    else if(what == "restore_dual") restore_dual();
    else if(what == "component_lazy") component_lazy();
    else if(what == "naive_wrong") naive_wrong();
    else if(what == "overlap_preintsol") overlap_preintsol();
    else { std::println(stderr, "unknown"); return 2; }
    flush_last();
}
