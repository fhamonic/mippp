// MOSEK: every callback code, what can be read in it, and what a row added from inside does.
// usage: probe_mosek_hooks codes [n] | read [n] | addrow <code: 80=IM_MIO> <call> <invalid|dfj> [n] | before <invalid|dfj> [n]
//        allrows <n> <seed> <0|1 from callback>
#include "mippp/solvers/mosek/all.hpp"
#include "cut_common.hpp"
using M = mippp::mosek_milp;
namespace v1 = mippp::mosek::impl::v1;
using v1::MSKtask_t;

static void hdr(M & m) {
    v1::MSKint32t a, b, c;
    (void)m.native_api().getversion(&a, &b, &c);
    std::println(stderr, "library {} version {}.{}.{}", m.native_api().library_path().string(), a, b, c);
}
static std::string code_str(M & m, int code) {
    char s[1024] = "";
    (void)m.native_api().callbackcodetostr(static_cast<v1::MSKcallbackcodee>(code), s);
    return s;
}
static constexpr int NCODES = 128;

// ------------------------------------------------------------- codes: in contract
struct codes_rec {
    long calls[NCODES] = {};
    int I_DEPTH = -1, I_NODES = -1, I_RELAX = -1, I_ROUNDS = -1, I_RESTARTS = -1, I_NSOL = -1, I_ACTIVE_CUTS = -1, D_BOUND = -1;
    int max_depth = -1, nodes = 0, relax = 0, rounds = 0, restarts = 0, active_cuts = 0;
    long im_mio_with_depth = 0;
    double first_bound = NAN, last_bound = NAN;
    long distinct_depths = 0;
    std::set<int> depths;
};
static int codes_cb(MSKtask_t, void * u, v1::MSKcallbackcodee c, const v1::MSKrealt * d, const v1::MSKint32t * i, const v1::MSKint64t *) {
    auto & r = *static_cast<codes_rec *>(u);
    const int k = static_cast<int>(c);
    if(k >= 0 && k < NCODES) ++r.calls[k];
    if(c == v1::MSK_CALLBACK_IM_MIO || c == v1::MSK_CALLBACK_NEW_INT_MIO || c == v1::MSK_CALLBACK_END_MIO) {
        r.max_depth = std::max(r.max_depth, i[r.I_DEPTH]);
        r.depths.insert(i[r.I_DEPTH]);
        r.nodes = i[r.I_NODES]; r.relax = i[r.I_RELAX]; r.rounds = i[r.I_ROUNDS]; r.restarts = i[r.I_RESTARTS]; r.active_cuts = i[r.I_ACTIVE_CUTS];
        if(std::isnan(r.first_bound)) r.first_bound = d[r.D_BOUND];
        r.last_bound = d[r.D_BOUND];
    }
    return 0;
}
static void bind_items(M & m, codes_rec & r) {
    auto & a = m.native_api();
    auto t = m.native_model().second;
    (void)a.getinfindex(t, v1::MSK_INF_INT_TYPE, "MSK_IINF_MIO_NODE_DEPTH", &r.I_DEPTH);
    (void)a.getinfindex(t, v1::MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_SOLVED_NODES", &r.I_NODES);
    (void)a.getinfindex(t, v1::MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_RELAX", &r.I_RELAX);
    (void)a.getinfindex(t, v1::MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_ROOT_CUT_ROUNDS", &r.I_ROUNDS);
    (void)a.getinfindex(t, v1::MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_RESTARTS", &r.I_RESTARTS);
    (void)a.getinfindex(t, v1::MSK_INF_INT_TYPE, "MSK_IINF_MIO_NUM_ACTIVE_ROOT_CUTS", &r.I_ACTIVE_CUTS);
    (void)a.getinfindex(t, v1::MSK_INF_DOU_TYPE, "MSK_DINF_MIO_OBJ_BOUND", &r.D_BOUND);
}
static int run_codes(int n) {
    M m; hdr(m); m.set_thread_limit(1);
    auto a = cp::build_atsp(m, n);
    m.set_time_limit(std::chrono::duration<double>(60));
    codes_rec r; bind_items(m, r);
    (void)m.native_api().putcallbackfunc(m.native_model().second, codes_cb, &r);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== codes atsp n={}: {}", n, cp::summary(m, t));
    long total = 0;
    for(int k = 0; k < NCODES; ++k) if(r.calls[k]) { std::println(stderr, "   {:>40} ({:3}): {}", code_str(m, k), k, r.calls[k]); total += r.calls[k]; }
    std::println(stderr, "   total calls {}; at the MIO codes: nodes={} relaxations={} root cut rounds={} restarts={} active root cuts={} depths seen={} max depth={} bound {:.6g} -> {:.6g}",
                 total, r.nodes, r.relax, r.rounds, r.restarts, r.active_cuts, r.depths.size(), r.max_depth, r.first_bound, r.last_bound);
    return 0;
}

// -------------------------------------------------------------- read: getxx in hook
struct read_rec {
    const mippp::mosek_api * api = nullptr;
    const cp::atsp * a = nullptr;
    std::size_t cols = 0;
    long calls[NCODES][3] = {};
    long ok[NCODES][3] = {};
    long frac[NCODES][3] = {};
    std::map<int, long> rc[3];
    double first_obj[NCODES][3];
    read_rec() { for(auto & row : first_obj) for(auto & v : row) v = NAN; }
};
static int read_cb(MSKtask_t t, void * u, v1::MSKcallbackcodee c, const v1::MSKrealt *, const v1::MSKint32t *, const v1::MSKint64t *) {
    auto & r = *static_cast<read_rec *>(u);
    const int k = static_cast<int>(c);
    if(k < 0 || k >= NCODES) return 0;
    std::vector<double> x(r.cols);
    for(int s = 0; s < 3; ++s) {
        const int e = static_cast<int>(r.api->getxx(t, static_cast<v1::MSKsoltypee>(s), x.data()));
        ++r.calls[k][s];
        ++r.rc[s][e];
        if(e == 0) {
            ++r.ok[k][s];
            if(!r.a->integral(x)) ++r.frac[k][s];
            if(std::isnan(r.first_obj[k][s])) r.first_obj[k][s] = r.a->objective(x);
        }
    }
    return 0;
}
static int run_read(int n) {
    M m; hdr(m); m.set_thread_limit(1);
    auto a = cp::build_atsp(m, n);
    m.set_time_limit(std::chrono::duration<double>(60));
    read_rec r; r.api = &m.native_api(); r.a = &a; r.cols = m.num_variables();
    (void)m.native_api().putcallbackfunc(m.native_model().second, read_cb, &r);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== read (getxx at every call, outside the contract except ITG at NEW_INT_MIO) atsp n={}: {}", n, cp::summary(m, t));
    static const char * const sn[] = {"ITR", "BAS", "ITG"};
    for(int k = 0; k < NCODES; ++k) if(r.calls[k][0]) {
        std::string line = std::format("   {:>36}:", code_str(m, k));
        for(int s = 0; s < 3; ++s) line += std::format(" {} ok {}/{} frac {} obj1 {:.6g};", sn[s], r.ok[k][s], r.calls[k][s], r.frac[k][s], r.first_obj[k][s]);
        std::println(stderr, "{}", line);
    }
    for(int s = 0; s < 3; ++s) for(auto [e, cnt] : r.rc[s]) std::println(stderr, "   getxx({}) rescode {}: {} times", sn[s], e, cnt);
    return 0;
}

// ----------------------------------------------------- rows: LP loop for DFJ rows
static std::vector<cp::raw_cut> collect_dfj(int n, int max_rounds = 60) {
    std::vector<cp::raw_cut> all;
    M m; m.set_thread_limit(1);
    auto a = cp::build_atsp(m, n);
    auto & api = m.native_api();
    auto t = m.native_model().second;
    const auto cols = m.num_variables();
    auto putvartype = cp::symf<int (*)(MSKtask_t, int, int)>(api.library_path(), "MSK_putvartype");
    for(std::size_t j = 0; j < cols; ++j) {
        const int e = putvartype(t, static_cast<int>(j), 0);
        if(e) std::println(stderr, "   collect_dfj: putvartype({}) returned {}", j, e);
    }
    std::vector<double> x(cols);
    for(int r = 0; r < max_rounds; ++r) {
        v1::MSKrescodee trm;
        const int oe = static_cast<int>(api.optimizetrm(t, &trm));
        int ge = static_cast<int>(api.getxx(t, v1::MSK_SOL_BAS, x.data()));
        if(ge) ge = static_cast<int>(api.getxx(t, v1::MSK_SOL_ITR, x.data()));
        // mosek_milp pins MSK_IPAR_OPTIMIZER to the mixed-integer one, which answers in ITG
        if(ge) ge = static_cast<int>(api.getxx(t, v1::MSK_SOL_ITG, x.data()));
        if(oe || ge) { std::println(stderr, "   collect_dfj: optimizetrm {} getxx {} at round {}", oe, ge, r); break; }
        auto cuts = a.separate(x);
        if(cuts.empty()) break;
        for(auto & c : cuts) {
            int row = 0;
            (void)api.getnumcon(t, &row);
            (void)api.appendcons(t, 1);
            (void)api.putarow(t, row, static_cast<int>(c.ind.size()), c.ind.data(), c.val.data());
            (void)api.putconbound(t, row, v1::MSK_BK_LO, c.rhs, 0.0);
            all.push_back(std::move(c));
        }
    }
    return all;
}
static cp::raw_cut invalid_cut(const cp::atsp & a, double rhs) {
    cp::raw_cut c; c.sense = 'G'; c.rhs = rhs;
    for(int i = 0; i < a.n; ++i) for(int j = 0; j < a.n; ++j) if(i != j) {
        c.ind.push_back(a.col[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
        c.val.push_back(a.d[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]);
    }
    return c;
}
struct addrow_rec {
    const mippp::mosek_api * api = nullptr;
    int code = 0;
    long at = 1, calls = 0, added = 0;
    int last_err = 0;
    std::map<int, long> errs;
    int rows_before = -1, rows_after = -1;
    long later_calls = 0;
    std::vector<cp::raw_cut> cuts;
};
static int add_rows(const mippp::mosek_api & api, MSKtask_t t, const std::vector<cp::raw_cut> & cuts, std::map<int, long> & errs, long & added) {
    int last = 0;
    for(const auto & c : cuts) {
        int row = 0;
        int e = static_cast<int>(api.getnumcon(t, &row));
        if(!e) e = static_cast<int>(api.appendcons(t, 1));
        if(!e) e = static_cast<int>(api.putarow(t, row, static_cast<int>(c.ind.size()), c.ind.data(), c.val.data()));
        if(!e) e = static_cast<int>(api.putconbound(t, row, c.sense == 'G' ? v1::MSK_BK_LO : v1::MSK_BK_UP, c.sense == 'G' ? c.rhs : 0.0, c.sense == 'G' ? 0.0 : c.rhs));
        ++errs[e];
        if(e) last = e; else ++added;
    }
    return last;
}
static int addrow_cb(MSKtask_t t, void * u, v1::MSKcallbackcodee c, const v1::MSKrealt *, const v1::MSKint32t *, const v1::MSKint64t *) {
    auto & r = *static_cast<addrow_rec *>(u);
    if(static_cast<int>(c) != r.code) return 0;
    ++r.calls;
    if(r.calls == r.at) {
        (void)r.api->getnumcon(t, &r.rows_before);
        r.last_err = add_rows(*r.api, t, r.cuts, r.errs, r.added);
        (void)r.api->getnumcon(t, &r.rows_after);
        std::println(stderr, "   inside code {} call {}: rows {} -> {}, added {} of {}, last rescode {}", r.code, r.calls, r.rows_before, r.rows_after, r.added, r.cuts.size(), r.last_err);
    } else if(r.calls > r.at) ++r.later_calls;
    return 0;
}
static void report_point(M & m, const cp::atsp & a, const std::vector<cp::raw_cut> & cuts) {
    std::vector<double> x(m.num_variables());
    const int e = static_cast<int>(m.native_api().getxx(m.native_model().second, v1::MSK_SOL_ITG, x.data()));
    long viol = 0;
    for(const auto & c : cuts) viol += c.violation(x) > 1e-6;
    int rows = 0; (void)m.native_api().getnumcon(m.native_model().second, &rows);
    std::println(stderr, "   final point (getxx rescode {}): objective {:.6g}, added rows violated {}, rows now {}", e, a.objective(x), viol, rows);
}
static int run_addrow(int code, long at, const std::string & kind, int n) {
    if(!std::getenv("SKIP_BASE")) {
        M m; hdr(m); m.set_thread_limit(1);
        auto a = cp::build_atsp(m, n);
        m.set_time_limit(std::chrono::duration<double>(60));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== addrow baseline: {}", cp::summary(m, t));
    }
    M m; m.set_thread_limit(1);
    auto a = cp::build_atsp(m, n);
    m.set_time_limit(std::chrono::duration<double>(60));
    addrow_rec r; r.api = &m.native_api(); r.code = code; r.at = at;
    if(kind == "invalid") r.cuts.push_back(invalid_cut(a, 550)); else r.cuts = collect_dfj(n);
    (void)m.native_api().putcallbackfunc(m.native_model().second, addrow_cb, &r);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== addrow in code {} ({}) at call {} cut={} ({} rows): {} calls of the code={} after the add={}", code, code_str(m, code), at, kind, r.cuts.size(), cp::summary(m, t), r.calls, r.later_calls);
    report_point(m, a, r.cuts);
    return 0;
}
static int run_before(const std::string & kind, int n) {
    M m; hdr(m); m.set_thread_limit(1);
    auto a = cp::build_atsp(m, n);
    m.set_time_limit(std::chrono::duration<double>(60));
    std::vector<cp::raw_cut> cuts;
    if(kind == "invalid") cuts.push_back(invalid_cut(a, 550)); else cuts = collect_dfj(n);
    std::map<int, long> errs; long added = 0;
    (void)add_rows(m.native_api(), m.native_model().second, cuts, errs, added);
    const double t = cp::timed_solve(m);
    std::println(stderr, "== the same rows as plain model rows before the solve, cut={} ({} rows, {} added): {}", kind, cuts.size(), added, cp::summary(m, t));
    report_point(m, a, cuts);
    return 0;
}

int main(int argc, char ** argv) {
    const std::string what = argc > 1 ? argv[1] : "codes";
    if(what == "codes") return run_codes(argc > 2 ? std::atoi(argv[2]) : 26);
    if(what == "read") return run_read(argc > 2 ? std::atoi(argv[2]) : 26);
    if(what == "addrow") return run_addrow(std::atoi(argv[2]), std::atol(argv[3]), argv[4], argc > 5 ? std::atoi(argv[5]) : 26);
    if(what == "dfjcount") { auto c = collect_dfj(argc > 2 ? std::atoi(argv[2]) : 26); std::println(stderr, "collect_dfj: {} rows", c.size()); return 0; }
    if(what == "before") return run_before(argv[2], argc > 3 ? std::atoi(argv[3]) : 26);
    return 2;
}
