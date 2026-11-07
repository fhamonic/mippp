// Q4: hint or binding. Filter, purge, force, measured by re-reading older cuts at later points.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct T {
    const c::cplex_api * api; const syms * s; int cols; int purgeable, local;
    std::function<void(ctxptr, const std::vector<double> &, cpxlong, cpxlong, int, T &)> body;
    long calls = 0;
    int add(ctxptr ctx, const raw_cut & r) {
        const int beg = 0;
        return api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &purgeable, &local);
    }
};
static int thook(ctxptr ctx, cpxlong id, void * usr) {
    auto & t = *static_cast<T *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    cpxlong uid = -1, depth = -1; int after = 0;
    t.s->infolong(ctx, INFO_NODEUID, &uid); t.s->infolong(ctx, INFO_NODEDEPTH, &depth); t.s->infoint(ctx, INFO_AFTERCUTLOOP, &after);
    std::vector<double> x(static_cast<std::size_t>(t.cols));
    t.s->point(ctx, x.data(), 0, t.cols - 1, nullptr);
    t.calls++;
    t.body(ctx, x, uid, depth, after, t);
    return 0;
}
static void only_user_cuts(M & m) {
    for(int p : {P_CLIQUES, P_COVERS, P_DISJCUTS, P_FLOWCOVERS, P_FLOWPATHS, P_FRACCUTS, P_GUBCOVERS, P_IMPLBD, P_LANDP, P_MCF, P_ZEROHALF, P_LOCALIMPLBD, P_BQP, P_RLT})
        param(m, p, -1);
    param(m, P_EACHCUTLIM, 0);
    param(m, P_HEURFREQ, -1); param(m, P_RINS, -1); param(m, P_FPHEUR, -1); param(m, P_LBHEUR, 0);
    auto [env, lp] = m.native_model(); m.native_api()._check(env, m.native_api().setdblparam(env, P_HEUREFFORT, 0.0));
    param(m, P_THREADS, 1);
}
static std::string numcuts(M & m) {
    auto [env, lp] = m.native_model(); int n = -1;
    const int e = syms::get(m).numcuts(env, lp, CUT_USER, &n);
    return std::format("CPXgetnumcuts(USER)={} err={}", n, e);
}
// ATSP: every cut ever added is re-read at every later call.
static void atsp_track(const std::string & mode, int n, std::size_t per_call, unsigned seed, bool fractional_only_separation = false) {
    M m; only_user_cuts(m);
    auto a = cp::build_atsp(m, n, false, seed);
    m.set_time_limit(std::chrono::duration<double>(30));
    const auto [pg, lc] = mode_flags(mode);
    T t{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc};
    struct seen { raw_cut c; cpxlong uid; long call; };
    std::vector<seen> added;
    std::set<std::string> keys;
    long same_node_prev = 0, same_node_older = 0, other_node = 0, readded = 0, checks = 0, cuts = 0, errors = 0, prev_call_cuts = 0;
    long call_no = 0;
    t.body = [&](ctxptr ctx, const std::vector<double> & x, cpxlong uid, cpxlong, int, T & tt) {
        ++call_no;
        for(const auto & s : added) {
            ++checks;
            if(s.c.violation(x) > 1e-5) {
                if(s.uid == uid) { (s.call == call_no - 1 ? same_node_prev : same_node_older)++; }
                else other_node++;
            }
        }
        for(const auto & r : a.separate(x, per_call)) {
            std::string k = std::format("{}", r.rhs);
            for(int j : r.ind) k += std::format(",{}", j);
            if(!keys.insert(k).second) readded++;
            if(tt.add(ctx, r)) errors++; else { cuts++; added.push_back({r, uid, call_no}); }
        }
    };
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, thook, &t));
    const double s = cp::timed_solve(m);
    std::println(stderr, "== atsp n={} seed={} {} cuts/call {}: {}\n   calls={} cuts={} errors={} re-added-identical={} older-cut-violated: same node prev call={} same node earlier={} other nodes={} of {} checks; {}",
                 n, seed, per_call, mode, cp::summary(m, s), t.calls, cuts, errors, readded, same_node_prev, same_node_older, other_node, checks, numcuts(m));
    std::fflush(stderr);
}
// MKP with an integer objective: sum c x <= floor(root LP) is valid, dense, and weakly violated.
static void mkp_dense(const std::string & mode) {
    M m; only_user_cuts(m);
    const int cols = 100;
    cp::build_mkp(m, 30, cols);
    auto [env, lp] = m.native_model();
    std::vector<double> obj(static_cast<std::size_t>(cols));
    cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *, int, int)>(m.native_api().library_path(), "CPXgetobj")(env, lp, obj.data(), 0, cols - 1);
    m.set_time_limit(std::chrono::duration<double>(5));
    const auto [pg, lc] = mode_flags(mode);
    T t{&m.native_api(), &syms::get(m), cols, pg, lc};
    std::optional<raw_cut> cut; int err = 0; long at = 0, checks = 0, viol = 0, root_calls = 0, depth_calls = 0; double lpobj = NAN;
    t.body = [&](ctxptr ctx, const std::vector<double> & x, cpxlong, cpxlong depth, int, T & tt) {
        (depth == 0 ? root_calls : depth_calls)++;
        double v = 0; for(int j = 0; j < cols; ++j) v += obj[static_cast<std::size_t>(j)] * x[static_cast<std::size_t>(j)];
        if(!cut) {
            if(depth != 0) return;
            raw_cut r; r.sense = 'L'; r.rhs = std::floor(v);
            for(int j = 0; j < cols; ++j) { r.ind.push_back(j); r.val.push_back(obj[static_cast<std::size_t>(j)]); }
            lpobj = v; cut = r; err = tt.add(ctx, r); at = t.calls; return;
        }
        if(depth == 0) { ++checks; viol += cut->violation(x) > 1e-5; }
    };
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, thook, &t));
    const double s = cp::timed_solve(m);
    std::println(stderr, "== mkp dense objective-floor cut at the root, {}: {}\n   root LP obj={:.3f} rhs={} err={} root calls={} tree calls={} later root points violating it: {}/{}; {}", mode, cp::summary(m, s), lpobj, cut ? cut->rhs : NAN, err, root_calls, depth_calls, viol, checks, numcuts(m));
    std::fflush(stderr);
}
// non-violated cut at every call. Does the node loop spin? root and tree, every purgeable.
static void spin(const std::string & mode, bool at_root, bool fresh) {
    M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0);
    auto g = cp::build_gadgets(m, 10, false);
    m.set_time_limit(std::chrono::duration<double>(5));
    const auto [pg, lc] = mode_flags(mode);
    T t{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), pg, lc};
    std::map<cpxlong, long> per; long added = 0; double rhs = 1e6;
    t.body = [&](ctxptr ctx, const std::vector<double> &, cpxlong uid, cpxlong depth, int, T & tt) {
        per[uid]++;
        if((depth == 0) != at_root) return;
        // fresh: a different, non-violated cut each time
        raw_cut r{{g.za[3]}, {1.0}, 'L', fresh ? (rhs += 1.0) : rhs};
        tt.add(ctx, r); added++;
    };
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, thook, &t));
    const double s = cp::timed_solve(m);
    long mx = 0; for(auto [u, n] : per) mx = std::max(mx, n);
    std::println(stderr, "== non-violated {} cut at every {} call, {}: {}\n   calls={} added={} nodes_fired={} max_calls_on_a_node={}; {}", fresh ? "FRESH" : "SAME", at_root ? "root" : "tree", mode, cp::summary(m, s), t.calls, added, per.size(), mx, numcuts(m));
    std::fflush(stderr);
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    const std::vector<std::string> modes = {"global-purge", "global-filter", "global-force", "local-purge", "local-filter", "local-force"};
    for(bool at_root : {true, false}) for(bool fresh : {false, true}) for(const auto & mode : modes) spin(mode, at_root, fresh);
    for(const auto & mode : modes) mkp_dense(mode);
    for(unsigned seed : {5u, 6u, 7u}) for(std::size_t per : {std::size_t{1}, std::size_t{30}}) for(const auto & mode : modes) atsp_track(mode, 31, per, seed);
}
