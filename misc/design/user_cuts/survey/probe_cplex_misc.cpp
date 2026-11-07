// Q1/Q3/Q7/Q14: objective read, post-loop duplicates, cuts against incumbents, native_id after removals.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct H {
    const c::cplex_api * api; const syms * s; int cols;
    std::function<void(ctxptr, const std::vector<double> &, cpxlong, cpxlong, int)> body;
    long calls = 0;
};
static int hhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & h = *static_cast<H *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(h.cols));
    h.s->point(ctx, x.data(), 0, h.cols - 1, nullptr);
    cpxlong uid = -1, d = -1; int after = 0;
    h.s->infolong(ctx, INFO_NODEUID, &uid); h.s->infolong(ctx, INFO_NODEDEPTH, &d); h.s->infoint(ctx, INFO_AFTERCUTLOOP, &after);
    ++h.calls;
    h.body(ctx, x, uid, d, after);
    return 0;
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    // a: objective through x == nullptr, on a max model with an offset
    {
        M m; param(m, P_THREADS, 1); param(m, P_HEURFREQ, -1); param(m, P_RINS, -1); param(m, P_FPHEUR, -1);
        cp::build_mkp(m, 10, 30);
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        api._check(env, api.chgobjoffset(env, lp, 1000.0));
        std::vector<double> obj(30);
        cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *, int, int)>(api.library_path(), "CPXgetobj")(env, lp, obj.data(), 0, 29);
        m.set_time_limit(std::chrono::duration<double>(2));
        H h{&api, &syms::get(m), 30};
        long checks = 0, bad = 0, null_err = 0, null_full_err = 0; double worst = 0;
        h.body = [&](ctxptr, const std::vector<double> &, cpxlong, cpxlong, int) {};
        struct W { H * h; long * checks; long * bad; long * null_err; long * null_full_err; double * worst; const std::vector<double> * obj; } w{&h, &checks, &bad, &null_err, &null_full_err, &worst, &obj};
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, [](ctxptr ctx, cpxlong, void * usr) -> int {
            auto & ww = *static_cast<W *>(usr);
            double o1 = NAN, o2 = 0;
            if(ww.h->s->point(ctx, nullptr, 0, -1, &o1)) ++*ww.null_err;
            if(ww.h->s->point(ctx, nullptr, 0, 29, &o2)) ++*ww.null_full_err;
            std::vector<double> x(30); ww.h->s->point(ctx, x.data(), 0, 29, nullptr);
            double manual = 1000; for(std::size_t j = 0; j < 30; ++j) manual += (*ww.obj)[j] * x[j];
            ++*ww.checks; *ww.worst = std::max(*ww.worst, std::fabs(manual - o1)); *ww.bad += std::fabs(manual - o1) > 1e-6;
            return 0;
        }, &w));
        cp::timed_solve(m);
        std::println(stderr, "== a getrelaxationpoint(nullptr, 0, -1, &obj): checks={} mismatch against c'x + offset={} worst={} errors: empty range={} full range with nullptr={}", checks, bad, worst, null_err, null_full_err);
    }
    // b: is the post-loop point the last in-loop point?
    {
        M m; param(m, P_THREADS, 1);
        auto a = cp::build_atsp(m, 26);
        m.set_time_limit(std::chrono::duration<double>(10));
        H h{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
        std::vector<double> prev; cpxlong prev_uid = -2; long after = 0, same = 0, diff = 0, first_at_node = 0;
        h.body = [&](ctxptr, const std::vector<double> & x, cpxlong uid, cpxlong, int af) {
            if(af) {
                ++after;
                if(uid != prev_uid) ++first_at_node;
                else { double mx = 0; for(std::size_t j = 0; j < x.size(); ++j) mx = std::max(mx, std::fabs(x[j] - prev[j])); (mx < 1e-9 ? same : diff)++; }
            }
            prev = x; prev_uid = uid;
        };
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, hhook, &h));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== b post-loop calls, atsp26 defaults, no cuts: {} calls={} post-loop={} identical to the previous call={} different={} first call at its node={}", cp::summary(m, t), h.calls, after, same, diff, first_at_node);
    }
    // c: an invalid global cut against an incumbent from a MIP start
    {
        std::vector<double> xstar; double opt = 0;
        {
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); opt = g.opt;
            m.set_time_limit(std::chrono::duration<double>(10));
            cp::timed_solve(m);
            auto [env, lp] = m.native_model();
            xstar.resize(m.num_variables());
            cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *, int, int)>(m.native_api().library_path(), "CPXgetx")(env, lp, xstar.data(), 0, static_cast<int>(xstar.size()) - 1);
        }
        for(const std::string mode : {"global-force", "global-purge"}) {
            M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0);
            auto g = cp::build_gadgets(m, 10, false);
            auto [env, lp] = m.native_model(); auto & api = m.native_api();
            std::vector<int> ind(xstar.size()); std::iota(ind.begin(), ind.end(), 0);
            const int beg = 0, effort = 1;  // CPX_MIPSTART_CHECKFEAS
            api._check(env, api.addmipstarts(env, lp, 1, static_cast<int>(ind.size()), &beg, ind.data(), xstar.data(), &effort, nullptr));
            m.set_time_limit(std::chrono::duration<double>(10));
            H h{&api, &syms::get(m), static_cast<int>(m.num_variables())};
            const raw_cut cut = g.invalid_global_cut(); bool added = false; int err = 0;
            const auto [pg, lc] = mode_flags(mode);
            struct I { H * hh; } ;
            static thread_local ctxptr current = nullptr;
            h.body = [&, pg, lc](ctxptr ctx, const std::vector<double> & x, cpxlong, cpxlong, int) {
                if(added || cut.violation(x) < 1e-4) return;
                added = true;
                const int b = 0;
                err = api.callbackaddusercuts(ctx, 1, static_cast<int>(cut.ind.size()), &cut.rhs, &cut.sense, &b, cut.ind.data(), cut.val.data(), &pg, &lc);
            };
            api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, hhook, &h));
            const double t = cp::timed_solve(m);
            std::vector<double> xs(m.num_variables()); double zsum = 0;
            cp::symf<int (*)(c::CPXCENVptr, c::CPXCLPptr, double *, int, int)>(api.library_path(), "CPXgetx")(env, lp, xs.data(), 0, static_cast<int>(xs.size()) - 1);
            for(int j : g.z) zsum += xs[static_cast<std::size_t>(j)];
            std::println(stderr, "== c MIP start at the optimum, then the invalid cut Sum z <= {} added {} (mode {}, err {}): {} ; returned point has Sum z = {} , violating the cut by {}", cut.rhs, added ? "yes" : "NO", mode, err, cp::summary(m, t), zsum, cut.violation(xs));
        }
    }
    // d: native_id after a removal, read in the hook
    {
        M m; param(m, P_THREADS, 1); param(m, P_HEURFREQ, -1); param(m, P_RINS, -1); param(m, P_FPHEUR, -1);
        auto x = m.add_binary_variables(12);
        const auto ids = std::views::iota(std::size_t{0}, std::size_t{12});
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return (3.0 + static_cast<double>(j)) * x(j); }));
        m.add_constraint(xsum(ids, [&](std::size_t j) { return (7.0 + 2.0 * static_cast<double>(j)) * x(j); }) <= 31.5);
        const auto gone = x(std::size_t{4}), kept = x(std::size_t{5});
        m.remove_variable(gone);
        m.set_time_limit(std::chrono::duration<double>(2));
        const auto before = m.native_id(kept);
        m.solve();
        std::println(stderr, "== d native_id(kept) before solve={} after={}, kept was handle index 5, one column removed before it; num cols {}", before, m.native_id(kept), m.native_api().getnumcols(m.native_model().first, m.native_model().second));
    }
    // e: purgeable or local left out
    for(auto [nm, pg_null, lc_null] : std::initializer_list<std::tuple<const char *, bool, bool>>{{"purgeable=NULL", true, false}, {"local=NULL", false, true}, {"both NULL", true, true}}) {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1);
        auto g = cp::build_gadgets(m, 8, false);
        m.set_time_limit(std::chrono::duration<double>(5));
        auto [env, lp] = m.native_model(); auto & api = m.native_api();
        H h{&api, &syms::get(m), static_cast<int>(m.num_variables())};
        int err = 12345; const auto r = g.hull_cut(3); long v = 0, ck = 0;
        const bool pn = pg_null, ln = lc_null;
        h.body = [&, pn, ln](ctxptr ctx, const std::vector<double> & x, cpxlong, cpxlong, int) {
            if(err == 12345 && r.violation(x) > 1e-4) {
                const int b = 0, pg = USECUT_PURGE, lc = 0;
                err = api.callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &b, r.ind.data(), r.val.data(), pn ? nullptr : &pg, ln ? nullptr : &lc);
                return;
            }
            if(err != 12345) { ++ck; v += r.violation(x) > 1e-6; }
        };
        api._check(env, api.callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, hhook, &h));
        const double t = cp::timed_solve(m);
        std::println(stderr, "== e {}: addusercuts err={} later viol/checks={}/{} | {}", nm, err, v, ck, cp::summary(m, t));
    }
}
