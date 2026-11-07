// Q7: firing with cuts. Rounds per node, limits, the post-loop call, integral relaxations.
#include "probe_cplex_common.cpp"
using cp::raw_cut;
struct call { cpxlong uid, depth; int after; };
struct F {
    const c::cplex_api * api; const syms * s; int cols; int purgeable = USECUT_FORCE, local = 0;
    std::function<void(ctxptr, const std::vector<double> &, const call &, F &)> body;
    std::vector<call> calls; long in_cand = 0;
    int add(ctxptr ctx, const raw_cut & r) {
        const int beg = 0;
        return api->callbackaddusercuts(ctx, 1, static_cast<int>(r.ind.size()), &r.rhs, &r.sense, &beg, r.ind.data(), r.val.data(), &purgeable, &local);
    }
};
static int fhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & f = *static_cast<F *>(usr);
    if(id == c::CPX_CALLBACKCONTEXT_CANDIDATE) { f.in_cand++; return 0; }
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    call k{};
    f.s->infolong(ctx, INFO_NODEUID, &k.uid);
    f.s->infolong(ctx, INFO_NODEDEPTH, &k.depth);
    f.s->infoint(ctx, INFO_AFTERCUTLOOP, &k.after);
    f.calls.push_back(k);
    std::vector<double> x(static_cast<std::size_t>(f.cols));
    f.s->point(ctx, x.data(), 0, f.cols - 1, nullptr);
    if(f.body) f.body(ctx, x, k, f);
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
static void report(const std::string & n, F & f, M & m, double secs, std::string extra = {}) {
    std::map<cpxlong, int> per; long d0 = 0, after = 0; int mx = 0;
    for(auto & k : f.calls) { mx = std::max(mx, ++per[k.uid]); d0 += k.depth == 0; after += k.after; }
    std::map<int, long> hist; for(auto [u, c] : per) hist[c]++;
    std::string h; int shown = 0;
    for(auto it = hist.begin(); it != hist.end() && shown < 12; ++it, ++shown) h += std::format(" {}x:{}", it->first, it->second);
    std::println(stderr, "== {}: {}\n   calls={} depth0={} after_loop={} nodes_fired={} max_calls_on_a_node={} per-node:{} {}", n, cp::summary(m, secs), f.calls.size(), d0, after, per.size(), mx, h, extra);
    std::fflush(stderr);
}
template <class Build>
static void run(const std::string & n, std::function<void(M &)> cfg, Build build,
                std::function<std::function<void(ctxptr, const std::vector<double> &, const call &, F &)>(typename std::invoke_result_t<Build, M &> &)> body,
                int purgeable = USECUT_FORCE, cpxlong mask = c::CPX_CALLBACKCONTEXT_RELAXATION, double tl = 10, std::function<std::string()> extra = {}) {
    M m; cfg(m);
    auto inst = build(m);
    m.set_time_limit(std::chrono::duration<double>(tl));
    F f{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables())};
    f.purgeable = purgeable;
    f.body = body(inst);
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, mask, fhook, &f));
    const double s = cp::timed_solve(m);
    report(n, f, m, s, extra ? extra() : "");
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    const auto gad = [](M & m) { return cp::build_gadgets(m, 10, false); };
    const auto ats = [](M & m) { return cp::build_atsp(m, 26); };
    const auto scope_root = [](M & m) { scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0); };

    // 1. the separator always has a violated cut: tighten by 1e-3 at every call, here on gadget 3
    using body_t = std::function<void(ctxptr, const std::vector<double> &, const call &, F &)>;
    for(int pg : {USECUT_FORCE, USECUT_PURGE, USECUT_FILTER}) for(bool in_tree : {false, true}) {
        run(std::format("1 endless supply of violated cuts purgeable={} {}", pg, in_tree ? "at the first depth-1 node" : "at the root"),
            scope_root, gad, [in_tree](auto & g) -> body_t {
                auto target = std::make_shared<cpxlong>(-1);
                return [g, in_tree, target](ctxptr ctx, const std::vector<double> & x, const call & k, F & f) {
                    if((k.depth == 0) == in_tree) return;
                    if(*target < 0) *target = k.uid;
                    if(k.uid != *target) return;
                    raw_cut r{{g.za[3], g.zb[3]}, {1.0, 1.0}, 'L', x[static_cast<std::size_t>(g.za[3])] + x[static_cast<std::size_t>(g.zb[3])] - 1e-3};
                    f.add(ctx, r);
                };
            }, pg);
    }
    // 2. same cut, not violated, every call: the node's loop must not spin
    run("2 same non-violated cut at every call", scope_root, gad, [](auto & g) -> body_t {
        return [g](ctxptr ctx, const std::vector<double> &, const call &, F & f) { f.add(ctx, raw_cut{{g.za[3]}, {1.0}, 'L', 1e6}); };
    });
    // 3. a violated cut added only in the post-loop call: is it honoured? is there a further call?
    {
        auto seen = std::make_shared<long>(0); auto err = std::make_shared<int>(0); auto viol_after = std::make_shared<long>(0); auto checks = std::make_shared<long>(0);
        for(bool add_in_after : {false, true}) {
            *seen = 0; *viol_after = 0; *checks = 0; *err = 0;
            auto cut = std::make_shared<std::optional<raw_cut>>();
            auto cutuid = std::make_shared<cpxlong>(-1);
            run(std::format("3 hull cut of gadget 3 added once, {}", add_in_after ? "ONLY from a post-loop call (AFTERCUTLOOP=1)" : "from an in-loop call"),
                scope_root, gad, [=](auto & g) -> body_t {
                    return [g, seen, err, viol_after, checks, cut, cutuid, add_in_after](ctxptr ctx, const std::vector<double> & x, const call & k, F & f) {
                        const auto h = g.hull_cut(3);
                        if(!*cut && h.violation(x) > 1e-4 && (k.after != 0) == add_in_after) { *cut = h; *cutuid = k.uid; *err = f.add(ctx, h); ++*seen; return; }
                        if(*cut) { ++*checks; *viol_after += h.violation(x) > 1e-6; }
                    };
                }, USECUT_FORCE, c::CPX_CALLBACKCONTEXT_RELAXATION, 10, [=] { return std::format("| added={} err={} later viol/checks={}/{}", *seen, *err, *viol_after, *checks); });
        }
    }
    // 4. does the hook see integral relaxations? count against candidates.
    {
        auto integral = std::make_shared<long>(0);
        run("4 gadgets scope: all y integral at a RELAXATION call?", scope_root, gad, [integral](auto & g) -> body_t {
            return [g, integral](ctxptr, const std::vector<double> & x, const call &, F &) {
                bool all = true;
                for(int j : g.y) { const double v = x[static_cast<std::size_t>(j)]; all = all && std::fabs(v - std::round(v)) < 1e-6; }
                *integral += all;
            };
        }, USECUT_FORCE, c::CPX_CALLBACKCONTEXT_RELAXATION | c::CPX_CALLBACKCONTEXT_CANDIDATE, 10, [integral] { return std::format("| calls_with_all_gadget_y_integral={}", *integral); });
        *integral = 0;
        run("4b atsp26 defaults: integral?", [](M & m) { param(m, P_THREADS, 1); }, ats, [integral](auto & a) -> body_t {
            return [a, integral](ctxptr, const std::vector<double> & x, const call &, F &) { *integral += a.integral(x); };
        }, USECUT_FORCE, c::CPX_CALLBACKCONTEXT_RELAXATION | c::CPX_CALLBACKCONTEXT_CANDIDATE, 20, [integral] { return std::format("| integral_calls={}", *integral); });
    }
    // 5. rounds on the ATSP: one DFJ cut per call, root rounds, with and without families
    for(int cutpass : {0, 1, 3}) {
        run(std::format("5 atsp26 one DFJ per call, only_user_cuts CUTPASS={}", cutpass), [cutpass](M & m) { only_user_cuts(m); param(m, P_CUTPASS, cutpass); }, ats, [](auto & a) -> body_t {
            return [a](ctxptr ctx, const std::vector<double> & x, const call &, F & f) { for(const auto & r : a.separate(x, 1)) f.add(ctx, r); };
        }, USECUT_PURGE);
    }
    // 6. limit parameters against user cuts
    for(auto [name, p, v] : std::initializer_list<std::tuple<const char *, int, int>>{{"EACHCUTLIM=0", P_EACHCUTLIM, 0}, {"Nodecuts=-1", P_NODECUTS, -1}, {"MIPSEARCH=2 dynamic", P_MIPSEARCH, 2}, {"MIPSEARCH=1 traditional", P_MIPSEARCH, 1}}) {
        const int pp = p, vv = v;
        run(std::format("6 atsp26 DFJ, defaults with {}", name), [pp, vv](M & m) { param(m, P_THREADS, 1); param(m, pp, vv); }, ats, [](auto & a) -> body_t {
            return [a](ctxptr ctx, const std::vector<double> & x, const call &, F & f) { for(const auto & r : a.separate(x)) f.add(ctx, r); };
        }, USECUT_PURGE);
    }
    {
        M probe; auto [env, lp] = probe.native_model();
        (void)lp;
        run("6 atsp26 DFJ, defaults with CUTSFACTOR=1.0", [](M & m) { param(m, P_THREADS, 1); auto [e, l] = m.native_model(); m.native_api()._check(e, m.native_api().setdblparam(e, P_CUTSFACTOR, 1.0)); }, ats, [](auto & a) -> body_t {
            return [a](ctxptr ctx, const std::vector<double> & x, const call &, F & f) { for(const auto & r : a.separate(x)) f.add(ctx, r); };
        }, USECUT_PURGE);
    }
    // 7. exitcutloop
    {
        auto exited = std::make_shared<long>(0), last = std::make_shared<long>(-99);
        run("7 CPXcallbackexitcutloop at every in-loop call, a cut in the same call is disregarded", scope_root, gad, [=](auto & g) -> body_t {
            return [g, exited](ctxptr ctx, const std::vector<double> &, const call & k, F & f) {
                if(k.after) return;
                f.add(ctx, g.hull_cut(3));
                if(f.s->exitcutloop(ctx) == 0) ++*exited;
            };
        }, USECUT_FORCE, c::CPX_CALLBACKCONTEXT_RELAXATION, 10, [=] { return std::format("| exits={}", *exited); });
    }
}
