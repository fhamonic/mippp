// C4 (root call), C8 (two calls per node, same LP), C2 (overlap by context).
#include "verify_cplex_common.hpp"
using namespace cp;
struct nodeinfo { int calls = 0, after = 0; std::vector<std::size_t> hashes; };
struct fctx {
    const vsyms * s; int cols; std::mutex mx;
    std::map<cxlong, nodeinfo> nodes;
    std::map<int, long> depth;
    long calls = 0, after_calls = 0;
    cxlong min_depth = 1000000;
    std::vector<double> prev; cxlong prev_uid = -1;
    long same_as_prev_on_after = 0, differ_on_after = 0, after_without_prev = 0;
    cp::separator sep; const cx::cplex_api * api;
};
static std::size_t hashv(const std::vector<double> & x) {
    std::size_t h = 0xcbf29ce484222325ull;
    for(double v : x) { auto b = std::bit_cast<std::uint64_t>(v); h ^= b; h *= 0x100000001b3ull; }
    return h;
}
static int fhook(ctxptr context, cxlong id, void * usr) {
    auto & h = *static_cast<fctx *>(usr);
    if(id != CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    try {
        std::vector<double> x(static_cast<std::size_t>(h.cols));
        if(h.s->point(context, x.data(), 0, h.cols - 1, nullptr)) return 0;
        cxlong depth = -1, uid = -1; int after = 0;
        h.s->infolong(context, CPXCALLBACKINFO_NODEDEPTH, &depth);
        h.s->infolong(context, CPXCALLBACKINFO_NODEUID, &uid);
        h.s->infoint(context, CPXCALLBACKINFO_AFTERCUTLOOP, &after);
        std::lock_guard l(h.mx);
        ++h.calls; h.depth[static_cast<int>(depth)]++;
        auto & n = h.nodes[uid]; ++n.calls; n.after += after; n.hashes.push_back(hashv(x));
        if(after) {
            ++h.after_calls;
            if(h.prev_uid != uid) ++h.after_without_prev;
            else {
                bool same = h.prev.size() == x.size();
                if(same) for(std::size_t j = 0; j < x.size(); ++j) if(std::fabs(x[j] - h.prev[j]) > 1e-12) { same = false; break; }
                (same ? h.same_as_prev_on_after : h.differ_on_after)++;
            }
        }
        h.prev = x; h.prev_uid = uid;
        if(h.sep) h.sep(x, [&](const raw_cut & c) { int b = 0, p = CPX_USECUT_PURGE, lo = 0; return h.api->callbackaddusercuts(context, 1, static_cast<int>(c.ind.size()), &c.rhs, &c.sense, &b, c.ind.data(), c.val.data(), &p, &lo); });
    } catch(...) { return 1; }
    return 0;
}
static std::vector<std::unique_ptr<fctx>> keep;
static fctx & attach(M & m, cp::separator sep = {}) {
    keep.push_back(std::make_unique<fctx>());
    auto & h = *keep.back();
    h.s = &vsyms::get(m); h.cols = static_cast<int>(m.num_variables()); h.sep = std::move(sep); h.api = &m.native_api();
    auto [env, lp] = m.native_model();
    m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, CPX_CALLBACKCONTEXT_RELAXATION, fhook, &h));
    return h;
}
static std::string depth_line(const fctx & h) {
    long d0 = h.depth.count(0) ? h.depth.at(0) : 0;
    int first = h.depth.empty() ? -1 : h.depth.begin()->first;
    return std::format("calls={} depth0_calls={} first_depth={} after_cutloop={} same_LP_as_prev={} differ={} after_without_prev={}", h.calls, d0, first, h.after_calls, h.same_as_prev_on_after, h.differ_on_after, h.after_without_prev);
}
using sett = std::function<void(M &)>;
int main(int, char ** argv) {
    { M m; print_version(m); }
    using std::chrono::duration;
    const std::string what = argv[1] ? argv[1] : "c4";
    if(what == "c4") {
        struct inst { const char * name; std::function<void(M &)> build; };
        const std::vector<inst> insts = {
            {"atsp26s5", [](M & m) { build_atsp(m, 26, false, 5); }},
            {"atsp31s5", [](M & m) { build_atsp(m, 31, false, 5); }},
            {"mkp5x60", [](M & m) { build_mkp(m, 5, 60, 3); }},
            {"mkp30x100", [](M & m) { build_mkp(m, 30, 100, 7); }},
            {"gadgets10", [](M & m) { build_gadgets(m, 10, false); }},
        };
        std::vector<std::pair<std::string, sett>> setts = {
            {"defaults", [](M &) {}},
            {"all-families-off", [](M & m) { all_families_off(m); }},
            {"CUTPASS=-1", [](M & m) { vparam(m, CPX_PARAM_CUTPASS, -1); }},
            {"CUTPASS=1", [](M & m) { vparam(m, CPX_PARAM_CUTPASS, 1); }},
            {"families-off+CUTPASS=5", [](M & m) { all_families_off(m); vparam(m, CPX_PARAM_CUTPASS, 5); }},
            {"families-off-presolve-off", [](M & m) { all_families_off(m); vparam(m, CPX_PARAM_PREIND, 0); }},
            {"defaults-presolve-off", [](M & m) { vparam(m, CPX_PARAM_PREIND, 0); }},
            {"defaults-REDUCE=0", [](M & m) { vparam(m, CPX_PARAM_REDUCE, 0); }},
            {"defaults-EACHCUTLIM=0", [](M & m) { vparam(m, CPX_PARAM_EACHCUTLIM, 0); }},
            {"defaults-CUTSFACTOR=1", [](M & m) { vparamd(m, CPX_PARAM_CUTSFACTOR, 1.0); }},
        };
        const std::pair<const char *, int> fams[] = {{"Cliques", CPX_PARAM_CLIQUES}, {"Covers", CPX_PARAM_COVERS}, {"Disj", CPX_PARAM_DISJCUTS}, {"FlowCovers", CPX_PARAM_FLOWCOVERS}, {"FlowPaths", CPX_PARAM_FLOWPATHS}, {"Gomory", CPX_PARAM_FRACCUTS}, {"GUB", CPX_PARAM_GUBCOVERS}, {"Implied", CPX_PARAM_IMPLBD}, {"LiftProj", CPX_PARAM_LANDPCUTS}, {"MCF", CPX_PARAM_MCFCUTS}, {"MIR", CPX_PARAM_MIRCUTS}, {"ZeroHalf", CPX_PARAM_ZEROHALFCUTS}, {"LocalImpl", CPX_PARAM_LOCALIMPLBD}, {"BQP", CPX_PARAM_BQPCUTS}, {"RLT", CPX_PARAM_RLTCUTS}};
        for(const auto & [fname, p] : fams) { const int pp = p; setts.push_back({std::format("only-{}", fname), [pp](M & m) { all_families_off(m); vparam(m, pp, 2); }}); }
        for(const auto & in : insts) for(const auto & [sname, st] : setts) {
            M m; m.set_thread_limit(1); in.build(m); st(m); m.set_time_limit(duration<double>(3.0));
            auto & h = attach(m);
            const double t = timed_solve(m);
            std::println(stderr, "C4 {} {}: {} | {}", in.name, sname, summary(m, t), depth_line(h));
        }
    } else if(what == "c8") {
        // no cut: per node, how many calls and how many on the same LP
        for(const auto & [iname, build] : std::initializer_list<std::pair<const char *, sett>>{
                {"atsp26s5", [](M & m) { build_atsp(m, 26, false, 5); }},
                {"mkp5x60", [](M & m) { build_mkp(m, 5, 60, 3); }},
                {"mkp30x100-4s", [](M & m) { build_mkp(m, 30, 100, 7); }}}) {
            M m; m.set_thread_limit(1); build(m); m.set_time_limit(duration<double>(4.0));
            auto & h = attach(m);
            const double t = timed_solve(m);
            std::map<int, long> per; long two_same = 0, two_diff = 0;
            for(const auto & [u, n] : h.nodes) { per[n.calls]++; if(n.calls == 2) (n.hashes[0] == n.hashes[1] ? two_same : two_diff)++; }
            std::string dist; for(auto [k, v] : per) dist += std::format(" {}calls:{}", k, v);
            std::println(stderr, "C8 {} no cut: {} | {} | nodes_seen={} dist:{} two_calls_identical_bits={} two_calls_diff={}", iname, summary(m, t), depth_line(h), h.nodes.size(), dist, two_same, two_diff);
        }
        // with DFJ: does the post-loop call differ?
        for(int seed : {5, 6, 7}) {
            M m; m.set_thread_limit(1); auto a = build_atsp(m, 31, false, static_cast<unsigned>(seed)); m.set_time_limit(duration<double>(20.0));
            auto & h = attach(m, [a](const std::vector<double> & x, const add_fn & add) { for(const auto & c : a.separate(x)) add(c); });
            const double t = timed_solve(m);
            std::println(stderr, "C8 atsp31 DFJ seed={}: {} | {} nodes_seen={}", seed, summary(m, t), depth_line(h), h.nodes.size());
        }
    }
    return 0;
}
