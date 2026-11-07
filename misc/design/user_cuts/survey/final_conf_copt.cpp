// final: cf::sanitize replaced by fp::prepare. argv[1]: conf (default), sweep, sweep_ns, bigm, tiny, harm.
// The COPT recipe under the shared set: mask MIPRELAX|MIPNODE, RELAXSOLUTION, cf::sanitize,
// AddCallbackUserCut. The shipped callback_func handles MIPNODE, as in a wrapper.
// NEG=none|no_fold|no_strip|no_empty|no_filter|no_node removes one piece.
// DELAY=d: the separator sleeps for its first d MIPRELAX calls, so that the search reaches
// the tree before cuts arrive (DFJ otherwise closes every ATSP at or near the root).
// SIZES=a,b,c and FUZZ=k override setup sizes.
#include "verify_copt_common.hpp"
#include "final_prepare.hpp"
#include "final_tests.hpp"
static std::string NEG = "none";
static long DELAY = 0;
static std::optional<cp::raw_cut> neg_sanitize(const cp::raw_cut & in, const std::vector<double> & x) {
    cp::raw_cut out{{}, {}, in.sense, in.rhs};
    if(NEG == "no_fold") { for(std::size_t k = 0; k < in.ind.size(); ++k) if(in.val[k] != 0.0) { out.ind.push_back(in.ind[k]); out.val.push_back(in.val[k]); } }
    else {
        std::map<int, double> f;
        for(std::size_t k = 0; k < in.ind.size(); ++k) f[in.ind[k]] += in.val[k];
        for(auto [j, v] : f) if(NEG == "no_strip" || v != 0.0) { out.ind.push_back(j); out.val.push_back(v); }
    }
    if(NEG != "no_empty" && out.ind.empty()) return std::nullopt;
    if(NEG != "no_filter" && NEG != "no_empty" && out.violation(x) <= 1e-9) return std::nullopt;
    return out;
}
static std::shared_ptr<recipe> last_rec;
int main(int argc, char ** argv) {
    if(const char * e = std::getenv("NEG")) NEG = e;
    if(const char * e = std::getenv("DELAY")) DELAY = std::atol(e);
    const std::string mode = argc > 1 ? argv[1] : "conf";
    cf::setup<V> B;
    B.attach = [](V & m, cp::separator sep) {
        recipe_opts o; o.fold = o.strip_zero = o.skip_empty = o.drop_satisfied = false;
        if(NEG == "no_node") o.mask_node = false;
        auto calls = std::make_shared<std::atomic<long>>(0);
        m.attach([sep = std::move(sep), calls](const std::vector<double> & x, const cp::add_fn & add) {
            if(calls->fetch_add(1) < DELAY) return;
            sep(x, [&](const cp::raw_cut & raw) -> int {
                int e = 0;
                for(const auto & r : fp::prepare(raw, x)) { e = add(r); if(e) break; }
                return e;
            });
        }, o);
        last_rec = m.rec;
    };
    B.one_thread = [](V & m) { m.set_thread_limit(1); };
    if(mode != "conf") {
        ft::bed<V> E;
        E.name = "copt";
        E.attach = B.attach;
        E.one_thread = B.one_thread;
        E.presolve_off = [](V & m) { m.iparam("Presolve", 0); };
        E.recipe_line = [] { return last_rec ? last_rec->line() : std::string("no recipe"); };
        if(!ft::run_mode(E, mode)) { std::println(stderr, "unknown mode {}", mode); return 2; }
        return 0;
    }
    B.limit = 60;
    B.stop_size = 26;
    B.first_limit = 0.02;
    if(const char * e = std::getenv("SIZES")) { B.atsp_sizes.clear(); std::string s(e); std::size_t p = 0; while(p < s.size()) { std::size_t q = s.find(',', p); if(q == std::string::npos) q = s.size(); B.atsp_sizes.push_back(std::atoi(s.substr(p, q - p).c_str())); p = q + 1; } }
    if(const char * e = std::getenv("FUZZ")) B.fuzz_seeds = std::atoi(e);
    std::println(stderr, "NEG={} DELAY={} sizes front={} back={} limit={} stop_size={} first_limit={} fuzz={}", NEG, DELAY, B.atsp_sizes.front(), B.atsp_sizes.back(), B.limit, B.stop_size, B.first_limit, B.fuzz_seeds);
    cf::run(B);
    return cf::finish();
}
