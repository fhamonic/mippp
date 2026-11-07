// The SCIP recipe under the shared conformance set. Controls: VERIFY_DROP=<piece>.
// Sizes: CF_SIZES="a,b,c", CF_STOP=n, CF_FIRST=seconds, CF_LIMIT=seconds, CF_FUZZ=n.
// final: fp::prepare + the tiny-term step. argv[1]: conf (default), sweep, sweep_ns, bigm, tiny, harm.
#include "final_scip_native.hpp"
#include "final_tests.hpp"
using vs::V;
static std::vector<int> ints(const char * e) {
    std::vector<int> v; std::string t(e); std::size_t p = 0;
    while(p <= t.size()) { auto q = t.find(',', p); if(q == std::string::npos) q = t.size(); v.push_back(std::stoi(t.substr(p, q - p))); p = q + 1; }
    return v;
}
static vs::plugin * last_plugin = nullptr;
int main(int argc, char ** argv) {
    const std::string mode = argc > 1 ? argv[1] : "conf";
    if(mode != "conf") {
        { V m; std::println(stderr, "library {} SCIP {}", m.native_api().library_path().string(), m.version()); }
        ft::bed<V> E;
        E.name = "scip";
        E.attach = [](V & m, cp::separator sep) { m.attach(std::move(sep)); last_plugin = m.P.get(); };
        E.one_thread = [](V &) {};
        E.presolve_off = [](V & m) { m.seti("presolving/maxrounds", 0); };
        E.recipe_line = [] { auto & p = *last_plugin; return std::format("scip(rows={} pool={} sanitized={} tiny_moved={} tiny_unviolated={} add_errors={} deep_rows={} infeasible={})", p.rows, p.pool_adds, p.sanitized, p.tiny_moved, p.tiny_unviolated, p.add_errors, p.deep_rows, p.infeasible_flags); };
        if(!ft::run_mode(E, mode)) { std::println(stderr, "unknown mode {}", mode); return 2; }
        return 0;
    }
    { V m; std::println(stderr, "library {} SCIP {} drop={}", m.native_api().library_path().string(), m.version(), std::getenv("VERIFY_DROP") ? std::getenv("VERIFY_DROP") : "none"); }
    cf::setup<V> B;
    // CF_DEEP=1: no cut at the root, so that instances DFJ closes at the root keep a tree
    static const bool deep = std::getenv("CF_DEEP") != nullptr;
    B.attach = [](V & m, cp::separator sep) {
        if(deep) {
            vs::plugin * pl = m.P.get();
            m.attach([sep = std::move(sep), pl](const std::vector<double> & x, const cp::add_fn & add) {
                sep(x, pl->n->getDepth(pl->s) == 0 ? cp::add_fn([](const cp::raw_cut &) { return 0; }) : add);
            });
        } else m.attach(std::move(sep));
    };
    B.one_thread = [](V &) {};
    if(const char * e = std::getenv("CF_SIZES")) B.atsp_sizes = ints(e);
    if(const char * e = std::getenv("CF_STOP")) B.stop_size = std::stoi(e);
    if(const char * e = std::getenv("CF_FIRST")) B.first_limit = std::stod(e);
    if(const char * e = std::getenv("CF_LIMIT")) B.limit = std::stod(e);
    if(const char * e = std::getenv("CF_FUZZ")) B.fuzz_seeds = std::stoi(e);
    cf::run(B);
    return cf::finish();
}
