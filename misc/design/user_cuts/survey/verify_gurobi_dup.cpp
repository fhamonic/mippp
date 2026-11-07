// Is a duplicate column in GRBcbcut only "last wins", or can it corrupt memory?
// argv: seed pattern precrush lazy. Pure native: no sanitize, no std::function around the call.
#include "verify_gurobi_common.hpp"
static const g::gurobi_api * API;
struct dctx { const cgm * G; int cols; int pattern; bool lazy; long calls = 0, cuts = 0, errs = 0; int first_err = 0; };
// Pattern >= 100: one column repeated (pattern - 100 + cols) times, at the first call only, from where W.
static int rep_hook(g::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<dctx *>(usr);
    const int W = c.lazy ? W_MIPSOL : W_MIPNODE;
    if(where != W || c.calls > 0) return 0;
    if(where == W_MIPNODE) { int status = 0; API->cbget(cbdata, where, NODE_STATUS, &status); if(status != ST_OPTIMAL) return 0; }
    ++c.calls;
    const int n = c.cols + c.pattern - 100;
    std::vector<int> ind(static_cast<size_t>(n), c.G->x[0]);
    std::vector<double> val(static_cast<size_t>(n), 1.0);
    const double rhs = 2.0 * n;
    const int e = c.lazy ? API->cblazy(cbdata, n, ind.data(), val.data(), '<', rhs) : API->cbcut(cbdata, n, ind.data(), val.data(), '<', rhs);
    if(e) { ++c.errs; if(!c.first_err) c.first_err = e; } else ++c.cuts;
    return 0;
}
static int dup_hook(g::GRBmodel *, void * cbdata, int where, void * usr) {
    if(where != W_MIPNODE) return 0;
    auto & c = *static_cast<dctx *>(usr);
    int status = 0; API->cbget(cbdata, where, NODE_STATUS, &status);
    if(status != ST_OPTIMAL) return 0;
    ++c.calls;
    std::vector<double> x(static_cast<size_t>(c.cols));
    if(API->cbget(cbdata, where, NODE_REL, x.data())) return 0;
    for(const auto & k : covers(*c.G, x)) {
        std::vector<int> ind; std::vector<double> val;
        for(size_t t = 0; t < k.ind.size(); ++t) {
            if(c.pattern == 0) { ind.push_back(k.ind[t]); val.push_back(1); }                                // control: no duplicates
            else if(c.pattern == 1) { ind.push_back(k.ind[t]); val.push_back(-1); ind.push_back(k.ind[t]); val.push_back(2); }   // every term twice, adjacent
            else if(c.pattern == 2) { ind.push_back(k.ind[t]); val.push_back(1); if(t == 0) { ind.push_back(k.ind[t]); val.push_back(1); } }  // one duplicate only
            else if(c.pattern == 3) { for(int r = 0; r < 4; ++r) { ind.push_back(k.ind[t]); val.push_back(0.25); } }           // four times each
            else if(c.pattern == 4) { ind.push_back(k.ind[t]); val.push_back(1); }
        }
        if(c.pattern == 4) { const size_t n = ind.size(); for(size_t t = 0; t < n; ++t) { ind.push_back(ind[t]); val.push_back(val[t]); } }  // whole cut twice, non-adjacent
        const int e = c.lazy ? API->cblazy(cbdata, static_cast<int>(ind.size()), ind.data(), val.data(), '<', k.rhs)
                             : API->cbcut(cbdata, static_cast<int>(ind.size()), ind.data(), val.data(), '<', k.rhs);
        if(e) { ++c.errs; if(!c.first_err) c.first_err = e; } else ++c.cuts;
    }
    return 0;
}
int main(int argc, char ** argv) {
    const unsigned seed = static_cast<unsigned>(std::atoi(argv[1]));
    const int pattern = std::atoi(argv[2]), precrush = std::atoi(argv[3]), lazy = argc > 4 ? std::atoi(argv[4]) : 0;
    const int cols = argc > 5 ? std::atoi(argv[5]) : 60, rows = argc > 6 ? std::atoi(argv[6]) : 10;
    const cgm G = make_cg(seed, static_cast<size_t>(cols), static_cast<size_t>(rows));
    M b; seti(b, "OutputFlag", 0); seti(b, "Threads", 1); setd(b, "TimeLimit", 20); G.r.load(b); const sol s0 = solve_native(b, false);
    M m; seti(m, "OutputFlag", 0); seti(m, "Threads", 1); setd(m, "TimeLimit", 20); seti(m, "PreCrush", precrush); if(lazy) seti(m, "LazyConstraints", 1); G.r.load(m);
    API = &m.native_api();
    dctx c{&G, num_vars(m), pattern, lazy != 0};
    auto [env, model] = m.native_model();
    m.native_api().setcallbackfunc(model, pattern >= 100 ? rep_hook : dup_hook, &c);
    const sol s = solve_native(m, false);
    std::println(stderr, "seed={} pattern={} precrush={} lazy={} | baseline {} {} | {} | calls={} cuts={} errs={}(first {})", seed, pattern, precrush, lazy, sname(s0.status), s0.value, line(s), c.calls, c.cuts, c.errs, c.first_err);
    return 0;
}
