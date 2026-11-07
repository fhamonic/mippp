#include "mippp/solvers/gurobi/all.hpp"
#include "probe_common.hpp"
using M = mippp::gurobi_milp;
using L = mippp::gurobi_lp;

// gurobi_c.h 12.0.1
enum { W_POLL = 0, W_PRE = 1, W_SPX = 2, W_MIP = 3, W_MIPSOL = 4, W_MIPNODE = 5, W_MSG = 6, W_BAR = 7 };
enum { MIP_OBJBST = 3000, MIP_OBJBND = 3001, MIP_NODCNT = 3002,
       SOL_OBJBST = 4003, SOL_OBJBND = 4004, SOL_NODCNT = 4005,
       NODE_OBJBST = 5003, NODE_OBJBND = 5004, NODE_NODCNT = 5005,
       RUNTIME = 6002, SPX_ITRCNT = 2000, SPX_OBJVAL = 2001, BAR_ITRCNT = 7001, BAR_PRIMOBJ = 7002 };

struct ctx {
    recorder * r;
    const mippp::gurobi::impl::v1::gurobi_api * api;
    int (*terminate)(void *);
    bool nonzero_return = false;
};
static std::vector<std::unique_ptr<ctx>> keep;

static int cb(mippp::gurobi::impl::v1::GRBmodel * model, void * cbdata, int where, void * usr) {
    auto * c = static_cast<ctx *>(usr);
    auto & G = *c->api;
    double p, d, n;
    switch(where) {
        case W_MIP:
            G.cbget(cbdata, where, MIP_OBJBST, &p);
            G.cbget(cbdata, where, MIP_OBJBND, &d);
            G.cbget(cbdata, where, MIP_NODCNT, &n);
            break;
        case W_MIPSOL:
            G.cbget(cbdata, where, SOL_OBJBST, &p);
            G.cbget(cbdata, where, SOL_OBJBND, &d);
            G.cbget(cbdata, where, SOL_NODCNT, &n);
            break;
        case W_MIPNODE:
            G.cbget(cbdata, where, NODE_OBJBST, &p);
            G.cbget(cbdata, where, NODE_OBJBND, &d);
            G.cbget(cbdata, where, NODE_NODCNT, &n);
            break;
        case W_POLL: c->r->count("POLLING"); return 0;
        case W_PRE: c->r->count("PRESOLVE"); return 0;
        case W_SPX: c->r->count("SIMPLEX"); return 0;
        case W_MSG: c->r->count("MESSAGE"); return 0;
        case W_BAR: c->r->count("BARRIER"); return 0;
        default: c->r->count("other"); return 0;
    }
    static const char * names[] = {"", "", "", "MIP", "MIPSOL", "MIPNODE"};
    if(c->r->record(names[where], p, d, (long long)n)) {
        if(c->nonzero_return) return 1;
        c->terminate(model);
    }
    return 0;
}
template <class X>
static void attach(X & m, recorder & r, bool nonzero = false) {
    auto [env, model] = m.native_model();
    keep.push_back(std::make_unique<ctx>(ctx{&r, &m.native_api(),
        symf<int (*)(void *)>(m.native_api().library_path(), "GRBterminate"), nonzero}));
    m.native_api()._check(env, m.native_api().setcallbackfunc(model, cb, keep.back().get()));
}

struct lp_ctx { recorder * r; const mippp::gurobi::impl::v1::gurobi_api * api; };
static int lp_cb(mippp::gurobi::impl::v1::GRBmodel *, void * cbdata, int where, void * usr) {
    auto * c = static_cast<lp_ctx *>(usr);
    double it = 0, obj = 0;
    if(where == W_SPX) {
        c->api->cbget(cbdata, where, SPX_ITRCNT, &it);
        c->api->cbget(cbdata, where, SPX_OBJVAL, &obj);
        c->r->record("SIMPLEX", obj, NAN, (long long)it);
    } else if(where == W_BAR) {
        int iti = 0;
        c->api->cbget(cbdata, where, BAR_ITRCNT, &iti);
        c->api->cbget(cbdata, where, BAR_PRIMOBJ, &obj);
        c->r->record("BARRIER", obj, NAN, iti);
    } else if(where == W_MSG) c->r->count("MESSAGE");
    else if(where == W_POLL) c->r->count("POLLING");
    else if(where == W_PRE) c->r->count("PRESOLVE");
    return 0;
}

int main(int argc, char ** argv) {
    std::string what = argc > 1 ? argv[1] : "all";
    backend<M> b{.name = "gurobi",
                 .attach = [](M & m, recorder & r) { attach(m, r); },
                 .one_thread = [](M & m) { m.set_thread_limit(1); }};
    if(what == "calib") {
        for(auto [rows, cols] : std::vector<std::pair<int,int>>{{5,40},{5,60},{10,60},{10,100},{30,100}}) {
            M m; build_mkp(m, make_mkp(rows, cols), +1, 100.0);
            m.set_time_limit(std::chrono::duration<double>(30));
            auto o = timed_solve(m);
            final_line(std::format("calib {}x{}", rows, cols), m, o.solve_s);
        }
        return 0;
    }
    run_milp_matrix(b);
    if(std::getenv("PROBE_QUICK")) return 0;
    {   // non-zero return instead of GRBterminate
        M m; build_mkp(m, make_mkp(30, 100), +1, 100.0);
        m.set_time_limit(std::chrono::duration<double>(10));
        recorder r; r.stop_after = 3; attach(m, r, true); r.start();
        try {
            auto o = timed_solve(m);
            final_line("H3 stop by a non-zero callback return", m, o.solve_s);
        } catch(const std::exception & e) {
            std::println(stderr, "== H3 non-zero return: solve() threw: {}", e.what());
        }
    }
    {   // LP
        L m; build_lp(m, 12000, 18000);
        m.set_time_limit(std::chrono::duration<double>(60));
        recorder r; lp_ctx c{&r, &m.native_api()};
        auto [env, model] = m.native_model();
        m.native_api()._check(env, m.native_api().setcallbackfunc(model, lp_cb, &c));
        r.start();
        auto o = timed_solve(m);
        final_line("L LP on gurobi_lp, hook", m, o.solve_s);
        hook_report(r, -1, o.solve_s);
    }
    run_slices<M>(30, 100);
}
