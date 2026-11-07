#include "mippp/solvers/gurobi/all.hpp"
#include "cut_common.hpp"
using M = mippp::gurobi_milp;
namespace g = mippp::gurobi::impl::v1;

// gurobi_c.h 12.0.1
enum { W_MIPNODE = 5, NODE_STATUS = 5001, NODE_REL = 5002, GRB_OPTIMAL_ = 2 };
struct ctx {
    cp::separator sep;
    const g::gurobi_api * api;
    int cols;
    int (*cbcut)(void *, int, const int *, const double *, char, double);
    int (*cbget)(void *, int, int, void *);
};
static std::vector<std::unique_ptr<ctx>> keep;

static int hook(g::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<ctx *>(usr);
    if(where != W_MIPNODE) return 0;
    int status = 0;
    c.api->cbget(cbdata, where, NODE_STATUS, &status);
    if(status != GRB_OPTIMAL_) return 0;
    std::vector<double> x(static_cast<std::size_t>(c.cols));
    c.api->cbget(cbdata, where, NODE_REL, x.data());
    c.sep(x, [&](const cp::raw_cut & r) {
        return c.api->cbcut(cbdata, static_cast<int>(r.ind.size()), r.ind.data(), r.val.data(),
                            r.sense == 'E' ? '=' : r.sense == 'L' ? '<' : '>', r.rhs);
    });
    return 0;
}
static void attach(M & m, cp::separator sep, const std::string &) {
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    api._check(env, api.setintparam(env, "PreCrush", 1));
    keep.push_back(std::make_unique<ctx>(ctx{std::move(sep), &api, static_cast<int>(m.num_variables())}));
    api._check(env, api.setcallbackfunc(model, hook, keep.back().get()));
}
static void one_thread(M & m) { m.set_thread_limit(1); }
static void scope_settings(M & m) {
    auto [env, model] = m.native_model();
    auto & api = m.native_api();
    for(auto [name, v] : {std::pair{"Presolve", 0}, {"Cuts", 0}, {"Heuristics", 0}, {"Symmetry", 0}, {"Disconnected", 0}, {"Threads", 1}})
        if(std::string(name) == "Heuristics") api._check(env, api.setdblparam(env, name, 0.0));
        else api._check(env, api.setintparam(env, name, v));
}
int main() {
    cp::backend<M> B;
    B.attach = attach; B.scope_settings = scope_settings; B.one_thread = one_thread;
    B.cut_modes = {"global"};
    cp::run_matrix(B);
}
