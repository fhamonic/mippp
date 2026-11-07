// The Gurobi recipe under the shared conformance set: PreCrush=1, GRBcbcut at MIPNODE.
#include "mippp/solvers/gurobi/all.hpp"
#include "cut_conformance.hpp"
using M = mippp::gurobi_milp;
namespace g = mippp::gurobi::impl::v1;
enum { W_MIPNODE = 5, NODE_STATUS = 5001, NODE_REL = 5002, GRB_OPTIMAL_ = 2 };
struct ctx { cp::separator sep; const g::gurobi_api * api; int cols; bool sanitize; };
static std::vector<std::unique_ptr<ctx>> keep;
static int hook(g::GRBmodel *, void * cbdata, int where, void * usr) {
    auto & c = *static_cast<ctx *>(usr);
    if(where != W_MIPNODE) return 0;
    int status = 0; c.api->cbget(cbdata, where, NODE_STATUS, &status);
    if(status != GRB_OPTIMAL_) return 0;
    std::vector<double> x(static_cast<std::size_t>(c.cols));
    c.api->cbget(cbdata, where, NODE_REL, x.data());
    c.sep(x, [&](const cp::raw_cut & raw) {
        const auto r = cf::sanitize(raw, x);
        if(!r) return 0;
        return c.api->cbcut(cbdata, static_cast<int>(r->ind.size()), r->ind.data(), r->val.data(), r->sense == 'E' ? '=' : r->sense == 'L' ? '<' : '>', r->rhs);
    });
    return 0;
}
int main() {
    cf::setup<M> B;
    B.attach = [](M & m, cp::separator sep) {
        auto [env, model] = m.native_model();
        auto & api = m.native_api();
        api._check(env, api.setintparam(env, "PreCrush", 1));
        keep.push_back(std::make_unique<ctx>(ctx{std::move(sep), &api, static_cast<int>(m.num_variables()), true}));
        api._check(env, api.setcallbackfunc(model, hook, keep.back().get()));
    };
    B.one_thread = [](M & m) { m.set_thread_limit(1); };
    cf::run(B);
    return cf::finish();
}
