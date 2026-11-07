// The repo's GLPK test file, unedited, with glpk_milp replaced by a class taking the recipe's
// path when VG_RECIPE=1: presolve off, root LP by the wrapper, a no-op separator at ICUTGEN.
// With VG_RECIPE=0 the same binary takes the wrapper's own solve(): the reference.
#include <cstdlib>
#include <cstdio>
#include <dlfcn.h>
#include "mippp/solvers/glpk/all.hpp"

struct vg_tally { long solves = 0, recipe = 0, root_fail = 0, empties = 0, fires = 0; ~vg_tally() { std::fprintf(stderr, "VG tally: solves=%ld recipe_path=%ld root_fail=%ld empty_shortcut=%ld icutgen_fires=%ld\n", solves, recipe, root_fail, empties, fires); } };
static vg_tally T;
namespace mippp::glpk::impl::v1 {
class vg_recipe_milp : public glpk_milp {
    using Scale = void (*)(glp_prob *, int);
    using Adv = void (*)(glp_prob *, int);
    static inline const bool recipe = [] { const char * e = std::getenv("VG_RECIPE"); return e && e[0] == '1'; }();
    static inline const bool stale_fix = [] { const char * e = std::getenv("VG_NOFIX"); return !e; }();
    static inline const bool sub_time = [] { const char * e = std::getenv("VG_NOSUB"); return !e; }();
    static void dispatch(glp_tree * t, void * u) {
        auto * self = static_cast<vg_recipe_milp *>(u);
        const int why = self->glp->ios_reason(t);
        if(why == 4) {  // GLP_ICUTGEN: read the whole point as a separator would
            glp_prob * P = self->glp->ios_get_prob(t);
            const int n = self->glp->get_num_cols(P);
            for(int j = 1; j <= n; ++j) self->sink += self->glp->get_col_prim(P, j);
            ++self->fired; ++T.fires;
        }
        if(self->progress_callback) progress_callback_func(t, static_cast<glpk_milp *>(self));
    }
    double sink = 0;
public:
    long fired = 0;
    using glpk_milp::glpk_milp;
    void solve() {
        ++T.solves;
        if(!recipe) { glpk_milp::solve(); return; }
        reset_status();
        const auto saved = _round_integral_sides();
        const bool quiet = !is_verbose();
        const int term_out = quiet ? glp->term_out(GLP_OFF) : GLP_ON;
        model_params.cb_func = dispatch;
        model_params.cb_info = this;
        const int presolve = model_params.presolve, tm = model_params.tm_lim;
        static const bool no_empty = std::getenv("VG_NOEMPTY") != nullptr;
        const bool off = no_empty || glp->get_num_cols(model) != 0;
        if(off) ++T.recipe; else ++T.empties;
        int ret = 0;
        bool at_root = false;
        if(off) {
            static const auto lib = glp->library_path();
            void * h = dlopen(lib.c_str(), RTLD_NOW | RTLD_NOLOAD);
            static const auto scale = reinterpret_cast<Scale>(dlsym(h, "glp_scale_prob"));
            static const auto adv = reinterpret_cast<Adv>(dlsym(h, "glp_adv_basis"));
            model_params.presolve = GLP_OFF;
            scale(model, 0x01 | 0x10 | 0x20 | 0x40);
            adv(model, 0);
            glp_smcp s;
            glp->init_smcp(&s);
            s.msg_lev = GLP_MSG_OFF;
            s.tm_lim = tm;
            const auto t0 = std::chrono::steady_clock::now();
            const int r = glp->simplex(model, &s);
            const double used = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if(r != 0) ret = r;
            else {
                const int st = glp->get_status(model);
                ret = st == GLP_OPT ? 0 : st == GLP_NOFEAS ? GLP_ENOPFS : st == GLP_UNBND ? GLP_ENODFS : GLP_EFAIL;
            }
            at_root = ret != 0;
            if(at_root) ++T.root_fail;
            if(ret == 0 && sub_time && tm != std::numeric_limits<int>::max()) model_params.tm_lim = std::max(tm - static_cast<int>(used * 1000), 0);
        }
        if(ret == 0) ret = glp->intopt(model, &model_params);
        model_params.presolve = presolve; model_params.tm_lim = tm;
        const bool crossed = (ret == GLP_EBOUND) && _has_crossed_bounds();
        _restore_sides(saved);
        if(quiet) glp->term_out(term_out);
        if(crossed) { _status = status::infeasible{}; _mark_results_valid(); return; }
        _status = _intopt_status(ret, at_root && stale_fix ? GLP_UNDEF : glp->mip_status(model));
        _mark_results_valid();
    }
};
}  // namespace mippp::glpk::impl::v1
namespace mippp { using glpk::impl::v1::vg_recipe_milp; }
#define glpk_milp vg_recipe_milp
#include "solvers/glpk.cpp"
