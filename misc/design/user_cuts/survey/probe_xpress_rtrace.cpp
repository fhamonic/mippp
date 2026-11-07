// Two traces of what a restart does to a cut.
// dual: the block instance of probe_xpress_extra presolverow, w >= 0.5 added once, firing by firing.
// s1: S1 instance, cut valid in the worse subtree only, then XPRStriggerrestart: does a local cut leak?
#include "probe_xpress_hook.hpp"
struct block { int t, s, w, v, tp; };
static block add_block(M & m) {
    auto b = m.add_binary_variables(3);
    auto c = m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
    block k{m.native_id(b(std::size_t{0})), m.native_id(b(std::size_t{1})), m.native_id(c(std::size_t{0})), m.native_id(c(std::size_t{1})), m.native_id(b(std::size_t{2}))};
    m.add_constraint(1.0 * b(std::size_t{0}) + 1.0 * b(std::size_t{1}) <= 1);
    m.add_constraint(1.0 * c(std::size_t{1}) + 1.0 * b(std::size_t{2}) == 1);
    m.set_objective_coefficient(b(std::size_t{0}), -10);
    m.set_objective_coefficient(b(std::size_t{1}), -10);
    m.set_objective_coefficient(c(std::size_t{0}), 3);
    m.set_objective_coefficient(c(std::size_t{1}), 0);
    m.set_objective_coefficient(b(std::size_t{2}), 1);
    return k;
}
struct tc { const x::xpress_api * api; std::shared_ptr<xp_fns> fn; int cols, watch; cp::raw_cut cut; std::string mode; bool added = false; long lines = 0, maxl = 40;
    cp::gadgets<M> * g = nullptr; long poked = 0; std::function<int(const std::vector<double> &)> trigger_state; int trigger_value = -2; int (*trigger)(prob_t, int *) = nullptr; bool restarted = false; int trig_status = -9; long sib_after = 0, sib_viol = 0, same_after = 0, same_viol = 0; int worse = 0; };
static int add(tc & c, prob_t cb) {
    const int start[2] = {0, static_cast<int>(c.cut.ind.size())};
    if(c.mode.starts_with("managed")) return c.fn->addmanaged(cb, c.mode == "managed_global", 1, &c.cut.sense, &c.cut.rhs, start, c.cut.ind.data(), c.cut.val.data());
    int cols = 0, n = 0, status = 0; double prhs = 0;
    c.api->getintattrib(cb, A_COLS, &cols);
    std::vector<int> pi(static_cast<std::size_t>(cols)); std::vector<double> pv(static_cast<std::size_t>(cols));
    int e = c.fn->presolverow(cb, c.cut.sense, start[1], c.cut.ind.data(), c.cut.val.data(), c.cut.rhs, cols, &n, pi.data(), pv.data(), &prhs, &status);
    if(e) return e;
    if(status < 0) return 9000 - status;
    const int s2[2] = {0, n}, type = 1;
    return c.api->addcuts(cb, 1, &type, &c.cut.sense, &prhs, s2, pi.data(), pv.data());
}
static void body(tc & c, prob_t cb, int * p_action, const char * who) {
    int node = 0, depth = 0, rounds = 0, cuts = 0, rows = 0, cols = 0, restarts = 0, tree = 0, lp = 0;
    c.api->getintattrib(cb, A_CURRENTNODE, &node); c.api->getintattrib(cb, A_NODEDEPTH, &depth); c.api->getintattrib(cb, A_CUTROUNDS, &rounds);
    c.api->getintattrib(cb, A_CUTS, &cuts); c.api->getintattrib(cb, A_ROWS, &rows); c.api->getintattrib(cb, A_COLS, &cols);
    c.api->getintattrib(cb, A_RESTARTS, &restarts); c.api->getintattrib(cb, 1335, &tree); c.api->getintattrib(cb, A_LPSTATUS, &lp);
    double obj = 0; c.api->getdblattrib(cb, D_LPOBJVAL, &obj);
    if(lp != LP_OPTIMAL) return;
    std::vector<double> xv(static_cast<std::size_t>(c.cols));
    c.api->getcallbacksolution(cb, nullptr, xv.data(), 0, c.cols - 1);
    const double viol = c.cut.violation(xv);
    const int st = c.trigger_state ? c.trigger_state(xv) : -1;
    bool now = false, trig = false; int err = 0;
    if(!c.added && viol > 1e-4 && (!c.trigger_state || st == c.worse)) { err = add(c, cb); c.added = true; now = true; if(p_action) *p_action = 2; }
    else if(c.added && c.trigger_state) {
        if(st == c.worse) { ++c.same_after; c.same_viol += viol > 1e-6; }
        else if(st == 1 - c.worse) { ++c.sib_after; c.sib_viol += viol > 1e-6; }
        if(c.trigger && !c.restarted) { c.restarted = true; const int e = c.trigger(cb, &c.trig_status); trig = true; if(e) err = e; }
    }
    if(c.g && c.restarted && tree > 0 && !now) {
        // after the restart, a valid global hull cut at each firing, so that the pool is processed
        for(int i = 1; i < c.g->k; ++i) {
            const auto h = c.g->hull_cut(i);
            if(h.violation(xv) > 1e-4) {
                const int start[2] = {0, static_cast<int>(h.ind.size())};
                if(!c.fn->addmanaged(cb, 1, 1, &h.sense, &h.rhs, start, h.ind.data(), h.val.data())) { ++c.poked; if(p_action) *p_action = 2; }
                break;
            }
        }
    }
    if(c.lines++ < c.maxl)
        std::println(stderr, "   {} node={} depth={} round={} RESTARTS={} TREERESTARTS={} COLS={} ROWS={} CUTS={} lp={:.4f} watched={:.4f} cut_viol={:.4f} y0state={} {} {}", who, node, depth, rounds, restarts, tree, cols, rows, cuts, obj, c.watch >= 0 ? xv[static_cast<std::size_t>(c.watch)] : NAN, viol, st, now ? std::format("<-- added err={}", err) : "", trig ? std::format("<-- XPRStriggerrestart status={} err={}", c.trig_status, err) : "");
}
static void h_opt(prob_t cb, void * d, int *) { body(*static_cast<tc *>(d), cb, nullptr, "optnode "); }
static void h_cut(prob_t cb, void * d, int, int * a) { body(*static_cast<tc *>(d), cb, a, "cutround"); }
template <class C>
static void reg(M & m, C & c) {
    auto & api = m.native_api();
    if(c.mode.starts_with("managed") || c.mode == "cutround_addcuts") api._check(m.native_model(), c.fn->addcbcutround(m.native_model(), h_cut, &c, 0));
    else api._check(m.native_model(), api.addcboptnode(m.native_model(), h_opt, &c, 0));
}
int main(int argc, char ** argv) {
    const std::string what = argc > 1 ? argv[1] : "dual";
    if(what == "dual") {
        for(const std::string mode : {"addcuts", "cutround_addcuts", "managed_global"}) for(const int rootp : {-1, 0}) {
            M m; one_thread(m);
            set_int(m, C_MIPDUALREDUCTIONS, 0);
            set_int(m, 8224, rootp);
            cp::build_atsp(m, 14);
            const block k = add_block(m);
            m.set_time_limit(std::chrono::duration<double>(20.0));
            tc c; c.api = &m.native_api(); c.fn = fns_of(m); c.cols = static_cast<int>(m.num_variables()); c.watch = k.w; c.cut = cp::raw_cut{{k.w}, {1.0}, 'G', 0.5}; c.mode = mode; c.maxl = 12;
            reg(m, c);
            std::println(stderr, "== dual trace mode {} ROOTPRESOLVE={}", mode, rootp);
            const double t = cp::timed_solve(m);
            std::println(stderr, "   {} (409 without the cut, 410.5 with it)", cp::summary(m, t));
        }
    } else {
        const bool poke = what == "s1poke";
        for(const std::string mode : poke ? std::vector<std::string>{"managed_local", "managed_global", "pool_like_addcuts_cutround"} : std::vector<std::string>{"addcuts", "pool_like_addcuts_cutround", "managed_local", "managed_global"}) for(const int flip : {0, 1}) {
            M m; scope_settings(m);
            set_int(m, C_MIPRESTART, 1);
            set_int(m, C_PRESOLVE, 1);
            auto g = cp::build_gadgets(m, 10, flip);
            m.set_time_limit(std::chrono::duration<double>(20.0));
            tc c; c.api = &m.native_api(); c.fn = fns_of(m); c.cols = static_cast<int>(m.num_variables()); c.watch = g.y[0]; c.cut = g.invalid_global_cut(); c.mode = mode == "pool_like_addcuts_cutround" ? "cutround_addcuts" : mode; c.maxl = poke ? 0 : 14;
            if(poke) c.g = &g;
            c.trigger_state = [&g](const std::vector<double> & xv) { return g.state(xv); }; c.worse = g.worse_state();
            void * h = dlopen(m.native_api().library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
            c.trigger = reinterpret_cast<int (*)(prob_t, int *)>(dlsym(h, "XPRStriggerrestart"));
            reg(m, c);
            std::println(stderr, "== s1+restart mode {} flip={}", mode, flip);
            const double t = cp::timed_solve(m);
            std::println(stderr, "   {} (opt {}) {}: subtree viol/checks {}/{} sibling viol/checks {}/{} trigger status={} poked={}", cp::summary(m, t), g.opt, std::fabs(m.get_solution_value() - g.opt) < 1e-4 ? "optimum KEPT" : "optimum LOST", c.same_viol, c.same_after, c.sib_viol, c.sib_after, c.trig_status, c.poked);
        }
    }
}
