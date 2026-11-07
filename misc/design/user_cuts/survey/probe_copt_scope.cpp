// S0 and S1 only, under each CUT_SCOPE variant: which one has a tree split on y0
// with the hook firing on both sides?
#include "probe_copt_common.cpp"
using namespace std::chrono;
int main() {
    for(const char * v : {"tree1", "tree1r1", "tree3", "noroot", "noroot_r1", "root_r1", "root_r1_tree1", "root1_tree1"}) {
        setenv("CUT_SCOPE", v, 1);
        for(const bool flip : {false, true}) {
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
            auto s = std::make_shared<cp::stats>();
            long st[3] = {0, 0, 0};
            attach(m, cp::watched([&](const auto & x, const auto &) { st[g.state(x) + 1]++; }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), "usercut");
            const double t = cp::timed_solve(m);
            std::println(stderr, "{:14} flip={} S0: {} | relax={} node={} y0 frac:{} y0=0:{} y0=1:{} obj_first={}", v, flip, cp::summary(m, t), m.hook->relax_calls.load(), m.hook->node_calls.load(), st[0], st[1], st[2], s->objs.empty() ? NAN : s->objs[0]);
        }
        for(const bool flip : {false, true}) {
            M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
            auto s = std::make_shared<cp::stats>();
            auto tr = std::make_shared<cp::scope_tracker>();
            tr->cut = g.invalid_global_cut(); tr->trigger = g.worse_state(); tr->state = [&g](const auto & x) { return g.state(x); };
            attach(m, cp::watched([tr](const auto & x, const cp::add_fn & add) { if(tr->wants_add(x)) tr->add_error = add(tr->cut); }, s, [&g](const auto & x) { return g.objective(x); }, [&g](const auto & x) { return g.point_ok(x); }), "usercut");
            const double t = cp::timed_solve(m);
            double value = NAN; try { value = m.get_solution_value(); } catch(const std::exception &) {}
            std::println(stderr, "{:14} flip={} S1: {} (opt {}) {} | {}", v, flip, cp::summary(m, t), g.opt, std::fabs(value - g.opt) < 1e-4 ? "KEPT" : "LOST", tr->line());
        }
    }
}
