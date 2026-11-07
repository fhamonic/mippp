// A trace of every MIPRELAX call around one added cut. MIPNODE ("after a MIP node was
// processed") delimits nodes at one thread, so calls between two MIPNODE bumps belong
// to one node. Arguments: <S1|S2> <flip 0|1> <how many calls to print>.
#include "probe_copt_common.cpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string which = argc > 1 ? argv[1] : "S2";
    const bool flip = argc > 2 && std::atoi(argv[2]);
    const int show = argc > 3 ? std::atoi(argv[3]) : 60;
    M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, flip); m.set_time_limit(duration<double>(20.0));
    const cp::raw_cut cut = which == "S1" ? g.invalid_global_cut() : which == "S2" ? g.hull_cut(3) : cp::raw_cut{};
    const int trigger = g.worse_state();
    long call = 0, after = 0;
    bool added = false;
    long same = 0, same_v = 0, sib = 0, sib_v = 0, y3frac = 0, y3frac_same = 0;
    attach(m, [&](const std::vector<double> & x, const cp::add_fn & add) {
        ++call;
        const int s = g.state(x);
        const double v = which == "none" ? 0 : cut.violation(x);
        const double y3 = x[static_cast<std::size_t>(g.y[3])];
        std::string tag;
        if(which != "none" && !added && s == trigger && v > 1e-4) {
            const int e = add(cut);
            added = true;
            tag = std::format(" <== ADD rc={}", e);
        } else if(added) {
            ++after;
            if(s == trigger) { ++same; same_v += v > 1e-6; if(std::fabs(y3 - std::round(y3)) > 1e-6) ++y3frac_same; }
            else if(s == 1 - trigger) { ++sib; sib_v += v > 1e-6; if(std::fabs(y3 - std::round(y3)) > 1e-6) ++y3frac; }
        }
        if(call <= show || (added && after <= show))
            std::println(stderr, "call {:5} nodes_done={:5} y0={} y3={:.3f} obj={:9.4f} viol={:9.4f}{}", call, m.hook->node_calls.load(), s, y3, g.objective(x), v, tag);
    }, "usercut");
    const double t = cp::timed_solve(m);
    std::println(stderr, "RESULT {} flip={} scope={}: {} (opt {}) relax={} node={} after-add subtree viol/checks={}/{} (y3 frac {}) sibling viol/checks={}/{} (y3 frac {})", which, flip, std::getenv("CUT_SCOPE") ? std::getenv("CUT_SCOPE") : "tree1", cp::summary(m, t), g.opt, m.hook->relax_calls.load(), m.hook->node_calls.load(), same_v, same, y3frac_same, sib_v, sib, y3frac);
}
