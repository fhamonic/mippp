// W as one process per case: cut_common.hpp runs them in one process and the
// duplicate-column case aborted it (realloc(): invalid next size), losing the rest.
// Usage: probe_copt_w gadgets|atsp <case>
#include "probe_copt_common.cpp"
using namespace std::chrono;
static const char * nm(int e) { return e == 0 ? "OK" : e == 3 ? "INVALID" : e == 5 ? "INTERNAL" : "other"; }
template <class G>
static cp::raw_cut gadget_case(const std::string & n, const G & g, int cols, bool & every) {
    using R = cp::raw_cut;
    if(n == "eq1") return R{{g.dummy}, {1.0}, 'E', 1.0};
    if(n == "eq0") return R{{g.dummy}, {1.0}, 'E', 0.0};
    if(n == "eq_two") return R{{g.dummy, g.za[3]}, {1.0, 0.0}, 'E', 1.0};
    if(n == "eq_hull") { auto c = g.hull_cut(3); c.sense = 'E'; return c; }
    if(n == "noterms") return R{{}, {}, 'L', 1.0};
    if(n == "noterms_bad") return R{{}, {}, 'L', -1.0};
    if(n == "plain") return g.hull_cut(3);
    if(n == "zero") { auto c = g.hull_cut(3); c.ind.push_back(g.za[1]); c.val.push_back(0.0); return c; }
    if(n == "zero_first") { auto c = g.hull_cut(3); c.ind.insert(c.ind.begin(), g.za[1]); c.val.insert(c.val.begin(), 0.0); return c; }
    if(n == "zero_only") return R{{g.za[1]}, {0.0}, 'L', 1.0};
    if(n == "zero_only_bad") return R{{g.za[1]}, {0.0}, 'L', -1.0};
    if(n == "tiny") { auto c = g.hull_cut(3); c.ind.push_back(g.za[1]); c.val.push_back(1e-12); return c; }
    if(n == "dup") { auto c = g.hull_cut(3); c.val[0] = 0.5; c.ind.push_back(c.ind[0]); c.val.push_back(0.5); return c; }
    if(n == "dup_nofirst") { auto c = g.hull_cut(3); c.val[2] *= 0.5; c.ind.push_back(c.ind[2]); c.val.push_back(c.val[2]); return c; }
    if(n == "dup_cancel") { auto c = g.hull_cut(3); c.ind.push_back(g.za[1]); c.val.push_back(1.0); c.ind.push_back(g.za[1]); c.val.push_back(-1.0); return c; }
    if(n == "notviol") return R{{g.za[3]}, {1.0}, 'L', 1e6};
    if(n == "same_every") { every = true; return g.hull_cut(3); }
    if(n == "empties") return R{{g.za[3]}, {1.0}, 'L', -1.0};
    if(n == "free") { auto c = g.hull_cut(3); c.sense = 'N'; return c; }
    if(n == "range") { auto c = g.hull_cut(3); c.sense = 'R'; return c; }
    if(n == "bigidx") return R{{cols + 5}, {1.0}, 'L', 1.0};
    if(n == "negidx") return R{{-1}, {1.0}, 'L', 1.0};
    if(n == "nan") { auto c = g.hull_cut(3); c.val[0] = NAN; return c; }
    if(n == "nanrhs") { auto c = g.hull_cut(3); c.rhs = NAN; return c; }
    if(n == "infrhs") { auto c = g.hull_cut(3); c.rhs = 1e30; return c; }
    if(n == "infcoef") { auto c = g.hull_cut(3); c.val[0] = 1e30; return c; }
    if(n == "hugecoef") { auto c = g.hull_cut(3); c.val[0] = 1e12; return c; }
    // Which layouts of zeros and duplicates matter. a..k use gadget 3, whose true cut has two terms.
    const int za = g.za[3], zb = g.zb[3], y3 = g.y[3], z1 = g.za[1], z2 = g.zb[1];
    const double r = g.c[3] * g.total0[3];
    if(n == "a") return R{{za, zb}, {1, 1}, 'L', r};
    if(n == "b") return R{{za, zb, z1}, {1, 1, 0.0}, 'L', r};
    if(n == "c") return R{{za, zb, y3}, {1, 1, -0.0}, 'L', r};
    if(n == "d") return R{{za, zb, y3}, {1, 1, 0.0}, 'L', r};
    if(n == "e") return R{{za, zb, y3, z1}, {1, 1, 0.0, 0.0}, 'L', r};
    if(n == "f") return R{{za, zb, y3, z1}, {1, 1, -0.0, -0.0}, 'L', r};
    if(n == "g") return R{{za, zb, y3, z1, z2}, {1, 1, 0.0, 0.0, 0.0}, 'L', r};
    if(n == "h") return R{{y3, za, zb}, {0.0, 1, 1}, 'L', r};
    if(n == "h2") return R{{z1, za, y3, zb}, {0.0, 1, 0.0, 1}, 'L', r};
    if(n == "i") return R{{za, zb, y3, z2}, {1, 1, 0.0, 1e-30}, 'L', r};
    if(n == "j") return R{{za, zb, za}, {0.5, 1, 0.5}, 'L', r};
    if(n == "j2") return R{{za, za, zb}, {0.5, 0.5, 1}, 'L', r};
    if(n == "k") return R{{za, za, zb, y3}, {0.5, 0.5, 1, 0.0}, 'L', r};
    if(n == "k2") return R{{za, zb, y3, za}, {0.5, 1, 0.0, 0.5}, 'L', r};
    if(n == "k3") return R{{za, zb, za, y3}, {0.5, 1, 0.5, 0.0}, 'L', r};
    // gadget 0: the hull cut has a real coefficient on y
    if(n == "m") return g.hull_cut(0);
    if(n == "m0") { auto c = g.hull_cut(0); c.ind.push_back(z1); c.val.push_back(0.0); return c; }
    if(n == "m00") { auto c = g.hull_cut(0); c.ind.push_back(z1); c.val.push_back(0.0); c.ind.push_back(z2); c.val.push_back(0.0); return c; }
    if(n == "m0mid") { auto c = g.hull_cut(0); c.ind.insert(c.ind.begin() + 1, z1); c.val.insert(c.val.begin() + 1, 0.0); return c; }
    if(n == "mdup") { auto c = g.hull_cut(0); c.val[0] = 0.5; c.ind.push_back(c.ind[0]); c.val.push_back(0.5); return c; }
    // singletons: does a one-term cut do anything at all?
    if(n == "s_y3_le0") return R{{y3}, {1}, 'L', 0.0};
    if(n == "s_za3_le1") return R{{za}, {1}, 'L', 1.0};
    if(n == "s_za3_ge1000") return R{{za}, {1}, 'G', 1000.0};
    if(n == "s_dummy_ge1") return R{{g.dummy}, {1}, 'G', 1.0};
    if(n == "two_empties") return R{{za, zb}, {1, 1}, 'L', -1.0};
    if(n == "two_tight") return R{{za, zb}, {1, 1}, 'L', 1.0};
    // the same valid two-term cut with one explicit zero on various columns
    if(n == "b_zb1") return R{{za, zb, g.zb[1]}, {1, 1, 0.0}, 'L', r};
    if(n == "b_za5") return R{{za, zb, g.za[5]}, {1, 1, 0.0}, 'L', r};
    if(n == "b_za3self") return R{{za, zb, g.za[4]}, {1, 1, 0.0}, 'L', r};
    if(n == "b_y5") return R{{za, zb, g.y[5]}, {1, 1, 0.0}, 'L', r};
    if(n == "b_dummy") return R{{za, zb, g.dummy}, {1, 1, 0.0}, 'L', r};
    if(n == "b_tiny") return R{{za, zb, z1}, {1, 1, 1e-13}, 'L', r};
    if(n == "b_tiny2") return R{{za, zb, z1}, {1, 1, 1e-9}, 'L', r};
    if(n == "b_tiny3") return R{{za, zb, z1}, {1, 1, 1e-300}, 'L', r};
    if(n == "b_dummy1") return R{{za, zb, g.dummy}, {1, 1, 1.0}, 'L', r};
    if(n == "dummy_two") return R{{g.dummy, g.za[3]}, {1.0, 1.0}, 'G', 1000.0};
    if(n == "eq2") return R{{za, zb}, {1, 1}, 'E', r};
    if(n == "eq2_low") return R{{za, zb}, {1, 1}, 'E', r - 1.0};
    if(n == "ge2") return R{{za, zb}, {1, 1}, 'G', r + 1.0};
    if(n == "bigidx2") return R{{za, cols + 5}, {1, 1}, 'L', 1.0};
    if(n == "negidx2") return R{{za, -1}, {1, 1}, 'L', 1.0};
    std::println(stderr, "unknown case {}", n); std::exit(2);
}
int main(int argc, char ** argv) {
    const std::string inst = argv[1], name = argv[2];
    if(inst == "gadgets") {
        M m; scope_settings(m); auto g = cp::build_gadgets(m, 10, false); m.set_time_limit(duration<double>(15.0));
        if(const char * pv = std::getenv("CUT_PRESOLVE")) m.native_api().SetIntParam(m.native_model().second, "Presolve", std::atoi(pv));
        auto s = std::make_shared<cp::stats>();
        bool every = false, once = false;
        const cp::raw_cut cut = gadget_case(name, g, static_cast<int>(m.num_variables()), every);
        int err = 0;
        attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
            if(every || !once) { once = true; const int e = add(cut); if(e) err = e; }
        }, s, [&g](const auto & x) { return g.objective(x); }), std::getenv("CUT_W_MODE") ? std::getenv("CUT_W_MODE") : "usercut");
        const double t = cp::timed_solve(m);
        std::println(stderr, "W {:14} scope={}: {} (opt {}) rc={} ({})  relax={} node={}\n   {}", name, std::getenv("CUT_SCOPE") ? std::getenv("CUT_SCOPE") : "noroot", cp::summary(m, t), g.opt, err, nm(err), m.hook->relax_calls.load(), m.hook->node_calls.load(), s->line());
        return 0;
    }
    // atsp: default settings, one thread, the separator's cuts with the fault in each
    M m; one_thread(m); auto a = cp::build_atsp(m, 26); m.set_time_limit(duration<double>(40.0));
    if(std::getenv("CUT_RC0")) m.native_api().SetIntParam(m.native_model().second, "RootCutLevel", 0);
    if(const char * pv = std::getenv("CUT_PRESOLVE")) m.native_api().SetIntParam(m.native_model().second, "Presolve", std::atoi(pv));
    auto s = std::make_shared<cp::stats>();
    int err = 0;
    const int spare = a.col[1][2], spare2 = a.col[3][4];
    attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) {
        for(auto c : a.separate(x)) {
            if(name == "zero" && std::ranges::find(c.ind, spare) == c.ind.end()) { c.ind.push_back(spare); c.val.push_back(0.0); }
            if(name == "zero2") { for(int j : {spare, spare2}) if(std::ranges::find(c.ind, j) == c.ind.end()) { c.ind.push_back(j); c.val.push_back(0.0); } }
            if(name == "zero_u") { c.ind.push_back(650 + 3); c.val.push_back(0.0); }
            if(name == "zero_u2") { c.ind.insert(c.ind.begin(), 650 + 3); c.val.insert(c.val.begin(), 0.0); }
            if(name == "dup") { c.val[0] = 0.5; c.ind.push_back(c.ind[0]); c.val.push_back(0.5); }
            const int e = add(c); if(e) err = e;
        }
    }, s, [&a](const auto & x) { return a.objective(x); }, [&a](const auto & x) { return a.point_ok(x); }, [&a](const auto & x) { return a.integral(x); }), "usercut");
    const double t = cp::timed_solve(m);
    std::println(stderr, "W atsp {:10}: {} (opt 549) rc={} ({})  relax={} node={}\n   {}", name, cp::summary(m, t), err, nm(err), m.hook->relax_calls.load(), m.hook->node_calls.load(), s->line());
}
