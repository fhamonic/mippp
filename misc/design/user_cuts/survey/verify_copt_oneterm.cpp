// O6. (1) y <= 0 is valid by integrality alone: y sits on an odd cycle, Sum x_i >= 4 + y over a 9-cycle of
// x_i + x_{i+1} <= 1. The LP has y = .5. One term against two terms, at the root and in the tree.
// (2) rowless and presolve-fixed columns: rc and status, cut offered once at the first hook call.
#include "verify_copt_common.hpp"
using namespace std::chrono;
int main(int argc, char ** argv) {
    const std::string exp = argv[1];
    if(exp == "odd") {
        const std::string form = argv[2];
        const int after = std::atoi(argv[3]);
        const unsigned seed = argc > 4 ? std::atoi(argv[4]) : 3;
        V m; m.iparam("Threads", 1); m.set_time_limit(duration<double>(30.0));
        // an MKP to have a tree, then the odd-cycle gadgets on top
        const std::size_t cols = 40, rows = 5;
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> A(1, 100), R(0, 50);
        std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
        for(auto & r : a) for(auto & v : r) v = A(rng);
        std::vector<double> cc(cols);
        for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; cc[j] = std::floor(s / rows + R(rng)); }
        auto x = m.add_binary_variables(cols);
        const auto ids = std::views::iota(std::size_t{0}, cols);
        for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= std::floor(s / 2)); }
        const std::size_t L = 9;
        auto ys = m.add_binary_variables(2);
        auto cyc = m.add_binary_variables(2 * L);
        for(std::size_t g = 0; g < 2; ++g) {
            for(std::size_t i = 0; i < L; ++i) m.add_constraint(cyc(g * L + i) + cyc(g * L + (i + 1) % L) <= 1);
            m.add_constraint(xsum(std::views::iota(std::size_t{0}, L), [&, g](std::size_t i) { return 1.0 * cyc(g * L + i); }) - ys(g) >= static_cast<double>((L - 1) / 2));
        }
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return cc[j] * x(j); }) + 200.0 * ys(std::size_t{0}) + 200.0 * ys(std::size_t{1}));
        const int y1 = m.native_id(ys(std::size_t{0})), y2 = m.native_id(ys(std::size_t{1}));
        std::atomic<long> calls{0}, seen_frac{0}, added{0}, rc_err{0}, before_frac{0}, later{0}, later_frac{0}, first_call{-1}, root_calls{0}, same_node_refire{0};
        std::atomic<int> add_node{-1};
        std::atomic<double> maxy{0};
        m.raw_mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
        cp::raw_cut cut;
        if(form == "one") cut = cp::raw_cut{{y1}, {1.0}, 'L', 0.0};
        if(form == "two") cut = cp::raw_cut{{y1, y2}, {1.0, 1.0}, 'L', 0.0};
        m.raw = [&](c::copt_prob *, void * cb, int ctx) {
            if(ctx != c::COPT_CBCONTEXT_MIPRELAX) return 0;
            ++calls;
            int nc = 0; m.native_api().GetCallbackInfo(cb, "NodeCnt", &nc);
            if(nc == 0) ++root_calls;
            std::vector<double> p(m.num_variables());
            if(m.native_api().GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, p.data())) return 0;
            const double v = p[(std::size_t)y1];
            if(v > maxy) maxy = v;
            if(v > 1e-6) ++seen_frac;
            if(form != "none") {
                if(added > 0) { ++later; if(v > 1e-6) ++later_frac; if(nc == add_node && v > 1e-6) ++same_node_refire; }
                if(added == 0 && nc >= after && cut.violation(p) > 1e-6) {
                    const int rc = m.native_api().AddCallbackUserCut(cb, (int)cut.ind.size(), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs);
                    ++added; add_node = nc; first_call = calls.load(); if(rc) ++rc_err;
                }
            }
            return 0;
        };
        const double t = cp::timed_solve(m);
        std::println(stderr, "odd {:5} after={:4} seed={}: {} calls={} root_calls={} y1>0 in {} calls, max y1={:.3f} | added={} at call {} node {} rc_err={} | later calls={} with y1>0: {}, same node after add with y1>0: {}", form, after, seed, cp::summary(m, t), calls.load(), root_calls.load(), seen_frac.load(), maxy.load(), added.load(), first_call.load(), add_node.load(), rc_err.load(), later.load(), later_frac.load(), same_node_refire.load());
    }
    if(exp == "rowless") {
        const std::string kase = argv[2];
        V m; m.iparam("Threads", 1); m.set_time_limit(duration<double>(20.0));
        setenv("CAPF", "0.2", 1);
        auto v = build_mcnd(m, 14, 30, 16, 1);
        if(std::getenv("NOHEUR")) for(const char * n : {"HeurLevel", "PreRootHeurLevel", "RoundingHeurLevel", "DivingHeurLevel", "SubMipHeurLevel", "FAPHeurLevel"}) m.iparam(n, 0);
        auto dc = m.add_variables(2, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});   // continuous, no row
        auto db = m.add_binary_variables(1);                                                 // binary, no row
        auto oc = m.add_variables(1, {.obj_coef = 3, .lower_bound = 0, .upper_bound = 1});   // cost: dual-fixed at 0
        auto f = m.add_binary_variables(2);
        m.add_constraint(1.0 * f(std::size_t{0}) == 1.0); m.add_constraint(1.0 * f(std::size_t{1}) == 1.0);
        auto red = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});   // only in a redundant row
        m.add_constraint(1.0 * red(std::size_t{0}) + 1.0 * dc(std::size_t{1}) <= 5.0);
        const int d = m.native_id(dc(std::size_t{0})), d2 = m.native_id(dc(std::size_t{1})), b = m.native_id(db(std::size_t{0})), o = m.native_id(oc(std::size_t{0})), f1 = m.native_id(f(std::size_t{0})), f2 = m.native_id(f(std::size_t{1})), r = m.native_id(red(std::size_t{0}));
        const std::map<std::string, std::pair<cp::raw_cut, const char *>> cases{
            {"none", {cp::raw_cut{}, "baseline"}},
            {"d>=1", {cp::raw_cut{{d}, {1.0}, 'G', 1.0}, "not valid, feasible d=0 removed"}},
            {"d==1", {cp::raw_cut{{d}, {1.0}, 'E', 1.0}, "not valid"}},
            {"d<=0.5", {cp::raw_cut{{d}, {1.0}, 'L', 0.5}, "not valid, may hold at the point"}},
            {"d<=1", {cp::raw_cut{{d}, {1.0}, 'L', 1.0}, "VALID, satisfied"}},
            {"d>=0", {cp::raw_cut{{d}, {1.0}, 'G', 0.0}, "VALID, satisfied"}},
            {"2d<=2", {cp::raw_cut{{d}, {2.0}, 'L', 2.0}, "VALID, satisfied"}},
            {"b>=1", {cp::raw_cut{{b}, {1.0}, 'G', 1.0}, "not valid"}},
            {"b<=1", {cp::raw_cut{{b}, {1.0}, 'L', 1.0}, "VALID, satisfied"}},
            {"d+d2>=1", {cp::raw_cut{{d, d2}, {1.0, 1.0}, 'G', 1.0}, "not valid, two rowless"}},
            {"d+d2<=2", {cp::raw_cut{{d, d2}, {1.0, 1.0}, 'L', 2.0}, "VALID, satisfied"}},
            {"o>=0", {cp::raw_cut{{o}, {1.0}, 'G', 0.0}, "VALID, dual-fixed col"}},
            {"o>=1", {cp::raw_cut{{o}, {1.0}, 'G', 1.0}, "not valid, cost column"}},
            {"f1+f2<=2", {cp::raw_cut{{f1, f2}, {1.0, 1.0}, 'L', 2.0}, "VALID, fixed cols, constant"}},
            {"f1+f2>=2", {cp::raw_cut{{f1, f2}, {1.0, 1.0}, 'G', 2.0}, "VALID, fixed cols, constant"}},
            {"f1+f2==2", {cp::raw_cut{{f1, f2}, {1.0, 1.0}, 'E', 2.0}, "VALID, fixed cols, constant"}},
            {".1f1+.2f2<=.3", {cp::raw_cut{{f1, f2}, {0.1, 0.2}, 'L', 0.3}, "VALID up to round-off 5.5e-17"}},
            {".1f1+.2f2>=.3", {cp::raw_cut{{f1, f2}, {0.1, 0.2}, 'G', 0.3}, "VALID, rounded the other way"}},
            {"f1<=1", {cp::raw_cut{{f1}, {1.0}, 'L', 1.0}, "VALID, fixed col"}},
            {"f1>=1", {cp::raw_cut{{f1}, {1.0}, 'G', 1.0}, "VALID, fixed col, tight"}},
            {"f1+d<=2", {cp::raw_cut{{f1, d}, {1.0, 1.0}, 'L', 2.0}, "VALID, fixed + rowless"}},
            {"f1+d>=1", {cp::raw_cut{{f1, d}, {1.0, 1.0}, 'G', 1.0}, "VALID, fixed + rowless: tight at d=0"}},
            {"r<=1", {cp::raw_cut{{r}, {1.0}, 'L', 1.0}, "VALID, column only in a redundant row"}},
            {"r>=1", {cp::raw_cut{{r}, {1.0}, 'G', 1.0}, "not valid"}},
        };
        const auto & [cut, note] = cases.at(kase);
        std::atomic<long> calls{0}, adds{0}, errs{0}; std::atomic<int> last_rc{0};
        std::atomic<double> dval{-1};
        if(kase != "none") {
            m.raw_mask = c::COPT_CBCONTEXT_MIPRELAX | c::COPT_CBCONTEXT_MIPNODE;
            m.raw = [&](c::copt_prob *, void * cb, int ctx) {
                if(ctx != c::COPT_CBCONTEXT_MIPRELAX) return 0;
                if(++calls > 1) return 0;
                std::vector<double> p(m.num_variables());
                m.native_api().GetCallbackInfo(cb, c::COPT_CBINFO_RELAXSOLUTION, p.data());
                dval = cut.violation(p);
                const int rc = m.native_api().AddCallbackUserCut(cb, (int)cut.ind.size(), cut.ind.data(), cut.val.data(), cut.sense, cut.rhs);
                ++adds; if(rc) { ++errs; last_rc = rc; }
                return 0;
            };
        }
        const double t = cp::timed_solve(m);
        std::println(stderr, "rowless {:14} [{}]: {} calls={} violation at point={:.3g} rc={}", kase, note, cp::summary(m, t), calls.load(), dval.load(), last_rc.load());
    }
}
