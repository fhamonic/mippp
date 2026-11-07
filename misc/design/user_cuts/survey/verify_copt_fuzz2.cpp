// D, oracle fuzz, second version: since COPT can prove a false optimum with no hook on knapsacks
// (vout_copt_falseopt_sweep*.txt), truth and every cut's rhs come from Gurobi, at zero gap. The search under test is COPT
// at default settings with one thread, or Presolve=0 with PRE0=1. Reports: COPT hook-free against truth, COPT with cuts
// against truth. usage: verify_copt_fuzz2 <family> from to [P]
#include "verify_copt_common.hpp"
#include "mippp/solvers/gurobi/all.hpp"
using namespace std::chrono;
template <class Mdl>
void build_family(Mdl & m, const std::string & fam, unsigned seed) {
    if(fam == "nd") { setenv("CAPF", "0.2", 1); build_mcnd(m, 8, 10, 6, seed); return; }
    if(fam == "mkp") { cp::build_mkp(m, 5, 30, seed); return; }
    std::mt19937 rng(seed);
    const std::size_t cols = 35, rows = 15;
    std::uniform_int_distribution<int> A(-5, 20), C(1, 40);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> b(rows), c(cols);
    for(std::size_t i = 0; i < rows; ++i) { double t = 0; for(double v : a[i]) t += std::max(v, 0.0); b[i] = std::floor(t * 0.4); }
    for(auto & v : c) v = C(rng);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    if(fam == "cg") {
        auto x = m.add_binary_variables(cols);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= b[i]);
    } else {
        auto x = m.add_integer_variables(cols, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
        auto s = m.add_variables(cols, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j) + 0.3 * c[j] * s(j); }));
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= 3.0 * b[i]);
        for(std::size_t j = 0; j < cols; ++j) m.add_constraint(s(j) - 2.5 * x(j) <= 0.0);
        m.add_constraint(xsum(ids, [&](std::size_t j) { return 1.0 * s(j); }) <= 60.0);
    }
}
int main(int argc, char ** argv) {
    const std::string fam = argv[1];
    const unsigned from = std::atoi(argv[2]), to = std::atoi(argv[3]);
    const int P = argc > 4 ? std::atoi(argv[4]) : 25;
    const bool pre0 = std::getenv("PRE0") != nullptr;
    long cases = 0, wrong = 0, nohook_wrong = 0, wrong_when_nohook_right = 0, adds = 0, seeds_run = 0;
    const auto cfg = [&](V & m) { m.iparam("Threads", 1); m.set_time_limit(duration<double>(20.0)); if(pre0) m.iparam("Presolve", 0); };
    for(unsigned seed = from; seed <= to; ++seed) {
        mippp::gurobi_milp g; g.set_thread_limit(1); g.set_time_limit(duration<double>(20.0)); g.set_optimality_tolerance(0.0); build_family(g, fam, seed);
        g.native_api().setintparam(g.native_model().first, "OutputFlag", 0);
        cp::timed_solve(g);
        if(!is_a<status::optimal>(g.get_status())) { continue; }
        const double truth = g.get_solution_value();
        std::vector<double> best(g.num_variables()); { auto s = g.get_solution(); for(auto v : g.variables()) best[(std::size_t)g.native_id(v)] = s[v]; }
        auto first = *g.variables().begin();
        std::vector<decltype(first)> vs; for(auto v : g.variables()) vs.push_back(v);
        ++seeds_run;
        double hookfree; std::string hs;
        { V b; cfg(b); build_family(b, fam, seed); const double t = cp::timed_solve(b); hookfree = cf::value_of(b); hs = cp::summary(b, t); }
        const auto right = [&](double v) { return std::fabs(v - truth) <= 1.1e-4 * std::fabs(truth); };
        nohook_wrong += !right(hookfree);
        // harvest fractional points on COPT's search
        std::vector<std::vector<double>> pts;
        { V h; cfg(h); build_family(h, fam, seed); long seen = 0; h.attach([&](const std::vector<double> & x, const cp::add_fn &) { ++seen; if(pts.size() < 400 && (seen < 30 || seen % 7 == 0)) pts.push_back(x); }); cp::timed_solve(h); }
        if(pts.empty()) { std::println(stderr, "{} seed={} NO POWER: the hook never fired | {}", fam, seed, hs); continue; }
        std::vector<double> objc(vs.size());
        { V o; build_family(o, fam, seed);
          using getcol_fn = int (*)(c::copt_prob *, const char *, int, const int *, double *);
          auto getcol = cp::symf<getcol_fn>(o.native_api().library_path(), "COPT_GetColInfo");
          std::vector<int> ids(vs.size()); for(std::size_t i = 0; i < vs.size(); ++i) ids[i] = g.native_id(vs[i]);
          if(getcol(o.native_model().second, "Obj", (int)ids.size(), ids.data(), objc.data())) return 2;
          if(fam == "nd") for(auto & w : objc) w = -w; }
        std::mt19937 rng(seed * 131u + 7u);
        std::uniform_int_distribution<std::size_t> ptk(0, pts.size() - 1);
        const int terms[] = {1, 2, 2, 3, 4, 6, 8, 15};
        std::uniform_int_distribution<int> termc(0, 7), md(0, 5), bin(0, 1), mult(1, 3);
        std::uniform_real_distribution<double> noise(0.8, 1.2), coin(0.0, 1.0);
        const double margins[] = {0, 0, 0, 1e-9, 1e-6, 1e-3};
        std::vector<cp::raw_cut> pool; long invalid = 0;
        for(int t = 0; t < 6 * P && (int)pool.size() < P; ++t) {
            const auto & p = pts[ptk(rng)];
            std::vector<std::size_t> frac;
            for(std::size_t i = 0; i < vs.size(); ++i) { const double w = p[(std::size_t)g.native_id(vs[i])]; if(std::fabs(w - std::round(w)) > 1e-4) frac.push_back(i); }
            std::vector<std::pair<int, double>> A;
            const int mode = t % 5;
            if(mode == 0) { for(std::size_t i = 0; i < vs.size(); ++i) if(objc[i] != 0) A.emplace_back((int)i, objc[i]); }
            else if(mode == 1) { for(std::size_t i = 0; i < vs.size(); ++i) if(objc[i] != 0) A.emplace_back((int)i, std::round(objc[i] * noise(rng) * 100) / 100); }
            else if(mode == 2) { for(std::size_t i = 0; i < vs.size(); ++i) if(objc[i] != 0 && coin(rng) < 0.6) A.emplace_back((int)i, objc[i]); }
            else if(mode == 3) { for(std::size_t i : frac) if(objc[i] != 0) A.emplace_back((int)i, std::round(objc[i] * noise(rng) * 100) / 100); }
            else {
                if(frac.empty()) continue;
                std::ranges::shuffle(frac, rng);
                const std::size_t nt = std::min<std::size_t>(frac.size(), (std::size_t)terms[termc(rng)]);
                for(std::size_t q = 0; q < nt; ++q) { const double w = p[(std::size_t)g.native_id(vs[frac[q]])]; const double fr = w - std::floor(w); A.emplace_back((int)frac[q], (fr >= 0.5 ? 1.0 : -1.0) * mult(rng)); }
            }
            if(A.empty()) continue;
            g.set_maximization();
            g.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * vs[(std::size_t)e.first]; }));
            cp::timed_solve(g);
            if(!is_a<status::optimal>(g.get_status())) continue;
            const double rhs = std::max(g.get_dual_bound(), g.get_solution_value()) + margins[md(rng)];
            cp::raw_cut c; c.sense = 'L'; c.rhs = rhs;
            for(auto [j, w] : A) { c.ind.push_back(g.native_id(vs[(std::size_t)j])); c.val.push_back(w); }
            bool separates = false;
            for(const auto & q : pts) if(c.violation(q) > 1e-6) { separates = true; break; }
            if(!separates) continue;
            if(bin(rng)) { for(auto & w : c.val) w = -w; c.rhs = -c.rhs; c.sense = 'G'; }
            if(c.violation(best) > 1e-6) { ++invalid; std::println(stderr, "   GUROBI oracle disagrees with its own optimum by {}", c.violation(best)); continue; }
            pool.push_back(std::move(c));
        }
        for(const int style : {0, 1}) {
            V m; cfg(m); build_family(m, fam, seed);
            std::atomic<long> offered{0}, calls{0}, oneterm{0};
            recipe_opts opt; opt.fold = opt.strip_zero = opt.skip_empty = opt.drop_satisfied = false;
            m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
                const long k = ++calls;
                if(style == 1 && k < 30) return;
                int given = 0;
                for(const auto & cu : pool) {
                    if(style == 1 && given == 3) break;
                    if(cu.violation(x) <= 1e-6) continue;
                    const auto r = cf::sanitize(cu, x);
                    if(!r) continue;
                    add(*r); ++offered; ++given; if(r->ind.size() == 1) ++oneterm;
                }
            }, opt);
            const double t = cp::timed_solve(m);
            const double v = cf::value_of(m);
            const bool ok = right(v) && is_a<status::optimal>(m.get_status());
            ++cases; wrong += !ok; adds += offered; if(!ok && right(hookfree)) ++wrong_when_nohook_right;
            std::println(stderr, "{} seed={} style={} {} {} | truth {} hook-free {} {} | pool={} calls={} offered={} oneterm={} errs={}", fam, seed, style, ok ? "OK   " : "WRONG", cp::summary(m, t), truth, hookfree, right(hookfree) ? "right" : "WRONG", pool.size(), calls.load(), offered.load(), oneterm.load(), m.last->native_err.load());
        }
    }
    std::println(stderr, "== {} pre0={} seeds={} hook-free wrong seeds={} cases={} wrong={} wrong where hook-free right={} adds={}", fam, pre0, seeds_run, nohook_wrong, cases, wrong, wrong_when_nohook_right, adds);
}
