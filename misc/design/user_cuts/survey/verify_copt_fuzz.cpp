// D, oracle fuzz. Every cut is valid by construction: a random sparse a, rhs = the proved max of a'x over the same
// model, found by an auxiliary MILP with zero gap; so tight at some feasible point, often the optimum. Added whenever
// violated, through cf::sanitize, at every call including nodes about to be pruned. Cuts must not change the optimum.
// usage: verify_copt_fuzz <family nd|cg|mkp|gi> <seed from> <seed to> [P]
#include "verify_copt_common.hpp"
using namespace std::chrono;
namespace {
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
        // general integers and continuous columns with an aggregated link
        auto x = m.add_integer_variables(cols, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 3});
        auto s = m.add_variables(cols, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 10});
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j) + 0.3 * c[j] * s(j); }));
        for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= 3.0 * b[i]);
        for(std::size_t j = 0; j < cols; ++j) m.add_constraint(s(j) - 2.5 * x(j) <= 0.0);
        m.add_constraint(xsum(ids, [&](std::size_t j) { return 1.0 * s(j); }) <= 60.0);
    }
}
}
int main(int argc, char ** argv) {
    const std::string fam = argv[1];
    const unsigned from = std::atoi(argv[2]), to = std::atoi(argv[3]);
    const int P = argc > 4 ? std::atoi(argv[4]) : 40;
    long total_fail = 0, total_cases = 0, total_adds = 0, total_aux = 0, total_oneterm = 0, total_prunable = 0, total_invalid_base = 0;
    for(unsigned seed = from; seed <= to; ++seed) {
        // baseline
        double base; std::string bs; std::vector<double> best;
        V b; b.iparam("Threads", 1); build_family(b, fam, seed); b.set_time_limit(duration<double>(30.0));
        { const double t = cp::timed_solve(b); base = b.get_solution_value(); bs = cp::summary(b, t); }
        if(!is_a<status::optimal>(b.get_status())) { std::println(stderr, "{} seed={} no baseline: {}", fam, seed, bs); continue; }
        { auto s = b.get_solution(); best.resize(b.num_variables()); for(auto v : b.variables()) best[(std::size_t)b.native_id(v)] = s[v]; }
        // the oracle
        std::vector<cp::raw_cut> pool;
        V o; o.iparam("Threads", 1); build_family(o, fam, seed); o.set_time_limit(duration<double>(5.0));
        o.set_optimality_tolerance(0.0);
        o.native_api()._check(nullptr, o.native_api().SetDblParam(o.native_model().second, "AbsGap", 0.0));
        auto first = *o.variables().begin();
        std::vector<decltype(first)> vs; for(auto v : o.variables()) vs.push_back(v);
        std::mt19937 rng(seed * 131u + 7u);
        // Fractional points of this very search, harvested by a no-op hook, so that cuts can be violated.
        std::vector<std::vector<double>> pts;
        {
            V h; h.iparam("Threads", 1); build_family(h, fam, seed); h.set_time_limit(duration<double>(30.0));
            long seen = 0;
            recipe_opts op;
            h.attach([&](const std::vector<double> & x, const cp::add_fn &) { ++seen; if(pts.size() < 400 && (seen < 30 || seen % 7 == 0)) pts.push_back(x); }, op);
            cp::timed_solve(h);
        }
        if(pts.empty()) { std::println(stderr, "{} seed={} NO POWER: the hook never fired", fam, seed); continue; }
        std::uniform_int_distribution<std::size_t> ptk(0, pts.size() - 1);
        const int terms[] = {1, 2, 2, 3, 4, 6, 8, 15};
        std::uniform_int_distribution<int> termc(0, 7), md(0, 5), bin(0, 1), mult(1, 3);
        const double margins[] = {0, 0, 0, 1e-9, 1e-6, 1e-3};
        long tried = 0;
        // objective coefficients, read back from the model, in the direction of improvement
        std::vector<double> objc(vs.size());
        {
            using getcol_fn = int (*)(c::copt_prob *, const char *, int, const int *, double *);
            auto getcol = cp::symf<getcol_fn>(o.native_api().library_path(), "COPT_GetColInfo");
            std::vector<int> ids(vs.size()); for(std::size_t i = 0; i < vs.size(); ++i) ids[i] = o.native_id(vs[i]);
            if(getcol(o.native_model().second, "Obj", (int)ids.size(), ids.data(), objc.data())) { std::println(stderr, "GetColInfo failed"); return 2; }
            if(fam == "nd") for(auto & w : objc) w = -w;
        }
        std::uniform_real_distribution<double> noise(0.8, 1.2), coin(0.0, 1.0);
        for(int t = 0; t < 6 * P && (int)pool.size() < P; ++t) {
            const auto & p = pts[ptk(rng)];
            std::vector<std::size_t> frac;
            for(std::size_t i = 0; i < vs.size(); ++i) { const double w = p[(std::size_t)o.native_id(vs[i])]; if(std::fabs(w - std::round(w)) > 1e-4) frac.push_back(i); }
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
                for(std::size_t q = 0; q < nt; ++q) { const double w = p[(std::size_t)o.native_id(vs[frac[q]])]; const double fr = w - std::floor(w); A.emplace_back((int)frac[q], (fr >= 0.5 ? 1.0 : -1.0) * mult(rng)); }
            }
            if(A.empty()) continue;
            o.set_maximization();
            o.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * vs[(std::size_t)e.first]; }));
            ++total_aux; ++tried;
            cp::timed_solve(o);
            if(!is_a<status::optimal>(o.get_status())) continue;
            const double rhs = std::max(o.get_dual_bound(), o.get_solution_value()) + margins[md(rng)];
            cp::raw_cut c; c.sense = 'L'; c.rhs = rhs;
            for(auto [j, w] : A) { c.ind.push_back(o.native_id(vs[(std::size_t)j])); c.val.push_back(w); }
            bool separates = false;
            for(const auto & q : pts) if(c.violation(q) > 1e-6) { separates = true; break; }
            if(!separates) continue;
            if(bin(rng)) { for(auto & w : c.val) w = -w; c.rhs = -c.rhs; c.sense = 'G'; }
            if(c.violation(best) > 1e-6) {
                ++total_invalid_base;
                V f; f.iparam("Threads", 1); build_family(f, fam, seed); f.set_time_limit(duration<double>(10.0));
                f.set_optimality_tolerance(0.0);
                f.native_api().SetDblParam(f.native_model().second, "AbsGap", 0.0);
                auto ff = *f.variables().begin(); std::vector<decltype(ff)> fv; for(auto v : f.variables()) fv.push_back(v);
                f.set_maximization();
                f.set_objective(xsum(A, [&](const std::pair<int, double> & e) { return e.second * fv[(std::size_t)e.first]; }));
                cp::timed_solve(f);
                double abest = 0; for(auto [j, w] : A) abest += w * best[(std::size_t)o.native_id(vs[(std::size_t)j])];
                { FILE * fo = std::fopen(std::format("vout_copt_falseopt_{}_{}_{}.txt", fam, seed, t).c_str(), "w"); for(auto [j, w] : A) std::fprintf(fo, "%d %.17g\n", j, w); std::fclose(fo); }
                std::println(stderr, "   HARNESS: t={} mode={} terms={} sense={} margin rhs={} violated by the baseline optimum by {} | a at the baseline optimum={} | reused aux value={} dual={} nodes={} | fresh aux value={} dual={} nodes={} status {}", t, mode, A.size(), c.sense, rhs, c.violation(best), abest, o.get_solution_value(), o.get_dual_bound(), o.get_node_count(), f.get_solution_value(), f.get_dual_bound(), f.get_node_count(), cp::status_name(f.get_status()));
                continue;
            }
            pool.push_back(std::move(c));
        }
        // the search with all of them available
        for(const int style : {0, 1}) {   // 0: every violated cut at every call, 1: starts at call 30, at most 3 per call
            V m; m.iparam("Threads", 1); build_family(m, fam, seed); m.set_time_limit(duration<double>(30.0));
            std::atomic<long> offered{0}, calls{0}, oneterm{0}, prunable{0};
            recipe_opts opt; opt.fold = opt.strip_zero = opt.skip_empty = opt.drop_satisfied = false;
            const bool maxim = true && fam != "nd";
            m.extra_mask = c::COPT_CBCONTEXT_MIPNODE;
            m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
                const long k = ++calls;
                if(style == 1 && k < 30) return;
                int given = 0;
                for(const auto & cu : pool) {
                    if(style == 1 && given == 3) break;
                    if(cu.violation(x) <= 1e-6) continue;
                    const auto r = cf::sanitize(cu, x);
                    if(!r) continue;
                    add(*r); ++offered; ++given;
                    if(r->ind.size() == 1) ++oneterm;
                }
            }, opt);
            const double t = cp::timed_solve(m);
            const double v = cf::value_of(m);
            const bool ok = cf::same(v, base) && is_a<status::optimal>(m.get_status());
            ++total_cases; total_fail += !ok; total_adds += offered; total_oneterm += oneterm;
            std::println(stderr, "{} seed={} style={} {} {} | base {} | pool={} calls={} offered={} oneterm={} errs={} last_rc={}", fam, seed, style, ok ? "OK   " : "WRONG", cp::summary(m, t), bs, pool.size(), calls.load(), offered.load(), oneterm.load(), m.last->native_err.load(), m.last->last_rc.load());
        }
    }
    std::println(stderr, "== {} cases={} wrong={} adds={} one-term adds={} aux solves={} harness-invalid={}", fam, total_cases, total_fail, total_adds, total_oneterm, total_aux, total_invalid_base);
}
