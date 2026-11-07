// P: does a cut outlive the solve? Market split with CG cuts: a status the search must decide, infeasible included.
#include "verify_xpress_hook.hpp"
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
using cut = cp::raw_cut;
static void persist() {
    for(const char * spec : {"R1", "R1,first2", "R2", "R2cr"}) for(const bool defaults : {true, false}) {
        M m;
        if(defaults) { m.set_thread_limit(1); for(auto [c, v] : std::vector<std::pair<int, int>>{{vx::C_MIPPRESOLVE, 0}, {8238, 0}, {vx::C_COVERCUTS, 0}, {vx::C_GOMCUTS, 0}, {vx::C_TREECOVERCUTS, 0}, {vx::C_TREEGOMCUTS, 0}, {vx::C_HEUREMPHASIS, 0}, {vx::C_FEASIBILITYPUMP, 0}, {vx::C_HEURBEFORELP, 0}, {vx::C_SYMMETRY, 0}, {vx::C_PRECOMPONENTS, 0}, {vx::C_MIPDUALREDUCTIONS, 0}}) vx::setint(m, c, v); }
        else { m.set_thread_limit(1); for(auto [c, v] : std::vector<std::pair<int, int>>{{vx::C_PRESOLVE, 0}, {vx::C_MIPPRESOLVE, 0}, {8238, 0}, {vx::C_COVERCUTS, 0}, {vx::C_GOMCUTS, 0}, {vx::C_TREECOVERCUTS, 0}, {vx::C_TREEGOMCUTS, 0}, {vx::C_HEUREMPHASIS, 0}, {vx::C_FEASIBILITYPUMP, 0}, {vx::C_HEURBEFORELP, 0}, {vx::C_SYMMETRY, 0}, {vx::C_PRECOMPONENTS, 0}, {vx::C_MIPRESTART, 0}, {vx::C_MIPDUALREDUCTIONS, 0}}) vx::setint(m, c, v); }
        auto g = cp::build_gadgets(m, 10, false);
        m.set_time_limit(duration<double>(20));
        auto s = std::make_shared<cp::stats>();
        const cut h = g.hull_cut(3);
        bool once = false, first_solve = true;
        auto & c = vx::attach(m, cp::watched([&](const auto & x, const cp::add_fn & add) { if(first_solve && !once && h.violation(x) > 1e-4) { once = true; add(h); } }, s), vx::parse(spec));
        const double t = cp::timed_solve(m);
        const std::string a = cp::summary(m, t);
        first_solve = false;
        for(std::size_t r = 12; r < 16; ++r) m.set_constraint_upper_bound(g.rows[r], m.get_constraint_upper_bound(g.rows[r]) + 50);
        const double t2 = cp::timed_solve(m);
        const double v = cf::value_of(m);
        std::println(stderr, "P [{}] {}: first {} native={} | after raising rows of gadget 3 by 50: {} new opt {} => {}", spec, defaults ? "presolve on" : "presolve off", a, c.added_native.load(), cp::summary(m, t2), g.opt + 100, std::fabs(v - (g.opt + 100)) < 1e-4 ? "cut did NOT persist" : "cut PERSISTED");
    }
}
// Market split: a x == b, x binary. Integer status is decided by search, so a cut can only speed it. Multiples of rows give CG cuts.
static void market(int rows, int cols, const std::vector<std::string> & specs) {
    int infeasible = 0, feasible = 0;
    for(int seed = 1; seed <= 24; ++seed) {
        const auto R = static_cast<std::size_t>(rows); const auto C = static_cast<std::size_t>(cols);
        std::mt19937 rng(static_cast<unsigned>(seed)); std::uniform_int_distribution<int> A(0, 99), O(1, 30);
        std::vector<std::vector<double>> a(R, std::vector<double>(C)); std::vector<double> b(R), c(C);
        for(auto & r : a) for(auto & v : r) v = A(rng);
        for(std::size_t i = 0; i < R; ++i) { double s = 0; for(double v : a[i]) s += v; b[i] = std::floor(s / 2); }
        for(auto & v : c) v = O(rng);
        const auto build = [&](M & m) {
            m.set_thread_limit(1); m.set_time_limit(duration<double>(30));
            auto x = m.add_binary_variables(C);
            const auto ids = std::views::iota(std::size_t{0}, C);
            m.set_maximization(); m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
            for(std::size_t i = 0; i < R; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) == b[i]);
            std::vector<int> col; for(std::size_t j = 0; j < C; ++j) col.push_back(m.native_id(x(j)));
            return col;
        };
        std::string truth; double tv = NAN; long tn = 0;
        { M m; build(m); const double t = cp::timed_solve(m); truth = cp::status_name(m.get_status()); tv = cf::value_of(m); tn = static_cast<long>(m.get_node_count()); (void)t; }
        const bool decided = truth.starts_with("optimal") || truth.starts_with("infeasible");
        if(!decided) continue;
        (truth.starts_with("infeasible") ? infeasible : feasible)++;
        for(const auto & spec : specs) {
            M m; const auto col = build(m);
            auto gen = std::make_shared<std::mt19937>(static_cast<unsigned>(seed) * 131u);
            auto s = std::make_shared<cp::stats>();
            auto & hook = vx::attach(m, cp::watched([a, b, col, gen, R, C](const std::vector<double> & p, const cp::add_fn & add) {
                const int ds[] = {2, 3, 4, 5, 7};
                std::uniform_int_distribution<std::size_t> D(0, 4);
                int added = 0;
                for(int att = 0; att < 200 && added < 3; ++att) {
                    const int d = ds[D(*gen)]; std::uniform_int_distribution<int> K(-(d - 1), d - 1);
                    std::vector<double> l(R); for(auto & v : l) v = static_cast<double>(K(*gen)) / d;
                    cut q{{}, {}, 'L', 0}; double rhs = 0; for(std::size_t i = 0; i < R; ++i) rhs += l[i] * b[i];
                    q.rhs = std::floor(rhs + 1e-9);
                    for(std::size_t j = 0; j < C; ++j) { double t = 0; for(std::size_t i = 0; i < R; ++i) t += l[i] * a[i][j]; q.ind.push_back(col[j]); q.val.push_back(std::floor(t + 1e-9)); }
                    if(q.violation(p) > 1e-4) { add(q); ++added; }
                }
            }, s), vx::parse(spec));
            const double t = cp::timed_solve(m);
            const std::string st = cp::status_name(m.get_status());
            const double v = cf::value_of(m);
            const bool ok = st == truth && (truth.starts_with("infeasible") || cf::same(v, tv));
            std::println(stderr, "{} market {}x{} seed={} [{}]: truth {} {} nodes {} | {} | calls={} native={} dropped={} errs={}", ok ? "OK " : "BAD", rows, cols, seed, spec, truth, tv, tn, cp::summary(m, t), s->calls.load(), hook.added_native.load(), hook.dropped.load(), s->add_errors.load());
        }
    }
    std::println(stderr, "market {}x{}: decided feasible {}, infeasible {}", rows, cols, feasible, infeasible);
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    vx::quiet_flush = true;
    if(w == "persist") persist();
    if(w == "market") { market(3, 24, {"R1", "R1,first2", "R2", "R2cr"}); market(4, 30, {"R1", "R2"}); }
}
