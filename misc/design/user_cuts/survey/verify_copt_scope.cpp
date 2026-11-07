// O1: a cut valid only in the worse subtree, on cut_common's gadgets, with my own S1b cut.
// O9: a two-term cut that excludes the optimal tour, accepted in the tree; is the final point bound by it?
#include "verify_copt_common.hpp"
using namespace std::chrono;
namespace {
struct setting { std::string name; std::vector<std::pair<const char *, int>> p; };
void base_settings(V & m) {
    for(const char * n : {"Presolve", "HeurLevel", "PreRootHeurLevel", "RoundingHeurLevel", "DivingHeurLevel", "SubMipHeurLevel", "FAPHeurLevel", "ConflictAnalysis"}) m.iparam(n, 0);
    m.iparam("Threads", 1); m.iparam("MipTasks", 1);
}
void s1(const setting & st, bool flip, bool partial, int k, bool use_base) {
    V m; if(use_base) base_settings(m); else m.iparam("Threads", 1);
    for(auto [a, b] : st.p) m.iparam(a, b);
    auto g = cp::build_gadgets(m, k, flip); m.set_time_limit(duration<double>(20.0));
    auto tr = std::make_shared<cp::scope_tracker>();
    if(partial) {
        // Gadget 0 at its worse value plus gadget 1 at its best integer value: holds when y0 is at
        // the worse side, whatever else; cuts the better side off (gadget 0 at 6 c0 against 4 c0).
        cp::raw_cut c; c.sense = 'L';
        for(int j : {g.za[0], g.zb[0], g.za[1], g.zb[1]}) { c.ind.push_back(j); c.val.push_back(1.0); }
        c.rhs = g.c[0] * 4 + g.c[1] * std::max(g.total0[1], g.total1[1]);
        tr->cut = c;
    } else tr->cut = g.invalid_global_cut();
    tr->trigger = g.worse_state();
    tr->state = [&g](const std::vector<double> & x) { return g.state(x); };
    long rc = 0;
    recipe_opts o; o.fold = o.strip_zero = o.skip_empty = o.drop_satisfied = false;
    m.attach([&](const std::vector<double> & x, const cp::add_fn & add) { if(tr->wants_add(x)) rc = add(tr->cut); }, o);
    const double t = cp::timed_solve(m);
    const double v = cf::value_of(m);
    std::println(stderr, "S1{} {:34} flip={} k={}: {} known opt {} => {} | rc={} {} relax={}", partial ? "b" : " ", st.name, flip, k, cp::summary(m, t), g.opt, std::fabs(v - g.opt) < 1e-4 ? "KEPT" : "LOST", rc, tr->line(), m.last->relax.load());
}
}
int main(int argc, char ** argv) {
    const std::string exp = argv[1];
    if(exp == "o1") {
        const std::vector<setting> sts{
            {"noroot (RootCutLevel=0)", {{"RootCutLevel", 0}}},
            {"noroot TreeCutLevel=1", {{"RootCutLevel", 0}, {"TreeCutLevel", 1}}},
            {"CutLevel=0 TreeCutLevel=1", {{"CutLevel", 0}, {"TreeCutLevel", 1}}},
            {"RootCutRounds=1", {{"RootCutRounds", 1}}},
            {"RootCutRounds=3", {{"RootCutRounds", 3}}},
            {"auto cuts", {}},
        };
        for(const auto & st : sts) for(bool partial : {false, true}) for(bool flip : {false, true}) s1(st, flip, partial, 10, true);
        // At true defaults, on the same gadgets, no settings at all
        for(bool partial : {false, true}) for(bool flip : {false, true}) s1({"true defaults", {}}, flip, partial, 10, false);
        for(bool partial : {false, true}) for(bool flip : {false, true}) s1({"true defaults k=14", {}}, flip, partial, 14, false);
    }
    if(exp == "o9") {
        for(const int n : {26, 30}) for(const unsigned seed : {5u, 7u}) {
            // the optimum, no hook
            std::vector<double> best; double bv;
            std::vector<int> arcs;
            cp::atsp a0;
            {
                V m; m.iparam("Threads", 1); a0 = cp::build_atsp(m, n, false, seed); m.set_time_limit(duration<double>(60.0));
                cp::timed_solve(m); bv = m.get_solution_value();
                auto s = m.get_solution();
                best.resize(m.num_variables());
                const auto vars = m.variables();
                for(auto v : vars) best[(std::size_t)m.native_id(v)] = s[v];
                for(int i = 0; i < n; ++i) for(int j = 0; j < n; ++j) if(i != j && best[(std::size_t)a0.col[(std::size_t)i][(std::size_t)j]] > 0.5) arcs.push_back(a0.col[(std::size_t)i][(std::size_t)j]);
                std::println(stderr, "o9 n={} seed={} baseline {} tour arcs {}", n, seed, cp::summary(m, 0.0), arcs.size());
            }
            for(long delay : {0L, 50L, 300L}) {
                V m; m.iparam("Threads", 1); auto a = cp::build_atsp(m, n, false, seed); m.set_time_limit(duration<double>(60.0));
                std::vector<cp::raw_cut> accepted;
                long calls = 0;
                std::atomic<long> inc = 0, inc_viol = 0;
                recipe_opts o; o.fold = o.strip_zero = o.skip_empty = o.drop_satisfied = false;
                // the cut: arcs a, b of the optimal tour cannot both be used. Added once when the LP violates it.
                m.extra_mask = c::COPT_CBCONTEXT_INCUMBENT;
                m.attach([&](const std::vector<double> & x, const cp::add_fn & add) {
                    if(++calls <= delay || !accepted.empty()) return;
                    for(std::size_t p = 0; p < arcs.size() && accepted.empty(); ++p) for(std::size_t q = p + 1; q < arcs.size(); ++q) {
                        cp::raw_cut cc{{arcs[p], arcs[q]}, {1.0, 1.0}, 'L', 1.0};
                        if(cc.violation(x) > 1e-2) { const int rc = add(cc); if(rc == 0) accepted.push_back(cc); break; }
                    }
                }, o);
                const double t = cp::timed_solve(m);
                double viol = NAN, val = cf::value_of(m);
                if(!accepted.empty()) {
                    auto s = m.get_solution();
                    std::vector<double> pt(m.num_variables());
                    for(auto v : m.variables()) pt[(std::size_t)m.native_id(v)] = s[v];
                    viol = accepted[0].violation(pt);
                }
                std::println(stderr, "   delay={} {} accepted={} at call>{} final value {} vs opt {}; final point violates the cut by {} relax={} native_adds={}", delay, cp::summary(m, t), accepted.size(), delay, val, bv, viol, calls, m.last->native_adds.load());
            }
        }
    }
}
