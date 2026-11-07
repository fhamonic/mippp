// X3 managed cuts by release; X7 equality on a duplicate column by release; X2 at true defaults by depth-gated DFJ.
#include "verify_xpress_hook.hpp"
#include <sys/wait.h>
#include <unistd.h>
using namespace mippp; using namespace mippp::operators;
using std::chrono::duration;
template <class F>
static void isolated(const std::string & name, F f, unsigned seconds = 60) {
    std::fflush(stderr);
    const pid_t p = fork();
    if(p == 0) { alarm(seconds); vx::quiet_flush = true; f(); std::fflush(stderr); _exit(0); }
    int st = 0; waitpid(p, &st, 0);
    if(WIFSIGNALED(st)) std::println(stderr, "CRASH {}: killed by signal {}", name, WTERMSIG(st));
    else if(WEXITSTATUS(st)) std::println(stderr, "CHILD {}: exit {}", name, WEXITSTATUS(st));
    else std::println(stderr, "ALIVE {}", name);
}
static void x3() {
    for(const char * spec : {"R1", "R1,add2", "R2cr", "R2"}) {
        M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
        auto a = cp::build_atsp(m, 26, false, 5);
        auto s = std::make_shared<cp::stats>();
        auto & c = vx::attach(m, cp::watched([&a](const auto & x, const cp::add_fn & add) { for(const auto & cut : a.separate(x)) add(cut); }, s), vx::parse(spec));
        const double t = cp::timed_solve(m);
        std::println(stderr, "X3 ATSP n=26 {}: {} | calls={} cuts={} errs={} | fired={} rows_first={} rows_max={} CUTS_max={} max_rounds={} nodes={}", spec, cp::summary(m, t), s->calls.load(), s->cuts.load(), s->add_errors.load(), c.fired.load(), c.rows_first, c.rows_max, c.cuts_max, c.max_rounds, c.nodes.size());
    }
    M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(30));
    auto a = cp::build_atsp(m, 26, false, 5);
    const double t = cp::timed_solve(m);
    std::println(stderr, "X3 ATSP n=26 no hook: {}", cp::summary(m, t));
}
// Duplicates: a tree block keeps the search alive; t, s identical in every row and in the objective.
template <class Mo> static void tree(Mo & m, unsigned seed = 3) {
    const std::size_t cols = 40, rows = 5;
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> A(1, 100), R(0, 50);
    std::vector<std::vector<double>> a(rows, std::vector<double>(cols));
    for(auto & r : a) for(auto & v : r) v = A(rng);
    std::vector<double> c(cols), cap(rows);
    for(std::size_t j = 0; j < cols; ++j) { double s = 0; for(auto & r : a) s += r[j]; c[j] = std::floor(s / static_cast<double>(rows) + R(rng)); }
    for(std::size_t i = 0; i < rows; ++i) { double s = 0; for(double v : a[i]) s += v; cap[i] = std::floor(s / 2); }
    auto x = m.add_binary_variables(cols);
    const auto ids = std::views::iota(std::size_t{0}, cols);
    m.set_maximization();
    m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
    for(std::size_t i = 0; i < rows; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * x(j); }) <= cap[i]);
}
static void x7_case(const std::string & name, const std::string & spec, int family, std::function<cp::raw_cut(int, int, int)> make, bool dual0) {
    M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(15));
    if(dual0) vx::setint(m, vx::C_MIPDUALREDUCTIONS, 0);
    tree(m);
    auto b = m.add_binary_variables(3);
    const std::size_t z = 0;
    if(family == 0) { m.add_constraint(b(z) + b(z + 1) + b(z + 2) == 1); m.set_objective_coefficient(b(z), 10); m.set_objective_coefficient(b(z + 1), 10); m.set_objective_coefficient(b(z + 2), 7); }
    if(family == 1) { m.add_constraint(b(z) + b(z + 1) - 2.0 * b(z + 2) == 0); m.set_objective_coefficient(b(z), 10); m.set_objective_coefficient(b(z + 1), 10); m.set_objective_coefficient(b(z + 2), -3); }
    if(family == 2) { m.add_constraint(b(z) + b(z + 1) <= 1); m.set_objective_coefficient(b(z), 10); m.set_objective_coefficient(b(z + 1), 10); m.set_objective_coefficient(b(z + 2), 7); }
    const cp::raw_cut cut = make(m.native_id(b(z)), m.native_id(b(z + 1)), m.native_id(b(z + 2)));
    bool once = false;
    vx::recipe r = vx::parse(spec); r.drop_satisfied = false;
    auto s = std::make_shared<cp::stats>();
    auto & c = vx::attach(m, cp::watched([&](const auto &, const cp::add_fn & add) { if(!once) { once = true; add(cut); } }, s), r);
    const double t = cp::timed_solve(m);
    std::println(stderr, "   {} [{}] dualred={}: {} | calls={} cuts={} errs={} (last {}) | native={} status{{{}}} COLS=[{},{}] input={}", name, spec, dual0 ? 0 : 1, cp::summary(m, t), s->calls.load(), s->cuts.load(), s->add_errors.load(), s->last_error, c.added_native.load(), [&] { std::string q; for(auto [k, v] : c.presolve_status) q += std::format("{}:{} ", k, v); return q; }(), c.cols_seen_lo, c.cols_seen_hi, c.input_cols);
}
static void x7(const std::string & only) {
    struct K { std::string name; int family; bool valid; std::function<cp::raw_cut(int, int, int)> make; };
    const std::vector<K> ks = {
        // family 0: t + s + q == 1, duplicates t,s merge: a truly valid equality has equal coefficients on t and s
        {"VALID E t + s + q == 1 (fam 0)", 0, true, [](int t, int s, int q) { return cp::raw_cut{{t, s, q}, {1, 1, 1}, 'E', 1}; }},
        {"VALID E 2t + 2s + 2q == 2 (fam 0)", 0, true, [](int t, int s, int q) { return cp::raw_cut{{t, s, q}, {2, 2, 2}, 'E', 2}; }},
        // family 1: t + s == 2 y: every integer point has t = s = y, so these are valid with unequal coefficients
        {"VALID E t - s == 0 (fam 1, t + s == 2 y)", 1, true, [](int t, int s, int) { return cp::raw_cut{{t, s}, {1, -1}, 'E', 0}; }},
        {"VALID E t - y == 0 (fam 1)", 1, true, [](int t, int, int y) { return cp::raw_cut{{t, y}, {1, -1}, 'E', 0}; }},
        {"VALID E t + s - 2 y == 0 (fam 1, the row)", 1, true, [](int t, int s, int y) { return cp::raw_cut{{t, s, y}, {1, 1, -2}, 'E', 0}; }},
        {"VALID E s - y == 0 (fam 1)", 1, true, [](int, int s, int y) { return cp::raw_cut{{s, y}, {1, -1}, 'E', 0}; }},
        {"VALID L t - s <= 0 (fam 1, inequality)", 1, true, [](int t, int s, int) { return cp::raw_cut{{t, s}, {1, -1}, 'L', 0}; }},
        {"VALID G t - s >= 0 (fam 1, inequality)", 1, true, [](int t, int s, int) { return cp::raw_cut{{t, s}, {1, -1}, 'G', 0}; }},
        // family 2: t + s <= 1, q independent: t,s duplicates; t == 0 cuts off t=1
        {"INVALID E t == 0 (fam 2)", 2, false, [](int t, int, int) { return cp::raw_cut{{t}, {1}, 'E', 0}; }},
        {"INVALID E t == 1 (fam 2)", 2, false, [](int t, int, int) { return cp::raw_cut{{t}, {1}, 'E', 1}; }},
        {"VALID L t + 2 s <= 2 (fam 2, unequal)", 2, true, [](int t, int s, int) { return cp::raw_cut{{t, s}, {1, 2}, 'L', 2}; }},
    };
    int i = 0;
    for(const auto & k : ks) { ++i; for(const char * spec : {"R1,no_split_eq", "R2,no_split_eq", "R1", "R2", "R2,no_split_eq,ignore_status", "R2cr,no_split_eq,ignore_status"}) for(const bool dual0 : {false, true}) {
        if(!only.empty() && only != std::to_string(i)) continue;
        if(const char * e = std::getenv("X7_SPEC")) if(std::string(e) != spec) continue;
        if(const char * e = std::getenv("X7_DUAL")) if(std::atoi(e) != (dual0 ? 0 : 1)) continue;
        const std::string nm = std::format("X7 {} [{}] dualred={}", k.name, spec, dual0 ? 0 : 1);
        isolated(nm, [&] { x7_case(k.name, spec, k.family, k.make, dual0); }, 40);
    } }
}
// X2 at true defaults: DFJ only below depth D. If the global flag matters, a cut from one subtree serves the others.
static void x2_depth(int n, int D, const std::string & spec, const std::string & settings) {
    M m; m.set_thread_limit(1); m.set_time_limit(duration<double>(60));
    auto a = cp::build_atsp(m, n, false, 5);
    if(settings == "PRESOLVE0") vx::setint(m, vx::C_PRESOLVE, 0);
    if(settings == "MIPRESTART0") vx::setint(m, vx::C_MIPRESTART, 0);
    auto s = std::make_shared<cp::stats>();
    auto & c = vx::attach(m, cp::watched([&a, D](const auto & x, const cp::add_fn & add) { if(vx::current_depth < D) return; for(const auto & cut : a.separate(x)) add(cut); }, s), vx::parse(spec));
    const double t = cp::timed_solve(m);
    std::println(stderr, "X2 depth>={} n={} {} settings={}: {} | calls={} cuts={} | fired={} nodes={} CUTS_max={} rows_max={} depth={}", D, n, spec, settings, cp::summary(m, t), s->calls.load(), s->cuts.load(), c.fired.load(), c.nodes.size(), c.cuts_max, c.rows_max, c.max_depth);
}
int main(int argc, char ** argv) {
    const std::string w = argc > 1 ? argv[1] : "";
    { M m; std::println(stderr, "release {}", vx::version(m)); }
    vx::quiet_flush = true;
    if(w == "x3") x3();
    if(w == "x7") x7(argc > 2 ? argv[2] : "");
    if(w == "x2") for(const char * st : {"default", "PRESOLVE0", "MIPRESTART0"}) for(const char * sp : {"R1,add2", "R1,add2,local", "R1,add2,first2", "R1,add2,first2,local", "R1", "R1,local", "R1,first2", "R1,first2,local", "R2", "R2cr"}) x2_depth(26, 2, sp, st);
}
