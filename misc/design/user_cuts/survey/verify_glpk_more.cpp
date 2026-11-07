// Targeted cases the conformance set has no power on.
#include "verify_glpk_inst.hpp"
using namespace vg;
static const cp::separator noop = [](const std::vector<double> &, const cp::add_fn &) {};
static cp::separator dfj(const cp::atsp & a) { return [a](const std::vector<double> & x, const cp::add_fn & add) { for(const auto & c : a.separate(x)) add(c); }; }

// (a) Stop inside the tree, after cuts landed, then solve the same model again.
static void stop_resume(int n, unsigned seed, int stop_after) {
    double want;
    { M f; auto a = cp::build_atsp(f, n, false, seed); f.p.presolve_off = true; attach(f, dfj(a)); f.set_time_limit(std::chrono::duration<double>(60)); f.solve(); want = f.get_solution_value(); std::println(stderr, "fresh recipe solve: {} value={} nodes={} it={}", st(f), want, f.closed, f.iters); }
    M m; auto a = cp::build_atsp(m, n, false, seed);
    m.p.presolve_off = true;
    auto & api = m.native_api();
    const int rows0 = api.get_num_rows(m.native_model());
    long calls = 0, inner_cuts = 0; int rows_in_tree = 0;
    auto & c = attach(m, [&, a](const std::vector<double> & x, const cp::add_fn & add) { ++calls; for(const auto & k : a.separate(x)) { add(k); ++inner_cuts; } });
    c.also = [&](glp_tree * t, int why) {
        if(why == R_ROWGEN) rows_in_tree = std::max(rows_in_tree, api.get_num_rows(api.ios_get_prob(t)) - rows0);
        if(why == R_CUTGEN && calls >= stop_after) api.ios_terminate(t);
    };
    m.set_time_limit(std::chrono::duration<double>(60));
    m.solve();
    std::println(stderr, "stopped after {} calls, {} cuts offered, max extra rows in the tree {}: {} ret={:#x} rows on P after={} (before {}), has value {}", calls, inner_cuts, rows_in_tree, st(m), m.native_ret, api.get_num_rows(m.native_model()), rows0, m.get_solution_value());
    c.also = nullptr;
    const long before = calls;
    m.solve();
    std::println(stderr, "re-solve: {} value={} (fresh {}) {} | calls in it {} rows on P after={} nodes={} it={}", st(m), m.get_solution_value(), want, std::fabs(m.get_solution_value() - want) < 1e-6 ? "SAME" : "DIFFERENT", calls - before, api.get_num_rows(m.native_model()), m.closed, m.iters);
}

// (b) Does the time limit hold? Root LP of a large MTZ model against a limit just above it.
static void overshoot(int n, double factor) {
    double root;
    { M m; cp::build_atsp(m, n, false, 1); auto & N = natives::of(m); N.scale_prob(m.native_model(), SF_PATH); N.adv_basis(m.native_model(), 0); v1::glp_smcp s; m.native_api().init_smcp(&s); s.msg_lev = v1::GLP_MSG_OFF; const auto t0 = clk::now(); m.native_api().simplex(m.native_model(), &s); root = std::chrono::duration<double>(clk::now() - t0).count(); std::println(stderr, "atsp n={} root LP alone {:.3f}s rows {}", n, root, m.native_api().get_num_rows(m.native_model())); }
    const double limit = root * factor;
    for(int kind = 0; kind < 3; ++kind) {
        M m; cp::build_atsp(m, n, false, 1);
        m.set_time_limit(std::chrono::duration<double>(limit));
        if(kind) { m.p.presolve_off = true; m.p.subtract_root_time = kind == 2; attach(m, noop); }
        m.solve();
        std::println(stderr, "  {} limit {:.3f}s: {} total {:.3f}s = {:.2f} x limit, root part {:.3f}s", kind == 0 ? "wrapper path, presolve on, no hook" : kind == 1 ? "recipe, full tm_lim on the tree" : "recipe, root time subtracted", limit, st(m), m.solve_seconds, m.solve_seconds / limit, m.root_seconds);
    }
}

// (c) Micro models through the recipe path.
static void micro() {
    const auto show = [](const char * name, M & m) { m.p.presolve_off = true; attach(m, noop); m.set_time_limit(std::chrono::duration<double>(3)); m.solve(); std::println(stderr, "micro {}: {} value={} ret={:#x}", name, st(m), m.get_solution_value(), m.native_ret); };
    { M m; auto x = m.add_binary_variables(3); m.set_maximization(); m.set_objective(1.0 * x(std::size_t{0}) + 2.0 * x(std::size_t{1})); show("no rows, binaries", m); }
    { M m; auto x = m.add_variables(2, {.obj_coef = 1, .lower_bound = 0, .upper_bound = 1.5}); m.set_maximization(); m.set_objective(1.0 * x(std::size_t{0}) + 1.0 * x(std::size_t{1})); show("no rows, continuous", m); }
    { M m; auto x = m.add_binary_variables(1); m.set_variable_lower_bound(x(std::size_t{0}), 1); m.add_constraint(1.0 * x(std::size_t{0}) <= 1); show("single fixed column, one row", m); }
    { M m; auto x = m.add_binary_variables(2); m.add_constraint(x(std::size_t{0}) + x(std::size_t{1}) >= 3); show("LP infeasible", m); }
    { M m; auto x = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0}); auto y = m.add_binary_variables(1); m.set_maximization(); m.set_objective(1.0 * x(std::size_t{0}) + 1.0 * y(std::size_t{0})); m.add_constraint(1.0 * y(std::size_t{0}) <= 1); show("LP unbounded", m); }
    { M m; show("no columns (shortcut on)", m); }
}

// (d) Does a recipe solve leave anything behind for the next solve?
static std::string fingerprint(M & m) { return std::format("it={} closed={} {} value={:.9g}", m.iters, m.closed, st(m), m.get_solution_value()); }
static void build_fam(M & m, const std::string & f, cp::separator * sep) {
    if(f == "atsp") { auto a = cp::build_atsp(m, 14, false, 1); *sep = dfj(a); }
    else if(f == "mkp") { cp::build_mkp(m, 5, 30, 1); *sep = noop; }
    else if(f == "gadgets") { cp::build_gadgets(m, 8, false); *sep = noop; }
}
static void aftermath(const std::string & f) {
    M a; cp::separator sa; build_fam(a, f, &sa); a.solve();
    const auto pristine = fingerprint(a);
    M b; cp::separator sb; build_fam(b, f, &sb); b.p.presolve_off = true; attach(b, sb); b.solve();
    const auto with_cuts = fingerprint(b);
    b.solve();
    const auto recipe_twice = fingerprint(b);
    b.p.presolve_off = false; b.hook = nullptr; b.solve();
    const auto after = fingerprint(b);
    std::println(stderr, "aftermath {}: pristine presolve-on [{}] | recipe with cuts [{}] | recipe again [{}] | presolve-on after recipe [{}] => {}", f, pristine, with_cuts, recipe_twice, after, pristine == after ? "SAME PATH" : "PATH CHANGED");
}

// (g) What the 4.65+ route eliminates: is the value of an eliminated column known in the hook?
static void route_cols(const std::string & path) {
    M m;
    auto & api = m.native_api(); auto & n = natives::of(m);
    glp_prob * P = m.native_model();
    if(n.read_mps(P, 2, nullptr, path.c_str()) != 0) { std::println(stderr, "cannot read"); return; }
    const int cols = api.get_num_cols(P);
    for(int j = 1; j <= cols; ++j) { auto s = std::format("c{}", j); api.set_col_name(P, j, s.c_str()); }
    const auto l = api.library_path();
    auto alloc = cp::symf<void * (*)()>(l, "glp_npp_alloc_wksp");
    auto load = cp::symf<void (*)(void *, glp_prob *, int, int)>(l, "glp_npp_load_prob");
    auto pre1 = cp::symf<int (*)(void *, int)>(l, "glp_npp_preprocess1");
    auto build = cp::symf<void (*)(void *, glp_prob *)>(l, "glp_npp_build_prob");
    for(const int hard : {v1::GLP_OFF, v1::GLP_ON}) {
        const int term = api.term_out(v1::GLP_OFF);
        void * prep = alloc();
        load(prep, P, 3, v1::GLP_ON);
        const int rc = pre1(prep, hard);
        glp_prob * Q = api.create_prob();
        if(rc == 0) build(prep, Q);
        api.term_out(term);
        std::vector<char> kept(static_cast<std::size_t>(cols) + 1, 0);
        for(int j = 1; j <= api.get_num_cols(Q); ++j) { const char * nm = api.get_col_name(Q, j); if(nm) kept[static_cast<std::size_t>(std::atoi(nm + 1))] = 1; }
        int gone = 0, gone_fixed = 0, gone_binary = 0;
        for(int j = 1; j <= cols; ++j) if(!kept[static_cast<std::size_t>(j)]) { ++gone; if(api.get_col_type(P, j) == v1::GLP_FX) ++gone_fixed; if(api.get_col_kind(P, j) == v1::GLP_BV) ++gone_binary; }
        std::println(stderr, "route {} hard={}: P {}x{} -> Q {}x{}: columns gone {} of which fixed by P's own bounds {}, binary {}: {} have no value in the hook", path.substr(path.rfind('/') + 1), hard, api.get_num_rows(P), cols, api.get_num_rows(Q), api.get_num_cols(Q), gone, gone_fixed, gone_binary, gone - gone_fixed);
    }
}


// DFJ below the root only, one cut per call: every cut is local to its subtree. Against the root-only
// and everywhere variants: nodes, and that the optimum stays.
static void deep(int n, unsigned seed) {
    double base;
    { M f; cp::build_atsp(f, n, false, seed); f.set_time_limit(std::chrono::duration<double>(120)); f.solve(); base = f.get_solution_value(); std::println(stderr, "baseline n={} seed={}: {} value={} nodes={}", n, seed, st(f), base, f.closed); }
    for(const int mode : {0, 1, 2}) {
        M m; auto a = cp::build_atsp(m, n, false, seed);
        m.p.presolve_off = true;
        auto & N = natives::of(m);
        glp_tree * cur = nullptr;
        long calls = 0, cuts = 0; int maxlevel = 0;
        auto & c = attach(m, [&, a](const std::vector<double> & x, const cp::add_fn & add) {
            const int level = N.node_level(cur, N.curr_node(cur));
            maxlevel = std::max(maxlevel, level);
            ++calls;
            if(mode == 0 && level == 0) return;
            if(mode == 1 && level != 0) return;
            const auto found = a.separate(x, mode == 0 ? 1 : 5);
            for(const auto & k : found) { add(k); ++cuts; }
        });
        c.also = [&](glp_tree * t, int) { cur = t; };
        m.set_time_limit(std::chrono::duration<double>(60));
        m.solve();
        std::println(stderr, "  {}: {} value={} {} nodes={} calls={} cuts offered={} max level={}", mode == 0 ? "below the root, 1 per call" : mode == 1 ? "root only" : "everywhere", st(m), m.get_solution_value(), std::fabs(m.get_solution_value() - base) < 1e-6 ? "SAME" : "DIFFERENT", m.closed, calls, cuts, maxlevel);
    }
}
int main(int argc, char ** argv) {
    const std::string w = argv[1];
    if(w == "stop") stop_resume(std::atoi(argv[2]), static_cast<unsigned>(std::atoi(argv[3])), std::atoi(argv[4]));
    else if(w == "over") overshoot(std::atoi(argv[2]), std::atof(argv[3]));
    else if(w == "micro") micro();
    else if(w == "after") aftermath(argv[2]);
    else if(w == "route") route_cols(argv[2]);
    else if(w == "deep") deep(std::atoi(argv[2]), static_cast<unsigned>(std::atoi(argv[3])));
    return 0;
}
