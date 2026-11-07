// Remaining checks: T5b extreme scales with and without the absolute drop; T8b an emptying cut deep in the tree;
// C10 dense but not parallel to the objective; binary parallel columns, are they merged at all?
#include "verify_cplex_common.hpp"
using namespace cp;
using std::chrono::duration;
static cf::setup<M> mkB(double limit, recipe R) {
    cf::setup<M> B;
    B.attach = [R](M & m, separator sep) { vattach(m, std::move(sep), R); };
    B.one_thread = [](M & m) { m.set_thread_limit(1); };
    B.limit = limit;
    return B;
}
int main(int, char ** argv) {
    { M m; print_version(m); }
    const std::string what = argv[1] ? argv[1] : "";
    if(what == "scale") {
        for(const bool noabs : {false, true}) {
            recipe R; if(noabs) { R.satisfied_tol = 0.0; R.inf_tol = 1.1e-6; }
            const auto B = mkB(10, R);
            for(const double S : {1e-30, 1e-20, 1e-15, 1e-12, 1e-10, 1e12, 1e15, 1e18, 1e20, 1e25}) for(const unsigned seed : {1u, 5u}) {
                cf::compare(B, std::format("T5b DFJ x {:g} atsp22s{} abs_tol={}", S, seed, noabs ? "0, inf_tol=1.1e-6" : "1e-9"), [=](M & m) {
                    auto a = build_atsp(m, 22, false, seed);
                    return separator([=](const std::vector<double> & x, const add_fn & add) {
                        for(auto c : a.separate(x)) { for(auto & w : c.val) w *= S; c.rhs *= S; add(c); }
                    });
                });
                std::println(stderr, "    [hook] {}", vkeep().back()->line());
            }
        }
    }
    if(what == "t8b") {
        long ok = 0, bad = 0, nop = 0, base_inf = 0;
        for(unsigned seed = 1; seed <= 200; ++seed) for(const int after : {3, 10, 50}) {
            const std::size_t C = 24;
            M m; m.set_thread_limit(1);
            std::mt19937 rng(seed);
            std::uniform_int_distribution<int> A(1, 9);
            auto x = m.add_binary_variables(C);
            const auto ids = std::views::iota(std::size_t{0}, C);
            std::vector<double> a(C), r(C), c(C);
            double t = 0; for(auto & v : a) { v = 2 * A(rng); t += v; }
            for(auto & v : c) v = A(rng);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * x(j); }));
            m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * x(j); }) == std::floor(t / 4) * 2 + 1);
            double s = 0; for(auto & v : r) { v = A(rng); s += v; }
            m.add_constraint(xsum(ids, [&](std::size_t j) { return r[j] * x(j); }) <= std::floor(s / 2));
            scope_settings(m); m.set_time_limit(duration<double>(5));
            std::vector<int> col; for(std::size_t j = 0; j < C; ++j) col.push_back(m.native_id(x(j)));
            auto n = std::make_shared<std::atomic<int>>(0);
            auto fired = std::make_shared<std::atomic<bool>>(false);
            vattach(m, [=](const std::vector<double> &, const add_fn & add) {
                if(n->fetch_add(1) < after) return;
                fired->store(true);
                raw_cut k{{}, {}, 'L', -1}; for(int j : col) { k.ind.push_back(j); k.val.push_back(1.0); } add(k);
            });
            const double tt = timed_solve(m); (void)tt;
            const bool inf = is_a<status::infeasible>(m.get_status());
            if(!fired->load()) { ++nop; base_inf += inf; continue; }
            if(inf) ++ok; else { ++bad; std::println(stderr, "T8b seed={} after={}: {}", seed, after, summary(m, tt)); }
        }
        std::println(stderr, "T8b emptying cut after 3/10/50 calls, integer-infeasible model, no presolve: infeasible={} other={} cut_never_reached={} (of those infeasible {})", ok, bad, nop, base_inf);
    }
    if(what == "c4merge") {
        // Is the conformance C4 model (binary a + b, degree rows, MTZ) merged? And binary duplicates in a pure knapsack, with cuts on them?
        for(const int reform : {3, 2, 0}) {
            const int n = 22; const auto N = static_cast<std::size_t>(n);
            M m; m.set_thread_limit(1); vparam(m, CPXPARAM_Preprocessing_Reformulations, reform); vparam(m, CPX_PARAM_SCRIND, 1); vparam(m, CPX_PARAM_MIPDISPLAY, 1);
            std::mt19937 rng(1); std::uniform_real_distribution<double> P(0, 100);
            std::vector<std::vector<double>> d(N, std::vector<double>(N)); for(auto & r : d) for(auto & v : r) v = std::round(P(rng));
            auto a = m.add_binary_variables(N * N); auto b = m.add_binary_variables(N * N);
            auto u = m.add_variables(N, {.obj_coef = 0, .lower_bound = 0, .upper_bound = static_cast<double>(n - 1)});
            const auto nodes = std::views::iota(std::size_t{0}, N); const auto id = [N](std::size_t i, std::size_t j) { return i * N + j; };
            m.set_minimization();
            m.set_objective(xsum(nodes, [&](std::size_t i) { return xsum(nodes, [&, i](std::size_t j) { return d[i][j] * (a(id(i, j)) + b(id(i, j))); }); }));
            for(std::size_t i : nodes) {
                m.set_variable_upper_bound(a(id(i, i)), 0); m.set_variable_upper_bound(b(id(i, i)), 0);
                m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return a(id(i, j)) + b(id(i, j)); }) == 1);
                m.add_constraint(xsum(nodes, [&, i](std::size_t j) { return a(id(j, i)) + b(id(j, i)); }) == 1);
            }
            for(std::size_t i = 1; i < N; ++i) for(std::size_t j = 1; j < N; ++j) if(i != j) m.add_constraint(u(i) - u(j) + static_cast<double>(n - 1) * (a(id(i, j)) + b(id(i, j))) <= static_cast<double>(n - 2));
            std::println("#### C4 model reform={}", reform); std::fflush(stdout);
            m.set_time_limit(duration<double>(2)); timed_solve(m); std::fflush(stdout);
        }
        for(const unsigned seed : {1u, 2u, 3u, 4u, 5u, 6u}) for(const int reform : {3, 2}) {
            M m; m.set_thread_limit(1); vparam(m, CPXPARAM_Preprocessing_Reformulations, reform);
            const std::size_t C = 40, R = 6;
            std::mt19937 rng(seed); std::uniform_int_distribution<int> A(1, 60), P(5, 80);
            std::vector<std::vector<double>> a(R, std::vector<double>(C)); for(auto & r : a) for(auto & v : r) v = A(rng);
            std::vector<double> c(C), b(R); for(auto & v : c) v = P(rng);
            for(std::size_t i = 0; i < R; ++i) { double t = 0; for(double v : a[i]) t += v; b[i] = std::floor(t * 0.8); }
            const auto ids = std::views::iota(std::size_t{0}, C);
            auto x1 = m.add_binary_variables(C); auto x2 = m.add_binary_variables(C);
            m.set_maximization();
            m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * (x1(j) + x2(j)); }));
            for(std::size_t i = 0; i < R; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * (x1(j) + x2(j)); }) <= b[i]);
            std::vector<int> c1, c2; for(std::size_t j = 0; j < C; ++j) { c1.push_back(m.native_id(x1(j))); c2.push_back(m.native_id(x2(j))); }
            m.set_time_limit(duration<double>(10));
            auto gen = std::make_shared<std::mt19937>(seed * 31u);
            auto & h = vattach(m, [=](const std::vector<double> & p, const add_fn & add) {
                const int ds[] = {2, 3, 4, 5, 7, 11, 13, 17, 19, 23};
                for(int attempt = 0, added = 0; attempt < 200 && added < 3; ++attempt) {
                    const std::size_t i = (*gen)() % a.size();
                    const double d = ds[(*gen)() % std::size(ds)];
                    raw_cut cut{{}, {}, 'L', std::floor(b[i] / d)};
                    for(std::size_t j = 0; j < c1.size(); ++j) { const double f = std::floor(a[i][j] / d); if(f == 0) continue; cut.ind.push_back(c1[j]); cut.val.push_back(f); cut.ind.push_back(c2[j]); cut.val.push_back(f); }
                    if(cut.violation(p) > 1e-4) { add(cut); ++added; }
                }
            });
            const double t = timed_solve(m);
            std::println(stderr, "binary duplicates knapsack seed={} Reform={}: {} | {}", seed, reform, summary(m, t), h.line());
        }
    }
    if(what == "bin1121") {
        // Any cut on a merged column, however harmless: does a binary duplicate answer 1121 as well? Raw call, nothing dropped.
        for(const int kind : {0, 1, 2}) for(const int reform : {3, 2}) {
            M m; m.set_thread_limit(1); vparam(m, CPXPARAM_Preprocessing_Reformulations, reform);
            const std::size_t C = 40, R = 6;
            std::mt19937 rng(1); std::uniform_int_distribution<int> A(1, 60), P(5, 80);
            std::vector<std::vector<double>> a(R, std::vector<double>(C)); for(auto & r : a) for(auto & v : r) v = A(rng);
            std::vector<double> c(C), b(R); for(auto & v : c) v = P(rng);
            for(std::size_t i = 0; i < R; ++i) { double t = 0; for(double v : a[i]) t += v; b[i] = std::floor(t * (kind == 0 ? 0.8 : 3.0)); }
            const auto ids = std::views::iota(std::size_t{0}, C);
            std::vector<int> c1, c2;
            auto build = [&](auto x1, auto x2) {
                m.set_maximization();
                m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * (x1(j) + x2(j)); }));
                for(std::size_t i = 0; i < R; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * (x1(j) + x2(j)); }) <= b[i]);
                for(std::size_t j = 0; j < C; ++j) { c1.push_back(m.native_id(x1(j))); c2.push_back(m.native_id(x2(j))); }
            };
            if(kind == 0) { auto x1 = m.add_binary_variables(C); auto x2 = m.add_binary_variables(C); build(x1, x2); }
            else if(kind == 1) { auto x1 = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); auto x2 = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); build(x1, x2); }
            else { auto x1 = m.add_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); auto x2 = m.add_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); auto y = m.add_binary_variables(1); (void)y; build(x1, x2); }
            m.set_time_limit(duration<double>(5));
            recipe Rr; Rr.drop_satisfied = false; Rr.fold = false;
            auto & h = vattach(m, [=](const std::vector<double> &, const add_fn & add) {
                static thread_local int n = 0;
                if(++n > 3) return;
                add(raw_cut{{c1[0], c2[0]}, {1.0, 1.0}, 'L', 20.0});
            }, Rr);
            const double t = timed_solve(m);
            std::println(stderr, "bin1121 {} duplicates Reform={}: {} | {}", kind == 0 ? "binary" : kind == 1 ? "integer" : "continuous", reform, summary(m, t), h.line());
        }
    }
    if(what == "empty") {
        // What sanitize's "skip what is empty" protects from: the raw native call with no terms, or only zero coefficients, at every call.
        for(const int kind : {0, 1, 2}) {
            M m; m.set_thread_limit(1); auto a = build_atsp(m, 26, false, 5); m.set_time_limit(duration<double>(4));
            recipe R; R.fold = false; R.drop_satisfied = false;
            const int c0 = a.col[0][1];
            auto & h = vattach(m, [=](const std::vector<double> &, const add_fn & add) {
                if(kind == 0) add(raw_cut{{}, {}, 'L', 1.0});
                if(kind == 1) add(raw_cut{{c0}, {0.0}, 'L', 1.0});
                if(kind == 2) add(raw_cut{{c0, c0}, {1.0, -1.0}, 'L', 1.0});
            }, R);
            const double t = timed_solve(m);
            std::println(stderr, "EMPTY kind={}: {} | {}", kind == 0 ? "no terms" : kind == 1 ? "one zero coefficient" : "x - x (last wins: -x <= 1)", summary(m, t), h.line());
        }
    }
    if(what == "spin4") {
        // the root spin at more than one thread, and the sub-threshold one
        for(const int th : {2, 4, 16}) for(const int par : {1, -1}) for(const int kind : {0, 1}) {
            M m; set_threads(m, th); vparam(m, CPXPARAM_Parallel, par); auto a = build_atsp(m, 26, false, 5); m.set_time_limit(duration<double>(4));
            recipe R; R.drop_satisfied = kind == 1;  // kind 0: satisfied cut, no drop; kind 1: full recipe, fresh cut violated by 5e-7
            auto & h = vattach(m, [=](const std::vector<double> & x, const add_fn & add) {
                if(kind == 0) add(raw_cut{{a.col[0][1]}, {1.0}, 'L', 1.0});
                else { auto cs = a.separate(x, 1); if(cs.empty()) return; raw_cut c = cs.front(); const double rhs = c.activity(x) + 5e-7; if(rhs > 1) return; c.rhs = rhs; add(c); }
            }, R);
            const double t = timed_solve(m);
            std::println(stderr, "SPIN4 threads={} parallel={} {}: {} | {}", th, par, kind ? "recipe, fresh cut violated by 5e-7" : "no drop, satisfied cut", summary(m, t), h.line());
        }
    }
    if(what == "dense") {
        for(const auto & [iname, rows, cols, seed] : std::initializer_list<std::tuple<const char *, int, int, unsigned>>{{"mkp30x100s7", 30, 100, 7}, {"mkp5x100s3", 5, 100, 3}, {"mkp10x200s3", 10, 200, 3}, {"mkp5x60s3", 5, 60, 3}}) for(const double mu : {0.0, 1.0, 3.0, 10.0, 100.0}) for(const auto & [mn, mp, add] : std::initializer_list<std::tuple<const char *, int, bool>>{{"FILTER", CPX_USECUT_FILTER, true}, {"PURGE", CPX_USECUT_PURGE, true}}) {
            M m; m.set_thread_limit(1); build_mkp(m, rows, cols, seed); m.set_time_limit(duration<double>(0.5));
            std::vector<double> obj(m.num_variables());
            { auto [e0, l0] = m.native_model(); m.native_api()._check(e0, m.native_api().getobj(e0, l0, obj.data(), 0, static_cast<int>(m.num_variables()) - 1)); }
            auto cosv = std::make_shared<double>(0.0);
            all_families_off(m); vparam(m, CPX_PARAM_MIRCUTS, 2); vparam(m, CPX_PARAM_EACHCUTLIM, 0);
            auto [env, lp] = m.native_model();
            const int n = static_cast<int>(m.num_variables()); const int nr = static_cast<int>(m.num_constraints());
            std::vector<std::vector<double>> a(static_cast<std::size_t>(nr), std::vector<double>(static_cast<std::size_t>(n), 0.0)); std::vector<double> b(static_cast<std::size_t>(nr));
            std::vector<int> ind(static_cast<std::size_t>(n)); std::vector<double> val(static_cast<std::size_t>(n)); int nz, beg, surplus;
            for(int i = 0; i < nr; ++i) {
                m.native_api()._check(env, m.native_api().getrows(env, lp, &nz, &beg, ind.data(), val.data(), n, &surplus, i, i));
                m.native_api()._check(env, m.native_api().getrhs(env, lp, &b[static_cast<std::size_t>(i)], i, i));
                for(int k = 0; k < nz; ++k) a[static_cast<std::size_t>(i)][static_cast<std::size_t>(ind[static_cast<std::size_t>(k)])] = val[static_cast<std::size_t>(k)];
            }
            auto cut = std::make_shared<std::optional<raw_cut>>();
            auto later = std::make_shared<std::array<long, 2>>(std::array<long, 2>{0, 0});
            auto vat = std::make_shared<double>(0.0);
            auto add_flag = add;
            recipe R; R.purgeable = mp; R.drop_satisfied = false; R.fold = false;
            vattach(m, [=](const std::vector<double> & x, const add_fn & addf) {
                if(*cut) { (*later)[0]++; (*later)[1] += (*cut)->violation(x) > 1e-6; return; }
                // (c + mu a_i) x <= floor(c x) + mu b_i: objective floor plus mu times a tight row. Valid, integer, as violated as
                // the floor itself, and less and less parallel to the objective as mu grows.
                std::size_t ti = a.size();
                for(std::size_t i = 0; i < a.size(); ++i) { double act = 0; for(std::size_t j = 0; j < a[i].size(); ++j) act += a[i][j] * x[j]; if(act >= b[i] - 1e-6) { ti = i; break; } }
                if(ti == a.size()) return;
                double act = 0; for(std::size_t j = 0; j < x.size(); ++j) act += obj[j] * x[j];
                const double fl = std::floor(act + 1e-9);
                if(act - fl < 1e-3) return;
                raw_cut best{{}, {}, 'L', fl + mu * b[ti]};
                double dot = 0, n1 = 0, n2 = 0;
                for(std::size_t j = 0; j < x.size(); ++j) { const double w = obj[j] + mu * a[ti][j]; if(w != 0) { best.ind.push_back(static_cast<int>(j)); best.val.push_back(w); } dot += w * obj[j]; n1 += w * w; n2 += obj[j] * obj[j]; }
                const double bestv = act - fl;
                *cosv = dot / std::sqrt(n1 * n2);
                if(best.ind.empty()) return;
                *cut = best; *vat = bestv;
                if(add_flag) addf(best);
            }, R);
            const double t = timed_solve(m);
            std::println(stderr, "C10 rotated floor {} mu={:g} cos(cut,obj)={:.3f} {}: nodes={} | nz={} viol_at_add={:.3f} later viol/checks {}/{} numcuts_user={}", iname, mu, *cosv, mn, m.get_node_count(), *cut ? (*cut)->ind.size() : 0, *vat, (*later)[1], (*later)[0], vnumcuts(m)); (void)t;
        }
    }
    if(what == "binmerge") {
        // Binary parallel columns against integer and continuous ones: the presolve log says whether they merge.
        for(const int kind : {0, 1, 2}) for(const int reform : {3, 2, 1, 0}) {
            M m; m.set_thread_limit(1); vparam(m, CPXPARAM_Preprocessing_Reformulations, reform); vparam(m, CPX_PARAM_SCRIND, 1); vparam(m, CPX_PARAM_MIPDISPLAY, 1);
            const std::size_t C = 25, R = 6;
            std::mt19937 rng(1);
            std::uniform_int_distribution<int> A(1, 60), P(5, 80);
            std::vector<std::vector<double>> a(R, std::vector<double>(C)); for(auto & r : a) for(auto & v : r) v = A(rng);
            std::vector<double> c(C), b(R); for(auto & v : c) v = P(rng);
            for(std::size_t i = 0; i < R; ++i) { double t = 0; for(double v : a[i]) t += v; b[i] = std::floor(t * 0.4); }
            const auto ids = std::views::iota(std::size_t{0}, C);
            auto build = [&](auto x1, auto x2) {
                m.set_maximization();
                m.set_objective(xsum(ids, [&](std::size_t j) { return c[j] * (x1(j) + x2(j)); }));
                for(std::size_t i = 0; i < R; ++i) m.add_constraint(xsum(ids, [&, i](std::size_t j) { return a[i][j] * (x1(j) + x2(j)); }) <= b[i]);
            };
            std::println("#### kind={} reform={}", kind == 0 ? "binary" : kind == 1 ? "integer" : "continuous", reform); std::fflush(stdout);
            if(kind == 0) { auto x1 = m.add_binary_variables(C); auto x2 = m.add_binary_variables(C); build(x1, x2); }
            else if(kind == 1) { auto x1 = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); auto x2 = m.add_integer_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); build(x1, x2); }
            else { auto x1 = m.add_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); auto x2 = m.add_variables(C, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 7}); auto y = m.add_binary_variables(1); (void)y; build(x1, x2); m.add_constraint(1.0 * y(std::size_t{0}) <= 1); }
            m.set_time_limit(duration<double>(1)); timed_solve(m); std::fflush(stdout);
        }
    }
}
