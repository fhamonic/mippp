// Q10: which duplicate entry wins? Order and sign, read back from the next LP. Upper bound of x is 5.
#include "probe_cplex_common.cpp"
struct D { const c::cplex_api * api; const syms * s; int cols, x; std::vector<double> vals; int st = 0, e = 12345; double before = NAN, after = NAN; };
static int dhook(ctxptr ctx, cpxlong id, void * usr) {
    auto & d = *static_cast<D *>(usr);
    if(id != c::CPX_CALLBACKCONTEXT_RELAXATION) return 0;
    std::vector<double> x(static_cast<std::size_t>(d.cols));
    d.s->point(ctx, x.data(), 0, d.cols - 1, nullptr);
    if(d.st == 0) {
        d.before = x[static_cast<std::size_t>(d.x)]; d.st = 1;
        std::vector<int> ind(d.vals.size(), d.x);
        const double rhs = 1; const char sense = 'L'; const int beg = 0, pg = USECUT_FORCE, lc = 0;
        d.e = d.api->callbackaddusercuts(ctx, 1, static_cast<int>(ind.size()), &rhs, &sense, &beg, ind.data(), d.vals.data(), &pg, &lc);
    } else if(d.st == 1) { d.after = x[static_cast<std::size_t>(d.x)]; d.st = 2; }
    return 0;
}
int main() {
    std::println(stderr, "library {}", M{}.native_api().library_path().string());
    for(const std::vector<double> vals : std::vector<std::vector<double>>{{0.8, 0.2}, {0.2, 0.8}, {0.5, 0.5}, {-0.5, 1.0}, {1.0, -0.5}, {0.25, 0.25, 0.5}, {0.5, 0.25, 0.25}}) {
        M m; scope_settings(m); param(m, P_MIRCUTS, 1); param(m, P_EACHCUTLIM, 0);
        const std::size_t Y = 18;
        std::mt19937 rng(9); std::uniform_int_distribution<int> A(5, 60), P(5, 80);
        auto y = m.add_binary_variables(Y);
        auto x = m.add_variables(1, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 5});
        std::vector<double> a(Y), p(Y); for(std::size_t j = 0; j < Y; ++j) { a[j] = A(rng); p[j] = P(rng); }
        const auto ids = std::views::iota(std::size_t{0}, Y);
        m.set_maximization();
        m.set_objective(xsum(ids, [&](std::size_t j) { return p[j] * y(j); }) + 1.0 * x(std::size_t{0}));
        m.add_constraint(xsum(ids, [&](std::size_t j) { return a[j] * y(j); }) <= 203.5);
        m.set_time_limit(std::chrono::duration<double>(5));
        D d{&m.native_api(), &syms::get(m), static_cast<int>(m.num_variables()), m.native_id(x(std::size_t{0})), vals};
        auto [env, lp] = m.native_model();
        m.native_api()._check(env, m.native_api().callbacksetfunc(env, lp, c::CPX_CALLBACKCONTEXT_RELAXATION, dhook, &d));
        cp::timed_solve(m);
        std::string v; for(double w : vals) v += std::format(" {}", w);
        double sum = 0; for(double w : vals) sum += w;
        std::println(stderr, "== coefficients{} on one column, rhs 1: add={} x before={} next call={} (if summed x<={}, if last wins {})", v, d.e, d.before, d.after, std::min(5.0, 1.0 / sum), vals.back() > 0 ? std::min(5.0, 1.0 / vals.back()) : 5.0);
    }
}
