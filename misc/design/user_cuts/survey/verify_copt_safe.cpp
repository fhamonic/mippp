// D: try to break "any valid cut is safe under the recipe at default settings". Recipe = cf::sanitize + add.
// usage: verify_copt_safe <case> [seed] [L|G]   One process per case. Prints OK or WRONG against the same model with no hook.
#include "verify_copt_common.hpp"
using namespace std::chrono;
namespace {
struct ext {
    std::vector<int> xs, xb; std::vector<double> xu;       // x_s + xb == u, xb continuous
    std::vector<int> ys, yb;                               // y_s + yb == 1, yb binary
    std::vector<std::vector<int>> slack_row; std::vector<double> slack_rhs;   // Sum_k x^k_e + s_e == cap_e
    std::vector<int> f, q, d;
};
struct inst { mcnd g; ext e; };
template <class Mdl>
inst build(Mdl & m, unsigned seed, std::size_t N = 14, std::size_t EX = 30, std::size_t K = 16) {
    setenv("CAPF", "0.2", 1);
    auto v = build_mcnd(m, N, EX, K, seed);
    inst I{v.g, {}};
    auto & E = I.e;
    const std::size_t A = v.g.A;
    const std::size_t nx = 30;
    auto xb = m.add_variables(nx);
    auto yb = m.add_binary_variables(nx);
    auto sl = m.add_variables(nx);
    auto f = m.add_binary_variables(6);
    auto q = m.add_variables(6, {.obj_coef = 7, .lower_bound = 0, .upper_bound = 1});
    auto d = m.add_variables(6, {.obj_coef = 0, .lower_bound = 0, .upper_bound = 1});
    for(std::size_t t = 0; t < nx; ++t) {
        const std::size_t k = t % K, a = (t * 3) % A;
        E.xs.push_back(v.g.x[k][a]); E.xb.push_back(m.native_id(xb(t))); E.xu.push_back(v.g.d[k]);
        m.add_constraint(1.0 * v.xv(v.idx(k, a)) + 1.0 * xb(t) == v.g.d[k]);
        const std::size_t a2 = (t * 5 + 1) % A;
        E.ys.push_back(v.g.y[a2]); E.yb.push_back(m.native_id(yb(t)));
        m.add_constraint(1.0 * v.yv(a2) + 1.0 * yb(t) == 1.0);
        const std::size_t a3 = (t * 7 + 2) % A;
        std::vector<int> row;
        for(std::size_t kk = 0; kk < K; ++kk) row.push_back(v.g.x[kk][a3]);
        row.push_back(m.native_id(sl(t)));
        E.slack_row.push_back(row); E.slack_rhs.push_back(v.g.cap[a3]);
        m.add_constraint(xsum(std::views::iota(std::size_t{0}, K), [&, a3](std::size_t kk) { return 1.0 * v.xv(v.idx(kk, a3)); }) + 1.0 * sl(t) == v.g.cap[a3]);
    }
    for(std::size_t t = 0; t < 6; ++t) {
        E.f.push_back(m.native_id(f(t))); m.add_constraint(1.0 * f(t) == 1.0);
        E.q.push_back(m.native_id(q(t))); E.d.push_back(m.native_id(d(t)));
    }
    return I;
}
void push(cp::raw_cut & c, int j, double v) { c.ind.push_back(j); c.val.push_back(v); }
void attach_recipe(V & m, cp::separator sep) {
    recipe_opts o; o.fold = o.strip_zero = o.skip_empty = o.drop_satisfied = false;
    m.attach([sep = std::move(sep)](const std::vector<double> & x, const cp::add_fn & add) {
        sep(x, [&](const cp::raw_cut & raw) -> int {
            auto r = cf::sanitize(raw, x);
            if(!r) return 0;
            if(std::getenv("NORMALIZE")) { double big = 0; for(double w : r->val) big = std::max(big, std::fabs(w)); if(big > 0) { for(auto & w : r->val) w /= big; r->rhs /= big; } }
            return add(*r); });
    }, o);
}
}
int main(int argc, char ** argv) {
    const std::string kase = argv[1];
    const unsigned seed = argc > 2 ? std::atoi(argv[2]) : 1;
    const char form = argc > 3 ? argv[3][0] : 'L';
    double base = NAN; std::string base_s;
    { V b; b.iparam("Threads", 1); build(b, seed); b.set_time_limit(duration<double>(40.0)); const double t = cp::timed_solve(b); base = b.get_solution_value(); base_s = cp::summary(b, t); }
    V m; m.iparam("Threads", 1); auto I = build(m, seed); m.set_time_limit(duration<double>(40.0));
    std::atomic<long> offered{0}, calls{0}, nidx{0};
    const auto & E = I.e; const auto & g = I.g;
    const std::size_t nE = E.xs.size();
    // tiny sweep: tiny<scale>_<coef>
    double tiny_scale = 0, tiny_coef = 0, scale = 1;
    if(kase.starts_with("tiny:")) { std::sscanf(kase.c_str(), "tiny:%lf:%lf", &tiny_scale, &tiny_coef); }
    if(kase.starts_with("scale:")) { std::sscanf(kase.c_str(), "scale:%lf", &scale); }
    double dup_eps = 0;
    const auto finish_form = [&](cp::raw_cut c) {   // written in the asked sense
        if(form == 'G') { for(auto & w : c.val) w = -w; c.sense = c.sense == 'L' ? 'G' : c.sense == 'G' ? 'L' : 'E'; c.rhs = -c.rhs; }
        return c;
    };
    const double opt = base;
    cp::raw_cut objc; objc.sense = 'G'; objc.ind = g.obj_ind; objc.val = g.obj_val; objc.rhs = opt;
    attach_recipe(m, [&](const std::vector<double> & p, const cp::add_fn & add) {
        ++calls;
        auto cuts = g.links(p, 10);
        for(auto c : cuts) {
            const std::size_t t = static_cast<std::size_t>(nidx++ % (long)nE);
            if(kase == "plain") {}
            else if(kase == "zslack") { for(int j : E.slack_row[t]) push(c, j, 1.0); c.rhs += E.slack_rhs[t]; }
            else if(kase == "zcont") { push(c, E.xs[t], 1.0); push(c, E.xb[t], 1.0); c.rhs += E.xu[t]; }
            else if(kase == "zbin") { push(c, E.ys[t], 1.0); push(c, E.yb[t], 1.0); c.rhs += 1.0; }
            else if(kase == "zboth") { push(c, E.ys[t], 1.0); push(c, E.yb[t], 1.0); push(c, E.xs[t], 1.0); push(c, E.xb[t], 1.0); c.rhs += 1.0 + E.xu[t]; }
            else if(kase == "fixed1") { push(c, E.f[t % 6], 1.0); c.rhs += 1.0; }
            else if(kase == "fixedneg") { push(c, E.f[t % 6], -1.0); c.rhs -= 1.0; }
            else if(kase == "fixedall") { for(int j : E.f) push(c, j, 1.0); c.rhs += 6.0; }
            else if(kase == "dualfix") { push(c, E.q[t % 6], 1.0); c.rhs += 1.0; }
            else if(kase == "dualfix_tight") { push(c, E.q[t % 6], -1.0); }
            else if(kase == "rowless") { push(c, E.d[t % 6], 1.0); c.rhs += 1.0; }
            else if(kase == "rowless_tight") { push(c, E.d[t % 6], -1.0); }
            else if(kase == "everything") { for(int j : E.slack_row[t]) push(c, j, 1.0); c.rhs += E.slack_rhs[t]; push(c, E.f[t % 6], 1.0); c.rhs += 1.0; push(c, E.q[t % 6], 1.0); c.rhs += 1.0; push(c, E.d[t % 6], 1.0); c.rhs += 1.0; push(c, E.ys[t], 1.0); push(c, E.yb[t], 1.0); c.rhs += 1.0; }
            else if(kase.starts_with("tiny:")) {
                for(auto & w : c.val) w *= tiny_scale;
                c.rhs *= tiny_scale;
                const int s = E.xs[t];
                const double u = E.xu[t];
                push(c, s, tiny_coef);
                if(tiny_coef > 0) c.rhs += tiny_coef * u;
            }
            else if(kase.starts_with("scale:")) { for(auto & w : c.val) w *= scale; c.rhs *= scale; }
            else if(kase == "fold_residue") {   // 0.1 + 0.2 - 0.3 on a real column: 5.55e-17 survives the zero strip
                push(c, E.xs[t], 0.1); push(c, E.xs[t], 0.2); push(c, E.xs[t], -0.3);
                c.rhs += 6e-17 * E.xu[t];
            }
            else if(kase == "fold_residue_neg") { push(c, E.xs[t], 0.3); push(c, E.xs[t], -0.1); push(c, E.xs[t], -0.2); }   // -5.55e-17 or 0
            else if(kase == "neardup") {
                add(finish_form(c)); ++offered;
                { auto n = c; n.rhs += 1e-12; add(finish_form(n)); ++offered; }
                { auto n = c; n.rhs += 1e-9; add(finish_form(n)); ++offered; }
                { auto n = c; n.rhs += 1e-7; add(finish_form(n)); ++offered; }
                { auto n = c; for(auto & w : n.val) w *= 2; n.rhs *= 2; add(finish_form(n)); ++offered; }
                { auto n = c; n.val[1] *= 1 + 1e-10; add(finish_form(n)); ++offered; }
                { auto n = c; std::swap(n.ind[0], n.ind[1]); std::swap(n.val[0], n.val[1]); add(finish_form(n)); ++offered; }
                continue;
            }
            else if(kase == "objfloor" || kase == "objfloor_eps") {}
            else { std::println(stderr, "unknown case {}", kase); std::exit(9); }
            add(finish_form(c)); ++offered;
        }
        if(kase == "objfloor") { add(objc); ++offered; }
        if(kase == "objfloor_eps") { auto o = objc; o.rhs = opt - 1e-7; add(o); ++offered; }
        if(kase == "objfloor_only") {}
    });
    const double t = cp::timed_solve(m);
    const double v = cf::value_of(m);
    const bool ok = cf::same(v, base) && is_a<status::optimal>(m.get_status());
    std::println(stderr, "{:24} seed={} form={} {} {} | base {} | calls={} offered={} {}", kase, seed, form, ok ? "OK   " : "WRONG", cp::summary(m, t), base_s, calls.load(), offered.load(), m.last ? m.last->line() : "");
}
