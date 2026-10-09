// Follow-ups 2: when a basis exists before any solve, infeasible LP detected
// by simplex, SetBasis on degenerate shapes, solution reads after SetBasis.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/solvers/copt/all.hpp"

using namespace mippp;
using namespace mippp::copt::impl::v1;
using params_t = model_variable_params_t<copt_lp>;
using variable = model_variable_t<copt_lp>;
using constraint = model_constraint_t<copt_lp>;

constexpr int LOWER = 0, BASIC = 1, UPPER = 2, SUPERBASIC = 3, FIXED = 4;
constexpr int UNTOUCHED = -77;
static const char * code_name(int c) {
    switch(c) {
        case LOWER: return "LOWER";
        case BASIC: return "BASIC";
        case UPPER: return "UPPER";
        case SUPERBASIC: return "SUPERBASIC";
        case FIXED: return "FIXED";
        case UNTOUCHED: return "untouched";
        default: return "?";
    }
}
struct nat {
    const copt_api & api;
    copt_prob * prob;
    explicit nat(copt_lp & m)
        : api(m.native_api()), prob(m.native_model().second) {}
    int iattr(const char * n) {
        int v = -999;
        int r = api.GetIntAttr(prob, n, &v);
        return r ? -1000 - r : v;
    }
    int setint(const char * n, int v) { return api.SetIntParam(prob, n, v); }
    int get_basis(std::vector<int> & c, std::vector<int> & r) {
        c.assign(static_cast<std::size_t>(iattr("Cols")), UNTOUCHED);
        r.assign(static_cast<std::size_t>(iattr("Rows")), UNTOUCHED);
        int dc = UNTOUCHED, dr = UNTOUCHED;
        return api.GetBasis(prob, c.empty() ? &dc : c.data(),
                            r.empty() ? &dr : r.data());
    }
    std::string state() {
        char buf[256];
        std::snprintf(buf, sizeof buf,
                      "LpStatus=%d HasBasis=%d HasLpSol=%d SimplexIter=%d "
                      "BarrierIter=%d",
                      iattr("LpStatus"), iattr("HasBasis"), iattr("HasLpSol"),
                      iattr("SimplexIter"), iattr("BarrierIter"));
        return buf;
    }
};
static std::string codes(const std::vector<int> & v) {
    std::string s;
    for(int c : v) s += std::string(code_name(c)) + " ";
    return s;
}

static void build_dense(copt_lp & m, std::size_t size = 200) {
    using namespace operators;
    m.set_time_limit(std::chrono::seconds(20));
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> coef(1, 1000);
    auto columns = std::views::iota(std::size_t{0}, size);
    auto x = m.add_variables(size);
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    m.set_maximization();
    m.set_objective(xsum(columns, [&](auto j) { return row[j] * x(j); }));
    for(std::size_t i = 0; i < size; ++i) {
        for(int & a : row) a = coef(rng);
        m.add_constraint(xsum(columns, [&](auto j) { return row[j] * x(j); }) <=
                         1000.0 * static_cast<double>(size));
    }
}

int main() {
    using namespace operators;
    std::printf("library: %s\n", copt_api::load().library_path().c_str());
    std::printf("\n=== 1. basis before any solve, by construction order ===\n");
    {
        copt_lp m;
        nat n(m);
        std::vector<int> c, r;
        std::printf("  empty: GetBasis ret=%d HasBasis=%d\n", n.get_basis(c, r), n.iattr("HasBasis"));
        auto x = m.add_variable();
        std::printf("  1 col: GetBasis ret=%d HasBasis=%d\n", n.get_basis(c, r), n.iattr("HasBasis"));
        auto y = m.add_variable(params_t{.obj_coef = 0, .lower_bound = std::nullopt, .upper_bound = 5});
        std::printf("  2 cols (y ub only): GetBasis ret=%d HasBasis=%d\n", n.get_basis(c, r), n.iattr("HasBasis"));
        m.add_constraint(x + y <= 4);
        int g = n.get_basis(c, r);
        std::printf("  2 cols + 1 row: GetBasis ret=%d HasBasis=%d %s| %s\n", g, n.iattr("HasBasis"), codes(c).c_str(), codes(r).c_str());
        auto w = m.add_variable(params_t{.obj_coef = 0, .lower_bound = std::nullopt, .upper_bound = std::nullopt});
        (void)w;
        g = n.get_basis(c, r);
        std::printf("  3 cols + 1 row: GetBasis ret=%d HasBasis=%d %s| %s\n", g, n.iattr("HasBasis"), codes(c).c_str(), codes(r).c_str());
    }
    {
        copt_lp m;
        nat n(m);
        std::vector<int> c, r;
        m.add_constraint(linear_expression_view(std::views::empty<std::pair<variable, double>>, 0.0) <= 4);
        int g = n.get_basis(c, r);
        std::printf("  0 cols + 1 row: GetBasis ret=%d HasBasis=%d %s| %s\n", g, n.iattr("HasBasis"), codes(c).c_str(), codes(r).c_str());
    }
    {
        copt_lp m;
        nat n(m);
        m.add_variable();
        m.add_variable();
        std::vector<int> c = {BASIC, LOWER}, r;
        int dr = 0;
        int s = n.api.SetBasis(n.prob, c.data(), &dr);
        std::printf("  2 cols, 0 rows: SetBasis ret=%d HasBasis=%d\n", s, n.iattr("HasBasis"));
    }

    std::printf("\n=== 2. infeasible dense LP (simplex has to find it) ===\n");
    for(int presolve : {-1, 0}) {
        copt_lp m;
        build_dense(m);
        nat n(m);
        n.setint("Presolve", presolve);
        m.add_constraint(xsum(std::views::iota(0, 200), [](int j) { return variable(j); }) >= 1e6);
        m.solve();
        std::vector<int> c, r;
        int g = n.get_basis(c, r);
        std::printf("  Presolve=%d: %s GetBasis ret=%d\n", presolve, n.state().c_str(), g);
        for(int method : {1}) {
            copt_lp m2;
            build_dense(m2);
            nat n2(m2);
            n2.setint("Presolve", presolve);
            n2.setint("LpMethod", method);
            m2.add_constraint(xsum(std::views::iota(0, 200), [](int j) { return variable(j); }) >= 1e6);
            m2.solve();
            g = n2.get_basis(c, r);
            std::printf("  Presolve=%d LpMethod=%d: %s GetBasis ret=%d\n", presolve, method, n2.state().c_str(), g);
        }
    }

    std::printf("\n=== 3. solution reads after SetBasis on a solved model ===\n");
    {
        copt_lp m;
        build_dense(m);
        nat n(m);
        m.solve();
        std::vector<int> c, r;
        n.get_basis(c, r);
        n.api.SetBasis(n.prob, c.data(), r.data());
        std::vector<double> x(200, NAN), d(200, NAN);
        int g = n.api.GetLpSolution(n.prob, x.data(), nullptr, d.data(), nullptr);
        double obj = NAN;
        int go = n.api.GetDblAttr(n.prob, "LpObjval", &obj);
        std::printf("  after SetBasis: %s GetLpSolution ret=%d x0=%g LpObjval ret=%d value=%g\n",
                    n.state().c_str(), g, x[0], go, obj);
        std::printf("  wrapper status still index %zu (1 = optimal)\n", m.get_status().index());
        try {
            auto sol = m.get_solution();
            std::printf("  wrapper get_solution()[x0] = %g\n", sol[variable(0)]);
        } catch(const std::exception & e) {
            std::printf("  wrapper get_solution() threw: %s\n", e.what());
        }
    }
    std::printf("\nDONE\n");
}
