// Free simplex vs dual simplex for the solve after set_basis() on mosek_lp:
// (1) optimal basis transferred to a fresh identical dense model;
// (2) column generation: solve, get_basis, add attractive column(s),
//     set_basis, solve. Iteration counters after every solve.
#include <cmath>
#include <cstdio>
#include <random>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/solvers/mosek/all.hpp"

using namespace mippp;
using namespace mippp::mosek::impl::v1;

using M = mosek_lp;
using var = model_variable_t<M>;
using con = model_constraint_t<M>;

const mosek_api * API = nullptr;

std::vector<var> build_dense(M & model) {
    using namespace operators;
    constexpr std::size_t size = 50;
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> coef(1, 1000);
    auto columns = std::views::iota(std::size_t{0}, size);
    std::vector<var> x;
    for(std::size_t j = 0; j < size; ++j) x.push_back(model.add_variable());
    std::vector<int> row(size);
    for(int & a : row) a = coef(rng);
    model.set_maximization();
    model.set_objective(xsum(columns, [&](auto j) { return row[j] * x[j]; }));
    for(std::size_t i = 0; i < size; ++i) {
        for(int & a : row) a = coef(rng);
        model.add_constraint(xsum(columns, [&](auto j) {
                                 return row[j] * x[j];
                             }) <= 1000.0 * static_cast<double>(size));
    }
    return x;
}

MSKtask_t T(M & m) { return m.native_model().second; }

struct counts {
    int prim, dual, ipm;
};
counts get_counts(M & m) {
    counts c{-1, -1, -1};
    API->_check(API->getintinf(T(m), MSK_IINF_SIM_PRIMAL_ITER, &c.prim));
    API->_check(API->getintinf(T(m), MSK_IINF_SIM_DUAL_ITER, &c.dual));
    API->_check(API->getintinf(T(m), MSK_IINF_INTPNT_ITER, &c.ipm));
    return c;
}
int get_optimizer(M & m) {
    MSKint32t v;
    API->_check(API->getintparam(T(m), MSK_IPAR_OPTIMIZER, &v));
    return v;
}
void set_optimizer(M & m, int v) {
    API->_check(API->putintparam(T(m), MSK_IPAR_OPTIMIZER,
                                 static_cast<MSKoptimizertypee>(v)));
}
const char * optname(int o) {
    switch(o) {
        case MSK_OPTIMIZER_FREE: return "FREE(ipm)";
        case MSK_OPTIMIZER_FREE_SIMPLEX: return "FREE_SIMPLEX";
        case MSK_OPTIMIZER_DUAL_SIMPLEX: return "DUAL_SIMPLEX";
        case MSK_OPTIMIZER_PRIMAL_SIMPLEX: return "PRIMAL_SIMPLEX";
        case MSK_OPTIMIZER_INTPNT: return "INTPNT";
    }
    return "?";
}
std::string status_name(M & m) {
    return std::visit(
        [](auto s) -> std::string {
            using S = decltype(s);
            if constexpr(std::same_as<S, status::optimal>) return "optimal";
            else if constexpr(std::same_as<S, status::iteration_limit>)
                return "iteration_limit";
            else if constexpr(std::same_as<S, status::unknown>)
                return "unknown";
            else return "other";
        },
        m.get_status());
}
void report(M & m, const char * label) {
    auto c = get_counts(m);
    std::printf("  %-48s status=%-9s obj=%.9g | primal=%d dual=%d ipm=%d | "
                "IPAR_OPTIMIZER after=%s\n",
                label, status_name(m).c_str(), m.get_solution_value(), c.prim,
                c.dual, c.ipm, optname(get_optimizer(m)));
}

// adds k attractive columns, each with entries in `rows_per_col` rows,
// objective = factor * (sum a_i y_i) + 1 so that its reduced cost is > 0
void add_attractive(M & m, const std::vector<double> & y, int k,
                    int rows_per_col, double factor, unsigned seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> row(0, 49), coef(100, 1000);
    for(int c = 0; c < k; ++c) {
        std::vector<std::pair<con, double>> entries;
        std::vector<int> used;
        double price = 0;
        while((int)entries.size() < rows_per_col) {
            int i = row(rng);
            if(std::ranges::find(used, i) != used.end()) continue;
            used.push_back(i);
            double a = coef(rng);
            entries.emplace_back(con(i), a);
            price += a * y[i];
        }
        m.add_column(entries, {.obj_coef = factor * price + 1.0});
    }
}

std::vector<double> duals(M & m) {
    auto d = m.get_dual_solution();
    std::vector<double> y(50);
    for(int i = 0; i < 50; ++i) y[i] = d[con(i)];
    return y;
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    API = &mosek_api::load();
    int maj, min, rev;
    API->getversion(&maj, &min, &rev);
    std::printf("MOSEK %d.%d.%d (%s)\n", maj, min, rev,
                API->library_path().c_str());

    std::printf("\n== baselines: cold solves of the dense model ==\n");
    double optimum = 0;
    std::vector<double> y;
    for(int opt : {MSK_OPTIMIZER_FREE, MSK_OPTIMIZER_FREE_SIMPLEX,
                   MSK_OPTIMIZER_DUAL_SIMPLEX, MSK_OPTIMIZER_PRIMAL_SIMPLEX}) {
        M m;
        build_dense(m);
        set_optimizer(m, opt);
        m.solve();
        std::string l = std::string("cold ") + optname(opt);
        report(m, l.c_str());
        if(opt == MSK_OPTIMIZER_FREE) {
            optimum = m.get_solution_value();
            y = duals(m);
        }
    }

    std::printf("\n== (1) basis -> fresh identical model ==\n");
    for(int opt : {(int)MSK_OPTIMIZER_FREE_SIMPLEX, (int)MSK_OPTIMIZER_DUAL_SIMPLEX,
                   -1}) {
        M a;
        build_dense(a);
        a.solve();
        M b;
        build_dense(b);
        b.set_basis(a.get_basis());
        if(opt >= 0) set_optimizer(b, opt);
        b.solve();
        std::string l = opt >= 0 ? std::string("warm native ") + optname(opt)
                                 : std::string("warm wrapper one-shot");
        report(b, l.c_str());
        std::printf("    |obj - optimum| = %.3g\n",
                    std::abs(b.get_solution_value() - optimum));
        if(opt < 0) {
            b.set_variable_upper_bound(var(0), 1.0);
            b.solve();
            report(b, "  next solve after a bound change");
        }
    }

    struct cg_case {
        int k, rows;
        double factor;
        unsigned seed;
    };
    for(cg_case cc : {cg_case{1, 5, 1.5, 1}, cg_case{1, 25, 1.5, 2},
                      cg_case{1, 10, 3.0, 3}, cg_case{5, 10, 1.5, 4},
                      cg_case{20, 10, 1.2, 5}}) {
        std::printf("\n== (2) column generation: %d column(s), %d rows each, "
                    "price factor %.1f, seed %u ==\n",
                    cc.k, cc.rows, cc.factor, cc.seed);
        double ext_opt = 0;
        {
            M cold;
            build_dense(cold);
            cold.solve();
            add_attractive(cold, duals(cold), cc.k, cc.rows, cc.factor,
                           cc.seed);
            M fresh;
            build_dense(fresh);
            add_attractive(fresh, y, cc.k, cc.rows, cc.factor, cc.seed);
            fresh.solve();
            report(fresh, "cold FREE(ipm) of the extended model");
            ext_opt = fresh.get_solution_value();
            for(int opt : {MSK_OPTIMIZER_FREE_SIMPLEX,
                           MSK_OPTIMIZER_DUAL_SIMPLEX,
                           MSK_OPTIMIZER_PRIMAL_SIMPLEX}) {
                M f2;
                build_dense(f2);
                add_attractive(f2, y, cc.k, cc.rows, cc.factor, cc.seed);
                set_optimizer(f2, opt);
                f2.solve();
                std::string l =
                    std::string("cold ") + optname(opt) + " of the extended";
                report(f2, l.c_str());
            }
        }
        for(int opt : {(int)MSK_OPTIMIZER_FREE_SIMPLEX, (int)MSK_OPTIMIZER_DUAL_SIMPLEX,
                       (int)MSK_OPTIMIZER_PRIMAL_SIMPLEX, -1}) {
            M m;
            build_dense(m);
            m.solve();
            const auto basis = m.get_basis();
            add_attractive(m, duals(m), cc.k, cc.rows, cc.factor, cc.seed);
            m.set_basis(basis);
            if(opt >= 0) set_optimizer(m, opt);
            m.solve();
            std::string l = opt >= 0
                                ? std::string("warm native ") + optname(opt)
                                : std::string("warm wrapper one-shot");
            report(m, l.c_str());
            std::printf("    |obj - cold extended| = %.3g (gain %.6g)\n",
                        std::abs(m.get_solution_value() - ext_opt),
                        m.get_solution_value() - optimum);
        }
        // in place without set_basis: MOSEK keeps its own keys
        for(int opt : {MSK_OPTIMIZER_FREE_SIMPLEX, MSK_OPTIMIZER_DUAL_SIMPLEX}) {
            M m;
            build_dense(m);
            m.solve();
            add_attractive(m, duals(m), cc.k, cc.rows, cc.factor, cc.seed);
            set_optimizer(m, opt);
            m.solve();
            std::string l = std::string("in place, no set_basis, ") +
                            optname(opt);
            report(m, l.c_str());
        }
    }

    std::printf("\n== edge: a column, no row ==\n");
    {
        M m;
        auto x = m.add_variable({.obj_coef = 1.0, .upper_bound = 2.0});
        m.set_maximization();
        try {
            m.set_basis(model_basis_t<M>{});
            std::printf("  set_basis on 0 rows: ok\n");
        } catch(const std::exception & e) {
            std::printf("  set_basis on 0 rows threw: %s\n", e.what());
        }
        try {
            m.solve();
            report(m, "solve");
            auto b = m.get_basis();
            std::printf("  get_basis x: index %zu\n", b.get_status(x).index());
        } catch(const std::exception & e) {
            std::printf("  threw: %s\n", e.what());
        }
    }
    std::printf("\n== edge: get_basis before any solve ==\n");
    {
        M m;
        auto x = m.add_variable();
        (void)x;
        try {
            (void)m.get_basis();
            std::printf("  no throw\n");
        } catch(const std::exception & e) {
            std::printf("  threw: %s\n", e.what());
        }
    }
    return 0;
}
