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
    std::printf("MOSEK %d.%d.%d sweep\n", maj, min, rev);
    std::vector<double> y0;
    {
        M a;
        build_dense(a);
        a.solve();
        y0 = duals(a);
    }
    std::printf("k rows factor seeds | sum free(primal+dual) sum dual(primal+dual) | free wins dual wins ties | cold ipm sum\n");
    for(int k : {1, 2, 5, 10}) {
        for(int rows : {3, 5, 10, 25, 50}) {
            for(double factor : {1.1, 1.5, 3.0}) {
                long sf = 0, sd = 0, sipm = 0;
                int wf = 0, wd = 0, ties = 0, bad = 0;
                const int seeds = 10;
                for(unsigned seed = 1; seed <= seeds; ++seed) {
                    int it[2];
                    double obj[2];
                    int idx = 0;
                    for(int opt : {(int)MSK_OPTIMIZER_FREE_SIMPLEX, (int)MSK_OPTIMIZER_DUAL_SIMPLEX}) {
                        M m;
                        build_dense(m);
                        m.solve();
                        const auto basis = m.get_basis();
                        add_attractive(m, y0, k, rows, factor, 1000 * k + 100 * rows + seed);
                        m.set_basis(basis);
                        set_optimizer(m, opt);
                        m.solve();
                        if(!is_a<status::optimal>(m.get_status())) ++bad;
                        auto c = get_counts(m);
                        it[idx] = c.prim + c.dual;
                        obj[idx] = m.get_solution_value();
                        ++idx;
                    }
                    {
                        M f;
                        build_dense(f);
                        add_attractive(f, y0, k, rows, factor, 1000 * k + 100 * rows + seed);
                        f.solve();
                        sipm += get_counts(f).ipm;
                    }
                    if(std::abs(obj[0] - obj[1]) > 1e-6 * std::abs(obj[0])) ++bad;
                    sf += it[0];
                    sd += it[1];
                    if(it[0] < it[1]) ++wf; else if(it[1] < it[0]) ++wd; else ++ties;
                }
                std::printf("%2d %2d %.1f %d | %5ld %5ld | %2d %2d %2d | %ld%s\n", k, rows, factor, seeds, sf, sd, wf, wd, ties, sipm, bad ? " BAD" : "");
            }
        }
    }
    return 0;
}
