// Replays fuzz seeds 1124 (Clp_dual: infeasible on a feasible LP) and 1090
// (highs_lp, primal simplex: unknown on an LP of optimum -31, with and
// without the solve that came before the last one).
#include <cxxabi.h>
#include <dlfcn.h>
#include <cstdlib>
#include <print>
#include <string>
#include <typeinfo>
#include <variant>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/solvers/clp/all.hpp"
#include "mippp/solvers/highs/all.hpp"

using namespace mippp;
using namespace mippp::operators;

template <class S>
std::string status_name(const S & s) {
    return std::visit(
        [](const auto & t) {
            int st = 0;
            char * d = abi::__cxa_demangle(typeid(t).name(), nullptr, nullptr, &st);
            std::string n = d ? d : "?";
            std::free(d);
            return n.substr(n.rfind("::") + 2);
        },
        s);
}

// clp_lp has no selector: the entry point is called on the native model.
// min 0.5 x0 + 2.5 x1 over free columns, x0 + 5 x1 >= 4 and 2 x1 == -2, of
// optimum 2 at (9, -1).
void clp_1124(const char * entry) {
    using entry_fun = int(clp::impl::v1::Clp_Simplex *, int);
    clp_lp m;
    const double inf = m.infinity();
    auto x0 = m.add_variable({.obj_coef = 0.5, .lower_bound = -inf, .upper_bound = inf});
    auto x1 = m.add_variable({.obj_coef = 2.5, .lower_bound = -inf, .upper_bound = inf});
    m.add_constraint(x0 + 5 * x1 >= 4);
    m.add_constraint(2 * x1 == -2);
    void * library = dlopen(m.native_api().library_path().c_str(), RTLD_NOW | RTLD_NOLOAD);
    auto solve = reinterpret_cast<entry_fun *>(dlsym(library, entry));
    solve(m.native_model(), 0);
    std::println("clp 1124 {:11}: Clp status {} (0 optimal, 1 primal infeasible, 2 dual infeasible), "
                 "secondary {}, iterations {}, objective {}",
                 entry, m.native_api().status(m.native_model()),
                 m.native_api().secondaryStatus(m.native_model()),
                 m.native_api().numberIterations(m.native_model()),
                 m.native_api().getObjValue(m.native_model()));
}

template <class A>
void highs_1090(const char * name, A algorithm, bool earlier_solve, bool with_x7) {
    highs_lp m;
    m.set_lp_algorithm(algorithm);
    auto v = [&](double c, double lb, double ub) {
        return m.add_variable({.obj_coef = c, .lower_bound = lb, .upper_bound = ub});
    };
    auto x0 = v(0, 0, 5), x1 = v(2, -3, 2), x2 = v(-0.5, -1, 0);
    (void)x0;
    if(earlier_solve) m.solve();
    m.set_objective(-2.5 * x2 + -3 * x1 + 6);
    auto x3 = v(-2.5, -3, 4);
    m.set_objective_coefficient(x1, 0);
    auto x4 = v(2, -4, 1), x5 = v(2, -4, 1), x6 = v(2, -4, 1);
    (void)x4;
    if(with_x7) {
        auto x7 = v(-2, -4, 1);
        m.add_constraint(0.5 * x1 + 2 * x5 + 1 * x7 == 2);
        m.add_constraint(2.5 * x3 >= 3);
        v(0, 3, 8);
        m.remove_variable(x7);
    } else {
        m.add_constraint(0.5 * x1 + 2 * x5 == 2);
        m.add_constraint(2.5 * x3 >= 3);
        v(0, 3, 8);
    }
    v(3, -4, -1);
    auto c2 = m.add_constraint(-3 * x1 + 2.5 * x6 >= -8);
    m.set_constraint_sense(c2, constraint_sense::less_equal);
    m.solve();
    int it = -1;
    m.native_api().getIntInfoValue(m.native_model(), "simplex_iteration_count", &it);
    std::string value = "-";
    if(is_a<status::optimal>(m.get_status())) value = std::to_string(m.get_solution_value());
    std::println("highs 1090 {:16} earlier_solve={} with_x7={}: {} value={} (model status {}, simplex iterations {})",
                 name, earlier_solve, with_x7, status_name(m.get_status()), value,
                 m.native_api().getModelStatus(m.native_model()), it);
}

int main() {
    clp_1124("Clp_dual");
    clp_1124("Clp_primal");
    for(bool earlier : {true, false})
        for(bool x7 : {true, false}) {
            highs_1090("primal_simplex", lp_algorithm::primal_simplex{}, earlier, x7);
            highs_1090("dual_simplex", lp_algorithm::dual_simplex{}, earlier, x7);
            highs_1090("automatic", lp_algorithm::automatic{}, earlier, x7);
        }
}
