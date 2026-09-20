// Cutting-stock solved by column generation.
//
// Showcases reading dual values (get_dual_solution) and adding columns to a
// live model (add_column). Stock rolls of a fixed length must be cut to meet a
// demand for shorter pieces while minimizing the number of rolls used. Cutting
// patterns are generated on demand: the pricing subproblem (an unbounded
// knapsack over the current duals) is solved with a small inline DP so this
// example depends only on mippp.
//
// Swap the two aliases to use another LP backend exposing dual solutions.

#include <cmath>
#include <print>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/solvers/highs/all.hpp"

#include "melon/algorithm/unbounded_knapsack_bnb.hpp"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;

int main() {
    constexpr int roll_length = 100;
    const std::vector<int> length = {14, 31, 36, 45};  // piece lengths
    const std::vector<double> demand = {211, 395, 610, 97};
    const std::size_t m = length.size();
    auto orders = std::views::iota(std::size_t{0}, m);

    lp_type model;
    using var_t = model_variable_t<lp_type>;

    model.set_minimization();

    auto demand_constrs = model.add_constraints(orders, [&](int o) {
        return empty_linear_expression<var_t, double> >= demand[o];
    });

    // Start with trivial patterns (a roll cut into pieces of a single length).
    std::vector<var_t> vars;
    std::vector<std::vector<int>> patterns;
    for(auto && o : orders) {
        std::vector<int> pattern(m, 0);
        pattern[o] = roll_length / length[o];
        vars.emplace_back(model.add_column(
            {{demand_constrs(o), static_cast<double>(pattern[o])}},
            {.obj_coef = 1, .lower_bound = 0}));
        patterns.emplace_back(std::move(pattern));
    }

    // Column-generation loop: solve the restricted master, price a new pattern
    // from the duals, and add it while it has negative reduced cost.
    for(;;) {
        model.solve();

        melon::unbounded_knapsack_bnb pricer(
            orders,
            [&, duals = model.get_dual_solution()](int o) {
                return duals[demand_constrs(o)];
            },
            length, roll_length);

        if(pricer.run().solution_value() - 1 <= 1e-6)
            break;  // no improving column remains

        vars.emplace_back(model.add_column(
            pricer.solution_items() | std::views::transform([&](auto && p) {
                auto && [order, count] = p;
                return std::make_pair(demand_constrs(order),
                                      static_cast<double>(count));
            }),
            {.obj_coef = 1, .lower_bound = 0}));

        std::vector<int> pattern(m, 0);
        for(auto && [o, count] : pricer.solution_items()) pattern[o] = count;
        patterns.emplace_back(std::move(pattern));
    }

    auto sol = model.get_solution();
    int rolls = 0;
    for(const auto & var : vars) rolls += static_cast<int>(std::ceil(sol[var]));

    std::println("LP relaxation bound : {} rolls", model.get_solution_value());
    std::println("rounded-up rolls    : {}", rolls);
    std::println("patterns generated  : {}", patterns.size());
    return 0;
}
