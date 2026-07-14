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

using namespace mippp;
using namespace mippp::operators;

using api_type = highs_api;
using lp_type = highs_lp;

// Unbounded knapsack: maximize sum(value[i] * count[i]) subject to
// sum(length[i] * count[i]) <= capacity. Returns {best value, counts}.
static std::pair<double, std::vector<int>> price(
    const std::vector<int> & length, const std::vector<double> & value,
    int capacity) {
    const std::size_t m = length.size();
    std::vector<double> best(static_cast<std::size_t>(capacity) + 1, 0.0);
    std::vector<int> choice(static_cast<std::size_t>(capacity) + 1, -1);
    for(int c = 1; c <= capacity; ++c) {
        best[static_cast<std::size_t>(c)] =
            best[static_cast<std::size_t>(c - 1)];
        for(std::size_t i = 0; i < m; ++i) {
            if(length[i] > c) continue;
            double candidate =
                best[static_cast<std::size_t>(c - length[i])] + value[i];
            if(candidate > best[static_cast<std::size_t>(c)]) {
                best[static_cast<std::size_t>(c)] = candidate;
                choice[static_cast<std::size_t>(c)] = static_cast<int>(i);
            }
        }
    }
    std::vector<int> counts(m, 0);
    for(int c = capacity; c > 0;) {
        int i = choice[static_cast<std::size_t>(c)];
        if(i < 0) {
            --c;  // capacity c not fully used: defer to c - 1
            continue;
        }
        ++counts[static_cast<std::size_t>(i)];
        c -= length[static_cast<std::size_t>(i)];
    }
    return {best[static_cast<std::size_t>(capacity)], std::move(counts)};
}

int main() {
    constexpr int roll_length = 100;
    const std::vector<int> length = {14, 31, 36, 45};  // piece lengths
    const std::vector<double> demand = {211, 395, 610, 97};
    const int m = static_cast<int>(length.size());
    auto orders = std::views::iota(0, m);

    api_type api;
    lp_type model(api);
    using variable_t = lp_type::variable;

    model.set_minimization();

    // Start with one trivial pattern per order (a roll cut into pieces of a
    // single length). patterns[p][o] = how many pieces of order o pattern p
    // yields.
    std::vector<std::vector<int>> patterns;
    std::vector<variable_t> vars;
    for(int o : orders) {
        std::vector<int> pattern(static_cast<std::size_t>(m), 0);
        pattern[static_cast<std::size_t>(o)] =
            roll_length / length[static_cast<std::size_t>(o)];
        vars.push_back(model.add_variable({.obj_coef = 1, .lower_bound = 0}));
        patterns.push_back(std::move(pattern));
    }

    // Meet the demand of every order. The inner lambda captures the outer loop
    // parameter by value ([&, o]): add_constraints registers the xsum
    // expression lazily, so a by-reference capture would dangle once the
    // generator lambda returns.
    auto demand_constrs = model.add_constraints(orders, [&](int o) {
        return xsum(std::views::iota(0, static_cast<int>(patterns.size())),
                    [&, o](int p) {
                        return patterns[static_cast<std::size_t>(p)]
                                       [static_cast<std::size_t>(o)] *
                               vars[static_cast<std::size_t>(p)];
                    }) >= demand[static_cast<std::size_t>(o)];
    });

    // Column-generation loop: solve the restricted master, price a new pattern
    // from the duals, and add it while it has negative reduced cost.
    int rounds = 0;
    for(;;) {
        model.solve();
        ++rounds;
        auto duals = model.get_dual_solution();

        std::vector<double> value(static_cast<std::size_t>(m));
        for(int o : orders)
            value[static_cast<std::size_t>(o)] = duals[demand_constrs(o)];

        auto [best, counts] = price(length, value, roll_length);
        if(best <= 1.0 + 1e-9) break;  // no improving column remains

        auto new_var = model.add_column(
            orders | std::views::filter([&](int o) {
                return counts[static_cast<std::size_t>(o)] > 0;
            }) | std::views::transform([&](int o) {
                return std::make_pair(
                    demand_constrs(o),
                    static_cast<double>(counts[static_cast<std::size_t>(o)]));
            }),
            {.obj_coef = 1, .lower_bound = 0});
        patterns.push_back(std::move(counts));
        vars.push_back(new_var);
    }

    auto sol = model.get_solution();
    int rolls = 0;
    for(const auto & var : vars) rolls += static_cast<int>(std::ceil(sol[var]));

    std::println("LP relaxation bound : {} rolls", model.get_solution_value());
    std::println("rounded-up rolls    : {}", rolls);
    std::println("patterns generated  : {}  (in {} master solves)",
                 patterns.size(), rounds);
    return 0;
}
