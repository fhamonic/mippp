// Check of the dual_smoother.hpp sketch (see dual_smoothing.md): cutting stock
// by column generation, minimizing and (negated) maximizing, for several
// alphas. Asserts the Lagrangian value of the center bounds the LP optimum at
// every round. From this directory, with assertions on (one line):
//   g++ -std=c++26 -O1 -I../../../include -I<melon>/include
//       cutting_stock_smoothing.cpp -o cutting_stock_smoothing -ldl
#include <cassert>
#include <cmath>
#include <functional>
#include <print>
#include <random>
#include <ranges>
#include <tuple>
#include <vector>

#include "mippp/solvers/highs/all.hpp"

#include "melon/algorithm/unbounded_knapsack_bnb.hpp"

#include "dual_smoother.hpp"

using namespace mippp;
using namespace mippp::operators;

struct instance {
    int roll_length;
    std::vector<int> length;
    std::vector<double> demand;
    // (piece, price, cap): pieces bought outside, a static column each
    std::vector<std::tuple<std::size_t, double, double>> buy = {};
};

struct outcome {
    double value;
    int master_solves, pricing_rounds, mispricings;
    double best;
};

template <typename Better>
outcome run(const instance & in, double alpha, bool maximize, double ref) {
    const int sign = maximize ? -1 : 1;  // objective coefficient of a roll
    const std::size_t m = in.length.size();
    auto orders = std::views::iota(std::size_t{0}, m);
    highs_lp model;
    using var_t = model_variable_t<highs_lp>;
    maximize ? model.set_maximization() : model.set_minimization();
    auto demand_constrs = model.add_constraints(orders, [&](std::size_t o) {
        return empty_linear_expression<var_t, double> >= in.demand[o];
    });
    for(auto o : orders)
        model.add_column({{demand_constrs(o),
                           static_cast<double>(in.roll_length / in.length[o])}},
                         {.obj_coef = double(sign), .lower_bound = 0});
    // static columns with finite bounds: the master part D is then only
    // piecewise linear, which exercises the interpolation's safe side
    for(auto && [o, price, cap] : in.buy)
        model.add_column(
            {{demand_constrs(o), 1.0}},
            {.obj_coef = sign * price, .lower_bound = 0, .upper_bound = cap});

    colgen::dual_smoother<highs_lp, Better> smoother(alpha);
    outcome res{0, 0, 0, 0, 0};
    for(bool improving = true; improving;) {
        model.solve();
        ++res.master_solves;
        const double z = model.get_solution_value();
        const auto duals = model.get_dual_solution();
        smoother.set_out_point(model.constraints(), duals, z);
        do {
            ++res.pricing_rounds;
            const auto sep = smoother.separation_point();
            // a pattern's value sum_o pi'_o a_o with pi' = sign * pi >= 0
            melon::unbounded_knapsack_bnb knapsack(
                orders,
                [&](std::size_t o) { return sign * sep[demand_constrs(o)]; },
                in.length, in.roll_length);
            knapsack.run();
            // pricing part: kappa * best reduced cost if improving, with
            // kappa = |z| >= sum of the rolls at the LP optimum
            const double best_rc = sign * (1.0 - knapsack.solution_value());
            const double pricing_part =
                std::abs(z) *
                (maximize ? std::max(0.0, best_rc) : std::min(0.0, best_rc));
            smoother.update(pricing_part);
            const double best = *smoother.center_value();
            assert(maximize ? best >= ref - 1e-6 : best <= ref + 1e-6);
            // accept the pattern if it improves at pi_out
            double rc_out = sign;
            for(auto && [o, count] : knapsack.solution_items())
                rc_out -= duals[demand_constrs(o)] * count;
            improving = maximize ? rc_out > 1e-9 : rc_out < -1e-9;
            if(improving)
                model.add_column(knapsack.solution_items() |
                                     std::views::transform([&](auto && p) {
                                         return std::make_pair(
                                             demand_constrs(p.first),
                                             double(p.second));
                                     }),
                                 {.obj_coef = double(sign), .lower_bound = 0});
        } while(!improving && smoother.reduce_alpha() &&
                (++res.mispricings, true));
    }
    res.value = model.get_solution_value();
    res.best = *smoother.center_value();
    return res;
}

instance random_instance(int n, unsigned seed) {
    std::mt19937 gen(seed);
    std::uniform_int_distribution<int> len(5, 50), dem(10, 100);
    instance in{100, {}, {}};
    for(int i = 0; i < n; ++i) {
        in.length.push_back(len(gen));
        in.demand.push_back(dem(gen));
    }
    return in;
}

// a fake model: only model_constraint_t / model_scalar_t are used
struct fake_model {
    model_variable<int, double> add_variable() { return {}; }
    model_constraint<int> add_constraint(auto &&) { return {}; }
};

auto by_row(const std::vector<double> & v) {
    return maps::function(
        [&v](const model_constraint<int> & c) { return v[c.uid()]; });
}

void unit_checks() {
    using constraint = model_constraint<int>;
    std::vector<constraint> rows{constraint(0), constraint(1)};
    colgen::dual_smoother<fake_model, std::greater<>> s(0.8);
    std::vector<double> out1{1.0, 2.0};
    s.set_out_point(rows, by_row(out1), 10.0);
    assert(s.separation_alpha() == 0.0);  // no center yet
    assert(s.separation_point()[rows[1]] == 2.0);
    assert(s.update(-1.0) && *s.center_value() == 9.0);
    std::vector<double> out2{3.0, 4.0};
    s.set_out_point(rows, by_row(out2), 8.0);
    assert(s.separation_alpha() == 0.8);
    assert(std::abs(s.separation_point()[rows[0]] - (0.8 * 1 + 0.2 * 3)) <
           1e-12);
    // schedule 0.8 -> 0.6 -> 0.4 -> 0.2 -> 0, then false
    std::vector<double> alphas;
    while(s.reduce_alpha()) alphas.push_back(s.separation_alpha());
    assert(alphas.size() == 4 && alphas.back() == 0.0);
    assert(std::abs(alphas[0] - 0.6) < 1e-12);
    // a worse value (lower bound) does not move the center
    assert(!s.update(-5.0) && *s.center_value() == 9.0);
    // a new row enters the center at 0
    std::vector<constraint> rows3{constraint(0), constraint(1), constraint(2)};
    std::vector<double> out3{3.0, 4.0, 5.0};
    s.set_out_point(rows3, by_row(out3), 8.0);
    assert(std::abs(s.separation_point()[rows3[2]] - 0.2 * 5.0) < 1e-12);
    s.reset();
    s.set_out_point(rows3, by_row(out3), 8.0);
    assert(s.separation_alpha() == 0.0);
    std::println("unit checks passed");
}

int main() {
    unit_checks();
    const instance small{100, {14, 31, 36, 45}, {211, 395, 610, 97}};
    // pieces may also be bought outside at factor * their share of a roll,
    // up to a third of their demand: dearer than cutting at 1.2 (bought
    // only early on), cheaper at 0.9 (at their cap in the optimum)
    auto with_buy = [](instance in, double factor) {
        for(std::size_t o = 0; o < in.length.size(); ++o)
            in.buy.emplace_back(o, factor * in.length[o] / in.roll_length,
                                in.demand[o] / 3);
        return in;
    };
    for(const auto & [name, in] :
        {std::pair{"small", small}, std::pair{"rand40", random_instance(40, 1)},
         std::pair{"rand80", random_instance(80, 2)},
         std::pair{"rand40 buy 1.2", with_buy(random_instance(40, 1), 1.2)},
         std::pair{"rand40 buy 0.9", with_buy(random_instance(40, 1), 0.9)},
         std::pair{"rand80 buy 1.2", with_buy(random_instance(80, 2), 1.2)},
         std::pair{"rand80 buy 0.9", with_buy(random_instance(80, 2), 0.9)}}) {
        const double ref = run<std::greater<>>(in, 0.0, false, 1e300).value;
        for(double alpha : {0.0, 0.5, 0.8, 0.9}) {
            const auto mn = run<std::greater<>>(in, alpha, false, ref);
            const auto mx = run<std::less<>>(in, alpha, true, -ref);
            std::println(
                "{:14} alpha {:.1f} | min: z {:.4f} best {:.4f} solves {:3} "
                "rounds {:3} mispricings {:2} | max: z {:.4f} best {:.4f} "
                "solves {:3} rounds {:3} mispricings {:2}",
                name, alpha, mn.value, mn.best, mn.master_solves,
                mn.pricing_rounds, mn.mispricings, mx.value, mx.best,
                mx.master_solves, mx.pricing_rounds, mx.mispricings);
            assert(std::abs(mn.value - ref) < 1e-6 &&
                   std::abs(mx.value + ref) < 1e-6);
            assert(std::abs(mn.best - ref) < 1e-6 &&
                   std::abs(mx.best + ref) < 1e-6);
        }
    }
}
