// A transportation plan that cannot be met, and the conflict that explains it.
//
// Showcases the diagnosis of an infeasible model through an irreducible
// infeasible subsystem (IIS): constraints and variable bounds that have no
// solution together, and have one as soon as any of them is dropped. The
// depots hold 640 tonnes for 550 tonnes of orders, yet the solve reports
// infeasible; the IIS narrows the model down to the four figures that
// disagree, printed with the program's own names.
//
// compute_iis_by_deletion runs on every model class: swap the alias to use
// another backend.

#include <cstddef>
#include <map>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/utility/solver_exceptions.hpp"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;

struct route {
    std::string from;
    std::string to;
    double trucks;  // tonnes a week that the trucks on the route can carry
    double cost;    // per tonne
};

// The sides a member needs, read by overload: the most derived tag wins, and a
// routine that cannot tell which side of a row conflicts reports plain member.
// A status variant lists only the tags its path reports, so is_a<member_both>
// would not compile on the row status of the Gurobi and CPLEX routines.
struct needed_sides {
    struct sides {
        bool lower, upper;
    };
    sides operator()(iis_status::absent) const { return {false, false}; }
    sides operator()(iis_status::member) const { return {false, false}; }
    sides operator()(iis_status::member_lower) const { return {true, false}; }
    sides operator()(iis_status::member_upper) const { return {false, true}; }
    sides operator()(iis_status::member_both) const { return {true, true}; }
};

// The IIS names the sides in conflict, and the model still holds their values:
// the deletion filter writes back every bound and side it relaxed.
template <typename Status>
void print_sides(std::string_view name, const Status & status, double lower,
                 double upper) {
    if(!is_a<iis_status::member>(status)) return;
    const auto [lower_side, upper_side] = std::visit(needed_sides{}, status);
    if(lower_side) std::println("  {:<26} >= {}", name, lower);
    if(upper_side) std::println("  {:<26} <= {}", name, upper);
    if(!lower_side && !upper_side)
        std::println("  {:<26} (side not named)", name);
}

// A template, so that the branch is discarded on a model class without a
// native routine: in main, if constexpr would still compile the call.
template <typename Model, typename Print>
void print_native_conflict(Model & model, Print & print_conflict) {
    if constexpr(has_iis<Model>) {
        // thrown when the loaded library lacks the routine, as HiGHS before
        // 1.14 does, or when the routine fails
        try {
            const auto native = model.compute_iis();
            if(native.get_outcome() == iis_outcome::irreducible) {
                std::println("Irreducible conflict, from the native routine:");
                print_conflict(native);
            }
        } catch(const solver_error & e) {
            std::println("No native answer: {}", e.what());
        }
    }
}

// Moves the upper side of a <= row. Gurobi ranges a row through a slack
// column, so its models have no row-bound setters and hold the row as a sense
// and an rhs: the upper side of a <= row is its rhs. A template, so that the
// branch a model class lacks is discarded.
template <typename Model>
void set_upper_side(Model & model, model_constraint_t<Model> c, double value) {
    if constexpr(has_modifiable_constraint_bounds<Model>)
        model.set_constraint_upper_bound(c, value);
    else
        model.set_constraint_rhs(c, value);
}

int main() {
    const std::map<std::string, double> stock = {
        {"Lyon", 300}, {"Marseille", 140}, {"Toulouse", 200}};
    const std::map<std::string, double> demand = {{"Avignon", 100},
                                                  {"Bordeaux", 130},
                                                  {"Grenoble", 110},
                                                  {"Montpellier", 120},
                                                  {"Nice", 90}};
    const std::vector<route> routes = {
        {"Lyon", "Grenoble", 150, 2},    {"Lyon", "Avignon", 40, 4},
        {"Lyon", "Montpellier", 80, 6},  {"Marseille", "Avignon", 120, 2},
        {"Marseille", "Nice", 120, 3},   {"Toulouse", "Montpellier", 100, 3},
        {"Toulouse", "Bordeaux", 160, 2}};
    const auto route_ids = std::views::iota(std::size_t{0}, routes.size());

    lp_type model;
    auto ship = model.add_variables(route_ids);  // tonnes a week, >= 0
    for(std::size_t r : route_ids)
        model.set_variable_upper_bound(ship(r), routes[r].trucks);
    model.set_objective(xsum(
        route_ids, [&](std::size_t r) { return routes[r].cost * ship(r); }));

    // add_constraints reads the terms of each xsum after the generator has
    // returned, so the filters capture the depot or the store by value.
    auto shipped_from = model.add_constraints(
        std::views::keys(stock), [&](const std::string & depot) {
            return xsum(route_ids |
                            std::views::filter([&, depot](std::size_t r) {
                                return routes[r].from == depot;
                            }),
                        ship) <= stock.at(depot);
        });
    auto delivered_to = model.add_constraints(
        std::views::keys(demand), [&](const std::string & store) {
            return xsum(route_ids |
                            std::views::filter([&, store](std::size_t r) {
                                return routes[r].to == store;
                            }),
                        ship) >= demand.at(store);
        });

    model.solve();
    if(!is_a<status::infeasible>(model.get_status())) {
        std::println("The solve does not report the plan infeasible.");
        return 1;
    }
    std::println("No plan meets every order: the solve reports infeasible.");

    // Each path returns a snapshot type of its own, hence the generic lambda.
    auto print_conflict = [&](const auto & iis) {
        for(const std::string & depot : std::views::keys(stock)) {
            const auto c = shipped_from(depot);
            print_sides("shipped from " + depot, iis.get_status(c),
                        model.get_constraint_lower_bound(c),
                        model.get_constraint_upper_bound(c));
        }
        for(const std::string & store : std::views::keys(demand)) {
            const auto c = delivered_to(store);
            print_sides("delivered to " + store, iis.get_status(c),
                        model.get_constraint_lower_bound(c),
                        model.get_constraint_upper_bound(c));
        }
        for(std::size_t r : route_ids) {
            const auto v = ship(r);
            print_sides("shipped " + routes[r].from + " -> " + routes[r].to,
                        iis.get_status(v), model.get_variable_lower_bound(v),
                        model.get_variable_upper_bound(v));
        }
    };

    // The filter solves the model about once per finite bound and side: on a
    // large model, bound the run with its second argument, an iis_limits.
    const auto iis = compute_iis_by_deletion(model);
    if(iis.get_outcome() != iis_outcome::irreducible) {
        std::println("The deletion filter proves no irreducible conflict.");
        return 1;
    }
    std::println("Irreducible conflict, from the deletion filter:");
    print_conflict(iis);

    // This model has a single IIS, so both paths find the same one.
    print_native_conflict(model, print_conflict);

    // With a single IIS, relaxing any one member far enough repairs the plan:
    // here, 10 more tonnes at Marseille. The analyses restore the model's data
    // but not its solution, so the plan is solved again.
    set_upper_side(model, shipped_from("Marseille"), 150);
    model.solve();
    if(is_a<status::optimal>(model.get_status()))
        std::println("With 150 tonnes at Marseille, the plan costs {:g}.",
                     model.get_solution_value());
    return 0;
}
