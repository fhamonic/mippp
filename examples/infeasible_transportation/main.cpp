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
// another backend. Several solvers also have a routine of their own,
// model.compute_iis(); docs/solving/infeasibility.md covers both paths.

#include <cstddef>
#include <map>
#include <print>
#include <ranges>
#include <string>
#include <vector>

#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/iis_by_deletion.hpp"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;

struct route {
    std::string from;
    std::string to;
    double trucks;  // tonnes a week that the trucks on the route can carry
    double cost;    // per tonne
};

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

    // The filter solves the model about once per finite bound and side: on a
    // large model, bound the run with an iis_limits, its last argument.
    const auto iis = compute_iis_by_deletion(model);
    if(!is<iis_outcome::irreducible>(iis.get_outcome())) {
        std::println("The deletion filter proves no irreducible conflict.");
        return 1;
    }
    // Each line is a side that the IIS names, with the program's figure for
    // it. A depot's row has an upper side only, a store's a lower side only,
    // and a route's tonnage both bounds, 0 and its trucks.
    std::println("Irreducible conflict:");
    for(const auto & [depot, tonnes] : stock)
        if(iis_status::sides_of(iis.get_status(shipped_from(depot))).upper)
            std::println("  {:<26} <= {}", "shipped from " + depot, tonnes);
    for(const auto & [store, tonnes] : demand)
        if(iis_status::sides_of(iis.get_status(delivered_to(store))).lower)
            std::println("  {:<26} >= {}", "delivered to " + store, tonnes);
    for(std::size_t r : route_ids) {
        const iis_sides sides = iis_status::sides_of(iis.get_status(ship(r)));
        const std::string name =
            "shipped " + routes[r].from + " -> " + routes[r].to;
        if(sides.lower) std::println("  {:<26} >= 0", name);
        if(sides.upper) std::println("  {:<26} <= {}", name, routes[r].trucks);
    }

    // With a single IIS, relaxing any one member far enough repairs the plan:
    // here, trucks for 50 tonnes instead of 40 from Lyon to Avignon,
    // routes[1]. The filter writes back every bound and side it relaxed, but
    // leaves the status unknown, so the plan is solved again.
    model.set_variable_upper_bound(ship(1), 50);
    model.solve();
    if(is_a<status::optimal>(model.get_status()))
        std::println("With 50 tonnes Lyon -> Avignon, the plan costs {:g}.",
                     model.get_solution_value());
    return 0;
}
