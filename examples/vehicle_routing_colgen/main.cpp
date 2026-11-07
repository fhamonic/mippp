// Vehicle routing with time windows, by column generation over routes, then
// price-and-branch.
//
// Showcases colgen::column_manager fed by a labeling pricer that returns
// several columns per round, and an integer plan built from every generated
// column. A route leaves the depot, serves customers within their time windows
// and the vehicle's capacity, and comes back. The master LP covers every
// customer at least once by routes of least total cost; pricing is the
// elementary shortest path problem with resource constraints, solved by
// forward labeling, and each round adds up to 100 routes of negative reduced
// cost. A MILP then partitions the customers among the generated routes; the
// LP optimum bounds the cost of every plan from below.
//
// The instance is Solomon's R101 (Solomon, M. M. (1987), "Algorithms for the
// vehicle routing and scheduling problems with time window constraints",
// Operations Research 35(2); data from SINTEF's TOP benchmark pages,
// https://www.sintef.no/projectweb/top/vrptw/), restricted to the depot and
// its first n customers, with vehicles of capacity 200, as many as needed.
// Distances and travel times are Euclidean, truncated to one decimal, the
// convention of the published optima. With HiGHS, the program proves 617.1
// optimal for n = 25 and finds 1044.0 for n = 50; for n = 100 it stops at
// 1644.0, above the published 1637.7, which takes more than price-and-branch,
// such as branch-and-price.
//
// The number of customers, from 1 to 100, is an optional argument, 25 by
// default. Swap the two aliases, and the include, to use other backends: an LP
// model with dual values, e.g. clp_lp or gurobi_lp, and a MILP model with
// add_column, e.g. gurobi_milp or cplex_milp.

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <print>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/solvers/highs/all.hpp"
#include "mippp/utility/column_manager.hpp"

using namespace mippp;
using namespace mippp::operators;

using lp_type = highs_lp;
using milp_type = highs_milp;

struct site {
    int x, y, demand, ready, due, service;
};

// R101 as published, in units, not tenths: the depot, then customers 1 to 100.
// clang-format off
constexpr site r101[] = {
    { 35,  35,   0,   0, 230,   0},
    { 41,  49,  10, 161, 171,  10},
    { 35,  17,   7,  50,  60,  10},
    { 55,  45,  13, 116, 126,  10},
    { 55,  20,  19, 149, 159,  10},
    { 15,  30,  26,  34,  44,  10},
    { 25,  30,   3,  99, 109,  10},
    { 20,  50,   5,  81,  91,  10},
    { 10,  43,   9,  95, 105,  10},
    { 55,  60,  16,  97, 107,  10},
    { 30,  60,  16, 124, 134,  10},
    { 20,  65,  12,  67,  77,  10},
    { 50,  35,  19,  63,  73,  10},
    { 30,  25,  23, 159, 169,  10},
    { 15,  10,  20,  32,  42,  10},
    { 30,   5,   8,  61,  71,  10},
    { 10,  20,  19,  75,  85,  10},
    {  5,  30,   2, 157, 167,  10},
    { 20,  40,  12,  87,  97,  10},
    { 15,  60,  17,  76,  86,  10},
    { 45,  65,   9, 126, 136,  10},
    { 45,  20,  11,  62,  72,  10},
    { 45,  10,  18,  97, 107,  10},
    { 55,   5,  29,  68,  78,  10},
    { 65,  35,   3, 153, 163,  10},
    { 65,  20,   6, 172, 182,  10},
    { 45,  30,  17, 132, 142,  10},
    { 35,  40,  16,  37,  47,  10},
    { 41,  37,  16,  39,  49,  10},
    { 64,  42,   9,  63,  73,  10},
    { 40,  60,  21,  71,  81,  10},
    { 31,  52,  27,  50,  60,  10},
    { 35,  69,  23, 141, 151,  10},
    { 53,  52,  11,  37,  47,  10},
    { 65,  55,  14, 117, 127,  10},
    { 63,  65,   8, 143, 153,  10},
    {  2,  60,   5,  41,  51,  10},
    { 20,  20,   8, 134, 144,  10},
    {  5,   5,  16,  83,  93,  10},
    { 60,  12,  31,  44,  54,  10},
    { 40,  25,   9,  85,  95,  10},
    { 42,   7,   5,  97, 107,  10},
    { 24,  12,   5,  31,  41,  10},
    { 23,   3,   7, 132, 142,  10},
    { 11,  14,  18,  69,  79,  10},
    {  6,  38,  16,  32,  42,  10},
    {  2,  48,   1, 117, 127,  10},
    {  8,  56,  27,  51,  61,  10},
    { 13,  52,  36, 165, 175,  10},
    {  6,  68,  30, 108, 118,  10},
    { 47,  47,  13, 124, 134,  10},
    { 49,  58,  10,  88,  98,  10},
    { 27,  43,   9,  52,  62,  10},
    { 37,  31,  14,  95, 105,  10},
    { 57,  29,  18, 140, 150,  10},
    { 63,  23,   2, 136, 146,  10},
    { 53,  12,   6, 130, 140,  10},
    { 32,  12,   7, 101, 111,  10},
    { 36,  26,  18, 200, 210,  10},
    { 21,  24,  28,  18,  28,  10},
    { 17,  34,   3, 162, 172,  10},
    { 12,  24,  13,  76,  86,  10},
    { 24,  58,  19,  58,  68,  10},
    { 27,  69,  10,  34,  44,  10},
    { 15,  77,   9,  73,  83,  10},
    { 62,  77,  20,  51,  61,  10},
    { 49,  73,  25, 127, 137,  10},
    { 67,   5,  25,  83,  93,  10},
    { 56,  39,  36, 142, 152,  10},
    { 37,  47,   6,  50,  60,  10},
    { 37,  56,   5, 182, 192,  10},
    { 57,  68,  15,  77,  87,  10},
    { 47,  16,  25,  35,  45,  10},
    { 44,  17,   9,  78,  88,  10},
    { 46,  13,   8, 149, 159,  10},
    { 49,  11,  18,  69,  79,  10},
    { 49,  42,  13,  73,  83,  10},
    { 53,  43,  14, 179, 189,  10},
    { 61,  52,   3,  96, 106,  10},
    { 57,  48,  23,  92, 102,  10},
    { 56,  37,   6, 182, 192,  10},
    { 55,  54,  26,  94, 104,  10},
    { 15,  47,  16,  55,  65,  10},
    { 14,  37,  11,  44,  54,  10},
    { 11,  31,   7, 101, 111,  10},
    { 16,  22,  41,  91, 101,  10},
    {  4,  18,  35,  94, 104,  10},
    { 28,  18,  26,  93, 103,  10},
    { 26,  52,   9,  74,  84,  10},
    { 26,  35,  15, 176, 186,  10},
    { 31,  67,   3,  95, 105,  10},
    { 15,  19,   1, 160, 170,  10},
    { 22,  22,   2,  18,  28,  10},
    { 18,  24,  22, 188, 198,  10},
    { 26,  27,  27, 100, 110,  10},
    { 25,  24,  20,  39,  49,  10},
    { 22,  27,  11, 135, 145,  10},
    { 25,  21,  12, 133, 143,  10},
    { 19,  21,  10,  58,  68,  10},
    { 20,  26,   9,  83,  93,  10},
    { 18,  18,  17, 185, 195,  10},
};
// clang-format on
constexpr int capacity = 200;

// The depot and customers 1 to n of R101, times and distances in tenths.
struct instance {
    std::vector<site> sites;
    std::vector<std::vector<int>> dist;
    std::size_t num_customers() const { return sites.size() - 1; }
};

int isqrt(int v) {
    int r = 0;
    while((r + 1) * (r + 1) <= v) ++r;
    return r;
}

instance make_instance(std::size_t n) {
    instance inst;
    for(std::size_t i = 0; i <= n; ++i) {
        site s = r101[i];
        s.ready *= 10;
        s.due *= 10;
        s.service *= 10;
        inst.sites.push_back(s);
    }
    inst.dist.assign(n + 1, std::vector<int>(n + 1));
    for(std::size_t i = 0; i <= n; ++i)
        for(std::size_t j = 0; j <= n; ++j) {
            const int dx = r101[i].x - r101[j].x;
            const int dy = r101[i].y - r101[j].y;
            // Truncated, not rounded: rounded distances have other optima
            // than the published ones.
            inst.dist[i][j] = isqrt(100 * (dx * dx + dy * dy));
        }
    return inst;
}

// the customers in visiting order, without the depot
using route = std::vector<std::size_t>;

int route_cost(const instance & inst, const route & r) {
    int cost = 0;
    std::size_t from = 0;
    for(std::size_t c : r) {
        cost += inst.dist[from][c];
        from = c;
    }
    return cost + inst.dist[from][0];
}

// indexed by customer number; bit 0, the depot, stays clear
using customer_set = std::bitset<std::size(r101)>;

// A path from the depot, ending at node.
struct label {
    std::size_t node = 0;
    std::size_t pred = 0;  // the label it extends
    double cost = 0.0;     // reduced cost
    int time = 0;          // start of service at node
    int load = 0;
    customer_set unreachable;  // visited, or out of reach
    bool dominated = false;
};

// Marks the customers that time or capacity put out of the label's reach. These
// marks are the only feasibility test of an extension, which checks neither
// due dates nor the capacity, and they keep a label's time within the buckets:
// marking only the visited customers builds routes that break time windows,
// and indexes past the buckets. Marking the unreachable ones too is Feillet et
// al.'s strengthening (Networks 44(3), 2004): labels then dominate each other
// when they differ only in customers neither can reach, and without it pricing
// 100 customers does not finish its first round in two minutes. Testing the
// direct arc only is exact because a detour through another customer never
// arrives earlier: truncation breaks the triangle inequality by at most a
// tenth, and every service time is longer. With zero service times, the pricer
// could miss routes.
void mark_unreachable(const instance & inst, label & l) {
    const int leave = l.time + inst.sites[l.node].service;
    for(std::size_t j = 1; j <= inst.num_customers(); ++j) {
        const site & s = inst.sites[j];
        if(l.load + s.demand > capacity || leave + inst.dist[l.node][j] > s.due)
            l.unreachable.set(j);
    }
}

// Every extension of b is open to a, at no greater reduced cost. Without the
// subset test, a label would discard one that can still visit customers it
// cannot, the pricer would miss routes, and the loop would stop above the LP
// bound.
bool dominates(const label & a, const label & b) {
    return a.cost <= b.cost && a.time <= b.time && a.load <= b.load &&
           (a.unreachable & ~b.unreachable).none();
}

// Up to k routes of reduced cost below -tolerance, each with its cost: one of
// least reduced cost if any is below -tolerance, then the least among those
// whose labels survive dominance. Dominance drops routes, so these are not the
// k least, and fewer than k may come back while more exist; the bound needs
// only the least. Arcs into customer j cost dist - pi[j], negative for many: a
// path of positive reduced cost may still close below zero, and cutting it off
// would miss routes.
std::vector<std::pair<route, int>> price_routes(const instance & inst,
                                                const std::vector<double> & pi,
                                                double tolerance,
                                                std::size_t k) {
    const std::size_t n = inst.num_customers();
    const site & depot = inst.sites[0];
    std::vector<label> labels(1);  // the depot, at time 0
    mark_unreachable(inst, labels[0]);
    std::vector<std::vector<std::size_t>> at_node(n + 1);
    std::vector<std::vector<std::size_t>> by_time(
        static_cast<std::size_t>(
            std::ranges::max(inst.sites, {}, &site::due).due) +
        1);
    std::vector<std::pair<double, std::size_t>> closed;
    auto extend = [&](std::size_t index) {
        // a copy: labels.push_back below reallocates
        const label from = labels[index];
        const int leave = from.time + inst.sites[from.node].service;
        if(from.node != 0 && leave + inst.dist[from.node][0] <= depot.due) {
            const double reduced_cost = from.cost + inst.dist[from.node][0];
            if(reduced_cost < -tolerance)
                closed.emplace_back(reduced_cost, index);
        }
        for(std::size_t j = 1; j <= n; ++j) {
            if(from.unreachable[j]) continue;
            label to{.node = j,
                     .pred = index,
                     .cost = from.cost + inst.dist[from.node][j] - pi[j],
                     .time = std::max(inst.sites[j].ready,
                                      leave + inst.dist[from.node][j]),
                     .load = from.load + inst.sites[j].demand,
                     .unreachable = from.unreachable};
            to.unreachable.set(j);
            mark_unreachable(inst, to);
            auto & rivals = at_node[j];
            if(std::ranges::any_of(rivals, [&](std::size_t r) {
                   return dominates(labels[r], to);
               }))
                continue;
            for(std::size_t r : rivals)
                if(dominates(to, labels[r])) labels[r].dominated = true;
            std::erase_if(rivals,
                          [&](std::size_t r) { return labels[r].dominated; });
            rivals.push_back(labels.size());
            by_time[static_cast<std::size_t>(to.time)].push_back(labels.size());
            labels.push_back(to);
        }
    };
    extend(0);
    // In time order, a label is dominated, if ever, before it is extended:
    // creation order extends dominated labels and doubles pricing time at 100
    // customers. Service times are positive, so extensions land in later
    // buckets only, never in the one being read.
    for(const auto & bucket : by_time)
        for(std::size_t index : bucket)
            if(!labels[index].dominated) extend(index);
    // The pairs are distinct, label indices breaking ties in reduced cost:
    // the routes kept do not depend on how the standard library sorts ties.
    const auto kept = std::ranges::next(
        closed.begin(), static_cast<std::ptrdiff_t>(k), closed.end());
    std::ranges::partial_sort(closed, kept);
    std::vector<std::pair<route, int>> best;
    for(std::size_t last :
        std::ranges::subrange(closed.begin(), kept) | std::views::values) {
        route r;
        for(std::size_t l = last; l != 0; l = labels[l].pred)
            r.push_back(labels[l].node);
        std::ranges::reverse(r);
        const int cost = route_cost(inst, r);
        best.emplace_back(std::move(r), cost);
    }
    return best;
}

int main(int argc, char ** argv) {
    const std::size_t n =
        (argc > 1) ? static_cast<std::size_t>(std::atoi(argv[1])) : 25;
    const instance inst = make_instance(n);
    auto customers = std::views::iota(std::size_t{1}, n + 1);

    lp_type model;
    model.set_minimization();
    auto cover_rows = model.add_constraints(customers, [](std::size_t) {
        return empty_linear_expression<model_variable_t<lp_type>, double> >=
               1.0;
    });

    // A column's seed is its route, and its Data the route's cost.
    colgen::column_manager<lp_type, route,
                           colgen::property_list<colgen::reduced_cost>,
                           colgen::property_list<>, int>
        columns;
    auto add_route = [&](lp_type & master, const auto & entry) {
        return master.add_column(
            entry.seed | std::views::transform([&](std::size_t c) {
                return std::make_pair(cover_rows(c), 1.0);
            }),
            {.obj_coef = static_cast<double>(entry.data), .lower_bound = 0});
    };
    // one route per customer: without them, the first master is infeasible
    // and has no duals
    for(std::size_t c : customers) {
        route alone{c};
        const int cost = route_cost(inst, alone);
        columns.emplace_master_column(model, std::move(alone), add_route, cost);
    }

    constexpr std::size_t routes_per_round = 100;
    // Reduced costs recomputed from the duals are off by the solver's dual
    // errors, of either sign (about 1e-11 with HiGHS and 1e-12 with Clp at 100
    // customers). Without a tolerance above them, rounding decides whether a
    // route at zero reduced cost reads improving, and the master takes in
    // routes that cannot lower the bound.
    constexpr double tolerance = 1e-6;
    std::size_t rounds = 0;
    for(;;) {
        model.solve();
        ++rounds;
        // read the master before activate_columns modifies it
        auto duals = model.get_dual_solution();
        std::vector<double> pi(n + 1, 0.0);
        for(std::size_t c : customers) pi[c] = duals[cover_rows(c)];

        columns.emplace_pool_columns(
            price_routes(inst, pi, tolerance, routes_per_round));
        // emplace_pool_columns does not price: the new routes hold a zero
        // reduced cost until this, and activation would find none
        columns.update_pool_columns([&](const auto & entry) {
            double reduced_cost = entry.data;
            for(std::size_t c : entry.seed) reduced_cost -= pi[c];
            return colgen::priced{reduced_cost};
        });
        // no improving column: the master is still as solved last
        if(columns.activate_columns(
               model, colgen::negative<colgen::reduced_cost>{tolerance},
               add_route) == 0)
            break;
    }

    if(!is_a<status::optimal>(model.get_status())) {
        std::println("the last master solve is not optimal: no bound");
        return 1;
    }
    const double bound = model.get_solution_value();

    // Price-and-branch: every customer on exactly one of the generated routes.
    milp_type milp;
    milp.set_minimization();
    auto partition_rows = milp.add_constraints(customers, [](std::size_t) {
        return empty_linear_expression<model_variable_t<milp_type>, double> ==
               1.0;
    });
    std::vector<std::pair<route, model_variable_t<milp_type>>> plan;
    auto add_integer_route = [&](const auto & entry) {
        auto used = milp.add_column(
            entry.seed | std::views::transform([&](std::size_t c) {
                return std::make_pair(partition_rows(c), 1.0);
            }),
            {.obj_coef = static_cast<double>(entry.data), .lower_bound = 0});
        milp.set_integer(used);
        plan.emplace_back(entry.seed, used);
    };
    std::ranges::for_each(columns.master_columns(), add_integer_route);
    // Empty here, since every improving route enters the master, but no longer
    // once activation takes a selector such as at_most_k_best: its leftover
    // routes may still serve the plan.
    std::ranges::for_each(columns.pool_columns(), add_integer_route);
    milp.solve();
    if(!is_a<status::optimal>(milp.get_status())) {
        std::println("the MILP is not solved: no plan");
        return 1;
    }
    auto solution = milp.get_solution();
    std::vector<route> chosen;
    for(const auto & [r, used] : plan)
        if(solution[used] > 0.5) chosen.push_back(r);
    std::ranges::sort(chosen);
    int cost = 0;
    for(const route & r : chosen) cost += route_cost(inst, r);

    // Costs are integers in tenths, so no plan costs less than the bound
    // rounded up. No route has a reduced cost below -tolerance, so a plan of
    // k <= n routes costs at least the sum of the duals minus k * tolerance,
    // and the master's value is that sum up to the solver's primal-dual gap:
    // rounded up as is, a bound just above a whole number of tenths would pass
    // the optimum.
    const double lower = std::ceil(bound - static_cast<double>(n) * tolerance);
    const double gap = (cost - lower) / cost;
    std::println("customers : {}", n);
    std::println("LP bound  : {:.2f}", bound / 10);
    std::println("rounds    : {}", rounds);
    std::println("columns   : {}, {} of them in the master",
                 columns.num_columns(), columns.num_master_columns());
    std::println("vehicles  : {}", chosen.size());
    for(const route & r : chosen) {
        std::print("    0");
        for(std::size_t c : r) std::print(" -> {}", c);
        std::println(" -> 0 ({:.1f})", route_cost(inst, r) / 10.0);
    }
    std::println("plan cost : {:.1f} (gap {:.2f}%){}", cost / 10.0, 100.0 * gap,
                 cost == lower ? ", proven optimal" : "");
    return 0;
}
