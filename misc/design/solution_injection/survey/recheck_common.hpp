// Is an injected point checked by the candidate callback before it becomes the
// incumbent? max x, x integer in [0,10], with a hidden rule x <= 6 that only the
// callback knows: it answers a candidate x > 6 with the weak lazy row
// x <= candidate - 1. On its first call it also injects x = 8, which satisfies
// every row the solver holds, that one included, and breaks the hidden rule.
// Truth: 6. A solver that takes the injected point unchecked ends at 7.75.
// The injected point alone has y = 1 (y costs 0.25), so it shows as "8*".
#pragma once
#include <cmath>
#include <mutex>
#include <print>
#include <string>
#include <vector>

template <class M, class Inject>
void recheck(const char * backend, Inject && inject, int inject_value = 8) {
    using namespace mippp;
    using namespace mippp::operators;
    M model;
    auto x = model.add_integer_variable({.lower_bound = 0, .upper_bound = 10});
    auto y = model.add_binary_variable();
    model.set_maximization();
    model.set_objective(x - 0.25 * y);
    model.add_constraint(x + y <= 11);
    std::mutex mtx;
    std::string seq;
    bool injected = false;
    int rc = -999;
    model.set_candidate_solution_callback([&](auto & handle) {
        const std::scoped_lock lock(mtx);
        auto sol = handle.get_solution();
        const double v = sol[x];
        seq += std::to_string(static_cast<int>(std::lround(v)));
        if(sol[y] > 0.5) seq += "*";  // only the injected point has y = 1
        if(v > 6.5) {
            seq += "r ";
            handle.add_lazy_constraint(x <= std::round(v) - 1);
        } else
            seq += "a ";
        if(!injected && inject_value >= 0) {
            injected = true;
            std::vector<double> p(2);
            p[x.id()] = inject_value;
            p[y.id()] = 1;
            rc = inject(handle, p, false, inject_value - 0.25);
        }
    });
    model.solve();
    std::println("{} inject={} rc={} candidates: {} -> value {} (truth 6)",
                 backend, inject_value, rc, seq, model.get_solution_value());
}

// A lazy constraint the candidate satisfies: does adding it still reject the
// candidate? max x + y, x and y integer in [0,5], x + y <= 7. The callback adds
// x + y <= 100 at every candidate. Truth: 7.
template <class M>
void satisfied(const char * backend) {
    using namespace mippp;
    using namespace mippp::operators;
    M model;
    auto x = model.add_integer_variable({.lower_bound = 0, .upper_bound = 5});
    auto y = model.add_integer_variable({.lower_bound = 0, .upper_bound = 5});
    model.set_maximization();
    model.set_objective(x + y);
    model.add_constraint(x + y <= 7);
    model.set_time_limit(std::chrono::duration<double>(10));
    std::mutex mtx;
    int calls = 0;
    model.set_candidate_solution_callback([&](auto & handle) {
        const std::scoped_lock lock(mtx);
        ++calls;
        handle.add_lazy_constraint(x + y <= 100);
    });
    model.solve();
    const auto & st = model.get_status();
    const std::string name = std::visit(
        [](const auto & s) { return std::string(typeid(s).name()); }, st);
    std::println("{} satisfied-lazy: calls={} status={} value={} (truth 7)",
                 backend, calls, name,
                 status::solution_available(st)
                     ? std::to_string(model.get_solution_value())
                     : std::string("none"));
}
