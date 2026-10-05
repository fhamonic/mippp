#pragma once

#include <chrono>
#include <cstddef>
#include <limits>
#include <stop_token>
#include <variant>

#include "mippp/utility/variant.hpp"

namespace mippp {

struct iis_limits {
    std::size_t max_solves = std::numeric_limits<std::size_t>::max();
    // one budget for the whole call, not per trial: it becomes a single
    // deadline when the call starts. NaN or negative throws
    // std::invalid_argument, and infinity means no deadline.
    std::chrono::duration<double> time_limit{
        std::numeric_limits<double>::infinity()};
    std::stop_token stop_token = {};
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// IIS outcome /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// How an IIS run ended, as status says how a solve ended. Each path's variant
// lists the tags it can tell apart, so test with is_a.
// clang-format off
namespace iis_outcome {
struct any {
    // the members form a subsystem proven infeasible, minimal or not
    bool conflict_available;
    explicit constexpr any(bool available = false)
        : conflict_available(available) {}
};
////////////////////////////////// Completed //////////////////////////////////
struct completed : any { using any::any; };
// with no member, the background alone is infeasible
struct irreducible : completed { constexpr irreducible() : completed(true) {} };
struct feasible : completed { constexpr feasible() : completed(false) {} };
///////////////////////////////// Incomplete //////////////////////////////////
// No decision, for a cause the routine does not name: a gap in its proof,
// numerical trouble, or a stop it cannot tell from those. It does not imply
// that a limit stopped the run.
struct incomplete : any { using any::any; };
// a filter trial proved neither infeasibility nor a feasible point
struct inconclusive_trial : incomplete { using incomplete::incomplete; };
// cut short from outside the routine
struct stopped : incomplete { using incomplete::incomplete; };
struct interrupted : stopped { using stopped::stopped; };
struct limit_reached : stopped { using stopped::stopped; };
struct time_limit : limit_reached { using limit_reached::limit_reached; };
struct solve_limit : limit_reached { using limit_reached::limit_reached; };
struct iteration_limit : limit_reached { using limit_reached::limit_reached; };
struct node_limit : limit_reached { using limit_reached::limit_reached; };
struct memory_limit : limit_reached { using limit_reached::limit_reached; };

template <variant_of<any> O>
[[nodiscard]] constexpr bool conflict_available(const O & o) noexcept {
    return std::visit([](any a) { return a.conflict_available; }, o);
}
}  // namespace iis_outcome
// clang-format on

}  // namespace mippp
