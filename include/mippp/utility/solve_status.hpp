#pragma once

#include <concepts>
#include <variant>

#include "mippp/detail/variant_helper.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Concepts ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T>
using model_solve_status_t =
    std::decay_t<decltype(std::declval<T &>().solve_status())>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Solve status /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// clang-format off
namespace status {
struct any {
    bool solution_available;
    explicit constexpr any(bool available = false)
        : solution_available(available) {}
};
/////////////////////////////////// Unknown ///////////////////////////////////
struct unknown : any { using any::any; };
////////////////////////////////// Completed //////////////////////////////////
struct completed : any { using any::any; };
struct optimal : completed { constexpr optimal() : completed(true) {} };
struct optimal_face_unbounded : optimal {};  // infinite number of solutions
struct optimal_infeasible_unscaled : optimal {};  // infeasible once unscaled
struct infeasible_or_unbounded : completed {
    constexpr infeasible_or_unbounded() : completed(false) {}
};
struct infeasible : infeasible_or_unbounded {};
struct primal_and_dual_infeasible : infeasible {};
struct unbounded : infeasible_or_unbounded {};
/////////////////////////////////// Stopped ///////////////////////////////////
struct stopped : any { using any::any; };
struct interrupted : stopped { using stopped::stopped; };
struct failed : stopped { using stopped::stopped; };
struct numerical_failure : failed { using failed::failed; };
struct out_of_memory : failed { using failed::failed; };
struct limit_reached : stopped { using stopped::stopped; };
struct time_limit : limit_reached { using limit_reached::limit_reached; };
struct iteration_limit : limit_reached { using limit_reached::limit_reached; };
struct node_limit : limit_reached { using limit_reached::limit_reached; };
struct solution_limit : limit_reached { using limit_reached::limit_reached; };
struct memory_limit : limit_reached { using limit_reached::limit_reached; };

template <variant_of<any> SV>
[[nodiscard]] constexpr bool solution_available(const SV & r) noexcept {
    return std::visit([](any a) { return a.solution_available; }, r);
}
}  // namespace status
// clang-format on

}  // namespace mippp