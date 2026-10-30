#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <functional>
#include <iterator>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/detail/variadic_helper.hpp"
#include "mippp/model_concepts.hpp"

namespace mippp {

// Pricing vocabulary lives in its own namespace: names this generic (`event`,
// `age`, `below`, `conjunction`) would collide with user code, and
// `conjunction` with `std::conjunction` beside a `using namespace std;`, in
// the `using namespace mippp;` that every example opens.
namespace colgen {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Properties //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A column property is a tag type declaring a per-column value together with
// (optional) static event handlers describing how this value reacts to the
// events of the column generation process. An event is any type; a property
// reacts to an event E by declaring
//
//     static void on(value_type &, const E &);
//         called whenever an event of type E is broadcast to the column
//     static value_type initial_value();
//         called to initialize the value (value-initialized if absent)
//
// The handler may be a constrained template, so a single property can react
// to a whole family of events (e.g. every event carrying the 'reduced_cost'
// property, tested with the 'carries<reduced_cost>' concept); see the Events
// section and the common properties below.
//
// Handlers that are not declared cost nothing: the dispatch is resolved at
// compile time, so a column state only pays for the properties it embeds and
// a property only pays for the events it handles. The events broadcast by the
// column manager are:
//
//     activated / deactivated   the column enters / leaves the master model
//     priced                    a pricing round refreshed a pool column
//     master_refreshed          a master solve refreshed a master column
//
// (see the Events section) and user code may broadcast its own events through
// column_manager::update_*_columns.

template <typename T>
concept column_property = requires { typename T::value_type; };

template <typename T>
struct property {
    using value_type = T;
};

template <column_property... Ps>
struct property_list {};

namespace detail {

template <column_property P>
constexpr auto property_initial_value() {
    if constexpr(requires { P::initial_value(); })
        return P::initial_value();
    else
        return typename P::value_type{};
}
// dispatches one event to one property, if that property handles it
template <column_property P, typename V, typename E>
constexpr void notify_property(V & value, const E & event) {
    if constexpr(requires { P::on(value, event); }) P::on(value, event);
}

template <column_property... Ps>
class properties_base {
private:
    [[no_unique_address]] std::tuple<typename Ps::value_type...> _properties;

public:
    constexpr properties_base() noexcept
        : _properties(property_initial_value<Ps>()...) {}

    static constexpr std::size_t num_properties() noexcept {
        return sizeof...(Ps);
    }
    template <column_property P>
    static constexpr bool has_property() noexcept {
        return mippp::detail::contains_v<P, Ps...>;
    }
    template <column_property P>
        requires mippp::detail::contains_v<P, Ps...>
    constexpr auto & get() & noexcept {
        return std::get<mippp::detail::index_of_v<P, Ps...>>(_properties);
    }
    template <column_property P>
        requires mippp::detail::contains_v<P, Ps...>
    constexpr const auto & get() const & noexcept {
        return std::get<mippp::detail::index_of_v<P, Ps...>>(_properties);
    }

    // broadcasts an event to every property; properties that do not handle
    // this event type are skipped at compile time
    template <typename E>
    constexpr void notify(const E & event) {
        (detail::notify_property<Ps>(get<Ps>(), event), ...);
    }
};

// Copies the values of the properties present in both sets. Used to preserve
// information across in_pool <-> in_master transitions : a property listed in
// both states survives round trips (its activated/deactivated handlers may
// then adjust it, e.g. 'age' resets itself while 'times_activated' persists).
template <column_property... Ps, column_property... Qs>
constexpr void transfer_common_properties(const properties_base<Ps...> & from,
                                          properties_base<Qs...> & to) {
    [[maybe_unused]] auto transfer_one = [&]<column_property P>() {
        if constexpr(mippp::detail::contains_v<P, Qs...>)
            to.template get<P>() = from.template get<P>();
    };
    (transfer_one.template operator()<Ps>(), ...);
}

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Events ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A data event is a bag of column-property values : it carries, for a set of
// properties, the values freshly observed for a column. It shares the property
// vocabulary of the column state, so a handler both tests and reads a payload
// through the same tag it stores :
//
//     static void on(value_type & v, const carries<reduced_cost> auto & e) {
//         v = e.template get<reduced_cost>();
//     }
//
// 'carries<P>' holds for any event exposing get<P>(), whichever event carries
// it. Signal events that carry no value (activated / deactivated below) are
// plain empty tags distinguished by their type.
template <column_property... Ps>
struct event {
    std::tuple<typename Ps::value_type...> _values;
    constexpr event(typename Ps::value_type... values)
        : _values(std::move(values)...) {}

    template <column_property P>
    static constexpr bool carries_property() noexcept {
        return mippp::detail::contains_v<P, Ps...>;
    }
    template <column_property P>
        requires mippp::detail::contains_v<P, Ps...>
    constexpr const typename P::value_type & get() const noexcept {
        return std::get<mippp::detail::index_of_v<P, Ps...>>(_values);
    }
};

// an event (or a column state) carries the property P if it exposes get<P>()
template <typename E, typename P>
concept carries = requires(const E & e) { e.template get<P>(); };

// the column entered / left the restricted master model
struct activated {};
struct deactivated {};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Common properties //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// last known reduced cost of the column
struct reduced_cost : property<double> {
    static constexpr void on(double & value,
                             const carries<reduced_cost> auto & e) noexcept {
        value = static_cast<double>(e.template get<reduced_cost>());
    }
};

// last known LP primal value of a master column
struct variable_value : property<double> {
    static constexpr void on(double & value,
                             const carries<variable_value> auto & e) noexcept {
        value = static_cast<double>(e.template get<variable_value>());
    }
};

// last known basis status of a master column
template <lp_basis_status Status>
struct variable_status : property<Status> {
    static constexpr void on(
        Status & value,
        const carries<variable_status<Status>> auto &
            e) noexcept(std::is_nothrow_copy_assignable_v<Status>) {
        value = e.template get<variable_status<Status>>();
    }
};

// number of events carrying a reduced cost received since the column entered
// its current state (pool or master), which is a number of rounds only if
// each round sends the column one such event: update_columns(priced) then
// update_master_columns(master_refreshed) ages master columns twice per round
struct age : property<std::size_t> {
    static constexpr void on(std::size_t & value,
                             const carries<reduced_cost> auto &) noexcept {
        ++value;
    }
    static constexpr void on(std::size_t & value, const activated &) noexcept {
        value = 0;
    }
    static constexpr void on(std::size_t & value,
                             const deactivated &) noexcept {
        value = 0;
    }
};

// number of times the column entered the master model
// (has to be listed in both states so that it survives transitions)
struct times_activated : property<std::size_t> {
    static constexpr void on(std::size_t & value, const activated &) noexcept {
        ++value;
    }
};

// the K last reduced costs of the column, most recent first
// (initialized at 0 to enable max/min senses, use with age)
template <std::size_t K>
struct reduced_cost_window {
    using value_type = std::array<double, K>;
    static constexpr void on(value_type & values,
                             const carries<reduced_cost> auto & e) noexcept {
        std::shift_right(values.begin(), values.end(), 1);
        values.front() = static_cast<double>(e.template get<reduced_cost>());
    }
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Convenience events //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// a pricing round refreshed a pool column's reduced cost
using priced = event<reduced_cost>;

// a restricted master solve refreshed a master column, exposing its LP primal
// value and basis status alongside its reduced cost
template <lp_basis_status Status>
using master_refreshed =
    event<reduced_cost, variable_value, variable_status<Status>>;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// States ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <column_property... Ps>
struct in_pool : detail::properties_base<Ps...> {};

template <typename Variable, column_property... Ps>
struct in_master : detail::properties_base<Ps...> {
    Variable var;
    explicit in_master(Variable v) : var(std::move(v)) {}
};

namespace detail {

template <typename List>
struct in_pool_state_from_property_list;
template <column_property... Ps>
struct in_pool_state_from_property_list<property_list<Ps...>> {
    using type = in_pool<Ps...>;
};

template <typename Variable, typename List>
struct in_master_state_from_property_list;
template <typename Variable, column_property... Ps>
struct in_master_state_from_property_list<Variable, property_list<Ps...>> {
    using type = in_master<Variable, Ps...>;
};

}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Entries ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// the Data of a column_manager whose columns carry nothing but their seed
struct no_data {};

// What the column manager hands to every callback: predicates, selector
// projections, add and remove callbacks, event makers. It refers to the
// column's seed, data and current state, and destructures as
// `auto && [seed, state] = entry`, or as `auto && [seed, data, state] = entry`
// when the manager has a Data.
template <typename Seed, typename State, typename Data = no_data>
struct column_entry {
    const Seed & seed;
    const Data & data;
    const State & state;
};
template <typename Seed, typename State>
struct column_entry<Seed, State, no_data> {
    const Seed & seed;
    const State & state;
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Predicates /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A predicate is any function object callable with an entry, returning
// whether the column is a candidate for activation (given pool entries) or
// for eviction (given master entries). A predicate reading a property P
// requires P to be listed in the property_list of that state (enforced at
// compile time by properties_base::get<P>).
//
// No predicate assumes an optimization sense: negative<reduced_cost> finds
// the improving columns of a minimizing master and positive<reduced_cost>
// those of a maximizing one, and eviction tests the opposite sign, as do
// window_above and window_below.

namespace detail {
template <typename S, typename Entry>
concept column_predicate = requires(const S & s, const Entry & entry) {
    { s(entry) } -> std::convertible_to<bool>;
};
}  // namespace detail

template <typename P>
struct below {
    P::value_type rhs;
    constexpr bool operator()(const auto & entry) const {
        return entry.state.template get<P>() < rhs;
    }
};

template <typename P>
struct above {
    P::value_type rhs;
    constexpr bool operator()(const auto & entry) const {
        return entry.state.template get<P>() > rhs;
    }
};

template <typename P>
struct negative : below<P> {
    explicit negative(typename P::value_type tolerance = {1e-10})
        : below<P>(-tolerance) {}
};

template <typename P>
struct positive : above<P> {
    explicit positive(typename P::value_type tolerance = {1e-10})
        : above<P>(tolerance) {}
};

// the K last reduced costs of the column all stayed above 'threshold'
template <std::size_t K>
struct window_above {
    double threshold = 0.0;
    constexpr bool operator()(const auto & entry) const {
        return std::ranges::all_of(
            entry.state.template get<reduced_cost_window<K>>(),
            [&](double rc) { return rc > threshold; });
    }
};

// the K last reduced costs of the column all stayed below 'threshold'
template <std::size_t K>
struct window_below {
    double threshold = 0.0;
    constexpr bool operator()(const auto & entry) const {
        return std::ranges::all_of(
            entry.state.template get<reduced_cost_window<K>>(),
            [&](double rc) { return rc < threshold; });
    }
};

// age > base + step * times_activated: a column that keeps coming back to
// the master stays there longer each time, which damps eviction thrashing.
// times_activated must be listed in both states, or each activation restarts
// it at 1.
struct age_above_with_backoff {
    std::size_t base = 0;
    std::size_t step = 0;
    constexpr bool operator()(const auto & entry) const {
        const auto & state = entry.state;
        return state.template get<age>() >
               base + step * state.template get<times_activated>();
    }
};

template <typename... Pr>
struct conjunction {
    std::tuple<Pr...> predicates;
    template <typename... T>
        requires std::constructible_from<std::tuple<Pr...>, T &&...>
    conjunction(T &&... args) : predicates(std::forward<T>(args)...) {}
    constexpr bool operator()(const auto & entry) const {
        return std::apply(
            [&](const Pr &... preds) { return (preds(entry) && ...); },
            predicates);
    }
};
template <typename... Pr>
conjunction(Pr...) -> conjunction<Pr...>;

template <typename... Pr>
struct disjunction {
    std::tuple<Pr...> predicates;
    template <typename... T>
        requires std::constructible_from<std::tuple<Pr...>, T &&...>
    disjunction(T &&... args) : predicates(std::forward<T>(args)...) {}
    constexpr bool operator()(const auto & entry) const {
        return std::apply(
            [&](const Pr &... preds) { return (preds(entry) || ...); },
            predicates);
    }
};
template <typename... Pr>
disjunction(Pr...) -> disjunction<Pr...>;

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Selectors //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A selector narrows the candidates that matched the predicate down to the
// ones to activate or evict: select(candidates, proj) returns a forward range
// of candidates, where proj maps a candidate (a node handle) to its entry so
// that comparators and samplers can read seeds, data and states. The manager
// presents the candidates in the order their seeds were first emplaced.

// every candidate, in the manager's order
struct all_candidates {
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj = {}) const {
        return std::views::all(std::forward<R>(candidates));
    }
};

// the first 'count' candidates, in the manager's order
struct at_most_k {
    std::size_t count = 0;
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj = {}) const {
        return std::views::take(std::forward<R>(candidates),
                                static_cast<std::ptrdiff_t>(count));
    }
};

// 'count' candidates drawn uniformly at random
template <typename Gen>
struct at_most_k_random {
    std::size_t count = 0;
    mutable Gen gen;
    template <std::ranges::range R, typename Proj = std::identity>
    auto select(R && candidates, Proj = {}) const {
        std::vector<std::ranges::range_value_t<R>> selected;
        selected.reserve(count);
        std::ranges::sample(candidates, std::back_inserter(selected),
                            static_cast<std::ptrdiff_t>(count), gen);
        return selected;
    }
};
template <typename Gen>
at_most_k_random(std::size_t, Gen) -> at_most_k_random<Gen>;

// the 'count' best candidates according to 'cmp', which compares entries
// (lesser is better), in no particular order
template <typename Comp>
struct at_most_k_best {
    std::size_t count = 0;
    [[no_unique_address]] Comp cmp;
    template <std::ranges::range R, typename Proj = std::identity>
    auto select(R && candidates, Proj proj = {}) const {
        auto selected = std::ranges::to<std::vector>(candidates);
        const std::size_t budget =
            std::min<std::size_t>(count, selected.size());
        std::ranges::nth_element(
            selected, selected.begin() + static_cast<std::ptrdiff_t>(budget),
            cmp, proj);
        selected.resize(budget);
        return selected;
    }
};
template <typename Comp>
at_most_k_best(std::size_t, Comp) -> at_most_k_best<Comp>;

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Seed hashing /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Mixes 'value' into 'seed' as boost::hash_combine does since Boost 1.81.
// Not the older seed ^ (value + 0x9e3779b9 + (seed << 6) + (seed >> 2)): with
// the identity std::hash<int> of libstdc++, it hashes the tuples (1, {2}) and
// (2, {1}) alike.
constexpr std::size_t hash_combine(std::size_t seed,
                                   std::size_t value) noexcept {
    std::size_t x = seed + 0x9e3779b9u + value;
    if constexpr(sizeof(std::size_t) >= 8) {
        constexpr auto m = static_cast<std::size_t>(0xe9846af9b1a615dULL);
        x ^= x >> 32;
        x *= m;
        x ^= x >> 32;
        x *= m;
        x ^= x >> 28;
    } else {
        x ^= x >> 16;
        x *= 0x21f0aaadu;
        x ^= x >> 15;
        x *= 0x735a2d97u;
        x ^= x >> 15;
    }
    return x;
}

namespace detail {
template <typename T>
concept std_hashable = requires(const T & t) {
    { std::hash<T>{}(t) } -> std::convertible_to<std::size_t>;
};
template <typename T>
concept tuple_like_seed = requires { std::tuple_size<T>::value; };
}  // namespace detail

// The default Hash of column_manager: std::hash<T> where it is enabled, so
// such a seed hashes as it always did, otherwise the combination of the
// elements of a range or of a std::pair / std::tuple, each hashed by
// seed_hash in turn.
template <typename T>
struct seed_hash {
    constexpr std::size_t operator()(const T & seed) const {
        if constexpr(detail::std_hashable<T>) {
            return std::hash<T>{}(seed);
        } else if constexpr(std::ranges::input_range<const T>) {
            using element = std::ranges::range_value_t<const T>;
            std::size_t h = 0;
            std::size_t n = 0;
            for(const auto & e : seed) {
                h = hash_combine(h, seed_hash<element>{}(e));
                ++n;
            }
            return hash_combine(h, n);
        } else if constexpr(detail::tuple_like_seed<T>) {
            return std::apply(
                [](const auto &... elements) {
                    std::size_t h = 0;
                    ((h = hash_combine(
                          h,
                          seed_hash<std::remove_cvref_t<decltype(elements)>>{}(
                              elements))),
                     ...);
                    return h;
                },
                seed);
        } else {
            static_assert(false,
                          "MIP++: column_manager cannot hash this seed type; "
                          "give it a std::hash specialization, make it a "
                          "range or a std::pair / std::tuple, or pass a Hash "
                          "to column_manager.");
        }
    }
};

}  // namespace colgen

}  // namespace mippp
