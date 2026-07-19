#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <limits>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/detail/variadic_helper.hpp"

namespace mippp {

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
        return contains_v<P, Ps...>;
    }
    template <column_property P>
        requires contains_v<P, Ps...>
    constexpr auto & get() & noexcept {
        return std::get<index_of_v<P, Ps...>>(_properties);
    }
    template <column_property P>
        requires contains_v<P, Ps...>
    constexpr const auto & get() const & noexcept {
        return std::get<index_of_v<P, Ps...>>(_properties);
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
        if constexpr(contains_v<P, Qs...>)
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
        return detail::contains_v<P, Ps...>;
    }
    template <column_property P>
        requires detail::contains_v<P, Ps...>
    constexpr const typename P::value_type & get() const noexcept {
        return std::get<detail::index_of_v<P, Ps...>>(_values);
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

// number of pricing rounds spent in the current state (pool or master)
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
// (has to be listed it in both states so that it survives transitions)
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
////////////////////////////////// Strategies /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// The activation/eviction candidates are presented to the strategies as
// entries, i.e. std::pair<const ColumnSeed &, const State &> (see the
// in_pool_entry/in_master_entry aliases of column_manager). A strategy is a
// type providing:
//
//     Pred predicate;
//         a member function object callable with an entry, returning whether
//         the column is a candidate for activation/eviction
//     auto select(candidates, proj) const;
//         restricts the candidates (given as a range of node handles) to the
//         ones to actually activate/evict; proj maps a node handle to its
//         entry so that comparators/samplers can inspect seeds and states
//
// Predicates are plain function objects : a predicate reading a property P
// simply requires P to be listed in the property_list given to the
// column_manager for the corresponding state (enforced at compile time by
// properties_base::get<P>).

namespace detail {
template <typename S, typename Entry>
concept column_predicate = requires(const S & s, const Entry & entry) {
    { s(entry) } -> std::convertible_to<bool>;
};
}  // namespace detail

//////////////////////////////// entry predicates /////////////////////////////

template <typename P>
struct below {
    P::value_type rhs;
    constexpr bool operator()(const auto & entry) const {
        const auto & state = std::get<1>(entry);
        return state.template get<P>() < rhs;
    }
};

template <typename P>
struct above {
    P::value_type rhs;
    constexpr bool operator()(const auto & entry) const {
        const auto & state = std::get<1>(entry);
        return state.template get<P>() > rhs;
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
        const auto & state = std::get<1>(entry);
        return std::ranges::all_of(state.template get<reduced_cost_window<K>>(),
                                   [&](double rc) { return rc > threshold; });
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

///////////////////////////// selection strategies ////////////////////////////

struct none {
    struct {
        constexpr bool operator()(const auto &) const { return false; }
    } predicate;
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R &&, Proj = {}) const {
        return std::views::empty<std::ranges::range_value_t<R>>;
    }
};
template <typename Pred = conjunction<>>
struct all {
    Pred predicate;
    template <typename... T>
        requires std::constructible_from<Pred, T &&...>
    all(T &&... args) : predicate(std::forward<T>(args)...) {}
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj = {}) const {
        return std::views::all(std::forward<R>(candidates));
    }
};
template <typename Pred>
all(Pred) -> all<Pred>;

// the first 'count' candidates, in unspecified order
template <typename Pred>
struct at_most_k : all<Pred> {
    std::size_t count;
    template <typename... T>
        requires std::constructible_from<Pred, T &&...>
    at_most_k(const std::size_t & count_, T &&... args)
        : all<Pred>(std::forward<T>(args)...), count(count_) {}
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj = {}) const {
        return std::views::take(std::forward<R>(candidates),
                                static_cast<std::ptrdiff_t>(count));
    }
};
template <typename Pred>
at_most_k(std::size_t, Pred) -> at_most_k<Pred>;

// 'count' candidates drawn uniformly at random
template <typename Pred, typename Gen>
struct at_most_k_random : at_most_k<Pred> {
    using at_most_k<Pred>::count;
    mutable Gen gen;
    template <typename G, typename... T>
        requires std::constructible_from<Gen, G &&> &&
                     std::constructible_from<Pred, T &&...>
    at_most_k_random(const std::size_t & count_, G && gen_, T &&... args)
        : at_most_k<Pred>(count_, std::forward<T>(args)...)
        , gen(std::forward<G>(gen_)) {}
    template <std::ranges::range R, typename Proj = std::identity>
    auto select(R && candidates, Proj = {}) const {
        std::vector<std::ranges::range_value_t<R>> selected;
        selected.reserve(count);
        std::ranges::sample(candidates, std::back_inserter(selected),
                            static_cast<std::ptrdiff_t>(count), gen);
        return selected;
    }
};
template <typename G, typename Pred>
at_most_k_random(std::size_t, G, Pred) -> at_most_k_random<Pred, G>;

// the 'count' best candidates according to 'cmp' (applied to entries, lesser
// is better)
template <typename Pred, typename Comp>
struct at_most_k_best : at_most_k<Pred> {
    using at_most_k<Pred>::count;
    [[no_unique_address]] Comp cmp;
    template <typename C, typename... T>
        requires std::constructible_from<Comp, C &&> &&
                     std::constructible_from<Pred, T &&...>
    at_most_k_best(const std::size_t & count_, C && cmp_, T &&... args)
        : at_most_k<Pred>(count_, std::forward<T>(args)...)
        , cmp(std::forward<C>(cmp_)) {}
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
template <typename C, typename Pred>
at_most_k_best(std::size_t, C, Pred) -> at_most_k_best<Pred, C>;

using evict_never = none;

// evicts columns whose reduced cost stayed above 'threshold' for their K last
// pricing rounds in the master
template <std::size_t K>
using evict_window_above = all<window_above<K>>;

// evicts columns that are currently unattractive and entered the master at
// least a given number of pricing rounds ago (protects freshly added columns)
using evict_unattractive_aged =
    all<conjunction<above<age>, above<reduced_cost>>>;

}  // namespace mippp
