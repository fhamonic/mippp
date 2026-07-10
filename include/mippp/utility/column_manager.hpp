#ifndef MIPPP_COLUMN_GENERATION_HPP
#define MIPPP_COLUMN_GENERATION_HPP

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/linear_expression.hpp"

namespace fhamonic::mippp {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Additional concepts /////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Properties //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A column property is a tag type declaring a per-column value together with
// (optional) static hooks describing how this value reacts to the events of
// the column generation process:
//
//     static void on_priced(value_type &, scalar reduced_cost);
//         called once per pricing round (see column_manager::update_columns)
//     static void on_activated(value_type &);
//         called when the column enters the master model
//     static void on_deactivated(value_type &);
//         called when the column is evicted from the master model
//     static value_type initial_value();
//         called to initialize the value (value-initialized if absent)
//
// Hooks that are not declared cost nothing: the dispatch is resolved at
// compile time, so a column state only pays for the properties it embeds and
// a property only pays for the hooks it declares.

template <typename T>
concept column_property = requires { typename T::value_type; };

template <typename T>
struct property {
    using value_type = T;
};

template <column_property... Ps>
struct property_list {};

namespace detail {

template <typename T, typename... Ts>
inline constexpr bool contains_v = (std::is_same_v<T, Ts> || ...);

template <typename T, typename... Ts>
    requires contains_v<T, Ts...>
inline constexpr std::size_t index_of_v = []() {
    constexpr std::array<bool, sizeof...(Ts)> matches{std::is_same_v<T, Ts>...};
    return static_cast<std::size_t>(std::ranges::find(matches, true) -
                                    matches.begin());
}();

template <column_property P>
constexpr auto property_initial_value() {
    if constexpr(requires { P::initial_value(); })
        return P::initial_value();
    else
        return typename P::value_type{};
}
template <column_property P, typename V, typename S>
constexpr void notify_priced(V & value, const S & reduced_cost) {
    if constexpr(requires { P::on_priced(value, reduced_cost); })
        P::on_priced(value, reduced_cost);
}
template <column_property P, typename V>
constexpr void notify_activated(V & value) {
    if constexpr(requires { P::on_activated(value); }) P::on_activated(value);
}
template <column_property P, typename V>
constexpr void notify_deactivated(V & value) {
    if constexpr(requires { P::on_deactivated(value); })
        P::on_deactivated(value);
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

    template <typename S>
    constexpr void notify_priced(const S & reduced_cost) {
        (detail::notify_priced<Ps>(get<Ps>(), reduced_cost), ...);
    }
    constexpr void notify_activated() {
        (detail::notify_activated<Ps>(get<Ps>()), ...);
    }
    constexpr void notify_deactivated() {
        (detail::notify_deactivated<Ps>(get<Ps>()), ...);
    }
};

// Copies the values of the properties present in both sets. Used to preserve
// information across in_pool <-> in_master transitions : a property listed in
// both states survives round trips (its on_activated/on_deactivated hooks may
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

//////////////////////////// property list algebra ////////////////////////////

template <typename List, column_property P>
struct list_append_unique;
template <column_property... Ps, column_property P>
struct list_append_unique<property_list<Ps...>, P>
    : std::conditional<contains_v<P, Ps...>, property_list<Ps...>,
                       property_list<Ps..., P>> {};

template <typename Acc, column_property... Ps>
struct fold_append_unique {
    using type = Acc;
};
template <typename Acc, column_property P, column_property... Ps>
struct fold_append_unique<Acc, P, Ps...>
    : fold_append_unique<typename list_append_unique<Acc, P>::type, Ps...> {};

template <typename Acc, typename... Lists>
struct property_lists_union {
    using type = Acc;
};
template <typename Acc, column_property... Ps, typename... Lists>
struct property_lists_union<Acc, property_list<Ps...>, Lists...>
    : property_lists_union<typename fold_append_unique<Acc, Ps...>::type,
                           Lists...> {};

}  // namespace detail

template <typename... Lists>
using property_list_union_t =
    typename detail::property_lists_union<property_list<>, Lists...>::type;

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Common properties //////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// last known reduced cost of the column
struct reduced_cost : property<double> {
    static constexpr void on_priced(double & value,
                                    std::floating_point auto rc) noexcept {
        value = static_cast<double>(rc);
    }
};

// number of pricing rounds spent in the current state (pool or master)
struct age : property<std::size_t> {
    static constexpr void on_priced(std::size_t & value,
                                    std::floating_point auto) noexcept {
        ++value;
    }
    static constexpr void on_activated(std::size_t & value) noexcept {
        value = 0;
    }
    static constexpr void on_deactivated(std::size_t & value) noexcept {
        value = 0;
    }
};

// number of times the column entered the master model
// (list it in both states so that it survives transitions)
struct times_activated : property<std::size_t> {
    static constexpr void on_activated(std::size_t & value) noexcept {
        ++value;
    }
};

// the K last reduced costs of the column, most recent first, initialized to
// -inf so that "all above threshold" predicates hold only once K real values
// have been observed
template <std::size_t K>
struct reduced_cost_window {
    using value_type = std::array<double, K>;
    static constexpr value_type initial_value() noexcept {
        value_type values;
        values.fill(-std::numeric_limits<double>::infinity());
        return values;
    }
    static constexpr void on_priced(value_type & values,
                                    std::floating_point auto rc) noexcept {
        std::shift_right(values.begin(), values.end(), 1);
        values.front() = static_cast<double>(rc);
    }
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// States ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <column_property... Ps>
struct in_pool : detail::properties_base<Ps...> {};

template <column_property... Ps>
struct in_master : detail::properties_base<Ps...> {};

namespace detail {
template <template <typename...> class State, typename List>
struct state_from_property_list;
template <template <typename...> class State, column_property... Ps>
struct state_from_property_list<State, property_list<Ps...>> {
    using type = State<Ps...>;
};

template <typename S>
struct strategy_required_properties {
    using type = property_list<>;
};
template <typename S>
    requires requires { typename S::required_properties; }
struct strategy_required_properties<S> {
    using type = typename S::required_properties;
};
}  // namespace detail

// Deduces the minimal state types embedding the properties required by the
// given strategies, e.g.
//     in_pool_state_t<activate_k_most_negative> == in_pool<reduced_cost>
template <typename... Strategies>
using in_pool_state_t = typename detail::state_from_property_list<
    in_pool,
    property_list_union_t<typename detail::strategy_required_properties<
        Strategies>::type...>>::type;

template <typename... Strategies>
using in_master_state_t = typename detail::state_from_property_list<
    in_master,
    property_list_union_t<typename detail::strategy_required_properties<
        Strategies>::type...>>::type;

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Strategies /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// A strategy is any function object callable with (seed, state), (state) or
// (seed) returning bool. It may additionally declare:
//
//     using required_properties = property_list<...>;
//         the properties it reads, so that state types can be deduced
//     std::size_t max_activations() const;   (activation strategies only)
//         budget of columns added per round; requires priority()
//     scalar priority(seed, state) / priority(state) const;
//         ranking used to select within the budget (greater is better)

namespace detail {
template <typename S, typename Seed, typename State>
concept column_predicate =
    requires(const S & s, const Seed & seed, const State & state) {
        { s(seed, state) } -> std::convertible_to<bool>;
    } || requires(const S & s, const State & state) {
        { s(state) } -> std::convertible_to<bool>;
    } || requires(const S & s, const Seed & seed) {
        { s(seed) } -> std::convertible_to<bool>;
    };

template <typename S, typename Seed, typename State>
    requires column_predicate<S, Seed, State>
constexpr bool invoke_column_predicate(const S & s, const Seed & seed,
                                       const State & state) {
    if constexpr(requires {
                     { s(seed, state) } -> std::convertible_to<bool>;
                 })
        return static_cast<bool>(s(seed, state));
    else if constexpr(requires {
                          { s(state) } -> std::convertible_to<bool>;
                      })
        return static_cast<bool>(s(state));
    else
        return static_cast<bool>(s(seed));
}

template <typename S>
concept bounded_activation_strategy = requires(const S & s) {
    { s.max_activations() } -> std::convertible_to<std::size_t>;
};

template <typename S, typename Seed, typename State>
constexpr auto invoke_priority(const S & s, const Seed & seed,
                               const State & state) {
    if constexpr(requires { s.priority(seed, state); })
        return s.priority(seed, state);
    else
        return s.priority(state);
}
}  // namespace detail

// adds every column whose reduced cost improves by more than 'tolerance'
struct activate_negative_reduced_cost {
    double tolerance = 1e-10;
    using required_properties = property_list<reduced_cost>;
    template <column_property... Ps>
    constexpr bool operator()(const in_pool<Ps...> & state) const {
        return state.template get<reduced_cost>() < -tolerance;
    }
};

// adds the (at most) 'count' columns with the most negative reduced costs
struct activate_k_most_negative {
    std::size_t count;
    double tolerance = 1e-10;
    using required_properties = property_list<reduced_cost>;
    template <column_property... Ps>
    constexpr bool operator()(const in_pool<Ps...> & state) const {
        return state.template get<reduced_cost>() < -tolerance;
    }
    constexpr std::size_t max_activations() const noexcept { return count; }
    template <column_property... Ps>
    constexpr double priority(const in_pool<Ps...> & state) const {
        return -state.template get<reduced_cost>();
    }
};

struct evict_never {
    using required_properties = property_list<>;
    template <typename State>
    constexpr bool operator()(const State &) const noexcept {
        return false;
    }
};

// evicts columns whose reduced cost stayed above 'threshold' for their K last
// pricing rounds in the master
template <std::size_t K>
struct evict_window_above {
    double threshold = 0.0;
    using required_properties = property_list<reduced_cost_window<K>>;
    template <column_property... Ps>
    constexpr bool operator()(const in_master<Ps...> & state) const {
        return std::ranges::all_of(state.template get<reduced_cost_window<K>>(),
                                   [&](double rc) { return rc > threshold; });
    }
};

// evicts columns that are currently unattractive and entered the master at
// least 'min_age' pricing rounds ago (protects freshly added columns)
struct evict_unattractive_aged {
    std::size_t min_age;
    double tolerance = 1e-10;
    using required_properties = property_list<age, reduced_cost>;
    template <column_property... Ps>
    constexpr bool operator()(const in_master<Ps...> & state) const {
        return state.template get<age>() >= min_age &&
               state.template get<reduced_cost>() > tolerance;
    }
};

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Column manager ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename ModelVariable, typename ColumnSeed, typename InPoolState = in_pool<>,
          typename InMasterState = in_master<>,
          typename Hash = std::hash<ColumnSeed>,
          typename KeyEqual = std::equal_to<ColumnSeed>>
class column_manager {
public:
    using variable = ModelVariable;
    using scalar = linear_expression_scalar_t<ModelVariable>;
    using column_seed = ColumnSeed;
    using in_pool_state = InPoolState;
    using in_master_state = InMasterState;

    struct emplace_columns_result {
        std::size_t num_inserted = 0;
        std::size_t num_already_in_pool = 0;
        // pricing regenerated columns already in the master : the pricing
        // problem is not using up to date duals or the process is cycling
        std::size_t num_already_in_master = 0;
    };
    struct manage_columns_result {
        std::size_t num_activated = 0;
        std::size_t num_evicted = 0;
    };

private:
    struct master_entry {
        variable var;
        [[no_unique_address]] InMasterState state;
    };
    using column_entry = std::variant<InPoolState, master_entry>;
    using map_value_type = std::pair<const ColumnSeed, column_entry>;

    std::unordered_map<ColumnSeed, column_entry, Hash, KeyEqual> _columns;
    std::size_t _num_master_columns = 0;

    std::vector<variable> _tmp_evicted_variables;
    std::vector<std::pair<scalar, map_value_type *>> _tmp_candidates;

public:
    [[nodiscard]] constexpr column_manager() = default;
    constexpr column_manager(const column_manager &) = default;
    constexpr column_manager(column_manager &&) = default;

    constexpr column_manager & operator=(const column_manager &) = default;
    constexpr column_manager & operator=(column_manager &&) = default;

    void reserve(std::size_t num_columns) { _columns.reserve(num_columns); }
    std::size_t num_columns() const noexcept { return _columns.size(); }
    std::size_t num_master_columns() const noexcept {
        return _num_master_columns;
    }
    std::size_t num_pool_columns() const noexcept {
        return _columns.size() - _num_master_columns;
    }
    bool contains(const ColumnSeed & seed) const {
        return _columns.contains(seed);
    }
    std::optional<variable> master_variable(const ColumnSeed & seed) const {
        auto it = _columns.find(seed);
        if(it == _columns.end() ||
           !std::holds_alternative<master_entry>(it->second))
            return std::nullopt;
        return std::get<master_entry>(it->second).var;
    }

    ////////////////////////////// emplacement //////////////////////////////

    template <typename S>
        requires std::constructible_from<ColumnSeed, S &&>
    bool emplace_column(S && seed) {
        return _columns.try_emplace(std::forward<S>(seed), InPoolState{})
            .second;
    }
    // registers a column already added to the master model, e.g. the initial
    // columns ensuring the feasibility of the restricted master problem
    template <typename S>
        requires std::constructible_from<ColumnSeed, S &&>
    bool emplace_master_column(S && seed, variable v) {
        auto && [it, inserted] = _columns.try_emplace(
            std::forward<S>(seed), master_entry{v, InMasterState{}});
        if(inserted) {
            std::get<master_entry>(it->second).state.notify_activated();
            ++_num_master_columns;
        }
        return inserted;
    }
    template <std::ranges::input_range R>
        requires std::constructible_from<ColumnSeed,
                                         std::ranges::range_reference_t<R>>
    emplace_columns_result emplace_columns(R && seeds) {
        emplace_columns_result result;
        for(auto && seed : seeds) {
            auto && [it, inserted] = _columns.try_emplace(
                std::forward<decltype(seed)>(seed), InPoolState{});
            if(inserted) {
                ++result.num_inserted;
                continue;
            }
            if(std::holds_alternative<master_entry>(it->second))
                ++result.num_already_in_master;
            else
                ++result.num_already_in_pool;
        }
        return result;
    }

    //////////////////////////////// pricing ////////////////////////////////

    template <typename SF>
        requires std::invocable<SF &, const ColumnSeed &> &&
                 std::convertible_to<
                     std::invoke_result_t<SF &, const ColumnSeed &>, scalar>
    void update_columns(SF && seed_reduced_cost_lambda) {
        for(auto & [seed, entry] : _columns) {
            std::visit(detail::overloaded{
                           [&](InPoolState & state) {
                               if constexpr(InPoolState::num_properties() > 0)
                                   state.notify_priced(static_cast<scalar>(
                                       seed_reduced_cost_lambda(seed)));
                           },
                           [&](master_entry & m) {
                               if constexpr(InMasterState::num_properties() > 0)
                                   m.state.notify_priced(static_cast<scalar>(
                                       seed_reduced_cost_lambda(seed)));
                           }},
                       entry);
        }
    }

    template <typename SF, typename VF>
        requires std::invocable<SF &, const ColumnSeed &> &&
                 std::convertible_to<
                     std::invoke_result_t<SF &, const ColumnSeed &>, scalar> &&
                 std::invocable<VF &, const variable &> &&
                 std::convertible_to<
                     std::invoke_result_t<VF &, const variable &>, scalar>
    void update_columns(SF && seed_reduced_cost_lambda,
                        VF && var_reduced_cost_lambda) {
        for(auto & [seed, entry] : _columns) {
            std::visit(detail::overloaded{
                           [&](InPoolState & state) {
                               if constexpr(InPoolState::num_properties() > 0)
                                   state.notify_priced(static_cast<scalar>(
                                       seed_reduced_cost_lambda(seed)));
                           },
                           [&](master_entry & m) {
                               if constexpr(InMasterState::num_properties() > 0)
                                   m.state.notify_priced(static_cast<scalar>(
                                       var_reduced_cost_lambda(m.var)));
                           }},
                       entry);
        }
    }

    /////////////////////////////// management ///////////////////////////////

    // Evicts from the master model the columns matched by the eviction
    // strategy, then adds to the master model the pool columns matched by the
    // activation strategy (within its budget if bounded). The lambdas
    // materialize the model modifications because a column is not only its
    // entries but also an objective coefficient, bounds, a name... e.g.
    //     [&](auto && seed) {
    //         return model.add_column(entries(seed), {.obj_coef = 1.0});
    //     },
    //     [&](auto && variables) { model.remove_variables(variables); }
    template <typename AS, typename ES, typename AF, typename RF>
        requires detail::column_predicate<AS, ColumnSeed, InPoolState> &&
                 detail::column_predicate<ES, ColumnSeed, InMasterState> &&
                 std::invocable<AF &, const ColumnSeed &> &&
                 std::convertible_to<
                     std::invoke_result_t<AF &, const ColumnSeed &>,
                     variable> &&
                 (std::invocable<RF &, const std::vector<variable> &> ||
                  std::invocable<RF &, variable>)
    manage_columns_result manage_columns(AS && activation_strategy,
                                         ES && eviction_strategy,
                                         AF && add_column_lambda,
                                         RF && remove_column_lambda) {
        manage_columns_result result;
        if constexpr(!std::same_as<std::decay_t<ES>, evict_never>) {
            _tmp_evicted_variables.resize(0);
            for(auto & node : _columns) {
                if(auto * m = std::get_if<master_entry>(&node.second);
                   m != nullptr && detail::invoke_column_predicate(
                                       eviction_strategy, node.first, m->state))
                    _evict(node);
            }
            result.num_evicted = _tmp_evicted_variables.size();
            if(!_tmp_evicted_variables.empty()) {
                if constexpr(std::invocable<RF &,
                                            const std::vector<variable> &>)
                    remove_column_lambda(std::as_const(_tmp_evicted_variables));
                else
                    for(const variable & v : _tmp_evicted_variables)
                        remove_column_lambda(v);
            }
        }
        if constexpr(detail::bounded_activation_strategy<std::decay_t<AS>>) {
            _tmp_candidates.resize(0);
            for(auto & node : _columns) {
                if(auto * state = std::get_if<InPoolState>(&node.second);
                   state != nullptr &&
                   detail::invoke_column_predicate(activation_strategy,
                                                   node.first, *state))
                    _tmp_candidates.emplace_back(
                        static_cast<scalar>(detail::invoke_priority(
                            activation_strategy, node.first, *state)),
                        &node);
            }
            const std::size_t budget = std::min<std::size_t>(
                activation_strategy.max_activations(), _tmp_candidates.size());
            std::ranges::nth_element(
                _tmp_candidates,
                _tmp_candidates.begin() + static_cast<std::ptrdiff_t>(budget),
                std::ranges::greater{},
                [](auto && candidate) { return candidate.first; });
            for(auto && [priority, node] :
                _tmp_candidates | std::views::take(budget))
                _activate(*node, add_column_lambda);
            result.num_activated = budget;
        } else {
            for(auto & node : _columns) {
                if(auto * state = std::get_if<InPoolState>(&node.second);
                   state != nullptr &&
                   detail::invoke_column_predicate(activation_strategy,
                                                   node.first, *state)) {
                    _activate(node, add_column_lambda);
                    ++result.num_activated;
                }
            }
        }
        return result;
    }
    template <typename AS, typename AF>
    manage_columns_result manage_columns(AS && activation_strategy,
                                         AF && add_column_lambda) {
        return manage_columns(
            std::forward<AS>(activation_strategy), evict_never{},
            std::forward<AF>(add_column_lambda), [](variable) {});
    }

    // erases from the pool (not the master) the columns matching the given
    // predicate, allowing to bound the memory footprint of the pool
    template <typename F>
        requires detail::column_predicate<F, ColumnSeed, InPoolState>
    std::size_t purge_pool(F && predicate) {
        return std::erase_if(_columns, [&](map_value_type & node) {
            auto * state = std::get_if<InPoolState>(&node.second);
            return state != nullptr && detail::invoke_column_predicate(
                                           predicate, node.first, *state);
        });
    }

    ///////////////////////////////// views /////////////////////////////////

    auto pool_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<InPoolState>(node.second);
               }) |
               std::views::transform(
                   [](const map_value_type & node)
                       -> std::pair<const ColumnSeed &, const InPoolState &> {
                       return {node.first, std::get<InPoolState>(node.second)};
                   });
    }
    auto master_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<master_entry>(node.second);
               }) |
               std::views::transform(
                   [](const map_value_type & node)
                       -> std::tuple<const ColumnSeed &, variable,
                                     const InMasterState &> {
                       auto & m = std::get<master_entry>(node.second);
                       return {node.first, m.var, m.state};
                   });
    }

private:
    template <typename AF>
    void _activate(map_value_type & node, AF & add_column_lambda) {
        master_entry m{add_column_lambda(node.first), InMasterState{}};
        detail::transfer_common_properties(std::get<InPoolState>(node.second),
                                           m.state);
        m.state.notify_activated();
        node.second = std::move(m);
        ++_num_master_columns;
    }
    void _evict(map_value_type & node) {
        master_entry & m = std::get<master_entry>(node.second);
        _tmp_evicted_variables.emplace_back(m.var);
        InPoolState state{};
        detail::transfer_common_properties(m.state, state);
        state.notify_deactivated();
        node.second = std::move(state);
        --_num_master_columns;
    }
};

}  // namespace fhamonic::mippp

#endif  // MIPPP_COLUMN_GENERATION_HPP
