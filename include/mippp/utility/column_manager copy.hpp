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
#include "mippp/model_concepts.hpp"

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
        values.fill(0.0);
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

template <typename Variable, column_property... Ps>
struct in_master : detail::properties_base<Ps...> {
    Variable var;
    template <typename V, typename... T>
    in_master(V && v, T &&... args)
        : detail::properties_base<Ps...>(std::forward<T>(args)...)
        , var(std::forward<V>(v)) {}
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

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Strategies
////////////////////////////////////
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

template <typename S, typename Seed, typename State>
concept column_predicate =
    requires(const S & s, const std::pair<Seed, State> & entry) {
        { s(entry) } -> std::convertible_to<bool>;
    };

template <typename S, typename Seed, typename State>
concept column_strategy =
    requires(const S & s, const std::pair<Seed, State> & entry) {
        { s.predicate(entry) } -> std::convertible_to<bool>;
        {
            s.select(std::views::empty<std::pair<Seed, State>>)
        } -> std::ranges::range;
    };

}  // namespace detail

////////////////////////////////
////////////////////////////////

template <typename P>
struct below {
    using required_properties = property_list<P>;
    P::value_type rhs;
    constexpr bool operator()(const auto & entry) const {
        const auto state = std::get<1>(entry);
        return state.template get<P>() < rhs;
    }
};

template <typename P>
struct above {
    using required_properties = property_list<P>;
    P::value_type rhs;
    constexpr bool operator()(const auto & entry) const {
        const auto state = std::get<1>(entry);
        return state.template get<P>() > rhs;
    }
};

template <typename P>
struct negative : below<P> {
    negative(typename P::value_type tolerance = {1e-10})
        : below<P>(-tolerance) {};
};

template <typename P>
struct positive : above<P> {
    positive(typename P::value_type tolerance = {1e-10})
        : above<P>(tolerance) {};
};

template <typename... Pr>
struct conjunction {
    using required_properties =
        property_list_union_t<typename Pr::required_properties...>;
    std::tuple<Pr...> predicates;
    template <typename... T>
    conjunction(T &&... args) : predicates(std::forward<T>(args)...) {};
    constexpr bool operator()(const auto & entry) const {
        return std::apply(
            [&](const Pr &... preds) { return (preds(entry) && ...); },
            predicates);
    }
};

template <typename... Pr>
struct disjunction {
    using required_properties =
        property_list_union_t<typename Pr::required_properties...>;
    std::tuple<Pr...> predicates;
    template <typename... T>
    disjunction(T &&... args) : predicates(std::forward<T>(args)...) {};
    constexpr bool operator()(const auto & entry) const {
        return std::apply(
            [&](const Pr &... preds) { return (preds(entry) || ...); },
            predicates);
    }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct none {
    using required_properties = property_list<>;
    struct {
        constexpr bool operator()(const auto &) { return false; }
    } predicate;
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj proj = {}) const {
        return std::views::empty<std::ranges::range_value_t<R>>;
    }
};
template <typename Pred = conjunction<>>
struct all {
    using required_properties = typename Pred::required_properties;
    Pred predicate;
    template <typename... T>
    all(T &&... args) : predicate(std::forward<T>(args)...) {};
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj proj = {}) const {
        return std::views::all(candidates);
    }
};
template <typename Pred>
struct at_most_k : all<Pred> {
    std::size_t count;
    template <typename... T>
    at_most_k(const std::size_t & count_, T &&... args)
        : all<Pred>(std::forward<T>(args)...), count(count_) {};
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj proj = {}) const {
        return std::views::take(candidates, count);
    }
};
template <typename Pred, typename Gen>
struct at_most_k_random : at_most_k<Pred> {
    using at_most_k<Pred>::count;
    Gen gen;
    template <typename G, typename... T>
    at_most_k_random(const std::size_t & count_, G && gen_, T &&... args)
        : at_most_k<Pred>(count_, std::forward<T>(args)...)
        , gen(std::forward<G>(gen_)) {};
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj proj = {}) const {
        std::vector<std::ranges::range_value_t<R>> selected;
        selected.reserve(count);
        std::ranges::sample(candidates, std::back_inserter(selected), count,
                            gen);
        return selected;
    }
};
template <typename Pred, typename Comp>
struct at_most_k_best : at_most_k<Pred> {
    using at_most_k<Pred>::count;
    [[no_unique_address]] Comp cmp;
    template <typename... T>
    at_most_k_best(const std::size_t & count_, Comp && cmp_, T &&... args)
        : at_most_k<Pred>(count_, std::forward<T>(args)...)
        , cmp(std::forward<Comp>(cmp_)) {};
    template <std::ranges::range R, typename Proj = std::identity>
    constexpr auto select(R && candidates, Proj proj = {}) const {
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

using evict_never = none;

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

using evict_unattractive_aged =
    all<conjunction<above<age>, above<reduced_cost>>>;

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Column manager ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace detail {
template <typename Seed, class State>
inline constexpr auto proj_state = [](auto & e) -> std::pair<Seed &, State &> {
    return {e.first, std::get<State>(e.second)};
};
}  // namespace detail

template <typename Model, typename ColumnSeed,
          typename InPoolProperties = property_list<>,
          typename InMasterProperties = property_list<>,
          typename Hash = std::hash<ColumnSeed>,
          typename KeyEqual = std::equal_to<ColumnSeed>>
class column_manager {
public:
    using variable = typename Model::variable;
    using scalar = typename Model::scalar;
    using column_seed = ColumnSeed;
    using in_pool_state = typename detail::in_pool_state_from_property_list<
        InPoolProperties>::type;
    using in_master_state = typename detail::in_master_state_from_property_list<
        variable, InMasterProperties>::type;

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
    using column_entry = std::variant<in_pool_state, in_master_state>;
    using map_value_type = std::pair<const ColumnSeed, column_entry>;

    std::unordered_map<ColumnSeed, column_entry, Hash, KeyEqual> _columns;
    std::size_t _num_master_columns = 0;

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
           !std::holds_alternative<in_master_state>(it->second))
            return std::nullopt;
        return std::get<in_master_state>(it->second).var;
    }

    ////////////////////////////// emplacement //////////////////////////////

    template <typename S>
        requires std::constructible_from<ColumnSeed, S &&>
    bool emplace_column(S && seed) {
        return _columns.try_emplace(std::forward<S>(seed), in_pool_state{})
            .second;
    }
    // registers a column already added to the master model, e.g. the initial
    // columns ensuring the feasibility of the restricted master problem
    template <typename S>
        requires std::constructible_from<ColumnSeed, S &&>
    bool emplace_master_column(S && seed, variable v) {
        auto && [it, inserted] =
            _columns.try_emplace(std::forward<S>(seed), in_master_state{v});
        if(inserted) {
            std::get<in_master_state>(it->second).notify_activated();
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
                std::forward<decltype(seed)>(seed), in_pool_state{});
            if(inserted) {
                ++result.num_inserted;
                continue;
            }
            if(std::holds_alternative<in_master_state>(it->second))
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
            std::visit(
                detail::overloaded{
                    [&](in_pool_state & state) {
                        if constexpr(in_pool_state::num_properties() > 0)
                            state.notify_priced(static_cast<scalar>(
                                seed_reduced_cost_lambda(seed)));
                    },
                    [&](in_master_state & state) {
                        if constexpr(in_master_state::num_properties() > 0)
                            state.notify_priced(static_cast<scalar>(
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
            std::visit(
                detail::overloaded{
                    [&](in_pool_state & state) {
                        if constexpr(in_pool_state::num_properties() > 0)
                            state.notify_priced(static_cast<scalar>(
                                seed_reduced_cost_lambda(seed)));
                    },
                    [&](in_master_state & state) {
                        if constexpr(in_master_state::num_properties() > 0)
                            state.notify_priced(static_cast<scalar>(
                                var_reduced_cost_lambda(state.var)));
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
    // requires detail::column_predicate<AS, ColumnSeed, in_pool_state> &&
    //          detail::column_predicate<ES, ColumnSeed, in_master_state> &&
    //          std::invocable<AF &, const ColumnSeed &> &&
    //          std::convertible_to<
    //              std::invoke_result_t<AF &, const ColumnSeed &>,
    //              variable> &&
    //          std::invocable<RF &, const std::vector<variable> &>
    manage_columns_result manage_columns(Model & model,
                                         AS && activation_strategy,
                                         ES && eviction_strategy,
                                         AF && add_column_lambda,
                                         RF && remove_columns_lambda) {
        manage_columns_result result;

        auto && eviction_candidates =
            std::views::filter(_columns, [&eviction_strategy](const auto & e) {
                auto state_ptr = std::get_if<in_master_state>(&e.second);
                return state_ptr != nullptr &&
                       eviction_strategy.predicate(
                           std::make_pair(e.first, *state_ptr));
            });
        auto && eviction_nodes = eviction_strategy.select(
            eviction_candidates,
            detail::proj_state<ColumnSeed, in_master_state>);
        remove_columns_lambda(
            model, std::views::transform(
                       eviction_nodes,
                       detail::proj_state<ColumnSeed, in_master_state>));
        for(auto & node : eviction_nodes) {
            in_pool_state new_state{};
            detail::transfer_common_properties(
                std::get<in_master_state>(node.second), new_state);
            new_state.notify_deactivated();
            node.second = std::move(new_state);
            ++result.num_evicted;
        }
        auto && activation_candidates = std::views::filter(
            _columns, [&activation_strategy](const auto & e) {
                auto state_ptr = std::get_if<in_pool_state>(&e.second);
                return state_ptr != nullptr &&
                       activation_strategy.predicate(
                           std::make_pair(e.first, *state_ptr));
            });
        auto && activation_nodes = activation_strategy.select(
            activation_candidates,
            detail::proj_state<ColumnSeed, in_pool_state>);
        for(auto & node : activation_nodes) {
            in_master_state state{add_column_lambda(model, node.first)};
            detail::transfer_common_properties(
                std::get<in_pool_state>(node.second), state);
            state.notify_activated();
            node.second = std::move(state);
            ++result.num_activated;
        }
        _num_master_columns += result.num_activated - result.num_evicted;
        return result;
    }
    template <typename AS, typename ES, typename AF>
        requires has_remove_variable<Model>
    manage_columns_result manage_columns(Model & model,
                                         AS && activation_strategy,
                                         ES && eviction_strategy,
                                         AF && add_column_lambda) {
        return manage_columns(model, std::forward<AS>(activation_strategy),
                              std::forward<ES>(eviction_strategy),
                              std::forward<AF>(add_column_lambda),
                              [](Model & m, auto && selected_entries) {
                                  m.remove_variables(std::views::transform(
                                      selected_entries, [](const auto & e) {
                                          return e.second.var;
                                      }));
                              });
    }
    template <typename AS, typename AF>
    manage_columns_result manage_columns(Model & model,
                                         AS && activation_strategy,
                                         AF && add_column_lambda) {
        return manage_columns(model, std::forward<AS>(activation_strategy),
                              none{}, std::forward<AF>(add_column_lambda),
                              [](Model &, auto &&) {});
    }

    // erases from the pool (not the master) the columns matching the given
    // predicate, allowing to bound the memory footprint of the pool
    template <typename F>
        requires detail::column_predicate<F, ColumnSeed, in_pool_state>
    std::size_t purge_pool(F && predicate) {
        return std::erase_if(_columns, [&](map_value_type & node) {
            auto * state = std::get_if<in_pool_state>(&node.second);
            return state != nullptr && predicate(node.first, *state);
        });
    }

    ///////////////////////////////// views /////////////////////////////////

    auto pool_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<in_pool_state>(node.second);
               }) |
               std::views::transform(
                   [](const map_value_type & node)
                       -> std::pair<const ColumnSeed &, const in_pool_state &> {
                       return {node.first,
                               std::get<in_pool_state>(node.second)};
                   });
    }
    auto master_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<in_master_state>(node.second);
               }) |
               std::views::transform(
                   [](const map_value_type & node)
                       -> std::tuple<const ColumnSeed &,
                                     const in_master_state &> {
                       return {node.first,
                               std::get<in_master_state>(node.second)};
                   });
    }
};

}  // namespace fhamonic::mippp

#endif  // MIPPP_COLUMN_GENERATION_HPP
