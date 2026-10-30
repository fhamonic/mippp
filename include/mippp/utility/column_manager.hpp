#pragma once

#include <concepts>
#include <cstddef>
#include <functional>
#include <optional>
#include <ranges>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/container/unordered_dense_map.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/utility/column_generation.hpp"

namespace mippp {

namespace colgen {

// the default remove callback of column_manager::evict_columns
struct remove_variables {
    template <typename Model, std::ranges::input_range R>
        requires has_remove_variable<Model>
    void operator()(Model & model, R && evicted_entries) const {
        model.remove_variables(std::views::transform(
            evicted_entries, [](const auto & e) { return e.state.var; }));
    }
};

namespace detail {
template <typename S, typename Candidates, typename Proj>
concept column_selector = requires(const S & s, Candidates & c, Proj proj) {
    { s.select(c, proj) } -> std::ranges::forward_range;
};
template <typename E, typename Seed, typename Data>
concept seed_data_pair =
    requires(E && e) {
        std::get<0>(std::forward<E>(e));
        std::get<1>(std::forward<E>(e));
    } &&
    std::constructible_from<Seed, decltype(std::get<0>(std::declval<E>()))> &&
    std::constructible_from<Data, decltype(std::get<1>(std::declval<E>()))>;
}  // namespace detail

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Column manager ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename Model, typename ColumnSeed,
          typename InPoolProperties = property_list<>,
          typename InMasterProperties = property_list<>,
          typename Data = no_data, typename Hash = seed_hash<ColumnSeed>,
          typename KeyEqual = std::equal_to<ColumnSeed>>
class column_manager {
public:
    using variable = model_variable_t<Model>;
    using scalar = model_scalar_t<Model>;
    using column_seed = ColumnSeed;
    using column_data = Data;
    using in_pool_state = typename detail::in_pool_state_from_property_list<
        InPoolProperties>::type;
    using in_master_state = typename detail::in_master_state_from_property_list<
        variable, InMasterProperties>::type;
    using in_pool_entry = column_entry<ColumnSeed, in_pool_state, Data>;
    using in_master_entry = column_entry<ColumnSeed, in_master_state, Data>;

    struct emplace_pool_columns_result {
        std::size_t num_inserted = 0;
        std::size_t num_already_in_pool = 0;
        // pricing regenerated columns already in the master : the pricing
        // problem is not using up to date duals or the process is cycling
        std::size_t num_already_in_master = 0;
    };

private:
    struct column {
        [[no_unique_address]] Data data;
        std::variant<in_pool_state, in_master_state> state;

        template <typename... DataArgs>
        explicit column(in_pool_state s, DataArgs &&... data_args)
            : data(std::forward<DataArgs>(data_args)...), state(std::move(s)) {}
        template <typename... DataArgs>
        explicit column(in_master_state s, DataArgs &&... data_args)
            : data(std::forward<DataArgs>(data_args)...), state(std::move(s)) {}
    };
    using column_map = unordered_dense_map<ColumnSeed, column, Hash, KeyEqual>;
    using map_value_type = column_map::value_type;

    template <typename State>
    static column_entry<ColumnSeed, State, Data> make_entry(
        const ColumnSeed & seed, const Data & data,
        const State & state) noexcept {
        if constexpr(std::same_as<Data, no_data>)
            return {seed, state};
        else
            return {seed, data, state};
    }
    template <typename State>
    static column_entry<ColumnSeed, State, Data> make_entry(
        const map_value_type & node, const State & state) noexcept {
        return make_entry(node.first, node.second.data, state);
    }
    struct in_pool_entry_proj {
        in_pool_entry operator()(const map_value_type * node) const noexcept {
            return make_entry(*node,
                              std::get<in_pool_state>(node->second.state));
        }
    };
    struct in_master_entry_proj {
        in_master_entry operator()(const map_value_type * node) const noexcept {
            return make_entry(*node,
                              std::get<in_master_state>(node->second.state));
        }
    };
    using candidates = std::vector<map_value_type *>;

    column_map _columns;
    std::size_t _num_master_columns = 0;
    // scratch buffer reused across the rounds; the node pointers stay valid
    // while selecting and transitioning because neither activate_columns nor
    // evict_columns inserts or erases
    candidates _tmp_candidates;

    template <typename AF>
    void _activate(Model & model, map_value_type & node, AF & add_column) {
        auto & pool_state = std::get<in_pool_state>(node.second.state);
        in_master_state new_state{static_cast<variable>(
            add_column(model, make_entry(node, pool_state)))};
        detail::transfer_common_properties(pool_state, new_state);
        new_state.notify(activated{});
        node.second.state = std::move(new_state);
        ++_num_master_columns;
    }

public:
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
        if(it == _columns.end()) return std::nullopt;
        auto * state = std::get_if<in_master_state>(&it->second.state);
        if(state == nullptr) return std::nullopt;
        return state->var;
    }

    ////////////////////////////// emplacement //////////////////////////////

    // An emplacement constructs the column's Data from data_args only when
    // it inserts: a seed already known keeps the data it was first given.

    template <typename S, typename... DataArgs>
        requires std::constructible_from<ColumnSeed, S &&> &&
                 std::constructible_from<Data, DataArgs &&...>
    bool emplace_pool_column(S && seed, DataArgs &&... data_args) {
        return _columns
            .try_emplace(std::forward<S>(seed), in_pool_state{},
                         std::forward<DataArgs>(data_args)...)
            .second;
    }
    // registers a column whose variable is already in the master model, e.g.
    // an initial column created before the rows it has entries in
    template <typename S, typename... DataArgs>
        requires std::constructible_from<ColumnSeed, S &&> &&
                 std::constructible_from<Data, DataArgs &&...>
    bool emplace_master_column(S && seed, variable v,
                               DataArgs &&... data_args) {
        auto && [it, inserted] =
            _columns.try_emplace(std::forward<S>(seed), in_master_state{v},
                                 std::forward<DataArgs>(data_args)...);
        if(inserted) {
            std::get<in_master_state>(it->second.state).notify(activated{});
            ++_num_master_columns;
        }
        return inserted;
    }
    // adds a column to the master model through the same add callback as
    // activate_columns, which is not called if the seed is already known;
    // nothing is registered if add_column throws
    template <typename S, typename AF, typename... DataArgs>
        requires std::constructible_from<ColumnSeed, S &&> &&
                 std::constructible_from<Data, DataArgs &&...> &&
                 std::invocable<AF &, Model &, const in_pool_entry &>
    bool emplace_master_column(Model & model, S && seed, AF && add_column,
                               DataArgs &&... data_args) {
        ColumnSeed key(std::forward<S>(seed));
        if(_columns.contains(key)) return false;
        column pending(in_pool_state{}, std::forward<DataArgs>(data_args)...);
        const auto & pool_state = std::get<in_pool_state>(pending.state);
        in_master_state master_state{static_cast<variable>(
            add_column(model, make_entry(key, pending.data, pool_state)))};
        detail::transfer_common_properties(pool_state, master_state);
        master_state.notify(activated{});
        _columns.try_emplace(std::move(key), std::move(master_state),
                             std::move(pending.data));
        ++_num_master_columns;
        return true;
    }
    // with a Data, the elements are (seed, data) pairs or tuples
    template <std::ranges::input_range R>
        requires(std::same_as<Data, no_data> &&
                 std::constructible_from<ColumnSeed,
                                         std::ranges::range_reference_t<R>>) ||
                (!std::same_as<Data, no_data> &&
                 detail::seed_data_pair<std::ranges::range_reference_t<R>,
                                        ColumnSeed, Data>)
    emplace_pool_columns_result emplace_pool_columns(R && seeds) {
        emplace_pool_columns_result result;
        for(auto && element : seeds) {
            using E = decltype(element);
            auto && [it, inserted] = [&] {
                if constexpr(std::same_as<Data, no_data>)
                    return _columns.try_emplace(std::forward<E>(element),
                                                in_pool_state{});
                else
                    return _columns.try_emplace(
                        std::get<0>(std::forward<E>(element)), in_pool_state{},
                        std::get<1>(std::forward<E>(element)));
            }();
            if(inserted) {
                ++result.num_inserted;
                continue;
            }
            if(std::holds_alternative<in_master_state>(it->second.state))
                ++result.num_already_in_master;
            else
                ++result.num_already_in_pool;
        }
        return result;
    }

    //////////////////////////////// pricing ////////////////////////////////

    // The update methods broadcast to the columns' properties the event that
    // make_event builds from each column's entry (see column_generation.hpp):
    // a reduced cost, but also a basis status, a primal value, or any
    // user-defined payload; properties that do not react to the event type
    // are skipped at compile time.

    // e.g. mgr.update_pool_columns([&](const auto & e) {
    //          return priced{reduced_cost(e.seed)};
    //      });
    template <typename F>
        requires std::invocable<F &, const in_pool_entry &>
    void update_pool_columns(F && make_event) {
        for(auto & node : _columns)
            if(auto * state = std::get_if<in_pool_state>(&node.second.state))
                state->notify(make_event(make_entry(node, *state)));
    }

    // e.g. after taking reduced_costs, solution and basis from the model:
    //      mgr.update_master_columns([&](const auto & e) {
    //          const auto & var = e.state.var;
    //          return master_refreshed<model_variable_basis_status_t<M>>{
    //              reduced_costs[var], solution[var], basis.get_status(var)};
    //      });
    template <typename F>
        requires std::invocable<F &, const in_master_entry &>
    void update_master_columns(F && make_event) {
        for(auto & node : _columns)
            if(auto * state = std::get_if<in_master_state>(&node.second.state))
                state->notify(make_event(make_entry(node, *state)));
    }

    // a single make_event for pool and master columns alike, e.g. when every
    // reduced cost is recomputed from the duals
    template <typename F>
        requires std::invocable<F &, const in_pool_entry &> &&
                 std::invocable<F &, const in_master_entry &>
    void update_columns(F && make_event) {
        for(auto & node : _columns)
            std::visit(
                [&](auto & state) {
                    state.notify(make_event(make_entry(node, state)));
                },
                node.second.state);
    }

    // pool_event for pool columns and master_event for master columns, in a
    // single pass
    template <typename PF, typename MF>
        requires std::invocable<PF &, const in_pool_entry &> &&
                 std::invocable<MF &, const in_master_entry &>
    void update_columns(PF && pool_event, MF && master_event) {
        for(auto & node : _columns) {
            if(auto * state = std::get_if<in_pool_state>(&node.second.state))
                state->notify(pool_event(make_entry(node, *state)));
            else {
                auto & ms = std::get<in_master_state>(node.second.state);
                ms.notify(master_event(make_entry(node, ms)));
            }
        }
    }

    /////////////////////////// activation, eviction ///////////////////////////

    // A round reads the master's solution, duals and basis before calling
    // either: both modify the master. Evicting before activating modifies the
    // master after its last solve even when the round finds nothing to
    // activate, and the solution read at convergence is then stale:
    //     if(!mgr.activate_columns(model, improving, add)) break;
    //     mgr.evict_columns(model, unattractive);

    // Adds to the master the pool columns matching the predicate, narrowed
    // by the selector; add_column(model, entry) creates the column, since a
    // column is not only its entries but also an objective coefficient,
    // bounds, a name..., and returns its variable. Returns the number of
    // columns activated.
    template <typename Pred, typename AF, typename Sel = all_candidates>
    std::size_t activate_columns(Model & model, Pred && predicate,
                                 AF && add_column, Sel && selector = Sel{}) {
        static_assert(
            detail::column_predicate<Pred, in_pool_entry>,
            "MIP++: activate_columns: the predicate must be callable with "
            "a pool entry and return a bool, e.g. negative<reduced_cost>{}.");
        static_assert(
            !detail::column_selector<AF, candidates, in_pool_entry_proj>,
            "MIP++: activate_columns(model, predicate, add, selector): the "
            "add callback comes third and the selector last.");
        static_assert(
            std::is_invocable_r_v<variable, AF &, Model &,
                                  const in_pool_entry &>,
            "MIP++: activate_columns: the add callback must create the "
            "column from (model, entry) and return its variable.");
        static_assert(
            detail::column_selector<Sel, candidates, in_pool_entry_proj>,
            "MIP++: activate_columns: the last argument must be a selector, "
            "e.g. at_most_k{n}.");
        _tmp_candidates.resize(0);
        for(auto & node : _columns) {
            if(auto * state = std::get_if<in_pool_state>(&node.second.state);
               state != nullptr && predicate(make_entry(node, *state)))
                _tmp_candidates.emplace_back(&node);
        }
        auto && selected =
            selector.select(_tmp_candidates, in_pool_entry_proj{});
        std::size_t num_activated = 0;
        for(map_value_type * node : selected) {
            _activate(model, *node, add_column);
            ++num_activated;
        }
        return num_activated;
    }

    // Removes from the master the columns matching the predicate, narrowed
    // by the selector, and puts them back in the pool; remove_columns(model,
    // entries) is not called if nothing is selected. Returns the number of
    // columns evicted.
    template <typename Pred, typename RF = remove_variables,
              typename Sel = all_candidates>
    std::size_t evict_columns(Model & model, Pred && predicate,
                              RF && remove_columns = RF{},
                              Sel && selector = Sel{}) {
        static_assert(
            detail::column_predicate<Pred, in_master_entry>,
            "MIP++: evict_columns: the predicate must be callable with a "
            "master entry and return a bool, e.g. positive<reduced_cost>{}.");
        static_assert(
            !detail::column_selector<RF, candidates, in_master_entry_proj>,
            "MIP++: evict_columns(model, predicate, remove, selector): the "
            "remove callback comes third and the selector last; pass "
            "remove_variables{} as the remove callback.");
        static_assert(
            !std::same_as<std::remove_cvref_t<RF>, remove_variables> ||
                has_remove_variable<Model>,
            "MIP++: evict_columns: this model has no remove_variables(); "
            "pass a remove callback.");
        static_assert(
            detail::column_selector<Sel, candidates, in_master_entry_proj>,
            "MIP++: evict_columns: the last argument must be a selector, "
            "e.g. at_most_k{n}.");
        _tmp_candidates.resize(0);
        for(auto & node : _columns) {
            if(auto * state = std::get_if<in_master_state>(&node.second.state);
               state != nullptr && predicate(make_entry(node, *state)))
                _tmp_candidates.emplace_back(&node);
        }
        auto && selected =
            selector.select(_tmp_candidates, in_master_entry_proj{});
        if(std::ranges::begin(selected) == std::ranges::end(selected)) return 0;
        remove_columns(model,
                       std::views::transform(selected, in_master_entry_proj{}));
        std::size_t num_evicted = 0;
        for(map_value_type * node : selected) {
            auto & master_state = std::get<in_master_state>(node->second.state);
            in_pool_state new_state{};
            detail::transfer_common_properties(master_state, new_state);
            new_state.notify(deactivated{});
            node->second.state = std::move(new_state);
            ++num_evicted;
        }
        _num_master_columns -= num_evicted;
        return num_evicted;
    }

    // erases from the pool (not the master) the columns matching the given
    // predicate, allowing to bound the memory footprint of the pool; the
    // remaining columns keep their order
    template <typename F>
        requires detail::column_predicate<F, in_pool_entry>
    std::size_t purge_pool(F && predicate) {
        return stable_erase_if(_columns, [&](map_value_type & node) {
            auto * state = std::get_if<in_pool_state>(&node.second.state);
            return state != nullptr && predicate(make_entry(node, *state));
        });
    }

    ///////////////////////////////// views /////////////////////////////////

    // in the order the seeds were first emplaced
    auto pool_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<in_pool_state>(
                       node.second.state);
               }) |
               std::views::transform([](const map_value_type & node) {
                   return make_entry(
                       node, std::get<in_pool_state>(node.second.state));
               });
    }
    auto master_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<in_master_state>(
                       node.second.state);
               }) |
               std::views::transform([](const map_value_type & node) {
                   return make_entry(
                       node, std::get<in_master_state>(node.second.state));
               });
    }
};

}  // namespace colgen

}  // namespace mippp
