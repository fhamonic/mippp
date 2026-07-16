#ifndef MIPPP_COLUMN_MANAGER_HPP
#define MIPPP_COLUMN_MANAGER_HPP

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <optional>
#include <ranges>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "mippp/model_concepts.hpp"
#include "mippp/utility/column_generation.hpp"
#include "mippp/utility/unordered_dense_map.hpp"

namespace mippp {

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// Column manager ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

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
    // the entry types presented to the strategies
    using in_pool_entry = std::pair<const ColumnSeed &, const in_pool_state &>;
    using in_master_entry =
        std::pair<const ColumnSeed &, const in_master_state &>;

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
    using column_map =
        unordered_dense_map<ColumnSeed, column_entry, Hash, KeyEqual>;
    using map_value_type = column_map::value_type;

    struct in_pool_entry_proj {
        in_pool_entry operator()(map_value_type * node) const {
            return {node->first, std::get<in_pool_state>(node->second)};
        }
        in_pool_entry operator()(const map_value_type & node) const {
            return {node.first, std::get<in_pool_state>(node.second)};
        }
    };
    struct in_master_entry_proj {
        in_master_entry operator()(map_value_type * node) const {
            return {node->first, std::get<in_master_state>(node->second)};
        }
        in_master_entry operator()(const map_value_type & node) const {
            return {node.first, std::get<in_master_state>(node.second)};
        }
    };

    column_map _columns;
    std::size_t _num_master_columns = 0;
    // scratch buffer reused across the pricing rounds; the node pointers stay
    // valid while selecting/transitioning because manage_columns never
    // inserts nor erases
    std::vector<map_value_type *> _tmp_candidates;

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
            std::get<in_master_state>(it->second).notify(activated{});
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

    // The update methods broadcast events (see column_generation.hpp) to the
    // columns' properties. The caller builds the event for each column, so an
    // update may carry a reduced cost, but also a basis status, a primal
    // value, or any user-defined payload; properties that do not react to the
    // broadcast event type are skipped at compile time.

    // broadcasts to every pool column the event produced by make_event(seed),
    // typically the pricing oracle's reduced cost, e.g.
    //     mgr.update_pool_columns(
    //         [&](const auto & seed) { return priced{reduced_cost(seed)}; });
    template <typename F>
        requires std::invocable<F &, const ColumnSeed &>
    void update_pool_columns(F && make_event) {
        for(auto & [seed, entry] : _columns)
            if(auto * state = std::get_if<in_pool_state>(&entry))
                state->notify(make_event(seed));
    }

    // broadcasts to every master column the event produced by
    // make_event(seed, var), typically the restricted master solution, e.g.
    //     auto solution = model.get_solution();
    //     auto reduced_costs = model.get_reduced_costs();
    //     auto basis = model.get_basis();
    //     mgr.update_master_columns([&](const auto &, const auto & var) {
    //         return master_refreshed<model_variable_basis_status_t<M>>{
    //             reduced_costs[var], solution[var], basis.get_status(var)};
    //     });
    template <typename F>
        requires std::invocable<F &, const ColumnSeed &, const variable &>
    void update_master_columns(F && make_event) {
        for(auto & [seed, entry] : _columns)
            if(auto * state = std::get_if<in_master_state>(&entry))
                state->notify(make_event(seed, state->var));
    }

    // broadcasts to every column the event produced by make_event(seed), e.g.
    // when reduced costs are recomputed from the duals for pool and master
    // columns alike
    template <typename F>
        requires std::invocable<F &, const ColumnSeed &>
    void update_columns(F && make_event) {
        for(auto & [seed, entry] : _columns)
            std::visit([&](auto & state) { state.notify(make_event(seed)); },
                       entry);
    }

    // broadcasts pool_event(seed) to pool columns and
    // master_event(seed, var) to master columns in a single pass
    template <typename PF, typename MF>
        requires std::invocable<PF &, const ColumnSeed &> &&
                 std::invocable<MF &, const ColumnSeed &, const variable &>
    void update_columns(PF && pool_event, MF && master_event) {
        for(auto & [seed, entry] : _columns) {
            if(auto * state = std::get_if<in_pool_state>(&entry))
                state->notify(pool_event(seed));
            else {
                auto & ms = std::get<in_master_state>(entry);
                ms.notify(master_event(seed, ms.var));
            }
        }
    }

    /////////////////////////////// management ///////////////////////////////

    // Evicts from the master model the columns selected by the eviction
    // strategy, then adds to the master model the pool columns selected by
    // the activation strategy. The lambdas materialize the model
    // modifications because a column is not only its entries but also an
    // objective coefficient, bounds, a name... e.g.
    //     [&](auto & model, auto && seed) {
    //         return model.add_column(entries(seed), {.obj_coef = 1.0});
    //     },
    //     [&](auto & model, auto && evicted_entries) {
    //         model.remove_variables(std::views::transform(
    //             evicted_entries, [](auto && e) { return e.second.var; }));
    //     }
    template <typename AS, typename ES, typename AF, typename RF>
        requires requires(AS & as, ES & es, AF & add, Model & m,
                          const in_pool_entry & pe, const in_master_entry & me,
                          std::vector<map_value_type *> & candidates,
                          const ColumnSeed & seed) {
            { as.predicate(pe) } -> std::convertible_to<bool>;
            { es.predicate(me) } -> std::convertible_to<bool>;
            {
                as.select(candidates, in_pool_entry_proj{})
            } -> std::ranges::input_range;
            {
                es.select(candidates, in_master_entry_proj{})
            } -> std::ranges::input_range;
            { add(m, seed) } -> std::convertible_to<variable>;
        }
    manage_columns_result manage_columns(Model & model,
                                         AS && activation_strategy,
                                         ES && eviction_strategy,
                                         AF && add_column_lambda,
                                         RF && remove_columns_lambda) {
        manage_columns_result result;
        // the candidates are materialized as node handles before any
        // selection or transition : the strategies may thus copy, reorder or
        // iterate them at will and the transitions below mutate the actual
        // map nodes instead of going through live filter views
        _tmp_candidates.resize(0);
        for(auto & node : _columns) {
            if(auto * state = std::get_if<in_master_state>(&node.second);
               state != nullptr &&
               eviction_strategy.predicate(in_master_entry{node.first, *state}))
                _tmp_candidates.emplace_back(&node);
        }
        auto && nodes_to_evict =
            eviction_strategy.select(_tmp_candidates, in_master_entry_proj{});
        remove_columns_lambda(
            model,
            std::views::transform(nodes_to_evict, in_master_entry_proj{}));
        for(map_value_type * node : nodes_to_evict) {
            in_pool_state new_state{};
            detail::transfer_common_properties(
                std::get<in_master_state>(node->second), new_state);
            new_state.notify(deactivated{});
            node->second = std::move(new_state);
            ++result.num_evicted;
        }
        _num_master_columns -= result.num_evicted;

        _tmp_candidates.resize(0);
        for(auto & node : _columns) {
            if(auto * state = std::get_if<in_pool_state>(&node.second);
               state != nullptr &&
               activation_strategy.predicate(in_pool_entry{node.first, *state}))
                _tmp_candidates.emplace_back(&node);
        }
        auto && nodes_to_activate =
            activation_strategy.select(_tmp_candidates, in_pool_entry_proj{});
        for(map_value_type * node : nodes_to_activate) {
            in_master_state new_state{add_column_lambda(model, node->first)};
            detail::transfer_common_properties(
                std::get<in_pool_state>(node->second), new_state);
            new_state.notify(activated{});
            node->second = std::move(new_state);
            ++result.num_activated;
        }
        _num_master_columns += result.num_activated;
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
                              [](Model & m, auto && evicted_entries) {
                                  m.remove_variables(std::views::transform(
                                      evicted_entries, [](const auto & e) {
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
        requires detail::column_predicate<F, in_pool_entry>
    std::size_t purge_pool(F && predicate) {
        return std::erase_if(_columns, [&](map_value_type & node) {
            auto * state = std::get_if<in_pool_state>(&node.second);
            return state != nullptr &&
                   predicate(in_pool_entry{node.first, *state});
        });
    }

    ///////////////////////////////// views /////////////////////////////////

    auto pool_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<in_pool_state>(node.second);
               }) |
               std::views::transform(in_pool_entry_proj{});
    }
    auto master_columns() const noexcept {
        return _columns | std::views::filter([](const map_value_type & node) {
                   return std::holds_alternative<in_master_state>(node.second);
               }) |
               std::views::transform(in_master_entry_proj{});
    }
};

}  // namespace mippp

#endif  // MIPPP_COLUMN_MANAGER_HPP
