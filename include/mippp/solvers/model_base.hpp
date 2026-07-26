#pragma once

#include <optional>
#include <ranges>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

namespace mippp {

template <std::integral _Index, std::floating_point _Scalar>
class model_base {
public:
    using variable = model_variable<_Index, _Scalar>;
    using constraint = model_constraint<_Index>;

    struct variable_params {
        _Scalar obj_coef = _Scalar{0};
        std::optional<_Scalar> lower_bound = std::nullopt;
        std::optional<_Scalar> upper_bound = std::nullopt;
    };

    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

protected:
    unsigned int register_count;
    std::vector<std::pair<unsigned int, unsigned int>> tmp_entry_index_cache;
    std::vector<_Index> tmp_indices;
    std::vector<_Scalar> tmp_scalars;

    [[nodiscard]] explicit model_base() : register_count(0) {}

    constexpr model_base(const model_base &) = default;
    constexpr model_base(model_base &&) = default;

    constexpr model_base & operator=(const model_base &) = default;
    constexpr model_base & operator=(model_base && other) = default;

    inline auto _make_variables_view(const std::size_t & offset,
                                     const std::size_t & count) {
        return variables_view(
            std::from_range,
            std::views::transform(
                std::views::iota(static_cast<_Index>(offset),
                                 static_cast<_Index>(offset + count)),
                [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_view(const std::size_t & offset,
                                             const std::size_t & count,
                                             IL && id_lambda) {
        return variables_view(
            typename detail::function_traits<IL>::arg_types(),
            std::views::transform(
                std::views::iota(static_cast<_Index>(offset),
                                 static_cast<_Index>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }
    template <typename NL, typename M>
        requires requires(M & model, typename M::variable v, std::string n) {
            model.set_variable_name(v, n);
        }
    inline auto _make_named_variables_view(const std::size_t & offset,
                                           const std::size_t & count,
                                           NL && name_lambda, M * model) {
        for(std::size_t i = 0; i < count; ++i) {
            model->set_variable_name(variable(static_cast<int>(offset + i)),
                                     name_lambda(i));
        }
        return _make_variables_view(offset, count);
    }
    template <typename IL, typename NL, typename M>
        requires requires(M & model, typename M::variable v, std::string n) {
            model.set_variable_name(v, n);
        }
    inline auto _make_indexed_named_variables_view(const std::size_t & offset,
                                                   const std::size_t & count,
                                                   IL && id_lambda,
                                                   NL && name_lambda,
                                                   M * model) {
        return lazily_named_variables_view(
            typename detail::function_traits<IL>::arg_types(),
            std::views::transform(
                std::views::iota(static_cast<_Index>(offset),
                                 static_cast<_Index>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda), std::forward<NL>(name_lambda), model);
    }

    struct EntityId {
        template <typename E>
            requires std::derived_from<std::decay_t<E>,
                                       model_entity_base<_Index>>
        _Index operator()(E && entity) {
            return entity.id();
        }
    };

    // ids_end = max_id + 1 ; for contiguous ids (ids_end = num_ids)
    void _prepare_coalescing(const std::size_t ids_end) {
        tmp_entry_index_cache.resize(ids_end);
    }
    void _reset_cache() {
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
    }

    template <std::ranges::range Entries, typename IdProj = EntityId>
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::is_invocable_r_v<_Index, IdProj,
                                       linear_term_variable_t<
                                           std::ranges::range_value_t<Entries>>>
    void _register_raw_entries(Entries && entries, IdProj proj = {}) {
        for(auto && [entity, coef] : entries) {
            tmp_indices.emplace_back(proj(entity));
            tmp_scalars.emplace_back(coef);
        }
    }
    template <std::ranges::range Entries, typename IdProj = EntityId>
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::is_invocable_r_v<_Index, IdProj,
                                       linear_term_variable_t<
                                           std::ranges::range_value_t<Entries>>>
    void _register_coalescing_entries(Entries && entries, IdProj proj = {}) {
        ++register_count;
        for(auto && [entity, coef] : entries) {
            const _Index entity_id = proj(entity);
            auto & p = *(tmp_entry_index_cache.data() +
                         static_cast<std::ptrdiff_t>(entity_id));
            if(p.first == register_count) {
                tmp_scalars[p.second] += static_cast<_Scalar>(coef);
                continue;
            }
            p = std::make_pair(register_count, tmp_indices.size());
            tmp_indices.emplace_back(entity_id);
            tmp_scalars.emplace_back(coef);
        }
    }

    template <bool raw, std::ranges::range Entries>
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::same_as<linear_term_variable_t<
                                  std::ranges::range_value_t<Entries>>,
                              variable>
    void _register_variables_entries(Entries && entries) {
        if constexpr(raw) {
            _register_raw_entries(std::forward<Entries>(entries));
        } else {
            _register_coalescing_entries(std::forward<Entries>(entries));
        }
    }

    template <bool raw, std::ranges::range Entries>
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::same_as<linear_term_variable_t<
                                  std::ranges::range_value_t<Entries>>,
                              constraint>
    void _register_constraints_entries(Entries && entries) {
        if constexpr(raw) {
            _register_raw_entries(std::forward<Entries>(entries));
        } else {
            _register_coalescing_entries(std::forward<Entries>(entries));
        }
    }
};

}  // namespace mippp
