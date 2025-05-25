#ifndef MIPPP_MODEL_BASE_HPP
#define MIPPP_MODEL_BASE_HPP

#include <optional>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

namespace fhamonic::mippp {

template <std::integral _Index, std::floating_point _Scalar>
class model_base {
public:
    using variable = model_variable<_Index, _Scalar>;

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

    inline auto _make_variables_range(const std::size_t & offset,
                                      const std::size_t & count) {
        return make_variables_range(ranges::view::transform(
            ranges::view::iota(static_cast<_Index>(offset),
                               static_cast<_Index>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_range(const std::size_t & offset,
                                              const std::size_t & count,
                                              IL && id_lambda) {
        return make_indexed_variables_range(
            typename detail::function_traits<IL>::arg_types(),
            ranges::view::transform(
                ranges::view::iota(static_cast<_Index>(offset),
                                   static_cast<_Index>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda));
    }

    void _reset_cache(std::size_t num_variables) {
        tmp_entry_index_cache.resize(num_variables);
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
    }

    template <ranges::range Entries>
    void _register_entries(Entries && entries) {
        ++register_count;
        for(auto && [entity, coef] : entries) {
            auto & p = tmp_entry_index_cache[entity.uid()];
            if(p.first == register_count) {
                tmp_scalars[p.second] += coef;
                continue;
            }
            p = std::make_pair(register_count, tmp_indices.size());
            tmp_indices.emplace_back(entity.id());
            tmp_scalars.emplace_back(coef);
        }
    }
    template <ranges::range Entries>
    void _register_raw_entries(Entries && entries) {
        for(auto && [entity, coef] : entries) {
            tmp_indices.emplace_back(entity.id());
            tmp_scalars.emplace_back(coef);
        }
    }
};

}  // namespace fhamonic::mippp

#endif  // MIPPP_MODEL_BASE_HPP