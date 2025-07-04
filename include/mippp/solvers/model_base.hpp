#ifndef MIPPP_MODEL_BASE_HPP
#define MIPPP_MODEL_BASE_HPP

#include <optional>
#include <ranges>
#include <vector>

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
        return variables_range(std::from_range_t{}, std::views::transform(
           std::views::iota(static_cast<_Index>(offset),
                               static_cast<_Index>(offset + count)),
            [](auto && i) { return variable{i}; }));
    }
    template <typename IL>
    inline auto _make_indexed_variables_range(const std::size_t & offset,
                                              const std::size_t & count,
                                              IL && id_lambda) {
        return variables_range(
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
    inline auto _make_named_variables_range(const std::size_t & offset,
                                            const std::size_t & count,
                                            NL && name_lambda, M * model) {
        for(std::size_t i = 0; i < count; ++i) {
            model->set_variable_name(variable(static_cast<int>(offset + i)),
                                     name_lambda(i));
        }
        return _make_variables_range(offset, count);
    }
    template <typename IL, typename NL, typename M>
        requires requires(M & model, typename M::variable v, std::string n) {
            model.set_variable_name(v, n);
        }
    inline auto _make_indexed_named_variables_range(const std::size_t & offset,
                                                    const std::size_t & count,
                                                    IL && id_lambda,
                                                    NL && name_lambda,
                                                    M * model) {
        return lazily_named_variables_range(
            typename detail::function_traits<IL>::arg_types(),
           std::views::transform(
               std::views::iota(static_cast<_Index>(offset),
                                   static_cast<_Index>(offset + count)),
                [](auto && i) { return variable{i}; }),
            std::forward<IL>(id_lambda), std::forward<NL>(name_lambda), model);
    }

    void _reset_cache(std::size_t num_variables) {
        tmp_entry_index_cache.resize(num_variables);
        tmp_indices.resize(0);
        tmp_scalars.resize(0);
    }

    template <std::ranges::range Entries>
    void _register_entries(Entries && entries) {
        ++register_count;
        for(auto && [entity, coef] : entries) {
            auto & p = tmp_entry_index_cache[entity.uid()];
            if(p.first == register_count) {
                tmp_scalars[p.second] += static_cast<_Scalar>(coef);
                continue;
            }
            p = std::make_pair(register_count, tmp_indices.size());
            tmp_indices.emplace_back(entity.id());
            tmp_scalars.emplace_back(coef);
        }
    }
    template <std::ranges::range Entries>
    void _register_raw_entries(Entries && entries) {
        for(auto && [entity, coef] : entries) {
            tmp_indices.emplace_back(entity.id());
            tmp_scalars.emplace_back(coef);
        }
    }
};

}  // namespace fhamonic::mippp

#endif  // MIPPP_MODEL_BASE_HPP