#pragma once

#include <cstddef>
#include <optional>
#include <ranges>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/linear_constraint.hpp"
#include "mippp/linear_expression.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"

namespace mippp {

// Backends derive from this class (and from remapping_model_base) with
// protected inheritance: it is an implementation detail, not a common base a
// user may name, so nothing here is frozen by the public API. The one
// member the concepts need, default_variable_params, is re-exposed by each
// backend with a public using-declaration.
template <std::integral Index, std::floating_point Scalar>
class model_base {
protected:
    using index = Index;
    using scalar = Scalar;
    using variable = model_variable<Index, Scalar>;
    using constraint = model_constraint<Index>;
    template <typename Map>
    struct variable_mapping : entity_mapping<variable, Map> {
        variable_mapping(Map && t)
            : entity_mapping<variable, Map>(std::move(t)) {}
    };
    template <typename Map>
    struct constraint_mapping : entity_mapping<constraint, Map> {
        constraint_mapping(Map && t)
            : entity_mapping<constraint, Map>(std::move(t)) {}
    };

    struct variable_params {
        Scalar obj_coef = Scalar{0};
        std::optional<Scalar> lower_bound = std::nullopt;
        std::optional<Scalar> upper_bound = std::nullopt;
    };

public:
    // the anchor model_variable_params_t deduces from
    static constexpr variable_params default_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = std::nullopt};

protected:
    std::vector<std::pair<unsigned int, unsigned int>> tmp_entry_index_cache;
    std::vector<Index> tmp_indices;
    std::vector<Scalar> tmp_scalars;
    unsigned int register_count;

    [[nodiscard]] explicit model_base() : register_count(0) {}

    constexpr model_base(const model_base &) = default;
    constexpr model_base(model_base &&) = default;

    constexpr model_base & operator=(const model_base &) = default;
    constexpr model_base & operator=(model_base && other) = default;

    template <typename KeyIndex = detail::positional_index>
    auto _variables_range(std::size_t offset, std::size_t count,
                          KeyIndex idx = {}) const {
        return entity_range(variable(static_cast<Index>(offset)), count,
                            std::move(idx));
    }
    template <typename IL, typename NL, typename M>
    auto _lazily_named_variables_range(std::size_t offset, std::size_t count,
                                       IL && id_lambda, NL && name_lambda,
                                       M * model) const {
        const variable front(static_cast<Index>(offset));
        return entity_range(
            front, count,
            detail::lazily_named_index(std::forward<IL>(id_lambda),
                                       std::forward<NL>(name_lambda), model,
                                       front, count));
    }

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////// Variable creation ///////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // Backends provide one hook,
    //     std::size_t _new_variables(std::size_t count,
    //                                const variable_params & params,
    //                                variable_kind kind);
    // returning the first handle id, declare `friend model_base<...>` for it
    // and re-expose the kinds they support with using-declarations. A
    // backend whose single additions differ, by recycling removed ids, also
    // provides
    //     variable _new_variable(const variable_params & params,
    //                            variable_kind kind);

public:
    enum class variable_kind { continuous, integer, binary };

protected:
    static constexpr variable_params binary_variable_params = {
        .obj_coef = 0, .lower_bound = 0, .upper_bound = 1};

    template <variable_kind Kind, typename KeyIndex = detail::positional_index>
    auto _add(this auto & self, std::size_t count,
              const variable_params & params, KeyIndex idx = {}) {
        return self._variables_range(self._new_variables(count, params, Kind),
                                     count, std::move(idx));
    }
    template <variable_kind Kind>
    variable _add_one(this auto & self, const variable_params & params) {
        if constexpr(requires { self._new_variable(params, Kind); })
            return self._new_variable(params, Kind);
        else
            return self.template _add<Kind>(1, params).front();
    }
    template <variable_kind Kind, typename KR>
    auto _add_keyed(this auto & self, KR & keys,
                    const variable_params & params) {
        return detail::keyed_entities(
            self, keys, detail::set_variable_name,
            self.template _add<Kind>(
                static_cast<std::size_t>(std::ranges::distance(keys)), params));
    }

public:
    variable add_variable(this auto & self,
                          variable_params params = default_variable_params) {
        return self.template _add_one<variable_kind::continuous>(params);
    }
    auto add_variables(this auto & self, std::size_t count,
                       variable_params params = default_variable_params) {
        return self.template _add<variable_kind::continuous>(count, params);
    }
    template <typename IL>
        requires(!std::convertible_to<IL, variable_params>)
    auto add_variables(this auto & self, std::size_t count, IL && id_lambda,
                       variable_params params = default_variable_params) {
        return self.template _add<variable_kind::continuous>(
            count, params, detail::lambda_index{std::forward<IL>(id_lambda)});
    }
    template <std::ranges::forward_range KR>
    auto add_variables(this auto & self, KR && keys,
                       variable_params params = default_variable_params) {
        return self.template _add_keyed<variable_kind::continuous>(keys,
                                                                   params);
    }

    variable add_integer_variable(
        this auto & self, variable_params params = default_variable_params) {
        return self.template _add_one<variable_kind::integer>(params);
    }
    auto add_integer_variables(
        this auto & self, std::size_t count,
        variable_params params = default_variable_params) {
        return self.template _add<variable_kind::integer>(count, params);
    }
    template <typename IL>
        requires(!std::convertible_to<IL, variable_params>)
    auto add_integer_variables(
        this auto & self, std::size_t count, IL && id_lambda,
        variable_params params = default_variable_params) {
        return self.template _add<variable_kind::integer>(
            count, params, detail::lambda_index{std::forward<IL>(id_lambda)});
    }
    template <std::ranges::forward_range KR>
    auto add_integer_variables(
        this auto & self, KR && keys,
        variable_params params = default_variable_params) {
        return self.template _add_keyed<variable_kind::integer>(keys, params);
    }

    variable add_binary_variable(this auto & self) {
        return self.template _add_one<variable_kind::binary>(
            binary_variable_params);
    }
    auto add_binary_variables(this auto & self, std::size_t count) {
        return self.template _add<variable_kind::binary>(
            count, binary_variable_params);
    }
    template <typename IL>
        requires(!std::convertible_to<IL, variable_params>)
    auto add_binary_variables(this auto & self, std::size_t count,
                              IL && id_lambda) {
        return self.template _add<variable_kind::binary>(
            count, binary_variable_params,
            detail::lambda_index{std::forward<IL>(id_lambda)});
    }
    template <std::ranges::forward_range KR>
    auto add_binary_variables(this auto & self, KR && keys) {
        return self.template _add_keyed<variable_kind::binary>(
            keys, binary_variable_params);
    }

    variable add_named_variable(
        this auto & self, const std::string & name,
        variable_params params = default_variable_params) {
        const variable v = self.add_variable(params);
        self.set_variable_name(v, name);
        return v;
    }
    template <typename IL, typename NL>
    auto add_named_variables(this auto & self, std::size_t count,
                             IL && id_lambda, NL && name_lambda,
                             variable_params params = default_variable_params) {
        const std::size_t offset =
            self._new_variables(count, params, variable_kind::continuous);
        return self._lazily_named_variables_range(
            offset, count, std::forward<IL>(id_lambda),
            std::forward<NL>(name_lambda), &self);
    }

protected:
    struct EntityId {
        template <typename E>
            requires std::derived_from<std::decay_t<E>,
                                       model_entity_base<Index>>
        Index operator()(E && entity) {
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
                 std::is_invocable_r_v<Index, IdProj,
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
                 std::is_invocable_r_v<Index, IdProj,
                                       linear_term_variable_t<
                                           std::ranges::range_value_t<Entries>>>
    void _register_coalescing_entries(Entries && entries, IdProj proj = {}) {
        ++register_count;
        for(auto && [entity, coef] : entries) {
            const Index entity_id = proj(entity);
            auto & p = *(tmp_entry_index_cache.data() +
                         static_cast<std::ptrdiff_t>(entity_id));
            if(p.first == register_count) {
                tmp_scalars[p.second] += static_cast<Scalar>(coef);
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
