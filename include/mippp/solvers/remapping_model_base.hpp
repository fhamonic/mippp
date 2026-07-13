#ifndef MIPPP_remapping_model_base_HPP
#define MIPPP_remapping_model_base_HPP

#include <concepts>
#include <optional>
#include <ranges>
#include <vector>

#include "mippp/solvers/model_base.hpp"

namespace mippp {

template <std::integral _Index, std::floating_point _Scalar>
class remapping_model_base : public model_base<_Index, _Scalar> {
protected:
    using typename model_base<_Index, _Scalar>::variable;
    using model_base<_Index, _Scalar>::register_count;
    using model_base<_Index, _Scalar>::tmp_entry_index_cache;
    using model_base<_Index, _Scalar>::tmp_indices;
    using model_base<_Index, _Scalar>::tmp_scalars;
    using model_base<_Index, _Scalar>::_register_entries;
    using model_base<_Index, _Scalar>::_register_raw_entries;

    std::vector<variable> _var_handles_to_delete;
    std::vector<variable> _free_var_handles;
    std::vector<int> _native_ids_map;
    std::vector<int> _handle_ids_map;
    bool _remap_ids;

    [[nodiscard]] explicit remapping_model_base()
        : model_base<_Index, _Scalar>(), _remap_ids(false) {}

    constexpr remapping_model_base(const remapping_model_base &) = default;
    constexpr remapping_model_base(remapping_model_base &&) = default;

    constexpr remapping_model_base & operator=(const remapping_model_base &) =
        default;
    constexpr remapping_model_base & operator=(remapping_model_base && other) =
        default;

    int _native_id(const variable variable_handle) const {
        if(!_remap_ids) return variable_handle.id();
        return _native_ids_map[static_cast<std::size_t>(variable_handle.id())];
    }
    variable _var_handle(const int native_id) const {
        if(!_remap_ids) return variable(native_id);
        return variable(_handle_ids_map[static_cast<std::size_t>(native_id)]);
    }

    void _extend_handle_ids_map(const std::size_t count) {
        _handle_ids_map.resize(_handle_ids_map.size() + count);
    }
    void _shrink_handle_ids_map(const std::size_t count) {
        _handle_ids_map.resize(_handle_ids_map.size() - count);
    }

    variable _new_var_handle(const int new_native_id) {
        if(!_remap_ids) return variable(new_native_id);
        int new_handle_id;
        if(_free_var_handles.empty()) {
            new_handle_id = static_cast<int>(_native_ids_map.size());
            _native_ids_map.push_back(new_native_id);
        } else {
            new_handle_id = _free_var_handles.back().id();
            _free_var_handles.pop_back();
            _native_ids_map[static_cast<std::size_t>(new_handle_id)] =
                new_native_id;
        }
        _handle_ids_map[static_cast<std::size_t>(new_native_id)] =
            new_handle_id;
        return variable(new_handle_id);
    }
    std::size_t _new_var_handle_range(const std::size_t num_native_ids,
                                      const std::size_t count) {
        if(!_remap_ids) return num_native_ids;
        const std::size_t new_handle_ids_begin = _native_ids_map.size();
        for(std::size_t i = 0; i < count; ++i) {
            _native_ids_map.emplace_back(num_native_ids + i);
            _handle_ids_map[num_native_ids + i] =
                static_cast<int>(new_handle_ids_begin + i);
        }
        return new_handle_ids_begin;
    }

    template <std::ranges::range Entries>
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::same_as<linear_term_variable_t<
                                  std::ranges::range_value_t<Entries>>,
                              variable>
    void _register_entries(Entries && entries) {
        ++register_count;
        if(_remap_ids) {
            for(auto && [entity, coef] : entries) {
                const std::size_t native_id =
                    static_cast<std::size_t>(_native_ids_map[entity.uid()]);
                auto & p = tmp_entry_index_cache[native_id];
                if(p.first == register_count) {
                    tmp_scalars[p.second] += static_cast<_Scalar>(coef);
                    continue;
                }
                p = std::make_pair(register_count, tmp_indices.size());
                tmp_indices.emplace_back(native_id);
                tmp_scalars.emplace_back(coef);
            }
            return;
        }
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
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::same_as<linear_term_variable_t<
                                  std::ranges::range_value_t<Entries>>,
                              variable>
    void _register_raw_entries(Entries && entries) {
        if(_remap_ids) {
            for(auto && [entity, coef] : entries) {
                tmp_indices.emplace_back(_native_ids_map[entity.uid()]);
                tmp_scalars.emplace_back(coef);
            }
            return;
        }
        for(auto && [entity, coef] : entries) {
            tmp_indices.emplace_back(entity.id());
            tmp_scalars.emplace_back(coef);
        }
    }
};

}  // namespace mippp

#endif  // MIPPP_MODEL_BASE_HPP