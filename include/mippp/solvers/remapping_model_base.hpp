#pragma once

#include <concepts>
#include <cstddef>
#include <optional>
#include <ranges>
#include <utility>
#include <vector>

#include "mippp/solvers/model_base.hpp"

namespace mippp {

template <std::integral Index, std::floating_point Scalar>
class remapping_model_base : protected model_base<Index, Scalar> {
protected:
    using typename model_base<Index, Scalar>::variable;
    using typename model_base<Index, Scalar>::constraint;
    using model_base<Index, Scalar>::_register_raw_entries;
    using model_base<Index, Scalar>::_register_coalescing_entries;

    std::vector<variable> _var_handles_to_delete;
    std::vector<variable> _free_var_handles;
    std::vector<Index> _native_ids_map;
    std::vector<Index> _handle_ids_map;
    bool _remap_ids;

    [[nodiscard]] explicit remapping_model_base()
        : model_base<Index, Scalar>(), _remap_ids(false) {}

    constexpr remapping_model_base(const remapping_model_base &) = default;
    constexpr remapping_model_base(remapping_model_base &&) = default;

    constexpr remapping_model_base & operator=(const remapping_model_base &) =
        default;
    constexpr remapping_model_base & operator=(remapping_model_base && other) =
        default;

    Index _native_id(const variable variable_handle) const {
        if(!_remap_ids) return variable_handle.id();
        return _native_ids_map[static_cast<std::size_t>(variable_handle.id())];
    }
    variable _var_handle(const Index native_id) const {
        if(!_remap_ids) return variable(native_id);
        return variable(_handle_ids_map[static_cast<std::size_t>(native_id)]);
    }

    void _extend_handle_ids_map(const std::size_t count) {
        _handle_ids_map.resize(_handle_ids_map.size() + count);
    }
    void _shrink_handle_ids_map(const std::size_t count) {
        _handle_ids_map.resize(_handle_ids_map.size() - count);
    }

    variable _new_var_handle(const Index new_native_id) {
        if(!_remap_ids) return variable(new_native_id);
        Index new_handle_id;
        if(_free_var_handles.empty()) {
            new_handle_id = static_cast<Index>(_native_ids_map.size());
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
            _native_ids_map.emplace_back(
                static_cast<Index>(num_native_ids + i));
            _handle_ids_map[num_native_ids + i] =
                static_cast<Index>(new_handle_ids_begin + i);
        }
        return new_handle_ids_begin;
    }

    template <bool raw, std::ranges::range Entries>
        requires linear_term<std::ranges::range_value_t<Entries>> &&
                 std::same_as<linear_term_variable_t<
                                  std::ranges::range_value_t<Entries>>,
                              variable>
    void _register_variables_entries(Entries && entries) {
        const auto native_id = [native_ids =
                                    _native_ids_map.data()](auto && e) {
            return *(native_ids + static_cast<std::ptrdiff_t>(e.id()));
        };
        if constexpr(raw) {
            if(!_remap_ids)
                _register_raw_entries(std::forward<Entries>(entries));
            else
                _register_raw_entries(std::forward<Entries>(entries),
                                      native_id);
        } else {
            if(!_remap_ids)
                _register_coalescing_entries(std::forward<Entries>(entries));
            else
                _register_coalescing_entries(std::forward<Entries>(entries),
                                             native_id);
        }
    }
};

}  // namespace mippp
