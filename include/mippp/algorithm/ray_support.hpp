#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

#include "mippp/algorithm/iis_messages.hpp"

namespace mippp::iis {

// Estimate each variable's contribution using A^T y: multiply the transpose
// of the constraint matrix by the full signed certificate, before discarding
// small row weights. The callback supplies sparse (column, coefficient) terms.
// Normalize y before multiplication and accumulate in extended precision. An
// invalid/overflowed projection is unavailable, never evidence to drop bounds.
// All-zero column contributions are valid, unlike an all-zero input ray.
template <std::floating_point Scalar, typename RowTerms>
    requires std::invocable<RowTerms &, std::size_t>
std::optional<std::vector<long double>> ray_column_magnitudes(
    std::size_t columns, std::span<const Scalar> ray, RowTerms && row_terms) {
    long double scale = 0;
    for(auto value : ray) {
        if(!std::isfinite(value)) return std::nullopt;
        scale = std::max(scale, std::abs(static_cast<long double>(value)));
    }
    if(scale == 0) return std::nullopt;
    std::vector<long double> weights(columns, 0);
    for(std::size_t row = 0; row < ray.size(); ++row) {
        const auto multiplier = static_cast<long double>(ray[row]) / scale;
        for(const auto & [column, coefficient] : std::invoke(row_terms, row)) {
            if(column >= columns || !std::isfinite(coefficient))
                return std::nullopt;
            weights[column] +=
                multiplier * static_cast<long double>(coefficient);
            if(!std::isfinite(weights[column])) return std::nullopt;
        }
    }
    for(auto & weight : weights) weight = std::abs(weight);
    return weights;
}

// Select candidate indices with significant certificate weights. Discarding
// small weights can remove a necessary constraint, so the caller must verify
// that the selected subsystem still conflicts before using it.
template <std::floating_point Scalar>
std::optional<std::vector<std::size_t>> ray_support(
    std::span<const Scalar> ray, double relative_tolerance = 1e-9) {
    if(!std::isfinite(relative_tolerance) || relative_tolerance < 0)
        throw std::invalid_argument(
            "The solver-hint threshold is invalid. " +
            detail::setting_help("relative_tolerance",
                                 detail::setting_value(relative_tolerance),
                                 "any finite number >= 0 (default 1e-9)"));
    Scalar scale = 0;
    for(auto value : ray) {
        if(!std::isfinite(value)) return std::nullopt;
        scale = std::max(scale, std::abs(value));
    }
    if(scale == 0) return std::nullopt;
    std::vector<std::size_t> support;
    for(std::size_t i = 0; i < ray.size(); ++i)
        if(std::abs(ray[i]) / scale > relative_tolerance) support.push_back(i);
    return support;
}
}  // namespace mippp::iis
