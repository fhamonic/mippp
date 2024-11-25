/**
 * @file MILP_Builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-08-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef MIPPP_MODEL_HPP
#define MIPPP_MODEL_HPP

#include <cassert>
#include <limits>
#include <ostream>
#include <sstream>
#include <vector>

#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/repeat_n.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_constraint.hpp"
#include "mippp/constraints/linear_constraint.hpp"
#include "mippp/detail/function_traits.hpp"
#include "mippp/expressions/linear_expression.hpp"
#include "mippp/solvers/all.hpp"
#include "mippp/variable.hpp"

namespace fhamonic {
namespace mippp {

template <typename Traits = default_model_traits>
class mip_model {
public:
    using solver_traits = Traits;
    using var_id_t = typename Traits::var_id_t;
    using constraint_id_t = typename Traits::constraint_id_t;
    using scalar_t = typename Traits::scalar_t;
    using var = variable<var_id_t, scalar_t>;
    using opt_sense = typename Traits::opt_sense;
    using var_category = typename Traits::var_category;

    static constexpr scalar_t minus_infinity = Traits::minus_infinity;
    static constexpr scalar_t infinity = Traits::infinity;

private:
    std::vector<scalar_t> _col_coef;
    std::vector<scalar_t> _col_lb;
    std::vector<scalar_t> _col_ub;
    std::vector<var_category> _col_type;
    std::vector<std::optional<std::string>> _col_name;

    std::vector<var_id_t> _vars;
    std::vector<scalar_t> _coefs;

    std::vector<int> _row_begins;
    std::vector<scalar_t> _row_lb;
    std::vector<scalar_t> _row_ub;

    opt_sense _sense;

public:
    [[nodiscard]] explicit mip_model(opt_sense sense = opt_sense::max)
        : _sense(sense) {}

    opt_sense get_opt_sense() const noexcept { return _sense; }
    mip_model & set_opt_sense(opt_sense s) noexcept {
        _sense = s;
        return *this;
    }

    struct var_options {
        scalar_t obj_coef = scalar_t{0};
        scalar_t lower_bound = scalar_t{0};
        scalar_t upper_bound = infinity;
        var_category type = var_category::continuous;
    };

    template <typename S>
        requires std::convertible_to<S, std::string_view>
    var add_variable(S && name, var_options options = {}) noexcept {
        _col_coef.push_back(options.obj_coef);
        _col_lb.push_back(options.lower_bound);
        _col_ub.push_back(options.upper_bound);
        _col_type.push_back(options.type);
        _col_name.emplace_back(std::forward<S>(name));
        return var(static_cast<var_id_t>(num_variables() - 1));
    }
    var add_variable(var_options options = {}) noexcept {
        return add_variable("x" + std::to_string(num_variables()), options);
    }

    template <typename IL, typename NL, typename... Args>
    class vars_range {
    public:
        using var_id_t = mip_model::var_id_t;
        using scalar_t = mip_model::scalar_t;
        using var = mip_model::var;

    private:
        const std::size_t _offset;
        const std::size_t _count;
        const IL _id_lambda;
        mutable NL _name_lambda;

    public:
        constexpr vars_range(detail::pack<Args...>, std::size_t offset,
                             std::size_t count, IL && id_lambda,
                             NL && name_lambda) noexcept
            : _offset(offset)
            , _count(count)
            , _id_lambda(std::forward<IL>(id_lambda))
            , _name_lambda(std::forward<NL>(name_lambda)) {}

        constexpr var operator()(Args... args) const noexcept {
            const var_id_t id = static_cast<var_id_t>(_id_lambda(args...));
            assert(static_cast<std::size_t>(id) < _count);
            const var_id_t var_num = static_cast<var_id_t>(_offset) + id;
            _name_lambda(var_num, args...);
            return var(var_num);
        }

        constexpr auto variables() const noexcept {
            return ranges::iota_view<var_id_t, var_id_t>(
                static_cast<var_id_t>(_offset),
                static_cast<var_id_t>(_offset + _count));
        }
        constexpr auto coefficients() const noexcept {
            return ranges::views::repeat_n(scalar_t{1}, _count);
        }
        constexpr scalar_t constant() const noexcept { return scalar_t{0}; }
    };

private:
    void _add_cols(std::size_t count, var_options options = {}) {
        const std::size_t new_size = num_variables() + count;
        _col_coef.resize(new_size, options.obj_coef);
        _col_lb.resize(new_size, options.lower_bound);
        _col_ub.resize(new_size, options.upper_bound);
        _col_type.resize(new_size, options.type);
        _col_name.resize(new_size);
    }

    template <typename IL, typename... Args>
        requires std::convertible_to<
            typename detail::function_traits<IL>::result_type, var_id_t>
    auto _add_variables(detail::pack<Args...>, std::size_t count,
                        IL && id_lambda, var_options options = {}) noexcept {
        const std::size_t offset = num_variables();
        _add_cols(count, options);
        return vars_range(
            typename detail::function_traits<IL>::arg_types(), offset, count,
            std::forward<IL>(id_lambda),
            [this](const var_id_t var_num, Args... args) mutable {
                if(_col_name[static_cast<std::size_t>(var_num)].has_value())
                    return;
                _col_name[static_cast<std::size_t>(var_num)].emplace(
                    "x" + std::to_string(var_num));
            });
    }
    template <typename IL, typename NL, typename... Args>
        requires std::convertible_to<
            typename detail::function_traits<IL>::result_type, var_id_t>
    auto _add_variables(detail::pack<Args...>, std::size_t count,
                        IL && id_lambda, NL && name_lambda,
                        var_options options = {}) noexcept {
        const std::size_t offset = num_variables();
        _add_cols(count, options);
        return vars_range(
            typename detail::function_traits<IL>::arg_types(), offset, count,
            std::forward<IL>(id_lambda),
            [this, name_lambda = std::forward<NL>(name_lambda)](
                const var_id_t var_num, Args... args) mutable {
                if(_col_name[static_cast<std::size_t>(var_num)].has_value())
                    return;
                _col_name[static_cast<std::size_t>(var_num)].emplace(
                    name_lambda(args...));
            });
    }

public:
    template <typename IL>
        requires std::convertible_to<
            typename detail::function_traits<IL>::result_type, var_id_t>
    auto add_variables(std::size_t count, IL && id_lambda,
                       var_options options = {}) noexcept {
        return _add_variables(typename detail::function_traits<IL>::arg_types(),
                              count, std::forward<IL>(id_lambda), options);
    }
    template <typename IL, typename NL>
        requires std::convertible_to<
            typename detail::function_traits<IL>::result_type, var_id_t>
    auto add_variables(std::size_t count, IL && id_lambda, NL && name_lambda,
                       var_options options = {}) noexcept {
        return _add_variables(typename detail::function_traits<IL>::arg_types(),
                              count, std::forward<IL>(id_lambda),
                              std::forward<NL>(name_lambda), options);
    }

    template <linear_expression_c E>
    mip_model & add_to_objective(E && le) noexcept {
        auto entries_range =
            ranges::views::zip(le.variables(), le.coefficients());
        for(auto && [v, c] : entries_range) {
            _col_coef[static_cast<std::size_t>(v)] += c;
        }
        return *this;
    }

    template <linear_constraint_c C>
    constraint_id_t add_constraint(C && lc) noexcept {
        _row_begins.emplace_back(num_entries());
        _row_lb.emplace_back(lc.lower_bound());
        _row_ub.emplace_back(lc.upper_bound());
        std::ranges::copy(lc.variables(), std::back_inserter(_vars));
        std::ranges::copy(lc.coefficients(), std::back_inserter(_coefs));
        return constraint_id_t(num_constraints() - 1);
    }

    std::size_t num_variables() const { return _col_coef.size(); }
    std::size_t num_constraints() const { return _row_begins.size(); }
    std::size_t num_entries() const { return _vars.size(); }

    // Variables
    scalar_t & obj_coef(var v) noexcept {
        return _col_coef[static_cast<std::size_t>(v.id())];
    }
    scalar_t & lower_bound(var v) noexcept {
        return _col_lb[static_cast<std::size_t>(v.id())];
    }
    scalar_t & upper_bound(var v) noexcept {
        return _col_ub[static_cast<std::size_t>(v.id())];
    }
    var_category & type(var v) noexcept {
        return _col_type[static_cast<std::size_t>(v.id())];
    }
    std::string & name(var v) noexcept {
        auto & opt_name = _col_name[static_cast<std::size_t>(v.id())];
        assert(opt_name.has_value());
        return opt_name.value();
    }

    scalar_t obj_coef(var v) const noexcept {
        return _col_coef[static_cast<std::size_t>(v.id())];
    }
    scalar_t lower_bound(var v) const noexcept {
        return _col_lb[static_cast<std::size_t>(v.id())];
    }
    scalar_t upper_bound(var v) const noexcept {
        return _col_ub[static_cast<std::size_t>(v.id())];
    }
    var_category type(var v) const noexcept {
        return _col_type[static_cast<std::size_t>(v.id())];
    }
    const std::string & name(var v) const noexcept {
        auto & opt_name = _col_name[static_cast<std::size_t>(v.id())];
        assert(opt_name.has_value());
        return opt_name.value();
    }

    // Views
    auto variables() const noexcept {
        return ranges::iota_view<var_id_t, var_id_t>(
            var_id_t{0}, static_cast<var_id_t>(num_variables()));
    }
    auto objective() const noexcept {
        return linear_expression<decltype(variables()),
                                 const std::vector<scalar_t> &, scalar_t>(
            variables(), _col_coef, scalar_t(0));
    }
    auto constraint(constraint_id_t constraint_id) const noexcept {
        assert(constraint_id < num_constraints());
        const std::size_t row_begin =
            static_cast<std::size_t>(_row_begins[constraint_id]);
        const std::size_t row_end =
            (constraint_id < num_constraints() - 1)
                ? static_cast<std::size_t>(_row_begins[constraint_id + 1])
                : num_entries();
        return linear_constraint(
            ranges::subrange(_vars.data() + row_begin, _vars.data() + row_end),
            ranges::subrange(_coefs.data() + row_begin,
                             _coefs.data() + row_end),
            _row_lb[constraint_id], _row_ub[constraint_id]);
    }
    auto constraint_ids() const noexcept {
        return ranges::iota_view<constraint_id_t, constraint_id_t>(
            constraint_id_t{0}, static_cast<constraint_id_t>(num_constraints()));
    }
    auto constraints() const noexcept {
        return ranges::views::transform(constraint_ids(),
                                        &mip_model::constraint);
    }

    auto optimization_sense() const noexcept { return _sense; }
    auto column_coefs() const noexcept { return _col_coef.data(); }
    auto column_lower_bounds() const noexcept { return _col_lb.data(); }
    auto column_upper_bounds() const noexcept { return _col_ub.data(); }
    auto column_types() const noexcept { return _col_type.data(); }
    auto column_name() const noexcept { return _col_name.data(); }
    auto row_begins() const noexcept { return _row_begins.data(); }
    auto var_entries() const noexcept { return _vars.data(); }
    auto coef_entries() const noexcept { return _coefs.data(); }
    auto row_lower_bounds() const noexcept { return _row_lb.data(); }
    auto row_upper_bounds() const noexcept { return _row_ub.data(); }
};

template <typename T, typename NL>
std::ostream & print_entries(std::ostream & os, const T & e,
                             const NL & name_lambda) {
    using var_id_t = typename T::var_id_t;
    using scalar_t = typename T::scalar_t;
    auto entries_range = ranges::views::zip(e.variables(), e.coefficients());
    auto it = entries_range.begin();
    const auto end = entries_range.end();
    if(it == end) return os;
    for(; it != end; ++it) {
        var_id_t v = (*it).first;
        scalar_t coef = (*it).second;
        if(coef == scalar_t(0)) continue;
        const scalar_t abs_coef = std::abs(coef);
        os << (coef < 0 ? "-" : "");
        if(abs_coef != 1) os << abs_coef << " ";
        os << name_lambda(v);
        break;
    }
    for(++it; it != end; ++it) {
        var_id_t v = (*it).first;
        scalar_t coef = (*it).second;
        if(coef == scalar_t(0)) continue;
        const scalar_t abs_coef = std::abs(coef);
        os << (coef < 0 ? " - " : " + ");
        if(abs_coef != 1) os << abs_coef << " ";
        os << name_lambda(v);
    }
    return os;
}

template <typename Traits>
std::ostream & operator<<(std::ostream & os, const mip_model<Traits> & model) {
    using var_id_t = mip_model<Traits>::var_id_t;
    using scalar_t = mip_model<Traits>::scalar_t;
    using var = variable<var_id_t, scalar_t>;
    auto name_lambda = [&](var_id_t id) { return model.name(var(id)); };
    os << (model.get_opt_sense() == mip_model<Traits>::opt_sense::min
               ? "Minimize"
               : "Maximize")
       << '\n';
    print_entries(os, model.objective(), name_lambda);
    os << "\nSubject To\n";
    for(auto && constr_id : model.constraint_ids()) {
        auto && constr = model.constraint(constr_id);
        const scalar_t lb = constr.lower_bound();
        const scalar_t ub = constr.upper_bound();
        if(ub < mip_model<Traits>::infinity) {
            os << "R" << constr_id << ": ";
            print_entries(os, constr, name_lambda);
            os << " <= " << ub << '\n';
        }
        if(lb > mip_model<Traits>::minus_infinity) {
            os << "R" << constr_id << "_low: ";
            print_entries(os, constr, name_lambda);
            os << " >= " << lb << '\n';
        }
    }
    auto no_trivial_bound_vars =
        ranges::views::filter(model.variables(), [&model](var_id_t v) {
            return model.lower_bound(var(v)) != scalar_t(0) ||
                   model.upper_bound(var(v)) != mip_model<Traits>::infinity;
        });
    if(ranges::distance(no_trivial_bound_vars) > 0) {
        os << "Bounds\n";
        for(auto && v : no_trivial_bound_vars) {
            if(model.lower_bound(var(v)) == model.upper_bound(var(v))) {
                os << name_lambda(v) << " = " << model.upper_bound(var(v))
                   << '\n';
                continue;
            }
            if(model.lower_bound(var(v)) != scalar_t(0)) {
                if(model.lower_bound(var(v)) ==
                   mip_model<Traits>::minus_infinity)
                    os << "-Inf <= ";
                else
                    os << model.lower_bound(var(v)) << " <= ";
            }
            os << name_lambda(v);
            if(model.upper_bound(var(v)) != mip_model<Traits>::infinity)
                os << " <= " << model.upper_bound(var(v));
            os << '\n';
        }
    }
    auto binary_vars =
        ranges::filter_view(model.variables(), [&model](var_id_t v) {
            return model.type(var(v)) ==
                   mip_model<Traits>::var_category::binary;
        });
    if(ranges::distance(binary_vars) > 0) {
        os << "Binary\n";
        for(auto && v : binary_vars) {
            os << ' ' << name_lambda(v);
        }
        os << '\n';
    }
    auto interger_vars =
        ranges::views::filter(model.variables(), [&model](var_id_t v) {
            return model.type(var(v)) ==
                   mip_model<Traits>::var_category::integer;
        });
    if(ranges::distance(interger_vars) > 0) {
        os << "General\n";
        for(auto && v : interger_vars) {
            os << ' ' << name_lambda(v);
        }
        os << '\n';
    }
    return os << "End" << '\n';
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_HPP