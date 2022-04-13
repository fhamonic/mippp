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
#include <vector>

#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/single.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include "mippp/concepts/id_value_map.hpp"
#include "mippp/concepts/linear_constraint.hpp"
#include "mippp/constraints/linear_constraint.hpp"
#include "mippp/detail/function_traits.hpp"
#include "mippp/expressions/linear_expression.hpp"
#include "mippp/solver_traits/all.hpp"
#include "mippp/variable.hpp"

namespace fhamonic {
namespace mippp {

template <typename SolverTraits>
class Model {
public:
    static constexpr double MINUS_INFTY = std::numeric_limits<double>::lowest();
    static constexpr double INFTY = std::numeric_limits<double>::max();

    using OptSense = typename SolverTraits::OptSense;
    using ColType = typename SolverTraits::ColType;
    using ModelType = typename SolverTraits::ModelType;

    using var_id_t = int;
    using scalar_t = double;
    using constraint_id_t = std::size_t;

    using Var = variable<var_id_t, scalar_t>;

private:
    std::vector<scalar_t> _col_coef;
    std::vector<scalar_t> _col_lb;
    std::vector<scalar_t> _col_ub;
    std::vector<ColType> _col_type;

    std::vector<var_id_t> _vars;
    std::vector<scalar_t> _coefs;

    std::vector<int> _row_begins;
    std::vector<scalar_t> _row_lb;
    std::vector<scalar_t> _row_ub;

    OptSense _sense;

public:
    Model(OptSense sense = OptSense::MAXIMIZE) : _sense(sense) {}

    OptSense opt_sense() const noexcept { return _sense; }
    Model & opt_sense(OptSense s) noexcept {
        _sense = s;
        return *this;
    }

    struct var_options {
        scalar_t obj_coef = scalar_t{0};
        scalar_t lower_bound = scalar_t{0};
        scalar_t upper_bound = INFTY;
        ColType type = ColType::CONTINUOUS;
    };

    Var add_var(var_options options = {}) noexcept {
        _col_coef.push_back(options.obj_coef);
        _col_lb.push_back(options.lower_bound);
        _col_ub.push_back(options.upper_bound);
        _col_type.push_back(options.type);
        return Var(static_cast<var_id_t>(nb_variables() - 1));
    }

private:
    template <typename T, typename... Args>
    auto add_vars(detail::pack<Args...>, std::size_t count, T && id_lambda,
                  var_options options = {}) noexcept {
        const std::size_t offset = nb_variables();
        const std::size_t new_size = offset + count;
        _col_coef.resize(new_size, options.obj_coef);
        _col_lb.resize(new_size, options.lower_bound);
        _col_ub.resize(new_size, options.upper_bound);
        _col_type.resize(new_size, options.type);
        return [offset, count,
                id_lambda = std::forward<T>(id_lambda)](Args... args) {
            const var_id_t id = id_lambda(args...);
            assert(static_cast<std::size_t>(id) < count);
            return Var(static_cast<var_id_t>(offset) + id);
        };
    }

public:
    template <typename T>
    requires std::convertible_to<
        typename detail::function_traits<T>::result_type, var_id_t>
    auto add_vars(std::size_t count, T && id_lambda,
                  var_options options = {}) noexcept {
        return add_vars(typename detail::function_traits<T>::arg_types(), count,
                        std::forward<T>(id_lambda), options);
    }

    template <std::ranges::range R,
              id_value_map<std::ranges::range_value_t<R>, var_id_t> M>
    auto add_vars(R && values, M && id_map, var_options options = {}) noexcept {
        using value_t = std::ranges::range_value_t<R>;
        std::size_t count = 0;
        for(auto && v : values) {
            id_map[v] = static_cast<var_id_t>(count);
            ++count;
        }
        return add_vars(
            detail::pack<value_t>(), count,
            [id_map = std::forward<M>(id_map)](value_t v) {
                return id_map.at(Var(v));
            },
            options);
    }

    template <linear_expression_c E>
    Model & add_obj(E && le) noexcept {
        auto entries_range =
            ranges::views::zip(le.variables(), le.coefficients());
        for(auto && [v, c] : entries_range) {
            _col_coef[static_cast<std::size_t>(v)] += c;
        }
        return *this;
    }

    template <linear_constraint_c C>
    constraint_id_t add_constraint(C && lc) noexcept {
        _row_begins.emplace_back(nb_entries());
        _row_lb.emplace_back(lc.lower_bound());
        _row_ub.emplace_back(lc.upper_bound());
        std::ranges::copy(lc.variables(), std::back_inserter(_vars));
        std::ranges::copy(lc.coefficients(), std::back_inserter(_coefs));
        return constraint_id_t(nb_constraints() - 1);
    }

    std::size_t nb_variables() const { return _col_coef.size(); }
    std::size_t nb_constraints() const { return _row_begins.size(); }
    std::size_t nb_entries() const { return _vars.size(); }

    // Variables
    scalar_t & obj_coef(Var v) noexcept {
        return _col_coef[static_cast<std::size_t>(v.id())];
    }
    scalar_t & lower_bound(Var v) noexcept {
        return _col_lb[static_cast<std::size_t>(v.id())];
    }
    scalar_t & upper_bound(Var v) noexcept {
        return _col_ub[static_cast<std::size_t>(v.id())];
    }
    ColType & type(Var v) noexcept {
        return _col_type[static_cast<std::size_t>(v.id())];
    }

    scalar_t obj_coef(Var v) const noexcept { return obj_coef(v); }
    scalar_t lower_bound(Var v) const noexcept { return lower_bound(v); }
    scalar_t upper_bound(Var v) const noexcept { return upper_bound(v); }
    ColType type(Var v) const noexcept { return type(v); }

    // Views
    auto variables() const noexcept {
        return ranges::iota_view<var_id_t, var_id_t>(
            var_id_t{0}, static_cast<var_id_t>(nb_variables()));
    }
    auto objective() const noexcept {
        return linear_expression<decltype(variables()),
                                 const std::vector<scalar_t> &, scalar_t>(
            variables(), _col_coef, scalar_t(0));
    }
    auto constraint(constraint_id_t constraint_id) const noexcept {
        assert(constraint_id < nb_constraints());
        const std::size_t row_begin =
            static_cast<std::size_t>(_row_begins[constraint_id]);
        const std::size_t row_end =
            (constraint_id < nb_constraints() - 1)
                ? static_cast<std::size_t>(_row_begins[constraint_id + 1])
                : nb_entries();
        return linear_constraint(
            ranges::subrange(_vars.data() + row_begin, _vars.data() + row_end),
            ranges::subrange(_coefs.data() + row_begin,
                             _coefs.data() + row_end),
            _row_lb[constraint_id], _row_ub[constraint_id]);
    }
    auto constraint_ids() const noexcept {
        return ranges::iota_view<constraint_id_t, constraint_id_t>(
            constraint_id_t{0}, static_cast<constraint_id_t>(nb_constraints()));
    }
    auto constraints() const noexcept {
        return ranges::views::transform(
            constraint_ids(), [this](auto && id) { return constraint(id); });
    }

    ModelType build() noexcept {
        ModelType model = SolverTraits::build(
            _sense, static_cast<int>(nb_variables()), _col_coef.data(),
            _col_lb.data(), _col_ub.data(), _col_type.data(),
            static_cast<int>(nb_constraints()), static_cast<int>(nb_entries()),
            _row_begins.data(), _vars.data(), _coefs.data(), _row_lb.data(),
            _row_ub.data());
        return model;
    }
};

template <typename T>
std::ostream & print_entries(std::ostream & os, const T & e) {
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
        os << "x" << v;
        break;
    }
    for(++it; it != end; ++it) {
        var_id_t v = (*it).first;
        scalar_t coef = (*it).second;
        if(coef == scalar_t(0)) continue;
        const scalar_t abs_coef = std::abs(coef);
        os << (coef < 0 ? " - " : " + ");
        if(abs_coef != 1) os << abs_coef << " ";
        os << "x" << v;
    }
    return os;
}

template <typename SolverTraits>
std::ostream & operator<<(std::ostream & os,
                          const Model<SolverTraits> & model) {
    using var_id_t = Model<SolverTraits>::var_id_t;
    using scalar_t = Model<SolverTraits>::scalar_t;
    using Var = variable<var_id_t, scalar_t>;
    os << (model.opt_sense() == Model<SolverTraits>::OptSense::MINIMIZE
               ? "Minimize"
               : "Maximize")
       << '\n';
    print_entries(os, model.objective());
    os << "\nSubject To\n";
    for(auto && constr_id : model.constraint_ids()) {
        auto && constr = model.constraint(constr_id);
        const double lb = constr.lower_bound();
        const double ub = constr.upper_bound();
        if(ub < Model<SolverTraits>::INFTY) {
            os << "R" << constr_id << ": ";
            print_entries(os, constr);
            os << " <= " << ub << '\n';
        }
        if(lb > Model<SolverTraits>::MINUS_INFTY) {
            os << "R" << constr_id << "_low: ";
            print_entries(os, constr);
            os << " >= " << lb << '\n';
        }
    }
    auto interger_vars =
        ranges::views::filter(model.variables(), [&model](var_id_t v) {
            return model.type(Var(v)) == Model<SolverTraits>::ColType::INTEGER;
        });
    if(ranges::distance(interger_vars) > 0) {
        os << "General\n";
        for(auto && v : interger_vars) {
            os << " x" << v;
        }
        os << '\n';
    }
    auto binary_vars =
        ranges::filter_view(model.variables(), [&model](var_id_t v) {
            return model.type(Var(v)) == Model<SolverTraits>::ColType::BINARY;
        });
    if(ranges::distance(binary_vars) > 0) {
        os << "Binary\n";
        for(auto && v : binary_vars) {
            os << " x" << v;
        }
        os << '\n';
    }
    auto no_trivial_bound_vars =
        ranges::views::filter(model.variables(), [&model](var_id_t v) {
            return model.lower_bound(Var(v)) != scalar_t(0) ||
                   model.upper_bound(Var(v)) != Model<SolverTraits>::INFTY;
        });
    if(ranges::distance(no_trivial_bound_vars) > 0) {
        os << "Bounds\n";
        for(auto && v : no_trivial_bound_vars) {
            if(model.lower_bound(Var(v)) == model.upper_bound(Var(v))) {
                os << "x" << v << " = " << model.upper_bound(Var(v)) << '\n';
                continue;
            }
            if(model.lower_bound(Var(v)) != scalar_t(0)) {
                if(model.lower_bound(Var(v)) ==
                   Model<SolverTraits>::MINUS_INFTY)
                    os << "-Inf <= ";
                else
                    os << model.lower_bound(Var(v)) << " <= ";
            }
            os << "x" << v;
            if(model.upper_bound(Var(v)) != Model<SolverTraits>::INFTY)
                os << " <= " << model.upper_bound(Var(v));
            os << '\n';
        }
    }
    return os << "End" << '\n';
}

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_HPP