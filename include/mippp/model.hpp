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

#include <range/v3/view/iota.hpp>
#include <range/v3/view/single.hpp>

#include "mippp/expressions/linear_constraint.hpp"
#include "mippp/expressions/linear_expression.hpp"
#include "mippp/variable.hpp"

namespace fhamonic {
namespace mippp {

template <typename SolverTraits>
class Model {
public:
    static constexpr double MINUS_INFINITY = std::numeric_limits<double>::min();
    static constexpr double INFINITY = std::numeric_limits<double>::max();

    using OptSense = typename SolverTraits::OptSense;
    using ColType = typename SolverTraits::ColType;
    using ModelType = typename SolverTraits::ModelType;

    using var_id_t = int;
    using scalar_t = double;

    using Var = variable<var_id_t, scalar_t>;

private:
    std::vector<scalar_t> _col_coef;
    std::vector<scalar_t> _col_lb;
    std::vector<scalar_t> _col_ub;
    std::vector<ColType> _col_type;

    std::vector<var_id_t> _vars;
    std::vector<scalar_t> _coefs;

    std::vector<std::size_t> _row_begins;
    std::vector<scalar_t> _row_lb;
    std::vector<scalar_t> _row_ub;

    OptSense _sense;

public:
    Model(OptSense sense) : _sense(sense) {}

    OptSense opt_sense() const noexcept { return _sense; }
    Model & opt_sense(OptSense s) noexcept {
        _sense = s;
        return *this;
    }

    struct var_options {
        scalar_t obj_coef = scalar_t{0};
        scalar_t lower_bound = scalar_t{0};
        scalar_t upper_bound = INFINITY;
        ColType type = ColType::CONTINUOUS;
    };

    Var add_var(var_options options = {}) {
        _col_coef.push_back(options.obj_coef);
        _col_lb.push_back(options.lower_bound);
        _col_ub.push_back(options.upper_bound);
        _col_type.push_back(options.type);
        return Var(static_cast<var_id_t>(nb_variables() - 1));
    }

private:
    template <typename T, typename... Args>
    auto add_vars(pack<Args...>, std::size_t count, T && id_lambda,
                  var_options options = {}) {
        const std::size_t offset = nb_variables();
        const std::size_t new_size = offset + count;
        _col_coef.resize(new_size, options.obj_coef);
        _col_lb.resize(new_size, options.lower_bound);
        _col_ub.resize(new_size, options.upper_bound);
        _col_type.resize(new_size, options.type);
        return [offset, count,
                id_lambda = std::forward<T>(id_lambda)](Args... args) {
            const var_id_t id = id_lambda(args...);
            assert(0 <= id && id < count);
            return Var(var_id_t{offset} + id);
        };
    }

public:
    template <typename T>
    auto add_vars(int count, T && id_lambda, var_options options = {}) {
        return add_vars(typename function_traits<T>::arg_types(), count,
                        std::forward<T>(id_lambda), options);
    }

    std::size_t nb_variables() const { return _col_coef.size(); }
    std::size_t nb_constraints() const { return _row_begins.size(); }
    std::size_t nb_entries() const { return _vars.size(); }

    // Variables
    double & obj_coef(Var v) const {
        return _col_coef[static_cast<std::size_t>(v.id())];
    }
    double & lower_bound(Var v) const {
        return _col_lb[static_cast<std::size_t>(v.id())];
    }
    double upper_bound(Var v) const {
        return _col_ub[static_cast<std::size_t>(v.id())];
    }
    Model & set_bounds(Var v, double lb, double ub) {
        _col_lb[static_cast<std::size_t>(v.id())] = lb;
        _col_ub[static_cast<std::size_t>(v.id())] = ub;
        return *this;
    }
    ColType & type(Var v) const {
        return _col_type[static_cast<std::size_t>(v.id())];
    }

    // Views
    auto variables() const {
        return ranges::iota_view<var_id_t, var_id_t>(
            var_id_t{0}, static_cast<var_id_t>(nb_variables()));
    }
    auto objective() const noexcept {
        return linear_expression_view(variables(), std::cref(obj_coef));
    }
    auto constraint(std::size_t constraint_id) const {
        assert(0 <= constraint_id && constraint_id < nb_constraints());
        const std::size_t row_begin = _row_begins[constraint_id];
        const std::size_t row_end = (constraint_id < nb_constraints() - 1)
                                        ? _row_begins[constraint_id + 1]
                                        : nb_entries();
        return linear_constraint(
            linear_expression(
                ranges::subrange(_vars + row_begin, _vars + row_end),
                ranges::subrange(_coefs + row_end, _coefs + row_end)),
            _row_lb[constraint_id], _row_ub[constraint_id]);
    }

    // Constraints

    ModelType build() {
        // ModelType model = SolverTraits::build(
        //     _sense, static_cast<int>(nbVars()), _col_coef.data(),
        //     _col_lb.data(), _col_ub.data(), _col_type.data(),
        //     static_cast<int>(nbConstrs()), static_cast<int>(nbEntries()),
        //     _row_begins.data(), _vars.data(), _coefs.data(), _row_lb.data(),
        //     _row_ub.data());
        // return model;
    }
};

template <typename T>
std::ostream & print_entries(std::ostream & os, const T & e) {
    // auto it = e.begin();
    // const auto end = e.end();
    // if(it == end) return os;
    // for(; it != end; ++it) {
    //     Var v = (*it).first;
    //     double coef = (*it).second;
    //     if(coef == 0.0) continue;

    //     const double abs_coef = std::abs(coef);
    //     os << (coef < 0 ? "-" : "");
    //     if(abs_coef != 1) os << abs_coef << " ";
    //     os << "x" << v.id();
    //     break;
    // }
    // for(++it; it != end; ++it) {
    //     Var v = (*it).first;
    //     double coef = (*it).second;
    //     if(coef == 0.0) continue;
    //     const double abs_coef = std::abs(coef);
    //     os << (coef < 0 ? " - " : " + ");
    //     if(abs_coef != 1) os << abs_coef << " ";
    //     os << "x" << v.id();
    // }
    // return os;
}

// template <typename SolverTraits>
// std::ostream & operator<<(std::ostream & os,
//                           const MILP_Builder<SolverTraits> & lp) {
//     using MILP = MILP_Builder<SolverTraits>;
//     os << (lp.getOptSense() == MILP::OptSense::MINIMIZE ? "Minimize"
//                                                         : "Maximize")
//        << std::endl;
//     print_entries(os, lp.objective());
//     os << std::endl << "Subject To" << std::endl;
//     for(Constr constr : lp.constraints()) {
//         const double lb = lp.getConstrLB(constr);
//         const double ub = lp.getConstrUB(constr);
//         if(ub != MILP::INFINITY) {
//             os << "R" << constr.id() << ": ";
//             print_entries(os, lp.entries(constr));
//             os << " <= " << ub << std::endl;
//         }
//         if(lb != MILP::MINUS_INFINITY) {
//             os << "R" << constr.id() << "_low: ";
//             print_entries(os, lp.entries(constr));
//             os << " >= " << lb << std::endl;
//         }
//     }
//     auto interger_vars = ranges::filter_view(lp.variables(), [&lp](Var v) {
//         return lp.getVarType(v) == MILP::ColType::INTEGER;
//     });
//     if(ranges::distance(interger_vars) > 0) {
//         os << "General" << std::endl;
//         for(Var v : interger_vars) {
//             os << " x" << v.id();
//         }
//         os << std::endl;
//     }
//     auto binary_vars = ranges::filter_view(lp.variables(), [&lp](Var v) {
//         return lp.getVarType(v) == MILP::ColType::BINARY;
//     });
//     if(ranges::distance(binary_vars) > 0) {
//         os << "Binary" << std::endl;
//         for(Var v : binary_vars) {
//             os << " x" << v.id();
//         }
//         os << std::endl;
//     }
//     auto no_trivial_bound_vars =
//         ranges::filter_view(lp.variables(), [&lp](Var v) {
//             return lp.getVarLB(v) != 0.0 || lp.getVarUB(v) != MILP::INFINITY;
//         });
//     if(ranges::distance(no_trivial_bound_vars) > 0) {
//         os << "Bounds" << std::endl;
//         for(Var v : no_trivial_bound_vars) {
//             if(lp.getVarLB(v) == lp.getVarUB(v)) {
//                 os << "x" << v.id() << " = " << lp.getVarUB(v) << std::endl;
//                 continue;
//             }
//             if(lp.getVarLB(v) != 0.0) {
//                 if(lp.getVarLB(v) == MILP::MINUS_INFINITY)
//                     os << "-Inf <= ";
//                 else
//                     os << lp.getVarLB(v) << " <= ";
//             }
//             os << "x" << v.id();
//             if(lp.getVarUB(v) != MILP::INFINITY) os << " <= " <<
//             lp.getVarUB(v); os << std::endl;
//         }
//     }
//     return os << "End" << std::endl;
// }

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_MODEL_HPP