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
#ifndef MILPPP_MILP_BUILDER_HPP
#define MILPPP_MILP_BUILDER_HPP

#include <cassert>
#include <limits>
#include <ostream>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>
#include <vector>

#include "milppp/function_traits.hpp"
#include "milppp/linear_expression_algebra.hpp"
#include "milppp/strong_types.hpp"

namespace fhamonic {
namespace milppp {

class IneqConstraintHandler {
private:
    std::vector<int> & _vars;
    std::vector<double> & _coefs;
    double & bound;

public:
    IneqConstraintHandler(std::vector<int> & vars, std::vector<double> & coefs,
                          double & b)
        : _vars(vars), _coefs(coefs), bound(b) {}

private:
    void lhs_impl(double c) { bound -= c; }
    void lhs_impl(const LinearTerm & t) {
        _vars.push_back(t.var);
        _coefs.push_back(t.coef);
    }

public:
    template <typename... Terms>
    IneqConstraintHandler & lhs(Terms &&... terms) {
        (lhs_impl(terms), ...);
        return *this;
    }

private:
    void rhs_impl(double c) { bound += c; }
    void rhs_impl(const LinearTerm & t) {
        _vars.push_back(t.var);
        _coefs.push_back(-t.coef);
    }

public:
    template <typename... Terms>
    IneqConstraintHandler & rhs(Terms &&... terms) {
        (rhs_impl(terms), ...);
        return *this;
    }
};

class RangeConstraintHandler {
private:
    std::vector<int> & _vars;
    std::vector<double> & _coefs;

public:
    RangeConstraintHandler(std::vector<int> & vars, std::vector<double> & coefs)
        : _vars(vars), _coefs(coefs) {}

private:
    void op_impl(const LinearTerm & t) {
        _vars.push_back(t.var);
        _coefs.push_back(t.coef);
    }

public:
    template <typename... Terms>
    RangeConstraintHandler & operator()(Terms &&... terms) {
        (op_impl(terms), ...);
        return *this;
    }
};

template <typename SolverTraits>
class MILP_Builder {
public:
    static constexpr double MINUS_INFINITY = std::numeric_limits<double>::min();
    static constexpr double INFINITY = std::numeric_limits<double>::max();
    using OptSense = typename SolverTraits::OptSense;
    using ColType = typename SolverTraits::ColType;
    using ModelType = typename SolverTraits::ModelType;

private:
    std::vector<double> _col_coef;
    std::vector<double> _col_lb;
    std::vector<double> _col_ub;
    std::vector<ColType> _col_type;

    std::vector<int> _vars;
    std::vector<double> _coefs;

    std::vector<int> _row_begins;
    std::vector<double> _row_lb;
    std::vector<double> _row_ub;

    OptSense _sense;

public:
    MILP_Builder(OptSense sense) : _sense(sense) {}

    OptSense getOptSense() const { return _sense; }
    MILP_Builder & setOptSense(OptSense s) {
        _sense = s;
        return *this;
    }

    Var addVar(double coef = 0.0, double lb = 0.0, double ub = INFINITY,
               ColType type = ColType::CONTINUOUS) {
        _col_coef.push_back(coef);
        _col_lb.push_back(lb);
        _col_ub.push_back(ub);
        _col_type.push_back(type);
        return Var(static_cast<int>(nbVars() - 1));
    }

private:
    template <typename T, typename... Args>
    auto addVars(pack<Args...>, std::size_t count, T && id_lambda, double coef,
                 double lb, double ub, ColType type) {
        const std::size_t offset = nbVars();
        const std::size_t new_size = offset + count;
        _col_coef.resize(new_size, coef);
        _col_lb.resize(new_size, lb);
        _col_ub.resize(new_size, ub);
        _col_type.resize(new_size, type);
        return [offset, count,
                id_lambda = std::forward<T>(id_lambda)](Args... args) {
            const int id = id_lambda(args...);
            assert(0 <= id && id < count);
            return Var(offset + id);
        };
    }

public:
    template <typename T>
    auto addVars(int count, T && id_lambda, double coef = 0.0, double lb = 0.0,
                 double ub = INFINITY, ColType type = ColType::CONTINUOUS) {
        return addVars(typename function_traits<T>::arg_types(), count,
                       std::forward<T>(id_lambda), coef, lb, ub, type);
    }

    std::size_t nbVars() const { return _col_coef.size(); }
    double getObjCoef(Var v) const {
        return _col_coef[static_cast<std::size_t>(v.id())];
    }
    MILP_Builder & setObjCoef(Var v, double coef) {
        _col_coef[static_cast<std::size_t>(v.id())] = coef;
        return *this;
    }
    double getVarLB(Var v) const {
        return _col_lb[static_cast<std::size_t>(v.id())];
    }
    double getVarUB(Var v) const {
        return _col_ub[static_cast<std::size_t>(v.id())];
    }
    MILP_Builder & setBounds(Var v, double lb, double ub) {
        _col_lb[static_cast<std::size_t>(v.id())] = lb;
        _col_ub[static_cast<std::size_t>(v.id())] = ub;
        return *this;
    }
    ColType getVarType(Var v) const {
        return _col_type[static_cast<std::size_t>(v.id())];
    }
    MILP_Builder & setType(Var v, ColType type) {
        _col_type[static_cast<std::size_t>(v.id())] = type;
        return *this;
    }

    IneqConstraintHandler addLessThanConstr() {
        _row_begins.push_back(static_cast<int>(nbEntries()));
        _row_lb.push_back(MINUS_INFINITY);
        _row_ub.push_back(0);
        IneqConstraintHandler handler(_vars, _coefs, _row_ub.back());
        return handler;
    }
    IneqConstraintHandler addGreaterThanConstr() {
        _row_begins.push_back(static_cast<int>(nbEntries()));
        _row_lb.push_back(0);
        _row_ub.push_back(INFINITY);
        IneqConstraintHandler handler(_vars, _coefs, _row_lb.back());
        return handler;
    }
    RangeConstraintHandler addRangeConstr(double lb, double ub) {
        _row_begins.push_back(static_cast<int>(nbEntries()));
        _row_lb.push_back(lb);
        _row_ub.push_back(ub);
        RangeConstraintHandler handler(_vars, _coefs);
        return handler;
    }

    std::size_t nbConstrs() const { return _row_begins.size(); }
    double getConstrLB(Constr constr) const {
        return _row_lb[static_cast<std::size_t>(constr.id())];
    }
    double getConstrUB(Constr constr) const {
        return _row_ub[static_cast<std::size_t>(constr.id())];
    }
    std::size_t nbEntries() const { return _vars.size(); }

    auto variables() const {
        auto view = ranges::views::transform(
            ranges::iota_view<int, int>(0, static_cast<int>(nbVars())),
            [](int var_id) { return Var(var_id); });
        return view;
    }
    auto constraints() const {
        auto view = ranges::views::transform(
            ranges::iota_view<int, int>(0, static_cast<int>(nbConstrs())),
            [](int constr_id) { return Constr(constr_id); });
        return view;
    }
    auto objective() const {
        auto view = ranges::view::zip(variables(), _col_coef);
        return view;
    }
    const auto entries() const {
        auto view = ranges::view::zip(_vars, _coefs);
        return view;
    }
    const auto entries(Constr constr) const {
        const std::size_t offset = static_cast<std::size_t>(
            _row_begins[static_cast<std::size_t>(constr.id())]);
        const std::size_t end =
            (constr.id() + 1 < static_cast<int>(nbConstrs())
                 ? static_cast<std::size_t>(
                       _row_begins[static_cast<std::size_t>(constr.id()) + 1])
                 : nbEntries());
        auto sub_vars = ranges::views::transform(
            ranges::subrange(
                _vars.begin() + static_cast<std::ptrdiff_t>(offset),
                _vars.begin() + static_cast<std::ptrdiff_t>(end)),
            [](int var_id) { return Var(var_id); });
        auto sub_coefs = ranges::subrange(
            _coefs.begin() + static_cast<std::ptrdiff_t>(offset),
            _coefs.begin() + static_cast<std::ptrdiff_t>(end));
        auto view = ranges::view::zip(sub_vars, sub_coefs);
        return view;
    }

    ModelType build() {
        ModelType model = SolverTraits::build(
            _sense, static_cast<int>(nbVars()), _col_coef.data(),
            _col_lb.data(), _col_ub.data(), _col_type.data(),
            static_cast<int>(nbConstrs()), static_cast<int>(nbEntries()),
            _row_begins.data(), _vars.data(), _coefs.data(), _row_lb.data(),
            _row_ub.data());
        return model;
    }
};

template <typename T>
std::ostream & print_entries(std::ostream & os, const T & e) {
    auto it = e.begin();
    const auto end = e.end();
    if(it == end) return os;
    for(; it != end; ++it) {
        Var v = (*it).first;
        double coef = (*it).second;
        if(coef == 0.0) continue;

        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? "-" : "");
        if(abs_coef != 1) os << abs_coef << " ";
        os << "x" << v.id();
        break;
    }
    for(++it; it != end; ++it) {
        Var v = (*it).first;
        double coef = (*it).second;
        if(coef == 0.0) continue;
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? " - " : " + ");
        if(abs_coef != 1) os << abs_coef << " ";
        os << "x" << v.id();
    }
    return os;
}

template <typename SolverTraits>
std::ostream & operator<<(std::ostream & os,
                          const MILP_Builder<SolverTraits> & lp) {
    using MILP = MILP_Builder<SolverTraits>;
    os << (lp.getOptSense() == MILP::OptSense::MINIMIZE ? "Minimize"
                                                        : "Maximize")
       << std::endl;
    print_entries(os, lp.objective());
    os << std::endl << "Subject To" << std::endl;
    for(Constr constr : lp.constraints()) {
        const double lb = lp.getConstrLB(constr);
        const double ub = lp.getConstrUB(constr);
        if(ub != MILP::INFINITY) {
            os << "R" << constr.id() << ": ";
            print_entries(os, lp.entries(constr));
            os << " <= " << ub << std::endl;
        }
        if(lb != MILP::MINUS_INFINITY) {
            os << "R" << constr.id() << "_low: ";
            print_entries(os, lp.entries(constr));
            os << " >= " << lb << std::endl;
        }
    }
    auto interger_vars = ranges::filter_view(lp.variables(), [&lp](Var v) {
        return lp.getVarType(v) == MILP::ColType::INTEGER;
    });
    if(ranges::distance(interger_vars) > 0) {
        os << "General" << std::endl;
        for(Var v : interger_vars) {
            os << " x" << v.id();
        }
        os << std::endl;
    }
    auto binary_vars = ranges::filter_view(lp.variables(), [&lp](Var v) {
        return lp.getVarType(v) == MILP::ColType::BINARY;
    });
    if(ranges::distance(binary_vars) > 0) {
        os << "Binary" << std::endl;
        for(Var v : binary_vars) {
            os << " x" << v.id();
        }
        os << std::endl;
    }
    auto no_trivial_bound_vars =
        ranges::filter_view(lp.variables(), [&lp](Var v) {
            return lp.getVarLB(v) != 0.0 || lp.getVarUB(v) != MILP::INFINITY;
        });
    if(ranges::distance(no_trivial_bound_vars) > 0) {
        os << "Bounds" << std::endl;
        for(Var v : no_trivial_bound_vars) {
            if(lp.getVarLB(v) == lp.getVarUB(v)) {
                os << "x" << v.id() << " = " << lp.getVarUB(v) << std::endl;
                continue;
            }
            if(lp.getVarLB(v) != 0.0) {
                if(lp.getVarLB(v) == MILP::MINUS_INFINITY)
                    os << "-Inf <= ";
                else
                    os << lp.getVarLB(v) << " <= ";
            }
            os << "x" << v.id();
            if(lp.getVarUB(v) != MILP::INFINITY) os << " <= " << lp.getVarUB(v);
            os << std::endl;
        }
    }
    return os << "End" << std::endl;
}

}  // namespace milppp
}  // namespace fhamonic

#endif  // MILPPP_MILP_BUILDER_HPP