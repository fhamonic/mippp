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
#ifndef MILPP_MILP_BUILDER_HPP
#define MILPP_MILP_BUILDER_HPP

#include <cassert>
#include <limits>
#include <ostream>
#include <vector>

#include <range/v3/view/filter.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include "milppp/function_traits.hpp"
#include "milppp/strong_types.hpp"

#include "milppp/linear_expression_algebra.hpp"


class IneqConstraintHandler {
private:
    std::vector<int> & vars;
    std::vector<double> & coefs;
    double & bound;
public:
    IneqConstraintHandler(std::vector<int> & vars, 
                        std::vector<double> & coefs,
                        double & b)
        : vars(vars)
        , coefs(coefs)
        , bound(b) {}
private:
    void lhs_impl(double c) {
        bound -= c;
    }
    void lhs_impl(const LinearTerm & t) {
        vars.push_back(t.var);
        coefs.push_back(t.coef);
    }
public:
    template<typename... Terms>
    IneqConstraintHandler & lhs(Terms&&... terms) {
        (lhs_impl(terms), ...);
        return *this;
    }
private:
    void rhs_impl(double c) {
        bound += c;
    }
    void rhs_impl(const LinearTerm & t) {
        vars.push_back(t.var);
        coefs.push_back(-t.coef);
    }
public:
    template<typename... Terms>
    IneqConstraintHandler & rhs(Terms&&... terms) {
        (rhs_impl(terms), ...);
        return *this;
    }
};

class RangeConstraintHandler {
private:
    std::vector<int> & vars;
    std::vector<double> & coefs;
public:
    RangeConstraintHandler(std::vector<int> & vars, 
                        std::vector<double> & coefs)
        : vars(vars)
        , coefs(coefs) {}
private:
    void op_impl(const LinearTerm & t) {
        vars.push_back(t.var);
        coefs.push_back(t.coef);
    }
public:
    template<typename... Terms>
    RangeConstraintHandler & operator()(Terms&&... terms) {
        (op_impl(terms), ...);
        return *this;
    }
};


class MILP_Builder {
public:
    static constexpr double MINUS_INFINITY = std::numeric_limits<double>::min();
    static constexpr double INFINITY = std::numeric_limits<double>::max();
    enum OptSense { MINIMIZE=-1, MAXIMIZE=1 };
    enum ColType { CONTINUOUS = 0, INTEGER = 1, BINARY = 2 };
private:
    std::vector<double> col_coef;
    std::vector<double> col_lb;
    std::vector<double> col_ub;
    std::vector<ColType> col_type;

    std::vector<int> vars;
    std::vector<double> coefs;

    std::vector<int> row_begins;
    std::vector<double> row_lb;
    std::vector<double> row_ub;

    OptSense sense;
public:
    MILP_Builder(OptSense sense)
        : sense(sense) {}

    OptSense getOptSense() const { return sense; }
    MILP_Builder & setOptSense(OptSense s) {
        sense = s;
        return *this;
    }

    Var addVar(double coef=0.0, double lb=0.0, double ub=INFINITY,
               ColType type=CONTINUOUS) {
        col_coef.push_back(coef);
        col_lb.push_back(lb);
        col_ub.push_back(ub);
        col_type.push_back(type);
        return Var(nbVars()-1);
    }
private:
    template <typename T, typename ... Args>
    auto addVars(pack<Args...>, int count, T&& id_lambda, 
                                double coef, double lb,
                                double ub, ColType type) {
        const int offset = nbVars();
        const int new_size = offset + count;
        col_coef.resize(new_size, coef);
        col_lb.resize(new_size, lb);
        col_ub.resize(new_size, ub);
        col_type.resize(new_size, type);
        return [offset, count, id_lambda = std::forward<T>(id_lambda)]
        (Args... args) {
            const int id = id_lambda(args...);
            assert(0 <= id && id < count);
            return Var(offset + id);
        };
    }
public:
    template <typename T>
    auto addVars(int count, T&& id_lambda, double coef=0.0, 
                 double lb=0.0, double ub=INFINITY, ColType type=CONTINUOUS) {
        return addVars(typename function_traits<T>::arg_types(),
                    count, std::forward<T>(id_lambda), coef, lb, ub, type);
    }

    size_t nbVars() const { return col_coef.size(); }
    double getObjCoef(Var v) const { return col_coef[v.id()]; }
    MILP_Builder & setObjCoef(Var v, double coef) {
        col_coef[v.id()] = coef;
        return *this;
    }
    double getVarLB(Var v) const { return col_lb[v.id()]; }
    double getVarUB(Var v) const { return col_ub[v.id()]; }
    MILP_Builder & setBounds(Var v, double lb, double ub) {
        col_lb[v.id()] = lb;
        col_ub[v.id()] = ub;
        return *this;
    }
    ColType getVarType(Var v) const { return col_type[v.id()]; }
    MILP_Builder & setType(Var v, ColType type) {
        col_type[v.id()] = type;
        return *this;
    }

    IneqConstraintHandler addLessThanConstr() {
        row_begins.push_back(vars.size());
        row_lb.push_back(MINUS_INFINITY);
        row_ub.push_back(0);
        IneqConstraintHandler handler(vars, coefs, row_ub.back());
        return handler;
    }
    IneqConstraintHandler addGreaterThanConstr() {
        row_begins.push_back(vars.size());
        row_lb.push_back(0);
        row_ub.push_back(INFINITY);
        IneqConstraintHandler handler(vars, coefs, row_lb.back());
        return handler;
    }
    RangeConstraintHandler addRangeConstr(double lb, double ub) {
        row_begins.push_back(vars.size());
        row_lb.push_back(lb);
        row_ub.push_back(ub);
        RangeConstraintHandler handler(vars, coefs);
        return handler;
    }

    size_t nbConstrs() const { return row_begins.size(); }
    double getConstrLB(Constr constr) const { return row_lb[constr.id()]; }
    double getConstrUB(Constr constr) const { return row_ub[constr.id()]; }


    auto variables() const {
        auto v = ranges::views::transform(
                    ranges::iota_view<int,int>(0, nbVars()),
                    [](int v) { return Var(v); });
        return v;
    }
    auto constraints() const {
        auto v = ranges::views::transform(
                    ranges::iota_view<int,int>(0, nbConstrs()),
                    [](int v) { return Constr(v); });
        return v;
    }
    auto objective() const {
        auto z = ranges::view::zip(
            variables(), 
            col_coef);
        return z;
    }
    const auto entries() const {
        auto z = ranges::view::zip(vars, coefs);
        return z;
    }
    const auto entries(Constr constr) const {
        const int offset = row_begins[constr.id()];
        const int end = (constr.id()+1 < static_cast<int>(nbConstrs())
                        ? row_begins[constr.id()+1] : vars.size());
        auto sub_vars = ranges::views::transform(
            ranges::subrange(vars.begin()+offset,vars.begin()+end),
            [](int v) { return Var(v); });
        auto sub_coefs = ranges::subrange(coefs.begin()+offset,
                                          coefs.begin()+end);
        auto z = ranges::view::zip(sub_vars, sub_coefs);
        return z;
    }
};

template <typename T>
std::ostream & print_entries(std::ostream & os, const T & e) {
    auto it = e.begin();
    const auto end = e.end();
    if(it == end)
        return os;
    
    Var v = (*it).first;
    double coef = (*it).second;
    if(coef != 0.0) {
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? "-" : "");
        if(abs_coef != 1)
            os << abs_coef << " ";
        os << "x" << v.id();
    }
    for(++it; it != end; ++it) {
        v = (*it).first;
        coef = (*it).second;
        if(coef == 0.0) continue;
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? " - " : " + ");
        if(abs_coef != 1)
            os << abs_coef << " ";
        os << "x" << v.id();
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const MILP_Builder& lp) {
    os << (lp.getOptSense()==MILP_Builder::MINIMIZE ? "Minimize" : "Maximize") << std::endl;
    print_entries(os, lp.objective());
    os << std::endl << "Subject To" << std::endl;
    for(Constr constr : lp.constraints()) {
        const double lb = lp.getConstrLB(constr);
        const double ub = lp.getConstrUB(constr);
        if(ub != MILP_Builder::INFINITY) {
            os << "R" << constr.id() << ": ";        
            print_entries(os, lp.entries(constr));
            os << " <= " << ub << std::endl;
        }
        if(lb != MILP_Builder::MINUS_INFINITY) {
            os << "R" << constr.id() << "_low: ";        
            print_entries(os, lp.entries(constr));
            os << " >= " << lb << std::endl;
        }
    }
    auto interger_vars = ranges::filter_view(lp.variables(),
        [&lp](Var v){ return lp.getVarType(v)==MILP_Builder::INTEGER; }
    );
    if(ranges::distance(interger_vars) > 0) {
        os << "General" << std::endl;
        for(Var v : interger_vars) {
            os << " x" << v.id();
        }
        os << std::endl;
    }
    auto binary_vars = ranges::filter_view(lp.variables(),
        [&lp](Var v){ return lp.getVarType(v)==MILP_Builder::BINARY; }
    );
    if(ranges::distance(binary_vars) > 0) {
        os << "Binary" << std::endl;
        for(Var v : binary_vars) {
            os << " x" << v.id();
        }
        os << std::endl;
    }
    auto no_trivial_bound_vars = ranges::filter_view(lp.variables(),
        [&lp](Var v){ return lp.getVarLB(v)!=0.0 
                    || lp.getVarUB(v)!=MILP_Builder::INFINITY; }
    );
    if(ranges::distance(no_trivial_bound_vars) > 0) {
        os << "Bounds" << std::endl;
        for(Var v : no_trivial_bound_vars) {
            if(lp.getVarLB(v) == lp.getVarUB(v)) {
                os << "x" << v.id() << " = " << lp.getVarUB(v) << std::endl;
                continue;
            }
            if(lp.getVarLB(v) != 0.0) {
                if(lp.getVarLB(v) == MILP_Builder::MINUS_INFINITY) os << "-Inf <= ";
                else os << lp.getVarLB(v) << " <= ";
            }
            os << "x" << v.id();
            if(lp.getVarUB(v) != MILP_Builder::INFINITY)
                os << " <= " << lp.getVarUB(v);
            os << std::endl;
        }
    } 
    return os << "End" << std::endl;
}

#endif //MILPP_MILP_BUILDER_HPP