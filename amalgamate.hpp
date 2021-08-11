/**
 * @file milppp.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef MILPPP_HPP
#define MILPPP_HPP

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

#ifndef MILPP_FUCTION_TRAITS_HPP
#define MILPP_FUCTION_TRAITS_HPP

template<typename... Args>
struct pack {};

template <typename T>
struct function_traits
    : public function_traits<decltype(&T::operator())>
{};
template <typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType(ClassType::*)(Args...) const> {
    using result_type = ReturnType;
    using arg_types = pack<Args...>;
};

#endif //MILPP_FUCTION_TRAITS_HPP
#ifndef MILPPP_STRONG_TYPES
#define MILPPP_STRONG_TYPES

class Var {
private:
    int _id;
public:
    constexpr Var(const Var & v) noexcept : _id(v.id()) {}
    explicit constexpr Var(const int & i) noexcept : _id(i) {}
    constexpr const int & id() const noexcept { return _id; }
    constexpr int & id() noexcept { return _id; }
};

class Constr {
private:
    int _id;
public:
    constexpr Constr(const Constr & c) noexcept : _id(c.id()) {}
    explicit constexpr Constr(const int & i) noexcept : _id(i) {}
    constexpr const int & id() const noexcept { return _id; }
    constexpr int & id() noexcept { return _id; }
};

#endif //MILP_BUILDER_STRONG_TYPES

/**
 * @file solver_builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief OSI_Builder class declaration
 * @version 0.1
 * @date 2021-08-4
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef MILPPP_EXPRESSION_ALGEBRA_HPP
#define MILPPP_EXPRESSION_ALGEBRA_HPP

#include <numeric>
#include <vector>

#include <range/v3/view/zip.hpp>
#include <range/v3/algorithm/for_each.hpp>

#ifndef MILPPP_LINEAR_EXPRESSION_HPP
#define MILPPP_LINEAR_EXPRESSION_HPP

#include <iostream>
#include <ostream>
#include <vector>

#include <range/v3/view/zip.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/algorithm/for_each.hpp>

struct LinearExpr {
    double constant;
    std::vector<int> vars;
    std::vector<double> coefs;

    LinearExpr()
        : constant(0) {}

    LinearExpr(LinearExpr&& e)
        : constant(e.constant)
        , vars(std::move(e.vars))
        , coefs(std::move(e.coefs)) { std::cout << "move LinearExpr" << std::endl; }
    LinearExpr(const LinearExpr & e)
        : constant(e.constant)
        , vars(e.vars)
        , coefs(e.coefs) { std::cout << "copy LinearExpr" << std::endl; }

    LinearExpr(int v, double c=1.0)
        : constant(0)
        , vars(1, v)
        , coefs(1, c) {}
    LinearExpr& add(double c) {
        constant += c;
        return *this;
    }
    LinearExpr& add(int v, double c=1.0) {
        vars.emplace_back(v);
        coefs.emplace_back(c);
        std::cout << "add LinearExpr" << std::endl; 
        return *this;
    }
    LinearExpr& simplify() {
        auto zip_view = ranges::view::zip(vars, coefs);
        ranges::sort(zip_view, [](auto p1, auto p2){ return p1.first < p2.first; }); 
    
        auto first = zip_view.begin();
        const auto end = zip_view.end();
        for(auto next = first+1; next != end; ++next) {
            if((*first).first != (*next).first) {
                if((*first).second != 0.0)
                    ++first;
                *first = *next;
                continue;
            }
            (*first).second += (*next).second;               
        }
        const std::size_t new_length = std::distance(zip_view.begin(), first+1);
        vars.resize(new_length);
        coefs.resize(new_length);
        return *this;
    }
};

inline std::ostream& operator<<(std::ostream& os,
                                const LinearExpr & e) {
    if(e.vars.size() > 0) {
        std::size_t i = 0;
        os << (e.coefs[i] < 0 ? "-" : "");
        if(std::abs(e.coefs[i])!=1)
           os << std::abs(e.coefs[i]) << "*";
        os << "x" << e.vars[i];
        for(++i; i < e.vars.size(); ++i) {
            os << (e.coefs[i] < 0 ? " - " : " + ");
            if(std::abs(e.coefs[i])!=1)
                os << std::abs(e.coefs[i]) << "*";
            os << "x" << e.vars[i];
        }
    }
    return os << (e.constant < 0 ? " - " : " + ") << std::abs(e.constant);
}

#endif //MILPPP_LINEAR_EXPRESSION_HPP
#ifndef MILPPP_LINEAR_CONSTRAINTS_HPP
#define MILPPP_LINEAR_CONSTRAINTS_HPP

#include <numeric>
#include <ostream>

#include <range/v3/algorithm/for_each.hpp>

enum InequalitySense { LESS=-1, EQUAL=0, GREATER=1 };

struct LinearIneqConstraint {
    InequalitySense sense;
    LinearExpr linear_expression;
    LinearIneqConstraint() = default;
    LinearIneqConstraint(InequalitySense s, LinearExpr && e)
        : sense(s)
        , linear_expression(std::move(e)) {};
    LinearIneqConstraint & simplify() {
        linear_expression.simplify();
        return *this;
    }
};

struct LinearRangeConstraint {
    double lower_bound, upper_bound;
    LinearExpr linear_expression;
    LinearRangeConstraint()
            : lower_bound{std::numeric_limits<double>::min()}
            , upper_bound{std::numeric_limits<double>::max()} {}
};

inline std::ostream& operator<<(std::ostream& os, 
                                const LinearIneqConstraint & constraint) {
    return os << constraint.linear_expression
              << (constraint.sense == LESS ? " <= 0" : " >= 0");
}
inline std::ostream& operator<<(std::ostream& os, 
                                const LinearRangeConstraint & constraint) {
    return os << constraint.lower_bound 
              << " <= " << constraint.linear_expression 
              << " <= " << constraint.upper_bound;
}

#endif //MILPPP_LINEAR_CONSTRAINTS_HPP

struct LinearTerm {
    double coef;
    int var;

    constexpr LinearTerm(LinearTerm&& t)
        : coef(t.coef)
        , var(t.var) { /*std::cout << "move ctor LinearTerm" << std::endl;*/ }
    constexpr LinearTerm(const LinearTerm & t)
        : coef(t.coef)
        , var(t.var) { /*std::cout << "copy ctor LinearTerm" << std::endl;*/ }

    constexpr LinearTerm(Var v, double c=1.0)
        : coef(c)
        , var(v.id()) { /*std::cout << "custom ctor LinearTerm" << std::endl;*/ }
    
    constexpr LinearTerm& operator*(double c) {
        coef *= c;
        return *this;
    }
    constexpr LinearTerm& operator-() {
        coef = -coef;
        return *this;
    }
};

constexpr LinearTerm operator*(Var v, double c) {
    LinearTerm t(v, c);
    return t;
}
constexpr LinearTerm operator-(Var v) {
    LinearTerm t(v, -1);
    return t;
}
constexpr LinearTerm operator*(double c, Var v) {
    LinearTerm t(v, c);
    return t;
}

inline LinearExpr& operator+(LinearExpr & e, double c) { 
    return e.add(c);
}
inline LinearExpr& operator+(LinearExpr&& e, double c) { 
    return e.add(c);
}
inline LinearExpr& operator+(LinearExpr & e, const LinearTerm & t) { 
    return e.add(t.var, t.coef);
}
inline LinearExpr& operator+(LinearExpr&& e, const LinearTerm & t) { 
    return e.add(t.var, t.coef);
}

inline LinearExpr operator+(const LinearTerm & t1, const LinearTerm & t2) {
    LinearExpr e(t1.var, t1.coef);
    e + t2;
    return e;
}
inline LinearExpr operator+(const LinearTerm & t, double c) {
    LinearExpr e(t.var, t.coef);
    e.add(c);
    return e;
}
inline LinearExpr operator+(double c, const LinearTerm & t) {
    LinearExpr e(t.var, t.coef);
    e.add(c);
    return e;
}

inline LinearExpr & operator-(LinearExpr & e1, const LinearExpr & e2) {
    e1.constant -= e2.constant;
    ranges::for_each(ranges::view::zip(e2.vars, e2.coefs), 
        [&e1](const auto p){ e1.add(p.first, -p.second); });
    return e1;
}

inline LinearIneqConstraint operator<=(LinearExpr && e1, const LinearExpr & e2) {
    e1 - e2;
    return LinearIneqConstraint(LESS, std::move(e1));
}
inline LinearIneqConstraint operator>=(LinearExpr && e1, const LinearExpr & e2) {
    e1 - e2;
    return LinearIneqConstraint(GREATER, std::move(e1));
}
inline LinearIneqConstraint operator<=(LinearExpr & e1, const LinearExpr & e2) {
    return std::move(e1) <= e2;
}
inline LinearIneqConstraint operator>=(LinearExpr & e1, const LinearExpr & e2) {
    return std::move(e1) >= e2;
}

#endif //MILPPP_EXPRESSION_ALGEBRA_HPP

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

template <typename SolverTraits>
class MILP_Builder {
public:
    static constexpr double MINUS_INFINITY = std::numeric_limits<double>::min();
    static constexpr double INFINITY = std::numeric_limits<double>::max();
    using OptSense = typename SolverTraits::OptSense;
    using ColType = typename SolverTraits::ColType;
    using ModelType = typename SolverTraits::ModelType;
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
               ColType type=ColType::CONTINUOUS) {
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
                 double lb=0.0, double ub=INFINITY, ColType type=ColType::CONTINUOUS) {
        return addVars(typename function_traits<T>::arg_types(),
                    count, std::forward<T>(id_lambda), coef, lb, ub, type);
    }

    std::size_t nbVars() const { return col_coef.size(); }
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

    std::size_t nbConstrs() const { return row_begins.size(); }
    double getConstrLB(Constr constr) const { return row_lb[constr.id()]; }
    double getConstrUB(Constr constr) const { return row_ub[constr.id()]; }
    std::size_t nbEntries() const { return vars.size(); }

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

    ModelType build() {
        ModelType model = SolverTraits::build(sense,
                              nbVars(), 
                              col_coef.data(),
                              col_lb.data(),
                              col_ub.data(),
                              col_type.data(),
                              nbConstrs(),
                              nbEntries(),
                              row_begins.data(),
                              vars.data(),
                              coefs.data(),
                              row_lb.data(),
                              row_ub.data());
        return model;
    }
};

template <typename T>
std::ostream & print_entries(std::ostream & os, const T & e) {
    auto it = e.begin();
    const auto end = e.end();
    if(it == end)
        return os;
    for(; it != end; ++it) {
        Var v = (*it).first;
        double coef = (*it).second;
        if(coef == 0.0)
            continue;
            
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? "-" : "");
        if(abs_coef != 1)
            os << abs_coef << " ";
        os << "x" << v.id();
        break;
    }
    for(++it; it != end; ++it) {
        Var v = (*it).first;
        double coef = (*it).second;
        if(coef == 0.0) continue;
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? " - " : " + ");
        if(abs_coef != 1)
            os << abs_coef << " ";
        os << "x" << v.id();
    }
    return os;
}

template <typename SolverTraits>
std::ostream& operator<<(std::ostream& os, const MILP_Builder<SolverTraits>& lp) {
    using MILP = MILP_Builder<SolverTraits>;
    os << (lp.getOptSense()==MILP::OptSense::MINIMIZE ? "Minimize" : "Maximize") << std::endl;
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
    auto interger_vars = ranges::filter_view(lp.variables(),
        [&lp](Var v){ return lp.getVarType(v)==MILP::ColType::INTEGER; }
    );
    if(ranges::distance(interger_vars) > 0) {
        os << "General" << std::endl;
        for(Var v : interger_vars) {
            os << " x" << v.id();
        }
        os << std::endl;
    }
    auto binary_vars = ranges::filter_view(lp.variables(),
        [&lp](Var v){ return lp.getVarType(v)==MILP::ColType::BINARY; }
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
                    || lp.getVarUB(v)!=MILP::INFINITY; }
    );
    if(ranges::distance(no_trivial_bound_vars) > 0) {
        os << "Bounds" << std::endl;
        for(Var v : no_trivial_bound_vars) {
            if(lp.getVarLB(v) == lp.getVarUB(v)) {
                os << "x" << v.id() << " = " << lp.getVarUB(v) << std::endl;
                continue;
            }
            if(lp.getVarLB(v) != 0.0) {
                if(lp.getVarLB(v) == MILP::MINUS_INFINITY) os << "-Inf <= ";
                else os << lp.getVarLB(v) << " <= ";
            }
            os << "x" << v.id();
            if(lp.getVarUB(v) != MILP::INFINITY)
                os << " <= " << lp.getVarUB(v);
            os << std::endl;
        }
    } 
    return os << "End" << std::endl;
}

#endif //MILPP_MILP_BUILDER_HPP

// #include "milppp/solver_traits/cbc_traits.hpp"
// #include "milppp/solver_traits/grb_traits.hpp"

#endif //MILPPP_HPP