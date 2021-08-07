/**
 * @file lp_builder.hpp
 * @author François Hamonic (francois.hamonic@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef LP_BUILDER_HPP
#define LP_BUILDER_HPP

#include <cassert>
#include <limits>
#include <ostream>
#include <vector>

#include <range/v3/view/iota.hpp>
#include <range/v3/view/zip.hpp>

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


class IneqConstraintHandler {
private:
    std::vector<int> & vars;
    std::vector<double> & coefs;
    double & upper_bound;
    int hs_coef;
public:
    IneqConstraintHandler(std::vector<int> & vars, 
                        std::vector<double> & coefs,
                        double & ub)
        : vars(vars)
        , coefs(coefs)
        , upper_bound(ub)
        , hs_coef(1) {}

    IneqConstraintHandler & operator()(double c) {
        upper_bound -= hs_coef * c;
        return *this;
    }
    IneqConstraintHandler & operator()(int v, double c=1.0) {
        vars.push_back(v);
        coefs.push_back(hs_coef * c);
        return *this;
    }
    IneqConstraintHandler & less() {
        assert(hs_coef != -1);
        hs_coef = -1;
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

    RangeConstraintHandler & operator()(int v, double c=1.0) {
        vars.push_back(v);
        coefs.push_back(c);
        return *this;
    }
};


class LP_Builder {
public:
    static constexpr double MINUS_INFINITY = std::numeric_limits<double>::min();
    static constexpr double INFINITY = std::numeric_limits<double>::max();
    enum OptSense { MINIMIZE=-1, MAXIMIZE=1 };
    enum ColType { CONTINUOUS = 0, INTEGRAL = 1 };
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
    LP_Builder(OptSense sense)
        : sense(sense) {}

    OptSense getOptSense() const { return sense; }
    LP_Builder & setOptSense(OptSense s) {
        sense = s;
        return *this;
    }

    int addVar(double coef=0.0, double lb=0.0, double ub=INFINITY,
               ColType type=CONTINUOUS) {
        col_coef.push_back(coef);
        col_lb.push_back(lb);
        col_ub.push_back(ub);
        col_type.push_back(type);
        return nbVars()-1;
    }
private:
    template <typename T, typename ... Args>
    auto addVars(pack<Args...>, int count, T id_lambda, 
                                double coef, double lb,
                                double ub, ColType type) {
        const int offset = nbVars();
        const int new_size = offset + count;
        col_coef.resize(new_size, coef);
        col_lb.resize(new_size, lb);
        col_ub.resize(new_size, ub);
        col_type.resize(new_size, type);
        return [offset, count, id_lambda] (Args... args) {
            const int id = id_lambda(args...);
            assert(0 <= id && id < count);
            return offset + id;
        };
    }
public:
    template <typename T>
    auto addVars(int count, T id_lambda, double coef=0.0, 
                 double lb=0.0, double ub=INFINITY, ColType type=CONTINUOUS) {
        return addVars(typename function_traits<T>::arg_types(),
                    count, id_lambda, coef, lb, ub, type);
    }

    size_t nbVars() const { return col_coef.size(); }
    double getObjCoef(int var_id) const { return col_coef[var_id]; }
    LP_Builder & setObjCoef(int var_id, double coef) {
        col_coef[var_id] = coef;
        return *this;
    }
    double getVarLB(int var_id) const { return col_lb[var_id]; }
    double getVarUB(int var_id) const { return col_ub[var_id]; }
    LP_Builder & setBounds(int var_id, double lb, double ub) {
        col_lb[var_id] = lb;
        col_ub[var_id] = ub;
        return *this;
    }
    ColType getVarType(int var_id) const { return col_type[var_id]; }
    LP_Builder & setType(int var_id, ColType type) {
        col_type[var_id] = type;
        return *this;
    }

    IneqConstraintHandler addIneqConstr() {
        row_begins.push_back(vars.size());
        row_lb.push_back(MINUS_INFINITY);
        row_ub.push_back(0);

        IneqConstraintHandler handler(vars, coefs, row_ub.back());
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
    double getConstrLB(int constr_id) const { return row_lb[constr_id]; }
    double getConstrUB(int constr_id) const { return row_ub[constr_id]; }


    auto variables() const {
        auto v = ranges::iota_view<int,int>(0, nbVars());
        return v;
    }
    auto constraints() const {
        auto v = ranges::iota_view<int,int>(0, nbConstrs());
        return v;
    }
    auto objective() const {
        auto z = ranges::view::zip(
            ranges::iota_view<int,int>(0, nbVars()), 
            col_coef);
        return z;
    }
    const auto entries() const {
        auto z = ranges::view::zip(vars, coefs);
        return z;
    }
    const auto entries(int constr_id) const {
        const int offset = row_begins[constr_id];
        const int end = (constr_id+1 < static_cast<int>(nbConstrs())
                        ? row_begins[constr_id+1] : vars.size());
        auto sub_vars = ranges::subrange(vars.begin()+offset, 
                                         vars.begin()+end);
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
    
    int var_id = (*it).first;
    double coef = (*it).second;
    if(coef != 0.0) {
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? "-" : "");
        if(abs_coef != 1)
            os << abs_coef << " ";
        os << "x" << var_id;
    }
    for(++it; it != end; ++it) {
        var_id = (*it).first;
        coef = (*it).second;
        if(coef == 0.0) continue;
        const double abs_coef = std::abs(coef);
        os << (coef < 0 ? " - " : " + ");
        if(abs_coef != 1)
            os << abs_coef << " ";
        os << "x" << var_id;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const LP_Builder& lp) {
    os << (lp.getOptSense()==LP_Builder::MINIMIZE ? "Minimize" : "Maximize") << std::endl;
    os << "OBJROW:";
    print_entries(os, lp.objective());
    os << std::endl << "Subject To" << std::endl;
    for(int constr_id : lp.constraints()) {
        const double lb = lp.getConstrLB(constr_id);
        const double ub = lp.getConstrUB(constr_id);
        if(lb != LP_Builder::MINUS_INFINITY) {
            os << "R" << constr_id << "LB: ";        
            print_entries(os, lp.entries(constr_id));
            os << " >= " << lb << std::endl;
        }
        if(ub != LP_Builder::INFINITY) {
            os << "R" << constr_id << "UB: ";        
            print_entries(os, lp.entries(constr_id));
            os << " <= " << ub << std::endl;
        }
    }
    return os;
}

#endif //LP_BUILDER_HPP