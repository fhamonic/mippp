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

#include <memory>
#include <limits>
#include <vector>

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


template <typename T>
struct IneqConstraintHandler {
    std::vector<int> & vars;
    std::vector<double> & coefs;
    double & upper_bound;
    const int begin;

    IneqConstraintHandler(std::vector<int> & vars, 
                        std::vector<double> & coefs,
                        double & ub)
        : vars(vars)
        , coefs(coefs)
        , upper_bound(ub)
        , begin(vars.size()) {}
    virtual ~IneqConstraintHandler() {}

    virtual T & operator()(double c)=0;
    virtual T & operator()(int v, double c=1.0)=0;
};

struct IneqConstraintRHSHandler
        : public IneqConstraintHandler<IneqConstraintRHSHandler> {
    virtual IneqConstraintRHSHandler & operator()(double c) {
        upper_bound += c;
        return *this;
    }
    virtual IneqConstraintRHSHandler & operator()(int v, double c=1.0) {
        vars.push_back(v);
        coefs.push_back(-c);
        return *this;
    }
};
struct IneqConstraintLHSHandler
        : public IneqConstraintHandler<IneqConstraintLHSHandler> {
    
    IneqConstraintLHSHandler(std::vector<int> & vars, 
                        std::vector<double> & coefs,
                        double & ub)
            : IneqConstraintHandler(vars, coefs, ub) {}
    virtual IneqConstraintLHSHandler & operator()(double c) {
        upper_bound -= c;
        return *this;
    }
    virtual IneqConstraintLHSHandler & operator()(int v, double c=1.0) {
        vars.push_back(v);
        coefs.push_back(c);
        return *this;
    }
    IneqConstraintRHSHandler & less() {
        return dynamic_cast<IneqConstraintRHSHandler&>(*this);
    }
};



class LP_Builder {
public:
    static constexpr double INFINITY = std::numeric_limits<double>::max();
    static constexpr int CONTINUOUS = 0;
    static constexpr int INTEGRAL = 1;
private:
    std::vector<double> col_coef;
    std::vector<double> col_lb;
    std::vector<double> col_ub;
    std::vector<int> col_type;

    std::vector<int> vars;
    std::vector<double> coefs;

    std::vector<int> row_begins;
    std::vector<double> row_lb;
    std::vector<double> row_ub;
public:
    LP_Builder() {}

    int addVar(double coef=0.0, double lb=0.0, double ub=INFINITY,
               int type=CONTINUOUS) {
        col_coef.push_back(coef);
        col_lb.push_back(lb);
        col_ub.push_back(ub);
        col_type.push_back(type);
        return col_coef.size()-1;
    }
private:
    template <typename T, typename ... Args>
    auto addVars(pack<Args...>, int count, T id_lambda, 
                                double coef, double lb,
                                double ub, int type) {
        const int offset = col_coef.size();
        const int new_size = offset + count;
        col_coef.resize(new_size, coef);
        col_lb.resize(new_size, lb);
        col_ub.resize(new_size, ub);
        col_type.resize(new_size, type);
        return [offset, id_lambda] (Args... args) {
            return offset + id_lambda(args...);
        };
    }
public:
    template <typename T>
    auto addVars(int count, T id_lambda, double coef=0.0, 
                 double lb=0.0, double ub=INFINITY, int type=CONTINUOUS) {
        return addVars(typename function_traits<T>::arg_types(),
                    count, id_lambda, coef, lb, ub, type);
    }

    LP_Builder & setObjective(int var_id, double coef) {
        col_coef[var_id] = coef;
        return *this;
    }
    LP_Builder & setBounds(int var_id, double lb, double ub) {
        col_lb[var_id] = lb;
        col_ub[var_id] = ub;
        return *this;
    }
    LP_Builder & setType(int var_id, int type) {
        col_type[var_id] = type;
        return *this;
    }

    IneqConstraintLHSHandler addIneqConstr() {
        row_begins.push_back(vars.size());
        row_lb.push_back(-INFINITY);
        row_ub.push_back(0);

        IneqConstraintLHSHandler lhs(vars, coefs, row_ub.back());
        return lhs;
    };
    void addRangeConstr(double lb, double ub) {
        
    }
};

#endif //LP_BUILDER_HPP