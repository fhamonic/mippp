#ifndef LP_BUILDER_HPP
#define LP_BUILDER_HPP

#include <memory>
#include <numeric>
#include <vector>

template<typename... Args>
struct pack {};

template <typename T>
struct function_traits
    : public function_traits<decltype(&T::operator())>
{};
template <typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType(ClassType::*)(Args...) const> {
    using result_type = Return;
    using arg_types = pack<Args...>;
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

    std::vector<int> row_begins;
    std::vector<double> row_lb;
    std::vector<double> row_ub;
    std::vector<int> vars;
    std::vector<double> coefs;
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
                                double coef=0.0, double lb=0.0,
                                double ub=INFINITY, int type=CONTINUOUS) {
        const int offset = col_coef.size();
        const int new_size = offset + count;
        col_coef.resize(new_size, 0);
        col_lb.resize(new_size, 0);
        col_ub.resize(new_size, INFINITY);
        col_type.resize(new_size, CONTINUOUS);
        return [offset, id_lambda] (Args... args) {
            return offset + id_lambda(args...);
        };
    }
public:
    template <typename T>
    auto addVars(int count, T id_lambda, double coef=0.0, 
                 double lb=0.0, double ub=INFINITY, int type=CONTINUOUS) {
        return test(typename function_traits<T>::arg_types(),
                    count, id_lambda);
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

    void addIneqConstr() {
        // return handler(*this);
    };
    void addRangeConstr(double lb, double ub) {
        //
    }
};

#endif //LP_BUILDER_HPP