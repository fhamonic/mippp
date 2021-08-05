#ifndef LP_BUILDER_HPP
#define LP_BUILDER_HPP

#include <memory>
#include <numeric>
#include <vector>

class VarType {
    protected:
        int _number;
        int _offset;
        double _default_lb;
        double _default_ub;
        int _integer;
        VarType(int number, double lb=0, double ub=INFTY, bool integer=false) : _number(number), _offset(0), _default_lb(lb), _default_ub(ub), _integer(integer) {}
        VarType() : VarType(0) {}
    public:
        void setOffset(int offset) { _offset = offset; }
        int getOffset() { return _offset; }
        int getNumber() const { return _number; }
        double getDefaultLB() { return _default_lb; }
        double getDefaultUB() { return _default_ub; }
        bool isInteger() { return _integer; }
};



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

class LP_Builder {
    using INFINITY = std::numeric_limits<double>::max();
    using CONTINUOUS = 0;
    using INTEGRAL = 1;

    std::vector<double> objective;
    std::vector<double> col_lb;
    std::vector<double> col_ub;
    std::vector<int> col_type;

    std::vector<int> row_begins;
    std::vector<double> row_lb;
    std::vector<double> row_ub;
    std::vector<int> vars;
    std::vector<double> coefs;

    void addVar(double lb=0, double ub=INFINITY, int type=CONTINUOUS) {
        // objective.push
    }
    


    template <typename T, typename ... Args>
    auto addVars(pack<Args...>, int count, T id_lambda) {
        const int offset = objective.size();
        const int new_size = offset + count;

        objective.resize(new_size, 0);
        col_lb.resize(new_size, 0);
        col_ub.resize(new_size, INFINITY);
        col_type.resize(new_size, CONTINUOUS);

        return [offset, id_lambda] (Args... args) {
            return offset + id_lambda(args...);
        };
    }
    template <typename T>
    auto addVars(int count, T id_lambda) {
        return test(typename function_traits<T>::arg_types(), count, id_lambda);
    }
}

#endif //LP_BUILDER_HPP