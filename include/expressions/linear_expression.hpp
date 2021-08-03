#ifndef LINEAR_EXPRESSION_HPP
#define LINEAR_EXPRESSION_HPP

#include <vector>

#include <range/v3/all.hpp>

struct LinearTerm;
struct LinearExpr;

struct Var {
    int id;
    explicit Var(int & i) noexcept : id(i) {}
    constexpr Var() noexcept : id(0) {}
    constexpr Var(const int & i) noexcept : id(i) {}
    constexpr const int & get() const noexcept { return id; }
    constexpr int & get() noexcept { return id; }
};

struct LinearTerm {
    int var;
    double coef;
    constexpr LinearTerm(Var v, double c=1.0)
        : var(v.get())
        , coef(c) {}
    constexpr LinearTerm& operator*(double c) {
        coef += c;
        return *this;
    }
};

constexpr LinearTerm operator*(Var v, double c) {
    return LinearTerm(v, c);
}
constexpr LinearTerm operator*(double c, Var v) {
    return LinearTerm(v, c);
}

struct LinearExpr {
    double constant;
    std::vector<int> vars;
    std::vector<double> coefs;

    LinearExpr() {}
    LinearExpr(LinearTerm t)
        : constant(0)
        , vars(1, t.var)
        , coefs(1, t.coef) {}
    LinearExpr& operator+(double c) {
        constant += c;
        return *this;
    }
    LinearExpr& operator+(LinearTerm t) {
        vars.emplace_back(t.var);
        coefs.emplace_back(t.coef);
        return *this;
    }
    LinearExpr& simplify() {
        auto zip_view = ranges::view::zip(vars, coefs);
        ranges::sort(zip_view, [](auto p1, auto p2){ return p1.first < p2.first; }); 
    
        const auto begin = zip_view.begin();
        auto first = begin;
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
        const size_t new_length = std::distance(begin, first+1);
        vars.resize(new_length);
        coefs.resize(new_length);
        return *this;
    }
};

inline LinearExpr operator+(LinearTerm t1, LinearTerm t2) {
    return LinearExpr(t1) + t2;
}

#endif //LINEAR_EXPRESSION_HPP