#ifndef LINEAR_EXPRESSION_HPP
#define LINEAR_EXPRESSION_HPP

#include <ostream>
#include <vector>

#include <range/v3/view/zip.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/algorithm/for_each.hpp>

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

    LinearExpr()
        : constant(0) {}
    LinearExpr(LinearTerm t)
        : constant(0)
        , vars(1, t.var)
        , coefs(1, t.coef) {}
    LinearExpr& add(double c) {
        constant += c;
        return *this;
    }
    LinearExpr& add(LinearTerm t) {
        vars.emplace_back(t.var);
        coefs.emplace_back(t.coef);
        return *this;
    }
    //////////////////
    LinearExpr& add(int v, double c=1.0) {
        vars.emplace_back(v);
        coefs.emplace_back(c);
        return *this;
    } ///////////////
    LinearExpr& operator+(double c) { return add(c); }
    LinearExpr& operator+(LinearTerm t) { return add(t); }
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
        const size_t new_length = std::distance(zip_view.begin(), first+1);
        vars.resize(new_length);
        coefs.resize(new_length);
        return *this;
    }
};

inline LinearExpr operator+(LinearTerm t1, LinearTerm t2) {
    return LinearExpr(t1) + t2;
}

inline std::ostream& operator<<(std::ostream& os,
                                const LinearExpr & linear_expr) {
    ranges::for_each(ranges::view::zip(linear_expr.coefs, linear_expr.vars), 
        [&os](const auto p){ os << p.first << "*x" << p.second << " + "; });
    return os << linear_expr.constant;
}

#endif //LINEAR_EXPRESSION_HPP