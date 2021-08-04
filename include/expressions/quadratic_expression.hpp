#ifndef QUADRATIC_EXPRESSION_HPP
#define QUADRATIC_EXPRESSION_HPP

#include <vector>

#include <range/v3/view/zip.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/algorithm/for_each.hpp>

#include "expressions/linear_expression.hpp"

struct QuadraticTerm;
struct QuadraticExpr;

struct QuadraticTerm {
    int var1;
    int var2;
    double coef;
    constexpr QuadraticTerm(Var v1, Var v2, double c)
        : var1(v1.get())
        , var2(v2.get())
        , coef(c) {}
    constexpr QuadraticTerm& operator*(double c) {
        coef += c;
        return *this;
    }
};

constexpr QuadraticTerm operator*(LinearTerm t, Var v) {
    return QuadraticTerm(t.var, v, t.coef);
}

struct QuadraticExpr {
    LinearExpr linear_expr;
    std::vector<int> vars1;
    std::vector<int> vars2;
    std::vector<double> coefs;
    QuadraticExpr() {}
    QuadraticExpr(LinearExpr&& e)
        : linear_expr(std::move(e)) {}
    QuadraticExpr(QuadraticTerm t)
        : vars1(1, t.var1)
        , vars2(1, t.var2)
        , coefs(1, t.coef) {}
    QuadraticExpr& add(double c) {
        linear_expr.add(c);
        return *this;
    }
    QuadraticExpr& add(LinearTerm t) {
        linear_expr.add(t);
        return *this;
    }
    QuadraticExpr& add(QuadraticTerm t) {
        vars1.emplace_back(t.var1);
        vars2.emplace_back(t.var2);
        coefs.emplace_back(t.coef);
        return *this;
    }
    //////////////////
    QuadraticExpr& add(int v, double c=1.0) {
        linear_expr.add(v, c);
        return *this;
    }
    QuadraticExpr& add(int v1, int v2, double c=1.0) {
        vars1.push_back(v1);
        vars2.push_back(v2);
        coefs.push_back(c);
        return *this;
    } ///////////////
    QuadraticExpr& operator+(double c) { return add(c); }
    QuadraticExpr& operator+(LinearTerm t) { return add(t); }
    QuadraticExpr& operator+(QuadraticTerm t) { return add(t); }
    QuadraticExpr& simplify() {
        linear_expr.simplify();
        auto zip_view = ranges::view::zip(vars1, vars2, coefs);
        ranges::sort(zip_view, [](auto t1, auto t2) {
            const int t1v1 = std::min(std::get<0>(t1),std::get<1>(t1));
            const int t2v1 = std::min(std::get<0>(t2),std::get<1>(t2));
            if(t1v1 < t2v1)
                return true;
            if(t1v1 == t2v1) {
                const int t1v2 = std::max(std::get<0>(t1),std::get<1>(t1));
                const int t2v2 = std::max(std::get<0>(t2),std::get<1>(t2));
                return t1v2 < t2v2;
            }
            return false;
        }); 
        // TODO
        return *this;
    }
};

inline QuadraticExpr operator+(QuadraticTerm t1, QuadraticTerm t2) {
    return std::forward<QuadraticExpr&&>(QuadraticExpr(t1) + t2);
}
inline QuadraticExpr operator+(LinearExpr&& le, QuadraticTerm t) {
    QuadraticExpr e(std::move(le));
    e.add(t);
    return e;
}

inline std::ostream& operator<<(std::ostream& os,
                                const QuadraticExpr & e) {
    if(e.vars1.size() > 0) {
        size_t i = 0;
        os << (e.coefs[i] < 0 ? "- " : "");
        if(e.coefs[i]!=1)
           os << std::abs(e.coefs[i]) << " * ";
        os << "x" << e.vars1[i] << "x" << e.vars2[i];
        for(++i; i < e.vars1.size(); ++i) {
            os << (e.coefs[i] < 0 ? " - " : " + ");
            if(e.coefs[i]!=1)
                os << std::abs(e.coefs[i]) << " * ";
            os << "x" << e.vars1[i] << "x" << e.vars2[i];
        }
    }
    return os << " " << e.linear_expr;
}

#endif //QUADRATIC_EXPRESSION_HPP