#ifndef QUADRATIC_EXPRESSION_HPP
#define QUADRATIC_EXPRESSION_HPP

#include <vector>

#include <range/v3/view/zip.hpp>
#include <range/v3/algorithm/sort.hpp>
#include <range/v3/algorithm/for_each.hpp>

#include "expressions/linear_expression.hpp"



struct QuadraticExpr {
    LinearExpr linear_expr;
    std::vector<int> vars1;
    std::vector<int> vars2;
    std::vector<double> coefs;
    QuadraticExpr() {}
    QuadraticExpr(LinearExpr&& e)
        : linear_expr(std::move(e)) {}
    QuadraticExpr(int v1, int v2, double c=1.0)
        : vars1(1, v1)
        , vars2(1, v2)
        , coefs(1, c) {}
    QuadraticExpr& add(double c) {
        linear_expr.add(c);
        return *this;
    }
    QuadraticExpr& add(int v, double c=1.0) {
        linear_expr.add(v, c);
        return *this;
    }
    QuadraticExpr& add(int v1, int v2, double c=1.0) {
        vars1.emplace_back(v1);
        vars2.emplace_back(v2);
        coefs.emplace_back(c);
        return *this;
    }
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

inline std::ostream& operator<<(std::ostream& os,
                                const QuadraticExpr & e) {
    if(e.vars1.size() > 0) {
        std::size_t i = 0;
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