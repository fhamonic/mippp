#ifndef LINEAR_EXPRESSION_HPP
#define LINEAR_EXPRESSION_HPP

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
        const size_t new_length = std::distance(zip_view.begin(), first+1);
        vars.resize(new_length);
        coefs.resize(new_length);
        return *this;
    }
};

inline std::ostream& operator<<(std::ostream& os,
                                const LinearExpr & e) {
    if(e.vars.size() > 0) {
        size_t i = 0;
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

#endif //LINEAR_EXPRESSION_HPP