#ifndef EXPRESSIONS_HPP
#define EXPRESSIONS_HPP

#include <vector>

struct LinearTerm;
struct LinearExpr;
struct QuadraticTerm;
struct QuadraticExpr;

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
};

LinearExpr operator+(LinearTerm t1, LinearTerm t2) {
    return LinearExpr(t1) + t2;
}

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
    QuadraticExpr(LinearExpr&& e)
        : linear_expr(std::move(e)) {}
    QuadraticExpr(QuadraticTerm t)
        : vars1(1, t.var1)
        , vars2(1, t.var2)
        , coefs(1, t.coef) {}
    QuadraticExpr& operator+(double c) {
        linear_expr + c;
        return *this;
    }
    QuadraticExpr& operator+(LinearTerm t) {
        linear_expr + t;
        return *this;
    }
    QuadraticExpr& operator+(QuadraticTerm t) {
        vars1.emplace_back(t.var1);
        vars2.emplace_back(t.var2);
        coefs.emplace_back(t.coef);
        return *this;
    }
};

QuadraticExpr operator+(QuadraticTerm t1, QuadraticTerm t2) {
    return std::forward<QuadraticExpr&&>(QuadraticExpr(t1) + t2);
}
QuadraticExpr operator+(LinearExpr&& t, QuadraticTerm v) {
    return QuadraticExpr(std::move(t)) + v;
}

#endif //EXPRESSIONS_HPP