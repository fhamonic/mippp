#ifndef MILPPP_STRONG_TYPES
#define MILPPP_STRONG_TYPES

class Var {
private:
    int _id;
public:
    constexpr Var(const Var & v) noexcept : _id(v.id()) {}
    explicit constexpr Var(const int & i) noexcept : _id(i) {}
    constexpr const int & id() const noexcept { return _id; }
    constexpr int & id() noexcept { return _id; }
};

class Constr {
private:
    int _id;
public:
    constexpr Constr(const Constr & c) noexcept : _id(c.id()) {}
    explicit constexpr Constr(const int & i) noexcept : _id(i) {}
    constexpr const int & id() const noexcept { return _id; }
    constexpr int & id() noexcept { return _id; }
};

#endif //MILP_BUILDER_STRONG_TYPES