#ifndef MIPPP_STRONG_TYPES
#define MIPPP_STRONG_TYPES

namespace fhamonic {
namespace mippp {

class Var {
private:
    int _id;

public:
    constexpr Var(const Var & v) noexcept : _id(v.id()) {}
    explicit constexpr Var(const int & i) noexcept : _id(i) {}
    [[nodiscard]] constexpr int id() const noexcept { return _id; }
};

class Constr {
private:
    int _id;

public:
    constexpr Constr(const Constr & c) noexcept : _id(c.id()) {}
    explicit constexpr Constr(const int & i) noexcept : _id(i) {}
    [[nodiscard]] constexpr int id() const noexcept { return _id; }
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MILP_BUILDER_STRONG_TYPES