#ifndef MIPPP_VARS_RANGE_HPP
#define MIPPP_VARS_RANGE_HPP

namespace fhamonic {
namespace mippp {

template <typename F, typename... Args>
class vars_range {
private:
    const std::size_t _offset;
    const std::size_t _count;
    const F _id_lambda;

public:
    constexpr vars_range(std::size_t offset, std::size_t count,
                         F && id_lambda) noexcept
        : _offset(offset)
        , _count(count)
        , _id_lambda(std::forward<F>(id_lambda)) {}

    Var operator()(Args... args) const {
        const int id = _id_lambda(args...);
        assert(0 <= id && id < _count);
        return Var(_offset + id);
    }
    Var operator[](std::size_t id) const {
        assert(0 <= id && id < _count);
        return Var(_offset + id);
    }

    // begin();
    // end();
};

}  // namespace mippp
}  // namespace fhamonic

#endif  // MIPPP_VARS_RANGE_HPP