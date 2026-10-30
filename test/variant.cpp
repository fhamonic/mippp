#include <gtest/gtest.h>

#include <variant>

#include "mippp/utility/variant.hpp"

// the one include is the point: the basis API needs these without status.hpp

using namespace mippp;

namespace {
struct shape {};
struct polygon : shape {};
struct triangle : polygon {};
struct circle : shape {};
struct unrelated {};
}  // namespace

using SV = std::variant<triangle, circle>;

static_assert(variant_of<SV, shape>);
static_assert(!variant_of<std::variant<triangle, unrelated>, shape>);
static_assert(variant_with_alternative<SV, circle>);
static_assert(
    !variant_with_alternative<SV, polygon>);  // a base is not an alternative
static_assert(variant_containing_a<SV, polygon>);
static_assert(!variant_containing_a<SV, unrelated>);

// An alternative without an overload of its own reaches its nearest base's:
// triangle reaches polygon's, not shape's. Checked at run time, not by
// static_assert: MSVC 17.11 rejects slicing the empty triangle into the
// by-value polygon in a constant expression (C2131, "read of an uninitialized
// symbol").
int visit_shape(const std::variant<triangle, circle, shape> & v) {
    return std::visit<int>(
        detail::overloaded{[](polygon) { return 1; }, [](circle) { return 2; },
                           [](shape) { return 0; }},
        v);
}

TEST(variant, visit_reaches_the_nearest_base_overload) {
    EXPECT_EQ(visit_shape(triangle{}), 1);
    EXPECT_EQ(visit_shape(circle{}), 2);
    EXPECT_EQ(visit_shape(shape{}), 0);
}

TEST(variant, is_matches_the_exact_alternative) {
    SV v = triangle{};
    ASSERT_TRUE(is<triangle>(v));
    ASSERT_FALSE(is<circle>(v));
}

TEST(variant, is_a_matches_any_derived_alternative) {
    SV v = triangle{};
    ASSERT_TRUE(is_a<polygon>(v));
    ASSERT_TRUE(is_a<shape>(v));
    ASSERT_FALSE(is_a<circle>(v));
}
