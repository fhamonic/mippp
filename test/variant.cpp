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
