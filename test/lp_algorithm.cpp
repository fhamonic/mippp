#include <gtest/gtest.h>

#include <concepts>
#include <variant>

#include "mippp/detail/lp_algorithm_codec.hpp"
#include "mippp/model_concepts.hpp"

using namespace mippp;

namespace {

using listed_algorithm =
    std::variant<lp_algorithm::automatic, lp_algorithm::dual_simplex,
                 lp_algorithm::interior_point, lp_algorithm::other>;

struct exact_model {
    listed_algorithm algorithm = lp_algorithm::automatic{};

    template <detail::settable_lp_algorithm<listed_algorithm> A>
    void set_lp_algorithm(A tag) {
        algorithm = tag;
    }
    listed_algorithm get_lp_algorithm() const { return algorithm; }
};

// what a backend must not write: a refinement converts to the base it takes
struct slicing_model {
    void set_lp_algorithm(lp_algorithm::automatic);
    void set_lp_algorithm(lp_algorithm::dual_simplex);
    void set_lp_algorithm(lp_algorithm::interior_point);
    listed_algorithm get_lp_algorithm() const;
};

struct setter_only_model {
    void set_lp_algorithm(lp_algorithm::automatic);
};

template <typename M, typename A>
concept takes_lp_algorithm =
    requires(M & model, A tag) { model.set_lp_algorithm(tag); };

}  // namespace

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////// Concept ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(has_lp_algorithm<exact_model>);
static_assert(has_lp_algorithm<exact_model, lp_algorithm::automatic>);
static_assert(has_lp_algorithm<exact_model, lp_algorithm::dual_simplex>);
static_assert(has_lp_algorithm<exact_model, lp_algorithm::interior_point>);
static_assert(!has_lp_algorithm<exact_model, lp_algorithm::primal_simplex>);
// neither a base of a listed tag nor a refinement of one
static_assert(!has_lp_algorithm<exact_model, lp_algorithm::simplex>);
static_assert(!has_lp_algorithm<
              exact_model, lp_algorithm::interior_point_without_crossover>);
static_assert(!takes_lp_algorithm<
              exact_model, lp_algorithm::interior_point_without_crossover>);
static_assert(!has_lp_algorithm<exact_model, lp_algorithm::other>);

// the call compiles there, and the concept still says no
static_assert(takes_lp_algorithm<
              slicing_model, lp_algorithm::interior_point_without_crossover>);
static_assert(!has_lp_algorithm<
              slicing_model, lp_algorithm::interior_point_without_crossover>);

static_assert(!has_lp_algorithm<setter_only_model>);
static_assert(!has_lp_algorithm<int>);

static_assert(
    std::same_as<model_lp_algorithm_t<exact_model>, listed_algorithm>);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Codec /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

namespace {

struct knobs {
    int method, crossover;
    bool operator==(const knobs &) const = default;
};
// clang-format off
constexpr knobs encode(std::same_as<lp_algorithm::automatic> auto)      { return {-1, -1}; }
constexpr knobs encode(std::same_as<lp_algorithm::dual_simplex> auto)   { return {1, -1}; }
constexpr knobs encode(std::same_as<lp_algorithm::interior_point> auto) { return {2, -1}; }
constexpr knobs encode(
    std::same_as<lp_algorithm::interior_point_without_crossover> auto)  { return {2, 0}; }
// clang-format on

using coded_algorithm =
    std::variant<lp_algorithm::automatic, lp_algorithm::dual_simplex,
                 lp_algorithm::interior_point,
                 lp_algorithm::interior_point_without_crossover,
                 lp_algorithm::other>;

coded_algorithm decode(knobs native) {
    return detail::decode_lp_algorithm(coded_algorithm{lp_algorithm::other{}},
                                       native,
                                       [](auto tag) { return encode(tag); });
}

}  // namespace

TEST(lp_algorithm_codec, decodes_what_each_tag_encodes) {
    EXPECT_TRUE(is<lp_algorithm::automatic>(decode({-1, -1})));
    EXPECT_TRUE(is<lp_algorithm::dual_simplex>(decode({1, -1})));
    EXPECT_TRUE(is<lp_algorithm::interior_point>(decode({2, -1})));
    EXPECT_TRUE(
        is<lp_algorithm::interior_point_without_crossover>(decode({2, 0})));
}

// A tag for knobs its encoder writes differently would make a caller that
// sets the algorithm it read reset them: a barrier under a crossover
// strategy and a dual simplex under no crossover are not the listed tags.
TEST(lp_algorithm_codec, knobs_no_tag_encodes_read_other) {
    EXPECT_TRUE(is<lp_algorithm::other>(decode({2, 3})));
    EXPECT_TRUE(is<lp_algorithm::other>(decode({1, 0})));
    EXPECT_TRUE(is<lp_algorithm::other>(decode({5, -1})));
}

TEST(lp_algorithm_codec, set_then_get_on_the_exact_model) {
    exact_model model;
    EXPECT_TRUE(is<lp_algorithm::automatic>(model.get_lp_algorithm()));
    model.set_lp_algorithm(lp_algorithm::interior_point{});
    EXPECT_TRUE(is<lp_algorithm::interior_point>(model.get_lp_algorithm()));
    EXPECT_TRUE(is_a<lp_algorithm::interior_point>(model.get_lp_algorithm()));
    EXPECT_FALSE(is_a<lp_algorithm::simplex>(model.get_lp_algorithm()));
}
