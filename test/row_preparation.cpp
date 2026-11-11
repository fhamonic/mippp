#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "mippp/detail/row_preparation.hpp"

using namespace mippp;

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double not_a_number = std::numeric_limits<double>::quiet_NaN();

struct row {
    std::vector<int> indices;
    std::vector<double> coefficients;

    auto prepared(const constraint_sense sense, const double rhs,
                  const std::vector<double> & point) {
        return detail::prepare_user_cut(indices, coefficients, sense, rhs,
                                        point.data());
    }
};

}  // namespace

TEST(strip_zero_entries, removes_zeros_of_either_sign) {
    std::vector<int> indices = {4, 0, 2, 7, 1};
    std::vector<double> coefficients = {0.0, 3.0, -0.0, -1.5, 0.0};
    detail::strip_zero_entries(indices, coefficients);
    ASSERT_EQ(indices, (std::vector<int>{0, 7}));
    ASSERT_EQ(coefficients, (std::vector<double>{3.0, -1.5}));
}

TEST(strip_zero_entries, leaves_an_empty_row_empty) {
    std::vector<int> indices;
    std::vector<double> coefficients;
    detail::strip_zero_entries(indices, coefficients);
    ASSERT_TRUE(indices.empty());
    ASSERT_TRUE(coefficients.empty());
}

TEST(row_violation, is_positive_where_the_point_violates_the_row) {
    const std::vector<int> indices = {0, 2};
    const std::vector<double> coefficients = {2.0, 1.0};
    const std::vector<double> point = {1.0, 100.0, 3.0};  // activity 5
    using enum constraint_sense;
    const auto violation = [&](const constraint_sense sense, const double rhs) {
        return detail::row_violation(indices, coefficients, sense, rhs,
                                     point.data());
    };
    ASSERT_DOUBLE_EQ(violation(less_equal, 4.0), 1.0);
    ASSERT_DOUBLE_EQ(violation(less_equal, 7.0), -2.0);
    ASSERT_DOUBLE_EQ(violation(greater_equal, 7.0), 2.0);
    ASSERT_DOUBLE_EQ(violation(greater_equal, 4.0), -1.0);
    ASSERT_DOUBLE_EQ(violation(equal, 4.0), 1.0);
    ASSERT_DOUBLE_EQ(violation(equal, 7.0), 2.0);
    ASSERT_DOUBLE_EQ(violation(equal, 5.0), 0.0);
}

TEST(prepare_user_cut, scales_to_a_largest_coefficient_of_one) {
    row cut{{0, 1}, {-4.0, 2.0}};
    const auto result =
        cut.prepared(constraint_sense::less_equal, 2.0, {0.0, 3.0});
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->sense, constraint_sense::less_equal);
    ASSERT_DOUBLE_EQ(result->rhs, 0.5);
    ASSERT_EQ(cut.indices, (std::vector<int>{0, 1}));
    ASSERT_DOUBLE_EQ(cut.coefficients[0], -1.0);
    ASSERT_DOUBLE_EQ(cut.coefficients[1], 0.5);
}

// The same cut written at another scale reaches the solver identical: its
// search must not depend on the units of the separator.
TEST(prepare_user_cut, a_scale_factor_changes_nothing) {
    const std::vector<double> point = {0.75, 0.75};
    row reference{{0, 1}, {1.0, 1.0}};
    const auto expected =
        reference.prepared(constraint_sense::less_equal, 1.0, point);
    ASSERT_TRUE(expected.has_value());
    for(const double scale : {1e-8, 1e-3, 1e6, 1e12}) {
        row cut{{0, 1}, {scale, scale}};
        const auto result =
            cut.prepared(constraint_sense::less_equal, scale, point);
        ASSERT_TRUE(result.has_value()) << scale;
        ASSERT_DOUBLE_EQ(result->rhs, expected->rhs) << scale;
        ASSERT_DOUBLE_EQ(cut.coefficients[0], reference.coefficients[0]);
        ASSERT_DOUBLE_EQ(cut.coefficients[1], reference.coefficients[1]);
    }
}

TEST(prepare_user_cut, strips_zeros_before_and_after_scaling) {
    row cut{{0, 1, 2, 3}, {0.0, 1e300, -0.0, 1e-300}};
    const auto result =
        cut.prepared(constraint_sense::greater_equal, 1e300, {1, 0.5, 1, 1});
    ASSERT_TRUE(result.has_value());
    // 1e-300 / 1e300 is a zero: the entry goes with the two written as such
    ASSERT_EQ(cut.indices, (std::vector<int>{1}));
    ASSERT_DOUBLE_EQ(cut.coefficients[0], 1.0);
    ASSERT_DOUBLE_EQ(result->rhs, 1.0);
}

TEST(prepare_user_cut, keeps_the_violated_side_of_an_equality) {
    {
        row cut{{0, 1}, {1.0, 1.0}};
        const auto result =
            cut.prepared(constraint_sense::equal, 1.0, {0.75, 0.75});
        ASSERT_TRUE(result.has_value());
        ASSERT_EQ(result->sense, constraint_sense::less_equal);
        ASSERT_DOUBLE_EQ(result->rhs, 1.0);
    }
    {
        row cut{{0, 1}, {1.0, 1.0}};
        const auto result =
            cut.prepared(constraint_sense::equal, 1.0, {0.25, 0.25});
        ASSERT_TRUE(result.has_value());
        ASSERT_EQ(result->sense, constraint_sense::greater_equal);
        ASSERT_DOUBLE_EQ(result->rhs, 1.0);
    }
}

TEST(prepare_user_cut, drops_a_cut_the_point_satisfies) {
    using enum constraint_sense;
    const std::vector<double> point = {0.5, 0.5};  // x + y = 1
    ASSERT_FALSE((row{{0, 1}, {1.0, 1.0}}).prepared(less_equal, 1.5, point));
    ASSERT_FALSE((row{{0, 1}, {1.0, 1.0}}).prepared(greater_equal, 0.5, point));
    ASSERT_FALSE((row{{0, 1}, {1.0, 1.0}}).prepared(equal, 1.0, point));
    // held at the point: the case that keeps a solver re-separating forever
    ASSERT_FALSE((row{{0, 1}, {1.0, 1.0}}).prepared(less_equal, 1.0, point));
    ASSERT_FALSE((row{{0, 1}, {1.0, 1.0}}).prepared(greater_equal, 1.0, point));
}

TEST(prepare_user_cut, measures_the_violation_on_the_scaled_cut) {
    using enum constraint_sense;
    const std::vector<double> point = {0.5, 0.5};
    ASSERT_FALSE(
        (row{{0, 1}, {1.0, 1.0}}).prepared(less_equal, 1.0 - 5e-7, point));
    ASSERT_TRUE(
        (row{{0, 1}, {1.0, 1.0}}).prepared(less_equal, 1.0 - 5e-6, point));
    // violated by 5e-3 as written, by 5e-7 once the largest coefficient is 1
    ASSERT_FALSE(
        (row{{0, 1}, {1e4, 1e4}}).prepared(less_equal, 1e4 - 5e-3, point));
    ASSERT_TRUE(
        (row{{0, 1}, {1e-4, 1e-4}}).prepared(less_equal, 1e-4 - 5e-10, point));
}

// The documented price of the scaling: w.x - K y <= r, with y = 0 at the
// point, is dropped once K exceeds a million times the violation.
TEST(prepare_user_cut, drops_a_big_m_cut_violated_below_the_tolerance) {
    using enum constraint_sense;
    const std::vector<double> point = {1.0, 1.0, 0.0};  // x0 + x1 = 2
    ASSERT_TRUE(
        (row{{0, 1, 2}, {1.0, 1.0, -1e5}}).prepared(less_equal, 1.5, point));
    ASSERT_FALSE(
        (row{{0, 1, 2}, {1.0, 1.0, -1e7}}).prepared(less_equal, 1.5, point));
}

TEST(prepare_user_cut, drops_an_empty_cut) {
    using enum constraint_sense;
    ASSERT_FALSE((row{{}, {}}).prepared(greater_equal, 1.0, {0.0}));
    // x - x folds to this, and reads as the violated 0 >= 1
    ASSERT_FALSE((row{{0}, {0.0}}).prepared(greater_equal, 1.0, {0.0}));
}

TEST(prepare_user_cut, drops_a_non_finite_cut) {
    using enum constraint_sense;
    const std::vector<double> point = {1.0, 1.0};
    for(const constraint_sense sense : {less_equal, greater_equal, equal}) {
        ASSERT_FALSE(
            (row{{0, 1}, {not_a_number, 1.0}}).prepared(sense, 0.5, point));
        ASSERT_FALSE(
            (row{{0, 1}, {1.0, 1.0}}).prepared(sense, not_a_number, point));
        ASSERT_FALSE(
            (row{{0, 1}, {infinity, 1.0}}).prepared(sense, 0.5, point));
        ASSERT_FALSE(
            (row{{0, 1}, {1.0, 1.0}}).prepared(sense, infinity, point));
        ASSERT_FALSE(
            (row{{0, 1}, {1.0, 1.0}}).prepared(sense, -infinity, point));
    }
}
