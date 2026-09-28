#pragma once

#include <gtest/gtest.h>

#include "mippp/model_concepts.hpp"
#include "mippp/utility/iis_by_deletion.hpp"
#include "mippp/utility/variant.hpp"

#include "iis_cases.hpp"

namespace mippp {

struct iis_native_path {
    // a routine that cannot side an equality row may report it whole
    static constexpr bool names_every_side = false;
    template <typename M>
    static auto compute(M & model) {
        return model.compute_iis();
    }
};

template <typename T>
struct IisTest : public iis_cases::fixture<T, iis_native_path> {
    using typename T::model_type;
    static_assert(has_iis<model_type>);
    using iis_cases::fixture<T, iis_native_path>::save;
    using iis_cases::fixture<T, iis_native_path>::expect_unchanged;
    using iis_cases::fixture<T, iis_native_path>::answer_text;

    // whether the routine returns or throws, the status of an earlier solve
    // is gone: the solver may now hold the analysis' own solution
    void check_status_is_unknown_after_compute_iis() {
        {
            auto model = this->new_model();
            build(model, iis_cases::feasible_model_case());
            model.solve();
            ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
            const auto iis = model.compute_iis();
            EXPECT_EQ(iis.get_outcome(), iis_outcome::feasible);
            EXPECT_TRUE(is<status::unknown>(model.get_status()));
            model.solve();
            EXPECT_TRUE(is_a<status::optimal>(model.get_status()));
        }
        {
            auto model = this->new_model();
            build(model, iis_cases::bounds_against_a_row_case());
            model.solve();
            ASSERT_TRUE(is_a<status::infeasible>(model.get_status()));
            const auto iis = model.compute_iis();
            EXPECT_EQ(iis.get_outcome(), iis_outcome::irreducible);
            EXPECT_TRUE(is<status::unknown>(model.get_status()));
            model.solve();
            EXPECT_TRUE(is_a<status::infeasible>(model.get_status()));
        }
    }

    // The two answers may differ: each one is validated on its own.
    void check_both_paths_find_valid_iis() {
        using M = model_type;
        if constexpr(!iis_by_deletion_model<M>) {
            GTEST_SKIP() << "the model has no modifiable row bounds yet";
        } else {
            const iis_cases::iis_case c =
                iis_cases::bounds_against_a_row_case();
            auto model = this->new_model();
            const iis_cases::built_case<M> built = build(model, c);
            const iis_cases::saved_model_data before = save(model, built);
            const auto native = model.compute_iis();
            ASSERT_EQ(native.get_outcome(), iis_outcome::irreducible);
            const iis_cases::case_answer native_answer =
                iis_oracle::read_answer(native, built.variables,
                                        built.constraints);
            EXPECT_TRUE(iis_oracle::is_iis(c.system, native_answer))
                << answer_text(native_answer);
            expect_unchanged(model, built, before);
            const auto by_deletion = compute_iis_by_deletion(model);
            ASSERT_EQ(by_deletion.get_outcome(), iis_outcome::irreducible);
            const iis_cases::case_answer deletion_answer =
                iis_oracle::read_answer(by_deletion, built.variables,
                                        built.constraints);
            EXPECT_TRUE(iis_oracle::is_iis(c.system, deletion_answer))
                << answer_text(deletion_answer);
            expect_unchanged(model, built, before);
        }
    }
};
TYPED_TEST_SUITE_P(IisTest);
GTEST_ALLOW_UNINSTANTIATED_PARAMETERIZED_TEST(IisTest);

TYPED_TEST_P(IisTest, bounds_against_a_row) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::bounds_against_a_row_case()); });
}
TYPED_TEST_P(IisTest, one_side_of_an_equality_row) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::one_side_of_an_equality_row_case());
    });
}
TYPED_TEST_P(IisTest, integer_equal_to_one_half) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::integer_equal_to_one_half_case());
    });
}
TYPED_TEST_P(IisTest, integers_summing_to_one_half) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::integers_summing_to_one_half_case());
    });
}
TYPED_TEST_P(IisTest, integer_in_a_fractional_interval) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::integer_in_a_fractional_interval_case());
    });
}
TYPED_TEST_P(IisTest, ranged_row_lower_side) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::ranged_row_lower_side_case());
    });
}
TYPED_TEST_P(IisTest, ranged_row_upper_side) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::ranged_row_upper_side_case());
    });
}
TYPED_TEST_P(IisTest, redundant_rows) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::redundant_rows_case()); });
}
TYPED_TEST_P(IisTest, two_disjoint_conflicts) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::two_disjoint_conflicts_case());
    });
}
TYPED_TEST_P(IisTest, chain_where_every_row_is_needed) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::chain_where_every_row_is_needed_case());
    });
}
TYPED_TEST_P(IisTest, crossed_variable_bounds) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::crossed_variable_bounds_case());
    });
}
TYPED_TEST_P(IisTest, crossed_row_sides) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::crossed_row_sides_case()); });
}
TYPED_TEST_P(IisTest, crossed_term_less_row) {
    this->SkipOnLicenseError([this]() {
        this->check_case(iis_cases::crossed_term_less_row_case());
    });
}
TYPED_TEST_P(IisTest, feasible_model) {
    this->SkipOnLicenseError(
        [this]() { this->check_case(iis_cases::feasible_model_case()); });
}
TYPED_TEST_P(IisTest, removed_variable_is_skipped) {
    this->SkipOnLicenseError(
        [this]() { this->check_removed_variable_is_skipped(); });
}
TYPED_TEST_P(IisTest, answer_survives_a_later_removal) {
    this->SkipOnLicenseError(
        [this]() { this->check_answer_survives_a_later_removal(); });
}
TYPED_TEST_P(IisTest, published_vectors_under_transforms) {
    this->SkipOnLicenseError([this]() { this->check_published_vectors(); });
}
TYPED_TEST_P(IisTest, model_modified_after_an_infeasible_solve) {
    this->SkipOnLicenseError(
        [this]() { this->check_model_modified_after_an_infeasible_solve(); });
}
TYPED_TEST_P(IisTest, model_data_and_result_survive_the_call) {
    this->SkipOnLicenseError(
        [this]() { this->check_model_data_and_result_survive_the_call(); });
}
TYPED_TEST_P(IisTest, column_less_model_names_the_violated_row) {
    this->SkipOnLicenseError([this]() { this->check_column_less_model(); });
}

TYPED_TEST_P(IisTest, status_is_unknown_after_compute_iis) {
    this->SkipOnLicenseError(
        [this]() { this->check_status_is_unknown_after_compute_iis(); });
}
TYPED_TEST_P(IisTest, both_paths_find_valid_iis) {
    this->SkipOnLicenseError(
        [this]() { this->check_both_paths_find_valid_iis(); });
}

REGISTER_TYPED_TEST_SUITE_P(
    IisTest, bounds_against_a_row, one_side_of_an_equality_row,
    integer_equal_to_one_half, integers_summing_to_one_half,
    integer_in_a_fractional_interval, ranged_row_lower_side,
    ranged_row_upper_side, redundant_rows, two_disjoint_conflicts,
    chain_where_every_row_is_needed, crossed_variable_bounds, crossed_row_sides,
    crossed_term_less_row, feasible_model, removed_variable_is_skipped,
    answer_survives_a_later_removal, published_vectors_under_transforms,
    model_modified_after_an_infeasible_solve,
    model_data_and_result_survive_the_call,
    column_less_model_names_the_violated_row,
    status_is_unknown_after_compute_iis, both_paths_find_valid_iis);

}  // namespace mippp
