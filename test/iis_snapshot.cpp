#include <gtest/gtest.h>

#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/iis_outcome.hpp"
#include "mippp/utility/iis_snapshot.hpp"
#include "mippp/utility/status.hpp"
#include "mippp/utility/variant.hpp"

#include "iis_outcome_assert.hpp"

using namespace mippp;

namespace {

using variable = model_variable<int, double>;
using constraint = model_constraint<int>;

// the surface model_variable_t and model_constraint_t deduce from
struct handles_only_model {
    variable add_variable();
    template <typename LC>
    constraint add_constraint(LC &&);
};

using sided_status =
    std::variant<iis_status::absent, iis_status::member_lower,
                 iis_status::member_upper, iis_status::member_both>;
// the rows of a routine that cannot name the side of an equality row
using row_status = std::variant<iis_status::absent, iis_status::member_lower,
                                iis_status::member_upper, iis_status::member>;

using outcome = std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                             iis_outcome::feasible, iis_outcome::time_limit>;

using sided_snapshot =
    iis_snapshot<variable, constraint, sided_status, sided_status, outcome>;
using row_snapshot =
    iis_snapshot<variable, constraint, sided_status, row_status, outcome>;

struct native_model : handles_only_model {
    sided_snapshot compute_iis();
};

enum class flaw {
    none,
    unsided_status,
    mutable_status,
    no_outcome,
    bare_outcome,
    int_variable_count,
    int_constraint_count
};

template <flaw F>
struct answer {
    using status = std::conditional_t<F == flaw::unsided_status,
                                      iis_status::member, sided_status>;
    status get_status(variable) const
        requires(F != flaw::mutable_status);
    status get_status(variable)
        requires(F == flaw::mutable_status);
    status get_status(constraint) const;
    std::conditional_t<F == flaw::bare_outcome, iis_outcome::irreducible,
                       outcome>
    get_outcome() const
        requires(F != flaw::no_outcome);
    std::conditional_t<F == flaw::int_variable_count, int, std::size_t>
    num_variable_members() const;
    std::conditional_t<F == flaw::int_constraint_count, int, std::size_t>
    num_constraint_members() const;
};

struct unsided_answer_model : handles_only_model {
    answer<flaw::unsided_status> compute_iis();
};

struct other_handles_model {
    model_variable<long, double> add_variable();
    template <typename LC>
    model_constraint<long> add_constraint(LC &&);
};

template <typename V, typename C, typename VS, typename CS, typename O>
concept snapshot_accepts = requires { typename iis_snapshot<V, C, VS, CS, O>; };

template <typename Status, typename Tag>
concept table_accepts =
    requires(mippp::detail::handle_status_table<Status> & table, Tag tag) {
        table.set(0, tag);
    };

template <typename Status, typename Tag>
concept table_counts =
    requires(const mippp::detail::handle_status_table<Status> & table) {
        table.template count_a<Tag>();
    };

struct possibly_lower : iis_status::member_lower {};

template <typename Status, typename Tag>
constexpr iis_sides sides_of_tag() {
    return iis_status::sides_of(Status(std::in_place_type<Tag>));
}

template <typename Status>
mippp::detail::handle_status_table<Status> make_table(
    std::size_t id_bound,
    std::initializer_list<std::pair<std::size_t, Status>> entries) {
    mippp::detail::handle_status_table<Status> table(id_bound);
    for(const auto & [id, status] : entries) table.set(id, status);
    return table;
}

}  // namespace

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Concepts /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(std::derived_from<iis_status::member_lower, iis_status::member>);
static_assert(std::derived_from<iis_status::member_upper, iis_status::member>);
static_assert(std::derived_from<iis_status::member_both, iis_status::member>);
static_assert(!std::derived_from<iis_status::member, iis_status::absent>);
static_assert(
    !std::derived_from<iis_status::member_both, iis_status::member_lower>);

static_assert(lp_iis_status<sided_status>);
static_assert(lp_iis_status<row_status>);
static_assert(lp_iis_status<const sided_status &>);
static_assert(!lp_iis_status<std::variant<iis_status::member_lower,
                                          iis_status::member_upper>>);
static_assert(!lp_iis_status<std::variant<iis_status::absent>>);
static_assert(!lp_iis_status<iis_status::member>);

static_assert(
    std::derived_from<iis_outcome::irreducible, iis_outcome::completed>);
static_assert(std::derived_from<iis_outcome::feasible, iis_outcome::completed>);
static_assert(
    !std::derived_from<iis_outcome::incomplete, iis_outcome::completed>);
static_assert(std::derived_from<iis_outcome::inconclusive_trial,
                                iis_outcome::incomplete>);
static_assert(
    !std::derived_from<iis_outcome::inconclusive_trial, iis_outcome::stopped>);
static_assert(
    std::derived_from<iis_outcome::interrupted, iis_outcome::stopped>);
static_assert(
    !std::derived_from<iis_outcome::interrupted, iis_outcome::limit_reached>);
static_assert(
    std::derived_from<iis_outcome::solve_limit, iis_outcome::limit_reached>);
static_assert(
    std::derived_from<iis_outcome::limit_reached, iis_outcome::incomplete>);
// a status tag reused here would let solve-status code read a conflict as a
// primal point
static_assert(!std::derived_from<iis_outcome::time_limit, status::any>);
static_assert(!variant_of<outcome, status::any>);

static_assert(iis_outcome::irreducible{}.conflict_available);
static_assert(!iis_outcome::feasible{}.conflict_available);
static_assert(is<iis_outcome::incomplete>(outcome{}));
static_assert(!iis_outcome::conflict_available(outcome{}));
static_assert(iis_outcome::conflict_available(outcome{
    iis_outcome::time_limit(true)}));
static_assert(!iis_outcome::conflict_available(outcome{
    iis_outcome::time_limit(false)}));

static_assert(lp_iis_outcome<outcome>);
static_assert(lp_iis_outcome<const outcome &>);
static_assert(!lp_iis_outcome<
              std::variant<iis_outcome::incomplete, iis_outcome::irreducible>>);
static_assert(!lp_iis_outcome<
              std::variant<iis_outcome::irreducible, iis_outcome::feasible>>);
// the exact incomplete tag: a path lists it even when it names every stop
static_assert(
    !lp_iis_outcome<std::variant<iis_outcome::stopped, iis_outcome::irreducible,
                                 iis_outcome::feasible>>);
static_assert(!lp_iis_outcome<iis_outcome::irreducible>);
static_assert(!lp_iis_outcome<std::variant<status::unknown, status::optimal,
                                           status::infeasible>>);
static_assert(!lp_iis_outcome<
              std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                           iis_outcome::feasible, status::time_limit>>);

static_assert(lp_iis<sided_snapshot, handles_only_model>);
static_assert(lp_iis<row_snapshot, handles_only_model>);
static_assert(!lp_iis<sided_snapshot, other_handles_model>);
static_assert(lp_iis<answer<flaw::none>, handles_only_model>);
static_assert(!lp_iis<answer<flaw::unsided_status>, handles_only_model>);
static_assert(!lp_iis<answer<flaw::mutable_status>, handles_only_model>);
static_assert(!lp_iis<answer<flaw::no_outcome>, handles_only_model>);
static_assert(!lp_iis<answer<flaw::bare_outcome>, handles_only_model>);
static_assert(!lp_iis<answer<flaw::int_variable_count>, handles_only_model>);
static_assert(!lp_iis<answer<flaw::int_constraint_count>, handles_only_model>);

// on each path's variant
using whole_or_sided = mippp::detail::iis_whole_or_sided_status;
using whole_or_one_side = mippp::detail::iis_whole_or_one_side_status;
constexpr iis_sides lower_side{.lower = true};
constexpr iis_sides upper_side{.upper = true};
constexpr iis_sides both_sides{.lower = true, .upper = true};
constexpr iis_sides one_unit{.lower = true, .upper = true, .whole = true};
static_assert(sides_of_tag<sided_status, iis_status::absent>() == iis_sides{});
static_assert(sides_of_tag<sided_status, iis_status::member_lower>() ==
              lower_side);
static_assert(sides_of_tag<sided_status, iis_status::member_upper>() ==
              upper_side);
static_assert(sides_of_tag<sided_status, iis_status::member_both>() ==
              both_sides);
static_assert(sides_of_tag<whole_or_sided, iis_status::absent>() ==
              iis_sides{});
static_assert(sides_of_tag<whole_or_sided, iis_status::member>() == one_unit);
static_assert(sides_of_tag<whole_or_sided, iis_status::member_lower>() ==
              lower_side);
static_assert(sides_of_tag<whole_or_sided, iis_status::member_upper>() ==
              upper_side);
static_assert(sides_of_tag<whole_or_sided, iis_status::member_both>() ==
              both_sides);
static_assert(sides_of_tag<whole_or_one_side, iis_status::absent>() ==
              iis_sides{});
static_assert(sides_of_tag<whole_or_one_side, iis_status::member>() ==
              one_unit);
static_assert(sides_of_tag<whole_or_one_side, iis_status::member_lower>() ==
              lower_side);
static_assert(sides_of_tag<whole_or_one_side, iis_status::member_upper>() ==
              upper_side);
static_assert(sides_of_tag<std::variant<iis_status::absent, possibly_lower,
                                        iis_status::member>,
                           possibly_lower>() == lower_side);
static_assert(noexcept(iis_status::sides_of(sided_status{})));

///////////////////////////////////////////////////////////////////////////////
//////////////////////////// Native decoding helpers //////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(std::holds_alternative<iis_status::member_upper>(
    mippp::detail::iis_row_status_by_sense<'L', 'G'>('L')));
static_assert(std::holds_alternative<iis_status::member_lower>(
    mippp::detail::iis_row_status_by_sense<'L', 'G'>('G')));
static_assert(std::holds_alternative<iis_status::member>(
    mippp::detail::iis_row_status_by_sense<'L', 'G'>('E')));
static_assert(std::holds_alternative<iis_status::member>(
    mippp::detail::iis_row_status_by_sense<'L', 'G'>('R')));

// whole asks for a bare member, which a status without one answers by sides
static_assert(std::holds_alternative<iis_status::member_both>(
    mippp::detail::iis_flagged_status<sided_status>(true, true, true)));
static_assert(std::holds_alternative<iis_status::member_lower>(
    mippp::detail::iis_flagged_status<sided_status>(true, false, true)));
static_assert(std::holds_alternative<iis_status::member>(
    mippp::detail::iis_flagged_status<mippp::detail::iis_whole_or_sided_status>(
        false, true, true)));
static_assert(std::holds_alternative<iis_status::member_upper>(
    mippp::detail::iis_flagged_status<mippp::detail::iis_whole_or_sided_status>(
        false, true, false)));

static_assert(std::same_as<model_iis_t<native_model>, sided_snapshot>);
static_assert(has_iis<native_model>);
static_assert(!has_iis<handles_only_model>);
static_assert(!has_iis<unsided_answer_model>);

static_assert(
    snapshot_accepts<variable, constraint, sided_status, row_status, outcome>);
static_assert(!snapshot_accepts<
              variable, constraint,
              std::variant<iis_status::member_lower, iis_status::absent>,
              sided_status, outcome>);
static_assert(!snapshot_accepts<
              variable, constraint, sided_status,
              std::variant<iis_status::member, iis_status::absent>, outcome>);
static_assert(
    !snapshot_accepts<variable, variable, sided_status, sided_status, outcome>);
// a value-initialized outcome must claim nothing
static_assert(!snapshot_accepts<
              variable, constraint, sided_status, sided_status,
              std::variant<iis_outcome::irreducible, iis_outcome::incomplete,
                           iis_outcome::feasible>>);
static_assert(
    !snapshot_accepts<
        variable, constraint, sided_status, sided_status,
        std::variant<iis_outcome::inconclusive_trial, iis_outcome::incomplete,
                     iis_outcome::irreducible, iis_outcome::feasible>>);
static_assert(
    !snapshot_accepts<
        variable, constraint, sided_status, sided_status,
        std::variant<status::unknown, status::optimal, status::infeasible>>);

static_assert(table_accepts<row_status, iis_status::member>);
static_assert(table_accepts<row_status, row_status>);
static_assert(!table_accepts<row_status, iis_status::member_both>);
static_assert(!table_accepts<sided_status, possibly_lower>);
// a tag no alternative derives from is a mistake, not a count of 0
static_assert(table_counts<row_status, iis_status::member>);
static_assert(!table_counts<row_status, iis_status::member_both>);
static_assert([] {
    mippp::detail::handle_status_table<row_status> table(3);
    table.set(1, iis_status::member{});
    return table.count_a<iis_status::member>() == 1 &&
           is<iis_status::member>(table.get(1)) &&
           is<iis_status::absent>(table.get(7));
}());

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Status table ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(handle_status_table, a_fresh_table_reads_the_first_alternative) {
    const mippp::detail::handle_status_table<sided_status> table(3);
    ASSERT_EQ(table.id_bound(), 3u);
    for(std::size_t id = 0; id < 3; ++id)
        ASSERT_TRUE(is<iis_status::absent>(table.get(id)));
    ASSERT_EQ(table.count_a<iis_status::member>(), 0u);
}

TEST(handle_status_table, reads_back_each_alternative) {
    const auto table =
        make_table<sided_status>(4, {{1, iis_status::member_lower{}},
                                     {2, iis_status::member_upper{}},
                                     {3, iis_status::member_both{}}});
    ASSERT_TRUE(is<iis_status::absent>(table.get(0)));
    ASSERT_TRUE(is<iis_status::member_lower>(table.get(1)));
    ASSERT_TRUE(is<iis_status::member_upper>(table.get(2)));
    ASSERT_TRUE(is<iis_status::member_both>(table.get(3)));
}

TEST(handle_status_table, a_later_set_overwrites) {
    auto table = make_table<sided_status>(2, {{1, iis_status::member_lower{}}});
    table.set(1, iis_status::absent{});
    ASSERT_TRUE(is<iis_status::absent>(table.get(1)));
    ASSERT_EQ(table.count_a<iis_status::member>(), 0u);
}

TEST(handle_status_table, ids_past_the_bound_read_the_first_alternative) {
    const auto table =
        make_table<sided_status>(2, {{1, iis_status::member_both{}}});
    ASSERT_TRUE(is<iis_status::absent>(table.get(2)));
    ASSERT_TRUE(is<iis_status::absent>(table.get(1000)));
    const mippp::detail::handle_status_table<sided_status> empty;
    ASSERT_EQ(empty.id_bound(), 0u);
    ASSERT_TRUE(is<iis_status::absent>(empty.get(0)));
}

TEST(handle_status_table, set_past_the_bound_throws) {
    mippp::detail::handle_status_table<sided_status> table(2);
    ASSERT_THROW(table.set(2, iis_status::member_lower{}), std::out_of_range);
    ASSERT_EQ(table.id_bound(), 2u);
}

TEST(handle_status_table, count_a_counts_every_derived_alternative) {
    const auto table =
        make_table<row_status>(6, {{0, iis_status::member_lower{}},
                                   {2, iis_status::member{}},
                                   {3, iis_status::member_upper{}},
                                   {5, iis_status::member_upper{}}});
    ASSERT_EQ(table.count_a<iis_status::member>(), 4u);
    ASSERT_EQ(table.count_a<iis_status::member_upper>(), 2u);
    ASSERT_EQ(table.count_a<iis_status::absent>(), 2u);
}

TEST(handle_status_table, holds_basis_statuses) {
    using basis_variant =
        std::variant<basis_status::nonbasic_at_lower_bound, basis_status::basic,
                     basis_status::nonbasic_at_upper_bound>;
    const auto table = make_table<basis_variant>(
        3, {{0, basis_status::basic{}},
            {2, basis_status::nonbasic_at_upper_bound{}}});
    ASSERT_TRUE(is<basis_status::basic>(table.get(0)));
    ASSERT_TRUE(is<basis_status::nonbasic_at_lower_bound>(table.get(1)));
    ASSERT_TRUE(is<basis_status::nonbasic_at_lower_bound>(table.get(7)));
    ASSERT_EQ(table.count_a<basis_status::nonbasic>(), 2u);
}

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Snapshot /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(iis_snapshot, keys_variables_and_constraints_apart) {
    const sided_snapshot iis(
        make_table<sided_status>(2, {{0, iis_status::member_lower{}}}),
        make_table<sided_status>(2, {{1, iis_status::member_upper{}}}),
        iis_outcome::irreducible{});
    ASSERT_TRUE(is<iis_status::member_lower>(iis.get_status(variable(0))));
    ASSERT_TRUE(is<iis_status::absent>(iis.get_status(variable(1))));
    ASSERT_TRUE(is<iis_status::absent>(iis.get_status(constraint(0))));
    ASSERT_TRUE(is<iis_status::member_upper>(iis.get_status(constraint(1))));
}

TEST(iis_snapshot, sided_tags_are_members) {
    const sided_snapshot iis(
        make_table<sided_status>(3, {{0, iis_status::member_lower{}},
                                     {1, iis_status::member_upper{}},
                                     {2, iis_status::member_both{}}}),
        {}, iis_outcome::irreducible{});
    for(int id = 0; id < 3; ++id)
        ASSERT_TRUE(is_a<iis_status::member>(iis.get_status(variable(id))));
}

TEST(iis_snapshot, an_unsided_row_is_a_member) {
    const row_snapshot iis(
        {},
        make_table<row_status>(
            2, {{0, iis_status::member{}}, {1, iis_status::member_lower{}}}),
        iis_outcome::irreducible{});
    ASSERT_TRUE(is<iis_status::member>(iis.get_status(constraint(0))));
    ASSERT_FALSE(is_a<iis_status::member_lower>(iis.get_status(constraint(0))));
    ASSERT_TRUE(is_a<iis_status::member>(iis.get_status(constraint(1))));
    ASSERT_FALSE(is<iis_status::member>(iis.get_status(constraint(1))));
    ASSERT_EQ(iis.num_constraint_members(), 2u);
}

TEST(iis_snapshot, counts_members_per_entity_kind) {
    const row_snapshot iis(
        make_table<sided_status>(5, {{0, iis_status::member_both{}},
                                     {3, iis_status::member_lower{}},
                                     {4, iis_status::member_upper{}}}),
        make_table<row_status>(
            4, {{1, iis_status::member{}}, {2, iis_status::member_lower{}}}),
        iis_outcome::irreducible{});
    ASSERT_EQ(iis.num_variable_members(), 3u);
    ASSERT_EQ(iis.num_constraint_members(), 2u);
}

TEST(iis_snapshot, reports_the_outcome) {
    const sided_snapshot complete({}, {}, iis_outcome::feasible{});
    ASSERT_TRUE(outcome_is<iis_outcome::feasible>(complete.get_outcome()));
    ASSERT_EQ(complete.num_variable_members(), 0u);
    ASSERT_EQ(complete.num_constraint_members(), 0u);

    const sided_snapshot stopped(
        make_table<sided_status>(1, {{0, iis_status::member_lower{}}}), {},
        iis_outcome::time_limit(true));
    ASSERT_TRUE(
        outcome_is<iis_outcome::time_limit>(stopped.get_outcome(), true));
}

TEST(iis_snapshot, handles_past_the_bound_are_absent) {
    const sided_snapshot iis(
        make_table<sided_status>(1, {{0, iis_status::member_both{}}}),
        make_table<sided_status>(1, {{0, iis_status::member_lower{}}}),
        iis_outcome::irreducible{});
    ASSERT_TRUE(is<iis_status::absent>(iis.get_status(variable(1))));
    ASSERT_TRUE(is<iis_status::absent>(iis.get_status(constraint(5))));
}

TEST(iis_snapshot, a_refinement_tag_keeps_the_unified_reading) {
    using refined_status =
        std::variant<iis_status::absent, iis_status::member_lower,
                     possibly_lower, iis_status::member_upper>;
    const iis_snapshot<variable, constraint, refined_status, sided_status,
                       outcome>
        iis(make_table<refined_status>(
                2, {{0, possibly_lower{}}, {1, iis_status::member_lower{}}}),
            {}, iis_outcome::incomplete(true));
    ASSERT_TRUE(is<possibly_lower>(iis.get_status(variable(0))));
    ASSERT_TRUE(is_a<iis_status::member_lower>(iis.get_status(variable(0))));
    ASSERT_FALSE(is<possibly_lower>(iis.get_status(variable(1))));
    ASSERT_EQ(iis.num_variable_members(), 2u);
}

TEST(iis_snapshot, copies_answer_alike) {
    const sided_snapshot original(
        make_table<sided_status>(2, {{1, iis_status::member_both{}}}), {},
        iis_outcome::irreducible{});
    const sided_snapshot copy = original;
    ASSERT_TRUE(is<iis_status::member_both>(copy.get_status(variable(1))));
    ASSERT_EQ(copy.num_variable_members(), 1u);
}

TEST(iis_answer, flags_members_and_hands_the_tables_over) {
    using whole_row_snapshot =
        iis_snapshot<variable, constraint, sided_status,
                     mippp::detail::iis_whole_or_sided_status, outcome>;
    mippp::detail::iis_answer<whole_row_snapshot> answer(3, 3);
    answer.flag_variable(0, true, false);
    answer.flag_variable(1, false, false, true);
    answer.flag_variable(2, true, true, true);
    answer.flag_constraint(0, false, true, true);
    answer.flag_constraint(1, false, true);
    answer.constraints.set(2, iis_status::member{});
    const whole_row_snapshot iis = answer.finish(iis_outcome::time_limit(true));
    EXPECT_TRUE(is<iis_status::member_lower>(iis.get_status(variable(0))));
    EXPECT_TRUE(is<iis_status::absent>(iis.get_status(variable(1))));
    EXPECT_TRUE(is<iis_status::member_both>(iis.get_status(variable(2))));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(constraint(0))));
    EXPECT_TRUE(is<iis_status::member_upper>(iis.get_status(constraint(1))));
    EXPECT_TRUE(is<iis_status::member>(iis.get_status(constraint(2))));
    EXPECT_EQ(iis.num_variable_members(), 2u);
    EXPECT_EQ(iis.num_constraint_members(), 3u);
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(iis.get_outcome(), true));
}

TEST(iis_outcome_assert, matches_the_exact_tag_and_names_the_actual_one) {
    const outcome stopped = iis_outcome::time_limit(true);
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(stopped));
    EXPECT_TRUE(outcome_is<iis_outcome::time_limit>(stopped, true));
    EXPECT_FALSE(outcome_is<iis_outcome::time_limit>(stopped, false));
    EXPECT_FALSE(outcome_is<iis_outcome::incomplete>(stopped));
    EXPECT_STREQ(outcome_is<iis_outcome::irreducible>(stopped).message(),
                 "the outcome is time_limit (conflict held), not irreducible");
    EXPECT_STREQ(
        outcome_is<iis_outcome::incomplete>(outcome{}, true).message(),
        "the outcome is incomplete (no conflict), not incomplete (conflict "
        "held)");
    EXPECT_TRUE(::testing::PrintToString(stopped).ends_with(
        "with value time_limit (conflict held))"));
}

// A tag without a name of its own would print its base's.
TEST(iis_outcome_assert, names_every_tag) {
    using every_tag =
        std::variant<iis_outcome::incomplete, iis_outcome::irreducible,
                     iis_outcome::feasible, iis_outcome::inconclusive_trial,
                     iis_outcome::stopped, iis_outcome::interrupted,
                     iis_outcome::limit_reached, iis_outcome::time_limit,
                     iis_outcome::solve_limit, iis_outcome::iteration_limit,
                     iis_outcome::node_limit, iis_outcome::memory_limit>;
    const auto describe = [](every_tag o) {
        return iis_outcome_assert_detail::describe(o);
    };
    EXPECT_EQ(describe(iis_outcome::incomplete(true)),
              "incomplete (conflict held)");
    EXPECT_EQ(describe(iis_outcome::irreducible{}), "irreducible");
    EXPECT_EQ(describe(iis_outcome::feasible{}), "feasible");
    EXPECT_EQ(describe(iis_outcome::inconclusive_trial{}),
              "inconclusive_trial (no conflict)");
    EXPECT_EQ(describe(iis_outcome::stopped{}), "stopped (no conflict)");
    EXPECT_EQ(describe(iis_outcome::interrupted{}),
              "interrupted (no conflict)");
    EXPECT_EQ(describe(iis_outcome::limit_reached{}),
              "limit_reached (no conflict)");
    EXPECT_EQ(describe(iis_outcome::time_limit{}), "time_limit (no conflict)");
    EXPECT_EQ(describe(iis_outcome::solve_limit{}),
              "solve_limit (no conflict)");
    EXPECT_EQ(describe(iis_outcome::iteration_limit{}),
              "iteration_limit (no conflict)");
    EXPECT_EQ(describe(iis_outcome::node_limit{}), "node_limit (no conflict)");
    EXPECT_EQ(describe(iis_outcome::memory_limit{}),
              "memory_limit (no conflict)");
}
