#include <gtest/gtest.h>

#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <variant>

#include "mippp/detail/handle_status_table.hpp"
#include "mippp/model_concepts.hpp"
#include "mippp/model_entities.hpp"
#include "mippp/utility/basis_snapshot.hpp"
#include "mippp/utility/variant.hpp"

using namespace mippp;

namespace {

using variable = model_variable<int, double>;
using constraint = model_constraint<int>;
using mippp::detail::basis_kind;

// the surface model_variable_t and model_constraint_t deduce from
struct handles_only_model {
    variable add_variable();
    template <typename LC>
    constraint add_constraint(LC &&);
};

// a backend that names no fixed entity and no superbasic one, as HiGHS
using column_status = std::variant<basis_status::nonbasic, basis_status::basic,
                                   basis_status::nonbasic_at_lower_bound,
                                   basis_status::nonbasic_at_upper_bound,
                                   basis_status::nonbasic_free>;
using row_status = std::variant<basis_status::basic, basis_status::nonbasic,
                                basis_status::nonbasic_at_lower_bound,
                                basis_status::nonbasic_at_upper_bound,
                                basis_status::nonbasic_fixed>;
using snapshot =
    basis_snapshot<variable, constraint, column_status, row_status>;

struct reading_model : handles_only_model {
    snapshot get_basis();
};

struct warm_model : reading_model {
    template <mippp::detail::lp_basis_over<variable, constraint> B>
    void set_basis(const B &);
};

// accepts its own snapshot only, not any lp_basis
struct narrow_warm_model : reading_model {
    void set_basis(const snapshot &);
};

struct mutable_reads {
    column_status get_status(variable);
    row_status get_status(constraint);
};
struct mutable_reads_model : handles_only_model {
    mutable_reads get_basis();
};

struct my_lower : basis_status::nonbasic_at_lower_bound {};
struct unrelated {};

template <typename V, typename C, typename VS, typename CS>
concept snapshot_accepts = requires { typename basis_snapshot<V, C, VS, CS>; };

template <typename Snapshot, typename Handle, typename Tag>
concept writes =
    requires(Snapshot & s, Handle h, Tag t) { s.set_status(h, t); };

template <typename Status>
constexpr basis_kind column_kind_of(Status status) {
    return mippp::detail::basis_kind_of<basis_kind::nonbasic>(status);
}

}  // namespace

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Tags /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(std::derived_from<basis_status::nonbasic_fixed,
                                basis_status::nonbasic_at_bound>);
static_assert(std::derived_from<basis_status::nonbasic_at_lower_bound,
                                basis_status::nonbasic_at_bound>);
static_assert(
    std::derived_from<basis_status::nonbasic_free, basis_status::superbasic>);
static_assert(
    std::derived_from<basis_status::superbasic, basis_status::nonbasic>);
static_assert(!std::derived_from<basis_status::nonbasic_free,
                                 basis_status::nonbasic_at_bound>);
static_assert(!std::derived_from<basis_status::nonbasic, basis_status::basic>);

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Concepts /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(lp_basis_status<column_status>);
static_assert(!lp_basis_status<basis_status::basic>);
static_assert(!lp_basis_status<std::variant<basis_status::nonbasic_at_bound,
                                            basis_status::superbasic>>);

static_assert(lp_basis<snapshot, handles_only_model>);
static_assert(!lp_basis<mutable_reads, handles_only_model>);
static_assert(has_lp_basis<reading_model>);
static_assert(has_modifiable_lp_basis<reading_model>);
static_assert(!has_lp_basis_warm_start<reading_model>);
static_assert(has_lp_basis_warm_start<warm_model>);
static_assert(!has_lp_basis_warm_start<narrow_warm_model>);
static_assert(!has_lp_basis<mutable_reads_model>);
static_assert(!has_lp_basis<handles_only_model>);
static_assert(std::same_as<model_basis_t<warm_model>, snapshot>);
static_assert(
    std::same_as<model_variable_basis_status_t<warm_model>, column_status>);
static_assert(
    std::same_as<model_constraint_basis_status_t<warm_model>, row_status>);

static_assert(
    snapshot_accepts<variable, constraint, column_status, row_status>);
// alternative 0 is the status of an entity the snapshot does not cover
static_assert(!snapshot_accepts<variable, constraint, row_status, row_status>);
static_assert(
    !snapshot_accepts<variable, constraint, column_status, column_status>);
static_assert(!snapshot_accepts<variable, variable, column_status, row_status>);
// basic and nonbasic are what an unlisted tag falls back to
static_assert(
    !snapshot_accepts<variable, constraint,
                      std::variant<basis_status::nonbasic,
                                   basis_status::nonbasic_at_lower_bound>,
                      row_status>);

static_assert(writes<snapshot, variable, basis_status::nonbasic_fixed>);
static_assert(writes<snapshot, constraint, basis_status::superbasic>);
static_assert(writes<snapshot, variable, my_lower>);
static_assert(!writes<snapshot, variable, unrelated>);

///////////////////////////////////////////////////////////////////////////////
////////////////////////////// Storage policy /////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// a listed tag is stored as given, an unlisted one as its nearest listed base
static_assert(is<basis_status::nonbasic_free>(
    mippp::detail::basis_stored_status<column_status>(
        basis_status::nonbasic_free{})));
static_assert(is<basis_status::nonbasic>(
    mippp::detail::basis_stored_status<column_status>(
        basis_status::nonbasic_fixed{})));
static_assert(is<basis_status::nonbasic>(
    mippp::detail::basis_stored_status<column_status>(
        basis_status::superbasic{})));
static_assert(
    is<basis_status::nonbasic>(mippp::detail::basis_stored_status<row_status>(
        basis_status::nonbasic_free{})));
static_assert(is<basis_status::nonbasic_at_lower_bound>(
    mippp::detail::basis_stored_status<column_status>(my_lower{})));
static_assert(is<basis_status::nonbasic>(
    mippp::detail::basis_stored_status<column_status>(
        basis_status::nonbasic_at_bound{})));

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Classifier ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(column_kind_of(column_status(basis_status::basic{})) ==
              basis_kind::basic);
static_assert(column_kind_of(column_status(basis_status::nonbasic{})) ==
              basis_kind::nonbasic);
static_assert(column_kind_of(row_status(basis_status::nonbasic_fixed{})) ==
              basis_kind::nonbasic_fixed);
static_assert(column_kind_of(column_status(basis_status::nonbasic_free{})) ==
              basis_kind::nonbasic_free);
// a refinement writes as the tag it refines
static_assert(column_kind_of(std::variant<basis_status::basic, my_lower>(
                  my_lower{})) == basis_kind::nonbasic_at_lower_bound);
static_assert(column_kind_of(std::variant<basis_status::basic,
                                          basis_status::nonbasic_at_bound>(
                  basis_status::nonbasic_at_bound{})) == basis_kind::nonbasic);
static_assert(
    column_kind_of(std::variant<basis_status::basic, basis_status::superbasic>(
        basis_status::superbasic{})) == basis_kind::superbasic);
// an alternative that is no basis tag writes as the entity's default
static_assert(column_kind_of(std::variant<basis_status::basic,
                                          basis_status::nonbasic, unrelated>(
                  unrelated{})) == basis_kind::nonbasic);
static_assert(mippp::detail::basis_kind_of<basis_kind::basic>(
                  std::variant<basis_status::basic, basis_status::nonbasic,
                               unrelated>(unrelated{})) == basis_kind::basic);

///////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Snapshot /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

TEST(basis_snapshot, default_is_the_slack_basis) {
    const snapshot basis;
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(variable(0))));
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(variable(9))));
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(constraint(0))));
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(constraint(9))));
}

TEST(basis_snapshot, keys_variables_and_constraints_apart) {
    mippp::detail::handle_status_table<column_status> columns(2);
    columns.set(1, basis_status::nonbasic_at_upper_bound{});
    mippp::detail::handle_status_table<row_status> rows(2);
    rows.set(0, basis_status::nonbasic_fixed{});
    const snapshot basis(std::move(columns), std::move(rows));
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(variable(0))));
    ASSERT_TRUE(is<basis_status::nonbasic_at_upper_bound>(
        basis.get_status(variable(1))));
    ASSERT_TRUE(
        is<basis_status::nonbasic_fixed>(basis.get_status(constraint(0))));
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(constraint(1))));
    // past the tables: entities created after the snapshot
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(variable(2))));
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(constraint(2))));
}

TEST(basis_snapshot, set_status_grows_and_stores_what_the_status_lists) {
    snapshot basis;
    basis.set_status(variable(4), basis_status::basic{});
    basis.set_status(variable(1), basis_status::nonbasic_fixed{});
    basis.set_status(constraint(3), basis_status::nonbasic_at_lower_bound{});
    basis.set_status(constraint(0), basis_status::nonbasic_free{});
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(variable(4))));
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(variable(1))));
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(variable(3))));
    ASSERT_TRUE(is<basis_status::nonbasic_at_lower_bound>(
        basis.get_status(constraint(3))));
    ASSERT_TRUE(is<basis_status::nonbasic>(basis.get_status(constraint(0))));
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(constraint(2))));
}

TEST(basis_snapshot, copies_alike) {
    snapshot basis;
    basis.set_status(variable(2), basis_status::basic{});
    snapshot copy = basis;
    copy.set_status(variable(2), basis_status::nonbasic_free{});
    ASSERT_TRUE(is<basis_status::basic>(basis.get_status(variable(2))));
    ASSERT_TRUE(is<basis_status::nonbasic_free>(copy.get_status(variable(2))));
}

TEST(handle_status_table, grow_and_set_reads_alternative_0_in_between) {
    mippp::detail::handle_status_table<row_status> table(1);
    table.grow_and_set(3, basis_status::nonbasic_fixed{});
    ASSERT_EQ(table.id_bound(), 4u);
    ASSERT_TRUE(is<basis_status::basic>(table.get(1)));
    ASSERT_TRUE(is<basis_status::nonbasic_fixed>(table.get(3)));
    table.grow_and_set(0, basis_status::nonbasic{});
    ASSERT_EQ(table.id_bound(), 4u);
    ASSERT_TRUE(is<basis_status::nonbasic>(table.get(0)));
}
