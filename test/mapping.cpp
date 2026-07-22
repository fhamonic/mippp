#undef NDEBUG
#include <gtest/gtest.h>

#include <concepts>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "mippp/mapping.hpp"

using namespace mippp;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// Concepts ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(mapping<std::vector<double>, int>);
static_assert(input_mapping_of<std::vector<double>, int, double>);
static_assert(output_mapping_of<std::vector<double>, int, double>);
static_assert(contiguous_mapping_of<std::vector<double>, int, double>);

// std::map subscripts but has no const operator[]: a mapping, yet not an
// input_mapping until adapted (the adapted view reads through .at()).
static_assert(mapping<std::map<int, double>, int>);
static_assert(!input_mapping<std::map<int, double>, int>);
static_assert(input_mapping_of<views::mapping_all_t<std::map<int, double> &>,
                               int, double>);

// a callable is not a mapping by itself; views::map lifts it into one
static_assert(!mapping<std::identity, int>);
static_assert(
    input_mapping_of<decltype(views::map(std::identity{})), int, int>);

// unique_ptr<T[]> subscripts but exposes no .data(): input, not contiguous
static_assert(input_mapping_of<std::unique_ptr<double[]>, int, double>);
static_assert(!contiguous_mapping<std::unique_ptr<double[]>, int>);

// reading through a const view must yield references, not decayed copies
using heavy_owning = views::mapping_all_t<std::vector<std::string>>;
static_assert(std::same_as<decltype(std::declval<const heavy_owning &>()[0u]),
                           const std::string &>);
static_assert(std::same_as<decltype(std::declval<heavy_owning &>()[0u]),
                           std::string &>);
// ref views are shallow-const: constness is carried by the Map type
using heavy_ref = views::mapping_all_t<std::vector<std::string> &>;
using heavy_const_ref = views::mapping_all_t<const std::vector<std::string> &>;
static_assert(std::same_as<decltype(std::declval<const heavy_ref &>()[0u]),
                           std::string &>);
static_assert(
    std::same_as<decltype(std::declval<const heavy_const_ref &>()[0u]),
                 const std::string &>);

// owning views refuse lvalues and ref views refuse rvalues
static_assert(!std::constructible_from<mapping_owning_view<std::vector<double>>,
                                       std::vector<double> &>);
static_assert(!std::constructible_from<mapping_ref_view<std::vector<double>>,
                                       std::vector<double>>);

///////////////////////////////////////////////////////////////////////////////
/////////////////////////////// views::mapping_all ////////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(mapping_all, lvalue_gives_ref_view_with_write_through) {
    std::vector<double> values{1.0, 2.0};
    auto view = views::mapping_all(values);
    static_assert(
        std::same_as<decltype(view), mapping_ref_view<std::vector<double>>>);
    static_assert(output_mapping_of<decltype(view), std::size_t, double>);
    view[0u] = 3.0;
    ASSERT_DOUBLE_EQ(values[0], 3.0);
    ASSERT_DOUBLE_EQ(view[1u], 2.0);
    ASSERT_EQ(view.data(), values.data());
}

GTEST_TEST(mapping_all, rvalue_gives_owning_view) {
    auto view = views::mapping_all(std::vector<double>{1.0, 2.0});
    static_assert(std::same_as<decltype(view),
                               mapping_owning_view<std::vector<double>>>);
    view[1u] = 5.0;
    ASSERT_DOUBLE_EQ(view[1u], 5.0);
    ASSERT_DOUBLE_EQ(view.data()[0], 1.0);
}

GTEST_TEST(mapping_all, unique_ptr_array_subscript_branch) {
    auto values = std::make_unique_for_overwrite<double[]>(3);
    values[2] = 7.0;
    auto view = views::mapping_all(std::move(values));
    ASSERT_DOUBLE_EQ(view[2u], 7.0);
}

GTEST_TEST(mapping_all, std_map_branches) {
    std::map<int, double> values{{4, 2.5}};
    // non-const map: subscript branch, std::map insert-on-miss semantics
    auto view = views::mapping_all(values);
    ASSERT_DOUBLE_EQ(view[4], 2.5);
    view[5] = 1.5;
    ASSERT_EQ(values.size(), 2u);
    // const map has no operator[]: reads go through the .at() branch
    const auto cview = views::mapping_all(std::as_const(values));
    ASSERT_DOUBLE_EQ(cview[5], 1.5);
    ASSERT_THROW((void)cview[6], std::out_of_range);
}

GTEST_TEST(mapping_all, callable_branch) {
    const auto view = views::mapping_all([](int i) { return 2 * i; });
    ASSERT_EQ(view[21], 42);
}

GTEST_TEST(mapping_all, view_pass_through_is_identity) {
    auto view = views::mapping_all(std::vector<double>{1.0});
    auto view2 = views::mapping_all(std::move(view));
    static_assert(std::same_as<decltype(view2), decltype(view)>);
    auto view3 = views::mapping_all(view2);  // lvalue view: cheap copy
    static_assert(std::same_as<decltype(view3), decltype(view)>);
    ASSERT_DOUBLE_EQ(view3[0u], 1.0);
}

///////////////////////////////////////////////////////////////////////////////
////////////////// owning views of capturing lambdas are movable //////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(mapping_owning_view, capturing_lambda_is_movable_view) {
    auto view =
        views::mapping_all([factor = 3.0](int i) { return factor * i; });
    using view_t = decltype(view);
    static_assert(std::movable<view_t>);
    static_assert(mapping_view<view_t, int>);

    // move assignment reconstructs the non-assignable lambda in place
    view_t other = view;
    other = std::move(view);
    ASSERT_DOUBLE_EQ(other[2], 6.0);

    // pass-through instead of re-wrapping, now that the concept is satisfied
    auto again = views::mapping_all(std::move(other));
    static_assert(std::same_as<decltype(again), view_t>);
    ASSERT_DOUBLE_EQ(again[3], 9.0);
}

GTEST_TEST(mapping_owning_view, copy_assignment_reconstructs) {
    auto storage = std::vector<double>{1.5, 2.5};
    auto view =
        views::map([storage](int i) { return storage[std::size_t(i)]; });
    auto copy = view;
    copy = view;  // copy-assign a non-assignable closure
    ASSERT_DOUBLE_EQ(copy[1], 2.5);
}

GTEST_TEST(views_map, copies_lvalue_callable) {
    int hits = 0;
    auto counter = [&hits, offset = 10](int i) {
        ++hits;
        return offset + i;
    };
    auto view = views::map(counter);
    ASSERT_EQ(view[5], 15);
    ASSERT_EQ(counter(0), 10);
    ASSERT_EQ(hits, 2);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////// base() //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

GTEST_TEST(mapping_views, base_access) {
    std::vector<double> values{1.0};
    auto ref = views::mapping_all(values);
    ASSERT_EQ(&ref.base(), &values);

    auto owning = views::mapping_all(std::vector<double>{4.0});
    owning.base().push_back(8.0);
    ASSERT_DOUBLE_EQ(owning[1u], 8.0);
    auto recovered = std::move(owning).base();
    ASSERT_EQ(recovered.size(), 2u);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////// Utility mappings ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static_assert(views::true_map{}[42]);
static_assert(!views::false_map{}[42]);
static_assert(views::identity_map{}[7] == 7);
static_assert(mapping_view<views::true_map, int>);

GTEST_TEST(utility_maps, element_map_chains_tuple_access) {
    std::tuple<int, std::pair<int, double>> t{1, {2, 3.5}};
    ASSERT_EQ((views::element_map<0>{}[t]), 1);
    ASSERT_DOUBLE_EQ((views::element_map<1, 1>{}[t]), 3.5);
    views::element_map<1, 0>{}[t] = 9;  // decltype(auto): writable
    ASSERT_EQ(std::get<1>(t).first, 9);
}

///////////////////////////////////////////////////////////////////////////////
///////////////// entity_mapping: entity-keyed projection layer ///////////////
///////////////////////////////////////////////////////////////////////////////

#include "mippp/model_entities.hpp"

using Var = model_variable<int, double>;

GTEST_TEST(entity_mapping, uid_branch_write_through) {
    entity_mapping<Var, std::vector<double>> m = std::vector<double>{1.0, 2.0};
    m[Var(0)] = 3.0;
    ASSERT_DOUBLE_EQ(m[Var(0)], 3.0);
    const auto & cm = m;
    static_assert(std::same_as<decltype(cm[Var(0)]), const double &>);
    static_assert(output_mapping_of<decltype(m), Var, double>);
}

GTEST_TEST(entity_mapping, unique_ptr_array_uid_branch) {
    auto sol = std::make_unique_for_overwrite<double[]>(3);
    sol[2] = 7.0;
    entity_mapping<Var, std::unique_ptr<double[]>> m = std::move(sol);
    ASSERT_DOUBLE_EQ(m[Var(2)], 7.0);
}

GTEST_TEST(entity_mapping, callable_gets_the_entity_itself) {
    auto lam = [](const Var & v) { return 2.0 * v.id(); };
    entity_mapping<Var, decltype(lam)> m = std::move(lam);
    ASSERT_DOUBLE_EQ(m[Var(3)], 6.0);
    static_assert(input_mapping_of<decltype(m), Var, double>);
}

GTEST_TEST(entity_mapping, associative_map_keyed_by_entity) {
    std::map<Var, double> values;
    values[Var(4)] = 2.5;
    entity_mapping<Var, std::map<Var, double>> m = std::move(values);
    const auto & cm = m;
    ASSERT_DOUBLE_EQ(cm[Var(4)], 2.5);  // const: .at() branch
    m[Var(5)] = 1.5;                    // non-const: subscript, inserts
    ASSERT_DOUBLE_EQ(cm[Var(5)], 1.5);
}
