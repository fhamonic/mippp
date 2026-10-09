// The code of docs/solving/basis.md. The page shows the sections between the
// --8<-- markers, so they stay plain user code; the tests below them check
// what the page says about their output.

#include <gtest/gtest.h>

#include <concepts>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <string_view>
#include <variant>

#include "mippp/solvers/clp/all.hpp"
// --8<-- [start:includes]
#include "mippp/solvers/highs/all.hpp"
// --8<-- [end:includes]

using namespace mippp;

#include "test_suites/all.hpp"

namespace basis_page {

// here rather than in each function: on MSVC a block-scope using-directive
// does not reach into a generic lambda
using namespace mippp::operators;

// --8<-- [start:describe]
// The most specific tag a status names, by derivation, so that it compiles
// on every backend's status variant.
template <typename Status>
std::string_view describe(const Status & status) {
    return std::visit(
        []<typename Tag>(const Tag &) -> std::string_view {
            if constexpr(std::derived_from<Tag, basis_status::basic>)
                return "basic";
            else if constexpr(std::derived_from<Tag,
                                                basis_status::nonbasic_fixed>)
                return "nonbasic, fixed";
            else if constexpr(std::derived_from<
                                  Tag, basis_status::nonbasic_at_lower_bound>)
                return "nonbasic, at its lower side";
            else if constexpr(std::derived_from<
                                  Tag, basis_status::nonbasic_at_upper_bound>)
                return "nonbasic, at its upper side";
            else if constexpr(std::derived_from<Tag, basis_status::superbasic>)
                return "nonbasic, at no bound";
            else
                return "nonbasic";
        },
        status);
}
// --8<-- [end:describe]

template <typename Model>
void workshop(Model & model) {
    // --8<-- [start:workshop-model]
    auto chairs = model.add_variable();
    auto tables = model.add_variable();
    auto desks = model.add_variable();
    auto overtime = model.add_variable({.lower_bound = 0, .upper_bound = 4});
    model.set_maximization();
    model.set_objective(4 * chairs + 7 * tables + desks - 3 * overtime);
    auto wood = model.add_constraint(2 * chairs + 4 * tables + 2 * desks <= 40);
    auto labour =
        model.add_constraint(chairs + 2 * tables + desks <= 18 + overtime);
    auto orders = model.add_constraint(tables >= 3);
    model.solve();
    // --8<-- [end:workshop-model]

    // --8<-- [start:workshop-basis]
    auto basis = model.get_basis();
    std::cout << "chairs:   " << describe(basis.get_status(chairs)) << '\n';
    std::cout << "tables:   " << describe(basis.get_status(tables)) << '\n';
    std::cout << "desks:    " << describe(basis.get_status(desks)) << '\n';
    std::cout << "overtime: " << describe(basis.get_status(overtime)) << '\n';
    std::cout << "wood:     " << describe(basis.get_status(wood)) << '\n';
    std::cout << "labour:   " << describe(basis.get_status(labour)) << '\n';
    std::cout << "orders:   " << describe(basis.get_status(orders)) << '\n';
    // --8<-- [end:workshop-basis]

    // --8<-- [start:workshop-stools]
    auto stools = model.add_column({{wood, 1.}, {labour, 1.}},
                                   {.obj_coef = 5, .lower_bound = 0});
    // stools did not exist when basis was taken: it reads nonbasic
    std::cout << "stools:   " << describe(basis.get_status(stools)) << '\n';
    model.set_basis(basis);
    model.solve();
    std::cout << "profit:   " << model.get_solution_value() << '\n';
    // --8<-- [end:workshop-stools]
}

// The workshop again, never solved, started from a basis of the caller's
// making: the plan without overtime, which breaks the labour row, so that
// the solve moves to the optimum.
template <typename Model>
double own_start(Model & model) {
    auto chairs = model.add_variable();
    auto tables = model.add_variable();
    auto desks = model.add_variable();
    auto overtime = model.add_variable({.lower_bound = 0, .upper_bound = 4});
    model.set_maximization();
    model.set_objective(4 * chairs + 7 * tables + desks - 3 * overtime);
    auto wood = model.add_constraint(2 * chairs + 4 * tables + 2 * desks <= 40);
    auto labour =
        model.add_constraint(chairs + 2 * tables + desks <= 18 + overtime);
    auto orders = model.add_constraint(tables >= 3);
    (void)desks;
    (void)labour;
    // --8<-- [start:own-start]
    model_basis_t<Model> start;  // the slack basis
    start.set_status(chairs, basis_status::basic{});
    start.set_status(tables, basis_status::basic{});
    start.set_status(wood, basis_status::nonbasic_at_upper_bound{});
    start.set_status(orders, basis_status::nonbasic_at_lower_bound{});
    model.set_basis(start);
    model.solve();
    // --8<-- [end:own-start]
    return model.get_solution_value();
}

}  // namespace basis_page

namespace {

// captures std::cout for one scope
class captured_cout {
    std::ostringstream _buffer;
    std::streambuf * _previous;

public:
    captured_cout() : _previous(std::cout.rdbuf(_buffer.rdbuf())) {}
    ~captured_cout() { std::cout.rdbuf(_previous); }
    std::string str() const { return _buffer.str(); }
};

constexpr std::string_view workshop_output =
    "chairs:   basic\n"
    "tables:   basic\n"
    "desks:    nonbasic, at its lower side\n"
    "overtime: basic\n"
    "wood:     nonbasic, at its upper side\n"
    "labour:   nonbasic, at its upper side\n"
    "orders:   nonbasic, at its lower side\n"
    "stools:   nonbasic\n"
    "profit:   89\n";

template <typename Fixture>
void check_workshop(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        std::string printed;
        {
            captured_cout capture;
            basis_page::workshop(model);
            printed = capture.str();
        }
        ASSERT_EQ(printed, workshop_output);
    });
}

}  // namespace

struct basis_page_highs_lp : model_test<highs_api, highs_lp> {
    static void SetUpTestSuite() { construct_api("HIGHS"); }
};
struct basis_page_clp_lp : model_test<clp_api, clp_lp> {
    static void SetUpTestSuite() { construct_api("CLP"); }
};

template <typename Fixture>
void check_own_start(Fixture & fixture) {
    fixture.SkipOnLicenseError([&]() {
        auto model = fixture.new_model();
        ASSERT_NEAR(basis_page::own_start(model), 71., TEST_EPSILON);
        ASSERT_TRUE(is_a<status::optimal>(model.get_status()));
    });
}

TEST_F(basis_page_highs_lp, workshop_prints_the_page_output) {
    check_workshop(*this);
}
TEST_F(basis_page_clp_lp, workshop_prints_the_page_output) {
    check_workshop(*this);
}
TEST_F(basis_page_highs_lp, own_start_reaches_the_optimum) {
    check_own_start(*this);
}
TEST_F(basis_page_clp_lp, own_start_reaches_the_optimum) {
    check_own_start(*this);
}
